/**
 ******************************************************************************
 * @file    robot.c
 * @brief   拨弹盘(DM4310)测试机器人:遥控器左右拨杆直接控制拨弹盘电机
 *          右拨杆[下]:停止               右拨杆[中/上]:按配置周期正转30°
 *          左拨杆[下]:停止               左拨杆[中/上]:按配置周期反转15°
 *          左拨杆的非[下]档位(反转指令)优先于右拨杆;左拨杆在[下]时交还给右拨杆控制
 ******************************************************************************
 */
/* Private includes --------------------------------------------------------- */
#include "robot.h"

#include <math.h>
#include "bsp_log.h"
#include "cmsis_os.h"
#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"

/* Private define ----------------------------------------------------------- */
#define LOADER_ONE_TURN_RAD (2.0f * PI)
#define LOADER_HOME_SLOT_COUNT 12U
#define LOADER_HOME_SLOT_RAD (LOADER_ONE_TURN_RAD / (float)LOADER_HOME_SLOT_COUNT)
/* 固定齿位对应的4310输出轴绝对反馈角度(rad),由台架实测得到 */
#define LOADER_HOME_POSITION_RAD (1.79617786)
/* 十二个齿位等价;回位沿正转方向最多走一个齿距,稳定后解锁拨弹 */
/* 回位仅适度增强位置环,速度环沿用正常增益,避免两环同时放大引起抖动 */
#define LOADER_HOME_ANGLE_KP_SCALE 2.0f
#define LOADER_HOME_SPEED_KP_SCALE 1.0f
#define LOADER_HOME_MAX_SPEED_RAD_S 6.0f
#define LOADER_HOME_ANGLE_TOLERANCE_RAD 0.03f
#define LOADER_HOME_SPEED_TOLERANCE_RAD_S 0.2f
#define LOADER_HOME_SETTLE_MS 100U
#define LOADER_HOME_TIMEOUT_MS 10000U
/* 反馈超过此时间未更新时停止输出,恢复后重新回位 */
#define LOADER_FEEDBACK_TIMEOUT_MS 100U
/* 推链力矩前馈(N·m):弹链阻力的基础份量由它承担,控制结构仍是位置环(角度环->速度环),
   前馈只是把速度环输出抬高一个基点,位置闭环照样把拨盘拉到目标角度.
   取3N·m:比推过弹链所需的约6N·m小,剩余部分由位置环残差(速度环P/I项)补足,
   既能顶开阻力,又不会在齿位之间空推时用力过猛 */
#define LOADER_PUSH_TORQUE_NM 3.0f
/* 目标角领先实测角超过该阈值才给推链前馈:停在齿位时撤掉,避免长期顶住弹链 */
#define LOADER_PUSH_GATE_RAD (1.0f * DEGREE_2_RAD)
/* 总力矩指令上限(含前馈):DMMotorSetPIDRef内部是"速度环输出 + 前馈",
   单靠speed_PID.MaxOut限不住总和,必须在robot.c里统一夹住.
   取7.5N·m,比推过弹链所需的6N·m留25%余量,同时低于DM4310峰值力矩,避免压变形弹丸 */
#define LOADER_TORQUE_LIMIT_NM 7.5f

/* Intermediate variables calculated by private functions ------------------ */
static RobotInstance *robot;

/* 上一次步进的时刻(ms),用于计算步进间隔 */
static uint32_t loader_step_time;
/* 模式切换或回位完成后,第一步到达前保持零速度前馈 */
static uint8_t loader_step_started;
/* 回位稳定后立即拨第一格,之后仍按配置的周期步进 */
static uint8_t loader_step_immediate;
/* 是否已经收到过电机反馈(即目标角度是否已有可信的起点) */
static uint8_t loader_feedback_ready;
/* 回位状态:首次转动及每次重新进入正向拨弹时先回到等价齿位 */
static uint8_t loader_home_done;
static uint8_t loader_home_target_valid;
static uint8_t loader_home_failed;
static uint8_t loader_home_settling;
static float loader_home_target_angle;
static float loader_normal_angle_kp;
static float loader_normal_angle_max_out;
static float loader_normal_speed_kp;
static uint32_t loader_home_start_time;
static uint32_t loader_home_settle_time;
/* 电机反馈时间戳由每帧反馈更新的DWT计数推导,用于识别运行中断线 */
static uint32_t loader_last_feed_count;
static uint32_t loader_last_feed_time;

/* Private function prototypes --------------------------------------------- */
static void LoaderApplyRef(float target_angle);
static void LoaderDisable(void);
static uint8_t LoaderFeedbackIsFresh(void);
static void LoaderHome(void);
static void LoaderStep(uint32_t period_ms, float step_rate);
static void LoaderSetMode(Loader_Mode_e mode);
static void RemoteControlSet(void);
static void LoaderControl(void);
static void RemoteControlDebug(void);

/* Private user code -------------------------------------------------------- */

/**
 * @brief 下发目标角度,并把最终力矩指令夹到LOADER_TORQUE_LIMIT_NM
 *
 * DMMotorSetPIDRef内部先算速度环(输出已被speed_PID.MaxOut限幅),
 * 再把力矩前馈直接加到结果上,所以两者的和可能超过任何单个限幅值;
 * 在这里统一夹一次,保证输出到DM4310的力矩指令不会超过机构承受上限
 *
 * @param target_angle 位置环的目标角度(rad)
 */
static void LoaderApplyRef(float target_angle)
{
    DMMotorSetPIDRef(robot->loader_motor, target_angle);
    LIMIT_MIN_MAX(robot->loader_motor->motor_controller.final_output, -LOADER_TORQUE_LIMIT_NM,
                  LOADER_TORQUE_LIMIT_NM);
}

/**
 * @brief 拨弹盘失能:停止力矩输出,并把目标角度同步为当前角度,
 *        这样重新使能时位置环误差为0,电机不会因目标突变而猛冲
 */
static void LoaderDisable(void)
{
    DMMotorStop(robot->loader_motor);
    /* 回位中途急停也退出临时增益,下次指令重新选择控制参数 */
    robot->loader_motor->motor_controller.angle_PID.Kp = loader_normal_angle_kp;
    robot->loader_motor->motor_controller.angle_PID.MaxOut = loader_normal_angle_max_out;
    robot->loader_motor->motor_controller.speed_PID.Kp = loader_normal_speed_kp;
    loader_speed_feedforward = 0.0f;
    loader_torque_feedforward = 0.0f;
    robot->target_angle = robot->loader_motor->measure.total_angle;
}

/**
 * @brief 确认已经收到4310反馈,并检查最近100ms内是否仍在持续收到新帧
 */
static uint8_t LoaderFeedbackIsFresh(void)
{
    uint32_t now = osKernelSysTick();
    uint32_t feed_count = robot->loader_motor->feed_cnt;

    if (robot->loader_motor->motor_can_instance->rx_len != 8U)
        return 0;

    if (feed_count != loader_last_feed_count)
    {
        loader_last_feed_count = feed_count;
        loader_last_feed_time = now;
    }

    return (uint32_t)(now - loader_last_feed_time) <= LOADER_FEEDBACK_TIMEOUT_MS;
}

/**
 * @brief 从十二个等价齿位中选择正转方向最近的一个,到位后解锁步进
 */
static void LoaderHome(void)
{
    uint32_t now = osKernelSysTick();
    float current_angle = robot->loader_motor->measure.total_angle;
    float home_error;

    if (loader_home_failed)
    {
        LoaderDisable();
        return;
    }

    if (!loader_home_target_valid)
    {
        /* 相邻等价齿位间隔30°,正向取模得到不超过一个齿距的回位量 */
        float forward_delta = fmodf(LOADER_HOME_POSITION_RAD - robot->loader_motor->measure.position,
                                    LOADER_HOME_SLOT_RAD);
        if (forward_delta < 0.0f)
            forward_delta += LOADER_HOME_SLOT_RAD;
        /* 已在某个等价齿位附近时不再绕过下一个齿距 */
        if (forward_delta > LOADER_HOME_SLOT_RAD - LOADER_HOME_ANGLE_TOLERANCE_RAD)
            forward_delta = 0.0f;

        loader_home_target_angle = current_angle + forward_delta;
        loader_home_start_time = now;
        loader_home_settling = 0;
        loader_home_target_valid = 1;
        PIDClear(&robot->loader_motor->motor_controller.angle_PID);
        PIDClear(&robot->loader_motor->motor_controller.speed_PID);
        LOGINFO("[loader] homing forward");
    }

    if ((uint32_t)(now - loader_home_start_time) > LOADER_HOME_TIMEOUT_MS)
    {
        loader_home_failed = 1;
        LoaderDisable();
        LOGERROR("[loader] homing timeout, switch both down to retry");
        return;
    }

    /* 普通正向回位和卡弹后对齐共用增强参数,避免普通回位仍以约1.2N·m等待积分 */
    home_error = fabsf(loader_home_target_angle - current_angle);
    robot->loader_motor->motor_controller.angle_PID.Kp = loader_normal_angle_kp * LOADER_HOME_ANGLE_KP_SCALE;
    robot->loader_motor->motor_controller.angle_PID.MaxOut = LOADER_HOME_MAX_SPEED_RAD_S;
    robot->loader_motor->motor_controller.speed_PID.Kp = loader_normal_speed_kp * LOADER_HOME_SPEED_KP_SCALE;
    loader_speed_feedforward = 0.0f;
    /* 回位要顶开弹链阻力:速度参考被限在6rad/s,速度环饱和推力只有4.2N·m,不够顶开弹链,
       故用推链前馈补足;到位后撤掉,避免停在齿位时长期顶住弹丸 */
    loader_torque_feedforward = (home_error > LOADER_HOME_ANGLE_TOLERANCE_RAD) ? LOADER_PUSH_TORQUE_NM : 0.0f;
    robot->target_angle = loader_home_target_angle;
    DMMotorEnable(robot->loader_motor);
    LoaderApplyRef(robot->target_angle);

    if (home_error <= LOADER_HOME_ANGLE_TOLERANCE_RAD &&
        fabsf(robot->loader_motor->measure.velocity) <= LOADER_HOME_SPEED_TOLERANCE_RAD_S)
    {
        if (loader_home_settling && (uint32_t)(now - loader_home_settle_time) >= LOADER_HOME_SETTLE_MS)
        {
            /* 上电回位需要低速稳定100ms才算到位 */
            loader_home_done = 1;
            robot->loader_motor->motor_controller.angle_PID.Kp = loader_normal_angle_kp;
            robot->loader_motor->motor_controller.angle_PID.MaxOut = loader_normal_angle_max_out;
            robot->loader_motor->motor_controller.speed_PID.Kp = loader_normal_speed_kp;
            loader_step_time = now;
            loader_step_started = 0;
            loader_step_immediate = 1;
            LOGINFO("[loader] homing done in %ums", (unsigned)(now - loader_home_start_time));
        }
        else if (!loader_home_settling)
        {
            loader_home_settling = 1;
            loader_home_settle_time = now;
        }
    }
    else
    {
        loader_home_settling = 0;
    }
}

/**
 * @brief 拨弹盘步进:每隔period_ms把目标角度增加(或减少)一个步距,并让电机跟踪该目标
 *
 * @param period_ms 步进间隔,单位ms
 * @param step_rate 该步距对应的平均角速度(rad/s),作为速度环前馈;符号即转向,正=正转,负=反转
 */
static void LoaderStep(uint32_t period_ms, float step_rate)
{
    DMMotorEnable(robot->loader_motor);
    loader_speed_feedforward = loader_step_started ? step_rate : 0.0f;

    if (loader_step_immediate || (uint32_t)(osKernelSysTick() - loader_step_time) >= period_ms)
    {
        loader_step_immediate = 0;
        loader_step_time = osKernelSysTick();
        /* 一个步距 = 平均角速度 × 步进间隔,符号与速度前馈一致,即反转时目标角度递减 */
        robot->target_angle += step_rate * ((float)period_ms * 0.001f);
        loader_step_started = 1;
        loader_speed_feedforward = step_rate;
    }

    /* 只在目标角仍领先实测角时给推链前馈:拨到齿位后撤掉,反转时目标角在后,同样为0,
       避免停在齿位时电机长期以推链力矩顶住弹丸 */
    loader_torque_feedforward = (robot->target_angle - robot->loader_motor->measure.total_angle >
                                 LOADER_PUSH_GATE_RAD)
                                    ? LOADER_PUSH_TORQUE_NM
                                    : 0.0f;

    /* 角度环->速度环的串级计算在DMMotorSetPIDRef内部完成,其输出为力矩参考 */
    LoaderApplyRef(robot->target_angle);
}

/**
 * @brief 拨弹盘模式切换:只在模式真正改变时做清理和日志
 *
 * @param mode 新的工作模式
 */
static void LoaderSetMode(Loader_Mode_e mode)
{
    Loader_Mode_e previous_mode = robot->loader_mode;

    if (robot->loader_mode == mode)
        return;

    /* 停止或反向时清掉步进状态,重新给正向指令会重新寻找最近齿位 */
    robot->loader_mode = mode;
    loader_step_immediate = 0;

    /* 从停止或反向切入正向时,重新寻找正转方向最近的等价齿位 */
    if ((mode == LOADER_STEP_SLOW || mode == LOADER_STEP_FAST) &&
        previous_mode != LOADER_STEP_SLOW && previous_mode != LOADER_STEP_FAST)
    {
        loader_home_done = 0;
        loader_home_target_valid = 0;
        loader_home_failed = 0;
        loader_home_settling = 0;
    }

    if (mode == LOADER_STEP_SLOW_REVERSE || mode == LOADER_STEP_FAST_REVERSE)
    {
        /* 手动反向直接从实测角度退半格,不能被尚未完成的正向回位挡住 */
        loader_home_target_valid = 0;
        loader_home_settling = 0;
        loader_step_immediate = 1;
        robot->target_angle = robot->loader_motor->measure.total_angle;
        loader_speed_feedforward = 0.0f;
        /* 反向拨弹不需要推链前馈(LoaderStep也会按"目标角在后"自动清零),这里显式清掉 */
        loader_torque_feedforward = 0.0f;
        robot->loader_motor->motor_controller.angle_PID.Kp = loader_normal_angle_kp;
        robot->loader_motor->motor_controller.angle_PID.MaxOut = loader_normal_angle_max_out;
        robot->loader_motor->motor_controller.speed_PID.Kp = loader_normal_speed_kp;
    }

    if (mode == LOADER_DISABLE)
    {
        /* 回位未完成时允许双下档暂停;下次给出转动指令重新选取正向目标 */
        if (!loader_home_done)
        {
            loader_home_target_valid = 0;
            loader_home_failed = 0;
        }
        LoaderDisable();
        LOGINFO("[loader] mode: disable");
        return;
    }

    /* 从当前时刻重新计时,并清除PID的历史状态,避免长时间失能后的积分冲击 */
    loader_step_time = osKernelSysTick();
    loader_step_started = 0;
    PIDClear(&robot->loader_motor->motor_controller.angle_PID);
    PIDClear(&robot->loader_motor->motor_controller.speed_PID);
    switch (mode)
    {
    case LOADER_STEP_SLOW:
        LOGINFO("[loader] mode: step 30deg/%ums", (unsigned)LOADER_STEP_PERIOD_MID_MS);
        break;
    case LOADER_STEP_FAST:
        LOGINFO("[loader] mode: step 30deg/%ums", (unsigned)LOADER_STEP_PERIOD_UP_MS);
        break;
    case LOADER_STEP_SLOW_REVERSE:
        LOGINFO("[loader] mode: reverse step 15deg/%ums", (unsigned)LOADER_STEP_PERIOD_MID_MS);
        break;
    case LOADER_STEP_FAST_REVERSE:
        LOGINFO("[loader] mode: reverse step 15deg/%ums", (unsigned)LOADER_STEP_PERIOD_UP_MS);
        break;
    default:
        break;
    }
}

/**
 * @brief 遥控器拨杆 -> 拨弹盘模式
 *        右拨杆[下/中/上] = 失能/慢速正转/快速正转
 *        左拨杆[下/中/上] = 失能/慢速反转/快速反转
 *        优先级:左拨杆的非[下]档位(反转指令)优先于右拨杆,拨下去即可立即反转;
 *                左拨杆在[下]时交还给右拨杆控制,所以正转随时可达;
 *                因此两个拨杆都在[下]时失能,单独使用任一拨杆时行为与标注完全一致
 */
static void RemoteControlSet(void)
{
    RC_ctrl_t *rc_data = robot->rc_data;

    /* 遥控器离线时失能,避免失控 */
    if (!RemoteControlIsOnline())
    {
        LoaderSetMode(LOADER_DISABLE);
        return;
    }

    /* 两个拨杆都在下档时优先急停,不进入回位或步进模式 */
    if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        LoaderSetMode(LOADER_DISABLE);
        DMMotorStop(robot->loader_motor);
        return;
    }

    /* 左拨杆:反转指令,优先于右拨杆 */
    if (switch_is_mid(rc_data[TEMP].rc.switch_left))
    {
        LoaderSetMode(LOADER_STEP_SLOW_REVERSE);
        return;
    }
    if (switch_is_up(rc_data[TEMP].rc.switch_left))
    {
        LoaderSetMode(LOADER_STEP_FAST_REVERSE);
        return;
    }

    /* 左拨杆在[下],交给右拨杆决定:下=失能,中=慢速正转,上=快速正转 */
    if (switch_is_mid(rc_data[TEMP].rc.switch_right))
        LoaderSetMode(LOADER_STEP_SLOW);
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))
        LoaderSetMode(LOADER_STEP_FAST);
    else
        LoaderSetMode(LOADER_DISABLE);
}

/**
 * @brief 根据当前工作模式控制拨弹盘电机
 *
 */
static void LoaderControl(void)
{
    /* 双下档也持续更新时间戳,避免停机期间断线后把旧反馈当作新反馈 */
    uint8_t feedback_fresh = LoaderFeedbackIsFresh();

    if (robot->loader_mode == LOADER_DISABLE)
    {
        LoaderDisable();
        return;
    }

    /* CAN反馈中断后不能继续追赶旧目标;恢复反馈后重新回位 */
    if (!feedback_fresh)
    {
        loader_feedback_ready = 0;
        loader_home_done = 0;
        loader_home_target_valid = 0;
        loader_home_failed = 0;
        LoaderDisable();
        return;
    }

    if (!loader_feedback_ready)
    {
        loader_feedback_ready = 1;
        robot->target_angle = robot->loader_motor->measure.total_angle;
        loader_step_time = osKernelSysTick();
    }

    if (!loader_home_done &&
        (robot->loader_mode == LOADER_STEP_SLOW || robot->loader_mode == LOADER_STEP_FAST))
    {
        LoaderHome();
        return;
    }

    switch (robot->loader_mode)
    {
    case LOADER_STEP_SLOW:
        LoaderStep(LOADER_STEP_PERIOD_MID_MS, LOADER_STEP_RATE_MID);
        break;
    case LOADER_STEP_FAST:
        LoaderStep(LOADER_STEP_PERIOD_UP_MS, LOADER_STEP_RATE_UP);
        break;
    case LOADER_STEP_SLOW_REVERSE:
        LoaderStep(LOADER_STEP_PERIOD_MID_MS, -LOADER_STEP_RATE_MID/2.0f);
        break;
    case LOADER_STEP_FAST_REVERSE:
        LoaderStep(LOADER_STEP_PERIOD_UP_MS, -LOADER_STEP_RATE_UP/2.0f);
        break;
    default:
        break;
    }
}

/**
 * @brief 遥控器诊断:每秒打印一次遥控器状态,便于定位"收不到遥控器数据"的问题
 *        online=0 且 rx=0            -> 串口上完全没有数据:检查接线/供电/是否插在带反相器的DBUS口
 *        online=1 但拨杆值不随遥控器变化 -> 能收到数据但协议不匹配(例如接的是VT13新遥控器)
 */
static void RemoteControlDebug(void)
{
    static uint32_t debug_time;
    uint16_t rx_cnt;

    if ((uint32_t)(osKernelSysTick() - debug_time) < 1000)
        return;
    debug_time = osKernelSysTick();

    /* 本次DMA接收已收到的字节数,DBUS一帧为18字节(完全没有数据时会一直为0) */
    rx_cnt = 0;
    if (huart3.hdmarx != NULL)
        rx_cnt = (uint16_t)(LOADER_RC_FRAME_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx));

    LOGINFO("[loader] rc online:%d rx:%d switch_right:%d", RemoteControlIsOnline(), rx_cnt,
            robot->rc_data[TEMP].rc.switch_right);
}

void RobotInit(void)
{
    robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));
    robot->loader_mode = LOADER_DISABLE;

    /* 遥控器使用DBUS协议串口(USART3, PC10/PC11),若实际接线不同请修改这里的串口 */
    robot->rc_data = RemoteControlInit(&huart3);

    robot->loader_motor = DMMotorInit(&loader_motor_config);
    /* 底层初始化会先使能电机,先置零力矩急停,再等待遥控器指令 */
    DMMotorStop(robot->loader_motor);
    /* DM电机模块未复制速度/力矩前馈指针,在本机器人中补齐以避免步进控制解引用空指针 */
    robot->loader_motor->motor_controller.speed_feedforward_ptr = &loader_speed_feedforward;
    robot->loader_motor->motor_controller.current_feedforward_ptr = &loader_torque_feedforward;
    loader_normal_angle_kp = robot->loader_motor->motor_controller.angle_PID.Kp;
    loader_normal_angle_max_out = robot->loader_motor->motor_controller.angle_PID.MaxOut;
    loader_normal_speed_kp = robot->loader_motor->motor_controller.speed_PID.Kp;
    loader_last_feed_time = osKernelSysTick();
    /* 以电机当前的实际角度作为目标角度的起点 */
    robot->target_angle = robot->loader_motor->measure.total_angle;
    loader_step_time = osKernelSysTick();

    LOGINFO("[loader] init done, dm4310 tx:0x01 rx:0x00");
}

/* 机器人核心控制任务,1kHz运行 */
void RobotTask(void)
{
    RemoteControlSet();
    LoaderControl();
    RemoteControlDebug();
}
