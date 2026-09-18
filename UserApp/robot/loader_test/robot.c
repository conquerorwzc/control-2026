/**
 ******************************************************************************
 * @file    robot.c
 * @brief   拨弹盘(DM4310)测试机器人:遥控器左右拨杆直接控制拨弹盘电机
 *          右拨杆[下]:失能               右拨杆[中]:每隔300ms正转30°   右拨杆[上]:每隔50ms正转30°
 *          左拨杆[下]:失能               左拨杆[中]:每隔300ms反转30°   左拨杆[上]:每隔50ms反转30°
 *          左拨杆的非[下]档位(反转指令)优先于右拨杆;左拨杆在[下]时交还给右拨杆控制
 ******************************************************************************
 */
#include "robot.h"

#include "bsp_log.h"
#include "cmsis_os.h"
#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"

static RobotInstance *robot;

/* 上一次步进的时刻(ms),用于计算步进间隔 */
static uint32_t loader_step_time;
/* 是否已经收到过电机反馈(即目标角度是否已有可信的起点) */
static uint8_t loader_feedback_ready;

/**
 * @brief 拨弹盘失能:停止力矩输出,并把目标角度同步为当前角度,
 *        这样重新使能时位置环误差为0,电机不会因目标突变而猛冲
 */
static void LoaderDisable(void)
{
    DMMotorStop(robot->loader_motor);
    loader_speed_feedforward = 0.0f;
    robot->target_angle = robot->loader_motor->measure.total_angle;
}

/**
 * @brief 拨弹盘步进:每隔period_ms把目标角度增加(或减少)一个步距,并让电机跟踪该目标
 *
 * @param period_ms 步进间隔,单位ms
 * @param step_rate 该步距对应的平均角速度(rad/s),作为速度环前馈;符号即转向,正=正转,负=反转
 */
static void LoaderStep(uint32_t period_ms, float step_rate)
{
    /* 尚未收到电机反馈时不允许使能:此时目标角度没有可信的起点,直接使能可能导致电机猛冲 */
    if (robot->loader_motor->motor_can_instance->rx_len == 0)
    {
        LoaderDisable();
        return;
    }

    if (!loader_feedback_ready)
    {
        /* 首次收到反馈,以电机的当前角度作为目标角度的起点,并从当前时刻开始计时 */
        loader_feedback_ready = 1;
        robot->target_angle = robot->loader_motor->measure.total_angle;
        loader_step_time = osKernelSysTick();
    }

    DMMotorEnable(robot->loader_motor);
    loader_speed_feedforward = step_rate;

    if ((uint32_t)(osKernelSysTick() - loader_step_time) >= period_ms)
    {
        loader_step_time = osKernelSysTick();
        /* 一个步距 = 平均角速度 × 步进间隔,符号与速度前馈一致,即反转时目标角度递减 */
        robot->target_angle += step_rate * ((float)period_ms * 0.001f);
    }

    /* 角度环->速度环的串级计算在DMMotorSetPIDRef内部完成,其输出为力矩参考 */
    DMMotorSetPIDRef(robot->loader_motor, robot->target_angle);
}

/**
 * @brief 拨弹盘模式切换:只在模式真正改变时做清理和日志
 *
 * @param mode 新的工作模式
 */
static void LoaderSetMode(Loader_Mode_e mode)
{
    if (robot->loader_mode == mode)
        return;
    robot->loader_mode = mode;

    if (mode == LOADER_DISABLE)
    {
        LoaderDisable();
        LOGINFO("[loader] mode: disable");
        return;
    }

    /* 从当前时刻重新计时,并清除PID的历史状态,避免长时间失能后的积分冲击 */
    loader_step_time = osKernelSysTick();
    PIDClear(&robot->loader_motor->motor_controller.angle_PID);
    PIDClear(&robot->loader_motor->motor_controller.speed_PID);
    switch (mode)
    {
    case LOADER_STEP_SLOW:
        LOGINFO("[loader] mode: step 30deg/300ms");
        break;
    case LOADER_STEP_FAST:
        LOGINFO("[loader] mode: step 30deg/50ms");
        break;
    case LOADER_STEP_SLOW_REVERSE:
        LOGINFO("[loader] mode: reverse step 30deg/300ms");
        break;
    case LOADER_STEP_FAST_REVERSE:
        LOGINFO("[loader] mode: reverse step 30deg/50ms");
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
    switch (robot->loader_mode)
    {
    case LOADER_DISABLE:
        LoaderDisable();
        break;
    case LOADER_STEP_SLOW:
        LoaderStep(LOADER_STEP_PERIOD_MID_MS, LOADER_STEP_RATE_MID);
        break;
    case LOADER_STEP_FAST:
        LoaderStep(LOADER_STEP_PERIOD_UP_MS, LOADER_STEP_RATE_UP);
        break;
    case LOADER_STEP_SLOW_REVERSE:
        LoaderStep(LOADER_STEP_PERIOD_MID_MS, -LOADER_STEP_RATE_MID);
        break;
    case LOADER_STEP_FAST_REVERSE:
        LoaderStep(LOADER_STEP_PERIOD_UP_MS, -LOADER_STEP_RATE_UP);
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
