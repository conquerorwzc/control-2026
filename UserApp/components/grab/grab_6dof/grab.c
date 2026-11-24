//
// grab_6dof: 7 关节达妙电机机械臂的执行组件
//   robot 层 : 读遥控/生成测试信号 -> 写 grab_ctrl_cmd -> 调 GrabTask   (不碰电机)
//   组件层   : 读 grab_ctrl_cmd -> 使能握手/回零斜坡/下发              (所有碰电机的代码)
//

/* Private includes ----------------------------------------------------------*/
#include "grab.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "math.h"
#include "user_lib.h"

/* Private variables ---------------------------------------------------------*/
static GrabInstance *grab;              // 组件单例, GrabInit 赋值
static Grab_Ctrl_Cmd_s *grab_ctrl_cmd;  // 指向实例内嵌命令层; robot.c 的同名指针指向同一块内存

/* Private function prototypes -----------------------------------------------*/
static void EnableHandshake(uint8_t idx); // 上电使能握手, 有界阻塞
static void HomingStep();                 // 每 1kHz 周期推进回零斜坡

/**
 * @brief 关节域 -> 电机域 (指令方向): ref = q × reduction
 */
static float JointToMotor(uint8_t i, float q)
{
    return q * grab->joint[i].reduction;
}

/**
 * @brief 电机域 -> 关节域 (反馈方向): q = θ ÷ reduction
 * @note  reduction 填 0 会在锚定时除零, GrabInit 里有守卫
 */
static float MotorToJoint(uint8_t i, float theta)
{
    return theta / grab->joint[i].reduction;
}

/**
 * @brief 上电使能(首次 GrabTask 调用时每台执行一次, 有界阻塞).
 *        DMMotorInit 里那次 0xFC 可能因电机启动期(1~2s)而丢失, 这里重试直到反馈确认.
 * @note  必须放 GrabTask(RobotTask 内首调)而不是 RobotInit: 中断在 os_task.c:56 __enable_irq 之前全关,
 *        RobotInit 期间读不到反馈帧.
 *        三分支: state==1 确认使能 / ==0 失能但活着, 重发 / 8~E 故障锁存, 先 0xFB 清错
 */
static void EnableHandshake(uint8_t idx)
{
    DMMotorInstance *motor = grab->joint[idx].motor;
    for (uint8_t attempt = 0; attempt < 20; attempt++)  // 总时长上界
    {
        uint32_t feed_snapshot = motor->feed_cnt;
        DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);  // 使能帧 0xFC
        osDelay(10);
        if (motor->feed_cnt == feed_snapshot)  // 无反馈: 电机还没启动完/不在线
        {
            osDelay(100);
            continue;
        }
        uint8_t state = motor->measure.state;
        if (state == 1)  // 已使能, 确认
        {
            LOGINFO("[robot] motor[%d] enabled", idx);
            return;
        }
        if (state >= 8)  // 故障码(欠压/通讯丢失等锁存): 先清错, 下一轮重发使能, 若仍无法成功，则可能存在故障
        {
            LOGERROR("[robot] motor[%d] fault state=%d, clear error", idx, state);
            DMMotorSetMode(DM_CMD_CLEAR_ERROR, motor);
            osDelay(10);
        }
        // state==0: 失能但活着, 循环重发 0xFC
    }
    LOGERROR("[robot] motor[%d] enable handshake FAILED, stay in ESTOP", idx);
}

void GrabHomingEntry()
{
    for (uint8_t i = 0; i < grab->grab_param.motor_num; i++)
    {
        Grab_Joint_s *j = &grab->joint[i];
        DMMotorEnable(j->motor);       //由失能态进到本函数，需要重新使能
        j->home_ref = MotorToJoint(i, j->motor->measure.total_angle);
        PIDClear(&j->motor->motor_controller.speed_PID);  // 清上次运行积分, 防 re-arm 跳变
        PIDClear(&j->motor->motor_controller.angle_PID);
    }
    LOGINFO("[robot] homing start, joint_ref=[%f %f %f %f %f %f]",
            grab->joint[0].home_ref, grab->joint[1].home_ref, grab->joint[2].home_ref,
            grab->joint[3].home_ref, grab->joint[4].home_ref, grab->joint[5].home_ref);
}

/**
 * @brief 每 1kHz 周期推进回零斜坡
 */
static void HomingStep()
{
    float dt = DWT_GetDeltaT(&grab->homing_dt_cnt);
    if (dt > 0.02f) dt = 0.02f;  // 首调(时间戳=0)/调度卡死: 不允许 dt 放大步长

    uint8_t all_arrived = 1;
    for (uint8_t i = 0; i < grab->grab_param.motor_num; i++)
    {
        Grab_Joint_s *j = &grab->joint[i];
        // 位置回零只对角度外环的关节有意义: 速度外环的 pid_ref 单位是 rad/s,
        // 把位置斜坡喂给它 = 让电机以 home_ref 的数值转圈. 速度外环保持零速(阻尼站立), 立即视为到点
        if (j->motor->motor_settings.outer_loop_type != ANGLE_LOOP)
        {
            grab_ctrl_cmd->cmd_speed[i] = 0.0f;  // 速度外环关节的 RUN 下发走 cmd_speed, 这里保持零速
            DMMotorSetPIDRef(j->motor, JointToMotor(i, grab_ctrl_cmd->cmd_speed[i]));
            continue;
        }
        float step = -j->home_ref;  // 目标恒为 0
        LIMIT_MIN_MAX(step, -grab->grab_param.homing_speed * dt, grab->grab_param.homing_speed * dt);
        j->home_ref += step;
        grab_ctrl_cmd->cmd_angle[i] = j->home_ref;
        DMMotorSetPIDRef(j->motor, JointToMotor(i, grab_ctrl_cmd->cmd_angle[i]));  // 斜坡期间闭环一直在跑, 电机受控运动
        if (fabsf(j->home_ref) >= grab->grab_param.homing_tol) all_arrived = 0;
    }
    if (all_arrived)
    {
        grab_ctrl_cmd->grab_mode = GRAB_RUN;  // HOMING->RUN 由组件写回命令层
        LOGINFO("[robot] homing done -> RUN");
    }
}

/**
 * @brief 机械臂组件初始化: 注册 DM 电机 + 初始化命令层
 */
GrabInstance *GrabInit(Grab_Init_Config_s *config)
{
    GrabInstance *grab_instance = (GrabInstance *)zmalloc(sizeof(GrabInstance));
    grab_instance->grab_param = config->grab_param;

    for (uint8_t i = 0; i < config->grab_param.motor_num; i++)
    {
        Grab_Joint_s *j = &grab_instance->joint[i];
        j->motor = DMMotorInit(&config->joint[i].motor_config);
        j->reduction = config->joint[i].reduction;
        if (j->reduction == 0.0f) LOGERROR("[grab] joint[%d] reduction == 0! MotorToJoint would div-by-0", i);
    }

    grab = grab_instance;
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;

    // 上电默认进 HOMING
    grab_ctrl_cmd->grab_mode = GRAB_HOMING;
    return grab_instance;
}

/**
 * @brief 机械臂任务: 首调完成使能握手, 之后刷新关节反馈并按 grab_mode 三态执行
 */
void GrabTask()
{
    static uint8_t first_run = 1;
    if (first_run)
    {
        for (uint8_t i = 0; i < grab->grab_param.motor_num; i++) EnableHandshake(i);
        if (grab_ctrl_cmd->grab_mode == GRAB_HOMING) GrabHomingEntry();
        first_run = 0;
    }

    // 关节反馈刷新: 全组件统一在此换算一次
    for (uint8_t i = 0; i < grab->grab_param.motor_num; i++)
    {
        Grab_Joint_s *j = &grab->joint[i];
        j->q = MotorToJoint(i, j->motor->measure.total_angle);
    }

    switch (grab_ctrl_cmd->grab_mode)
    {
    case GRAB_HOMING:
        HomingStep();
        break;
    case GRAB_RUN:
        // 按每台的闭环类型选命令数组: 角度外环读 cmd_angle(rad), 速度外环读 cmd_speed(rad/s), 下发换算电机域
        for (uint8_t i = 0; i < grab->grab_param.motor_num; i++)
        {
            Grab_Joint_s *j = &grab->joint[i];
            DMMotorSetPIDRef(j->motor,
                             JointToMotor(i,
                                          j->motor->motor_settings.outer_loop_type == ANGLE_LOOP
                                              ? grab_ctrl_cmd->cmd_angle[i]
                                              : grab_ctrl_cmd->cmd_speed[i]));
        }
        break;
    default:  // GRAB_ESTOP
        for (uint8_t i = 0; i < grab->grab_param.motor_num; i++)
        {
            Grab_Joint_s *j = &grab->joint[i];
            DMMotorStop(j->motor);
            DMMotorSetRef(j->motor, 0.0f);
            grab_ctrl_cmd->cmd_angle[i] = 0.0f;
            grab_ctrl_cmd->cmd_speed[i] = 0.0f;
        }
        break;
    }
}
