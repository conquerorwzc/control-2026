//
// grab_6dof: 7 关节达妙电机机械臂的执行组件
//   robot 层 : 读遥控 -> 写 grab_ctrl_cmd -> 调 GrabTask        (不碰电机)
//   组件层   : 读 grab_ctrl_cmd -> 使能握手/锚定/下发            (所有碰电机的代码)
//   无回零动作: 上电/急停恢复均以锚定时刻的当前位置为参考直接保持
//

/* Private includes ----------------------------------------------------------*/
#include "grab.h"

#include "bsp_log.h"
#include "cmsis_os.h"
#include "user_lib.h"

/* Private variables ---------------------------------------------------------*/
static GrabInstance *grab;              // 组件单例, GrabInit 赋值
static Grab_Ctrl_Cmd_s *grab_ctrl_cmd;  // 指向实例内嵌命令层; robot.c 的同名指针指向同一块内存

/* Private function prototypes -----------------------------------------------*/
static void EnableHandshake(uint8_t idx); // 上电使能握手, 有界阻塞

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
    LOGERROR("[robot] motor[%d] enable handshake FAILED, go ESTOP", idx);
    grab_ctrl_cmd->grab_mode = GRAB_ESTOP;
}

void GrabAnchorEntry()
{
    for (uint8_t i = 0; i < grab->grab_param.motor_num; i++)
    {
        Grab_Joint_s *j = &grab->joint[i];

        PIDClear(&j->motor->motor_controller.speed_PID);  // 清上次运行积分, 防 re-arm 跳变
        PIDClear(&j->motor->motor_controller.angle_PID);
        DMMotorSetRef(j->motor, 0.0f);       //清除控制器中参与 final_output,防止回到RUN，使能后，新输出计算前受旧输出影响

        DMMotorEnable(j->motor);       //由失能态进到本函数，需要重新使能
        //将当前位置作为REF，避免跳变
        grab_ctrl_cmd->cmd_angle[i] = MotorToJoint(i, j->motor->measure.total_angle);
        LOGINFO("[robot] anchor joint[%d] ref=%f", i, grab_ctrl_cmd->cmd_angle[i]);

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
        j->limit = config->joint[i].limit;
        if (j->reduction == 0.0f) LOGERROR("[grab] joint[%d] reduction == 0! MotorToJoint would div-by-0", i);
        if (j->limit <= 0.0f) LOGERROR("[grab] joint[%d] limit <= 0, joint will not move", i);
    }

    grab = grab_instance;
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;

    grab_ctrl_cmd->grab_mode = GRAB_RUN;
    return grab_instance;
}

/**
 * @brief 机械臂任务: 首调完成使能握手与锚定, 之后刷新关节反馈并按 grab_mode 两态执行
 */
void GrabTask()
{
    static uint8_t first_run = 1;
    if (first_run)
    {
        for (uint8_t i = 0; i < grab->grab_param.motor_num; i++) EnableHandshake(i);
        if (grab_ctrl_cmd->grab_mode != GRAB_ESTOP) GrabAnchorEntry();
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
    case GRAB_RUN:
        for (uint8_t i = 0; i < grab->grab_param.motor_num; i++)
        {
            Grab_Joint_s *j = &grab->joint[i];
            DMMotorSetPIDRef(j->motor, JointToMotor(i, grab_ctrl_cmd->cmd_angle[i]));
        }
        break;
    default:  // GRAB_ESTOP
        for (uint8_t i = 0; i < grab->grab_param.motor_num; i++)
        {
            Grab_Joint_s *j = &grab->joint[i];
            DMMotorStop(j->motor);                  //置ESTOP位，不再输出力矩
        }
        break;
    }
}
