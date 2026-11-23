//
// grab_6dof: 7 台达妙电机机械臂的执行组件 (三态状态机 ESTOP/HOMING/RUN)
//
// 拆分自 robot/armored_grab/robot.c, 结构仿照 components/grab/5-DOF:
//   robot 层 : 读遥控/生成测试信号 -> 写 grab_ctrl_cmd -> 调 GrabTask   (不碰电机)
//   组件层   : 读 grab_ctrl_cmd -> 使能握手/回零斜坡/下发              (所有碰电机的代码)
// 命令层 cmd_angle[]/cmd_speed[] 是绝对目标; grab_mode 的写入归属:
//   robot 写 ESTOP/HOMING, HOMING->RUN 由组件在回零到点时写回 cmd.
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
 * @brief 上电使能(首次 GrabTask 调用时每台执行一次, 有界阻塞).
 *        DMMotorInit 里那次 0xFC 可能因电机启动期(1~2s)而丢失, 这里重试直到反馈确认.
 * @note  必须放 GrabTask(RobotTask 内首调)而不是 RobotInit: 中断在 os_task.c:56 __enable_irq 之前全关,
 *        RobotInit 期间读不到反馈帧.
 *        三分支: state==1 确认使能 / ==0 失能但活着, 重发 / 8~E 故障锁存, 先 0xFB 清错
 */
static void EnableHandshake(uint8_t idx)
{
    for (uint8_t attempt = 0; attempt < 20; attempt++)  // 总时长上界
    {
        uint32_t feed_snapshot = grab->motor[idx]->feed_cnt;
        DMMotorSetMode(DM_CMD_MOTOR_MODE, grab->motor[idx]);  // 使能帧 0xFC
        osDelay(10);
        if (grab->motor[idx]->feed_cnt == feed_snapshot)  // 无反馈: 电机还没启动完/不在线
        {
            osDelay(100);
            continue;
        }
        uint8_t state = grab->motor[idx]->measure.state;
        if (state == 1)  // 已使能, 确认
        {
            LOGINFO("[robot] motor[%d] enabled", idx);
            return;
        }
        if (state >= 8)  // 故障码(欠压/通讯丢失等锁存): 先清错, 下一轮重发使能, 若仍无法成功，则可能存在故障
        {
            LOGERROR("[robot] motor[%d] fault state=%d, clear error", idx, state);
            DMMotorSetMode(DM_CMD_CLEAR_ERROR, grab->motor[idx]);
            osDelay(10);
        }
        // state==0: 失能但活着, 循环重发 0xFC
    }
    LOGERROR("[robot] motor[%d] enable handshake FAILED, stay in ESTOP", idx);
}

void GrabHomingEntry()
{
    for (uint8_t i = 0; i < grab->motor_num; i++)
    {
        DMMotorEnable(grab->motor[i]);       //由失能态进到本函数，需要重新使能
        grab->home_ref[i] = grab->motor[i]->measure.total_angle;  // 锚定: 出发点由电机说了算
        PIDClear(&grab->motor[i]->motor_controller.speed_PID);  // 清上次运行积分, 防 re-arm 跳变
        PIDClear(&grab->motor[i]->motor_controller.angle_PID);
    }
    LOGINFO("[robot] homing start, ref=[%f %f %f %f %f %f]",
            grab->home_ref[0], grab->home_ref[1], grab->home_ref[2], grab->home_ref[3], grab->home_ref[4],
            grab->home_ref[5]);
}

/**
 * @brief 每 1kHz 周期推进回零斜坡
 */
static void HomingStep()
{
    float dt = DWT_GetDeltaT(&grab->homing_dt_cnt);
    if (dt > 0.02f) dt = 0.02f;  // 首调(时间戳=0)/调度卡死: 不允许 dt 放大步长

    uint8_t all_arrived = 1;
    for (uint8_t i = 0; i < grab->motor_num; i++)
    {
        // 位置回零只对角度外环的电机有意义: 速度外环的 pid_ref 单位是 rad/s,
        // 把位置斜坡喂给它 = 让电机以 home_ref 的数值转圈. 速度外环保持零速(阻尼站立), 立即视为到点
        if (grab->motor[i]->motor_settings.outer_loop_type != ANGLE_LOOP)
        {
            grab_ctrl_cmd->cmd_speed[i] = 0.0f;  // 速度外环电机的 RUN 下发走 cmd_speed, 这里保持零速
            DMMotorSetPIDRef(grab->motor[i], grab_ctrl_cmd->cmd_speed[i]);
            continue;
        }
        float step = -grab->home_ref[i];  // 目标恒为 0
        LIMIT_MIN_MAX(step, -grab->grab_param.homing_speed * dt, grab->grab_param.homing_speed * dt);
        grab->home_ref[i] += step;
        grab_ctrl_cmd->cmd_angle[i] = grab->home_ref[i];
        DMMotorSetPIDRef(grab->motor[i], grab_ctrl_cmd->cmd_angle[i]);  // 斜坡期间闭环一直在跑, 电机受控运动
        if (fabsf(grab->home_ref[i]) >= grab->grab_param.homing_tol) all_arrived = 0;
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
    grab_instance->motor_num = config->motor_num;
    grab_instance->grab_param = config->grab_param;

    for (uint8_t i = 0; i < grab_instance->motor_num; i++)
    {
        grab_instance->motor[i] = DMMotorInit(&config->motor_config[i]);
    }

    grab = grab_instance;
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;

    // 上电默认进 HOMING
    grab_ctrl_cmd->grab_mode = GRAB_HOMING;
    return grab_instance;
}

/**
 * @brief 机械臂任务: 首调完成使能握手, 之后按 grab_mode 三态执行
 */
void GrabTask()
{
    static uint8_t first_run = 1;
    if (first_run)
    {
        for (uint8_t i = 0; i < grab->motor_num; i++) EnableHandshake(i);
        if (grab_ctrl_cmd->grab_mode == GRAB_HOMING) GrabHomingEntry();
        first_run = 0;
    }

    switch (grab_ctrl_cmd->grab_mode)
    {
    case GRAB_HOMING:
        HomingStep();
        break;
    case GRAB_RUN:
        // 按每台的闭环类型选命令数组: 角度外环读 cmd_angle(rad), 速度外环读 cmd_speed(rad/s)
        for (uint8_t i = 0; i < grab->motor_num; i++)
            DMMotorSetPIDRef(grab->motor[i],
                             grab->motor[i]->motor_settings.outer_loop_type == ANGLE_LOOP
                                 ? grab_ctrl_cmd->cmd_angle[i]
                                 : grab_ctrl_cmd->cmd_speed[i]);
        break;
    default:  // GRAB_ESTOP
        for (uint8_t i = 0; i < grab->motor_num; i++)
        {
            DMMotorStop(grab->motor[i]);
            DMMotorSetRef(grab->motor[i], 0.0f);
            grab_ctrl_cmd->cmd_angle[i] = 0.0f;
            grab_ctrl_cmd->cmd_speed[i] = 0.0f;
        }
        break;
    }
}
