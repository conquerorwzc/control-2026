#include "robot.h"

#include <math.h>

#include "bsp_dwt.h"
#include "bsp_gpio.h"
#include "bsp_log.h"
#include "remote_control.h"

#include "grab.h"
#include "robot_config.h"

/* =====================================================================
 *   EmergencyHandler : 读遥控, 写 grab_ctrl_cmd->grab_mode (ESTOP / 恢复仪式)
 *   CmdMap/Swing : 在 RUN 态写 grab_ctrl_cmd->cmd_angle
 *   GrabTask (组件)  : 按 grab_mode 执行握手/回零/下发
 * ===================================================================== */

static RC_ctrl_t *rc_data;

// 急停恢复条件: 本轮连接后见过 DOWN + 左开关不在 DOWN(UP/MID 任一工作档) + 四轴回中
static uint8_t saw_down_after_connect = 0;

// 机械臂组件实例与命令入口.
GrabInstance *grab;
Grab_Ctrl_Cmd_s *grab_ctrl_cmd;

// 测试信号幅值 (watch 同上, 保留全局符号)
float swing_amp[GRAB_MOTOR_NUM] = {SWING_AMP_0, SWING_AMP_1, SWING_AMP_2,
                                   SWING_AMP_3, SWING_AMP_4, SWING_AMP_5, SWING_AMP_6};

static GPIOInstance *gpio_24V_R_EN;
static GPIO_Init_Config_s gpio_init_config_24v = {
    .GPIO_Pin = POWER_24V_R_Pin,
    .GPIOx = POWER_5V_GPIO_Port,
    .pin_state = GPIO_PIN_SET,
  };

static GPIOInstance *gpio_5V_EN;
static GPIO_Init_Config_s gpio_init_config_5v = {
    .GPIO_Pin = POWER_5V_Pin,
    .GPIOx = POWER_24V_R_GPIO_Port,
    .pin_state = GPIO_PIN_SET,
  };

#if SWING_TEST
static uint32_t swing_t0_ms = 0;
static uint8_t swing_first_call = 1;

/**
 * @brief 摆动时间基重锚, 在请求进入 HOMING 时由本文件调用:
 *        保证 RUN 首周期 sin(0)=0 == homing 终点 ref=0, 交接零跳变.
 */
static void SwingReset(void)
{
    swing_first_call = 1;
}

/**
 * @brief 正弦摆动测试: cmd_angle = A·sin(2πft). 仅 RUN 态被调用, 急停/回零期间自动挂起,
 *        时间轴不推进; 恢复必经 SwingReset, 首相位恒为 0, 不存在"一上来就是幅值"的上电大抖.
 *        幅值/频率见 robot_config.h SWING_*.
 */
static void SwingUpdate(void)
{
    if (swing_first_call)
    {
        swing_t0_ms = DWT_GetTimeline_ms();
        swing_first_call = 0;
    }
    float t_s = (DWT_GetTimeline_ms() - swing_t0_ms) / 1000.0f;
    float swing = sinf(SWING_OMEGA * t_s);  // sin(0)=0: 速度参考从 0 平滑爬升, 无阶跃

#if defined(TUNE_MOTOR_IDX)
    grab_ctrl_cmd->cmd_angle[TUNE_MOTOR_IDX] = swing_amp[TUNE_MOTOR_IDX] * swing;  // 调参态: 只驱动目标台, 其余保持 homing 终点
#else
    for (uint8_t i = 0; i < GRAB_MOTOR_NUM; i++)  // 写满命令层; 组件按 motor_num 下发
        grab_ctrl_cmd->cmd_angle[i] = swing_amp[i] * swing;
#endif
}
#endif

/**
 * @brief 摇杆→单轴增量: raw 偏离回中 = 目标以 rate rad/s 移动
 *        target 限幅 ±limit(=CMD_ANGLE_MAX_x, 行程边界).
 */
static float AxisStep(float target, int16_t raw, float rate, float limit, float dt)
{
    if (raw > -ROCKER_DEADZONE && raw < ROCKER_DEADZONE) return target;
    target += (float)raw / 660.0f * rate * dt;
    LIMIT_MIN_MAX(target, -limit, limit);
    return target;
}

/**
 * @brief 映射表: 左sw上档驱动0~3号, 中档驱动4~6号, DOWN 不映射(急停职责).
 *        上档: 左摇杆上下=0, 左摇杆左右=1, 右摇杆上下=2, 右摇杆左右=3.
 *        中档: 镜像上档轴位, 左摇杆上下=4, 左摇杆左右=5, 右摇杆上下=6; rocker_r_ 中档不映射.
 *        速率/限幅见 robot_config.h CMD_RATE_x / CMD_ANGLE_MAX_x.
 */
static void CmdMap(void)
{
    // 增量映射的积分步长
    static uint32_t cmd_dt_cnt = 0;
    float dt = DWT_GetDeltaT(&cmd_dt_cnt);
    if (dt > 0.05f) dt = 0.05f;  // 首调(时间戳=0)/长时间未进RUN: 不允许 dt 放大步长

    uint8_t sw = rc_data->rc.switch_left;
    if (switch_is_up(sw))
    {
        grab_ctrl_cmd->cmd_angle[0] = AxisStep(grab_ctrl_cmd->cmd_angle[0], rc_data->rc.rocker_l1, CMD_RATE_0, CMD_ANGLE_MAX_0, dt);
        grab_ctrl_cmd->cmd_angle[1] = AxisStep(grab_ctrl_cmd->cmd_angle[1], rc_data->rc.rocker_l_, CMD_RATE_1, CMD_ANGLE_MAX_1, dt);
        grab_ctrl_cmd->cmd_angle[2] = AxisStep(grab_ctrl_cmd->cmd_angle[2], rc_data->rc.rocker_r1, CMD_RATE_2, CMD_ANGLE_MAX_2, dt);
        grab_ctrl_cmd->cmd_angle[3] = AxisStep(grab_ctrl_cmd->cmd_angle[3], rc_data->rc.rocker_r_, CMD_RATE_3, CMD_ANGLE_MAX_3, dt);
    }
    else if (switch_is_mid(sw))
    {
        grab_ctrl_cmd->cmd_angle[4] = AxisStep(grab_ctrl_cmd->cmd_angle[4], rc_data->rc.rocker_l1, CMD_RATE_4, CMD_ANGLE_MAX_4, dt);
        grab_ctrl_cmd->cmd_angle[5] = AxisStep(grab_ctrl_cmd->cmd_angle[5], rc_data->rc.rocker_l_, CMD_RATE_5, CMD_ANGLE_MAX_5, dt);
        grab_ctrl_cmd->cmd_angle[6] = AxisStep(grab_ctrl_cmd->cmd_angle[6], rc_data->rc.rocker_r1, CMD_RATE_6, CMD_ANGLE_MAX_6, dt);
    }
}

/**
 * @brief 急停状态机(输入侧): 只读遥控、写 grab_mode, 不碰电机.
 *        进入 ESTOP: 遥控离线 / 左开关在 DOWN(UP/MID 均为工作档).
 *        退出(只到 HOMING 不直接到 RUN): 遥控在线 + 本轮连接后拨到过 DOWN + 左开关不在 DOWN + 四轴回中.
 */
static void EmergencyHandler(void)
{
    if (!RemoteControlIsOnline())
    {
        if (grab_ctrl_cmd->grab_mode != GRAB_ESTOP) LOGINFO("[robot] rc offline -> ESTOP");
        grab_ctrl_cmd->grab_mode = GRAB_ESTOP;
        saw_down_after_connect = 0;  // 重连后必须重新"打下-拨上"
        return;
    }

    uint8_t sw = rc_data->rc.switch_left;
    if (switch_is_down(sw)) saw_down_after_connect = 1;  // 在线时见过 DOWN 即记录
    if (switch_is_down(sw))
    {
        if (grab_ctrl_cmd->grab_mode != GRAB_ESTOP) LOGINFO("[robot] sw DOWN -> ESTOP");
        grab_ctrl_cmd->grab_mode = GRAB_ESTOP;
        return;
    }

    if (grab_ctrl_cmd->grab_mode != GRAB_ESTOP) return;

    // ESTOP + 在线 + 非 DOWN(UP/MID 任一): 检查恢复授权
    uint8_t centered =
        rc_data->rc.rocker_l1 > -ROCKER_DEADZONE && rc_data->rc.rocker_l1 < ROCKER_DEADZONE &&
        rc_data->rc.rocker_l_ > -ROCKER_DEADZONE && rc_data->rc.rocker_l_ < ROCKER_DEADZONE &&
        rc_data->rc.rocker_r1 > -ROCKER_DEADZONE && rc_data->rc.rocker_r1 < ROCKER_DEADZONE &&
        rc_data->rc.rocker_r_ > -ROCKER_DEADZONE && rc_data->rc.rocker_r_ < ROCKER_DEADZONE;
    if (saw_down_after_connect && centered)
    {
#if SWING_TEST
        SwingReset();  // 测试信号时间基重锚(原 HomingEntry 内职责, 输入侧自管)
#endif
        GrabHomingEntry();  // 锚定+清积分(组件公有入口), 紧挨下面的状态赋值
        grab_ctrl_cmd->grab_mode = GRAB_HOMING;
    }
}

void RobotInit()
{
    gpio_24V_R_EN = GPIORegister(&gpio_init_config_24v);
    GPIOSet(gpio_24V_R_EN);

    gpio_5V_EN = GPIORegister(&gpio_init_config_5v);
    GPIOSet(gpio_5V_EN);
    rc_data = RemoteControlInit(&huart5);

    grab = GrabInit(&grab_init_config);           // 电机注册在组件内, 发送线程由 os_task.c DMMotorTaskInit 创建
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;         // 命令层指针, 上电默认 grab_mode = GRAB_HOMING
}

void RobotTask()
{
    EmergencyHandler();

    // 测试信号/摇杆映射只在 RUN 写命令层: HOMING 期间命令层归 homing 斜坡所有, 不能抢写
    if (grab_ctrl_cmd->grab_mode == GRAB_RUN)
    {
#if SWING_TEST
        SwingUpdate();
#else
        CmdMap();
#endif
    }

    GrabTask();  // 首次调用在此完成使能握手, 之后按 grab_mode 执行
}
