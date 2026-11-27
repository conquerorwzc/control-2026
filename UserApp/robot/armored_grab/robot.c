#include "robot.h"

#include "bsp_dwt.h"
#include "bsp_gpio.h"
#include "bsp_log.h"
#include "remote_control.h"

#include "grab.h"
#include "robot_config.h"

/* =====================================================================
 *   EmergencyHandler : 读遥控, 写 grab_ctrl_cmd->grab_mode (ESTOP / 恢复仪式)
 *   CmdMap           : 在 RUN 态写 grab_ctrl_cmd->cmd_angle
 *   GrabTask         : 按 grab_mode 执行握手/锚定/下发
 * ===================================================================== */

static RC_ctrl_t *rc_data;

// 急停恢复条件: 本轮连接后见过 DOWN + 左开关不在 DOWN(UP/MID 任一工作档) + 四轴回中
static uint8_t saw_down_after_connect = 0;

// 机械臂组件实例与命令入口.
GrabInstance *grab;
Grab_Ctrl_Cmd_s *grab_ctrl_cmd;

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

/**
 * @brief 摇杆→单轴增量: raw 偏离回中 = 目标以 rate rad/s 移动
 *        target 限幅 ±joint[i].limit(行程边界).
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
 *        速率见 robot_config.h CMD_RATE_x, 限幅在关节档案 grab->joint[i].limit.
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
        grab_ctrl_cmd->cmd_angle[0] = AxisStep(grab_ctrl_cmd->cmd_angle[0], rc_data->rc.rocker_l1, CMD_RATE_0, grab->joint[0].limit, dt);
        grab_ctrl_cmd->cmd_angle[1] = AxisStep(grab_ctrl_cmd->cmd_angle[1], rc_data->rc.rocker_l_, CMD_RATE_1, grab->joint[1].limit, dt);
        grab_ctrl_cmd->cmd_angle[2] = AxisStep(grab_ctrl_cmd->cmd_angle[2], rc_data->rc.rocker_r1, CMD_RATE_2, grab->joint[2].limit, dt);
        grab_ctrl_cmd->cmd_angle[3] = AxisStep(grab_ctrl_cmd->cmd_angle[3], rc_data->rc.rocker_r_, CMD_RATE_3, grab->joint[3].limit, dt);
    }
    else if (switch_is_mid(sw))
    {
        grab_ctrl_cmd->cmd_angle[4] = AxisStep(grab_ctrl_cmd->cmd_angle[4], rc_data->rc.rocker_l1, CMD_RATE_4, grab->joint[4].limit, dt);
        grab_ctrl_cmd->cmd_angle[5] = AxisStep(grab_ctrl_cmd->cmd_angle[5], rc_data->rc.rocker_l_, CMD_RATE_5, grab->joint[5].limit, dt);
        grab_ctrl_cmd->cmd_angle[6] = AxisStep(grab_ctrl_cmd->cmd_angle[6], rc_data->rc.rocker_r1, CMD_RATE_6, grab->joint[6].limit, dt);
    }
}

/**
 * @brief 急停状态机(输入侧): 写 grab_mode.
 *        进入 ESTOP: 遥控离线 / 左开关在 DOWN(UP/MID 均为工作档).
 *        退出: 遥控在线 + 本轮连接后拨到过 DOWN + 左开关不在 DOWN + 四轴回中.
 *        恢复以当前位置重新锚定.
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
        GrabAnchorEntry();  // 锚定当前位置+清积分(组件公有入口), 紧挨下面的状态赋值
        grab_ctrl_cmd->grab_mode = GRAB_RUN;
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
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;         // 命令层指针, 上电默认 grab_mode = GRAB_RUN(待首次锚定)
}

void RobotTask()
{
    EmergencyHandler();

    // 摇杆映射只在 RUN 写命令层: 急停期间命令层归零, 恢复时由锚定入口重写
    if (grab_ctrl_cmd->grab_mode == GRAB_RUN)
    {
        CmdMap();
    }

    GrabTask();  // 首次调用在此完成使能握手+锚定当前位置, 之后按 grab_mode 执行
}
