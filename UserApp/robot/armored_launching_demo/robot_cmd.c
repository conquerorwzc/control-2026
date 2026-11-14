#include <stdlib.h>

#include "message_center.h"
#include "remote_control.h"
#include "robot_cmd.h"
#include "robot_config.h"
#include "usart.h"

static RC_ctrl_t *rc;
static Publisher_t *launcher_cmd_pub; // 保存动作指令话题的发布者

/**
 * @brief 检测一次过阈值事件，摇杆回中后才允许再次触发。
 * @param stick 用于判断的通道值。左摇杆传入绝对值，右摇杆传入原值。
 * @param[in,out] centered 是否已回中并允许再次触发。
 * @return 本轮首次越过触发阈值时返回 true。
 */
static bool StickOnce(int16_t stick, bool *centered)
{
    if (stick >= -100 && stick <= 100) // 摇杆进入中位区间
    {
        *centered = true; // 回到中位，允许下一次上推
    }

    if (*centered && stick >= 400) // 回过中位后，向上推过触发阈值
    {
        *centered = false; // 本次上推已触发，继续按住也只发一次
        return true;
    }

    return false;
}

/**
 * @brief 在 USART3 上初始化遥控器接收。
 * @details 保存驱动返回的遥控器数据指针，并注册 launcher_cmd 话题的发布者。
 */
void RobotCmdInit(void)
{
    rc = RemoteControlInit(&huart3);
    launcher_cmd_pub = PubRegister(LAUNCHER_CMD_TOPIC, sizeof(LauncherCommand));
}

/**
 * @brief 发布左摇杆上推上簧、下拨回位及右摇杆上推击发的请求。
 * @details 左右摇杆分别记录回中状态，每次拨动只触发一次。
 *          遥控器离线时清除回中标记，恢复后需先回中才能触发。
 */
void RobotCmdTask(void)
{
    static bool left_centered;
    static bool right_centered;

    // 遥控器离线或尚无有效数据
    if (!RemoteControlIsOnline() || rc->rc.switch_right == 0)
    {
        // 离线或尚未收到有效帧。恢复后，两根摇杆各自回中才能触发。
        left_centered = false;
        right_centered = false;
        return;
    }

    const int16_t left_stick = rc->rc.rocker_l1;
    const bool left_triggered = StickOnce(abs(left_stick), &left_centered);

    // 组装本轮检测到的动作请求
    LauncherCommand command =
    {
        .load = left_triggered && left_stick > 0,                // 左摇杆上推，启动上簧
        .return_requested = left_triggered && left_stick < 0,    // 左摇杆下拨，手动回位
        .release = StickOnce(rc->rc.rocker_r1, &right_centered), // 右摇杆上推，触发击发
    };

    // 本轮有动作才发布
    if (command.load || command.return_requested || command.release)
    {
        PubPushMessage(launcher_cmd_pub, &command);              // 将本轮请求发给执行模块
    }
}