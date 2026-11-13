#include "robot_cmd.h"

#include "message_center.h"
#include "remote_control.h"
#include "robot_config.h"
#include "usart.h"

static RC_ctrl_t *rc;
static Publisher_t *launcher_cmd_pub; // 保存动作指令话题的发布者

/**
 * @brief 检测单根摇杆的一次上推，摇杆回中后才允许再次触发。
 * @param stick 本轮读到的摇杆通道值。
 * @param[in,out] centered 该摇杆是否已经回中并可以触发。
 * @return 本轮触发时为 true，其余情况为 false。
 */
static bool StickUpOnce(int16_t stick, bool *centered)
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
 * @brief 将左右摇杆的单次上推转换为上簧和击发请求。
 * @details 两根摇杆独立检测回中和上推。左摇杆产生上簧请求，右摇杆产生击发请求。
 *          遥控器离线时清除回中标记，重新连接后先回中才能触发。
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

    // 生成本轮的动作消息
    LauncherCommand command = {
        .load = StickUpOnce(rc->rc.rocker_l1, &left_centered),
        .release = StickUpOnce(rc->rc.rocker_r1, &right_centered),
    };

    // 至少有一个摇杆在本轮触发
    if (command.load || command.release)
    {
        PubPushMessage(launcher_cmd_pub, &command);
    }
}