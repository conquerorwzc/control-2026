#include "latch_release.h"

#include <float.h>
#include <stdbool.h>

#include "../robot_config.h"
#include "bsp_dwt.h"
#include "can.h"
#include "dji_motor.h"
#include "message_center.h"

static DJIMotorInstance *latch_motor; // 保存 M2006 电机实例
static Subscriber_t *launcher_cmd_sub;
static bool release_zero_ready;      // 标记是否已经记录上电零点
static float release_zero_rotor_deg; // 上电零点，单位为转子角度

static volatile int release_request;               // 0 停机，1 击发一次
static volatile float release_target_deg = -45.0f; // 带方向的输出轴目标角度
static volatile float release_angle_deg;           // 输出轴相对零点的实际角度

/**
 * @brief 初始化击发机构的 M2006 串级位置环。
 * @details 电机使用 CAN1 上的 ID 4。注册后先停机，并订阅 launcher_cmd 话题。
 */
void LatchReleaseInit(void)
{
    Motor_Init_Config_s config = {
        .motor_type = M2006,
        .can_init_config =
            {
                .can_handle = &hcan1,
                .tx_id = 4,
            }, // 创建击发电机的初始化配置
        .controller_setting_init_config =
            {
                .outer_loop_type = ANGLE_LOOP,           // 外环为位置环，接收目标角度
                .close_loop_type = ANGLE_AND_SPEED_LOOP, // 位置环给速度目标，速度环给电流指令
                .angle_feedback_source = MOTOR_FEED,
                .speed_feedback_source = MOTOR_FEED, // 电机自身返回数据
            },                                       // 配置串级闭环
        .controller_param_init_config =
            {
                .angle_PID = {.Kp = 15.0f,
                              .Ki = 1.0f,
                              .MaxOut = FLT_MAX, // 转子目标速度上限，单位 °/s
                              .Improve = PID_Integral_Limit,
                              .IntegralLimit = 1000.00f},
                .speed_PID = {.Kp = 25.0f,
                              .Ki = 1.0f,
                              .MaxOut = 10000.0f, // 电调电流指令限幅
                              .Improve = PID_Integral_Limit,
                              .IntegralLimit = 1000.0f},
            },
    };

    latch_motor = DJIMotorInit(&config);
    DJIMotorStop(latch_motor);

    // 初始化击发模块的收件队列
    launcher_cmd_sub = SubRegister(LAUNCHER_CMD_TOPIC, sizeof(LauncherCommand));
}

/**
 * @brief 记录击发拨臂的上电零点，并处理右摇杆触发的单次击发。
 * @details 首次收到有效反馈时，以当前角度为零点。收到右摇杆请求后，
 *          按输出轴目标 -45° 执行位置控制。满 800 ms 后清除请求并停止输出，
 *          由弹簧带回拨臂。停机期间仍更新实际角度，供 Ozone 观察。
 */
void LatchReleaseTask(void)
{
    static bool releasing;         // 本次击发是否已经开始计时
    static float release_start_ms; // 本次击发的开始时间，单位 ms

    LauncherCommand command = {0};             // 本轮默认没有新动作
    SubGetMessage(launcher_cmd_sub, &command); // 取走击发模块收到的本轮消息

    // 尚未收到反馈或反馈超时时，清除击发请求
    if (latch_motor->feed_cnt == 0 || !DaemonIsOnline(latch_motor->daemon))
    {
        release_request = 0;
        releasing = false;
        DJIMotorStop(latch_motor);
        return;
    }

    const float rotor_deg = latch_motor->measure.total_angle; // 读取转子的多圈角度

    if (!release_zero_ready)
    {
        release_zero_rotor_deg = rotor_deg;
        release_zero_ready = true;
    }

    // 停机后仍更新角度，Ozone 可以看到弹簧把拨臂弹回哪里。
    release_angle_deg = (rotor_deg - release_zero_rotor_deg) / 36.0f;

    // 右摇杆本轮触发
    if (command.release && !releasing)
    {
        release_request = 1; // 右摇杆上推一次
    }

    if (release_request != 1)
    {
        releasing = false;
        DJIMotorStop(latch_motor);
        return;
    }

    const float now_ms = DWT_GetTimeline_ms();
    if (!releasing)
    {
        release_start_ms = now_ms; // 收到本次指令时只记一次起点
        releasing = true;
        PIDClear(&latch_motor->motor_controller.angle_PID);
        PIDClear(&latch_motor->motor_controller.speed_PID);
    }

    if (now_ms - release_start_ms >= 800.0f)
    {
        release_request = 0; // 清掉指令，等待下次手动写 1
        releasing = false;
        DJIMotorStop(latch_motor); // 给零电流，由结构弹簧带回拨臂
        return;
    }

    const float target_rotor_deg = release_zero_rotor_deg + release_target_deg * 36.0f;

    DJIMotorSetPIDRef(latch_motor, target_rotor_deg);
    DJIMotorEnable(latch_motor);
}