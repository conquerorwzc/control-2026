#include "spring_loader.h"
#include <float.h>
#include <math.h>
#include <stdbool.h>

#include "../robot_config.h"
#include "bsp_dwt.h"
#include "can.h"
#include "dji_motor.h"
#include "message_center.h"

static DJIMotorInstance *motor_l;
static DJIMotorInstance *motor_r;
static Subscriber_t *launcher_cmd_sub;

/**
 * @brief 初始化上簧机构的左右两台 M3508。
 * @details 左右电机分别使用 CAN1 上的 ID 2 和 ID 3。注册后先停机，
 *          并订阅 launcher_cmd 话题，由周期任务在反馈就绪后开始回零。
 */
void SpringLoaderInit(void)
{

    Motor_Init_Config_s config_l = {
        .motor_type = M3508,
        .can_init_config =
            {
                .can_handle = &hcan1,
                .tx_id = 2,
            },
        .controller_setting_init_config =
            {
                .outer_loop_type = SPEED_LOOP,           // 上电先用速度环回零
                .close_loop_type = ANGLE_AND_SPEED_LOOP, // 配置位置环和速度环
                .speed_feedback_source = MOTOR_FEED,
                .angle_feedback_source = MOTOR_FEED, // 位置环读取电机多圈角度
            },
        .controller_param_init_config =
            {
                .speed_PID =
                    {
                        .Kp = 0.70f,
                        .MaxOut = 16384.0f,
                    },
                .angle_PID =
                    {
                        .Kp = 20.0f,
                        .Ki = 3.0f,
                        .Kd = 0.0f,
                        .MaxOut = FLT_MAX,             // 位置环输出的速度上限（单位 °/s）直接给最大
                        .Improve = PID_Integral_Limit, // 启用积分限幅
                        .IntegralLimit = 3000.0f,      // 积分项的限幅
                    },
            },
    };

    motor_l = DJIMotorInit(&config_l);
    DJIMotorStop(motor_l);
    // 注册左侧3508电机

    Motor_Init_Config_s config_r = {
        .motor_type = M3508,
        .can_init_config =
            {
                .can_handle = &hcan1,
                .tx_id = 3,
            },
        .controller_setting_init_config =
            {
                .outer_loop_type = SPEED_LOOP,           // 上电先用速度环回零
                .close_loop_type = ANGLE_AND_SPEED_LOOP, // 配置位置环和速度环
                .speed_feedback_source = MOTOR_FEED,
                .angle_feedback_source = MOTOR_FEED, // 位置环读取电机多圈角度
            },
        .controller_param_init_config =
            {
                .speed_PID =
                    {
                        .Kp = 0.88f,
                        .MaxOut = 16384.0f,
                    },
                .angle_PID =
                    {
                        .Kp = 20.0f,
                        .Ki = 3.0f,
                        .Kd = 0.0f,
                        .MaxOut = FLT_MAX,             // 位置环输出的速度上限（单位 °/s）直接给最大
                        .Improve = PID_Integral_Limit, // 启用积分限幅
                        .IntegralLimit = 3000.0f,      // 积分项的限幅
                    },
            },
    };

    motor_r = DJIMotorInit(&config_r);
    DJIMotorStop(motor_r);
    // 注册右侧3508电机

    launcher_cmd_sub = SubRegister(LAUNCHER_CMD_TOPIC, sizeof(LauncherCommand)); // 初始化上簧模块的收件队列
}

typedef struct
{
    float zero_angle;     // 到顶时零度角
    float last_moving_ms; // 上次堵转持续时间
    bool done;            // 是否已停机
} HomeAxis;               // 单侧电机的回零状态

static HomeAxis home_l;          // 左侧电机回零状态
static HomeAxis home_r;          // 右侧电机回零状态
static bool home_pending = true; // 每次上电或复位后，等待执行一次回零
static bool homing;              // 整次回零是否正在进行
static float home_start_ms;      // 整次回零的开始时间，单位 ms

/**
 * @brief 驱动单侧电机向上找零，满足堵转判定后记录多圈角度并停机。
 * @param motor 本侧电机实例。
 * @param[in,out] axis 本侧的零点、堵转计时和回零完成状态。
 * @param speed_ref 带方向的转子目标速度，单位为 °/s。
 * @param now_ms 本轮任务的时间戳，单位为 ms。
 */
static void HomeOneMotor(DJIMotorInstance *motor, HomeAxis *axis, float speed_ref, float now_ms)
{
    if (axis->done) // 这一侧已经到顶并停机
    {
        return;
    }

    if (fabsf(motor->measure.speed_aps) >= 60.0f ||
        fabsf(motor->measure.real_current) <= 300.0f) // 未堵转状态：转速大于60，电流小于300
    {
        axis->last_moving_ms = now_ms;
    }
    else if (now_ms - axis->last_moving_ms >= 80.0f) // 堵转条件持续80ms
    {
        axis->zero_angle = motor->measure.total_angle; // 保存这一刻的多圈角度作为零点
        DJIMotorStop(motor);                           // 电机停机
        axis->done = true;                             // 标记这一侧回零完成
        return;                                        // 结束本轮，跳过下方的使能
    }

    DJIMotorSetPIDRef(motor, speed_ref);
    DJIMotorEnable(motor);
}

static const float kStandbyDeg = 1000.0f; // 相对顶部零点，向下为正
static volatile int position_request;    // 0 停机，1 下拉，2 正常回程，3 上电慢速待机
static volatile float pull_target_deg = 15000.0f; // 下拉目标，单位为电机转子角度
static volatile float left_down_deg;              // 左侧相对顶部的角度，向下为正
static volatile float right_down_deg;             // 右侧相对顶部的角度，向下为正

/**
 * @brief 执行双电机下拉，两侧达到上簧阈值并持续指定时间后切换到回程目标。
 * @param feedback_ready 两侧电机的反馈是否有效。
 * @param now_ms 本轮任务的时间戳，单位为 ms。
 */
static void RunLoadingCycle(bool feedback_ready, float now_ms)
{
    static float at_latch_since_ms = -1.0f;
    static int previous_request;

    int request = position_request;

    if (!feedback_ready || (request != 1 && request != 2 && request != 3))
    {
        position_request = 0;
        at_latch_since_ms = -1.0f;
        previous_request = 0;

        DJIMotorStop(motor_l);
        DJIMotorStop(motor_r);
        return;
    }

    left_down_deg = motor_l->measure.total_angle - home_l.zero_angle;
    right_down_deg = home_r.zero_angle - motor_r->measure.total_angle;

    // 两侧实际角度都达到上簧阈值
    const bool both_at_latch = left_down_deg >= 13400.0f && right_down_deg >= 13400.0f;

    if (request == 1 && both_at_latch)
    {
        if (at_latch_since_ms < 0.0f)
        {
            at_latch_since_ms = now_ms; // 两侧首次同时到位，从现在开始计时
        }
        if (now_ms - at_latch_since_ms >= 100.0f)
        {
            request = 2;
            position_request = 2;

            at_latch_since_ms = -1.0f;
        }
    }
    else
    {
        at_latch_since_ms = -1.0f; // 任一侧掉到阈值以下，重新计时
    }

    if (request != previous_request)
    {
        // 位置环的输出是速度环的目标，只有上电待机阶段限到 800°/s。
        const float speed_limit = (request == 3) ? 800.0f : FLT_MAX;
        motor_l->motor_controller.angle_PID.MaxOut = speed_limit;
        motor_r->motor_controller.angle_PID.MaxOut = speed_limit;

        PIDClear(&motor_l->motor_controller.angle_PID);
        PIDClear(&motor_r->motor_controller.angle_PID);
        previous_request = request;
    }

    const float down_deg = (request == 1) ? pull_target_deg : kStandbyDeg;
    const float target_l = home_l.zero_angle + down_deg;
    const float target_r = home_r.zero_angle - down_deg;

    DJIMotorOuterLoop(motor_l, ANGLE_LOOP);
    DJIMotorOuterLoop(motor_r, ANGLE_LOOP);
    DJIMotorSetPIDRef(motor_l, target_l);
    DJIMotorSetPIDRef(motor_r, target_r);
    DJIMotorEnable(motor_l);
    DJIMotorEnable(motor_r);
}

/**
 * @brief 执行上电回零，并处理左摇杆触发的上簧动作。
 * @details 两路反馈就绪后分别向上找零。两侧记零后，左摇杆上推一次启动下拉，
 *          摇杆回中后本次动作继续。两侧实际角度均达到 13400° 并持续 100 ms
 *          后切换到回程，位置环随后保持在 1000° 目标。这里的角度均为转子角度。
 */
void SpringLoaderTask(void)
{
    const float now_ms = DWT_GetTimeline_ms(); // 读取本轮时间，单位 ms
    const bool feedback_ready = motor_l->feed_cnt != 0 && motor_r->feed_cnt != 0 && DaemonIsOnline(motor_l->daemon) &&
                                DaemonIsOnline(motor_r->daemon);
    // 准备就绪判定：两侧都收到过反馈 且 两侧反馈都未超时

    LauncherCommand command = {0};             // 先清零，队列为空时两个请求都是 false
    SubGetMessage(launcher_cmd_sub, &command); // 有新消息就取走，回零期间的消息也在本轮消费

    // 两侧都完成上电回零后才处理上簧
    if (home_l.done && home_r.done)
    {
        // 上电慢速移动时，等两侧都到 1000° 附近才接收上簧指令。
        const bool standby_ready =
            position_request != 3 ||
            (fabsf(left_down_deg - kStandbyDeg) <= 50.0f &&
             fabsf(right_down_deg - kStandbyDeg) <= 50.0f);

        if (feedback_ready && command.load && position_request != 1 && standby_ready)
        {
            position_request = 1; // 启动一次下拉
        }

        RunLoadingCycle(feedback_ready, now_ms);
        return;
    }

    if (!homing) // 当前没有在回零
    {
        if (!home_pending || !feedback_ready)
        {
            return;
        } // 本次上电机会已用掉，或两路反馈还没到齐，结束本轮

        home_pending = false; // 两路反馈都到齐，现在才消费本次上电请求
        home_l = (HomeAxis){.last_moving_ms = now_ms};
        home_r = (HomeAxis){.last_moving_ms = now_ms}; // 重置左右侧状态
        home_start_ms = now_ms;                        // 从这一刻开始计算整次回零的耗时
        homing = true;                                 // 进入回零，本轮继续向下执行
    }

    if (!feedback_ready || now_ms - home_start_ms >= 10000.0f)
    {
        DJIMotorStop(motor_l);
        DJIMotorStop(motor_r);
        homing = false;
        return;
    } // 任意一侧反馈失效，或回零达到 10 秒： 停止输出，跳过下方控制

    HomeOneMotor(motor_l, &home_l, -800.0f, now_ms);
    HomeOneMotor(motor_r, &home_r, 800.0f, now_ms);

    if (home_l.done && home_r.done)
    {
        homing = false;       // 两侧都已到顶，各自保存了零点
        position_request = 3; // 下一轮切到位置环，慢速去 1000° 待机位
    }
}