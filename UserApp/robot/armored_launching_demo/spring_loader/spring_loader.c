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
static float left_pull_ff;  // 左电机本轮的前馈电流指令，正值驱动皮带向下
static float right_pull_ff; // 右电机本轮的前馈电流指令，负值驱动皮带向下
static const float kPullFeedforward = (1.0f / 0.3f) * (16384.0f / 20.0f);  // 每侧约 1 N·m ，计算得对应的电流指令幅值
static Subscriber_t *launcher_cmd_sub;

/**
 * @brief 注册左右两台 M3508，配置串级位置环和电流前馈。
 * @details 两侧均使用 CAN1，左侧 ID 为 3，右侧 ID 为 2。
 *          注册后先停机并订阅动作请求，由周期任务在反馈就绪后开始找零。
 */
void SpringLoaderInit(void)
{

    Motor_Init_Config_s config_l = {
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
                .feedforward_flag = CURRENT_FEEDFORWARD,
            },
        .controller_param_init_config =
            {
                .current_feedforward_ptr = &left_pull_ff,
                .speed_PID =
                    {
                        .Kp = 0.70f,
                        .MaxOut = 16384.0f,
                    },
                .angle_PID =
                    {
                        .Kp = 25.0f,
                        .Ki = 15.0f,
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
                .tx_id = 2,
            },
        .controller_setting_init_config =
            {
                .outer_loop_type = SPEED_LOOP,           // 上电先用速度环回零
                .close_loop_type = ANGLE_AND_SPEED_LOOP, // 配置位置环和速度环
                .speed_feedback_source = MOTOR_FEED,
                .angle_feedback_source = MOTOR_FEED, // 位置环读取电机多圈角度
                .feedforward_flag = CURRENT_FEEDFORWARD,
            },
        .controller_param_init_config =
            {
                .current_feedforward_ptr = &right_pull_ff,
                .speed_PID =
                    {
                        .Kp = 0.74f,
                        .MaxOut = 16384.0f,
                    },
                .angle_PID =
                    {
                        .Kp = 25.0f,
                        .Ki = 15.0f,
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
    float zero_angle;     // 本侧找零时记录的转子多圈角度
    float last_moving_ms; // 最近一次未满足堵转条件的时间戳，单位为 ms
    bool done;            // 本侧是否已记零并停机
} HomeAxis;               // 单侧电机的回零状态

static HomeAxis home_l;          // 左侧电机回零状态
static HomeAxis home_r;          // 右侧电机回零状态
static bool home_pending = true; // 每次上电或复位后，等待执行一次回零
static bool homing;              // 整次回零是否正在进行
static float home_start_ms;      // 整次回零的开始时间，单位 ms

/**
 * @brief 驱动单侧电机向下找零，堵转条件持续 80 ms 后保存零点并停机。
 * @param motor 本侧电机实例。
 * @param[in,out] axis 本侧的零点、堵转计时和找零完成标记。
 * @param speed_ref 带方向的转子速度给定，单位为 °/s。
 * @param now_ms 本轮任务的时间戳，单位为 ms。
 */
static void HomeOneMotor(DJIMotorInstance *motor, HomeAxis *axis, float speed_ref, float now_ms)
{
    // 本侧已完成找零，跳过这一轮
    if (axis->done)
    {
        return;
    }

    if (fabsf(motor->measure.speed_aps) >= 60.0f ||
        fabsf(motor->measure.real_current) <= 300.0f) // 转速绝对值达到 60 或电流绝对值不超过 300，重新计时
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

static const float kStandbyDeg = -2000.0f;       // 上电待机、手动回位和自动回位共用的目标
static const float kLatchDeg = 9340.0f;          // 两侧实际角度都要超过的锁扣阈值
static volatile int position_request;            // 0 停机，1 上簧下拉，2 回位，3 上电慢速待机
static volatile float pull_target_deg = 10000.0f; // 下拉目标，单位为电机转子角度，已完成实测
static volatile float left_down_deg;             // 左侧相对顶部的角度，向下为正
static volatile float right_down_deg;            // 右侧相对顶部的角度，向下为正

/**
 * @brief 周期执行上簧下拉、自动回位和待机位置控制。
 * @details 下拉时各侧实际角度超过 -300° 后叠加本侧前馈。
 *          两侧同时超过锁扣阈值并持续 100 ms，或本次下拉用时达到 3 s，
 *          本轮即撤掉前馈并切换到回位目标。
 * @param feedback_ready 两侧电机反馈是否有效。
 * @param now_ms 本轮任务的时间戳，单位为 ms。
 */
static void RunLoadingCycle(bool feedback_ready, float now_ms)
{
    static float pull_start_ms;                  // 本次下拉开始的时间，用于判断 3 s 超时
    static float at_latch_since_ms = -1.0f;      // 两侧首次同时过阈值的时间，-1 表示尚未计时
    static int previous_request;

    int request = position_request;

    if (!feedback_ready || (request != 1 && request != 2 && request != 3))
    {
        position_request = 0;
        // 停机时撤掉前馈
        left_pull_ff = 0.0f;
        right_pull_ff = 0.0f;
        // 清除本次连续到位计时
        at_latch_since_ms = -1.0f;
        previous_request = 0;

        DJIMotorStop(motor_l);
        DJIMotorStop(motor_r);
        return;
    }

    left_down_deg = motor_l->measure.total_angle - home_l.zero_angle;
    right_down_deg = home_r.zero_angle - motor_r->measure.total_angle;

    // 仅在刚进入下拉的这一轮记录起点
    if (request == 1 && previous_request != 1)
    {
        pull_start_ms = now_ms; // 刚进入下拉，记录开始时间
    }

    // 两侧实际角度均已越过锁扣阈值
    const bool both_at_latch = left_down_deg > kLatchDeg && right_down_deg > kLatchDeg;

    // 只有下拉中且两侧同时越过阈值才计时
    if (request == 1 && both_at_latch)
    {
        // 本段连续到位尚未开始计时
        if (at_latch_since_ms < 0.0f)
        {
            at_latch_since_ms = now_ms; // 两侧首次同时超过阈值，开始计时
        }
    }
    else
    {
        at_latch_since_ms = -1.0f; // 退出下拉或任一侧掉回阈值，清除连续计时
    }

    // 两侧连续越过阈值满 100 ms
    const bool latch_held = at_latch_since_ms >= 0.0f && now_ms - at_latch_since_ms >= 100.0f;

    // 到位满 100 ms 或下拉达到 3 s
    if (request == 1 && (latch_held || now_ms - pull_start_ms >= 3000.0f))
    {
        request = 2; // 到位满 100 ms 或下拉超时，本轮撤掉前馈并回位
        position_request = 2;
        at_latch_since_ms = -1.0f;
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

    // 下拉越过 -300° 后加前馈
    left_pull_ff = (request == 1 && left_down_deg > -300.0f) ? kPullFeedforward : 0.0f;
    right_pull_ff = (request == 1 && right_down_deg > -300.0f) ? -kPullFeedforward : 0.0f;

    // 下拉用上簧目标，回位和上电待机用同一待机目标
    const float down_deg = (request == 1) ? pull_target_deg : kStandbyDeg;

    const float target_l = home_l.zero_angle + down_deg;
    const float target_r = home_r.zero_angle - down_deg;

    DJIMotorOuterLoop(motor_l, ANGLE_LOOP);
    DJIMotorOuterLoop(motor_r, ANGLE_LOOP);
    DJIMotorSetPIDRef(motor_l, target_l);
    DJIMotorSetPIDRef(motor_r, target_r);
    // 限制 PID 与前馈相加后的总电流指令
    LIMIT_MIN_MAX(motor_l->motor_controller.final_output, -16384.0f, 16384.0f);
    LIMIT_MIN_MAX(motor_r->motor_controller.final_output, -16384.0f, 16384.0f);
    DJIMotorEnable(motor_l);
    DJIMotorEnable(motor_r);
}

/**
 * @brief 处理上电找零、左摇杆上簧和手动回位。
 * @details 两侧反馈就绪后分别向下找零，记零后自动向上退到待机位。
 *          左摇杆上推启动一次上簧，回中后本次动作继续。
 *          下拨优先回位，上簧过程中的到位计时和超时由 RunLoadingCycle() 处理。
 */
void SpringLoaderTask(void)
{
    const float now_ms = DWT_GetTimeline_ms(); // 读取本轮时间，单位 ms
    const bool feedback_ready = motor_l->feed_cnt != 0 && motor_r->feed_cnt != 0 && DaemonIsOnline(motor_l->daemon) &&
                                DaemonIsOnline(motor_r->daemon);
    // 准备就绪判定：两侧都收到过反馈 且 两侧反馈都未超时

    LauncherCommand command = {0};             // 先清零，队列为空时两个请求都是 false
    SubGetMessage(launcher_cmd_sub, &command); // 有新消息就取走，回零期间的消息也在本轮消费

    // 两侧都取得零点后才处理上簧和回位
    if (home_l.done && home_r.done)
    {
        const bool standby_ready = position_request != 3 ||                   // 仅在上电慢速待机阶段检查待机位置
                   (fabsf(left_down_deg - kStandbyDeg) <= 50.0f &&
                   fabsf(right_down_deg - kStandbyDeg) <= 50.0f);             // 两侧距待机目标不超过 50°

        // 有效反馈下，优先处理下拨回位
        if (feedback_ready && command.return_requested)
        {
            position_request = 2; // 中止当前下拉，切换到待机位置目标
        }
        // 满足启动条件时接收一次上推
        else if (feedback_ready && command.load && position_request != 1 && standby_ready)
        {
            position_request = 1; // 上推一次，开始试拉
        }

        // 按本轮状态更新位置目标和电流输出
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

    HomeOneMotor(motor_l, &home_l, 800.0f, now_ms);  // 左侧向下找零
    HomeOneMotor(motor_r, &home_r, -800.0f, now_ms); // 右侧向下找零

    if (home_l.done && home_r.done)
    {
        homing = false;       // 两侧都已在下端各自记零
        position_request = 3; // 下一轮用位置环向上退到待机位
    }
}
