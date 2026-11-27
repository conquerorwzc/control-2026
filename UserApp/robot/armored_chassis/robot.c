/**
 * @file robot.c
 * @brief 重装底盘台架应用，负责电机注册、遥控状态、麦轮运动和四路齿条抬升。
 */

#include <math.h>
#include <stdbool.h>

#include "bsp_dwt.h"
#include "remote_control.h"
#include "robot.h"
#include "robot_config.h"

static DJIMotorInstance *wheel_lf;
static DJIMotorInstance *wheel_rf;
static DJIMotorInstance *wheel_lr;
static DJIMotorInstance *wheel_rr;
static DJIMotorInstance *guide_left;
static DJIMotorInstance *guide_right;
static DJIMotorInstance *side_left_3508;
static DJIMotorInstance *side_right_3508;
static DJIMotorInstance *side_left_2006;
static DJIMotorInstance *side_right_2006;

static DMMotorInstance *lift_lf;
static DMMotorInstance *lift_rf;
static DMMotorInstance *lift_lr;
static DMMotorInstance *lift_rr;

// 抬升数组顺序为左前、右前、左后、右后。模数为 2，前侧 30 齿，后侧 50 齿。
static const float kLiftPitchRadiusMm[4] = {30.0f, 30.0f, 50.0f, 50.0f};
static const float kLiftSpeedMmS = 50.0f;  // 共用位移轨迹的速度，单位 mm/s
static const float kLiftSyncGain = 2.0f;   // 由位移误差换算纠偏速度，单位 1/s
static const float kLiftMaxSyncMmS = 1.0f; // 单轴纠偏速度的幅值上限，单位 mm/s

static volatile float lift_target_mm;      // Ozone 写入的目标位移，相对本次使能起点
static volatile float lift_ref_mm;         // 四路共用的当前位移参考
static volatile float lift_position_mm[4]; // 各齿条相对起点的实测位移，由编码器角度换算
static float lift_start_angle_rad[4];      // 同一次使能时记录的四个角度起点
static float lift_speed_ff_rad_s[4];       // 叠加到各轴速度环参考值的角速度前馈

static bool lift_reference_ready; // 四个角度起点已采集
static bool lift_home_started;

static RC_ctrl_t *remote_data;
static bool motors_enabled; // 记录 DJI 电机的输出状态，便于在挡位切换时更新
static float rotate_command;

/**
 * @brief 注册四个麦轮、六个引导和侧边机构电机、四个抬升电机，并初始化遥控接收。
 * @details 各电机注册后将软件输出置为停止。遥控接收使用 UART5 的 DBUS 接口。
 */
void RobotInit(void)
{
    wheel_lf = DJIMotorInit(&wheel_lf_config);
    DJIMotorStop(wheel_lf);
    wheel_rf = DJIMotorInit(&wheel_rf_config);
    DJIMotorStop(wheel_rf);
    wheel_lr = DJIMotorInit(&wheel_lr_config);
    DJIMotorStop(wheel_lr);
    wheel_rr = DJIMotorInit(&wheel_rr_config);
    DJIMotorStop(wheel_rr);

    guide_left = DJIMotorInit(&guide_left_config);
    DJIMotorStop(guide_left);
    guide_right = DJIMotorInit(&guide_right_config);
    DJIMotorStop(guide_right);

    side_left_3508 = DJIMotorInit(&side_left_3508_config);
    DJIMotorStop(side_left_3508);
    side_right_3508 = DJIMotorInit(&side_right_3508_config);
    DJIMotorStop(side_right_3508);
    side_left_2006 = DJIMotorInit(&side_left_2006_config);
    DJIMotorStop(side_left_2006);
    side_right_2006 = DJIMotorInit(&side_right_2006_config);
    DJIMotorStop(side_right_2006);

    lift_lf = DMMotorInit(&lift_lf_config);
    DMMotorStop(lift_lf);
    lift_rf = DMMotorInit(&lift_rf_config);
    DMMotorStop(lift_rf);
    lift_lr = DMMotorInit(&lift_lr_config);
    DMMotorStop(lift_lr);
    lift_rr = DMMotorInit(&lift_rr_config);
    DMMotorStop(lift_rr);

    remote_data = RemoteControlInit(&huart5);
}

/**
 * @brief 对居中的遥控通道应用死区并归一化。
 * @param[in] input 遥控通道原始值，正常范围为 -660 到 660。
 * @return 死区 [-10, 10] 内返回 0，其余返回按 660 归一化的有符号输入。
 */
static float RemoteAxis(int16_t input)
{
    if (input >= -10 && input <= 10)
    {
        return 0.0f;
    }
    return (float)input / 660.0f;
}

/**
 * @brief 统一切换麦轮、引导和侧边机构的 DJI 电机输出状态。
 * @param[in] enabled 为 true 时允许输出，为 false 时由驱动发送零电流指令。
 */
static void SetDjiMotorsEnabled(bool enabled)
{
    DJIMotorInstance *const dji_motors[] = {
        wheel_lf,    wheel_rf,       wheel_lr,        wheel_rr,       guide_left,
        guide_right, side_left_3508, side_right_3508, side_left_2006, side_right_2006,
    };

    for (unsigned int i = 0; i < sizeof(dji_motors) / sizeof(dji_motors[0]); ++i)
    {
        if (enabled)
        {
            DJIMotorEnable(dji_motors[i]);
        }
        else
        {
            DJIMotorStop(dji_motors[i]);
        }
    }
}

/**
 * @brief 根据左摇杆和拨轮输入解算四个麦轮的速度目标并更新速度环。
 * @param[in] remote 当前有效的遥控数据。
 * @details 车体前进、左移和逆时针旋转为正。四轮速度目标采用电机转子角速度，单位为 °/s。
 */
static void WheelTask(const RC_ctrl_t *remote)
{
    const float max_wheel_speed_deg_s = 55536.0f;
    const float forward = RemoteAxis(remote->rc.rocker_l1);
    const float left = RemoteAxis(remote->rc.rocker_l_);
    const float dial = RemoteAxis(remote->rc.dial);
    // 保留拨轮的转向符号，减小中心附近的灵敏度并平滑旋转指令。
    const float rotate_target = -0.50f * dial * fabsf(dial);

    rotate_command += 0.10f * (rotate_target - rotate_command);
    const float rotate = rotate_command;

    // 三轴叠加时统一缩放四轮目标，保持麦轮解算给出的速度比例。
    const float demand = fabsf(forward) + fabsf(left) + fabsf(rotate);
    const float speed_scale = max_wheel_speed_deg_s / fmaxf(1.0f, demand);

    DJIMotorSetPIDRef(wheel_lf, (forward - left - rotate) * speed_scale);
    DJIMotorSetPIDRef(wheel_rf, (forward + left + rotate) * speed_scale);
    DJIMotorSetPIDRef(wheel_lr, (forward + left - rotate) * speed_scale);
    DJIMotorSetPIDRef(wheel_rr, (forward - left + rotate) * speed_scale);
}

/**
 * @brief 使用共用上行轨迹寻找机械上挡块。
 * @param[in] motors 左前、右前、左后、右后的电机实例。
 * @param[in] dt 控制周期，单位为 s。
 * @return 回零可继续或已完成时返回 true，本次回零失败时返回 false。
 */
static bool LiftHomeTask(DMMotorInstance *const motors[4], float dt)
{

    static float all_stopped_s;

    if (!lift_home_started)
    {
        for (unsigned int i = 0; i < 4; ++i)
        {
            DMMotorInstance *motor = motors[i];
            const float threshold = kLiftHomeContactTorqueNm[i];
            const float limit = kLiftHomeTorqueLimitNm[i];

            if (threshold <= 0.0f || threshold >= limit || limit > motor->motor_controller.speed_PID.MaxOut)
                return false;

            lift_start_angle_rad[i] = motor->measure.total_angle;
            lift_speed_ff_rad_s[i] = 0.0f;
            motor->motor_controller.speed_feedforward_ptr = &lift_speed_ff_rad_s[i];
            motor->motor_settings.feedforward_flag |= SPEED_FEEDFORWARD;
        }

        lift_ref_mm = 0.0f;

        all_stopped_s = 0.0f;
        lift_home_started = true;
        return true;
    }

    bool all_at_stop = true;
    for (unsigned int i = 0; i < 4; ++i)
    {
        const float speed_mm_s =
            motors[i]->measure.velocity * kLiftPitchRadiusMm[i];
        const bool at_stop =
            fabsf(speed_mm_s) <= 0.5f &&
            motors[i]->measure.torque >= kLiftHomeContactTorqueNm[i];

        all_at_stop &= at_stop;
    }

    all_stopped_s = all_at_stop ? all_stopped_s + dt : 0.0f;

    if (lift_ref_mm >= 310.0f)
        return false;

    if (all_stopped_s >= 0.10f)
    {
        for (unsigned int i = 0; i < 4; ++i)
        {
            lift_start_angle_rad[i] = motors[i]->measure.total_angle;
            lift_speed_ff_rad_s[i] = 0.0f;
            DMMotorSetRef(motors[i], 0.0f);
        }

        lift_ref_mm = 0.0f;
        lift_target_mm = 0.0f;
        lift_reference_ready = true;
        return true;
    }


    lift_ref_mm = fminf(310.0f, lift_ref_mm + kLiftSpeedMmS * dt);

    for (unsigned int i = 0; i < 4; ++i)
    {
        DMMotorInstance *motor = motors[i];
        const float radius_mm = kLiftPitchRadiusMm[i];
        PIDInstance *speed_pid = &motor->motor_controller.speed_PID;
        const float normal_max_out = speed_pid->MaxOut;

        lift_speed_ff_rad_s[i] = kLiftSpeedMmS / radius_mm;
        speed_pid->MaxOut = kLiftHomeTorqueLimitNm[i];
        DMMotorSetPIDRef(motor, lift_start_angle_rad[i] + lift_ref_mm / radius_mm);
        speed_pid->MaxOut = normal_max_out;
        DMMotorEnable(motor);
    }

    return true;
}

/**
 * @brief 将共用齿条位移指令换算为四个抬升电机的角度目标，并补偿各轴的位移差。
 * @param[in] enabled 整机是否允许抬升电机输出。
 * @details 四路反馈均有效后采集各轴当前角度作为位移起点，清零旧位移指令。
 *          每周期在毫米坐标下推进共用轨迹，再换算为各轴的角度目标和速度前馈。
 *          失能或任一路反馈不可用时，四路均停止力矩输出并清除起点状态。
 */
static void LiftTask(bool enabled, float lift_stick)
{
    DMMotorInstance *const motors[] = {lift_lf, lift_rf, lift_lr, lift_rr};


    // 记录相邻两次调用的实际时间间隔，单位为秒。
    static uint32_t last_tick;
    const float dt = DWT_GetDeltaT(&last_tick);

    bool all_ready = enabled;
    for (unsigned int i = 0; i < 4 && all_ready; ++i)
    {
        DMMotorInstance *motor = motors[i];
        all_ready = motor->measure.state == 1u && DaemonIsOnline(motor->daemon);
    }

    if (!all_ready)
    {
        lift_home_started = false;
    }
    else if (!lift_reference_ready)
    {
        if (LiftHomeTask(motors, dt))
            return;

        all_ready = false;
    }

    if (!all_ready)
    {
        for (unsigned int i = 0; i < 4; ++i)
            DMMotorStop(motors[i]);

        return;
    }

    // 先更新共用位移参考，再为四路分别计算角度目标。
    float target_mm = fmaxf(kLiftMinTargetMm, fminf(0.0f, lift_target_mm));

    if (lift_stick > 0.0f)
    {
        target_mm = kLiftMinTargetMm;  // 上推，齿条向下伸出，车身抬高
    }
    else if (lift_stick < 0.0f)
    {
        target_mm = 0.0f;             // 下推，齿条向上收回
    }

    const float remaining_mm = target_mm - lift_ref_mm;
    float ref_speed_mm_s = 0.0f;

    if (fabsf(remaining_mm) <= kLiftSpeedMmS * dt)
    {
        lift_ref_mm = target_mm;
    }
    else
    {
        ref_speed_mm_s = copysignf(kLiftSpeedMmS, remaining_mm);
        lift_ref_mm += ref_speed_mm_s * dt;
    }

    if (lift_stick != 0.0f)
    {
        lift_target_mm = lift_ref_mm;
    }

    // 齿条位移等于相对起点的转角乘以分度圆半径。
    float sum_mm = 0.0f;
    for (unsigned int i = 0; i < 4; ++i)
    {
        lift_position_mm[i] = (motors[i]->measure.total_angle - lift_start_angle_rad[i]) * kLiftPitchRadiusMm[i];
        sum_mm += lift_position_mm[i];
    }
    const float average_mm = sum_mm / 4.0f;

    for (unsigned int i = 0; i < 4; ++i)
    {
        const float radius_mm = kLiftPitchRadiusMm[i];

        // 由各轴相对平均位移的偏差计算速度纠偏，再换算为 rad/s。
        const float sync_mm_s = kLiftSyncGain * (average_mm - lift_position_mm[i]);
        const float correction_mm_s = fmaxf(-kLiftMaxSyncMmS, fminf(kLiftMaxSyncMmS, sync_mm_s));

        lift_speed_ff_rad_s[i] = (ref_speed_mm_s + correction_mm_s) / radius_mm;
        const float angle_ref_rad = lift_start_angle_rad[i] + lift_ref_mm / radius_mm;

        DMMotorSetPIDRef(motors[i], angle_ref_rad);
        DMMotorEnable(motors[i]);
    }
}

/**
 * @brief 周期处理遥控状态，并调度麦轮速度控制与四路抬升。
 * @details 遥控在线且右拨杆处于中位时允许控制。其他状态下停止电机输出并清除旋转指令。
 *          恢复控制时，先更新本周期的运动目标，再开放 DJI 电机输出。
 */
void RobotTask(void)
{
    const RC_ctrl_t *remote = &remote_data[TEMP];

    if (!RemoteControlIsOnline() || !switch_is_mid(remote->rc.switch_right))
    {
        LiftTask(false, 0.0f);

        if (motors_enabled)
        {
            SetDjiMotorsEnabled(false);
            motors_enabled = false;
        }

        rotate_command = 0.0f;
        return;
    }

    WheelTask(remote);
    LiftTask(true, RemoteAxis(remote->rc.rocker_r1));

    if (!motors_enabled)
    {
        SetDjiMotorsEnabled(true);
        motors_enabled = true;
    }
}
