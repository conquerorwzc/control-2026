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
static volatile float guide_offset_deg[2]; // 左、右引导相对使能位置的转子角度
static DJIMotorInstance *side_left_3508;
static DJIMotorInstance *side_right_3508;
static DJIMotorInstance *side_left_2006;
static DJIMotorInstance *side_right_2006;

static DMMotorInstance *lift_lf;
static DMMotorInstance *lift_rf;
static DMMotorInstance *lift_lr;
static DMMotorInstance *lift_rr;

// 抬升数组按左前、右前、左后、右后排列。分度圆半径为模数 × 齿数 / 2，单位 mm。
static const float kLiftPitchRadiusMm[4] = {30.0f, 30.0f, 50.0f, 50.0f};
static const float kLiftSpeedMmS = 50.0f;  // 回零和正常升降共用的齿条轨迹速度，单位 mm/s
static const float kLiftSyncGain = 2.0f;   // 将各轴相对平均位移的偏差换算为纠偏速度，单位 1/s
static const float kLiftMaxSyncMmS = 1.0f; // 单轴纠偏速度的幅值上限，单位 mm/s

static volatile float lift_target_mm;      // 松杆后的保持目标，可在摇杆回中时由 Ozone 写入
static volatile float lift_ref_mm;         // 四路共用的轨迹参考，回零后以上挡块为 0 mm
static volatile float lift_position_mm[4]; // 各齿条相对上挡块的实测位移，回零后更新
static float lift_start_angle_rad[4];      // 回零时保存搜索起始角，触顶后改存各轴零位角
static float lift_speed_ff_rad_s[4];       // 叠加到各轴速度环参考值的角速度前馈，单位 rad/s

static bool lift_reference_ready; // 四路上挡块零位已建立
static bool lift_home_started;    // 本次回零搜索已初始化

static RC_ctrl_t *remote_data;
static bool motors_enabled; // 记录 DJI 电机的输出状态，便于在挡位切换时更新
static float rotate_command;

/**
 * @brief 注册四个麦轮、六个引导和侧边机构电机、四个抬升电机，并初始化遥控接收。
 * @details 注册后停止各电机的控制输出，等待周期任务根据遥控状态开放输出。
 *          遥控接收使用 UART5 的 DBUS 接口。
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
 * @brief 对已减去中位偏置的遥控通道应用死区并归一化。
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
        wheel_lf, wheel_rf,
        wheel_lr, wheel_rr,
        side_left_3508,
        side_right_3508,
        side_left_2006,
        side_right_2006,
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
 * @details 左摇杆竖直控制前后，水平控制横移，左拨轮控制自转。横移通道按实车方向直接取值。
 *          解算采用前进、左移和逆时针为正，四轮速度目标为电机转子角速度，单位为 °/s。
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
 * @brief 以共用上行轨迹寻找机械上挡块，建立四路各自的零位。
 * @param[in,out] motors 按左前、右前、左后、右后排列的抬升电机实例。
 * @param[in] dt 相邻两次任务调用的实际时间间隔，单位为 s。
 * @return 正在回零或本周期完成回零时返回 true。力矩配置无效或达到搜索行程上限时返回 false，由调用方停止输出。
 * @pre 调用方已确认允许抬升输出，且四路驱动反馈有效。
 */
static bool LiftHomeTask(DMMotorInstance *const motors[4], float dt)
{
    static float all_stopped_s;

    if (!lift_home_started)
    {
        // 用各轴当前角度作为搜索起点，同时绑定后续共用的速度前馈。
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

    // 用齿条线速度统一判断前后轴是否停住，再检查向上的反馈力矩。
    bool all_at_stop = true;
    for (unsigned int i = 0; i < 4; ++i)
    {
        const float speed_mm_s = motors[i]->measure.velocity * kLiftPitchRadiusMm[i];
        const bool at_stop = fabsf(speed_mm_s) <= 0.5f && motors[i]->measure.torque >= kLiftHomeContactTorqueNm[i];

        all_at_stop &= at_stop;
    }

    // 四轴须同时持续满足条件，任一轴未满足就重新计时。
    all_stopped_s = all_at_stop ? all_stopped_s + dt : 0.0f;

    // 搜索上限按共用参考位移判断，避免找不到上挡块时继续推进指令。
    if (lift_ref_mm >= 310.0f)
        return false;

    if (all_stopped_s >= 0.10f)
    {
        // 以各轴此刻的实测角度建立 0 mm，并撤掉本周期的触顶力矩。
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

    // 在毫米坐标推进，再按各自的齿轮半径换算成角度目标和角速度前馈。
    lift_ref_mm = fminf(310.0f, lift_ref_mm + kLiftSpeedMmS * dt);

    for (unsigned int i = 0; i < 4; ++i)
    {
        DMMotorInstance *motor = motors[i];
        const float radius_mm = kLiftPitchRadiusMm[i];
        PIDInstance *speed_pid = &motor->motor_controller.speed_PID;
        const float normal_max_out = speed_pid->MaxOut;

        lift_speed_ff_rad_s[i] = kLiftSpeedMmS / radius_mm;
        // DMMotorSetPIDRef 会立即计算力矩，因此仅在本次计算期间使用回零上限。
        speed_pid->MaxOut = kLiftHomeTorqueLimitNm[i];
        DMMotorSetPIDRef(motor, lift_start_angle_rad[i] + lift_ref_mm / radius_mm);
        speed_pid->MaxOut = normal_max_out;
        DMMotorEnable(motor);
    }

    return true;
}

/**
 * @brief 管理四路抬升回零、遥控升降和齿条同步。
 * @param[in] enabled 整机是否允许抬升电机输出。
 * @param[in] lift_stick 应用死区后的右摇杆竖直输入。正值伸出齿条，负值收回，零值保持位置目标。
 * @details 四路反馈有效时先完成机械回零。回零后，四轴以上挡块为 0 mm，向下伸出为负位移。
 *          四轴跟随同一条位移轨迹，以各轴相对平均位移的偏差修正速度前馈。
 *          失能或反馈不可用时停止四路力矩输出，并重置尚未完成的回零搜索。
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

    // 摇杆回中时采用保持目标或 Ozone 指令，推杆时向对应行程端点推进。
    float target_mm = fmaxf(kLiftMinTargetMm, fminf(0.0f, lift_target_mm));

    if (lift_stick > 0.0f)
    {
        target_mm = kLiftMinTargetMm; // 上推，齿条向下伸出，车身抬高
    }
    else if (lift_stick < 0.0f)
    {
        target_mm = 0.0f; // 下推，齿条向上收回
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

    // 记下这一周期到达的轨迹位置，供松杆后保持。
    if (lift_stick != 0.0f)
    {
        lift_target_mm = lift_ref_mm;
    }

    // 齿条位移等于相对零位的转角乘以分度圆半径。
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

        // 纠偏量叠加在齿条速度前馈上，角度目标仍取自同一位移轨迹。
        const float sync_mm_s = kLiftSyncGain * (average_mm - lift_position_mm[i]);
        const float correction_mm_s = fmaxf(-kLiftMaxSyncMmS, fminf(kLiftMaxSyncMmS, sync_mm_s));

        lift_speed_ff_rad_s[i] = (ref_speed_mm_s + correction_mm_s) / radius_mm;
        const float angle_ref_rad = lift_start_angle_rad[i] + lift_ref_mm / radius_mm;

        DMMotorSetPIDRef(motors[i], angle_ref_rad);
        DMMotorEnable(motors[i]);
    }
}

/**
 * @brief 周期运行左右引导电机的位置环。
 * @param[in] enabled 整机是否允许引导电机输出。
 * @details 每轴收到有效反馈并使能后，以当前转子角度为起点。
 *          guide_offset_deg 为相对起点的角度指令，单位为 °。
 */
static void GuideTask(bool enabled)
{
    DJIMotorInstance *const motors[] = {guide_left, guide_right};
    static float start_angle_deg[2];
    static bool reference_ready[2];

    for (unsigned int i = 0; i < 2; ++i)
    {
        DJIMotorInstance *motor = motors[i];

        if (!enabled || !DaemonIsOnline(motor->daemon) || motor->motor_can_instance->rx_len != 8u)
        {
            DJIMotorStop(motor);
            reference_ready[i] = false;
            continue;
        }

        if (!reference_ready[i])
        {
            start_angle_deg[i] = motor->measure.total_angle;
            guide_offset_deg[i] = 0.0f;
            reference_ready[i] = true;
        }

        DJIMotorSetPIDRef(motor, start_angle_deg[i] + guide_offset_deg[i]);
        DJIMotorEnable(motor);
    }
}

/**
 * @brief 周期处理遥控状态，并调度麦轮速度控制与四路抬升。
 * @details 遥控在线且右拨杆居中时，左摇杆和拨轮控制麦轮，右摇杆竖直通道控制抬升。
 *          撤销使能时停止电机输出并清除旋转滤波状态。恢复使能时先更新目标，再开放 DJI 电机输出。
 */
void RobotTask(void)
{
    const RC_ctrl_t *remote = &remote_data[TEMP];

    if (!RemoteControlIsOnline() || !switch_is_mid(remote->rc.switch_right))
    {
        LiftTask(false, 0.0f);
        GuideTask(false);

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
    GuideTask(true);

    if (!motors_enabled)
    {
        SetDjiMotorsEnabled(true);
        motors_enabled = true;
    }
}
