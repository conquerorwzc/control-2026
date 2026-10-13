#include "observer_vofa.h"

#include "vofa.h"
#include "FreeRTOS.h"
#include "task.h"

enum { kChannelCount = 25, kPeriodMs = 20 };
_Static_assert(kChannelCount <= VOFA_MAX_CHANNELS, "observer telemetry exceeds VOFA buffer");

static UART_HandleTypeDef *observer_uart;
static uint32_t last_tick;
static uint32_t sequence;

void ObserverVOFAInit(UART_HandleTypeDef *huart)
{
    if (observer_uart != NULL || huart == NULL || huart->hdmatx == NULL) return;
    observer_uart = huart;
    last_tick = HAL_GetTick();
    sequence = 0u;
    VOFAInit(huart);
}

static uint8_t TelemetryReady(uint32_t period_ms)
{
    if (observer_uart == NULL || period_ms == 0u) return 0u;
    // 25 float + 帧尾，8N1 共 1040 bit/帧；按当前实际波特率保留至少 20% 带宽。
    if ((uint64_t)observer_uart->Init.BaudRate * period_ms * 4u <
        (uint64_t)(kChannelCount * sizeof(float) + 4u) * 10u * 1000u * 5u) return 0u;
    const uint32_t now = HAL_GetTick();
    const uint32_t slots = (uint32_t)(now - last_tick) / period_ms;
    if (slots == 0u) return 0u;
    last_tick += slots * period_ms;
    sequence = (sequence + slots) & 0x00ffffffu;

    // 必须在 VOFAJustFloatSend 改写其静态 DMA 缓冲区前判断发送是否结束。
    // 不使用 BSP 的 USARTIsReady：其当前实现用按位或判断，始终返回未就绪。
    return observer_uart->gState == HAL_UART_STATE_READY;
}

static uint8_t MotorIndex(uint8_t motor_id)
{
    // 数组为 LF/ID1、RF/ID4、LB/ID2、RB/ID3，不是 CAN ID 顺序。
    static const uint8_t kIdToIndex[4] = {0u, 2u, 3u, 1u};
    return kIdToIndex[motor_id - 1u];
}

typedef struct {
    float current, temperature, dt, applied_output;
    uint8_t enabled;
} MotorTelemetry;

static MotorTelemetry MotorSnapshot(const DJIMotorInstance *motor)
{
    MotorTelemetry sample;
    taskENTER_CRITICAL();
    sample.current = motor->measure.real_current;
    sample.temperature = motor->measure.temperature;
    sample.dt = motor->dt;
    sample.enabled = motor->stop_flag != MOTOR_STOP;
    // 逻辑极性；实际 CAN 打包还会执行 motor_reverse_flag 的方向变换和 int16 转换。
    sample.applied_output = sample.enabled ? motor->motor_controller.final_output : 0.0f;
    taskEXIT_CRITICAL();
    return sample;
}

void ObserverVOFASend(const ChassisInstance *chassis, uint8_t motor_id)
{
    if (chassis == NULL || motor_id < 1u || motor_id > 4u) return;
    const uint8_t index = MotorIndex(motor_id);
    if (chassis->wheel_motor[index] == NULL || !TelemetryReady(kPeriodMs)) return;
    const MotorTelemetry motor = MotorSnapshot(chassis->wheel_motor[index]);

    const ChassisObserverTelemetry_s *sample = &chassis->observer_telemetry;
    float data[kChannelCount] = {
        (float)sequence,                  // 0: 采集时隙序号，跳号表示漏采/漏帧
        sample->sample_time,              // 1: 采样时刻 s
        sample->dt,                       // 2: 观测器 dt s
        sample->gyro[0],                  // 3: gyro_x rad/s
        sample->gyro[1],                  // 4: gyro_y rad/s
        sample->gyro[2],                  // 5: gyro_z rad/s
        sample->motion_accel[0],          // 6: IMU 点去重力 ax m/s^2
        sample->motion_accel[1],          // 7: IMU 点去重力 ay m/s^2
        sample->motion_accel[2],          // 8: IMU 点去重力 az m/s^2
        sample->center_accel[0],          // 9: 杆臂补偿后的中心 ax m/s^2
        sample->center_accel[1],          // 10: 杆臂补偿后的中心 ay m/s^2
        chassis->speed_observer.wheel.vx,    // 11: 轮速 vx m/s
        chassis->speed_observer.wheel.vy,    // 12: 轮速 vy m/s
        chassis->speed_observer.wheel.wz,    // 13: 轮速 wz rad/s
        chassis->speed_observer.fused.vx,   // 14: EKF vx m/s
        chassis->speed_observer.fused.vy,   // 15: EKF vy m/s
        chassis->speed_observer.fused.wz,   // 16: EKF wz rad/s
        sample->motor_speed[index],       // 17: 所选电机轴 deg/s
        motor.current,                    // 18: 所选电机反馈电流，协议码值
        motor.temperature,                // 19: 所选电机温度 degC
        (float)motor_id,                   // 20: 所选电机 CAN ID 1~4
        sample->temperature,              // 21: BMI088 温度 degC
        (float)sample->status,            // 22: 有效性位图
        (float)sample->imu_count,         // 23: INS 更新计数，模 2^24
        (sample->motor_speed[0] - sample->motor_speed[1] - sample->motor_speed[2] + sample->motor_speed[3]) *
            (0.25f / chassis->speed_observer.model.motor_per_linear), // 24: 四轮运动学残差 m/s
    };
    VOFAJustFloatSend(data, kChannelCount);
}

static void FourMotorVOFASend(const ChassisInstance *chassis, uint32_t period_ms)
{
    typedef struct {
        float ref, measure, output, applied, current, dt, max_out;
        uint8_t enabled;
    } MotorSample;
    MotorSample motors[4];
    for (int id = 1; id <= 4; ++id)
        if (chassis->wheel_motor[MotorIndex((uint8_t)id)] == NULL) return;
    if (!TelemetryReady(period_ms)) return;

    // 同一控制周期的四个 PID 输入/输出；CAN 反馈电流复制在短临界区内。
    // 同帧不表示四个电机的 CAN 测速在硬件上同时采样。
    taskENTER_CRITICAL();
    for (int id = 1; id <= 4; ++id)
    {
        const DJIMotorInstance *motor = chassis->wheel_motor[MotorIndex((uint8_t)id)];
        const PIDInstance *pid = &motor->motor_controller.speed_PID;
        motors[id - 1] = (MotorSample){
            .ref = pid->Ref, .measure = pid->Measure, .output = pid->Output,
            .applied = motor->motor_controller.final_output, .current = motor->measure.real_current,
            .dt = pid->dt, .max_out = pid->MaxOut, .enabled = motor->stop_flag != MOTOR_STOP,
        };
    }
    taskEXIT_CRITICAL();

    uint32_t status = chassis->observer_telemetry.status;
    float max_dt = 0.0f;
    float data[kChannelCount] = {
        (float)sequence, chassis->observer_telemetry.sample_time, (float)CHASSIS_TUNING_FOUR_MOTORS,
    };
    for (int i = 0; i < 4; ++i)
    {
        const MotorSample *motor = &motors[i];
        if (motor->dt > max_dt) max_dt = motor->dt;
        if (motor->enabled) status |= 1u << (16 + i);
        if (motor->enabled && fabsf(motor->applied - motor->output) > 1.0f) status |= 1u << 21;
        if (motor->max_out > 0.0f && fabsf(motor->output) >= motor->max_out) status |= 1u << 22;
        const int base = 5 + 5 * i;
        data[base] = motor->ref;
        data[base + 1] = motor->measure;
        data[base + 2] = motor->output;
        data[base + 3] = motor->enabled ? motor->applied : 0.0f;
        data[base + 4] = motor->current;
    }
    if (chassis->speed_hold_active) status |= 1u << 20;
    data[3] = max_dt;
    data[4] = (float)status;
    VOFAJustFloatSend(data, kChannelCount);
}

// 同帧记录平移与旋转，避免单轴页遗漏平移刹停后的 wz 和积分状态。
static void TranslationWzVOFASend(const ChassisInstance *chassis, uint8_t motor_id, uint32_t period_ms)
{
    const uint8_t index = MotorIndex(motor_id);
    const DJIMotorInstance *wheel = chassis->wheel_motor[index];
    if (wheel == NULL || !TelemetryReady(period_ms)) return;
    const MotorTelemetry motor = MotorSnapshot(wheel);
    uint32_t status = chassis->observer_telemetry.status;
    status |= ((uint32_t)chassis->chassis_ctrl_cmd.chassis_mode & 7u) << 16;
    if (motor.enabled) status |= 1u << 19;
    if (status & (1u << (8u + index))) status |= 1u << 20;
    if (chassis->speed_hold_updated) status |= 1u << 21;

    float data[kChannelCount] = {
        (float)sequence, chassis->observer_telemetry.sample_time,
        (float)CHASSIS_TUNING_TRANSLATION_WZ, (float)status,             // 0~3
        chassis->target_velocity.vx, chassis->speed_observer.fused.vx,  // 4~5
        chassis->vx_hold_pid.Output, chassis->control_velocity.vx,     // 6~7
        chassis->target_velocity.vy, chassis->speed_observer.fused.vy,  // 8~9
        chassis->target_velocity.wz, chassis->speed_observer.fused.wz,  // 10~11
        chassis->wz_hold_pid.Pout, chassis->wz_hold_pid.Iout,            // 12~13
        chassis->wz_hold_pid.Output, chassis->control_velocity.wz,      // 14~15
        chassis->speed_observer.wheel.vx, chassis->speed_observer.wheel.vy, // 16~17
        chassis->speed_observer.wheel.wz, chassis->observer_telemetry.gyro[2], // 18~19
        chassis->vx_hold_pid.Kp, chassis->vx_hold_pid.Ki,                // 20~21
        chassis->wz_hold_pid.Kp, chassis->wz_hold_pid.Ki,                // 22~23
        motor.applied_output,                                         // 24: 代表电机逻辑电流指令
    };
    VOFAJustFloatSend(data, kChannelCount);
}

// 三轴使用对称的目标/反馈/修正/逆解通道，保持 25 路和既有串口带宽。
static void ThreeAxesVOFASend(const ChassisInstance *chassis, uint8_t motor_id, uint32_t period_ms)
{
    const uint8_t index = MotorIndex(motor_id);
    const DJIMotorInstance *wheel = chassis->wheel_motor[index];
    if (wheel == NULL || !TelemetryReady(period_ms)) return;
    const MotorTelemetry motor = MotorSnapshot(wheel);
    uint32_t status = chassis->observer_telemetry.status;
    status |= ((uint32_t)chassis->chassis_ctrl_cmd.chassis_mode & 7u) << 16;
    if (motor.enabled) status |= 1u << 19;
    if (status & (1u << (8u + index))) status |= 1u << 20;
    if (chassis->speed_hold_updated) status |= 1u << 21;

    float data[kChannelCount] = {
        (float)sequence, chassis->observer_telemetry.sample_time,
        (float)CHASSIS_TUNING_THREE_AXES, (float)status,                  // 0~3
        chassis->target_velocity.vx, chassis->speed_observer.fused.vx,   // 4~5
        chassis->vx_hold_pid.Output, chassis->control_velocity.vx,      // 6~7
        chassis->target_velocity.vy, chassis->speed_observer.fused.vy,   // 8~9
        chassis->vy_hold_pid.Output, chassis->control_velocity.vy,      // 10~11
        chassis->target_velocity.wz, chassis->speed_observer.fused.wz,   // 12~13
        chassis->wz_hold_pid.Output, chassis->control_velocity.wz,      // 14~15
        chassis->speed_observer.wheel.vx, chassis->speed_observer.wheel.vy, // 16~17
        chassis->speed_observer.wheel.wz, chassis->observer_telemetry.gyro[2], // 18~19
        chassis->vx_hold_pid.Kp, chassis->vy_hold_pid.Kp, chassis->wz_hold_pid.Kp, // 20~22
        (float)chassis->chassis_ctrl_cmd.max_power, motor.applied_output, // 23~24
    };
    VOFAJustFloatSend(data, kChannelCount);
}

// 同帧对齐起停、航向和限功率前后四轮指令，保持既有 25 路/104 字节带宽。
static void StartStopVOFASend(const ChassisInstance *chassis, uint32_t period_ms)
{
    for (int i = 0; i < 4; ++i)
        if (chassis->wheel_motor[i] == NULL) return;
    if (!TelemetryReady(period_ms)) return;

    float yaw_measure = 0.0f;
    uint8_t all_enabled = 1u;
    float limited[4];
    taskENTER_CRITICAL();
    if (chassis->yaw_hold_updated)
        yaw_measure = chassis->yaw_hold_pid.Measure;
    else if (chassis->chassis_IMU != NULL)
        yaw_measure = chassis->chassis_IMU->YawTotalAngle;
    for (int id = 1; id <= 4; ++id)
    {
        const uint8_t index = MotorIndex((uint8_t)id);
        const uint8_t enabled = chassis->wheel_motor[index]->stop_flag != MOTOR_STOP;
        if (!enabled) all_enabled = 0u;
        limited[id - 1] = enabled ? chassis->observer_telemetry.motor_limited_current[index] : 0.0f;
    }
    taskEXIT_CRITICAL();

    uint32_t status = chassis->observer_telemetry.status;
    status |= ((uint32_t)chassis->chassis_ctrl_cmd.chassis_mode & 7u) << 16;
    if (all_enabled) status |= 1u << 19;
    if ((status & (0x0fu << 8)) == (0x0fu << 8)) status |= 1u << 20;
    if (chassis->speed_hold_updated && chassis->yaw_hold_updated) status |= 1u << 21;
    const float yaw_ref = chassis->yaw_hold_updated ? chassis->yaw_hold_pid.Ref :
                                                   chassis->chassis_ctrl_cmd.yaw_hold_ref;
    float data[kChannelCount] = {
        (float)sequence, chassis->observer_telemetry.sample_time,
        (float)CHASSIS_TUNING_START_STOP, (float)status,                // 0~3
        yaw_ref, yaw_measure, yaw_ref - yaw_measure,                   // 4~6: deg
        chassis->yaw_hold_updated ? chassis->yaw_hold_pid.Output : 0.0f, // 7: rad/s，本轮角环贡献
        chassis->target_velocity.vx, chassis->speed_observer.fused.vx, // 8~9: m/s
        chassis->target_velocity.vy, chassis->speed_observer.fused.vy, // 10~11: m/s
        chassis->target_velocity.wz, chassis->speed_observer.fused.wz, // 12~13: rad/s
        chassis->control_velocity.wz,                                // 14: rad/s
        chassis->observer_telemetry.requested_power,                  // 15: model W
        chassis->observer_telemetry.power_limit_scale,                // 16: 0~1
    };
    for (int id = 1; id <= 4; ++id)
    {
        data[16 + id] = chassis->observer_telemetry.motor_requested_current[MotorIndex((uint8_t)id)];
        data[20 + id] = limited[id - 1]; // 逻辑极性/协议码值；CAN 打包还会执行电机方向变换。
    }
    VOFAJustFloatSend(data, kChannelCount);
}

void ChassisTuningVOFASend(const ChassisInstance *chassis, ChassisTuningView_e view,
                          uint8_t motor_id, uint32_t period_ms)
{
    if (chassis == NULL || (unsigned)view > CHASSIS_TUNING_START_STOP ||
        motor_id < 1u || motor_id > 4u || period_ms == 0u) return;
    if (view == CHASSIS_TUNING_FOUR_MOTORS)
    {
        FourMotorVOFASend(chassis, period_ms);
        return;
    }
    if (view == CHASSIS_TUNING_TRANSLATION_WZ)
    {
        TranslationWzVOFASend(chassis, motor_id, period_ms);
        return;
    }
    if (view == CHASSIS_TUNING_THREE_AXES)
    {
        ThreeAxesVOFASend(chassis, motor_id, period_ms);
        return;
    }
    if (view == CHASSIS_TUNING_START_STOP)
    {
        StartStopVOFASend(chassis, period_ms);
        return;
    }
    const uint8_t index = MotorIndex(motor_id);
    const DJIMotorInstance *wheel = chassis->wheel_motor[index];
    if (wheel == NULL || !TelemetryReady(period_ms)) return;

    const MotorTelemetry motor = MotorSnapshot(wheel);
    const PIDInstance *pid = &wheel->motor_controller.speed_PID;
    float ref = pid->Ref, measure = pid->Measure;
    float applied = motor.applied_output;
    float aux1 = motor.current, aux2 = motor.temperature, aux3 = motor.dt;
    uint8_t pid_updated = 1u; // ChassisTask 每周期调用电机 PID，即使 stop_flag 使 CAN 输出为零。

    if (view == CHASSIS_TUNING_VX || view == CHASSIS_TUNING_VY || view == CHASSIS_TUNING_WZ) {
        if (view == CHASSIS_TUNING_VX) {
            pid = &chassis->vx_hold_pid;
            ref = chassis->target_velocity.vx;
            measure = chassis->speed_observer.fused.vx;
            applied = chassis->control_velocity.vx;
            aux1 = chassis->speed_observer.wheel.vx;
            aux2 = chassis->observer_telemetry.center_accel[0];
        } else if (view == CHASSIS_TUNING_VY) {
            pid = &chassis->vy_hold_pid;
            ref = chassis->target_velocity.vy;
            measure = chassis->speed_observer.fused.vy;
            applied = chassis->control_velocity.vy;
            aux1 = chassis->speed_observer.wheel.vy;
            aux2 = chassis->observer_telemetry.center_accel[1];
        } else {
            pid = &chassis->wz_hold_pid;
            ref = chassis->target_velocity.wz;
            measure = chassis->speed_observer.fused.wz;
            applied = chassis->control_velocity.wz;
            aux1 = chassis->speed_observer.wheel.wz;
            aux2 = chassis->observer_telemetry.gyro[2];
        }
        aux3 = motor.applied_output;
        pid_updated = chassis->speed_hold_updated;
    } else if (view == CHASSIS_TUNING_YAW) {
        pid = &chassis->yaw_hold_pid;
        pid_updated = chassis->yaw_hold_updated;
        ref = pid_updated ? pid->Ref : chassis->chassis_ctrl_cmd.yaw_hold_ref;
        measure = 0.0f;
        if (pid_updated) measure = pid->Measure;
        else if (chassis->chassis_IMU != NULL) {
            taskENTER_CRITICAL();
            measure = chassis->chassis_IMU->YawTotalAngle;
            taskEXIT_CRITICAL();
        }
        applied = chassis->control_velocity.wz;
        aux1 = chassis->target_velocity.wz;
        aux2 = chassis->speed_observer.fused.wz;
        aux3 = chassis->observer_telemetry.gyro[2];
    }

    uint32_t status = chassis->observer_telemetry.status;
    status |= ((uint32_t)chassis->chassis_ctrl_cmd.chassis_mode & 7u) << 16;
    if (motor.enabled) status |= 1u << 19;
    if (status & (1u << (8u + index))) status |= 1u << 20;
    if (pid_updated) status |= 1u << 21;

    float data[kChannelCount] = {
        (float)sequence, chassis->observer_telemetry.sample_time, (float)view, (float)motor_id,
        ref, measure, pid->Err, pid->Pout, pid->Iout, pid->Dout, pid->Output, applied,
        pid->Kp, pid->Ki, pid->Kd, pid->MaxOut, pid->IntegralLimit, pid->DeadBand, pid->dt,
        aux1, aux2, aux3, (float)status, (float)chassis->chassis_ctrl_cmd.max_power, (float)pid->Improve,
    };
    VOFAJustFloatSend(data, kChannelCount);
}
