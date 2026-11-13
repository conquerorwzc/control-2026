#include "speed_observer.h"

#include "bsp_dwt.h"
#include "user_lib.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    float radius, x_projection, y_projection, motor_per_linear;
} OmniKinematics;

typedef struct {
    float last_gyro, angular_accel, squared_wz;
    uint8_t seeded;
} SpeedObserverLeverArm_s; // 杆臂补偿

struct SpeedObserverInstance
{
    SpeedObserverResult_s output;
    OmniKinematics model;
    SpeedObserverConfig_s config;
    float covariance[3][3];
    uint8_t initialized, seeded, have_accel;
    float imu_offset[2], timeout;
    SpeedObserverLeverArm_s lever_arm;
    float center_accel[2];
    uint64_t imu_stamp;
    uint32_t wheel_stamp[4], wheel_seen[4];
    uint32_t dt;
    float wheel_age[4], imu_age, correction_age;
};

// 长度为 m，减速比为电机转速 / 轮转速；几何尺寸和减速比应为正。
static OmniKinematics OmniKinematicsInit(float wheel_base, float track_width, float wheel_radius, float reduction)
{
    const float half_base = wheel_base * 0.5f;
    const float half_track = track_width * 0.5f;
    OmniKinematics model;
    model.radius = sqrtf(half_base * half_base + half_track * half_track);
    model.x_projection = half_track / model.radius;
    model.y_projection = half_base / model.radius;
    model.motor_per_linear = reduction * (180.0f / 3.14159265358979323846f) / wheel_radius;
    return model;
}

// 使用实测电机轴速度 deg/s 进行最小二乘正解。
static Speed_s OmniForward(const OmniKinematics *model, const float motor_speed[4])
{
    const float scale = 0.25f / model->motor_per_linear;
    Speed_s body;
    body.vx = (-motor_speed[0] + motor_speed[1] - motor_speed[2] + motor_speed[3]) * scale / model->x_projection;
    body.vy = (motor_speed[0] + motor_speed[1] - motor_speed[2] - motor_speed[3]) * scale / model->y_projection;
    body.wz = (motor_speed[0] + motor_speed[1] + motor_speed[2] + motor_speed[3]) * scale / model->radius;
    return body;
}

static void OmniInverse(const OmniKinematics *model, const Speed_s *body, float motor_speed[4])
{
    const float x = model->x_projection * body->vx;
    const float y = model->y_projection * body->vy;
    const float spin = model->radius * body->wz;
    motor_speed[0] = (-x + y + spin) * model->motor_per_linear;
    motor_speed[1] = (x + y + spin) * model->motor_per_linear;
    motor_speed[2] = (-x - y + spin) * model->motor_per_linear;
    motor_speed[3] = (x - y + spin) * model->motor_per_linear;
}
// 按 IEEE-754 位模式检查，避免 -Ofast 优化掉 NaN/Inf 判断。
static uint8_t Finite(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

static uint8_t CheckInit(SpeedObserverInstance *observer, const SpeedObserver_Init_Config_s *init)
{
    if (observer == NULL || init == NULL)
        return 0u;
    const SpeedObserverConfig_s *config = &init->filter;
    memset(observer, 0, sizeof(*observer));
    // 固定滤波参数由机器人配置保证；这里只检查运动学除数和时间限制。
    const float positive[] = {init->wheel_base, init->track_width, init->wheel_radius,
                              init->reduction, init->sensor_timeout, config->max_dt};
    for (int i = 0; i < 6; ++i)
        if (!Finite(positive[i]) || positive[i] <= 0.0f)
            return 0u;
    return 1u;
}

SpeedObserverInstance *SpeedObserverInit(const SpeedObserver_Init_Config_s *init)
{
    SpeedObserverInstance *observer = zmalloc(sizeof(*observer));
    if (observer == NULL)
        return NULL;
    if (!CheckInit(observer, init))
    {
        free(observer);
        return NULL;
    }
    observer->model = OmniKinematicsInit(init->wheel_base, init->track_width, init->wheel_radius, init->reduction);
    observer->timeout = init->sensor_timeout;
    observer->correction_age = init->sensor_timeout + 1.0f;
    memcpy(observer->imu_offset, init->imu_offset, sizeof(observer->imu_offset));
    observer->config = init->filter;
    observer->initialized = 1u;
    DWT_GetDeltaT(&observer->dt);
    return observer;
}

typedef struct
{
    Speed_s wheel;
    float gyro_z, accel_x, accel_y;
    uint8_t wheel_valid, gyro_valid, accel_valid;
} FilterInput_s;

typedef struct
{
    Speed_s fused;
    float covariance[3][3];
} FilterResult_s;

typedef struct
{
    FilterInput_s sample;
    uint8_t dt_valid, feedback_valid, imu_new, accel_new;
} ObserverInput_s;

// 单轴量测校正，Joseph 形式保持协方差对称与半正定。
static void Correct(float state[3], float p[3][3], int axis, float measurement, float variance)
{
    float gain[3], a[3][3], ap[3][3] = {{0}}, updated[3][3] = {{0}};
    const float innovation = measurement - state[axis];
    const float s = p[axis][axis] + variance;
    for (int i = 0; i < 3; ++i)
    {
        gain[i] = p[i][axis] / s;
        state[i] += gain[i] * innovation;
        for (int j = 0; j < 3; ++j)
            a[i][j] = (i == j ? 1.0f : 0.0f) - (j == axis ? gain[i] : 0.0f);
    }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                ap[i][j] += a[i][k] * p[k][j];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            for (int k = 0; k < 3; ++k)
                updated[i][j] += ap[i][k] * a[j][k];
            updated[i][j] += gain[i] * variance * gain[j];
        }
    memcpy(p, updated, sizeof(updated));
}

static void FilterCalculate(const SpeedObserverInstance *observer, const FilterInput_s *input, float dt,
                            FilterResult_s *result)
{
    // 输入已经通过 CheckInput；这里只计算候选结果。
    float state[3];
    float (*p)[3] = result->covariance;
    memset(p, 0, sizeof(result->covariance));
    if (!observer->seeded)
    {
        // 首帧轮速只用于初始化，不重复校正。
        state[0] = input->wheel.vx;
        state[1] = input->wheel.vy;
        state[2] = input->wheel.wz;
        for (int i = 0; i < 3; ++i)
            p[i][i] = observer->config.initial_variance[i];
    }
    else
    {
        state[0] = observer->output.fused.vx;
        state[1] = observer->output.fused.vy;
        state[2] = observer->output.fused.wz;
        float f[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        float fp[3][3] = {{0}};
        if (input->accel_valid)
        {
            // 机体系速度导数 = 机体系惯性加速度 - 角速度叉乘速度。
            f[0][1] = state[2] * dt;
            f[0][2] = state[1] * dt;
            f[1][0] = -state[2] * dt;
            f[1][2] = -state[0] * dt;
            const float vx = state[0], vy = state[1];
            state[0] += (input->accel_x + state[2] * vy) * dt;
            state[1] += (input->accel_y - state[2] * vx) * dt;
        }
        // 加速度不可用时，按机体系速度随机游走预测。
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    fp[i][j] += f[i][k] * observer->covariance[k][j];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    p[i][j] += fp[i][k] * f[j][k];
        for (int i = 0; i < 3; ++i)
            p[i][i] += observer->config.process_noise[i] * dt;
        if (input->accel_valid)
            for (int i = 0; i < 2; ++i)
                p[i][i] += observer->config.accel_variance[i] * dt * dt;
        if (input->wheel_valid)
        {
            const float wheel[3] = {input->wheel.vx, input->wheel.vy, input->wheel.wz};
            for (int i = 0; i < 3; ++i)
                Correct(state, p, i, wheel[i], observer->config.wheel_variance[i]);
        }
    }
    if (input->gyro_valid)
        Correct(state, p, 2, input->gyro_z, observer->config.gyro_variance);
    for (int i = 0; i < 3; ++i)
        for (int j = i + 1; j < 3; ++j)
            p[j][i] = p[i][j] = 0.5f * p[i][j] + 0.5f * p[j][i];
    result->fused = (Speed_s){state[0], state[1], state[2]};
}

static uint8_t LeverArmUpdate(SpeedObserverLeverArm_s *filter, float ax, float ay, float gyro_z, float offset_x,
                              float offset_y, float dt, float time_constant, float center_accel[2])
{
    // 只计算历史和中心加速度，统一由最终滤波结果检查处理数值异常。
    const float squared_wz = gyro_z * gyro_z;
    if (!filter->seeded)
    {
        // 首帧或恢复帧只建立差分历史，避免启动冲击。
        *filter = (SpeedObserverLeverArm_s){.last_gyro = gyro_z, .squared_wz = squared_wz, .seeded = 1u};
        return 0u;
    }
    // 使用与 INS 加速度匹配的后向欧拉低通及实际采样间隔。
    const float weight = dt / (time_constant + dt);
    // 差分与低通合并，避免生成过大的中间导数。
    const float angular_accel =
        (time_constant * filter->angular_accel + (gyro_z - filter->last_gyro)) / (time_constant + dt);
    const float filtered_squared_wz = filter->squared_wz + weight * (squared_wz - filter->squared_wz);
    // 切向和向心项均使用匹配低通，减少起停时与 INS 加速度的相位差。
    const float center_x = ax + angular_accel * offset_y + filtered_squared_wz * offset_x;
    const float center_y = ay - angular_accel * offset_x + filtered_squared_wz * offset_y;
    filter->last_gyro = gyro_z;
    filter->angular_accel = angular_accel;
    filter->squared_wz = filtered_squared_wz;
    center_accel[0] = center_x;
    center_accel[1] = center_y;
    return 1u;
}

// 只检查输入并维护采样资格，不执行运动学、杆臂补偿或滤波计算。
static uint8_t CheckInput(SpeedObserverInstance *observer, const SpeedObserverInput_s *input, float dt,
                          ObserverInput_s *checked)
{
    if (observer == NULL || input == NULL || !observer->initialized)
        return 0u;
    observer->output.status = input->imu_ready ? kSpeedObserverImuReady : 0u;
    if (!Finite(dt) || dt <= 0.0f)
    {
        observer->lever_arm = (SpeedObserverLeverArm_s){0};
        observer->have_accel = 0u;
        observer->imu_age = 0.0f;
        observer->correction_age = observer->timeout + 1.0f;
        return 0u;
    }
    checked->dt_valid = dt <= observer->config.max_dt;
    observer->correction_age = checked->dt_valid ? observer->correction_age + dt : observer->timeout + 1.0f;
    uint8_t wheels_available = 1u;
    FilterInput_s *sample = &checked->sample;
    sample->wheel_valid = 1u;
    for (int i = 0; i < 4; ++i)
    {
        if (input->wheel_online[i])
            observer->output.status |= (uint16_t)(1u << (8 + i));
        observer->wheel_age[i] = input->wheel_stamp[i] != observer->wheel_seen[i] ? 0.0f : observer->wheel_age[i] + dt;
        observer->wheel_seen[i] = input->wheel_stamp[i];
        if (!input->wheel_online[i] || input->wheel_stamp[i] == 0 ||
            observer->wheel_age[i] > observer->timeout || !Finite(input->motor_speed[i]))
            wheels_available = 0u;
        if (input->wheel_stamp[i] == observer->wheel_stamp[i])
            sample->wheel_valid = 0u;
    }
    sample->wheel_valid &= wheels_available;
    checked->feedback_valid = checked->dt_valid && wheels_available &&
                              (sample->wheel_valid || observer->correction_age <= observer->timeout);
    const uint64_t stamp = input->imu_ready ? input->imu_stamp : 0u;
    if (checked->dt_valid)
        observer->imu_age += dt;
    checked->imu_new = stamp != 0 && stamp != observer->imu_stamp;
    sample->gyro_z = input->gyro_z;
    sample->gyro_valid = checked->dt_valid && checked->imu_new && Finite(input->gyro_z);
    checked->accel_new = sample->gyro_valid && Finite(input->accel_x) && Finite(input->accel_y) &&
                         Finite(input->accel_lpf) && input->accel_lpf >= 0.0f;
    // 超时、重启、暂停或坏样本共用一次历史清空。
    if (!checked->dt_valid || stamp == 0 || stamp < observer->imu_stamp ||
        observer->imu_age > observer->timeout || (checked->imu_new && !checked->accel_new))
    {
        observer->lever_arm = (SpeedObserverLeverArm_s){0};
        observer->have_accel = 0u;
    }
    if (!checked->dt_valid)
    {
        observer->seeded = 0u;
        observer->output.status |= kSpeedObserverTimingReset;
    }
    return 1u;
}

// 仅检查最终速度和协方差对角线；异常时丢弃本周期结果并清空计算历史。
static uint8_t CheckResult(SpeedObserverInstance *observer, const FilterResult_s *result)
{
    const float velocity[3] = {result->fused.vx, result->fused.vy, result->fused.wz};
    for (int i = 0; i < 3; ++i)
        if (!Finite(velocity[i]) || !Finite(result->covariance[i][i]) || result->covariance[i][i] < 0.0f)
        {
            observer->seeded = observer->have_accel = 0u;
            observer->lever_arm = (SpeedObserverLeverArm_s){0};
            return 0u;
        }
    return 1u;
}

static uint8_t UpdateWithDt(SpeedObserverInstance *observer, const SpeedObserverInput_s *input, float dt)
{
    ObserverInput_s checked = {0};
    if (!CheckInput(observer, input, dt, &checked))
        return 0u;

    // 计算阶段只使用输入检查给出的资格，不重复检查传感器或中间结果。
    FilterInput_s *sample = &checked.sample;
    observer->output.wheel = sample->wheel = OmniForward(&observer->model, input->motor_speed);
    if (checked.imu_new)
    {
        if (checked.accel_new)
            observer->have_accel = LeverArmUpdate(&observer->lever_arm, input->accel_x, input->accel_y, input->gyro_z,
                                                  observer->imu_offset[0], observer->imu_offset[1], observer->imu_age,
                                                  input->accel_lpf, observer->center_accel);
        observer->imu_age = 0.0f;
        observer->imu_stamp = input->imu_stamp;
    }
    sample->accel_valid = observer->have_accel;
    sample->accel_x = observer->center_accel[0];
    sample->accel_y = observer->center_accel[1];
    observer->output.status |= (uint16_t)((sample->wheel_valid << 1) | (sample->gyro_valid << 2) | (sample->accel_valid << 3));
    if (!observer->seeded && !sample->wheel_valid)
        return 0u;
    FilterResult_s result;
    FilterCalculate(observer, sample, checked.dt_valid ? dt : 0.0f, &result);
    if (!CheckResult(observer, &result))
        return 0u;

    observer->output.fused = result.fused;
    memcpy(observer->covariance, result.covariance, sizeof(result.covariance));
    observer->seeded = 1u;
    observer->output.status |= kSpeedObserverUpdated | kSpeedObserverSeeded;
    if (sample->wheel_valid)
    {
        memcpy(observer->wheel_stamp, input->wheel_stamp, sizeof(observer->wheel_stamp));
        observer->correction_age = 0.0f;
    }
    if (checked.feedback_valid)
        observer->output.status |= kSpeedObserverValid;
    return 1u;
}

uint8_t SpeedObserverUpdate(SpeedObserverInstance *observer, const SpeedObserverInput_s *input)
{
    if (observer == NULL)
        return 0u;
    return UpdateWithDt(observer, input, DWT_GetDeltaT(&observer->dt));
}

const SpeedObserverResult_s *SpeedObserverGetResult(const SpeedObserverInstance *observer)
{
    return observer == NULL ? NULL : &observer->output;
}

void SpeedObserverInverse(const SpeedObserverInstance *observer, const Speed_s *body, float motor_speed[4])
{
    if (observer == NULL)
    {
        memset(motor_speed, 0, 4 * sizeof(*motor_speed));
        return;
    }
    OmniInverse(&observer->model, body, motor_speed);
}
