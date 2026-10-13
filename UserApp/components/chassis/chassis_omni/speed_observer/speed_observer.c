#include "speed_observer.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// 长度为 m，减速比为电机转速 / 轮转速；几何尺寸和减速比应为正。
static OmniKinematics OmniKinematicsInit(float wheel_base, float track_width, float wheel_radius,
                                                float reduction) {
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
static Speed_s OmniForward(const OmniKinematics *model, const float motor_speed[4]) {
    const float scale = 0.25f / model->motor_per_linear;
    Speed_s body;
    body.vx = (-motor_speed[0] + motor_speed[1] - motor_speed[2] + motor_speed[3]) * scale / model->x_projection;
    body.vy = (motor_speed[0] + motor_speed[1] - motor_speed[2] - motor_speed[3]) * scale / model->y_projection;
    body.wz = (motor_speed[0] + motor_speed[1] + motor_speed[2] + motor_speed[3]) * scale / model->radius;
    return body;
}

// 按 IEEE-754 位模式检查，避免 -Ofast 优化掉 NaN/Inf 判断。
static uint8_t Finite(float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

static uint8_t VelocityValid(const Speed_s *v) { return v != NULL && Finite(v->vx) && Finite(v->vy) && Finite(v->wz); }

uint8_t SpeedObserverInit(SpeedObserver_s *observer, const SpeedObserver_Init_Config_s *init) {
    if (observer == NULL || init == NULL) return 0u;
    const SpeedObserverConfig_s *config = &init->filter;
    memset(observer, 0, sizeof(*observer));
    for (int i = 0; i < 3; ++i)
        if (!Finite(config->initial_variance[i]) || config->initial_variance[i] <= 0.0f ||
            !Finite(config->process_noise[i]) || config->process_noise[i] <= 0.0f ||
            !Finite(config->wheel_variance[i]) || config->wheel_variance[i] <= 0.0f)
            return 0u;
    for (int i = 0; i < 2; ++i)
        if (!Finite(config->accel_variance[i]) || config->accel_variance[i] < 0.0f) return 0u;
    if (!Finite(config->gyro_variance) || config->gyro_variance <= 0.0f || !Finite(config->max_dt) ||
        config->max_dt <= 0.0f)
        return 0u;
    if (!Finite(init->wheel_base) || init->wheel_base <= 0.0f || !Finite(init->track_width) ||
        init->track_width <= 0.0f || !Finite(init->wheel_radius) || init->wheel_radius <= 0.0f ||
        !Finite(init->reduction) || init->reduction <= 0.0f || !Finite(init->sensor_timeout) ||
        init->sensor_timeout <= 0.0f || !Finite(init->imu_offset[0]) || !Finite(init->imu_offset[1]))
        return 0u;
    observer->model = OmniKinematicsInit(init->wheel_base, init->track_width, init->wheel_radius, init->reduction);
    observer->use_imu = init->use_imu;
    observer->use_accel = init->use_imu && init->use_accel;
    observer->timeout = init->sensor_timeout;
    observer->correction_age = init->sensor_timeout + 1.0f;
    memcpy(observer->imu_offset, init->imu_offset, sizeof(observer->imu_offset));
    observer->config = *config;
    observer->initialized = 1u;
    return 1u;
}

static uint8_t Reset(SpeedObserver_s *observer, const Speed_s *velocity) {
    if (observer == NULL || !observer->initialized || !VelocityValid(velocity)) return 0u;
    observer->fused = *velocity;
    memset(observer->covariance, 0, sizeof(observer->covariance));
    for (int i = 0; i < 3; ++i) observer->covariance[i][i] = observer->config.initial_variance[i];
    observer->seeded = 1u;
    observer->used_gyro = observer->used_accel = 0u;
    return 1u;
}

typedef struct {
    Speed_s wheel;
    float gyro_z, accel_x, accel_y;
    uint8_t wheel_valid, gyro_valid, accel_valid;
} FilterInput_s;

// 单轴量测校正，Joseph 形式保持协方差对称与半正定。
static void Correct(float state[3], float p[3][3], int axis, float measurement, float variance) {
    float gain[3], a[3][3], ap[3][3] = {{0}}, updated[3][3] = {{0}};
    const float innovation = measurement - state[axis];
    const float s = p[axis][axis] + variance;
    for (int i = 0; i < 3; ++i) {
        gain[i] = p[i][axis] / s;
        state[i] += gain[i] * innovation;
        for (int j = 0; j < 3; ++j) a[i][j] = (i == j ? 1.0f : 0.0f) - (j == axis ? gain[i] : 0.0f);
    }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) ap[i][j] += a[i][k] * p[k][j];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) updated[i][j] += ap[i][k] * a[j][k];
            updated[i][j] += gain[i] * variance * gain[j];
        }
    memcpy(p, updated, sizeof(updated));
}

static uint8_t FilterUpdate(SpeedObserver_s *observer, const FilterInput_s *input, float dt) {
    if (observer == NULL || !observer->initialized || input == NULL || !Finite(dt) || dt <= 0.0f ||
        dt > observer->config.max_dt || (input->wheel_valid && !VelocityValid(&input->wheel)) ||
        (input->gyro_valid && !Finite(input->gyro_z)) ||
        (input->accel_valid && (!Finite(input->accel_x) || !Finite(input->accel_y))))
        return 0u;
    if (!observer->seeded) {
        if (!input->wheel_valid) return 0u;
        // 首帧轮速只用于初始化，不重复校正。
        if (!Reset(observer, &input->wheel)) return 0u;
        if (input->gyro_valid) {
            float state[3] = {observer->fused.vx, observer->fused.vy, observer->fused.wz};
            Correct(state, observer->covariance, 2, input->gyro_z, observer->config.gyro_variance);
            observer->fused.wz = state[2];
            observer->used_gyro = 1u;
        }
        return 1u;
    }
    float state[3] = {observer->fused.vx, observer->fused.vy, observer->fused.wz};
    float f[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float fp[3][3] = {{0}}, p[3][3] = {{0}};
    if (input->accel_valid) {
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
            for (int k = 0; k < 3; ++k) fp[i][j] += f[i][k] * observer->covariance[k][j];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) p[i][j] += fp[i][k] * f[j][k];
    for (int i = 0; i < 3; ++i) p[i][i] += observer->config.process_noise[i] * dt;
    if (input->accel_valid)
        for (int i = 0; i < 2; ++i) p[i][i] += observer->config.accel_variance[i] * dt * dt;
    if (input->wheel_valid) {
        const float wheel[3] = {input->wheel.vx, input->wheel.vy, input->wheel.wz};
        for (int i = 0; i < 3; ++i) Correct(state, p, i, wheel[i], observer->config.wheel_variance[i]);
    }
    if (input->gyro_valid) Correct(state, p, 2, input->gyro_z, observer->config.gyro_variance);
    // 仅提交有限结果，失败更新不覆盖已有融合状态。
    for (int i = 0; i < 3; ++i) {
        if (!Finite(state[i]) || !Finite(p[i][i]) || p[i][i] < 0.0f) return 0u;
        for (int j = i; j < 3; ++j) {
            if (!Finite(p[i][j]) || !Finite(p[j][i])) return 0u;
            p[j][i] = p[i][j] = 0.5f * (p[i][j] + p[j][i]);
        }
    }
    observer->fused = (Speed_s){state[0], state[1], state[2]};
    memcpy(observer->covariance, p, sizeof(p));
    observer->used_gyro = input->gyro_valid;
    observer->used_accel = input->accel_valid;
    return 1u;
}

static uint8_t LeverArmUpdate(SpeedObserverLeverArm_s *filter, float ax, float ay, float gyro_z, float offset_x,
                              float offset_y, float dt, float time_constant, float center_accel[2]) {
    if (filter == NULL) return 0u;
    if (center_accel == NULL || !Finite(ax) || !Finite(ay) || !Finite(gyro_z) || !Finite(offset_x) ||
        !Finite(offset_y) || !Finite(dt) || dt <= 0.0f || !Finite(time_constant) || time_constant < 0.0f) {
        *filter = (SpeedObserverLeverArm_s){0};
        return 0u;
    }
    const float squared_wz = gyro_z * gyro_z;
    if (!Finite(squared_wz)) {
        *filter = (SpeedObserverLeverArm_s){0};
        return 0u;
    }
    if (!filter->seeded) {
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
    if (!Finite(angular_accel) || !Finite(filtered_squared_wz) || !Finite(center_x) || !Finite(center_y)) {
        *filter = (SpeedObserverLeverArm_s){0};
        return 0u;
    }
    filter->last_gyro = gyro_z;
    filter->angular_accel = angular_accel;
    filter->squared_wz = filtered_squared_wz;
    center_accel[0] = center_x;
    center_accel[1] = center_y;
    return 1u;
}

uint8_t SpeedObserverUpdate(SpeedObserver_s *observer, const SpeedObserverInput_s *input, float dt) {
    if (observer == NULL || !observer->initialized || input == NULL) return 0u;
    const uint8_t dt_valid = Finite(dt) && dt > 0.0f && dt <= observer->config.max_dt;
    if (dt_valid)
        observer->correction_age += dt;
    else
        observer->correction_age = observer->timeout + 1.0f;
    observer->valid = 0u;
    observer->status = input->imu_ready ? 1u : 0u;
    const uint32_t *wheel_stamp = input->wheel_stamp;
    const uint8_t *wheel_online = input->wheel_online;
    const uint64_t imu_stamp = observer->use_imu && input->imu_ready ? input->imu_stamp : 0u;
    const float gyro_z = input->gyro_z, accel_x = input->accel_x, accel_y = input->accel_y;
    const float accel_lpf = input->accel_lpf;
    observer->wheel = OmniForward(&observer->model, input->motor_speed);
    FilterInput_s sample = {.wheel = observer->wheel, .wheel_valid = 1u};
    for (int i = 0; i < 4; ++i) {
        if (wheel_online[i]) observer->status |= (uint16_t)(1u << (8 + i));
        observer->wheel_age[i] += dt;
        if (wheel_stamp[i] != observer->wheel_seen[i]) observer->wheel_age[i] = 0.0f;
        observer->wheel_seen[i] = wheel_stamp[i];
        if (!wheel_online[i] || wheel_stamp[i] == 0 || wheel_stamp[i] == observer->wheel_stamp[i] ||
            observer->wheel_age[i] > observer->timeout)
            sample.wheel_valid = 0u;
    }
    // 暂停、时钟异常或 INS 重启时清空差分历史，避免角加速度冲击。
    if (dt_valid) {
        observer->imu_age += dt;
        observer->gyro_dt += dt;
    }
    if (!dt_valid || imu_stamp == 0 || imu_stamp < observer->imu_stamp || observer->imu_age > observer->timeout ||
        observer->gyro_dt > observer->timeout) {
        observer->lever_arm = (SpeedObserverLeverArm_s){0};
        observer->have_accel = 0u;
    }
    if (imu_stamp != 0 && imu_stamp != observer->imu_stamp) {
        sample.gyro_z = gyro_z;
        sample.gyro_valid = Finite(gyro_z);
        observer->have_accel = 0u;
        if (observer->use_accel && dt_valid) {
            observer->have_accel =
                LeverArmUpdate(&observer->lever_arm, accel_x, accel_y, gyro_z, observer->imu_offset[0],
                               observer->imu_offset[1], observer->gyro_dt, accel_lpf, observer->center_accel);
        }
        observer->gyro_dt = observer->imu_age = 0.0f;
        observer->imu_stamp = imu_stamp;
    }
    sample.accel_valid = observer->have_accel && observer->imu_age <= observer->timeout;
    sample.accel_x = observer->center_accel[0];
    sample.accel_y = observer->center_accel[1];
    if (sample.wheel_valid) observer->status |= (1u << 1);
    if (sample.gyro_valid) observer->status |= (1u << 2);
    if (sample.accel_valid) observer->status |= (1u << 3);
    observer->updated = 0u;
    if (dt > observer->config.max_dt) {
        observer->status |= (1u << 6);
        // 调试暂停等长间隔不强行积分；仅用新鲜轮速重置估计，不改任何电机指令。
        if (sample.wheel_valid)
            observer->updated = Reset(observer, &sample.wheel);
        else
            observer->seeded = 0u;
        observer->lever_arm = (SpeedObserverLeverArm_s){0};
        observer->have_accel = 0u;
    } else
        observer->updated = FilterUpdate(observer, &sample, dt);
    if (observer->updated) {
        observer->status |= (1u << 4);

        if (sample.wheel_valid) {
            memcpy(observer->wheel_stamp, wheel_stamp, sizeof(observer->wheel_stamp));
            observer->correction_age = 0.0f;
        }
    }
    if (observer->seeded) observer->status |= (1u << 5);
    observer->valid = dt_valid && observer->updated && observer->seeded &&
                      observer->correction_age <= observer->timeout && Finite(observer->fused.vx) &&
                      Finite(observer->fused.vy) && Finite(observer->fused.wz);
    for (int i = 0; i < 4; ++i)
        if (!wheel_online[i] || observer->wheel_age[i] > observer->timeout) observer->valid = 0u;
    return observer->updated;
}
