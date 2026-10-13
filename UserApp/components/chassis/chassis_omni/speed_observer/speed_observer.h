#pragma once

#include <stdint.h>

// 底盘机体系速度：+x 向前、+y 向左、+wz 俯视逆时针。
typedef struct {
    float vx;  // m/s
    float vy;  // m/s
    float wz;  // rad/s
} Speed_s;

typedef struct {
    float initial_variance[3];
    float process_noise[3];   // 连续随机游走噪声密度，Q = q * dt。
    float wheel_variance[3];  // 轮速正解量测方差。
    float gyro_variance;      // (rad/s)^2
    float accel_variance[2];  // (m/s^2)^2；输入噪声按方差乘 dt^2 累加。
    float max_dt;             // 最大预测间隔，超限由更新函数重置。
} SpeedObserverConfig_s;

typedef struct {
    SpeedObserverConfig_s filter;
    float wheel_base, track_width, wheel_radius;  // m
    float reduction;
    float imu_offset[2];   // 底盘中心指向 IMU，m
    float sensor_timeout;  // s
    uint8_t use_imu, use_accel;
} SpeedObserver_Init_Config_s;

// 一次采样快照；计数用于区分新量测，禁止把同一样本重复融合。
typedef struct {
    float motor_speed[4];  // 电机轴 deg/s，左前/右前/左后/右后
    uint32_t wheel_stamp[4];
    uint8_t wheel_online[4];
    uint64_t imu_stamp;
    float gyro_z;            // rad/s
    float accel_x, accel_y;  // IMU 点去重力加速度，m/s^2
    float accel_lpf;         // INS 加速度低通时间常数，s
    uint8_t imu_ready;
} SpeedObserverInput_s;

// 以下类型仅存放算法所需状态，不是额外的速度输出。
typedef struct {
    float radius, x_projection, y_projection, motor_per_linear;
} OmniKinematics;

typedef struct {
    float last_gyro, angular_accel, squared_wz;
    uint8_t seeded;
} SpeedObserverLeverArm_s;

typedef struct {
    Speed_s wheel;    // 四轮运动学正解
    Speed_s fused;    // 卡尔曼融合结果
    uint8_t updated;  // 本轮融合/重置成功
    uint8_t valid;    // 本轮融合成功，且轮速反馈未超时
    uint16_t status;  // 保留原采集有效性位定义

    // 内部状态由 Init/Update 维护，调用方只读取上述结果。
    OmniKinematics model;
    SpeedObserverConfig_s config;
    float covariance[3][3];
    uint8_t initialized, seeded, used_gyro, used_accel;
    uint8_t use_imu, use_accel, have_accel;
    float imu_offset[2], timeout;
    SpeedObserverLeverArm_s lever_arm;
    float center_accel[2];
    uint64_t imu_stamp;
    uint32_t wheel_stamp[4], wheel_seen[4];
    float wheel_age[4], imu_age, gyro_dt, correction_age;
} SpeedObserver_s;

uint8_t SpeedObserverInit(SpeedObserver_s *observer, const SpeedObserver_Init_Config_s *config);
// 周期内只调用此函数，完成轮速正解、样本检查、杆臂补偿和卡尔曼更新。
uint8_t SpeedObserverUpdate(SpeedObserver_s *observer, const SpeedObserverInput_s *input, float dt);

// 输出使用电机驱动完成方向校正后的速度约定。
static inline void OmniInverse(const OmniKinematics *model, const Speed_s *body, float motor_speed[4]) {
    const float x = model->x_projection * body->vx;
    const float y = model->y_projection * body->vy;
    const float spin = model->radius * body->wz;
    motor_speed[0] = (-x + y + spin) * model->motor_per_linear;
    motor_speed[1] = (x + y + spin) * model->motor_per_linear;
    motor_speed[2] = (-x - y + spin) * model->motor_per_linear;
    motor_speed[3] = (x - y + spin) * model->motor_per_linear;
}

