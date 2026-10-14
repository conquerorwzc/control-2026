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
    float imu_offset[2];      // 底盘中心指向 IMU，底盘系 m；INS CenterOffset 必须保持 0。
    float sensor_timeout;     // 观测器采样新鲜度限制，s；不修改底盘急停策略。
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

typedef enum {
    kSpeedObserverImuReady = 1u << 0,
    kSpeedObserverWheelUpdated = 1u << 1,
    kSpeedObserverGyroUpdated = 1u << 2,
    kSpeedObserverAccelValid = 1u << 3,
    kSpeedObserverUpdated = 1u << 4,
    kSpeedObserverSeeded = 1u << 5,
    kSpeedObserverTimingReset = 1u << 6,
    kSpeedObserverValid = 1u << 7,
    kSpeedObserverWheelLfOnline = 1u << 8,
    kSpeedObserverWheelRfOnline = 1u << 9,
    kSpeedObserverWheelLbOnline = 1u << 10,
    kSpeedObserverWheelRbOnline = 1u << 11,
} SpeedObserverStatus_e;

// 对外只提供只读结果；所有计算历史均保存在不透明实例中。
typedef struct {
    Speed_s wheel;
    Speed_s fused;
    uint16_t status; // kSpeedObserverUpdated / kSpeedObserverValid 等状态位
} SpeedObserver_s;

typedef struct SpeedObserverInstance SpeedObserverInstance;

SpeedObserverInstance *SpeedObserverInit(const SpeedObserver_Init_Config_s *config);
uint8_t SpeedObserverUpdate(SpeedObserverInstance *observer, const SpeedObserverInput_s *input, float dt);
const SpeedObserver_s *SpeedObserverGetResult(const SpeedObserverInstance *observer);
void SpeedObserverInverse(const SpeedObserverInstance *observer, const Speed_s *body, float motor_speed[4]);
