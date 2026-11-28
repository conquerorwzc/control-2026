#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "dji_motor.h"
#include "dmmotor.h"

enum
{
    CHASSIS_LEFT = 0,
    CHASSIS_RIGHT,
    CHASSIS_LEG_COUNT,
    CHASSIS_FRONT = 0,
    CHASSIS_REAR,
    CHASSIS_LEFT_FRONT = 0,
    CHASSIS_LEFT_REAR,
    CHASSIS_RIGHT_FRONT,
    CHASSIS_RIGHT_REAR,
    CHASSIS_JOINT_COUNT
};

typedef struct
{
    float first_m;
    float second_m;
    float virtual_second_m;
    float singular_epsilon_m;
} ChassisGeometryConfig;

typedef struct
{
    Motor_Init_Config_s motor_config;
    float zero_rad;
    float direction;
    float center_rad;
    float kp_nm_per_rad, kd_nm_per_rad_s;
    float max_torque_nm;
} HipMotor_Init_Config_s;

typedef struct
{
    Motor_Init_Config_s motor_config;
    float direction;
    float reduction;
    float kp_a_per_rad_s;
    float max_speed_rad_s;
    float max_current_a;
} WheelMotor_Init_Config_s;

typedef struct
{
    unsigned joint_mask; // 左前、左后、右前、右后。
    unsigned wheel_mask; // 左轮、右轮。
    uint32_t feedback_timeout_ms;
    HipMotor_Init_Config_s hip[CHASSIS_JOINT_COUNT];
    WheelMotor_Init_Config_s wheel[CHASSIS_LEG_COUNT];
    ChassisGeometryConfig geometry;
} Chassis_Init_Config_s;

typedef struct
{
    bool enabled;
    bool wheel_current_mode;
    unsigned hip_torque_mask; // 选中的髋轴使用力矩参考，其余使用位置参考。
    float hip_position_rad[CHASSIS_JOINT_COUNT];
    float hip_torque_nm[CHASSIS_JOINT_COUNT]; // 力矩模式的指令，或位置模式的前馈力矩。
    float wheel_reference[CHASSIS_LEG_COUNT]; // 轮速 rad/s 或直接电流 A。
} ChassisRequest;

typedef struct
{
    bool valid;
    bool updated;
    bool seen;
    uint32_t feed_counter;
    uint64_t feedback_time_us;
    float q_rad;
    float qd_rad_s;
    float tau_nm;
    uint16_t torque_raw; // unmodified 12-bit MIT feedback field
    float temperature_c;
    float command_torque_preclip_nm;
    float command_torque_nm;
} ChassisJointFeedback;

typedef struct
{
    bool valid;
    bool updated;
    bool seen;
    uint32_t feed_counter;
    uint64_t feedback_time_us;
    float speed_rad_s;
    float current_raw; // C620 torque-current feedback counts in the logical wheel direction
    float temperature_c;
    float command_current_preclip_a;
    float command_current_a;
} ChassisWheelFeedback;

typedef struct
{
    DMMotorInstance *front_motor;
    DMMotorInstance *rear_motor;
    DJIMotorInstance *wheel_motor;
    ChassisJointFeedback joint[2];
    ChassisWheelFeedback wheel_feedback;
    bool geometry_valid;
    float hip_delta_rad; // 前髋减后髋，按标定姿态对齐装配分支。
    float length_m;
    float angle_rad;
} ChassisLeg;

typedef struct
{
    Chassis_Init_Config_s *config;
    bool enabled;
    ChassisLeg legs[CHASSIS_LEG_COUNT];
    uint64_t command_time_us;
    bool clipped;
} ChassisInstance;

// Config must outlive the chassis. Motor init fills callbacks in its motor entries.
ChassisInstance *ChassisInit(Chassis_Init_Config_s *config);
// 机器人先读本拍反馈，再给底盘参考指令。
void ChassisFeedbackUpdate(ChassisInstance *chassis);
void ChassisTask(ChassisInstance *chassis, const ChassisRequest *request);
