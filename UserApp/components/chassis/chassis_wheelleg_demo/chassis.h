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
    CHASSIS_JOINT_COUNT = 4
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
    unsigned joint_mask; // order LF, LR, RF, RR; each configured leg requires both joints
    unsigned wheel_mask; // order left, right; wheels can be registered independently
    uint32_t feedback_timeout_ms;
    Motor_Init_Config_s joint[CHASSIS_JOINT_COUNT];
    Motor_Init_Config_s wheel[CHASSIS_LEG_COUNT];
    uint8_t joint_feedback_id[CHASSIS_JOINT_COUNT]; // expected DM payload ID (low nibble)
    float joint_zero_rad[CHASSIS_JOINT_COUNT];
    float joint_direction[CHASSIS_JOINT_COUNT]; // +1 or -1 relative to driver feedback/command
    float motor_per_joint[CHASSIS_JOINT_COUNT]; // motor rad per joint rad
    float joint_kp_nm_per_rad[CHASSIS_JOINT_COUNT];
    float joint_kd_nm_per_rad_s[CHASSIS_JOINT_COUNT];
    float joint_max_torque_nm[CHASSIS_JOINT_COUNT];
    float wheel_direction[CHASSIS_LEG_COUNT]; // +1 or -1 relative to raw C620 frame
    float motor_per_wheel[CHASSIS_LEG_COUNT];
    float wheel_kp_a_per_rad_s[CHASSIS_LEG_COUNT];
    float wheel_max_current_a[CHASSIS_LEG_COUNT];
    ChassisGeometryConfig geometry;
} Chassis_Init_Config_s;

typedef struct
{
    unsigned joint_enable_mask;
    unsigned wheel_enable_mask;
    unsigned joint_direct_torque_mask; // enabled joints using a matched holding torque instead of PD
    unsigned wheel_current_mode_mask;  // enabled wheels using direct current, including zero-current coast
    float q_ref_rad[CHASSIS_JOINT_COUNT];
    float joint_torque_ref_nm[CHASSIS_JOINT_COUNT];
    float joint_torque_offset_nm[CHASSIS_JOINT_COUNT]; // added to the PD or direct torque before clipping
    float wheel_speed_ref_rad_s[CHASSIS_LEG_COUNT];
    float wheel_current_ref_a[CHASSIS_LEG_COUNT];
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
    float length_m;
    float angle_rad;
} ChassisLeg;

typedef struct
{
    Chassis_Init_Config_s *config;
    unsigned joint_enabled_mask;
    unsigned wheel_enabled_mask;
    ChassisLeg legs[CHASSIS_LEG_COUNT];
    uint64_t command_time_us;
    bool fault;
    bool clipped;
} ChassisInstance;

// Config must outlive the chassis. Motor init fills callbacks in its motor entries.
ChassisInstance *ChassisInit(Chassis_Init_Config_s *config);
// 机器人先读本拍反馈，再给底盘参考指令。
void ChassisFeedbackUpdate(ChassisInstance *chassis);
void ChassisTask(ChassisInstance *chassis, const ChassisRequest *request);
