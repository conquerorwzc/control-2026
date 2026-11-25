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
    Motor_Init_Config_s motor;
    uint8_t feedback_id; // expected DM payload ID (low nibble)
    float zero_rad;
    float direction;       // +1 or -1, relative to the driver feedback/command
    float motor_per_joint; // motor rad per logical joint rad
    float kp_nm_per_rad;
    float kd_nm_per_rad_s;
    float max_torque_nm;
} ChassisJointConfig;

typedef struct
{
    Motor_Init_Config_s motor;
    float direction; // +1 or -1, relative to the driver feedback/command
    float motor_per_wheel;
    float kp_a_per_rad_s;
    float max_current_a;
} ChassisWheelConfig;

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
    ChassisJointConfig joint[CHASSIS_JOINT_COUNT];
    ChassisWheelConfig wheel[CHASSIS_LEG_COUNT];
    ChassisGeometryConfig geometry;
#if defined(STM32H723xx)
    unsigned reserved_std_filters[3]; // already registered on FDCAN1, 2, 3
#endif
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
    bool geometry_updated;
    float length_m;
    float angle_rad;
} ChassisLeg;

typedef struct
{
    Chassis_Init_Config_s config;
    ChassisRequest request;
    ChassisRequest applied_request;
    ChassisLeg legs[CHASSIS_LEG_COUNT];
    uint64_t command_time_us;
    bool fault;
    bool clipped;
} ChassisInstance;

// Returns NULL on invalid configuration or insufficient CAN receive filters, before registering motors.
ChassisInstance *ChassisInit(const Chassis_Init_Config_s *config);
// Call first each control tick, then set request and apply it with ChassisTask.
void ChassisFeedbackUpdate(ChassisInstance *chassis);
void ChassisTask(ChassisInstance *chassis);
