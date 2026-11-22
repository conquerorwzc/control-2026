#pragma once

#include <stdint.h>

// 双板 CAN 通信协议,云台板 <-> 底盘板,各 24 字节

#pragma pack(1)

typedef struct {
    int16_t vx_stick;
    int16_t vy_stick;
    int16_t wz_stick;
    int16_t dial;
    uint16_t gimbal_yaw_angle;  // 0-360 度
    uint8_t switch_right;
    uint8_t super_cap_boost;
    uint8_t reserved[12];
} GimbalToChassis_s;

typedef struct {
    uint16_t chassis_power_limit;
    uint16_t buffer_energy;
    uint16_t shooter_heat;
    uint16_t heat_limit;
    uint16_t super_cap_voltage_x100;  // cap_v * 100
    uint8_t robot_id;
    uint8_t super_cap_mode;
    uint8_t reserved[12];
} ChassisToGimbal_s;

#pragma pack()

_Static_assert(sizeof(GimbalToChassis_s) == 24, "GimbalToChassis_s size");
_Static_assert(sizeof(ChassisToGimbal_s) == 24, "ChassisToGimbal_s size");
