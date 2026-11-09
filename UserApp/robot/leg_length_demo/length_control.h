#pragma once
#include "controller.h"
#include <stdbool.h>

typedef struct
{
    float first_m, second_m, virtual_second_m, epsilon;
    float minimum_m, maximum_m;
    float damping, torque_limit_nm;
    PID_Init_Config_s pid;
} LegLengthConfig;

typedef struct
{
    float length_m, speed_m_s;
    float jacobian[2]; // d(length)/d(front,rear angle), m/rad
    float force_n, torque_nm[2];
    PIDInstance pid;
} LegLengthState;

bool LegLengthGeometry(const LegLengthConfig *config, const float q[2], const float qd[2], LegLengthState *state);
bool LegLengthStep(const LegLengthConfig *config, float target_m, LegLengthState *state);
float LegLengthTarget(float dial, float full_scale, const LegLengthConfig *config);
bool LegLengthRequested(bool online, unsigned left_switch, unsigned right_switch);
