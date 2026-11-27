#pragma once

#include "chassis.h"
#include "remote_control.h"

#define SYSID_CONTROL_HZ 500u

typedef enum
{
    SYSID_STATE_NEED_ESTOP,
    SYSID_STATE_ESTOP,
    SYSID_STATE_READY,
    SYSID_STATE_RUNNING = 4,
    SYSID_STATE_FINISHED,
    SYSID_STATE_FAULT,
    SYSID_STATE_DRY_RUNNING
} SysIdState;

typedef enum
{
    SYSID_MODE_NONE,
    SYSID_MODE_HIP_IN_PHASE,
    SYSID_MODE_HIP_ANTI_PHASE,
    SYSID_MODE_HIP_STEP,
    SYSID_MODE_WHEEL_STEP_10,
    SYSID_MODE_WHEEL_STEP_30,
    SYSID_MODE_WHEEL_STEP_60,
    SYSID_MODE_WHEEL_SWEEP,
    SYSID_MODE_HIP_RAMP,
    SYSID_MODE_HIP_TRIANGLE_02,
    SYSID_MODE_HIP_TRIANGLE_05,
    SYSID_MODE_HIP_TRIANGLE_10,
    SYSID_MODE_WHEEL_CURRENT_RAMP,
    SYSID_MODE_LEG_LENGTH
} SysIdMode;

typedef enum
{
    SYSID_HIP,
    SYSID_WHEEL_STEP,
    SYSID_WHEEL_SWEEP,
    SYSID_HIP_STEP,
    SYSID_HIP_RAMP,
    SYSID_HIP_TRIANGLE,
    SYSID_WHEEL_CURRENT_RAMP,
    SYSID_WHEEL_STEADY,
    SYSID_WHEEL_COAST
} SysIdTrial;

#define SYSID_IS_HIP(trial)                                                                                            \
    ((trial) == SYSID_HIP || (trial) == SYSID_HIP_STEP || (trial) == SYSID_HIP_RAMP || (trial) == SYSID_HIP_TRIANGLE)

typedef enum
{
    SYSID_PHASE_NONE,
    SYSID_PHASE_BASELINE,
    SYSID_PHASE_EXCITE,
    SYSID_PHASE_RECOVERY,
    SYSID_PHASE_MOTION_DETECTED
} SysIdPhase;

typedef enum
{
    SYSID_HIP_FRONT,
    SYSID_HIP_REAR,
    SYSID_HIP_BOTH_IN_PHASE,
    SYSID_HIP_BOTH_ANTI_PHASE
} SysIdHipPattern;

typedef struct
{
    ChassisInstance *chassis;
    RC_ctrl_t *rc_data;
    SysIdState state;
    SysIdMode mode;
    SysIdPhase phase;
    float length_target_m;
} RobotInstance;

void RobotInit(void);
void RobotTask(void);
RobotInstance *RobotGet(void);
