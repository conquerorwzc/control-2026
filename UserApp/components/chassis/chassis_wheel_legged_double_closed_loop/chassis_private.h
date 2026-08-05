/**
 ******************************************************************************
 * @file    chassis_private.h
 * @brief   双闭环轮腿底盘 component 的文件内部接口
 ******************************************************************************
 */
#pragma once

#include "chassis.h"
#include "wheel_odometry.h"

void WheelLeggedLegInit(WheelLeggedLegInstance_t *leg, WheelLeggedLegInitConfig_t *config);
void WheelLeggedLegUpdate(WheelLeggedLegInstance_t *leg);
void WheelLeggedChassisStateUpdate(WheelLeggedChassisInstance_t *chassis);
void WheelLeggedChassisLqrUpdate(WheelLeggedChassisLqr_t *lqr,
                                 const float state_vector[WHEEL_LEGGED_LQR_STATE_COUNT], uint8_t state_valid,
                                 uint8_t origin_captured, float left_leg_length, float right_leg_length);
void WheelLeggedLegVmcUpdate(WheelLeggedLegInstance_t *leg);
void WheelLeggedChassisApplyMotorOutput(WheelLeggedChassisInstance_t *chassis);
