/**
 ******************************************************************************
 * @file    chassis_private.h
 * @brief   双闭环轮腿底盘 component 的文件内部接口
 ******************************************************************************
 */
#pragma once

#include "chassis.h"

void WheelLeggedLegInit(WheelLeggedLegInstance_t *leg, WheelLeggedLegInitConfig_t *config);
void WheelLeggedLegUpdate(WheelLeggedLegInstance_t *leg);
void WheelLeggedChassisApplyJointMotorState(WheelLeggedChassisInstance_t *chassis);
