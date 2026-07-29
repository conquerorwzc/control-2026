/**
 ******************************************************************************
 * @file    joint_transmission.h
 * @brief   达妙反馈角到机构主动轴角的纯传动换算
 ******************************************************************************
 */
#pragma once

#include <stdbool.h>

typedef enum {
  JOINT_TRANSMISSION_OK = 0,          /* 传动换算成功。 */
  JOINT_TRANSMISSION_NOT_CONFIGURED,  /* 链传动参数尚未标定。 */
  JOINT_TRANSMISSION_INVALID_ARGUMENT,/* 配置、输入或输出指针无效。 */
  JOINT_TRANSMISSION_NUMERIC_ERROR,   /* 参数或计算结果为零、NaN 或 Inf。 */
} JointTransmissionStatus_e;

typedef struct {
  bool configured;  /* 传动参数是否已经完成标定。 */
  float gain;       /* 电机角到主动轴角的绝对传动比。 */
  float direction;  /* 传动方向，取 +1 或 -1。 */
  float zero_offset;/* 电机零位对应的主动轴建模角，单位 rad。 */
} JointTransmissionConfig_t;

/* 按 joint_angle = zero_offset + direction * gain * motor_angle 换算主动轴角。 */
JointTransmissionStatus_e JointTransmissionMotorToJoint(const JointTransmissionConfig_t *config, float motor_angle,
                                                        float *joint_angle);
/* 按 motor_angle=(joint_angle-zero_offset)/(direction*gain) 反算电机累计角。 */
JointTransmissionStatus_e JointTransmissionJointToMotor(const JointTransmissionConfig_t *config, float joint_angle,
                                                        float *motor_angle);
