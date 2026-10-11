/**
 ******************************************************************************
 * @file    speed_observer.h
 * @brief   Wheel-legged chassis longitudinal position/velocity observer.
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

#include "kalman_filter.h"

typedef struct
{
    float initial_variance[3]; /* 初始 P 对角线；初始化和 Reset 时应用。 */
    float jerk_process_noise;
    float position_measurement_noise;
    float velocity_measurement_noise;
    float acceleration_measurement_noise;
} SpeedObserverConfig_s;

typedef struct
{
    KalmanFilter_t filter;
    SpeedObserverConfig_s config;
    float position;                 /* Fused longitudinal position, m. */
    float velocity;                 /* Fused longitudinal velocity, m/s. */
    float acceleration;             /* Fused longitudinal acceleration, m/s^2. */
    float position_measurement;     /* Wheel/leg geometric odometry position, m. */
    float velocity_measurement;     /* Wheel/leg geometric odometry velocity, m/s. */
    float acceleration_measurement; /* Gravity-compensated body-frame acceleration, m/s^2. */
    uint8_t initialized;
    uint8_t seeded;
} SpeedObserver_s;

uint8_t SpeedObserverInit(SpeedObserver_s *observer, const SpeedObserverConfig_s *config);
uint8_t SpeedObserverUpdate(SpeedObserver_s *observer, float position, float velocity, float acceleration, float dt);
void SpeedObserverReset(SpeedObserver_s *observer, float position, float velocity, float acceleration);
