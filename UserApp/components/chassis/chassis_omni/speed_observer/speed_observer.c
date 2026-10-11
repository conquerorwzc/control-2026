/**
 ******************************************************************************
 * @file    speed_observer.c
 * @brief   Fuses wheel/leg odometry position and velocity with IMU acceleration.
 ******************************************************************************
 */
#include "speed_observer.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static uint8_t ConfigValid(const SpeedObserverConfig_s *config);
static uint8_t MemoryValid(const KalmanFilter_t *filter);
static void SetCovariance(SpeedObserver_s *observer);
static void SetProcessNoise(SpeedObserver_s *observer, float dt);

uint8_t SpeedObserverInit(SpeedObserver_s *observer, const SpeedObserverConfig_s *config)
{
    if (observer == NULL || ConfigValid(config) == 0u)
    {
        return 0u;
    }

    memset(observer, 0, sizeof(*observer));
    observer->config = *config;
    Kalman_Filter_Init(&observer->filter, 3u, 0u, 3u);
    if (MemoryValid(&observer->filter) == 0u)
    {
        return 0u;
    }

    observer->filter.UseAutoAdjustment = 0u;
    observer->filter.H_data[0] = 1.0f;
    observer->filter.H_data[1] = 0.0f;
    observer->filter.H_data[2] = 0.0f;
    observer->filter.H_data[3] = 0.0f;
    observer->filter.H_data[4] = 1.0f;
    observer->filter.H_data[5] = 0.0f;
    observer->filter.H_data[6] = 0.0f;
    observer->filter.H_data[7] = 0.0f;
    observer->filter.H_data[8] = 1.0f;
    SetCovariance(observer);
    observer->initialized = 1u;
    return 1u;
}

uint8_t SpeedObserverUpdate(SpeedObserver_s *observer, float position, float velocity, float acceleration, float dt)
{
    if (observer == NULL || observer->initialized == 0u || !isfinite(position) || !isfinite(velocity) ||
        !isfinite(acceleration) || !isfinite(dt) || dt < 0.0f)
    {
        return 0u;
    }

    observer->position_measurement = position;
    observer->velocity_measurement = velocity;
    observer->acceleration_measurement = acceleration;
    if (observer->seeded == 0u)
    {
        SpeedObserverReset(observer, position, velocity, acceleration);
        return 1u;
    }
    if (dt == 0.0f)
    {
        return 1u;
    }

    /* State is [position, velocity, acceleration], with constant acceleration over one control period. */
    const float half_dt_squared = 0.5f * dt * dt;
    observer->filter.F_data[0] = 1.0f;
    observer->filter.F_data[1] = dt;
    observer->filter.F_data[2] = half_dt_squared;
    observer->filter.F_data[3] = 0.0f;
    observer->filter.F_data[4] = 1.0f;
    observer->filter.F_data[5] = dt;
    observer->filter.F_data[6] = 0.0f;
    observer->filter.F_data[7] = 0.0f;
    observer->filter.F_data[8] = 1.0f;
    SetProcessNoise(observer, dt);
    observer->filter.MeasuredVector[0] = position;
    observer->filter.MeasuredVector[1] = velocity;
    observer->filter.MeasuredVector[2] = acceleration;
    Kalman_Filter_Update(&observer->filter);

    const float filtered_position = observer->filter.FilteredValue[0];
    const float filtered_velocity = observer->filter.FilteredValue[1];
    const float filtered_acceleration = observer->filter.FilteredValue[2];
    if (!isfinite(filtered_position) || !isfinite(filtered_velocity) || !isfinite(filtered_acceleration))
    {
        return 0u;
    }
    observer->position = filtered_position;
    observer->velocity = filtered_velocity;
    observer->acceleration = filtered_acceleration;
    return 1u;
}

void SpeedObserverReset(SpeedObserver_s *observer, float position, float velocity, float acceleration)
{
    if (observer == NULL || observer->initialized == 0u || !isfinite(position) || !isfinite(velocity) ||
        !isfinite(acceleration))
    {
        return;
    }

    SetCovariance(observer);
    observer->filter.xhat_data[0] = position;
    observer->filter.xhat_data[1] = velocity;
    observer->filter.xhat_data[2] = acceleration;
    observer->filter.FilteredValue[0] = position;
    observer->filter.FilteredValue[1] = velocity;
    observer->filter.FilteredValue[2] = acceleration;
    observer->position_measurement = position;
    observer->velocity_measurement = velocity;
    observer->acceleration_measurement = acceleration;
    observer->position = position;
    observer->velocity = velocity;
    observer->acceleration = acceleration;
    observer->seeded = 1u;
}

static uint8_t ConfigValid(const SpeedObserverConfig_s *config)
{
    return (uint8_t)(config != NULL && isfinite(config->initial_variance[0]) && config->initial_variance[0] > 0.0f &&
                     isfinite(config->initial_variance[1]) && config->initial_variance[1] > 0.0f &&
                     isfinite(config->initial_variance[2]) && config->initial_variance[2] > 0.0f &&
                     isfinite(config->jerk_process_noise) && config->jerk_process_noise > 0.0f &&
                     isfinite(config->position_measurement_noise) && config->position_measurement_noise > 0.0f &&
                     isfinite(config->velocity_measurement_noise) && config->velocity_measurement_noise > 0.0f &&
                     isfinite(config->acceleration_measurement_noise) && config->acceleration_measurement_noise > 0.0f);
}

static uint8_t MemoryValid(const KalmanFilter_t *filter)
{
    return (uint8_t)(filter != NULL && filter->FilteredValue != NULL && filter->MeasuredVector != NULL &&
                     filter->xhat_data != NULL && filter->xhatminus_data != NULL && filter->P_data != NULL &&
                     filter->Pminus_data != NULL && filter->F_data != NULL && filter->FT_data != NULL &&
                     filter->H_data != NULL && filter->HT_data != NULL && filter->Q_data != NULL &&
                     filter->R_data != NULL && filter->K_data != NULL && filter->S_data != NULL &&
                     filter->temp_matrix_data != NULL && filter->temp_matrix_data1 != NULL &&
                     filter->temp_vector_data != NULL && filter->temp_vector_data1 != NULL);
}

static void SetCovariance(SpeedObserver_s *observer)
{
    const float identity[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    memcpy(observer->filter.P_data, identity, sizeof(identity));
    for (uint32_t index = 0u; index < 3u; index++)
    {
        observer->filter.P_data[index * 4u] = observer->config.initial_variance[index];
    }
    memset(observer->filter.Q_data, 0, 9u * sizeof(float));
    memset(observer->filter.R_data, 0, 9u * sizeof(float));
    observer->filter.R_data[0] = observer->config.position_measurement_noise;
    observer->filter.R_data[4] = observer->config.velocity_measurement_noise;
    observer->filter.R_data[8] = observer->config.acceleration_measurement_noise;
}

static void SetProcessNoise(SpeedObserver_s *observer, float dt)
{
    /* Discrete covariance of a constant-acceleration model driven by white jerk. */
    const float q = observer->config.jerk_process_noise;
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt3 * dt;
    const float dt5 = dt4 * dt;
    observer->filter.Q_data[0] = q * dt5 / 20.0f;
    observer->filter.Q_data[1] = q * dt4 / 8.0f;
    observer->filter.Q_data[2] = q * dt3 / 6.0f;
    observer->filter.Q_data[3] = observer->filter.Q_data[1];
    observer->filter.Q_data[4] = q * dt3 / 3.0f;
    observer->filter.Q_data[5] = q * dt2 / 2.0f;
    observer->filter.Q_data[6] = observer->filter.Q_data[2];
    observer->filter.Q_data[7] = observer->filter.Q_data[5];
    observer->filter.Q_data[8] = q * dt;
}
