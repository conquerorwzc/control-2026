#ifndef SENTRY_GIMBAL_H
#define SENTRY_GIMBAL_H

/**
 * @brief 初始化云台,会被RobotInit()调用
 *
 */
void SentryGimbalInit();

/**
 * @brief 云台任务
 *
 */
void SentryGimbalTask();

#endif // SENTRY_GIMBAL_H