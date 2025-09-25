#ifndef ROBOT_H
#define ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(INFANTRY_WHEEL_LEGGED)
#include "infantry_wheel_legged/robot.h"
#elif defined(SENDTY_DOUBLE_HEADED)
#include "sentry_double_headed/robot.h"
#endif

void RobotInit(void);
void RobotTask(void);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_H