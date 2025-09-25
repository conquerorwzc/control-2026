#include "robot.h"

#if defined(INFANTRY_WHEEL_LEGGED)
#include "infantry_wheel_legged/robot.c"
#elif defined(SENDTY_DOUBLE_HEADED)
#include "sentry_double_headed/robot.c"
#endif