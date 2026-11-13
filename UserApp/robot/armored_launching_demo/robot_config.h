

#ifndef CONTROL_2026_ROBOT_CONFIG_H
#define CONTROL_2026_ROBOT_CONFIG_H
#define ONE_BOARD

#include <stdbool.h>

#define LAUNCHER_CMD_TOPIC "launcher_cmd"

typedef struct {
    bool load;      // 左摇杆本轮触发上簧
    bool release;   // 右摇杆本轮触发击发
} LauncherCommand;

#endif // CONTROL_2026_ROBOT_CONFIG_H
