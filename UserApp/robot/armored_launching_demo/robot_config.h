

#ifndef CONTROL_2026_ROBOT_CONFIG_H
#define CONTROL_2026_ROBOT_CONFIG_H
#define ONE_BOARD

#include "ins_task.h"

#include <stdbool.h>

#define LAUNCHER_CMD_TOPIC "launcher_cmd"

typedef struct {
    bool load;      // 左摇杆本轮触发上簧
    bool release;   // 右摇杆本轮触发击发
    bool return_requested;  // 左摇杆下拨时，本轮发出一次手动回位请求
} LauncherCommand;
static IMU_Init_Config_s imu_init_config = {
    .flag = 1,
    .offset_flag=0,
    .scale = {1.0f, 1.0f, 1.0f},
    .Yaw = 0.0f,
    .Pitch = 0.0f,
    .Roll = 0.0f,
    .GyroOffset={0.0f, 0.0f, 0.0f},
};
#endif // CONTROL_2026_ROBOT_CONFIG_H
