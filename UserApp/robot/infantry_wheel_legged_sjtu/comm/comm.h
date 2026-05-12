#ifndef INFANTRY_COMM_H
#define INFANTRY_COMM_H

#include "chassis.h"
#include "super_cap.h"

typedef struct RobotInstance RobotInstance;


#ifndef ONE_BOARD
#pragma pack(1)
typedef struct {
  struct {
    SuperCap_Ctrl_Cmd_e super_cap_ctrl_cmd;
  } main;

  struct {
    float Pitch;
    float YawTotalAngle;
    float yaw_speed;
  } motion;

  struct {
    float bullet_speed;
    uint8_t robot_id;
    int shooter_17mm_barrel_heat;
    int shoot_heat_limit;
  } gamestate;
} Chassis_Upload_Data_s;

typedef struct {
  uint8_t robot_mode;
  uint8_t gimbal_mode;
  uint8_t friction_mode;
  uint8_t loader_mode;
  uint8_t fire_flag;
  int16_t ui_chassis_relative_angle_deg_x10;
} UI_Remote_Status_s;

typedef struct {
  struct {
    SuperCap_Ctrl_Cmd_e super_cap_ctrl_cmd;
    uint8_t force_refresh_ui;
    uint8_t gimbal_aligned;
  } main;

  struct {
    Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
  } motion;

  struct {
    UI_Remote_Status_s ui_status;
  } gamestate;
} Chassis_Fetch_Data_s;

#pragma pack()
#endif

void RobotCommInit(RobotInstance* robot);
void RobotCommTask(RobotInstance* robot);

#endif  // INFANTRY_COMM_H
