#pragma once

#include "robot.h"

#define UI_CENTER_X 960
#define UI_CENTER_Y 540

#pragma pack(1)
typedef struct {
  uint32_t relative_flag : 1;
  uint32_t leg_flag : 1;
  uint32_t status_flag : 1;
  uint32_t cap_flag : 1;
  uint32_t aim_flag : 1;
  uint32_t speed_flag : 1;
} UI_Interactive_Flag_t;
#pragma pack()

typedef struct {
  UI_Interactive_Flag_t UI_Interactive_Flag;

  /* 底盘相对角度 */
  float chassis_relative_angle;
  float last_chassis_relative_angle;

  /* 五连杆腿部位姿 */
  uint8_t leg_valid;
  uint8_t last_leg_valid;
  float leg_phi1;
  float leg_phi2;
  float leg_phi3;
  float leg_phi4;
  float last_leg_phi1;
  float last_leg_phi2;
  float last_leg_phi3;
  float last_leg_phi4;

  /* 机器人各子系统模式 */
  Robot_Mode_e robot_mode;
  Robot_Mode_e last_robot_mode;
  Chassis_Mode_e chassis_mode;
  Chassis_Mode_e last_chassis_mode;
  Gimbal_Mode_e gimbal_mode;
  Gimbal_Mode_e last_gimbal_mode;
  Friction_Mode_e friction_mode;
  Friction_Mode_e last_friction_mode;
  Loader_Mode_e loader_mode;
  Loader_Mode_e last_loader_mode;

  /* Chassis attitude, deg */
  float chassis_pitch;
  float last_chassis_pitch;
  float chassis_roll;
  float last_chassis_roll;

  /* 超级电容 */
  SuperCap_Mode_e super_cap_mode;
  SuperCap_Mode_e last_super_cap_mode;
  SuperCap_Ctrl_Cmd_e super_cap_ctrl_cmd;
  SuperCap_Ctrl_Cmd_e last_super_cap_ctrl_cmd;
  float cap_voltage;
  float last_cap_voltage;
  uint8_t cap_error;
  uint8_t last_cap_error;

  /* 速度 */
  float speed;
  float last_speed;
  uint8_t speed_is_prostrate;
  uint8_t last_speed_is_prostrate;

  /* 瞄准目标 */
  uint8_t aim_target_flag;
  uint8_t last_aim_target_flag;

  /* 外部强制刷新标志 */
  uint8_t force_refresh_ui;
} Referee_Interactive_info_t;

Referee_Interactive_info_t *getUI(void);
void MyUIInit(RobotInstance *robot);
void UITask(RobotInstance *robot);
