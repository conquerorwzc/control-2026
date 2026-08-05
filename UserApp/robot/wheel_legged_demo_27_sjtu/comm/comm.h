#ifndef INFANTRY_COMM_H
#define INFANTRY_COMM_H

#include "buzzer.h"
#include "can_comm.h"
#include "chassis.h"
#include "super_cap.h"

typedef struct RobotInstance RobotInstance;


#ifndef ONE_BOARD
#define COMM_LOST_LOCAL_LINE_NUM 3
#define COMM_LOST_BEEP_NUM 3
#define COMM_LOST_BEEP_LOUDNESS 0.5f

/* 当前报警模块不提供旧 SJTU 的单次鸣叫接口；保留掉线事件参数供后续接入。 */
typedef struct {
  uint16_t frequency;
  uint8_t count;
  float loudness;
} ChassisCommLostBeepConfig_t;

typedef enum {
  COMM_LOST_UP_MAIN = 0,
  COMM_LOST_FETCH_MAIN,
  COMM_LOST_UP_MOTION,
  COMM_LOST_FETCH_MOTION,
  COMM_LOST_UP_GAMESTATE,
  COMM_LOST_FETCH_GAMESTATE,
  COMM_LOST_ID_NUM,
} CommLostId_e;

typedef struct {
  CANCommInstance* comm;
  CommLostId_e id;
} CommLostLineConfig_s;

typedef struct {
  CANCommInstance* comm;
  CommLostId_e id;
  ChassisCommLostBeepConfig_t beep;
  uint8_t armed;
  uint8_t last_online;
} CommLostLine_s;

typedef struct {
  CommLostLine_s lines[COMM_LOST_LOCAL_LINE_NUM];
} CommLostMonitor_s;

#pragma pack(1)
typedef struct {
  struct {
    SuperCap_Ctrl_Cmd_e super_cap_ctrl_cmd;
  } main;

  struct {
    float Pitch;
    float YawTotalAngle;
    float yaw_speed;
    float max_theta;
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
  uint8_t fire_mode;
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
