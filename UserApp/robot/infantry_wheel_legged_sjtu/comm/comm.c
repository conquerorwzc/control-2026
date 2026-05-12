#include "comm.h"

#include <string.h>

#include "robot_config.h"

#include "bsp_dwt.h"
#include "master_process.h"
#include "ui.h"
#include "user_lib.h"
#include "vofa.h"
#include "can_comm.h"

#if !defined(ONE_BOARD)
void RobotCommInit(RobotInstance* robot) {
  if (robot == NULL) return;

#if defined(GIMBAL_BOARD)
  robot->gimbal = GimbalInit(&gimbal_init_config);
  robot->shoot = ShootInit(&shoot_init_config);
  robot->shoot->shoot_ctrl_cmd.heat_mode = REFEREE_CONTROL;

  robot->vision_recv_data = VisionInit(&gimbal_init_config.imu_init_config);

  robot->chassis_upload_data = (Chassis_Upload_Data_s*)zmalloc(sizeof(Chassis_Upload_Data_s));
  robot->chassis_fetch_data = (Chassis_Fetch_Data_s*)zmalloc(sizeof(Chassis_Fetch_Data_s));

  robot->chassis = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));
  robot->chassis->imu = (INS_t*)zmalloc(sizeof(INS_t));
  robot->chassis->super_cap = (SuperCapInstance*)zmalloc(sizeof(SuperCapInstance));
  robot->main_comm = CANCommInit(&gimbal_main_comm_conf);
  robot->motion_comm = CANCommInit(&gimbal_motion_comm_conf);
  robot->gamestate_comm = CANCommInit(&gimbal_gamestate_comm_conf);
  VOFAInit(&huart6);
#endif

#if defined(CHASSIS_BOARD)
  robot->chassis->chassis_ctrl_cmd.max_power = robot->referee_data->GameRobotState.chassis_power_limit;

  robot->chassis_upload_data = (Chassis_Upload_Data_s*)zmalloc(sizeof(Chassis_Upload_Data_s));
  robot->chassis_fetch_data = (Chassis_Fetch_Data_s*)zmalloc(sizeof(Chassis_Fetch_Data_s));
  robot->main_comm = CANCommInit(&chassis_main_comm_conf);
  robot->motion_comm = CANCommInit(&chassis_motion_comm_conf);
  robot->gamestate_comm = CANCommInit(&chassis_gamestate_comm_conf);
  VOFAInit(&huart1);
#endif
}

void RobotCommTask(RobotInstance* robot) {
  if (robot == NULL || robot->main_comm == NULL || robot->motion_comm == NULL || robot->gamestate_comm == NULL) return;

  static float last_comm_time = 0.0f;
  if (DWT_GetTimeline_ms() - last_comm_time < 20.f) return;
  last_comm_time = DWT_GetTimeline_ms();

  Chassis_Upload_Data_s* chassis_upload_data = robot->chassis_upload_data;
  Chassis_Fetch_Data_s* chassis_fetch_data = robot->chassis_fetch_data;
  if (chassis_upload_data == NULL || chassis_fetch_data == NULL) return;

#if defined(GIMBAL_BOARD)
  static SuperCap_Ctrl_Cmd_e last_local_cmd = NORMAL;
  static float last_change_time = 0;

  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;

  if (robot->chassis->super_cap->super_cap_ctrl_cmd != last_local_cmd) {
    last_change_time = DWT_GetTimeline_ms();
    last_local_cmd = robot->chassis->super_cap->super_cap_ctrl_cmd;
  }

  chassis_fetch_data->motion.chassis_ctrl_cmd = *chassis_ctrl_cmd;
  chassis_fetch_data->main.super_cap_ctrl_cmd = robot->chassis->super_cap->super_cap_ctrl_cmd;
  chassis_fetch_data->main.gimbal_aligned = robot->update_flag.is_gimbal_aligned;
  chassis_fetch_data->gamestate.ui_status.ui_chassis_relative_angle_deg_x10 = (int16_t)(robot->offset_angle * 10.0f);
  chassis_fetch_data->gamestate.ui_status.robot_mode = (uint8_t)robot->robot_mode;
  chassis_fetch_data->gamestate.ui_status.gimbal_mode = (uint8_t)gimbal_ctrl_cmd->gimbal_mode;
  chassis_fetch_data->gamestate.ui_status.friction_mode = (uint8_t)shoot_ctrl_cmd->friction_mode;
  chassis_fetch_data->gamestate.ui_status.loader_mode = (uint8_t)shoot_ctrl_cmd->load_mode;
  chassis_fetch_data->gamestate.ui_status.fire_flag =
      (uint8_t)(robot->vision_recv_data != NULL && robot->vision_recv_data->shoot_receive.fire_flag != 0);

  memcpy(&chassis_upload_data->main, CANCommGet(robot->main_comm), sizeof(chassis_upload_data->main));
  memcpy(&chassis_upload_data->motion, CANCommGet(robot->motion_comm), sizeof(chassis_upload_data->motion));
  memcpy(&chassis_upload_data->gamestate, CANCommGet(robot->gamestate_comm), sizeof(chassis_upload_data->gamestate));
  robot->chassis->imu->Pitch = chassis_upload_data->motion.Pitch;
  robot->chassis->imu->YawTotalAngle = chassis_upload_data->motion.YawTotalAngle;
  robot->chassis->imu->Gyro[2] = chassis_upload_data->motion.yaw_speed;
  shoot_ctrl_cmd->initial_speed = chassis_upload_data->gamestate.bullet_speed;
  shoot_ctrl_cmd->shooter_barrel_heat = chassis_upload_data->gamestate.shooter_17mm_barrel_heat;
  shoot_ctrl_cmd->shooter_barrel_heat_limit = chassis_upload_data->gamestate.shoot_heat_limit;
  VisionSetRefereeData(chassis_upload_data->gamestate.bullet_speed, chassis_upload_data->gamestate.robot_id);

  if (DWT_GetTimeline_ms() - last_change_time > 500.0f) {
    robot->chassis->super_cap->super_cap_ctrl_cmd = chassis_upload_data->main.super_cap_ctrl_cmd;
    last_local_cmd = chassis_upload_data->main.super_cap_ctrl_cmd;
  }

  CANCommSend(robot->main_comm, (uint8_t*)&chassis_fetch_data->main);
  CANCommSend(robot->motion_comm, (uint8_t*)&chassis_fetch_data->motion);
  CANCommSend(robot->gamestate_comm, (uint8_t*)&chassis_fetch_data->gamestate);
  chassis_fetch_data->main.force_refresh_ui = 0;
#elif defined(CHASSIS_BOARD)
  memcpy(&chassis_fetch_data->main, CANCommGet(robot->main_comm), sizeof(chassis_fetch_data->main));
  memcpy(&chassis_fetch_data->motion, CANCommGet(robot->motion_comm), sizeof(chassis_fetch_data->motion));
  memcpy(&chassis_fetch_data->gamestate, CANCommGet(robot->gamestate_comm), sizeof(chassis_fetch_data->gamestate));

  chassis_upload_data->motion.Pitch = robot->chassis->imu->Pitch;
  chassis_upload_data->motion.YawTotalAngle = robot->chassis->imu->YawTotalAngle;
  chassis_upload_data->motion.yaw_speed = robot->chassis->imu->Gyro[2];
  chassis_upload_data->gamestate.bullet_speed = robot->referee_data->ShootData.initial_speed;
  chassis_upload_data->gamestate.robot_id = robot->referee_data->GameRobotState.robot_id;
  chassis_upload_data->gamestate.shooter_17mm_barrel_heat =
      robot->referee_data->PowerHeatData.shooter_17mm_barrel_heat;
  chassis_upload_data->gamestate.shoot_heat_limit = robot->referee_data->GameRobotState.shooter_barrel_heat_limit;
  chassis_upload_data->main.super_cap_ctrl_cmd = robot->chassis->super_cap->super_cap_ctrl_cmd;

  robot->chassis->chassis_ctrl_cmd = chassis_fetch_data->motion.chassis_ctrl_cmd;
  robot->chassis->super_cap->super_cap_ctrl_cmd = chassis_fetch_data->main.super_cap_ctrl_cmd;
  robot->update_flag.is_gimbal_aligned = chassis_fetch_data->main.gimbal_aligned;
  robot->chassis->update_flag.gimbal_aligned = robot->update_flag.is_gimbal_aligned;

  if (chassis_fetch_data->main.force_refresh_ui) {
    Referee_Interactive_info_t* ui_data = getUI();
    if (ui_data != NULL) {
      ui_data->force_refresh_ui = 1;
    }
  }

  CANCommSend(robot->main_comm, (uint8_t*)&chassis_upload_data->main);
  CANCommSend(robot->motion_comm, (uint8_t*)&chassis_upload_data->motion);
  CANCommSend(robot->gamestate_comm, (uint8_t*)&chassis_upload_data->gamestate);
#endif
}
#else
void RobotCommInit(RobotInstance* robot) { (void)robot; }

void RobotCommTask(RobotInstance* robot) { (void)robot; }
#endif
