#include "comm.h"

#include <string.h>

#include "robot_config.h"

#include "bsp_dwt.h"
#include "master_process.h"
#include "ui.h"
#include "user_lib.h"
#include "vofa.h"

#if !defined(ONE_BOARD)
#define COMM_PERIOD_100HZ_MS 10.0f
#define COMM_PERIOD_50HZ_MS 20.0f
#define COMM_PERIOD_20HZ_MS 50.0f

static CommLostMonitor_s comm_lost_monitor;

static ChassisCommLostBeepConfig_t LineMakeBeep(CommLostId_e id) {
  static const uint16_t freq[COMM_LOST_ID_NUM] = {
      [COMM_LOST_UP_MAIN] = DoFreq,
      [COMM_LOST_FETCH_MAIN] = ReFreq,
      [COMM_LOST_UP_MOTION] = MiFreq,
      [COMM_LOST_FETCH_MOTION] = FaFreq,
      [COMM_LOST_UP_GAMESTATE] = SoFreq,
      [COMM_LOST_FETCH_GAMESTATE] = LaFreq,
  };

  ChassisCommLostBeepConfig_t beep = {
      .frequency = SoFreq,
      .count = COMM_LOST_BEEP_NUM,
      .loudness = COMM_LOST_BEEP_LOUDNESS,
  };

  if (id < COMM_LOST_ID_NUM) {
    beep.frequency = freq[id];
  }
  return beep;
}

static void LineInit(CommLostLine_s* line, const CommLostLineConfig_s* config) {
  if (line == NULL || config == NULL) return;

  line->comm = config->comm;
  line->id = config->id;
  line->beep = LineMakeBeep(config->id);
  line->armed = 0;
  line->last_online = 0;
}

static void MonitorInit(CommLostMonitor_s* monitor, const CommLostLineConfig_s* configs) {
  if (monitor == NULL || configs == NULL) return;

  for (size_t i = 0; i < COMM_LOST_LOCAL_LINE_NUM; i++) {
    LineInit(&monitor->lines[i], &configs[i]);
  }
}

static uint8_t LineNotifyLost(CommLostLine_s* line) {
  /* TODO：接入当前 alarm 模块的非阻塞掉线提示；本阶段只确认状态边沿。 */
  return line != NULL;
}

static void LineTask(CommLostLine_s* line) {
  if (line == NULL || line->comm == NULL) return;

  if (!line->armed) {
    if (!CANCommIsOnline(line->comm)) return;

    line->armed = 1;
    line->last_online = 1;
  }

  uint8_t is_online = CANCommIsOnline(line->comm);
  if (is_online) {
    line->last_online = 1;
    return;
  }

  if (line->last_online && LineNotifyLost(line)) {
    line->last_online = 0;
  }
}

static void MonitorTask(CommLostMonitor_s* monitor) {
  if (monitor == NULL) return;

  for (size_t i = 0; i < sizeof(monitor->lines) / sizeof(monitor->lines[0]); i++) {
    LineTask(&monitor->lines[i]);
  }
}

static uint8_t CommPeriodElapsed(float now_ms, float* last_ms, float period_ms) {
  if (now_ms - *last_ms < period_ms) {
    return 0;
  }

  *last_ms = now_ms;
  return 1;
}

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
  const CommLostLineConfig_s gimbal_lost_configs[COMM_LOST_LOCAL_LINE_NUM] = {
      {robot->main_comm, COMM_LOST_UP_MAIN},
      {robot->motion_comm, COMM_LOST_UP_MOTION},
      {robot->gamestate_comm, COMM_LOST_UP_GAMESTATE},
  };
  MonitorInit(&comm_lost_monitor, gimbal_lost_configs);
  VOFAInit(&huart6);
#endif

#if defined(CHASSIS_BOARD)
  robot->chassis->chassis_ctrl_cmd.max_power = robot->referee_data->GameRobotState.chassis_power_limit;

  robot->chassis_upload_data = (Chassis_Upload_Data_s*)zmalloc(sizeof(Chassis_Upload_Data_s));
  robot->chassis_fetch_data = (Chassis_Fetch_Data_s*)zmalloc(sizeof(Chassis_Fetch_Data_s));
  robot->main_comm = CANCommInit(&chassis_main_comm_conf);
  robot->motion_comm = CANCommInit(&chassis_motion_comm_conf);
  robot->gamestate_comm = CANCommInit(&chassis_gamestate_comm_conf);
  const CommLostLineConfig_s chassis_lost_configs[COMM_LOST_LOCAL_LINE_NUM] = {
      {robot->main_comm, COMM_LOST_FETCH_MAIN},
      {robot->motion_comm, COMM_LOST_FETCH_MOTION},
      {robot->gamestate_comm, COMM_LOST_FETCH_GAMESTATE},
  };
  MonitorInit(&comm_lost_monitor, chassis_lost_configs);
  VOFAInit(&huart1);
#endif
}

void RobotCommTask(RobotInstance* robot) {
  if (robot == NULL || robot->main_comm == NULL || robot->motion_comm == NULL || robot->gamestate_comm == NULL) return;

  Chassis_Upload_Data_s* chassis_upload_data = robot->chassis_upload_data;
  Chassis_Fetch_Data_s* chassis_fetch_data = robot->chassis_fetch_data;
  if (chassis_upload_data == NULL || chassis_fetch_data == NULL) return;

  float now_ms = DWT_GetTimeline_ms();
  MonitorTask(&comm_lost_monitor);

#if defined(GIMBAL_BOARD)
  static SuperCap_Ctrl_Cmd_e last_local_cmd = NORMAL;
  static float last_change_time = 0;
  static float last_fetch_main_send_time = -COMM_PERIOD_50HZ_MS;
  static float last_fetch_motion_send_time = -COMM_PERIOD_20HZ_MS;
  static float last_fetch_gamestate_send_time = -COMM_PERIOD_20HZ_MS;

  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;

  if (robot->chassis->super_cap->super_cap_ctrl_cmd != last_local_cmd) {
    last_change_time = now_ms;
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
  chassis_fetch_data->gamestate.ui_status.fire_mode = (uint8_t)GetFireMode();

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

  if (now_ms - last_change_time > 500.0f) {
    robot->chassis->super_cap->super_cap_ctrl_cmd = chassis_upload_data->main.super_cap_ctrl_cmd;
    last_local_cmd = chassis_upload_data->main.super_cap_ctrl_cmd;
  }

  if (CommPeriodElapsed(now_ms, &last_fetch_main_send_time, COMM_PERIOD_50HZ_MS)) {
    CANCommSend(robot->main_comm, (uint8_t*)&chassis_fetch_data->main);
    chassis_fetch_data->main.force_refresh_ui = 0;
  }
  if (CommPeriodElapsed(now_ms, &last_fetch_motion_send_time, COMM_PERIOD_20HZ_MS)) {
    CANCommSend(robot->motion_comm, (uint8_t*)&chassis_fetch_data->motion);
  }
  if (CommPeriodElapsed(now_ms, &last_fetch_gamestate_send_time, COMM_PERIOD_20HZ_MS)) {
    CANCommSend(robot->gamestate_comm, (uint8_t*)&chassis_fetch_data->gamestate);
  }
#elif defined(CHASSIS_BOARD)
  static float last_upload_main_send_time = -COMM_PERIOD_20HZ_MS;
  static float last_upload_motion_send_time = -COMM_PERIOD_100HZ_MS;
  static float last_upload_gamestate_send_time = -COMM_PERIOD_20HZ_MS;

  memcpy(&chassis_fetch_data->main, CANCommGet(robot->main_comm), sizeof(chassis_fetch_data->main));
  memcpy(&chassis_fetch_data->motion, CANCommGet(robot->motion_comm), sizeof(chassis_fetch_data->motion));
  memcpy(&chassis_fetch_data->gamestate, CANCommGet(robot->gamestate_comm), sizeof(chassis_fetch_data->gamestate));

  chassis_upload_data->motion.Pitch = robot->chassis->imu->Pitch;
  chassis_upload_data->motion.YawTotalAngle = robot->chassis->imu->YawTotalAngle;
  chassis_upload_data->motion.yaw_speed = robot->chassis->imu->Gyro[2];
  float abs_l = fabsf(robot->chassis->state_var.theta_l);
  float abs_r = fabsf(robot->chassis->state_var.theta_r);
  chassis_upload_data->motion.max_theta = abs_l > abs_r ? abs_l : abs_r;
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

  if (CommPeriodElapsed(now_ms, &last_upload_main_send_time, COMM_PERIOD_20HZ_MS)) {
    CANCommSend(robot->main_comm, (uint8_t*)&chassis_upload_data->main);
  }
  if (CommPeriodElapsed(now_ms, &last_upload_motion_send_time, COMM_PERIOD_100HZ_MS)) {
    CANCommSend(robot->motion_comm, (uint8_t*)&chassis_upload_data->motion);
  }
  if (CommPeriodElapsed(now_ms, &last_upload_gamestate_send_time, COMM_PERIOD_20HZ_MS)) {
    CANCommSend(robot->gamestate_comm, (uint8_t*)&chassis_upload_data->gamestate);
  }
#endif
}
#else
void RobotCommInit(RobotInstance* robot) { (void)robot; }

void RobotCommTask(RobotInstance* robot) { (void)robot; }
#endif
