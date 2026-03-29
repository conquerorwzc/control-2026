/**
 * @file    chassis.h
 * @author  Shiyu Li
 * @date    2026/3/28
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   wheel_leg->prostrate
 * @note    wheel_motor[0]————>右轮
 *          wheel_motor[1]————>左轮
 */
#include "chassis.h"

#include <math.h>
#include <string.h>

#include "general_def.h"
#include "referee.h"
#include "user_lib.h"

static ChassisInstance* chassis;
static Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd;
static referee_info_t* referee_data;

static float wheel_speed_ref[2];

/**
 * @brief   超电取电策略
 */
static void SuperCapModeControl() {
  switch (chassis->super_cap_mode) {
    case SAFETY_MODE:
      if (chassis->super_cap->cap_msg.cap_v > 18.0f) chassis->super_cap_mode = PASSIVE_MODE;
      chassis->chassis_ctrl_cmd.max_power =
          referee_data->GameRobotState
              .chassis_power_limit;  // referee_data->GameRobotState.chassis_power_limit;//TODO:用超电记得改;
      break;
    case FORCED_CHARGING_MODE:
      if (chassis->super_cap->cap_msg.cap_v < 8.0f) chassis->super_cap_mode = SAFETY_MODE;
      if (chassis->super_cap->cap_msg.cap_v > 18.0f) chassis->super_cap_mode = PASSIVE_MODE;
      chassis->chassis_ctrl_cmd.max_power = (uint16_t)(0.4 * referee_data->GameRobotState.chassis_power_limit);
      break;
    case CHARGING_MODE:
      if (chassis->super_cap->cap_msg.cap_v < 10.0f) chassis->super_cap_mode = FORCED_CHARGING_MODE;
      if (chassis->super_cap->cap_msg.cap_v > 18.0f) chassis->super_cap_mode = PASSIVE_MODE;
      chassis->chassis_ctrl_cmd.max_power =
          referee_data->GameRobotState.chassis_power_limit -
          (uint16_t)powf((float)referee_data->GameRobotState.chassis_power_limit * 0.05f, 2);
      break;
    case PASSIVE_MODE:
      if (chassis_ctrl_cmd->SuperCapBoost == 1) chassis->super_cap_mode = ACTIVE_MODE;
      if (chassis->super_cap->cap_msg.cap_v < 12.0f) chassis->super_cap_mode = CHARGING_MODE;
      chassis->chassis_ctrl_cmd.max_power =
          referee_data->GameRobotState.chassis_power_limit -
          (uint16_t)powf((float)referee_data->GameRobotState.chassis_power_limit * 0.04f, 2);
      break;
    case ACTIVE_MODE:
      if (chassis->super_cap->cap_msg.cap_v < 12.0f) chassis->super_cap_mode = CHARGING_MODE;
      if (chassis_ctrl_cmd->SuperCapBoost != 1) chassis->super_cap_mode = PASSIVE_MODE;
      chassis->chassis_ctrl_cmd.max_power = 200;
      break;
    default:
      chassis->super_cap_mode = SAFETY_MODE;
  }
}

/**
 * @brief 卧倒模式
 *
 * 差速：右轮 = vx + wz，左轮 = vx - wz
 */
void ChassisProstrateMode(void) {
#define VX_TO_MOTOR (30000.0f / 660.0f)
#define WZ_PID_TO_MOTOR 3000.0f
#define WZ_FF_TO_MOTOR (20000.0f / 660.0f)  // wz 前馈(摇杆量级) → 电机量
  float wz_pid = PIDCalculate(&chassis->yaw_prostrate_PID, chassis->imu->YawTotalAngle * DEGREE_2_RAD,
                              chassis_ctrl_cmd->target_yaw);
  float vx_motor = chassis_ctrl_cmd->vx * VX_TO_MOTOR;
  float wz_motor = wz_pid * WZ_PID_TO_MOTOR + chassis_ctrl_cmd->wz * WZ_FF_TO_MOTOR;

  // 差速分配
  wheel_speed_ref[0] = -1.0f * (vx_motor - wz_motor);  // 右轮 leg[0]
  wheel_speed_ref[1] = vx_motor + wz_motor;  // 左轮 leg[1]
}

static void EnableJointMotor() {
  for (int i=0;i<4;i++)
    DMMotorOuterLoop(chassis->joint_motor[i], ANGLE_LOOP);
  DMMotorSetPIDRef(chassis->joint_motor[0], 0.1f);
  DMMotorSetPIDRef(chassis->joint_motor[1], -0.1f);
  DMMotorSetPIDRef(chassis->joint_motor[2], 0.1f);
  DMMotorSetPIDRef(chassis->joint_motor[3], -0.1f);
}


/**
 * @brief  最终输出
 * @note   leg[0]->joint_motor[0] ————> joint_motor[0]
 *         leg[0]->joint_motor[1] ————> joint_motor[1]
 *         leg[1]->joint_motor[0] ————> joint_motor[2]
 *         leg[1]->joint_motor[1] ————> joint_motor[3]
 *
 */
static void LimitChassisOutput(void) {
  for (int i = 0; i < 2; i++) {
    VAL_LIMIT(wheel_speed_ref[i], -53000.0f, 53000.0f);
    DJIMotorSetPIDRef(chassis->wheel_motor[i], wheel_speed_ref[i]);
  }
  // 不使能关节电机
  for (int i = 0; i < 4; i++) DMMotorStop(chassis->joint_motor[i]);
  // 使能关节电机，锁死
  // EnableJointMotor();
}

ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config) {
  ChassisInstance* chassis_instance = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));

  referee_data = GetReferee();

  for (int i = 0; i < 4; i++)
    chassis_instance->joint_motor[i] = DMMotorInit(&chassis_init_config->joint_motor_config[i]);
  for (int i = 0; i < 2; i++)
    chassis_instance->wheel_motor[i] = DJIMotorInit(&chassis_init_config->wheel_motor_config[i]);
  PIDInit(&chassis_instance->yaw_prostrate_PID, &chassis_init_config->yaw_prostrate_PID_config);

  chassis_instance->imu = INS_Init(&chassis_init_config->imu_init_config);

  chassis_instance->param = chassis_init_config->param;
  chassis = chassis_instance;
  chassis_ctrl_cmd = &chassis->chassis_ctrl_cmd;

  // chassis->super_cap_mode = SAFETY_MODE;

  return chassis_instance;
}

void ChassisTask(void) {
  if (chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_POWER_OFF) {
    for (int i = 0; i < 4; i++) DMMotorStop(chassis->joint_motor[i]);
    for (int i = 0; i < 2; i++) DJIMotorStop(chassis->wheel_motor[i]);
  } else {
    for (int i = 0; i < 4; i++) DMMotorEnable(chassis->joint_motor[i]);
    for (int i = 0; i < 2; i++) DJIMotorEnable(chassis->wheel_motor[i]);
  }
  switch (chassis->chassis_ctrl_cmd.chassis_mode) {
    case CHASSIS_PROSTRATE:
      ChassisProstrateMode();
      break;
    default:
      break;
  }

  // SuperCapModeControl();
  // SuperCapSendMessage(chassis->super_cap, (int16_t)referee_data->GameRobotState.chassis_power_limit,
  // referee_data->PowerHeatData.buffer_energy, referee_data->GameRobotState.power_management_chassis_output);

  LimitChassisOutput();
}