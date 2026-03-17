/**
******************************************************************************
* @file    robot.c
* @author  Enhao Zhang
* @date    2025/8/8
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief Infantry wheel-legged robot control module
******************************************************************************
* @attention
* None
*
******************************************************************************
*/

#include "robot.h"

#include "robot_config.h"

// bsp
#include "bsp_gpio.h"
// modules
#include "can_comm.h"
#include "general_def.h"
#include "user_lib.h"
#include "vofa.h"
// robot components
#include "ctrl.h"
#include "ui.h"

static RobotInstance* robot;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd;
static Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd;
static Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd;
static Vision_Receive_s* vision_recv_data;
// vofa数据
float visualized_data[20];

void VOFATask() {
#if defined(GIMBAL_BOARD)
  visualized_data[0] = robot->gimbal->gimbal_IMU_data->Yaw;
  visualized_data[1] = robot->gimbal->gimbal_IMU_data->Pitch;
#elif defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  visualized_data[0] = robot->chassis->leg[0]->real_model.T;
  visualized_data[1] = robot->chassis->leg[1]->real_model.T;
#endif
  VOFAJustFloatSend(visualized_data, 20);
}


// 双板通信
#if !defined(ONE_BOARD)
static Chassis_Upload_Data_s* chassis_upload_data;
static Chassis_Fetch_Data_s* chassis_fetch_data;

static void DoubleBoardCommsInit() {
#if defined(GIMBAL_BOARD)
  robot->gimbal = GimbalInit(&gimbal_init_config);
  robot->shoot = ShootInit(&shoot_init_config);
  gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  // 视觉接收初始化
  vision_recv_data = VisionInit(&gimbal_init_config.imu_init_config);
  robot->vision_recv_data = vision_recv_data;  // Assign to robot instance for access in ctrl module

  robot->chassis_upload_data = (Chassis_Upload_Data_s*)zmalloc(sizeof(Chassis_Upload_Data_s));
  robot->chassis_fetch_data = (Chassis_Fetch_Data_s*)zmalloc(sizeof(Chassis_Fetch_Data_s));
  chassis_upload_data = robot->chassis_upload_data;
  chassis_fetch_data = robot->chassis_fetch_data;

  robot->chassis = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));
  robot->chassis->imu = (INS_t*)zmalloc(sizeof(INS_t));
  robot->can_comm = CANCommInit(&gimbal_comm_conf);
  VOFAInit(&huart1);
#endif
#if defined(CHASSIS_BOARD)
  chassis_ctrl_cmd->max_power = robot->referee_data->GameRobotState.chassis_power_limit;

  robot->chassis_upload_data = (Chassis_Upload_Data_s*)zmalloc(sizeof(Chassis_Upload_Data_s));
  robot->chassis_fetch_data = (Chassis_Fetch_Data_s*)zmalloc(sizeof(Chassis_Fetch_Data_s));
  chassis_upload_data = robot->chassis_upload_data;
  chassis_fetch_data = robot->chassis_fetch_data;
  robot->can_comm = CANCommInit(&chassis_comm_conf);  // can comm初始化
  VOFAInit(&huart1);
#endif
}

static void DoubleBoardComms() {
#if defined(GIMBAL_BOARD)
// 接收底盘回传数据
  *chassis_upload_data = *(Chassis_Upload_Data_s*)CANCommGet(robot->can_comm);
  robot->chassis->imu->Roll = chassis_upload_data->Roll;
  robot->chassis->imu->Pitch = chassis_upload_data->Pitch;
  robot->chassis->imu->YawTotalAngle = chassis_upload_data->YawTotalAngle;
  robot->chassis->imu->Gyro[2] = chassis_upload_data->YawSpeed;
  shoot_ctrl_cmd->initial_speed = chassis_upload_data->bullet_speed;
// 发送底盘控制指令
  chassis_fetch_data->chassis_ctrl_cmd = *chassis_ctrl_cmd;
  CANCommSend(robot->can_comm, (void*)chassis_fetch_data);
#elif defined(CHASSIS_BOARD)
// 接收底盘控制指令
  *chassis_fetch_data = *(Chassis_Fetch_Data_s*)CANCommGet(robot->can_comm);
  robot->chassis->chassis_ctrl_cmd = chassis_fetch_data->chassis_ctrl_cmd;
// 发送底盘回传数据
  chassis_upload_data->Pitch = robot->chassis->imu->Pitch;
  chassis_upload_data->Roll = robot->chassis->imu->Roll;
  chassis_upload_data->YawTotalAngle = robot->chassis->imu->YawTotalAngle;
  chassis_upload_data->YawSpeed = robot->chassis->imu->Gyro[2];
  chassis_upload_data->bullet_speed = robot->referee_data->ShootData.initial_speed;
  CANCommSend(robot->can_comm, (void*)chassis_upload_data);
#endif
}
#endif

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 * @note 单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle() {
  float angle = robot->gimbal->yaw_motor->measure.angle_single_round;
#if YAW_CHASSIS_ALIGN_ECD > 4096  // 如果大于180度
  if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
    robot->offset_angle = angle - YAW_ALIGN_ANGLE;
  else if (angle > 180.0f + YAW_ALIGN_ANGLE)
    robot->offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
  else
    robot->offset_angle = angle - YAW_ALIGN_ANGLE;
#else  // 小于180度
  if (angle > YAW_ALIGN_ANGLE)
    robot->offset_angle = angle - YAW_ALIGN_ANGLE;
  else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
    robot->offset_angle = angle - YAW_ALIGN_ANGLE;
  else
    robot->offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

/**
 * @brief 云台对齐底盘正方向
 * @note 每次进入 recovery 时，覆盖 gimbal yaw 指令将云台转向底盘正前方。
 * 对齐完成前持续强制 CHASSIS_RECOVERY；对齐后放行，恢复正常控制。
 * 必须在 CalcOffsetAngle() 之后调用。
 */
static void GimbalAlignToChassisForward(void) {
  static uint8_t gimbal_aligned = 0;
  static uint8_t was_recovery = 0;

  uint8_t is_recovery = (chassis_ctrl_cmd->chassis_mode == CHASSIS_RECOVERY);
  if (is_recovery && !was_recovery) {
    gimbal_aligned = 0;
  }

  if (!gimbal_aligned) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_RECOVERY;
    gimbal_ctrl_cmd->yaw = robot->gimbal->gimbal_IMU_data->YawTotalAngle + robot->offset_angle;
    // 5°误差内认为对齐完成
    if (fabsf(robot->offset_angle) < 5.0f) {
      gimbal_aligned = 1;
    }
  }

  was_recovery = (chassis_ctrl_cmd->chassis_mode == CHASSIS_RECOVERY);
}

/**
 * @brief 机器人核心控制任务，200 Hz 频率运行（必须高于视觉发送频率）
 * @note  该函数在 RobotTask 中被循环调用，负责解析输入、生成控制指令并协调各模块
 */
void RobotCMDTask() {
// 控制板指令处理与控制逻辑 (只有底盘板不用处理)
#if !defined(CHASSIS_BOARD)
  // TODO: OSDCtrl(robot); // 图传链路控制
  JoyStickCtrl(robot);
  // MouseKeyCtrl(robot);
#if defined(GIMBAL_BOARD)
  CalcOffsetAngle();
  GimbalAlignToChassisForward();
#endif
  EmergencyHandler(robot);  // 急停必须在 CAN 发送之前,确保 POWER_OFF 优先级最高
#endif

// 双板通信
#if !defined(ONE_BOARD)
  DoubleBoardComms();
#endif
}

void RobotInit() {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));
#if !defined(CHASSIS_BOARD)
  // 遥控器初始化
#if defined(STM32F4)
  robot->rc_data = RemoteControlInit(&huart3);
#elif defined(STM32H7)
  robot->rc_data = RemoteControlInit(&huart5);
#endif
  PIDInit(&robot->chassis_rotate_PID, &chassis_rotate_PID_config);
#endif
#if !defined(GIMBAL_BOARD)
  robot->referee_data = RefereeInit(&huart7);  // 裁判系统初始化
  // robot->super_cap = SuperCapInit(&super_cap_config);
  robot->chassis = ChassisInit(&chassis_init_config);
#endif
#if !defined(ONE_BOARD)
  DoubleBoardCommsInit();
#endif
  chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  chassis_ctrl_cmd->leg_length = chassis_init_config.param.initial_leg_length;  // 初始腿长
  DWT_GetDeltaT(&robot->DWT_CNT);
  chassis_ctrl_cmd->max_power = 60;  // 测试用

  // UI初始化
  // MyUIInit(robot);
}

void RobotTask() {
  robot->dt = DWT_GetDeltaT(&robot->DWT_CNT);
  RobotCMDTask();
  VOFATask();
#if !defined(CHASSIS_BOARD)
  GimbalTask();
  ShootTask();
  VisionSend();
#endif

#if !defined(GIMBAL_BOARD)
  ChassisTask();
#endif
}
