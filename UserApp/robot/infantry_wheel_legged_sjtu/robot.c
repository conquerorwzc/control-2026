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

#include "bsp_gpio.h"
#include "can_comm.h"
#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"
#include "ui.h"
#include "ctrl.h"

static RobotInstance* robot;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd;
static Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd;
static Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd;
static Vision_Receive_s* vision_recv_data;

#if !defined(ONE_BOARD)
static Chassis_Upload_Data_s* chassis_upload_data;
static Chassis_Fetch_Data_s* chassis_fetch_data;
#endif

static RC_ctrl_t* rc_data;

/* Intermediate variables calculated by private functions */
static float angle;

// vofa数据
float visualized_data[20];

void VOFATask() {
#if defined(GIMBAL_BOARD)
  visualized_data[0] = robot->gimbal->gimbal_IMU_data->Yaw;
  visualized_data[1] = robot->gimbal->gimbal_IMU_data->Pitch;
  visualized_data[2] = robot->shoot->friction_motor[0]->motor_controller.pid_ref;
  visualized_data[3] = robot->shoot->friction_motor[0]->measure.speed_aps;
  visualized_data[4] = robot->shoot->friction_motor[1]->motor_controller.pid_ref;
  visualized_data[5] = robot->shoot->friction_motor[1]->measure.speed_aps;
  visualized_data[6] = robot->shoot->shoot_ctrl_cmd.initial_speed;
  visualized_data[7] = robot->shoot->loader_motor->measure.total_angle;
  visualized_data[8] = robot->gimbal->pitch_motor->motor_controller.final_output;
  visualized_data[9] = robot->gimbal->pitch_motor->motor_controller.pid_ref;
#elif defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  visualized_data[0] = robot->chassis->power_ctrl.P_total;
  visualized_data[1] = robot->chassis->power_ctrl.vel_max;
  visualized_data[2] = robot->chassis->power_ctrl.P_limit;
  visualized_data[3] = robot->chassis->limited_vx;           // 限制后速度
  visualized_data[4] = robot->chassis->chassis_ctrl_cmd.vx;  // 原始指令速度
  visualized_data[5] = robot->chassis->state_var.v_b_h;      // 实际速度
  // 新增：旋转占比（调试小陀螺功率分配）
  float w_L = robot->chassis->leg[1]->wheel_motor->measure.speed_aps * DEGREE_2_RAD;
  float w_R = robot->chassis->leg[0]->wheel_motor->measure.speed_aps * DEGREE_2_RAD;
  float w_sum = fabsf(w_L + w_R);
  float w_diff = fabsf(w_L - w_R);
  visualized_data[6] = (w_sum + w_diff > 0.1f) ? w_diff / (w_sum + w_diff) : 0.0f;  // rotate_ratio
  visualized_data[7] = robot->chassis->power_ctrl.P_wheel_L;
  visualized_data[8] = robot->chassis->power_ctrl.P_wheel_R;
  visualized_data[9] = robot->chassis->imu->Yaw;
  visualized_data[10] = robot->chassis->imu->Pitch;
  visualized_data[11] = robot->chassis->imu->Roll;
#endif
  VOFAJustFloatSend(visualized_data, 20);
}

#define robot_lost_control (abs(robot->chassis->imu->Pitch) > 13.0f)

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle() {
  angle = robot->gimbal->yaw_motor->measure.angle_single_round;

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
 *
 * 每次进入 recovery 时，覆盖 gimbal yaw 指令将云台转向底盘正前方。
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
 * @brief  紧急停止,包括遥控器右拨杆往下/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler() {
  if (robot_lost_control) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_RECOVERY;
  }
  // 两switch都在下或者遥控器断连，断电
  if ((switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left)) |
      switch_is_off(rc_data[TEMP].rc.switch_right)) {
    robot->robot_mode = ROBOT_POWER_OFF;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    chassis_ramp.planning_v = 0.0f;
    chassis_ramp.expected_a = 0.0f;
    LOGERROR("[CMD] emergency stop!");

  } else {
    LOGINFO("[CMD] reinstate, robot ready");
  }  // 底盘失能
  if (switch_is_down(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
  }  // 发射失能
  if (switch_is_down(rc_data[TEMP].rc.switch_left)) {
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }
}
/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
  RemoteControlSet(robot);
  MouseKeySet(robot);
#if defined(GIMBAL_BOARD)
  CalcOffsetAngle();
  GimbalAlignToChassisForward();
#endif
  EmergencyHandler();  // 急停必须在 CAN 发送之前,确保 POWER_OFF 优先级最高
#if defined(GIMBAL_BOARD)
  chassis_fetch_data->chassis_ctrl_cmd = *chassis_ctrl_cmd;
  *chassis_upload_data = *(Chassis_Upload_Data_s*)CANCommGet(robot->can_comm);
  robot->chassis->imu->Roll = chassis_upload_data->Roll;
  robot->chassis->imu->Pitch = chassis_upload_data->Pitch;
  robot->chassis->imu->YawTotalAngle = chassis_upload_data->YawTotalAngle;
  robot->chassis->imu->Gyro[2] = chassis_upload_data->YawSpeed;
  shoot_ctrl_cmd->initial_speed = chassis_upload_data->bullet_speed;

  CANCommSend(robot->can_comm, (void*)chassis_fetch_data);
#endif
#elif defined(CHASSIS_BOARD)
  chassis_upload_data->Pitch = robot->chassis->imu->Pitch;
  chassis_upload_data->Roll = robot->chassis->imu->Roll;
  chassis_upload_data->YawTotalAngle = robot->chassis->imu->YawTotalAngle;
  chassis_upload_data->YawSpeed = robot->chassis->imu->Gyro[2];
  chassis_upload_data->bullet_speed = robot->referee_data->ShootData.initial_speed;

  *chassis_fetch_data = *(Chassis_Fetch_Data_s*)CANCommGet(robot->can_comm);
  robot->chassis->chassis_ctrl_cmd = chassis_fetch_data->chassis_ctrl_cmd;
  CANCommSend(robot->can_comm, (void*)chassis_upload_data);
#endif
}

void RobotInit() {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)

  // 遥控器初始化
#if defined(STM32F4)
  robot->rc_data = RemoteControlInit(&huart3);
#elif defined(STM32H7)
  robot->rc_data = RemoteControlInit(&huart5);
#endif
  // rc_data_last handled in CtrlInit now

  PIDInit(&robot->chassis_rotate_PID, &chassis_rotate_PID_config);
  rc_data = robot->rc_data;
#if defined(GIMBAL_BOARD)
  robot->gimbal = GimbalInit(&gimbal_init_config);
  robot->shoot = ShootInit(&shoot_init_config);
  gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  vision_recv_data = VisionInit(&gimbal_init_config.imu_init_config);
  robot->vision_recv_data = vision_recv_data; // Assign to robot instance for access in ctrl module

  robot->chassis_upload_data = (Chassis_Upload_Data_s*)zmalloc(sizeof(Chassis_Upload_Data_s));
  robot->chassis_fetch_data = (Chassis_Fetch_Data_s*)zmalloc(sizeof(Chassis_Fetch_Data_s));
  chassis_upload_data = robot->chassis_upload_data;
  chassis_fetch_data = robot->chassis_fetch_data;

  robot->chassis = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));
  robot->chassis->imu = (INS_t*)zmalloc(sizeof(INS_t));
  robot->can_comm = CANCommInit(&gimbal_comm_conf);
  VOFAInit(&huart1);
#endif
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  robot->referee_data = RefereeInit(&huart7);  // 裁判系统初始化
  // robot->super_cap = SuperCapInit(&super_cap_config);
  robot->chassis = ChassisInit(&chassis_init_config);
#if defined(CHASSIS_BOARD)
  chassis_ctrl_cmd->max_power = robot->referee_data->GameRobotState.chassis_power_limit;

  robot->chassis_upload_data = (Chassis_Upload_Data_s*)zmalloc(sizeof(Chassis_Upload_Data_s));
  robot->chassis_fetch_data = (Chassis_Fetch_Data_s*)zmalloc(sizeof(Chassis_Fetch_Data_s));
  chassis_upload_data = robot->chassis_upload_data;
  chassis_fetch_data = robot->chassis_fetch_data;
  robot->can_comm = CANCommInit(&chassis_comm_conf);  // can comm初始化
  VOFAInit(&huart1);
#endif
#endif
  chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  chassis_ctrl_cmd->leg_length = chassis_init_config.param.initial_leg_length;  // 初始腿长
  DWT_GetDeltaT(&robot->DWT_CNT);
  chassis_ctrl_cmd->max_power = 60;  // 测试用

  // UI初始化
  MyUIInit(robot);
  // Ctrl module initialization
  CtrlInit(robot);
}

void RobotTask() {
  robot->dt = DWT_GetDeltaT(&robot->DWT_CNT);
  RobotCMDTask();
  VOFATask();
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
  GimbalTask();
  ShootTask();
  VisionSend();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  ChassisTask();
#endif
}
