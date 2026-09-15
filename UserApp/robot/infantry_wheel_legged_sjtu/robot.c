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

#include <math.h>

#include "robot_config.h"

// bsp
#include "bsp_gpio.h"
// modules
#include "can_comm.h"
#include "general_def.h"
#include "master_process.h"
#include "user_lib.h"
#include "vofa.h"
// robot components
#include "ctrl.h"
#include "power_control.h"
#include "ui.h"
static RobotInstance* robot;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd;
static Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd;
static Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd;
// vofa数据
float visualized_data[20];

void VOFATask() {
#if defined(GIMBAL_BOARD)
  // visualized_data[0] = robot->gimbal->gimbal_IMU_data->Yaw;
  // visualized_data[1] = robot->gimbal->gimbal_IMU_data->Pitch;
  // visualized_data[2] = robot->gimbal->gimbal_IMU_data->Roll;
  visualized_data[0] = robot->gimbal->gimbal_ctrl_cmd.chassis_rotate_wz;
  visualized_data[1] = robot->chassis->imu->Gyro[2];
  visualized_data[2] = robot->gimbal->gimbal_ctrl_cmd.yaw;
  visualized_data[3] = robot->gimbal->gimbal_IMU_data->Yaw;
  visualized_data[4] = robot->shoot->friction_motor[0]->measure.speed_aps;
  visualized_data[5] = robot->shoot->friction_motor[0]->motor_controller.pid_ref;
  visualized_data[7] = shoot_ctrl_cmd->initial_speed;
  visualized_data[8] = robot->shoot->loader_motor->motor_controller.pid_ref;
  visualized_data[9] = robot->shoot->loader_motor->measure.total_angle;
  visualized_data[10] = robot->shoot->loader_motor->motor_controller.speed_PID.Ref;
  visualized_data[11] = robot->shoot->loader_motor->measure.speed_aps;
  visualized_data[12] = robot->shoot->loader_motor->motor_controller.final_output;
#elif defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  // visualized_data[0] = robot->chassis->state_var.x_b;
  // visualized_data[1] = robot->chassis->state_var.x_b_d;
  // visualized_data[2] = robot->chassis->chassis_ctrl_cmd.vx;
  // visualized_data[0] = robot->chassis->chassis_ctrl_cmd.wz;
  // visualized_data[1] = robot->chassis->state_var.phi_d;
  // visualized_data[2] = robot->chassis->chassis_ctrl_cmd.target_yaw;
  // visualized_data[3] = robot->chassis->state_var.phi;
  // === 双板通信调试 (定位 POWER_OFF 卡死) ===
  // [10] chassis_mode: 0=POWER_OFF 1=RECOVERY 2=ON 3=JUMP_READY 4=JUMP_START 5=PROSTRATE 6=STAIR
  // [11] gimbal_aligned: 0=未对齐, 1=已对齐
  // [12] cmd.vx (raw, 上层下发, 应跟随摇杆/WASD)
  // visualized_data[10] = (float)robot->chassis->chassis_ctrl_cmd.chassis_mode;
  // visualized_data[11] = (float)robot->chassis->update_flag.gimbal_aligned;
  // visualized_data[12] = robot->chassis->chassis_ctrl_cmd.vx;
  // visualized_data[4] = robot->chassis->state_var.theta_b * RAD_2_DEGREE;
  // visualized_data[5] = robot->chassis->roll_PID.Ref * RAD_2_DEGREE;
  // visualized_data[6] = robot->chassis->roll_PID.Measure * RAD_2_DEGREE;
  // visualized_data[0] = robot->chassis->power_ctrl->P_total;
  // visualized_data[1] = robot->chassis->power_ctrl->P_total_ref;
  // visualized_data[2] = robot->chassis->power_ctrl->scale_combined;
  // visualized_data[3] = robot->chassis->power_ctrl->P[0];
  // visualized_data[4] = robot->chassis->power_ctrl->I[0];
  // visualized_data[5] = robot->chassis->power_ctrl->w[0];
  // visualized_data[6] = robot->chassis->state_var.phi_d;

  visualized_data[6]= robot->chassis->super_cap->cap_msg.cap_v;
  visualized_data[7] = robot->chassis->super_cap->cap_msg.out_p;
  visualized_data[8] = robot->chassis->super_cap->cap_msg.in_p;
  visualized_data[9] = robot->chassis->super_cap->cap_msg.error_detect;

  // visualized_data[0] = robot->chassis->chassis_ctrl_cmd.vx;
  // visualized_data[1] = robot->chassis->vaEstimateKF.FilteredValue[0];
  // visualized_data[2] = robot->chassis->chassis_ctrl_cmd.wz;
  // visualized_data[3] = -1.0f * robot->chassis->imu->Gyro[2];

#endif
  VOFAJustFloatSend(visualized_data, 20);
}

RobotInstance* RobotGetInstance(void) { return robot; }

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
 * @note 上电时先接管 gimbal yaw 指令，将云台转向当前底盘正前方；
 * recovery 中也会接管 yaw；站立目标在对齐前保持 CHASSIS_RECOVERY，卧倒目标直接放行
 * CHASSIS_PROSTRATE。
 * 必须在 CalcOffsetAngle() 之后调用。
 */
static void GimbalAlignToChassisForward(void) {
  static uint8_t was_recovery = 0;

  uint8_t is_recovery = (chassis_ctrl_cmd->chassis_mode == CHASSIS_RECOVERY);
  if (is_recovery && !was_recovery) {
    robot->update_flag.is_gimbal_aligned = 0;
  }

  if (!robot->update_flag.is_gimbal_aligned) {
    uint8_t target_is_prostrate = robot->robot_mode == ROBOT_CHASSIS_PROSTRATE_FOLLOW ||
                                  robot->robot_mode == ROBOT_CHASSIS_PROSTRATE_ROTATE;
    chassis_ctrl_cmd->chassis_mode = target_is_prostrate ? CHASSIS_PROSTRATE : CHASSIS_RECOVERY;
    gimbal_ctrl_cmd->yaw = robot->gimbal->gimbal_IMU_data->YawTotalAngle + robot->offset_angle;
    // 5°误差内认为对齐完成；同时把 chassis_mode 一次性切到当前目标姿态。
    if (fabsf(robot->offset_angle) < 5.0f) {
      robot->update_flag.is_gimbal_aligned = 1;
      chassis_ctrl_cmd->chassis_mode = target_is_prostrate ? CHASSIS_PROSTRATE : CHASSIS_ON;
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
  JoyStickCtrl(robot);
  MouseKeyCtrl(robot);
  CtrlSolve(robot);
#if defined(GIMBAL_BOARD)
  CalcOffsetAngle();
  GimbalAlignToChassisForward();
  shoot_ctrl_cmd->shooter_barrel_heat = robot->referee_data->PowerHeatData.shooter_17mm_barrel_heat;
#endif
  EmergencyHandler(robot);  // 急停必须在 CAN 发送之前,确保 POWER_OFF 优先级最高
#endif
  RobotCommTask(robot);
}

void RobotInit() {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));
#if defined(GIMBAL_BOARD)
  robot->update_flag.is_gimbal_aligned = 0;
#elif defined(ONE_BOARD)
  robot->update_flag.is_gimbal_aligned = 1;
#endif
#if !defined(CHASSIS_BOARD)
  // 遥控器初始化
#if defined(STM32F4)
#ifdef USE_RC_CTRL
  robot->rc_data = RemoteControlInit(&huart3);
#elifdef USE_OCD_CTRL
  robot->rc_data = VT13RemoteInit(&huart1);
#endif

#elif defined(STM32H7)
#ifdef USE_RC_CTRL
  robot->rc_data = RemoteControlInit(&huart5);
#elifdef USE_OCD_CTRL
  robot->rc_data = VT13RemoteInit(&huart5);  // Assuming VT13RemoteInit on H7
#endif
#endif
#endif
#if !defined(GIMBAL_BOARD)
  robot->referee_data = RefereeInit(&huart7);  // 裁判系统初始化
  robot->chassis = ChassisInit(&chassis_init_config);
#endif
#if !defined(ONE_BOARD)
  RobotCommInit(robot);
#endif
  chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  chassis_ctrl_cmd->leg_length = chassis_init_config.param.initial_leg_length;  // 初始腿长
#if defined(GIMBAL_BOARD)
  gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
#endif
  DWT_GetDeltaT(&robot->DWT_CNT);
  // chassis_ctrl_cmd->max_power = 60;  // 测试用
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
  float raw_vx = chassis_ctrl_cmd->vx;
  float raw_yaw = chassis_ctrl_cmd->target_yaw;

  ChassisTask();

  // 恢复原始指令，防止对后续积分逻辑造成干扰
  chassis_ctrl_cmd->vx = raw_vx;
  chassis_ctrl_cmd->target_yaw = raw_yaw;
#endif
}
