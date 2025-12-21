/**
 * @file    chassis.h
 * @author  Enhao Zhang
 * @date    2025/8/8
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   Parallel Wheel-Legged Chassis Module
 */
#include "chassis.h"

#include "arm_math.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "speed_observer.h"
#include "user_lib.h"

static ChassisInstance* chassis;
static LegInstance* leg[2];
static Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd;  // 声明但不初始化

// 中间变量
static float chassis_aver_v;
static float q2i_coeff;

// robot param
static float robot_weight;
static float track_width;
static float wheel_radius;
static float wheel_reduction_ratio;

static void ChassisCtrlUpdate() {
  chassis->roll_comp =
      PIDCalculate(&chassis->roll_PID, chassis->chassis_IMU->Roll * DEGREE_2_RAD, chassis->chassis_ctrl_cmd.roll);

  for (int i = 0; i < 2; i++) {
    leg[i]->update_flag.is_controlled = chassis->chassis_ctrl_cmd.vx != 0;
    // leg[i]->update_flag.is_controlled = 1;
    leg[i]->leg_ctrl_cmd.x_d_ref = chassis->chassis_ctrl_cmd.vx;
    leg[i]->leg_ctrl_cmd.length_ref =
        chassis->chassis_ctrl_cmd.leg_length -
        (float)(1 - 2 * i) * track_width * (chassis->chassis_ctrl_cmd.roll - chassis->chassis_IMU->Roll * DEGREE_2_RAD);
    LegCtrlUpdate(leg[i], chassis->chassis_IMU);
    float leg_force_ff =
        0.7f * 9.8f * robot_weight / 2.0f / mcos(leg[i]->state_var.theta);  // 不超过半边重力的一半(看机器）
    leg[i]->virtual_model.F += leg_force_ff - (float)(1 - 2 * i) * chassis->roll_comp;
    VAL_LIMIT(leg[i]->virtual_model.F, -500.0f, 500.0f);
    leg[i]->real_model.T -= (float)(1 - 2 * i) * chassis->chassis_ctrl_cmd.wz;
  }

  chassis->delta_theta_comp =
      PIDCalculate(&chassis->delta_theta_PID, leg[0]->state_var.theta - leg[1]->state_var.theta, 0);
  for (int i = 0; i < 2; i++) {
    leg[i]->virtual_model.Tp -= (float)(1 - 2 * i) * chassis->delta_theta_comp;
    JointTorqueUpdate(leg[i]);
  }
}

/**
 * @brief 倒地自启
 * @todo 没测过
 */
static void ChassisRecovery() {
  for (int i = 0; i < 2; i++) {
    DMMotorOuterLoop(leg[i]->joint_motor[0], ANGLE_LOOP);
    DMMotorOuterLoop(leg[i]->joint_motor[1], ANGLE_LOOP);
    DMMotorSetPIDRef(leg[i]->joint_motor[0], -0.1);
    DMMotorSetPIDRef(leg[i]->joint_motor[1], 0.1);
    leg[i]->real_model.Tp_1 = leg[i]->joint_motor[0]->motor_controller.final_output;
    leg[i]->real_model.Tp_2 = leg[i]->joint_motor[1]->motor_controller.final_output;
    leg[i]->update_flag.is_controlled = chassis->chassis_ctrl_cmd.vx != 0;
    // leg[i]->update_flag.is_controlled = 1;
    if (abs((leg[i]->joint_motor[0]->measure.position - (-0.1f))) <= 0.5f &&
        abs(leg[i]->joint_motor[1]->measure.position - (0.1f)) <= 0.5f) {
      leg[i]->leg_ctrl_cmd.x_d_ref = chassis->chassis_ctrl_cmd.vx;
      LegCtrlUpdate(leg[i], chassis->chassis_IMU);
      leg[i]->real_model.T -= (float)(1 - 2 * i) * chassis->chassis_ctrl_cmd.wz;
    } else {
      leg[i]->leg_ctrl_cmd.x_d_ref = chassis->chassis_ctrl_cmd.vx;
      LegCtrlUpdate(leg[i], chassis->chassis_IMU);
      leg[i]->real_model.T -= (float)(1 - 2 * i) * chassis->chassis_ctrl_cmd.wz;
      // leg[i]->real_model.T = 0;
    }
  }
}

/**
 * @brief 功率控制
 * @todo 有待模块化,djimotor也得改改
 */
static void PowerControl() {}

float ref = 0;
/**
 * @brief 预测电机功率并进行限制
 *
 */
static void LimitChassisOutput() {
  for (int i = 0; i < 2; i++) {
    VAL_LIMIT(leg[i]->real_model.Tp_1, -20.0f, 20.0f);
    VAL_LIMIT(leg[i]->real_model.Tp_2, -20.0f, 20.0f);
    // VAL_LIMIT(leg[i]->real_model.Tp_1, -3.0f, 3.0f);
    // VAL_LIMIT(leg[i]->real_model.Tp_2, -3.0f, 3.0f);
    VAL_LIMIT(leg[i]->real_model.T, -2.45f, 2.45f);
    DMMotorSetRef(leg[i]->joint_motor[0], leg[i]->real_model.Tp_1);
    DMMotorSetRef(leg[i]->joint_motor[1], leg[i]->real_model.Tp_2);
    // DMMotorSetRef(leg[0]->joint_motor[1], 0);
    // DMMotorSetRef(leg[0]->joint_motor[0], 0);
    // DMMotorSetRef(leg[1]->joint_motor[0], leg[1]->real_model.Tp_1);
    // DMMotorSetRef(leg[1]->joint_motor[1], leg[1]->real_model.Tp_2);
    // DMMotorSetRef(leg[i]->joint_motor[0], 0);
    // DMMotorSetRef(leg[i]->joint_motor[1], 0);
    DJIMotorSetRef(leg[i]->wheel_motor, leg[i]->real_model.T * q2i_coeff * (16384.0f / 20.0f));
    // DJIMotorSetRef(leg[0]->wheel_motor, 0);
    // DJIMotorSetRef(leg[i]->wheel_motor, ref);
    // DJIMotorSetRef(leg[i]->wheel_motor, 0);
  }
  // PowerControl();
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed() {
#if 1
  ObserverVarUpdate(leg[0], chassis->chassis_IMU);
  ObserverVarUpdate(leg[1], chassis->chassis_IMU);
  chassis_aver_v = (leg[0]->observer_var.vb + leg[1]->observer_var.vb) / 2.0f;
  xvEstimateKF_Update(&chassis->vaEstimateKF, chassis->chassis_IMU->MotionAccel_n[1], chassis_aver_v);
  for (int i = 0; i < 2; i++) {
    leg[i]->state_var.x_d = chassis->vaEstimateKF.FilteredValue[0];
  }
#else
#define X_D_FILTER_ALPHA 0.1f
  for (int i = 0; i < 2; i++) {
    // 计算当前速度值
    float current_x_d = -1 * (leg[0]->wheel_motor->measure.speed_aps + leg[1]->wheel_motor->measure.speed_aps) / 2 /
                        wheel_reduction_ratio * DEGREE_2_RAD * wheel_radius;
    // 应用一阶低通滤波
    leg[i]->state_var.x_d = leg[i]->state_var.x_d * (1.0f - X_D_FILTER_ALPHA) + current_x_d * X_D_FILTER_ALPHA;
  }
#endif
}

ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config) {
  ChassisInstance* chassis_instance = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));

  chassis_instance->leg[0] = LegInit(&chassis_init_config->leg_init_config[0]);
  chassis_instance->leg[1] = LegInit(&chassis_init_config->leg_init_config[1]);

  robot_weight = chassis_init_config->chassis_param.robot_weight;
  track_width = chassis_init_config->chassis_param.track_width;
  wheel_radius = chassis_init_config->leg_init_config[0].leg_param.wheel_radius;
  wheel_reduction_ratio = chassis_init_config->leg_init_config[0].leg_param.wheel_reduction_ratio;
  q2i_coeff = (3591.0f / 187.0f) / wheel_reduction_ratio / 0.3f;

  PIDInit(&chassis_instance->delta_theta_PID, &chassis_init_config->delta_theta_PID_config);
  PIDInit(&chassis_instance->roll_PID, &chassis_init_config->roll_PID_config);

  chassis_instance->chassis_IMU = INS_Init(&chassis_init_config->imu_init_config);

  xvEstimateKF_Init(&chassis_instance->vaEstimateKF);

  chassis = chassis_instance;
  leg[0] = chassis->leg[0];
  leg[1] = chassis->leg[1];
  chassis_ctrl_cmd = &chassis->chassis_ctrl_cmd;  // 在运行时初始化指针
  return chassis_instance;
}

/* 机器人底盘控制核心任务 */
void ChassisTask() {
  if (chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_POWER_OFF) {
    // 如果出现重要模块离线或遥控器设置为急停,让电机停止
    for (int i = 0; i < 2; i++) {
      DMMotorStop(chassis->leg[i]->joint_motor[0]);
      DMMotorStop(chassis->leg[i]->joint_motor[1]);
      DJIMotorStop(chassis->leg[i]->wheel_motor);
    }
  } else {
    // 正常工作
    for (int i = 0; i < 2; i++) {
      DMMotorEnable(chassis->leg[i]->joint_motor[0]);
      DMMotorEnable(chassis->leg[i]->joint_motor[1]);
      DJIMotorEnable(chassis->leg[i]->wheel_motor);
    }
  }

  // 根据电机的反馈速度和IMU(如果有)计算真实速度
  EstimateSpeed();

  switch (chassis->chassis_ctrl_cmd.chassis_mode) {
    case CHASSIS_RECOVERY:
      ChassisRecovery();
      break;
    case CHASSIS_ON:
      ChassisCtrlUpdate();
      break;
    default:
      break;
  }

  // 功率控制与输出限幅
  LimitChassisOutput();
}