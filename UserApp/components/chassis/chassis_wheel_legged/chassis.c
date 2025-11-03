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
#include "user_lib.h"

static ChassisInstance* chassis;
static Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd;  // 声明但不初始化

// robot param
static float robot_weight;

static void ChassisCtrlUpdate() {
  LegInstance* leg[2] = {chassis->leg[0], chassis->leg[1]};

  chassis->roll_comp =
      PIDCalculate(&chassis->roll_PID, chassis->chassis_IMU_data->Gyro[1], chassis->chassis_ctrl_cmd.roll);

  for (int i = 0; i < 2; i++) {
    LegCtrlUpdate(leg[i], chassis->chassis_IMU_data);
    float leg_force_ff = 9.8f * robot_weight / 2.0f / mcos(leg[i]->state_var.theta);
    leg[i]->virtual_model.F += leg_force_ff + (float)(1 - 2 * i) * chassis->roll_comp;
    VAL_LIMIT(leg[i]->virtual_model.F, -100.0f, 100.0f);
  }

  chassis->delta_theta_comp =
      PIDCalculate(&chassis->delta_theta_PID, leg[0]->state_var.theta - leg[1]->state_var.theta, 0);

  for (int i = 0; i < 2; i++) {
    leg[i]->virtual_model.Tp += (float)(1 - 2 * i) * chassis->delta_theta_comp;
    leg[i]->real_model.T += (float)(1 - 2 * i) * chassis->chassis_ctrl_cmd.wz;
    JointTorqueUpdate(leg[i]);
  }
}

static void ChassisRecovery() {
  LegInstance* leg[2] = {chassis->leg[0], chassis->leg[1]};
  for (int i = 0; i < 2; i++) {
    DMMotorOuterLoop(leg[i]->joint_motor[0], ANGLE_LOOP);
    DMMotorOuterLoop(leg[i]->joint_motor[1], ANGLE_LOOP);
    DMMotorPIDCal(leg[i]->joint_motor[0], 0);
    DMMotorPIDCal(leg[i]->joint_motor[1], 0);
    DMMotorPIDCal(leg[i]->wheel_motor, 0);
    LegCtrlUpdate(leg[i], chassis->chassis_IMU_data);
    leg[i]->real_model.T += (float)(1 - 2 * i) * chassis->chassis_ctrl_cmd.wz;
    leg[i]->real_model.Tp_1 = leg[i]->joint_motor[0]->motor_controller.final_output;
    leg[i]->real_model.Tp_2 = leg[i]->joint_motor[1]->motor_controller.final_output;
  }
}
/**
 * @brief 功率模型
 * @todo 有待模块化,djimotor也得改改
 */
static void PowerControl() {
  // 功率模型常量
  static const float k0 = 0.7441993412640775f;
  static const float k1 = 0.006444284468539646f;
  static const float k2 = 0.0001423857226262331f;
  static const float k3 = 0.015644430204543864f;
  static const float k4 = 0.1580143850678086f;
  static const float k5 = 2.896721772539512e-05f;

  // 获取电机速度反馈,化成单位rad/s
  float motor_speed_fdb[2];
  for (int i = 0; i < 2; i++) {
    motor_speed_fdb[i] = (float)chassis->leg[i]->wheel_motor->measure.velocity / 6.f;
  }

  // 获取当前电机参考电流，统一位单位为A
  float motor_current_list[2];
  for (int i = 0; i < 2; i++) {
    motor_current_list[i] = (float)chassis->leg[i]->wheel_motor->motor_controller.final_output;
  }

  float initial_give_power[2] = {0.0f};  // 每个电机的初始估计功率
  float initial_total_power = 0.0f;      // 估计初始总功率

  // 计算每个电机的功率贡献
  for (int i = 0; i < 2; i++) {
    initial_give_power[i] =
        k0 + k1 * motor_current_list[i] / (16384.0f / 20.0f) + k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
        k3 * motor_current_list[i] / (16384.0f / 20.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
        k4 * motor_current_list[i] / (16384.0f / 20.0f) * motor_current_list[i] / (16384.0f / 20.0f) +
        k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f);

    // 只累加正向功率
    if (initial_give_power[i] > 0) {
      initial_total_power += initial_give_power[i];
    }
  }
  // 功率超限时进行动态调整
  if (initial_total_power > (float)chassis_ctrl_cmd->max_power) {
    float power_scale = (float)chassis_ctrl_cmd->max_power / initial_total_power;  // 削减功率比例
    float scaled_give_power[2];
    // 计算缩放后的功率目标
    for (int i = 0; i < 2; i++) {
      scaled_give_power[i] = initial_give_power[i] * power_scale;
    }

    // 重新计算每个电机的电流参考值
    for (int i = 0; i < 2; i++) {
      // 二次方程系数计算，参数
      float a = k4 / (16384.0f / 20.0f) / (16384.0f / 20.0f);
      float b = k1 / (16384.0f / 20.0f) + k3 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) / (16384.0f / 20.0f);
      float c = k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
                k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) -
                scaled_give_power[i] + k0;
      float discriminant = b * b - 4 * a * c;  // 判别式
      if (discriminant >= 0) {
        float sqrt_disc = sqrtf(discriminant);
        float temp1 = (-b + sqrt_disc) / (2 * a);
        float temp2 = (-b - sqrt_disc) / (2 * a);

        // 选择最接近当前电流的解
        if (motor_current_list[i] > 0) {
          motor_current_list[i] = (fabsf(temp1 - motor_current_list[i]) < fabsf(temp2 - motor_current_list[i]))
                                      ? fminf(16000.f, temp1)
                                      : fminf(16000.f, temp2);
        } else {
          motor_current_list[i] = (fabsf(temp1 - motor_current_list[i]) < fabsf(temp2 - motor_current_list[i]))
                                      ? fmaxf(-16000.f, temp1)
                                      : fmaxf(-16000.f, temp2);
        }
      } else {
        // 无解时归零
        motor_current_list[i] = 0.0f;
      }
    }
  }
  for (int i = 0; i < 2; i++) {
    chassis->leg[i]->wheel_motor->motor_controller.final_output = (int16_t)(motor_current_list[i]);
  }
}

/**
 * @brief 预测电机功率并进行限制
 *
 */
static void LimitChassisOutput() {
  LegInstance* leg[2] = {chassis->leg[0], chassis->leg[1]};
  for (int i = 0; i < 2; i++) {
    VAL_LIMIT(leg[i]->real_model.Tp_1, -3.0f, 3.0f);
    VAL_LIMIT(leg[i]->real_model.Tp_2, -3.0f, 3.0f);
    VAL_LIMIT(leg[i]->real_model.T, -1.0f, 1.0f);
    // DMMotorSetRef(leg[i]->joint_motor[0], leg[i]->real_model.Tp_1);
    // DMMotorSetRef(leg[i]->joint_motor[1], leg[i]->real_model.Tp_2);
    // DMMotorSetRef(leg[i]->wheel_motor, leg[i]->real_model.T);
  }
  // PowerControl();
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed() {
  // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
  // chassis_feedback_data.vx vy wz =
  // DJIMotor得改otherfeed
}

ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config) {
  ChassisInstance* chassis_instance = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));
  chassis = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));

  chassis->leg[0] = LegInit(&chassis_init_config->leg_init_config[0]);
  chassis->leg[1] = LegInit(&chassis_init_config->leg_init_config[1]);

  robot_weight = chassis_init_config->chassis_param.robot_weight;

  PIDInit(&chassis->delta_theta_PID, &chassis_init_config->delta_theta_PID_config);
  PIDInit(&chassis->roll_PID, &chassis_init_config->roll_PID_config);

  chassis->chassis_IMU_data = INS_Init();

  chassis = chassis_instance;
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
      DMMotorStop(chassis->leg[i]->wheel_motor);
    }
  } else {
    // 正常工作
    for (int i = 0; i < 2; i++) {
      DMMotorEnable(chassis->leg[i]->joint_motor[0]);
      DMMotorEnable(chassis->leg[i]->joint_motor[1]);
      DMMotorEnable(chassis->leg[i]->wheel_motor);
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