/**
 ******************************************************************************
 * @file    chassis.c
 * @author  Enhao Zhang
 * @date    2025/8/8
 * @brief   chassis control for parallel wheel-legged robot
 ******************************************************************************
 * @attention
 * None
 *
 ******************************************************************************
 */
#include "chassis.h"

#include "general_def.h"
#include "user_lib.h"

static ChassisInstance* chassis;
static Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd;  // 声明但不初始化

/**
 * @brief 根据控制命令计算每个腿的控制输出
 */
static void LegCalculate() {
  // 已在主任务中实现
}

/**
 * @brief 限制底盘输出
 */
static void LimitChassisOutput() {
  // 已在主任务中实现
}

/**
 * @brief 根据每个腿的反馈,计算底盘的实际运动状态
 */
static void EstimateSpeed() {
  // 根据电机速度和陀螺仪的角速度进行解算
  // chassis_feedback_data.vx vy wz =
}

ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config) {
  chassis = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));

  chassis->leg[0] = LegInit(&chassis_init_config.leg_init_config[0]);
  chassis->leg[1] = LegInit(&chassis_init_config.leg_init_config[1]);

  PIDInit(&chassis->delta_theta_PID, &chassis_init_config.delta_theta_PID_config);
  PIDInit(&chassis->roll_PID, &chassis_init_config.roll_PID_config);
  PIDInit(&chassis->yaw_PID, &chassis_init_config.yaw_PID_config);

  chassis->chassis_IMU_data = INS_Init();

  chassis_ctrl_cmd = &chassis->chassis_cmd_recv;  // 在运行时初始化指针
}

void ChassisTask() {
#ifdef ONE_BOARD
  SubGetMessage(chassis->chassis_sub, &chassis->chassis_cmd_recv);
#endif

  LegInstance* leg[2] = {chassis->leg[0], chassis->leg[1]};

  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_ZERO_FORCE) {
    // 如果出现重要模块离线或遥控器设置为急停,让电机停止
    for (int i = 0; i < 2; i++) {
      DMMotorStop(leg[i]->joint_motor[0]);
      DMMotorStop(leg[i]->joint_motor[1]);
      DMMotorStop(leg[i]->wheel_motor);
    }
  } else {
    // 正常工作
    for (int i = 0; i < 2; i++) {
      DMMotorEnable(leg[i]->joint_motor[0]);
      DMMotorEnable(leg[i]->joint_motor[1]);
      DMMotorEnable(leg[i]->wheel_motor);
    }
  }

  // 根据电机的反馈速度和IMU(如果有)计算真实速度
  EstimateSpeed();

  // 根据控制模式进行正运动学解算,计算底盘输出
  LegCalculate();

  chassis->roll_comp = PIDCalculate(&chassis->roll_PID, chassis->chassis_IMU_data->Gyro[1], chassis_ctrl_cmd->roll);
  chassis->yaw_comp = PIDCalculate(&chassis->yaw_PID, chassis->chassis_IMU_data->Gyro[2], chassis_ctrl_cmd->yaw);

  for (int i = 0; i < 2; i++) {
    LegControlUpdate(leg[i], chassis->chassis_IMU_data);
    float leg_force_ff = 9.8f * ROBOT_WEIGHT / 2.0f / leg[i]->state_var.theta;
    leg[i]->virtual_model.F += leg_force_ff + (float)(1 - 2 * i) * chassis->roll_comp;
    VAL_LIMIT(leg[i]->virtual_model.F, -100.0f, 100.0f);
  }
  chassis->delta_theta_comp =
      PIDCalculate(&chassis->delta_theta_PID, leg[0]->state_var.theta - leg[1]->state_var.theta, 0);
  for (int i = 0; i < 2; i++) {
    leg[i]->virtual_model.Tp += (float)(1 - 2 * i) * chassis->delta_theta_comp;
    leg[i]->real_model.T += (float)(1 - 2 * i) * chassis->yaw_comp;
    JointTorqueUpdate(leg[i]);
    VAL_LIMIT(leg[i]->real_model.Tp_1, -3.0f, 3.0f);
    VAL_LIMIT(leg[i]->real_model.Tp_2, -3.0f, 3.0f);
    VAL_LIMIT(leg[i]->real_model.T, -1.0f, 1.0f);
    // DMMotorSetRef(leg[i]->joint_motor[1], leg[i]->real_model.Tp_1);
    // DMMotorSetRef(leg[i]->joint_motor[0], leg[i]->real_model.Tp_2);
    // DMMotorSetRef(leg[i]->wheel_motor, leg[i]->real_model.T);
  }

  // 功率控制与输出限幅
  LimitChassisOutput();

#ifdef ONE_BOARD
  PubPushMessage(chassis->chassis_pub, (void*)&chassis->chassis_feedback_data);
#endif
}