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

#include "robot_def.h"
#include "user_lib.h"

static ChassisInstance* chassis;

static Chassis_Init_Config_s chassis_init_config = {
    .leg_init_config[0] = {.length_PID_config =
                               {
                                   .Kp = 0.01f,
                                   .Ki = 0.01f,
                                   .Kd = 0.01f,
                                   .MaxOut = 1000.0f,
                                   .DeadBand = 0.01f,
                                   .Improve = PID_IMPROVE_NONE,
                                   .IntegralLimit = 100.0f,
                               },
                           .length_d_PID_config =
                               {
                                   .Kp = 0.01f,
                                   .Ki = 0.01f,
                                   .Kd = 0.01f,
                                   .MaxOut = 1000.0f,
                                   .DeadBand = 0.01f,
                                   .Improve = PID_IMPROVE_NONE,
                                   .IntegralLimit = 100.0f,
                               },
                           .joint_motor_config[0] =
                               {
                                   .controller_setting_init_config =
                                       {
                                           .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                                           .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                                       },
                                   .motor_type = J4310,
                                   .fdcan_init_config =
                                       {
                                           .can_handle = &hfdcan1,
                                           .tx_id = 0x06,
                                           .rx_id = 0x03,
                                       },
                               },
                           .joint_motor_config[1] =
                               {
                                   .controller_setting_init_config =
                                       {
                                           .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                                           .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                                       },
                                   .motor_type = J4310,
                                   .fdcan_init_config =
                                       {
                                           .can_handle = &hfdcan1,
                                           .tx_id = 0x08,
                                           .rx_id = 0x04,
                                       },
                               },
                           .wheel_motor_config =
                               {
                                   .controller_setting_init_config =
                                       {
                                           .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                                           .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                                       },
                                   .motor_type = H6215,
                                   .fdcan_init_config =
                                       {
                                           .can_handle = &hfdcan1,
                                           .tx_id = 0x01,
                                           .rx_id = 0x00,
                                       },
                               }},
    .leg_init_config[1] =
        {
            .length_PID_config =
                {
                    .Kp = 0.01f,
                    .Ki = 0.01f,
                    .Kd = 0.01f,
                    .MaxOut = 1000.0f,
                    .DeadBand = 0.01f,
                    .Improve = PID_IMPROVE_NONE,
                    .IntegralLimit = 100.0f,
                },
            .length_d_PID_config =
                {
                    .Kp = 0.01f,
                    .Kd = 0.01f,
                    .Ki = 0.01f,
                    .MaxOut = 1000.0f,
                    .DeadBand = 0.01f,
                    .Improve = PID_IMPROVE_NONE,
                    .IntegralLimit = 100.0f,
                },
            .joint_motor_config[0] =
                {
                    .controller_setting_init_config =
                        {
                            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
                        },
                    .motor_type = J4310,
                    .fdcan_init_config =
                        {
                            .can_handle = &hfdcan2,
                            .tx_id = 0x08,
                            .rx_id = 0x04,
                        },
                },
            .joint_motor_config[1] =
                {
                    .controller_setting_init_config =
                        {
                            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
                        },
                    .motor_type = J4310,
                    .fdcan_init_config =
                        {
                            .can_handle = &hfdcan2,
                            .tx_id = 0x06,
                            .rx_id = 0x03,
                        },
                },
            .wheel_motor_config =
                {
                    .controller_setting_init_config =
                        {
                            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
                        },
                    .motor_type = H6215,
                    .fdcan_init_config =
                        {
                            .can_handle = &hfdcan2,
                            .tx_id = 0x01,
                            .rx_id = 0x00,
                        },
                },
        },
    .delta_theta_PID_config =
        {
            .Kp = 11.0f,
            .Ki = 0.2f,
            .Kd = 0.1f,
            .MaxOut = 2.0f,
            .DeadBand = 0.01f,
            .Improve = PID_IMPROVE_NONE,
            .IntegralLimit = 0.0f,
        },
    .roll_PID_config =
        {
            .Kp = 100.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .MaxOut = 100.0f,
            .DeadBand = 0.0f,
            .Improve = PID_IMPROVE_NONE,
            .IntegralLimit = 0.0f,
        },
    .yaw_PID_config =
        {
            .Kp = 2.0f,
            .Ki = 0.0f,
            .Kd = 0.2f,
            .MaxOut = 1.0f,
            .DeadBand = 0.01f,
            .Improve = PID_IMPROVE_NONE,
            .IntegralLimit = 0.0f,
        },
};

void ChassisInit() {
  chassis = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));

  chassis->leg[0] = LegInit(&chassis_init_config.leg_init_config[0]);
  chassis->leg[1] = LegInit(&chassis_init_config.leg_init_config[1]);

  PIDInit(&chassis->delta_theta_PID, &chassis_init_config.delta_theta_PID_config);
  PIDInit(&chassis->roll_PID, &chassis_init_config.roll_PID_config);
  PIDInit(&chassis->yaw_PID, &chassis_init_config.yaw_PID_config);

  chassis->chassis_IMU_data = INS_Init();

#ifdef ONE_BOARD  // 单板控制整车,则通过pubsub来传递消息
  chassis->chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
  chassis->chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif  // ONE_BOARD
}

void ChassisTask() {
#ifdef ONE_BOARD
  SubGetMessage(chassis->chassis_sub, &chassis->chassis_cmd_recv);
#endif

  LegInstance* leg[2] = {chassis->leg[0], chassis->leg[1]};

  if (chassis->chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
    // 如果出现重要模块离线或遥控器设置为急停,让电机停止
    DMMotorStop(leg[0]->joint_motor[0]);
    DMMotorStop(leg[0]->joint_motor[1]);
    DMMotorStop(leg[0]->wheel_motor);
    DMMotorStop(leg[1]->joint_motor[0]);
    DMMotorStop(leg[1]->joint_motor[1]);
    DMMotorStop(leg[1]->wheel_motor);
  } else {
    // 正常工作
    DMMotorEnable(leg[0]->joint_motor[0]);
    DMMotorEnable(leg[0]->joint_motor[1]);
    DMMotorEnable(leg[0]->wheel_motor);
    DMMotorEnable(leg[1]->joint_motor[0]);
    DMMotorEnable(leg[1]->joint_motor[1]);
    DMMotorEnable(leg[1]->wheel_motor);
  }

  chassis->roll_comp =
      PIDCalculate(&chassis->roll_PID, chassis->chassis_IMU_data->Gyro[1], chassis->chassis_cmd_recv.roll);
  chassis->yaw_comp =
      PIDCalculate(&chassis->yaw_PID, chassis->chassis_IMU_data->Gyro[2], chassis->chassis_cmd_recv.yaw);

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

#ifdef ONE_BOARD
  PubPushMessage(chassis->chassis_pub, (void*)&chassis->chassis_feedback_data);
#endif
}