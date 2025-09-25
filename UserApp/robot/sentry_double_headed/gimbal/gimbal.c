/**
 * @file gimbal.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author modified by SRM control team 2026
 * @brief gimbal for gimbal_double_headed
 *
 * @date 2025-09-26
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "gimbal.h"

#include "bmi088.h"
#include "dji_motor.h"
#include "general_def.h"
#include "ins_task.h"
#include "message_center.h"
#include "robot_def.h"

static attitude_t *gimbal_IMU_data;  // 云台IMU数据
static DJIMotorInstance *main_yaw_motor, *yaw_motor[2], *pitch_motor[2];

static Publisher_t *gimbal_pub;                    // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                   // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data;  // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;          // 来自cmd的控制信息

// static BMI088Instance *bmi088; // 云台IMU
void GimbalInit() {
  gimba_IMU_data = INS_Init();  // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
  // YAW
  Motor_Init_Config_s main_yaw_config = {
      .fdcan_init_config =
          {
              .can_handle = &hfdcan1,
              .tx_id = 2,
          },
      .controller_param_init_config =
          {
              .angle_PID =
                  {
                      .Kp = 1,  // 8
                      .Ki = 0,
                      .Kd = 0,
                      .DeadBand = 0.1,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .IntegralLimit = 5,

                      .MaxOut = 20,
                  },
              .speed_PID =
                  {
                      .Kp = 6000,  // 50
                      .Ki = 100,   // 200
                      .Kd = 0,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .IntegralLimit = 12000,
                      .MaxOut = 25000,
                  },
              .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
              // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
              .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = OTHER_FEED,
              .speed_feedback_source = OTHER_FEED,
              .outer_loop_type = ANGLE_LOOP,
              .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
              .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
          },
      .motor_type = GM6020};

  Motor_Init_Config_s yaw_config = {
      .controller_param_init_config =
          {
              .angle_PID =
                  {
                      .Kp = 1,  // 8
                      .Ki = 0,
                      .Kd = 0,
                      .DeadBand = 0.1,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .IntegralLimit = 5,

                      .MaxOut = 30,
                  },
              .speed_PID =
                  {
                      .Kp = 1000,  // 50
                      .Ki = 50,    // 200
                      .Kd = 0,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .IntegralLimit = 7000,
                      .MaxOut = 28000,
                  },
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,
              .outer_loop_type = ANGLE_LOOP,
              .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
              .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
          },
      .motor_type = GM6020,
  };

  Motor_Init_Config_s pitch_config = {
      .controller_param_init_config =
          {
              .angle_PID =
                  {
                      .Kp = 1.2,  // 8
                      .Ki = 0,
                      .Kd = 0,
                      .DeadBand = 0,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .IntegralLimit = 5,

                      .MaxOut = 30,
                  },
              .speed_PID =
                  {
                      .Kp = 2000,  // 50
                      .Ki = 100,   // 200
                      .Kd = 0,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .IntegralLimit = 7000,
                      .MaxOut = 28000,
                  },
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,
              .outer_loop_type = ANGLE_LOOP,
              .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
              .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
          },
      .motor_type = GM6020};
  // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
  main_yaw_motor = DJIMotorInit(&main_yaw_config);
  yaw_config.fdcan_init_config.can_handle = &hfdcan3;
  yaw_config.fdcan_init_config.tx_id = 4;
  yaw_motor[0] = DJIMotorInit(&yaw_config);  // LEFT
  yaw_config.fdcan_init_config.can_handle = &hfdcan2;
  yaw_config.fdcan_init_config.tx_id = 4;
  yaw_motor[1] = DJIMotorInit(&yaw_config);  // RIGHT
  pitch_config.fdcan_init_config.can_handle = &hfdcan3;
  pitch_config.fdcan_init_config.tx_id = 2;
  pitch_motor[0] = DJIMotorInit(&pitch_config);  // LEFT
  pitch_config.fdcan_init_config.can_handle = &hfdcan2;
  pitch_config.fdcan_init_config.tx_id = 2;
  pitch_motor[1] = DJIMotorInit(&pitch_config);  // RIGHT

  gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
  gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask() {
  // 获取云台控制数据
  // 后续增加未收到数据的处理
  SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

  // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
  // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
  switch (gimbal_cmd_recv.gimbal_mode) {
    // 停止
    case GIMBAL_ZERO_FORCE:
      DJIMotorStop(main_yaw_motor);
      for (int i = 0; i < 2; i++) {
        DJIMotorStop(yaw_motor[i]);
        DJIMotorStop(pitch_motor[i]);
      }
      break;

    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE:  // 后续只保留此模式
      DJIMotorEnable(main_yaw_motor);
      DJIMotorChangeFeed(main_yaw_motor, ANGLE_LOOP, OTHER_FEED);
      DJIMotorPIDCal(main_yaw_motor, gimbal_cmd_recv.main_yaw);  // mian_yaw和pitch会在robot_cmd中处理好多圈和单圈
      for (int i = 0; i < 2; i++) {
        DJIMotorEnable(yaw_motor[i]);
        DJIMotorEnable(pitch_motor[i]);
        DJIMotorChangeFeed(yaw_motor[i], ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_motor[i], SPEED_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_motor[i], ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_motor[i], SPEED_LOOP, MOTOR_FEED);
        DJIMotorPIDCal(yaw_motor[i], gimbal_cmd_recv.yaw[i]);
        DJIMotorPIDCal(pitch_motor[i], gimbal_cmd_recv.pitch[i]);
      }
      break;

    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE:  // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
      DJIMotorEnable(main_yaw_motor);
      DJIMotorChangeFeed(main_yaw_motor, ANGLE_LOOP, OTHER_FEED);
      DJIMotorPIDCal(main_yaw_motor, gimbal_cmd_recv.main_yaw);  // mian_yaw和pitch会在robot_cmd中处理好多圈和单圈
      for (int i = 0; i < 2; i++) {
        DJIMotorEnable(yaw_motor[i]);
        DJIMotorEnable(pitch_motor[i]);

        DJIMotorChangeFeed(yaw_motor[i], ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_motor[i], SPEED_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_motor[i], ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_motor[i], SPEED_LOOP, MOTOR_FEED);

        DJIMotorPIDCal(yaw_motor[i], gimbal_cmd_recv.right_yaw);
        DJIMotorPIDCal(pitch_motor[i], gimbal_cmd_recv.right_pitch);
      }
      break;

    default:
      break;
  }

  // 在合适的地方添加pitch重力补偿前馈力矩
  // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
  // ...

  // 设置反馈数据,主要是imu和yaw的ecd
  gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
  gimbal_feedback_data.mian_yaw_motor_single_round_angle = main_yaw_motor->measure.angle_single_round;
  for (int i = 0; i < 2; i++) {
    gimbal_feedback_data.right_yaw_motor_angle = yaw_motor[i]->measure.total_angle;
    gimbal_feedback_data.right_pitch_motor_angle = pitch_motor[i]->measure.total_angle;
  }

  // 推送消息
  PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
