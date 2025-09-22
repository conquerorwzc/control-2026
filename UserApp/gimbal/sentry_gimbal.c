#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *main_yaw_motor,*right_yaw_motor,*right_pitch_motor,*left_yaw_motor,*left_pitch_motor;

static Publisher_t *sentry_gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *sentry_gimbal_sub;                  // cmd控制消息订阅者
static Sentry_Gimbal_Upload_Data_s sentry_gimbal_feedback_data; // 回传给cmd的云台状态信息
static Sentry_Gimbal_Ctrl_Cmd_s sentry_gimbal_cmd_recv;         // 来自cmd的控制信息

// static BMI088Instance *bmi088; // 云台IMU
void SentryGimbalInit()
{
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s main_yaw_config = {
        .fdcan_init_config = {
            .can_handle = &hfdcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.3, // 8
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5,

                .MaxOut = 20,
            },
            .speed_PID = {
                .Kp = 6000 , // 50
                .Ki = 100, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 12000,
                .MaxOut = 25000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};

  // RIGHT_YAW
  Motor_Init_Config_s right_yaw_config = {
    .fdcan_init_config = {
      .can_handle = &hfdcan1,
      .tx_id = 2,
  },
  .controller_param_init_config = {
      .angle_PID = {
        .Kp = 0, // 8
        .Ki = 0,
        .Kd = 0,
        .DeadBand = 0,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 0,

        .MaxOut = 0,
    },
    .speed_PID = {
        .Kp = 0 , // 50
        .Ki = 0, // 200
        .Kd = 0,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 12000,
        .MaxOut = 25000,
    },
},
.controller_setting_init_config = {
      .angle_feedback_source = MOTOR_FEED,
      .speed_feedback_source = MOTOR_FEED,
      .outer_loop_type = ANGLE_LOOP,
      .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
      .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
  },
  .motor_type = GM6020};

  // RIGHT_YAW
  Motor_Init_Config_s left_yaw_config = {
    .fdcan_init_config = {
      .can_handle = &hfdcan1,
      .tx_id = 2,
  },
  .controller_param_init_config = {
      .angle_PID = {
        .Kp = 0, // 8
        .Ki = 0,
        .Kd = 0,
        .DeadBand = 0,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 0,

        .MaxOut = 0,
    },
    .speed_PID = {
        .Kp = 0 , // 50
        .Ki = 0, // 200
        .Kd = 0,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 12000,
        .MaxOut = 25000,
    },
},
.controller_setting_init_config = {
      .angle_feedback_source = MOTOR_FEED,
      .speed_feedback_source = MOTOR_FEED,
      .outer_loop_type = ANGLE_LOOP,
      .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
      .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
  },
  .motor_type = GM6020};

    // RIGHT_PITCH
    Motor_Init_Config_s right_pitch_config = {
      .fdcan_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 2,
    },
    .controller_param_init_config = {
        .angle_PID = {
          .Kp = 0, // 8
          .Ki = 0,
          .Kd = 0,
          .DeadBand = 0,
          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
          .IntegralLimit = 0,

          .MaxOut = 0,
      },
      .speed_PID = {
          .Kp = 0 , // 50
          .Ki = 0, // 200
          .Kd = 0,
          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
          .IntegralLimit = 12000,
          .MaxOut = 25000,
      },
  },
  .controller_setting_init_config = {
        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    },
    .motor_type = GM6020};

  // LEFT_PITCH
  Motor_Init_Config_s left_pitch_config = {
    .fdcan_init_config = {
      .can_handle = &hfdcan1,
      .tx_id = 2,
  },
  .controller_param_init_config = {
      .angle_PID = {
        .Kp = 0, // 8
        .Ki = 0,
        .Kd = 0,
        .DeadBand = 0,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 0,

        .MaxOut = 0,
    },
    .speed_PID = {
        .Kp = 0 , // 50
        .Ki = 0, // 200
        .Kd = 0,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 12000,
        .MaxOut = 25000,
    },
},
.controller_setting_init_config = {
      .angle_feedback_source = MOTOR_FEED,
      .speed_feedback_source = MOTOR_FEED,
      .outer_loop_type = ANGLE_LOOP,
      .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
      .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
  },
  .motor_type = GM6020};
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    main_yaw_motor = DJIMotorInit(&main_yaw_config);
    right_yaw_motor=DJIMotorInit(&right_yaw_config);
    right_pitch_motor = DJIMotorInit(&right_pitch_config);
    left_yaw_motor = DJIMotorInit(&left_yaw_config);
    left_yaw_motor = DJIMotorInit(&left_pitch_config);

    sentry_gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    sentry_gimbal_sub = SubRegister("gimbal_cmd", sizeof(Sentry_Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void Sentry_GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(sentry_gimbal_sub, &sentry_gimbal_cmd_recv);

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (sentry_gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(main_yaw_motor);
        DJIMotorStop(right_yaw_motor);
        DJIMotorStop(right_pitch_motor);
        DJIMotorStop(left_yaw_motor);
        DJIMotorStop(left_yaw_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(main_yaw_motor);
        DJIMotorEnable(right_yaw_motor);
        DJIMotorEnable(right_pitch_motor);
        DJIMotorEnable(left_yaw_motor);
        DJIMotorEnable(left_yaw_motor);
        DJIMotorChangeFeed(main_yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(main_yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(right_yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(right_yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(right_pitch_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(right_pitch_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(left_yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(left_yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(left_pitch_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(left_pitch_motor, SPEED_LOOP, MOTOR_FEED);
        DJMotorPIDCal(main_yaw_motor, sentry_gimbal_cmd_recv.main_yaw); // mian_yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJMotorPIDCal(right_yaw_motor, sentry_gimbal_cmd_recv.right_yaw);
        DJMotorPIDCal(right_pitch_motor, sentry_gimbal_cmd_recv.right_yaw);
        DJMotorPIDCal(left_yaw_motor, sentry_gimbal_cmd_recv.left_yaw);
        DJMotorPIDCal(left_pitch_motor, sentry_gimbal_cmd_recv.left_pitch);
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(main_yaw_motor);
        DJIMotorEnable(right_yaw_motor);
        DJIMotorEnable(right_pitch_motor);
        DJIMotorEnable(left_yaw_motor);
        DJIMotorEnable(left_yaw_motor);
        DJIMotorChangeFeed(main_yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(main_yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(right_yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(right_yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(right_pitch_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(right_pitch_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(left_yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(left_yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(left_pitch_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(left_pitch_motor, SPEED_LOOP, MOTOR_FEED);
        DJMotorPIDCal(main_yaw_motor, sentry_gimbal_cmd_recv.main_yaw); // mian_yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJMotorPIDCal(right_yaw_motor, sentry_gimbal_cmd_recv.right_yaw);
        DJMotorPIDCal(right_pitch_motor, sentry_gimbal_cmd_recv.right_yaw);
        DJMotorPIDCal(left_yaw_motor, sentry_gimbal_cmd_recv.left_yaw);
        DJMotorPIDCal(left_pitch_motor, sentry_gimbal_cmd_recv.left_pitch);
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    sentry_gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    sentry_gimbal_feedback_data.mian_yaw_motor_single_round_angle = main_yaw_motor->measure.angle_single_round;
    sentry_gimbal_feedback_data.right_yaw_motor_angle = right_yaw_motor->measure.total_angle;
    sentry_gimbal_feedback_data.right_pitch_motor_angle = right_yaw_motor->measure.total_angle;
    sentry_gimbal_feedback_data.left_yaw_motor_angle = right_yaw_motor->measure.total_angle;
    sentry_gimbal_feedback_data.left_pitch_motor_angle = right_yaw_motor->measure.total_angle;

    // 推送消息
    PubPushMessage(sentry_gimbal_pub, (void *)&sentry_gimbal_feedback_data);
}