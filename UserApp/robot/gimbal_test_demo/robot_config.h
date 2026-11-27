#ifndef CONTROL_2026_ROBOT_CONFIG_H
#define CONTROL_2026_ROBOT_CONFIG_H

#include "gimbal.h"
#include "shoot.h"

#define BOARD_TX_ID 0x10
#define BOARD_RX_ID 0x11

// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 5326
#define PITCH_HORIZON_ECD 5748  // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 22.0f   // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -27.0f  // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE 296.5 //hero的计算比较特殊，直接从读出来
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)  // pitch水平时电机的角度,0-360
#define GYRO2GIMBAL_DIR_YAW 1    // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1  // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1   // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

static Gimbal_Init_Config_s gimbal_init_config = {
    .yaw_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 1.0f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .DeadBand = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 100.0f,
                            .MaxOut = 500.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 1200.0f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .DeadBand = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5000.0f,
                            .MaxOut = 20000.0f,
                        },
                },
            .controller_setting_init_config =
                {
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                },
            .motor_type = GM6020,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 6,
                },
        },
  .pitch_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 3.0f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .DeadBand = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 100.0f,
                            .MaxOut = 500.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 1200.0f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .DeadBand = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5000.0f,
                            .MaxOut = 20000.0f,
                        },
                },
            .controller_setting_init_config =
                {
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                },
            .motor_type = GM6020,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 2,
                },
        },
  .imu_init_config = {
      .flag = 1,
      .offset_flag=1,
      .scale = {1.0f, 1.0f, 1.0f},
      .Yaw = 90.0f,
      .Pitch = 0.0f,
      .Roll = 0.0f,
      .GyroOffset = {0.00142297475f,0.00189829199f,-0.0019042585f},
    },
  .pitch_feedforward_scale = 0.0f,
};

#define FRICTION_MOTOR_CONFIG(handle, id, direction) \
  ((Motor_Init_Config_s){                            \
      .controller_param_init_config =                \
          {                                          \
              .speed_PID =                           \
                  {                                  \
                      .Kp = 2.0f,                    \
                      .Ki = 0.05f,                   \
                      .Kd = 0.01f,                   \
                      .Improve = PID_Integral_Limit, \
                      .IntegralLimit = 10000.0f,     \
                      .MaxOut = 15000.0f,            \
                  },                                 \
          },                                         \
      .controller_setting_init_config =              \
          {                                          \
              .angle_feedback_source = MOTOR_FEED,   \
              .speed_feedback_source = MOTOR_FEED,   \
              .outer_loop_type = SPEED_LOOP,         \
              .close_loop_type = SPEED_LOOP,         \
              .motor_reverse_flag = direction,       \
              .feedback_reverse_flag = direction,    \
          },                                         \
      .motor_type = M3508,                           \
      .can_init_config =                             \
          {                                          \
              .can_handle = handle,                  \
              .tx_id = id,                           \
          },                                         \
  })

static Shoot_Init_Config_s shoot_init_config = {
    .shoot_param =
        {
            .one_bullet_delta_angle = 60.0f,              // 一发弹丸拨盘转动角度(输出侧)
            .reduction_ratio_loader = 36.0f,             //  拨盘减速比
            .num_per_circle = 6,                          // 拨盘一圈装载量(模块不读, 仅注释)
            .loader_direction = -1,                       // 拨盘方向, 1 正 -1 反
            .friction_num = 3,                            // 摩擦轮数量(模块不读, 以 FRICTION_NUM 为准)
            .friction_speed = 5000.0f,                    //  首次调试先用 5000, 确认方向后改 26500
            .friction_speed_min = 4000.0f,                //
            .friction_speed_max = 55000.0f,               //
            .friction_coefficients = {1.0f, -1.0f, 1.1f}, // 三轮: 第2个反向, 第3个快10%配平
            .deadtime_burstfire = 500,                    // 连发间隔 2 发/秒
            .deadtime_onebullet = 1000,                   // 单发间隔 1 发/秒
            .target_speed = 11.5f,                        // 目标弹速(注意: 闭环门槛写死 16 m/s, 见说明)
            .bullet_speed_deadband = 0.3f,                // (模块写死 0.4, 此字段无效)
            .bullet_speed_adjustment = 400.0f,            // 弹速误差 → 转速增益 aps/(m/s)
            .one_barrel_heat_value = 100,                 // 一发消耗热量
            .shooter_barrel_cooling_value = 24,           // 每秒冷却
            .shooter_barrel_heat_limit = 200,             // 热量上限
        },
    /* 三轮全部挂在 CAN2 (PB5/PB6), ID 1/2/4 空闲 */
    .friction_motor_config[0] = FRICTION_MOTOR_CONFIG(&hcan1, 4, MOTOR_DIRECTION_NORMAL),
    .friction_motor_config[1] = FRICTION_MOTOR_CONFIG(&hcan1, 5, MOTOR_DIRECTION_NORMAL),
    .friction_motor_config[2] = FRICTION_MOTOR_CONFIG(&hcan2, 4, MOTOR_DIRECTION_NORMAL),

    .loader_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 35.0f,
                            .Ki = 0.0f,
                            .Kd = 0.008f,
                            .MaxOut = 40000.0f,       // 角度环输出=速度环目标上限(aps)
                        },
                    .speed_PID =
                        {
                            .Kp = 1.8f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .Improve = PID_Integral_Limit | PID_ErrorHandle,  // ErrorHandle: 卡弹自动反转
                            .IntegralLimit = 7000.0f,
                            .MaxOut = 15000.0f,       // 电流指令上限(<16384 满量程)
                        },
                },
            .motor_type = M2006,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 3,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .controller_setting_init_config.angle_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.speed_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.outer_loop_type = ANGLE_LOOP,
            .controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP,
        },
};

static CANComm_Init_Config_s comm_config = {
  .recv_data_len = 24,        // 接收数据长度，根据实际需求调整
  .send_data_len = 24,        // 发送数据长度，根据实际需求调整
  .daemon_count = 1000,      // 看门狗重载计数，根据实际需求调整
  .can_config = {
    .can_handle = &hcan1,  // 假设使用CAN1，根据实际使用的CAN句柄调整
    .tx_id = BOARD_TX_ID,        // 发送ID，根据实际需求调整
    .rx_id = BOARD_RX_ID,        // 接收ID，根据实际需求调整
    .id = NULL                   // 将在CANCommInit中设置
  }
};

// CAN实例配置（用于数据存储）
static CANInstance board_can_comm_data = {
  .can_handle = &hcan1,
  .tx_id = BOARD_TX_ID,          // 与comm_config中的ID保持一致
  .rx_id = BOARD_RX_ID,
  .txconf = {
    .StdId = BOARD_TX_ID,      // 发送ID
    .IDE = CAN_ID_STD,   // 标准帧
    .RTR = CAN_RTR_DATA, // 数据帧
    .DLC = 0x08,         // 数据长度8字节
  }
};
#endif  // CONTROL_2026_ROBOT_CONFIG_H
