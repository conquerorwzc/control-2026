#ifndef CONTROL_2026_ROBOT_CONFIG_H
#define CONTROL_2026_ROBOT_CONFIG_H

#include "gimbal.h"
#include "shoot.h"
#include "can_comm.h"
#include "HI05.h"
#define BOARD_TX_ID 0x10
#define BOARD_RX_ID 0x311

// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 5550
#define PITCH_HORIZON_ECD 5748  // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 13.0f   // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -30.0f  // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)
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
                      .Kp = 2.0f,  //0.8
                      .Ki = 0.0f,
                      .Kd = 0.035f,
                      .DeadBand = 0.0f,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .IntegralLimit = 5.0f,
                      .MaxOut = 25.0f,
                  },
                    .speed_PID =
                    {
                      .Kp = 5000.0f,   //6000
                      .Ki = 200.0f,   //10
                      .Kd = 0.0f,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .IntegralLimit = 12000.0f,
                      .MaxOut = 25000.0f, //25000
                  },

                },
            .motor_type = GM6020,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 1,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
  .pitch_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 0.55f,  // 1
                            .Ki = 0.00f,
                            .Kd = 0.00f,
                            .MaxOut = 20.0f,  //25
                            .DeadBand = 0.00f,
                         // .Derivative_LPF_RC=0.0085f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 0.0f,  //5
                        },
                    .speed_PID =
                        {
                            .Kp = 1.0f,  // 0.5
                            .Ki = 30.f,  // 0.1
                            .Kd = 0.00f,
                            .MaxOut = 11.5f,  //8
                            .DeadBand = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 1.5f,
                        },
                },
            .controller_setting_init_config =
                {
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .angle_feedback_source = OTHER_FEED,
                    .speed_feedback_source = OTHER_FEED,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                },
            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hcan2,
                    .tx_id = 0x01,      //0x01
                    .rx_id = 0x206,     //0x206Ò
                },
        },
  .imu_init_config = {
      .flag = 1,
      .offset_flag = 1,
      .scale = {1.0f, 1.0f, 1.0f},
      .Yaw = 0.0f,
      .Pitch = 0.0f,
      .Roll = 0.0f,
    .GyroOffset = {0.00253310893f, 0.00196733163f, 0.000239364381f},
  },
  //.hi05_uart_handle = &huart1,
};
//1.2 //0.2
#define FRICTION_MOTOR_CONFIG(handle, id, direction) \
((Motor_Init_Config_s) { \
.controller_param_init_config = { \
.speed_PID = { \
.Kp = 1.2f, \
.Ki = 0.2f, \
.Kd = 0.0f, \
.Improve = PID_Integral_Limit, \
.IntegralLimit = 1000.0f, \
.MaxOut = 15000.0f, \
}, \
}, \
.controller_setting_init_config ={\
.angle_feedback_source = MOTOR_FEED,\
.speed_feedback_source = MOTOR_FEED,\
.outer_loop_type = SPEED_LOOP,\
.close_loop_type = SPEED_LOOP,\
.motor_reverse_flag = direction,\
.feedback_reverse_flag = direction,\
},\
.motor_type = M3508, \
.can_init_config = { \
.can_handle = handle, \
.tx_id = id, \
}, \
})

static Shoot_Init_Config_s shoot_init_config = {
    .shoot_param =
        {
            .one_bullet_delta_angle = 90.0f,          // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出60
            .reduction_ratio_loader = 36.0f,         // 3508拨盘电机的减速比,舵轮
            .num_per_circle = 10,                      // 拨盘一圈的装载量6
            .loader_direction = 1,                    // 拨盘旋转方向,1为正向，-1为反向
            .friction_num = 2,                        //摩擦轮数量
            .friction_speed = 40000.0f,               //摩擦轮速度
            .friction_coefficients = {1.0f, -1.0f}, //摩擦轮速度比例系数
            .deadtime_burstfire = 50,
            .deadtime_onebullet = 200,               //弹丸发射间隔
            .target_speed = 24.0f,
            .bullet_speed_adjustment = 10.0f,
        },
    .friction_motor_config[0] = FRICTION_MOTOR_CONFIG(&hcan2, 8, MOTOR_DIRECTION_NORMAL),
    .friction_motor_config[1] = FRICTION_MOTOR_CONFIG(&hcan2, 4, MOTOR_DIRECTION_NORMAL),

    .loader_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 25.0f,   //40
                            .Ki = 0.0f,
                            .Kd = 0.0f,    //0.0
                            .MaxOut = 30000.0f,   //50000
                        },
                    .speed_PID =
                        {
                            .Kp = 2.0f,
                            .Ki = 0.4f,
                            .Kd = 0.0f,
                            .Improve = PID_Integral_Limit | PID_ErrorHandle,
                            .IntegralLimit = 5000.0f,    //7000
                            .MaxOut = 10000.0f,   //16000
                        },
                },
            .motor_type = M2006,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 7,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            // .controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .controller_setting_init_config.angle_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.speed_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.outer_loop_type = ANGLE_LOOP,
            .controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP,
        },
};

static CANComm_Init_Config_s comm_config = {
  .recv_data_len = 36,        // 接收数据长度，根据实际需求调整
  .send_data_len = 36,        // 发送数据长度，根据实际需求调整
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
#pragma pack(1)
typedef struct {
  float bullet_speed;
  uint16_t HP;
  uint16_t Heat;
} upload_data;
#pragma pack()
extern upload_data* upload;
#endif  // CONTROL_2026_ROBOT_CONFIG_H
