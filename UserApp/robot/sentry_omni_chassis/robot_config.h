/**
******************************************************************************
* @file    robot_config.h
* @brief   robot配置模块实现文件，用于集中管理机器人配置与参数
******************************************************************************
*/
#pragma once

#include "robot.h"
#include "can_comm.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_CONFIG_PARAM_WARNING
#define ROBOT_CONFIG_PARAM_WARNING
#pragma message \
    "check if you have configured the parameters in robot_config.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define CHASSIS_BOARD  // 单板控制整车
// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) || \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

// 两种采集均独占原裁判 USART6，USART1 导航和 CAN 遥控保持工作。
// 0=正常裁判通信，1=原噪声采集，2=新增 PID 调参采集；不可混发。
#ifndef CHASSIS_VOFA_MODE
#define CHASSIS_VOFA_MODE 2
#endif
#if CHASSIS_VOFA_MODE < 0 || CHASSIS_VOFA_MODE > 2
#error Unsupported CHASSIS_VOFA_MODE
#endif
// 调参页：0=单电机，1=vx，2=vy，3=wz，4=yaw_hold，5=四电机，6=平移/wz，7=三轴同步，8=起停诊断。
#ifndef CHASSIS_TUNING_VOFA_VIEW
#define CHASSIS_TUNING_VOFA_VIEW 8
#endif
#if CHASSIS_TUNING_VOFA_VIEW < 0 || CHASSIS_TUNING_VOFA_VIEW > 8
#error Unsupported CHASSIS_TUNING_VOFA_VIEW
#endif
#ifndef CHASSIS_VOFA_MOTOR_ID
#define CHASSIS_VOFA_MOTOR_ID 1
#endif
#if CHASSIS_VOFA_MOTOR_ID < 1 || CHASSIS_VOFA_MOTOR_ID > 4
#error Unsupported CHASSIS_VOFA_MOTOR_ID
#endif
// PID 采集：460800 baud、4ms/250Hz，占 56.4%，低于无线透传 1Mbps 上限。
#ifndef CHASSIS_TUNING_VOFA_BAUDRATE
#define CHASSIS_TUNING_VOFA_BAUDRATE 460800u
#endif
#ifndef CHASSIS_TUNING_VOFA_PERIOD_MS
#define CHASSIS_TUNING_VOFA_PERIOD_MS 4
#endif
#if CHASSIS_TUNING_VOFA_PERIOD_MS < 1 || CHASSIS_TUNING_VOFA_BAUDRATE == 0 || \
    CHASSIS_TUNING_VOFA_BAUDRATE * CHASSIS_TUNING_VOFA_PERIOD_MS * 4u < 1040000u * 5u
#error CHASSIS_TUNING_VOFA bandwidth must leave at least 20 percent margin
#endif

// 物理参数：长度配置仍为 mm，进入运动学时统一转 m。
#define CHASSIS_WHEEL_RADIUS_MM 76.475f
#define CHASSIS_WHEEL_REDUCTION 19.0f
// 遥控原始通道 [-660, 660] 直接映射为 SI 目标速度，与轮径/减速比无关。
#define REMOTE_CHANNEL_MAX 660.0f
#define REMOTE_MAX_LINEAR_SPEED_MPS 2.0f  // vx/vy 各轴满量程；未对合速度限幅
#define REMOTE_SPIN_LOW_RADPS 1.5f
#define REMOTE_SPIN_HIGH_RADPS 5.0f
// 保持原跟随模式右摇杆的满量程角速度，独立于拨轮两档设置。
#define REMOTE_FOLLOW_MAX_WZ_RADPS 0.8092162f
// 航向采集：右摇杆每次越过触发阈值给一个角度阶跃，回中后可再次触发。
#define REMOTE_YAW_HOLD_STEP_DEG 10.0f
#define REMOTE_YAW_HOLD_TRIGGER 330
#define REMOTE_YAW_HOLD_REARM 100



//can通信任务初始化时间 单位ms
#define CAN_COMM_TASK_INIT_TIME 100
//双板can通信设备
#define BOARD_CAN hcan2
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 3845  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define PITCH_HORIZON_ECD 3494      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 30.0f   // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -30.0f  // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)  // pitch水平时电机的角度,0-360
#define GYRO2GIMBAL_DIR_YAW 1    // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1  // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1   // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

// 四电机共用速度环；2026-10-13 带载日志设计，平动响应优先，待实机验证。
#define WHEEL_MOTOR_CONFIG(handle, id) \
((Motor_Init_Config_s) { \
    .can_init_config = { \
        .can_handle = handle, \
        .tx_id = id, \
    }, \
    .controller_param_init_config = { \
        .speed_PID = { \
            .Kp = 2.2f, \
            .Ki = 4.0f, \
            .Kd = 0.0f, \
            .IntegralLimit = 5000.0f, \
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
            .MaxOut = 16000, \
        }, \
        .current_PID = { \
            .Kp = 0, \
            .Ki = 0, \
            .Kd = 0, \
            .IntegralLimit = 3000, \
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
            .MaxOut = 15000, \
        }, \
    }, \
    .controller_setting_init_config = { \
        .angle_feedback_source = MOTOR_FEED, \
        .speed_feedback_source = MOTOR_FEED, \
        .outer_loop_type = SPEED_LOOP, \
        .close_loop_type = SPEED_LOOP, \
        .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,\
        .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,\
    }, \
    .motor_type = M3508, \
})

static Chassis_Init_Config_s chassis_init_config = {
    // vx/vy 按用户约定共用纯 P=0.4；wz 增强平动抗偏航，保持零积分避免拖尾。
    .enable_speed_hold = 1,
    .vx_hold_pid = {
        .Kp = 0.4f, .Ki = 0.0f, .Kd = 0.0f, // 联合数据复核后增强 P，纯前后跟随候选，待复测
        .DeadBand = 0.01f,       // m/s
        .MaxOut = 0.5f,         // 速度修正上限 m/s，不是整车速度上限
        .IntegralLimit = 0.2f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    },
    .vy_hold_pid = {
        .Kp = 0.4f, .Ki = 0.0f, .Kd = 0.0f, // 按用户要求沿用 vx 参数，不单独整定
        .DeadBand = 0.01f,
        .MaxOut = 0.5f,
        .IntegralLimit = 0.2f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    },
    .wz_hold_pid = {
        // yaw2.csv：起停仍有反向偏转，小幅增强即时抑制；新值待起停页复测。
        .Kp = 0.9f, .Ki = 0.0f, .Kd = 0.0f,
        .DeadBand = 0.03f,       // rad/s；降低静止 gyro 噪声引起的速度修正
        .MaxOut = 1.5f,         // 角速度修正上限 rad/s，给增强后的 P 留出余量
        .IntegralLimit = 0.0f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    },
    .enable_yaw_hold = 1,
    .yaw_hold_pid = {
        .Kp = 0.12f, .Ki = 0.0f, .Kd = 0.0f, // 平动1.csv 复核：加强小角差纠偏；新值待复测
        .DeadBand = 0.3f,        // deg
        .MaxOut = 1.2f,          // 航向环角速度贡献上限 rad/s；10 deg 达到上限
        .IntegralLimit = 0.0f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    },
    // 2026-10-12 CSV 首轮整定：优先纯平动 vx/vy、纯转动 wz，保留机械运动加速度。
    // 以下均为方差/噪声密度；依据与限制见 speed_observer/noise_calibration/2026-10-12-tuned/整定说明.md。
    .observer_config = {
        .initial_variance = {0.25f, 0.25f, 0.25f},
        .process_noise = {0.025f, 0.035f, 1.5f},  // Q=q*dt；xy 采用稳健过程一致性，wz 保留起停变化
        .wheel_variance = {0.0007f, 0.0005f, 0.015f},
        .gyro_variance = 0.0002f,
        .accel_variance = {0.04f, 0.0002f}, // 新版匹配低通后的静止随机噪声，不含真实周期运动
        .max_dt = 0.05f,
    },
    .enable_imu_sampling = 1, // 开启 BMI088/INS 采集任务，采集开关与融合开关独立
    .observer_imu_calibrated = 1, // 已按实机验证结果启用 gyro_z 观测
    .observer_accel_enabled = 1,  // 已按实机验证结果启用去重力、杆臂补偿后的加速度预测
    .observer_imu_offset = {0.0f, -0.1335f}, // 中心 -> IMU：位于 -y（右侧）约 133.5 mm，单位 m
    .observer_sensor_timeout = 0.02f,
    .imu_init_config = {
        .flag = 1,
        .scale = {1.0f, 1.0f, 1.0f},
        .offset_flag = 0, // 启用 INS 时沿用启动静止零偏标定
        .Yaw = 180.0f, .Pitch = 0.0f, .Roll = 0.0f, // 原始 +X 朝后、+Y 朝右、+Z 朝上，校正为底盘系
        .CenterOffset = {0.0f, 0.0f, 0.0f}, // 杆臂由速度观测器适配层补偿，避免重复
    },
    .chassis_param =
        {
            // 机器人底盘修改的参数,单位为mm(毫米)
            .wheel_base = 345.96f,              // 纵向轴距(前进后退方向)
            .track_width = 345.96f,             // 横向轮距(左右平移方向)
            .center_gimbal_offset_x = 0.0f,    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
            .center_gimbal_offset_y = 0.0f,    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
            .wheel_radius = CHASSIS_WHEEL_RADIUS_MM,             // 轮子半径
            .wheel_reduction_ratio = CHASSIS_WHEEL_REDUCTION,  // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
            //3508功率模型参数
      .power_param.k0=0.7441993412640775f,
      .power_param.k1=0.0090164284468539646f,
      .power_param.k2=0.0001988857226262331f,
      .power_param.k3=0.024694430204543864f,
      .power_param.k4=0.20160143850678086f,
      .power_param.k5=3.715221772539512e-05f,
        },
    .wheel_motor_config[0] = WHEEL_MOTOR_CONFIG(&hcan1,1),
    .wheel_motor_config[1] = WHEEL_MOTOR_CONFIG(&hcan1,4),
    .wheel_motor_config[2] = WHEEL_MOTOR_CONFIG(&hcan1,2),
    .wheel_motor_config[3] = WHEEL_MOTOR_CONFIG(&hcan1,3),
    // 跟随角度环：输入度，输出 rad/s；保留此前已换算的实际增益与限幅。
    .follow_pid={
        .Kp = -0.08582596f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .DeadBand = 1.0f,
        .IntegralLimit = 1.2260851f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 24.5217023f,
    },


};

static SuperCap_Init_Config_s super_cap_config = {
  .can_config = {
    .can_handle = &hcan1,
    .tx_id = 0x210,  // 超级电容默认接收id
    .rx_id = 0x211,  // 超级电容默认发送id,注意tx和rx在其他人看来是反的
}};

#ifndef CONTROL_2026_ROBOT_CONFIG_H
#define CONTROL_2026_ROBOT_CONFIG_H
#if DEVICE_ROLE_TX
// 发送板配置
#define BOARD_TX_ID 0x218
#define BOARD_RX_ID 0x219
#else
// 接收板配置
#define BOARD_TX_ID 0x211
#define BOARD_RX_ID 0x210
#endif

static CANComm_Init_Config_s comm_config = {
  .recv_data_len = 30,        // 接收数据长度，根据实际需求调整
  .send_data_len = 30,        // 发送数据长度，根据实际需求调整
  .daemon_count = 10,      // 看门狗重载计数，根据实际需求调整
  .can_config = {
    .can_handle = &hcan2,  // 假设使用CAN1，根据实际使用的CAN句柄调整
    .tx_id = BOARD_TX_ID,        // 发送ID，根据实际需求调整
    .rx_id = BOARD_RX_ID,        // 接收ID，根据实际需求调整
    .id = NULL                   // 将在CANCommInit中设置
  }
};

// CAN实例配置（用于数据存储）
static CANInstance board_can_comm_data = {
  .can_handle = &BOARD_CAN,
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
