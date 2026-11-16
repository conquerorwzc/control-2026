
#ifndef CONTROL_2026_ROBOT_CONFIG_H
#define CONTROL_2026_ROBOT_CONFIG_H

#include "ins_task.h"
#include "tim.h"


#define ONE_BOARD
#define SERVO_FREQ      50.0f      //TIM1配的是50
#define SERVO_PERIOD    (1.0f / SERVO_FREQ)
#define SERVO_SPAN_MIN  500.0f     // 0°   对应的脉宽 (us)
#define SERVO_SPAN_MAX  2500.0f    //180° 对应的脉宽 (us)
#define SERVO_DUTY_MIN  (SERVO_SPAN_MIN / 1000000.0f * SERVO_FREQ)
#define SERVO_DUTY_MAX  (SERVO_SPAN_MAX / 1000000.0f * SERVO_FREQ)


#define SERVO_PWM_TIM   (&htim1)
#define SERVO_CH_NUM    4
#define SERVO_CH_TABLE  { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 }

 /*一行一个舵面: {left, zero, right}
  left  = 命令 0°   时实际给的舵机角
  zero  = 命令 90°  时的舵机角(中位)
  right = 命令 180° 时的舵机角*/
#define SERVO_LZR_TABLE                        \
  {                                            \
    {0.0f, 90.0f, 180.0f},   /* 舵面 1 */      \
    {0.0f, 88.0f, 180.0f},   /* 舵面 2 */      \
    {0.0f, 92.0f, 180.0f},   /* 舵面 3 */      \
    {0.0f, 98.0f, 180.0f},   /* 舵面 4 */      \
  }
//11.14对于第一个镖做的中位标定
#define WING_DIRS  {1, 1, 1, 1}   //舵面方向 1为正

// 舵面攻角上限 (deg), 参考 北理wing.h 的 ALPHA_MAX
#define ALPHA_MAX  10.0f

//飞行状态机, 参考 func/FSM_Config.h
#define STATIC_TO_SHOOTING_TIME_DECIDE                 5000.0f //ms
#define SHOOTING_TO_STATIC_TIME_DECIDE                 1000.0f //ms
#define SHOOTING_ACCELERATION_THRESHOLD                40.0f   //m/s^2 发射判据
#define SHOOTING_TO_STATIC_ACCELERATION_THRESHOLD      60.0f   //m/s^2 落地判据
#define SHOOTING_WEIGHTLESSNESS_ACCELERATION_THRESHOLD 2.0f    //m/s^2 失重判据

#define BENCH_UART      (&huart1)//调试串口
#define BENCH_BAUDRATE  115200

//角速度环 (用的队内 PID)
#define RATE_KP       0.3f //攻角(deg)/角速度误差(deg/s): 线性映射给到 0.3
#define RATE_KI       0.0f
#define RATE_KD       0.0f
#define RATE_I_LIMIT  5.0f
#define RATE_MAX_DPS  200.0f//攻角最大变化率(deg/s)貌似现在还没用上
#define RATE_LPF_ALPHA    0.1f //角速度一阶低通系数(0~1): 越小越平滑, 但延迟越大
#define RATE_SPIKE_DPS    300.0f //尖峰抑制(deg/s): 单帧超过它视为无效采样, 丢弃



//IMU配置
static IMU_Init_Config_s imu_config = {
    .flag = 1,
    .offset_flag = 0,
    .scale = {1.0f, 1.0f, 1.0f},
    .Yaw = 0.0f,
    .Pitch = 0.0f,
    .Roll = 0.0f,
    .CenterOffset = {0.0f, 0.0f, 0.0f},
};

#endif // CONTROL_2026_ROBOT_CONFIG_H
