/**
 ******************************************************************************
 * @file    robot_config.h
 * @brief   飞镖机器人(dart)的硬件配置与机构参数
 ******************************************************************************
 * @attention
 * 带 TODO 的参数必须按实车实测后修改, 否则可能出现方向相反/顶死/异常停机等问题。
 ******************************************************************************
 * 硬件一览(全部挂在 CAN1):
 *   yaw 电机      M2006 + C610   ID 1
 *   同步带电机(左) M3508 + C620   ID 2
 *   扳机位置电机   M3508 + C620   ID 3
 *   同步带电机(右) M3508 + C620   ID 4
 *   拉扳机舵机     PWM 舵机        TIM1_CH1 (PE9)
 ******************************************************************************
 */
#pragma once

#include "dart_shoot.h"

// 编译warning, 提醒开发者检查机器人参数
#ifndef DART_CONFIG_PARAM_WARNING
#define DART_CONFIG_PARAM_WARNING
#pragma message \
    "check if you have configured the parameters in dart/robot_config.h, IF NOT, please refer to the comments AND DO IT!"
#endif

/* ================= 开发板类型定义(只能存在一个) ================= */
#define ONE_BOARD  // 飞镖机器人只有一块主控板
#if defined(ONE_BOARD) && defined(CHASSIS_BOARD)
#error Conflict board definition! You can only define one board type.
#endif

/* ================= 硬件 ID 配置(须按实车修改) ================= */
#define DART_YAW_MOTOR_ID 1      // yaw 2006, CAN1
#define DART_BELT_MOTOR_ID_L 2   // 同步带 3508(左), CAN1
#define DART_TRIGGER_MOTOR_ID 3  // 扳机位置调整 3508, CAN1
#define DART_BELT_MOTOR_ID_R 4   // 同步带 3508(右), CAN1
#define DART_SERVO_ID 0          // 拉扳机的 PWM 舵机(注册索引 0~6, 不能与其他舵机冲突)

/* ================= 舵机 PWM(定时器/通道) ================= */
// 当前工程已在 CubeMX 中把 PE9 配置为 SERVO(对应 TIM1_CH1, PWM 周期 20ms), 因此默认使用 TIM1_CH1;
// 注意舵机失能时 ServoStop() 会停掉整个 TIM1(该定时器当前只用于本舵机);
// 已占用的 PWM: TIM1_CH2~CH4(预留)、TIM4_CH3(蜂鸣器)、TIM10_CH1(IMU 加热)
#define DART_SERVO_TIM_HANDLE (&htim1)
#define DART_SERVO_TIM_CHANNEL TIM_CHANNEL_1

/* ================= 机构参数: 位置累加(同步带 / yaw) ================= */
// 摇杆只决定累加方向, 摇杆推多少无关; 累加速率 = *_pos_rate(电机总角度/秒) 乘上控制周期
// TODO 实测: 0x1 表示摇杆前推(右推)时同步带(yaw)的累加方向, 反了就改成 -1
#define DART_BELT_DIRECTION 1
#define DART_YAW_DIRECTION 1
// TODO 实测: 累加速率不要超过对应电机角度环的 MaxOut(即速度限幅), 否则目标会一直跑在前面追不上
#define DART_BELT_POS_RATE 2000.0f  // 同步带位置累加速率, 单位: 电机总角度/秒(3508 输出轴 2000/19.2 ≈ 104°/s)
#define DART_YAW_POS_RATE 1500.0f   // yaw 位置累加速率, 单位: 电机总角度/秒(2006 输出轴 1500/36 ≈ 42°/s)

/* ================= 机构参数: 软限位(相对进入 AIM 档时的位置) ================= */
// 累加式位置控制没有机械零点, 这里用"进入 AIM 档时的位置 ± 限幅"当作软限位保护机构;
// 置 0 表示不限幅, 顶到限位时日志会打印 "... target hit soft limit"
// TODO 实测: 注意 M3508/M2006 的编码器在减速箱前, 这里的单位都是转子侧角度
#define DART_BELT_TARGET_LIMIT 30000.0f  // 同步带软限位(≈ 输出轴 1560°, 按实际行程标定)
#define DART_YAW_TARGET_LIMIT 3600.0f    // yaw 软限位(2006 减速比 36, 3600 ≈ 输出轴 100°), 防止绕线

/* ================= 机构参数: 扳机位置电机(速度环) ================= */
#define DART_TRIGGER_SPEED_LIMIT 1000.0f  // 扳机位置电机速度限幅(度/秒)

/* ================= 机构参数: PWM 舵机 ================= */
// 舵机角度 -> 脉宽 -> 占空比 的映射由 dart_shoot 组件完成, 这里只需要给出两个档位的角度
// TODO 实测: 左拨杆中档 / 上档对应的舵机角度, 用 DartShootSetServoAngle() 或临时改这里的值扫描确定
#define DART_SERVO_ANGLE_MID 20.0f  // 中档角度(占位值, 待实测)
#define DART_SERVO_ANGLE_UP 80.0f   // 上档角度(占位值, 待实测)
#define DART_SERVO_MIN_ANGLE 0.0f   // 舵机机械最小角度
#define DART_SERVO_MAX_ANGLE 180.0f // 舵机机械最大角度
#define DART_SERVO_MIN_PULSE_S 0.0005f  // 最小角度对应脉宽(0.5ms)
#define DART_SERVO_MAX_PULSE_S 0.0025f  // 最大角度对应脉宽(2.5ms)
#define DART_SERVO_PERIOD_S 0.02f       // PWM 周期(50Hz)

/* ================= 遥控器参数 ================= */
#define RC_DEADZONE 50                    // 摇杆死区
#define DART_RC_STICK_MAX 660.0f          // 摇杆最大值(约 ±660)
#define DART_RC_DIR_THRESHOLD 300         // 位置累加方向判定阈值(超过该值才认为摇杆有方向指令)
#define DART_STATUS_LOG_PERIOD_MS 1000.0f // 状态日志周期

/**
 * @brief 同步带 3508 电机配置(位置环 + 速度环串级)
 *
 * @note  单位: 角度/位置用 measure.total_angle(度), 速度环参考值/反馈值为 measure.speed_aps(度/秒),
 *        角度环的输出是速度环的参考值, 所以角度环 MaxOut = 允许的最大速度(度/秒)
 */
#define DART_BELT_MOTOR_CONFIG(can_h, _id, _reverse)                                             \
  {                                                                                              \
      .motor_type = M3508,                                                                       \
      .can_init_config =                                                                         \
          {                                                                                      \
              .can_handle = can_h,                                                               \
              .tx_id = _id,                                                                      \
          },                                                                                     \
      .controller_setting_init_config =                                                          \
          {                                                                                      \
              .angle_feedback_source = MOTOR_FEED,                                               \
              .speed_feedback_source = MOTOR_FEED,                                               \
              .outer_loop_type = ANGLE_LOOP,                                                     \
              .close_loop_type = ANGLE_AND_SPEED_LOOP,                                           \
              .motor_reverse_flag = _reverse,                                                    \
              .feedback_reverse_flag = _reverse,                                                 \
          },                                                                                     \
      .controller_param_init_config =                                                            \
          {                                                                                      \
              .angle_PID =                                                                       \
                  {                                                                              \
                      .Kp = 20.0f,                                                               \
                      .Ki = 0.3f,                                                                \
                      .Kd = 0.0f,                                                                \
                      .MaxOut = 4000.0f,                                                         \
                      .IntegralLimit = 2000.0f,                                                  \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,                    \
                  },                                                                             \
              .speed_PID =                                                                       \
                  {                                                                              \
                      .Kp = 6.0f,                                                                \
                      .Ki = 0.05f,                                                               \
                      .Kd = 0.0f,                                                                \
                      .MaxOut = 16000.0f,                                                        \
                      .IntegralLimit = 6000.0f,                                                  \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,                    \
                  },                                                                             \
          },                                                                                     \
  }

/**
 * @brief yaw 2006 电机配置(位置环 + 速度环串级)
 *
 * @note  M2006 使用 C610 电调, 电流指令上限为 ±10000
 */
#define DART_YAW_MOTOR_CONFIG(can_h, _id, _reverse)                                              \
  {                                                                                              \
      .motor_type = M2006,                                                                       \
      .can_init_config =                                                                         \
          {                                                                                      \
              .can_handle = can_h,                                                               \
              .tx_id = _id,                                                                      \
          },                                                                                     \
      .controller_setting_init_config =                                                          \
          {                                                                                      \
              .angle_feedback_source = MOTOR_FEED,                                               \
              .speed_feedback_source = MOTOR_FEED,                                               \
              .outer_loop_type = ANGLE_LOOP,                                                     \
              .close_loop_type = ANGLE_AND_SPEED_LOOP,                                           \
              .motor_reverse_flag = _reverse,                                                    \
              .feedback_reverse_flag = _reverse,                                                 \
          },                                                                                     \
      .controller_param_init_config =                                                            \
          {                                                                                      \
              .angle_PID =                                                                       \
                  {                                                                              \
                      .Kp = 12.0f,                                                               \
                      .Ki = 0.2f,                                                                \
                      .Kd = 0.0f,                                                                \
                      .MaxOut = 2000.0f,                                                         \
                      .IntegralLimit = 1000.0f,                                                  \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,                    \
                  },                                                                             \
              .speed_PID =                                                                       \
                  {                                                                              \
                      .Kp = 4.0f,                                                                \
                      .Ki = 0.05f,                                                               \
                      .Kd = 0.0f,                                                                \
                      .MaxOut = 10000.0f,                                                        \
                      .IntegralLimit = 3000.0f,                                                  \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,                    \
                  },                                                                             \
          },                                                                                     \
  }

/**
 * @brief 扳机位置调整 3508 电机配置(单环速度)
 *
 * @note  只用于慢速调整扳机位置, 因此限幅较小; 摇杆没有输入时上层会直接失能该电机
 */
#define DART_TRIGGER_MOTOR_CONFIG(can_h, _id, _reverse)  \
  {                                                      \
      .motor_type = M3508,                               \
      .can_init_config =                                 \
          {                                              \
              .can_handle = can_h,                       \
              .tx_id = _id,                              \
          },                                             \
      .controller_setting_init_config =                  \
          {                                              \
              .angle_feedback_source = MOTOR_FEED,       \
              .speed_feedback_source = MOTOR_FEED,       \
              .outer_loop_type = SPEED_LOOP,             \
              .close_loop_type = SPEED_LOOP,             \
              .motor_reverse_flag = _reverse,            \
              .feedback_reverse_flag = _reverse,         \
          },                                             \
      .controller_param_init_config =                    \
          {                                              \
              .speed_PID =                               \
                  {                                      \
                      .Kp = 5.0f,                        \
                      .Ki = 0.0f,                        \
                      .Kd = 0.0f,                        \
                      .MaxOut = 8000.0f,                 \
                      .IntegralLimit = 3000.0f,          \
                      .Improve = PID_Integral_Limit,     \
                  },                                     \
          },                                             \
  }

/**
 * @brief 拉扳机的 PWM 舵机配置
 */
#define DART_TRIGGER_SERVO_CONFIG(_htim, _channel)  \
  {                                                 \
      .servo_type = PWM_Servo,                      \
      .pwm_init_config =                            \
          {                                         \
              .htim = _htim,                        \
              .channel = _channel,                  \
              .period = DART_SERVO_PERIOD_S,        \
              .dutyratio = 0.0f,                    \
          },                                        \
      .servo_id = DART_SERVO_ID,                    \
  }

/**
 * @brief dart_shoot 组件初始化配置
 *
 * @note  两个同步带电机固定在同一个同步带上, 使用同一个位置/速度参考, 只有右电机装配相反因此取反;
 *        同步带与 yaw 使用位置环, 扳机位置电机使用速度环。
 */
static DartShoot_Init_Config_s dart_shoot_init_config = {
    .belt_motor_config =
        {
            DART_BELT_MOTOR_CONFIG(&hcan1, DART_BELT_MOTOR_ID_L, MOTOR_DIRECTION_NORMAL),
            DART_BELT_MOTOR_CONFIG(&hcan1, DART_BELT_MOTOR_ID_R, MOTOR_DIRECTION_REVERSE),
        },
    .yaw_motor_config = DART_YAW_MOTOR_CONFIG(&hcan1, DART_YAW_MOTOR_ID, MOTOR_DIRECTION_NORMAL),
    .trigger_motor_config = DART_TRIGGER_MOTOR_CONFIG(&hcan1, DART_TRIGGER_MOTOR_ID, MOTOR_DIRECTION_NORMAL),
    .trigger_servo_config = DART_TRIGGER_SERVO_CONFIG(DART_SERVO_TIM_HANDLE, DART_SERVO_TIM_CHANNEL),
    .param =
        {
            .belt_direction = DART_BELT_DIRECTION,
            .yaw_direction = DART_YAW_DIRECTION,
            .belt_pos_rate = DART_BELT_POS_RATE,
            .yaw_pos_rate = DART_YAW_POS_RATE,
            .belt_target_limit = DART_BELT_TARGET_LIMIT,
            .yaw_target_limit = DART_YAW_TARGET_LIMIT,
            .trigger_speed_limit = DART_TRIGGER_SPEED_LIMIT,
            .servo_angle_mid = DART_SERVO_ANGLE_MID,
            .servo_angle_up = DART_SERVO_ANGLE_UP,
            .servo_angle_min = DART_SERVO_MIN_ANGLE,
            .servo_angle_max = DART_SERVO_MAX_ANGLE,
            .servo_min_pulse_s = DART_SERVO_MIN_PULSE_S,
            .servo_max_pulse_s = DART_SERVO_MAX_PULSE_S,
            .servo_period_s = DART_SERVO_PERIOD_S,
        },
};
