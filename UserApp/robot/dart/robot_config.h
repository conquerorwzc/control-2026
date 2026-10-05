/**
 ******************************************************************************
 * @file    robot_config.h
 * @brief   飞镖机器人(dart)的硬件配置与机构参数
 ******************************************************************************
 * @attention
 * 带 TODO 的参数必须按实车实测后修改, 否则可能出现拉不到位/堵转/异常停机等问题。
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
#define DART_PULL_MOTOR_ID_L 1   // 同步带 3508(左), CAN1
#define DART_PULL_MOTOR_ID_R 2   // 同步带 3508(右), CAN1
#define DART_TRIGGER_MOTOR_ID 3  // 扳机位置调整 3508, CAN1
#define DART_SERVO_ID 0          // 拉扳机的 PWM 舵机(注册索引 0~6, 不能与其他舵机冲突)

/* ================= 舵机 PWM(定时器/通道) ================= */
// 当前工程已在 CubeMX 中把 PE9 配置为 SERVO(对应 TIM1_CH1, PWM 周期 20ms), 因此默认使用 TIM1_CH1;
// 已占用的 PWM: TIM1_CH2~CH4(预留)、TIM4_CH3(蜂鸣器)、TIM10_CH1(IMU 加热)
#define DART_SERVO_TIM_HANDLE (&htim1)
#define DART_SERVO_TIM_CHANNEL TIM_CHANNEL_1

/* ================= 机构参数: 速度(单位 度/秒, 3508 输出轴空载约 2900 度/秒) ================= */
#define DART_PULL_DIRECTION 1              // TODO 实测: 拉滑块时同步带电机的转向(1 或 -1), 复位/回零方向取其反方向
#define DART_PULL_SPEED 2000.0f            // TODO 实测: 拉滑块速度
#define DART_RELOAD_SPEED 1200.0f          // TODO 实测: 复位(收回滑块)速度
#define DART_HOME_SPEED 500.0f             // TODO 实测: 回零速度, 建议低速
#define DART_PULL_SPEED_LIMIT 2500.0f      // 手动测试时同步带电机的速度限幅
#define DART_TRIGGER_SPEED_LIMIT 1000.0f   // 扳机位置调整电机的速度限幅

/* ================= 机构参数: 行程与到位判定 ================= */
// pull_travel 单位是电机总角度(measure.total_angle), 换算:
//   电机总角度 = 滑块行程(mm) / 同步带轮周长(mm) * 360 * 电机到同步带的减速比
//   注意 M3508 编码器在转子侧, 减速比约 19.2, 即输出轴转一圈约 19.2 * 360 = 6912(度)
#define DART_PULL_TRAVEL 30000.0f          // TODO 实测: 从起点到扳机位的行程
#define DART_PULL_TOLERANCE 500.0f         // 到位位置容差
#define DART_PULL_CURRENT_THRESHOLD 6000.0f  // 顶到扳机/限位的电流阈值(3508 反馈原始值, 约 ±16384 对应 ±20A)
#define DART_PULL_STALL_TIME_MS 200.0f     // 电流/堵转需要连续保持的时间
#define DART_PULL_TIMEOUT_MS 3000.0f       // 拉滑块超时
#define DART_HOME_TIMEOUT_MS 8000.0f       // 回零超时
#define DART_RELOAD_TIMEOUT_MS 4000.0f     // 复位超时

/* ================= 机构参数: PWM 舵机(拉扳机) ================= */
// 舵机角度 -> 脉宽 -> 占空比 的映射由 dart_shoot 组件完成, 这里只需要给出两个极限位置的脉宽
#define DART_SERVO_LOCK_ANGLE 20.0f        // TODO 实测: 锁止位(扳机未被拉下, 滑块被挡住)
#define DART_SERVO_RELEASE_ANGLE 80.0f     // TODO 实测: 释放位(扳机被拉下, 滑块飞出)
#define DART_SERVO_MIN_ANGLE 0.0f          // 舵机机械最小角度
#define DART_SERVO_MAX_ANGLE 180.0f        // 舵机机械最大角度
#define DART_SERVO_MIN_PULSE_S 0.0005f     // 最小角度对应脉宽(0.5ms)
#define DART_SERVO_MAX_PULSE_S 0.0025f     // 最大角度对应脉宽(2.5ms)
#define DART_SERVO_PERIOD_S 0.02f          // PWM 周期(50Hz)
#define DART_SERVO_RELEASE_TIME_MS 200.0f  // 击发时舵机保持在释放位的时间

/* ================= 测试程序参数(详见 robot.c 的说明) ================= */
#define RC_DEADZONE 50                   // 摇杆死区
#define DART_TEST_STICK_MAX 660.0f       // 摇杆最大值(约 ±660)
#define DART_TEST_CMD_THRESHOLD 300      // 触发单步指令(拉滑块/复位/击发/清异常)的摇杆阈值
#define DART_TEST_MODE_THRESHOLD 600     // 启动/停止自动循环测试的摇杆阈值
#define DART_TEST_SERVO_TRIM_STEP 0.03f  // 舵机角度微调步长(°/周期, 1kHz 下约 30°/s)
#define DART_TEST_LOG_PERIOD_MS 500.0f   // 手动模式状态日志周期
#define DART_TEST_AUTO_CYCLE_NUM 5       // 自动循环测试次数
#define DART_TEST_AUTO_MAX_FAIL 3        // 连续失败上限, 超过后停止自动循环测试

/* 摇杆边沿检测结果 */
#define DART_TEST_EDGE_NONE 0  // 无边沿
#define DART_TEST_EDGE_UP 1    // 上跳沿(摇杆向前/向右推到底)
#define DART_TEST_EDGE_DOWN 2  // 下跳沿(摇杆向后/向左推到底)

/**
 * @brief 同步带 3508 电机配置(单环速度)
 *
 * @note  单位: 速度环参考值/反馈值均为 度/秒(measure.speed_aps), PID 输出为 C620 电流指令(±16384)
 *        使能 PID_ErrorHandle: 电机被扳机/限位顶住时(速度误差持续 >95%) 会上报堵转,
 *        组件用它作为"已经拉到扳机位"的兜底判据。
 */
#define DART_PULL_MOTOR_CONFIG(can_h, _id, _reverse)                                               \
  {                                                                                                \
      .motor_type = M3508,                                                                         \
      .can_init_config =                                                                           \
          {                                                                                        \
              .can_handle = can_h,                                                                 \
              .tx_id = _id,                                                                        \
          },                                                                                       \
      .controller_setting_init_config =                                                            \
          {                                                                                        \
              .angle_feedback_source = MOTOR_FEED,                                                 \
              .speed_feedback_source = MOTOR_FEED,                                                 \
              .outer_loop_type = SPEED_LOOP,                                                       \
              .close_loop_type = SPEED_LOOP,                                                       \
              .motor_reverse_flag = _reverse,                                                      \
              .feedback_reverse_flag = _reverse,                                                   \
          },                                                                                       \
      .controller_param_init_config =                                                              \
          {                                                                                        \
              .speed_PID =                                                                         \
                  {                                                                                \
                      .Kp = 6.0f,                                                                  \
                      .Ki = 0.05f,                                                                 \
                      .Kd = 0.0f,                                                                  \
                      .MaxOut = 16000.0f,                                                          \
                      .IntegralLimit = 6000.0f,                                                    \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_ErrorHandle,   \
                  },                                                                               \
          },                                                                                       \
  }

/**
 * @brief 扳机位置调整 3508 电机配置(单环速度), 只用于慢速调整扳机位置, 因此限幅较小
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
 * @note  两个同步带电机固定在同一个同步带上, 使用同一速度参考, 仅转向可以按实车装配取反;
 *        三个 3508 全部使用速度环。
 */
static DartShoot_Init_Config_s dart_shoot_init_config = {
    .pull_motor_config =
        {
            DART_PULL_MOTOR_CONFIG(&hcan1, DART_PULL_MOTOR_ID_L, MOTOR_DIRECTION_NORMAL),
            DART_PULL_MOTOR_CONFIG(&hcan1, DART_PULL_MOTOR_ID_R, MOTOR_DIRECTION_REVERSE),
        },
    .trigger_motor_config = DART_TRIGGER_MOTOR_CONFIG(&hcan1, DART_TRIGGER_MOTOR_ID, MOTOR_DIRECTION_NORMAL),
    .trigger_servo_config = DART_TRIGGER_SERVO_CONFIG(DART_SERVO_TIM_HANDLE, DART_SERVO_TIM_CHANNEL),
    .param =
        {
            .pull_direction = DART_PULL_DIRECTION,
            .pull_speed = DART_PULL_SPEED,
            .reload_speed = DART_RELOAD_SPEED,
            .home_speed = DART_HOME_SPEED,
            .pull_speed_limit = DART_PULL_SPEED_LIMIT,
            .trigger_speed_limit = DART_TRIGGER_SPEED_LIMIT,
            .pull_travel = DART_PULL_TRAVEL,
            .pull_position_tolerance = DART_PULL_TOLERANCE,
            .pull_current_threshold = DART_PULL_CURRENT_THRESHOLD,
            .pull_stall_time_ms = DART_PULL_STALL_TIME_MS,
            .pull_timeout_ms = DART_PULL_TIMEOUT_MS,
            .home_timeout_ms = DART_HOME_TIMEOUT_MS,
            .reload_timeout_ms = DART_RELOAD_TIMEOUT_MS,
            .servo_lock_angle = DART_SERVO_LOCK_ANGLE,
            .servo_release_angle = DART_SERVO_RELEASE_ANGLE,
            .servo_angle_min = DART_SERVO_MIN_ANGLE,
            .servo_angle_max = DART_SERVO_MAX_ANGLE,
            .servo_min_pulse_s = DART_SERVO_MIN_PULSE_S,
            .servo_max_pulse_s = DART_SERVO_MAX_PULSE_S,
            .servo_period_s = DART_SERVO_PERIOD_S,
            .servo_release_time_ms = DART_SERVO_RELEASE_TIME_MS,
        },
};
