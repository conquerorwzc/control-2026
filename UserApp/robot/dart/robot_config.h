/**
 * @file robot_config.h
 * @brief dart 发射架机器人全部硬件接线、机械参数与控制参数
 *
 * 电机布局(默认全部挂 hcan1):
 *   - yaw:    M2006 x1, 角度环串级 PID 控制发射架 yaw
 *   - belt:   M3508 x2, 水平对置驱动同步带挡块, 位置环串级 PID, 严格同步
 *   - trigger:M3508 x1, 控制扳机位置(决定拉簧拉伸量, 即射力, 可调)
 *
 * 坐标约定:
 *   - 同步带逻辑坐标: 0 = 释放方向硬限位(堵转校准得到), 正方向 = 储能方向
 *   - 扳机/yaw 逻辑坐标: 0 = 开机位置
 *
 * @attention 所有行程/角度/力矩参数均为占位初值, 必须按实机机械结构标定;
 *            电机方向不对时只翻转对应 DART_XXX_REVERSE 宏。
 */
#pragma once

/* ================= 板型: 单板控制发射架 ================= */
#define ONE_BOARD
// #define CHASSIS_BOARD
// #define GIMBAL_BOARD

#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) || \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error "只能定义一个板型宏: ONE_BOARD / CHASSIS_BOARD / GIMBAL_BOARD"
#endif

/* ================= 硬件接线(CAN ID / 串口) ================= */
#define DART_YAW_CAN (&hcan1)      // yaw 电机总线
#define DART_BELT_CAN (&hcan1)     // 同步带电机总线
#define DART_TRIGGER_CAN (&hcan1)  // 扳机电机总线

#define DART_YAW_CAN_ID 1      // M2006 yaw
#define DART_BELT_L_CAN_ID 2   // M3508 同步带电机(左)
#define DART_BELT_R_CAN_ID 3   // M3508 同步带电机(右)
#define DART_TRIGGER_CAN_ID 4  // M3508 扳机

/* 遥控器串口: F407 用 huart3(DBUS), H7 用 huart5 */
#ifdef STM32H723XX
#define DART_RC_UART (&huart5)
#else
#define DART_RC_UART (&huart3)
#endif

/* ================= 电机方向 =================
 * 两个同步带电机镜像安装, 配置使得逻辑正方向均为储能方向;
 * 若实机转向与定义相反, 翻转对应宏即可。 */
#define DART_YAW_REVERSE MOTOR_DIRECTION_NORMAL
#define DART_BELT_L_REVERSE MOTOR_DIRECTION_NORMAL
#define DART_BELT_R_REVERSE MOTOR_DIRECTION_REVERSE  // 对置安装, 需反转
#define DART_TRIGGER_REVERSE MOTOR_DIRECTION_NORMAL

/* ================= 零位校准参数 ================= */
#define DART_CALI_SPEED_DPS 8.0f          // 校准顶硬限位速度 (deg/s)
#define DART_CALI_DIRECTION (-1.0f)       // 释放方向符号; 实机相反时改 +1.0f
#define DART_CALI_MAX_OUT 1500.0f         // 校准前/校准中严格限流 (M3508 满量程 16384)
#define DART_CALI_INTEGRAL_LIMIT 800.0f   // 校准中积分限幅
#define DART_CALI_STALL_SPEED_DPS 2.0f    // 堵转判定: 速度阈值 (deg/s)
#define DART_CALI_STALL_MS 300            // 堵转判定: 持续时间 (ms)
#define DART_CALI_TIMEOUT_MS 8000         // 校准单步超时 (ms)
#define DART_CALI_BACKOFF_DEG 15.0f       // 顶到限位后回撤距离, 即回缩位 (deg)

/* ================= 同步带(储能)参数 ================= */
#define DART_BELT_HOME_DEG DART_CALI_BACKOFF_DEG  // 逻辑回缩位: 挡块退出发射平台活动范围
#define DART_BELT_CHARGE_DEG 360.0f               // 储能行程 (deg, 实机标定)
#define DART_BELT_POS_TOL_DEG 3.0f                // 位置到位容差 (deg)
#define DART_BELT_MAX_OUT 8000.0f                 // 正常工作电流限幅
#define DART_BELT_INTEGRAL_LIMIT 3000.0f          // 正常工作积分限幅
#define DART_BELT_MAX_SPEED_DPS 360.0f            // 位置环输出限幅 = 最大速度
#define DART_BELT_SYNC_WARN_DEG 10.0f             // 双电机位置偏差告警阈值 (deg)
#define DART_CHARGE_STALL_SPEED_DPS 3.0f          // 储能完成堵转判定速度阈值
#define DART_CHARGE_STALL_MS 300                  // 储能完成堵转判定持续时间
#define DART_CHARGE_COMPLETE_ON_STALL 1           // 1: 到位或双电机堵转(平台被扳机卡住)均判完成
#define DART_CHARGE_TIMEOUT_MS 5000               // 储能拉拽超时 (ms)
#define DART_CHARGE_HOLD_MS 200                   // 拉拽到位后保持时间 (ms)
#define DART_RETRACT_TIMEOUT_MS 5000              // 挡块回撤超时 (ms)

/* ================= 扳机(射力)参数 =================
 * 逻辑坐标以开机位置为 0; 卡位可调范围 [MIN, MAX];
 * 释放位必须在可调范围之外, 保证发射时扳机确实脱开。 */
#define DART_TRIGGER_CATCH_DEFAULT_DEG 0.0f   // 默认卡位(开机位置), 即默认射力
#define DART_TRIGGER_MIN_DEG (-60.0f)         // 卡位可调下限 (拉伸量小, 射力小)
#define DART_TRIGGER_MAX_DEG 60.0f            // 卡位可调上限 (拉伸量大, 射力大)
#define DART_TRIGGER_RELEASE_DEG (-90.0f)     // 发射释放位 (实机标定)
#define DART_TRIGGER_POS_TOL_DEG 2.0f         // 位置到位容差 (deg)
#define DART_TRIGGER_MAX_OUT 6000.0f          // 电流限幅
#define DART_TRIGGER_INTEGRAL_LIMIT 2000.0f   // 积分限幅
#define DART_TRIGGER_MAX_SPEED_DPS 300.0f     // 位置环输出限幅 = 最大速度
#define DART_TRIGGER_ADJ_DPS 60.0f            // 拨轮满行程时射力调节速度 (deg/s)
#define DART_TRIGGER_SETTLE_TIMEOUT_MS 2000   // 扳机就位超时 (ms)
#define DART_FIRE_DWELL_MS 500                // 发射时扳机释放位保持时间 (ms)
#define DART_FIRE_TIMEOUT_MS 3000             // 发射序列单步超时 (ms)

/* ================= yaw 参数 ================= */
#define DART_YAW_SENSITIVITY_DPS 60.0f   // 满杆 yaw 角速度 (deg/s)
#define DART_YAW_SOFT_LIMIT_DEG 500.0f   // 相对开机位置的软限位 (deg)
#define DART_YAW_LEAD_LIMIT_DEG 30.0f    // 目标角超前反馈的限幅, 防目标跑飞 (deg)
#define DART_YAW_MAX_OUT 4000.0f         // 电流限幅 (M2006/C610 满量程 10000)
#define DART_YAW_INTEGRAL_LIMIT 1500.0f  // 积分限幅

/* ================= 遥控器参数 ================= */
#define DART_RC_DEADZONE 20      // 摇杆/拨轮死区
#define DART_STICK_FULL 660      // 摇杆满行程
#define DART_TASK_DT_S 0.001f    // RobotTask 标称周期 (s)

/* DR16 操作映射:
 *   右开关: 下 = 安全停机(全部电机停); 中 = 使能(上电首次使能自动校准); 上 = 使能 + 调试点动
 *   左开关: 下 = 待机; 中(上升沿) = 储能命令(故障/未校准时为重新校准); 上(上升沿) = 发射命令
 *   右摇杆水平: yaw 增量式角度目标
 *   侧边拨轮:    扳机卡位(射力)微调, 仅 IDLE/READY 生效
 *   调试档(右开关上): 左摇杆竖直 = 双同步带电机同速点动; 左摇杆水平 = 扳机点动 */

/* ================= 调试点动参数 ================= */
#define DART_DEBUG_BELT_MAX_SPEED_DPS 60.0f     // 同步带点动满杆速度 (deg/s)
#define DART_DEBUG_TRIGGER_MAX_SPEED_DPS 60.0f  // 扳机点动满杆速度 (deg/s)

/* ================= 任务周期 ================= */
#define DART_SYNC_WARN_PERIOD_MS 1000  // 同步偏差告警的最小打印间隔

/**
 * @brief yaw 电机(M2006)配置: 角度环串级速度环
 */
#define DART_YAW_MOTOR_CONFIG(can_h, _id)                                      \
  {                                                                            \
      .motor_type = M2006,                                                     \
      .can_init_config =                                                       \
          {                                                                    \
              .can_handle = can_h,                                             \
              .tx_id = _id,                                                    \
          },                                                                   \
      .controller_setting_init_config =                                        \
          {                                                                    \
              .angle_feedback_source = MOTOR_FEED,                             \
              .speed_feedback_source = MOTOR_FEED,                             \
              .outer_loop_type = ANGLE_LOOP,                                   \
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                      \
              .motor_reverse_flag = DART_YAW_REVERSE,                          \
              .feedback_reverse_flag = DART_YAW_REVERSE,                       \
          },                                                                   \
      .controller_param_init_config =                                          \
          {                                                                    \
              .speed_PID =                                                     \
                  {                                                            \
                      .Kp = 2.0f,                                              \
                      .Ki = 0.3f,                                              \
                      .Kd = 0.0f,                                              \
                      .MaxOut = DART_YAW_MAX_OUT,                              \
                      .IntegralLimit = DART_YAW_INTEGRAL_LIMIT,                \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral, \
                  },                                                           \
              .angle_PID =                                                     \
                  {                                                            \
                      .Kp = 12.0f,                                             \
                      .Ki = 0.0f,                                              \
                      .Kd = 0.0f,                                              \
                      .MaxOut = DART_YAW_SENSITIVITY_DPS * 4.0f,               \
                      .DeadBand = 0.2f,                                        \
                  },                                                           \
          },                                                                   \
  }

/**
 * @brief 同步带电机(M3508)配置: 位置环串级速度环
 * @note 速度环启用 PID_ErrorHandle 作为堵转检测的备份判据
 */
#define DART_BELT_MOTOR_CONFIG(can_h, _id, _reverse)                                    \
  {                                                                                     \
      .motor_type = M3508,                                                              \
      .can_init_config =                                                                \
          {                                                                             \
              .can_handle = can_h,                                                      \
              .tx_id = _id,                                                             \
          },                                                                            \
      .controller_setting_init_config =                                                 \
          {                                                                             \
              .angle_feedback_source = MOTOR_FEED,                                      \
              .speed_feedback_source = MOTOR_FEED,                                      \
              .outer_loop_type = ANGLE_LOOP,                                            \
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                               \
              .motor_reverse_flag = _reverse,                                           \
              .feedback_reverse_flag = _reverse,                                        \
          },                                                                            \
      .controller_param_init_config =                                                   \
          {                                                                             \
              .speed_PID =                                                              \
                  {                                                                     \
                      .Kp = 5.0f,                                                       \
                      .Ki = 0.8f,                                                       \
                      .Kd = 0.0f,                                                       \
                      .MaxOut = DART_BELT_MAX_OUT,                                      \
                      .IntegralLimit = DART_BELT_INTEGRAL_LIMIT,                        \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_ErrorHandle, \
                  },                                                                    \
              .angle_PID =                                                              \
                  {                                                                     \
                      .Kp = 20.0f,                                                      \
                      .Ki = 0.0f,                                                       \
                      .Kd = 0.0f,                                                       \
                      .MaxOut = DART_BELT_MAX_SPEED_DPS,                                \
                      .DeadBand = 0.5f,                                                 \
                  },                                                                    \
          },                                                                            \
  }

/**
 * @brief 扳机电机(M3508)配置: 位置环串级速度环
 */
#define DART_TRIGGER_MOTOR_CONFIG(can_h, _id, _reverse)                          \
  {                                                                              \
      .motor_type = M3508,                                                       \
      .can_init_config =                                                         \
          {                                                                      \
              .can_handle = can_h,                                               \
              .tx_id = _id,                                                      \
          },                                                                     \
      .controller_setting_init_config =                                          \
          {                                                                      \
              .angle_feedback_source = MOTOR_FEED,                               \
              .speed_feedback_source = MOTOR_FEED,                               \
              .outer_loop_type = ANGLE_LOOP,                                     \
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                        \
              .motor_reverse_flag = _reverse,                                    \
              .feedback_reverse_flag = _reverse,                                 \
          },                                                                     \
      .controller_param_init_config =                                            \
          {                                                                      \
              .speed_PID =                                                       \
                  {                                                              \
                      .Kp = 5.0f,                                                \
                      .Ki = 0.5f,                                                \
                      .Kd = 0.0f,                                                \
                      .MaxOut = DART_TRIGGER_MAX_OUT,                            \
                      .IntegralLimit = DART_TRIGGER_INTEGRAL_LIMIT,              \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,   \
                  },                                                             \
              .angle_PID =                                                       \
                  {                                                              \
                      .Kp = 15.0f,                                               \
                      .Ki = 0.0f,                                                \
                      .Kd = 0.0f,                                                \
                      .MaxOut = DART_TRIGGER_MAX_SPEED_DPS,                      \
                      .DeadBand = 0.2f,                                          \
                  },                                                             \
          },                                                                     \
  }
