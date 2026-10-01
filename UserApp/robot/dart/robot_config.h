/**
 * @file robot_config.h
 * @brief dart 发射架机器人全部硬件接线、机械参数与控制参数
 *
 * 电机布局(默认全部挂 hcan1):
 *   - yaw:    M2006 x1, 角度环串级 PID 控制发射架 yaw
 *   - belt:   M3508 x2, 水平对置驱动同步带挡块, 位置环串级 PID, 严格同步
 *   - screw:  M3508 x1, 扳机丝杆, 控制扳机整体位置(决定拉簧拉伸量=射力, 可调)
 *   - servo:  PWM 舵机 x1, 扳机的发射动作(卡位角扣住/释放角放开), 50Hz 占空比->角度
 *
 * 坐标约定:
 *   - 同步带逻辑坐标: 0 = 释放方向硬限位(堵转校准得到), 正方向 = 储能方向
 *   - 丝杆/yaw 逻辑坐标: 0 = 开机位置
 *   - 舵机: 直接给角度, 映射到脉宽见 DART_SERVO_* 宏
 *
 * @attention 所有行程/角度/力矩参数均为占位初值, 必须按实机机械结构标定;
 *            电机方向不对时只翻转对应 DART_XXX_REVERSE 宏;
 *            舵机换定时器/通道只改 DART_SERVO_PWM_TIM / DART_SERVO_PWM_CHANNEL 两行。
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
#define DART_YAW_CAN (&hcan1)     // yaw 电机总线
#define DART_BELT_CAN (&hcan1)    // 同步带电机总线
#define DART_SCREW_CAN (&hcan1)   // 扳机丝杆电机总线

#define DART_YAW_CAN_ID 1     // M2006 yaw
#define DART_BELT_L_CAN_ID 2  // M3508 同步带电机(左)
#define DART_BELT_R_CAN_ID 3  // M3508 同步带电机(右)
#define DART_SCREW_CAN_ID 4   // M3508 扳机丝杆

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
#define DART_SCREW_REVERSE MOTOR_DIRECTION_NORMAL

/* ================= 零位校准参数 ================= */
#define DART_CALI_SPEED_DPS 8.0f          // 校准顶硬限位速度 (deg/s)
#define DART_CALI_DIRECTION (-1.0f)       // 释放方向符号; 实机相反时改 +1.0f
#define DART_CALI_MAX_OUT 1500.0f         // 校准前/校准中严格限流 (M3508 满量程 16384)
#define DART_CALI_INTEGRAL_LIMIT 800.0f   // 校准中积分限幅
#define DART_CALI_STALL_SPEED_DPS 2.0f    // 堵转判定: 速度阈值 (deg/s)
#define DART_CALI_STALL_MS 300            // 堵转判定: 持续时间 (ms)
#define DART_CALI_TIMEOUT_MS 8000         // 校准单步超时 (ms)
#define DART_CALI_BACKOFF_DEG 15.0f       // 顶到限位后回撤距离, 即释放位置(回缩位) (deg)
#define DART_CALI_BACKOFF_SPEED_DPS 20.0f // 顶到限位后回撤限速 (deg/s, 顶死后必须慢速退开)

/* ================= 同步带(储能)参数 =================
 * 扳机锁定状态下是单向通道: 发射平台会滑过扳机继续被向后拉, 因此储能时同步带不会堵转,
 * 储能只按位置行程推进(拉到位 -> 限速复位到释放位置), 不依赖堵转判完成;
 * 仅当超过滑台行程等异常顶死时, 由堵转看门狗直接判故障停机。
 * 位置环限速: 各阶段通过角度环 MaxOut 限速, 最终不超过 DART_BELT_MAX_SPEED_DPS(电机级硬上限) */
#define DART_BELT_HOME_DEG DART_CALI_BACKOFF_DEG  // 释放位置(回缩位): 挡块退出发射平台活动范围
#define DART_BELT_CHARGE_DEG 360.0f               // 储能行程: 零位起沿储能方向的电机轴角度 (deg, 实机标定, 须小于滑台行程)
#define DART_BELT_POS_TOL_DEG 3.0f                // 位置到位容差 (deg)
#define DART_BELT_MAX_OUT 8000.0f                 // 正常工作电流限幅
#define DART_BELT_INTEGRAL_LIMIT 3000.0f          // 正常工作积分限幅
#define DART_BELT_MAX_SPEED_DPS 360.0f            // 位置环速度硬上限 (各阶段限速都会被它再钳一次)
#define DART_BELT_HOME_SPEED_DPS 120.0f           // 复位到释放位置/挡块回撤限速 (deg/s)
#define DART_BELT_CHARGE_SPEED_DPS 120.0f         // 储能拉拽限速 (deg/s)
#define DART_BELT_SYNC_WARN_DEG 10.0f             // 双电机位置偏差告警阈值 (deg)
#define DART_CHARGE_STALL_SPEED_DPS 3.0f          // 储能堵转看门狗速度阈值 (deg/s, 超滑台行程会顶死)
#define DART_CHARGE_STALL_MS 300                  // 储能堵转看门狗持续时间 (ms)
#define DART_CHARGE_TIMEOUT_MS 5000               // 储能拉到位超时 (ms) -> 故障
#define DART_RETRACT_TIMEOUT_MS 5000              // 复位到释放位置超时 (ms) -> 故障

/* ================= 扳机丝杆(射力)参数 =================
 * 扳机整体装在丝杆上, 由 M3508 位置环控制;
 * 丝杆位置决定卡住发射平台时拉簧的拉伸量, 直接影响发射力量和速度(可调)。
 * 逻辑坐标以开机位置为 0。 */
#define DART_SCREW_POS_DEFAULT_DEG 0.0f    // 默认射力位置(开机位置)
#define DART_SCREW_POS_MIN_DEG (-60.0f)    // 射力位置下限 (拉伸量小, 射力小)
#define DART_SCREW_POS_MAX_DEG 60.0f       // 射力位置上限 (拉伸量大, 射力大)
#define DART_SCREW_POS_TOL_DEG 2.0f        // 位置到位容差 (deg)
#define DART_SCREW_MAX_OUT 6000.0f         // 电流限幅
#define DART_SCREW_INTEGRAL_LIMIT 2000.0f  // 积分限幅
#define DART_SCREW_MAX_SPEED_DPS 300.0f    // 位置环输出限幅 = 最大速度
#define DART_SCREW_ADJ_DPS 60.0f           // 拨轮满行程时射力调节速度 (deg/s)
#define DART_SCREW_SETTLE_TIMEOUT_MS 2000  // 储能前丝杆就位超时 (ms)

/* ================= 扳机舵机(PWM 发射动作)参数 =================
 * 舵机通过 50Hz PWM 占空比控制角度: 脉宽线性映射到 [ANGLE_MIN, ANGLE_MAX]。
 * 换定时器/通道只改下面两行(所选定时器需在 CubeMX 配好对应通道的 PWM,
 * 建议预分频使计数 tick = 1us, 便于按脉宽微秒数计算)。 */
#define DART_SERVO_PWM_TIM (&htim1)             // ← 舵机 PWM 定时器(待定, 按实际接线改)
#define DART_SERVO_PWM_CHANNEL (TIM_CHANNEL_1)  // ← 舵机 PWM 通道
#define DART_SERVO_PWM_PERIOD_S (0.02f)         // 50Hz PWM 周期 (s)
#define DART_SERVO_PULSE_MIN_US 500.0f          // 角度下限对应脉宽 (us)
#define DART_SERVO_PULSE_MAX_US 2500.0f         // 角度上限对应脉宽 (us)
#define DART_SERVO_ANGLE_MIN_DEG 0.0f           // 舵机机械角度下限 (deg)
#define DART_SERVO_ANGLE_MAX_DEG 180.0f         // 舵机机械角度上限 (deg)
#define DART_SERVO_CATCH_DEG 90.0f              // 卡位角: 扣住发射平台(待发/上电默认)
#define DART_SERVO_RELEASE_DEG 0.0f             // 释放角: 放开发射平台(发射)
#define DART_SERVO_SETTLE_MS 300                // 舵机动作等待时间 (ms, 开环无反馈)
#define DART_FIRE_DWELL_MS 500                  // 发射时释放角保持时间 (ms)

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
 *   侧边拨轮:    正常档 = 射力(丝杆位置)微调, 仅 IDLE/READY 生效
 *                调试档 = 直接给舵机角度(满行程映射角度范围, 标定卡位角/释放角用)
 *   调试档(右开关上): 左摇杆竖直 = 双同步带电机同速点动; 左摇杆水平 = 扳机丝杆点动 */

/* ================= 调试点动参数 ================= */
#define DART_DEBUG_BELT_MAX_SPEED_DPS 60.0f   // 同步带点动满杆速度 (deg/s)
#define DART_DEBUG_SCREW_MAX_SPEED_DPS 60.0f  // 扳机丝杆点动满杆速度 (deg/s)

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
 * @brief 扳机丝杆电机(M3508)配置: 位置环串级速度环
 */
#define DART_SCREW_MOTOR_CONFIG(can_h, _id, _reverse)                            \
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
                      .MaxOut = DART_SCREW_MAX_OUT,                              \
                      .IntegralLimit = DART_SCREW_INTEGRAL_LIMIT,                \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,   \
                  },                                                             \
              .angle_PID =                                                       \
                  {                                                              \
                      .Kp = 15.0f,                                               \
                      .Ki = 0.0f,                                                \
                      .Kd = 0.0f,                                                \
                      .MaxOut = DART_SCREW_MAX_SPEED_DPS,                        \
                      .DeadBand = 0.2f,                                          \
                  },                                                             \
          },                                                                     \
  }
