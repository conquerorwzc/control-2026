//
// heavy_three_friction_launcher —— 发射机构 + 云台台架测试机器人
// 只包含「云台 + 发射机构」，不含底盘、超级电容、裁判系统
//

#pragma once

#include "gimbal.h"
#include "shoot.h"

#if defined(USE_DUAL_RC)
#include "remote_control.h"
#endif

typedef enum {
  ROBOT_POWER_OFF = 0,
  ROBOT_POWER_ON,
} Robot_Mode_e;

/* 调试快照: 每帧刷新, 在 Ozone / watch 里加 bench_dbg 就能看到台架全部关键量 */
typedef struct {
  /* 遥控 */
  uint8_t rc_online;      // 遥控是否在线(守护进程判定)
  uint8_t sw_left;        // 左拨杆原始值: 1=上, 3=中, 2=下
  int16_t rocker_l1;      // 左摇杆竖直(向下为负), 发弹触发用
  int16_t rocker_r_;      // 右摇杆水平, yaw 用
  int16_t rocker_r1;      // 右摇杆竖直, pitch 用
  int16_t dial;           // 拨轮, 摩擦轮微调用
  /* 逻辑状态 */
  uint8_t stick_armed;    // 单发回中锁
  uint8_t load_mode_now;  // 当帧下发的拨盘模式(LOAD_STOP/1_BULLET/BURSTFIRE/REVERSE)
  uint8_t loader_busy;    // 上一发是否还没走完(到位门限)
  /* 云台 */
  float yaw_angle;
  float yaw_speed;        // 度/秒
  float pitch_angle;
  float pitch_speed;      // 度/秒
  /* 拨盘 */
  float loader_angle;     // 电机总角度
  float loader_delta;     // 相对上一发起始角走过的角度, 接近 ±6000 即完成一发
  float loader_speed;     // 度/秒
  int16_t loader_current; // C620 实际电流
  uint8_t loader_temp;    // 温度
  uint32_t loader_err_type;  // PID 堵转标志(2 = PID_MOTOR_BLOCKED_ERROR, 见 controller.h)
  uint32_t loader_err_cnt;   // 堵转计数, 超过 500 置位
  /* 摩擦轮 ×3 —— 指令侧 */
  float fric_ref[3];      // 下发给该轮的速度环目标(已乘系数), 单位 °/s
  float fric_out[3];      // 速度环最终输出, 电调电流原始值(±16384 对应 ±20A)
  uint8_t fric_stop[3];   // 1 = 该轮正处于 MOTOR_STOP(输出被清零)
  /* 摩擦轮 ×3 —— 反馈侧 */
  float fric_speed[3];    // 度/秒, 收到 0x201/0x202/0x203 才有数
  int16_t fric_current[3];
  uint8_t fric_temp[3];
  /* 摩擦轮 ×3 —— 换算量 */
  float fric_v[3];        // 轮面线速度(米/秒) = |speed_aps| * deg2rad * R / 传动比
  float fric_v_ref[3];    // 目标线速度(米/秒), 同式作用于 fric_ref
  float fric_track[3];    // 实际/目标 转速比(1.0 表示无静差, 停火时置 0)
} BenchDebug_s;

extern volatile BenchDebug_s bench_dbg;

typedef struct {
  Robot_Mode_e robot_mode;  // 机器人整体工作状态

#if defined(USE_DUAL_RC)
  RC_ctrl_t *rc_data;  // 旧版 DJI 遥控(DBUS)数据指针, 是长度为 2 的数组, 用 rc_data[TEMP]
#else
  void *rc_data;  // 未定义宏时的缺省回退
#endif

  // 台架不接裁判系统，也不保留裁判数据指针：热量上限/当前热量直接由
  // Shoot_Ctrl_Cmd_s 给出（见 robot.c 的 RobotInit）
  GimbalInstance *gimbal;
  ShootInstance *shoot;
} RobotInstance;

/**
 * @brief 机器人初始化，会在 StartROBOTTASK 中调用
 */
void RobotInit();

/**
 * @brief 机器人任务，200Hz 运行
 */
void RobotTask();

RobotInstance *getRobot();
