#ifndef INFANTRY_CTRL_H
#define INFANTRY_CTRL_H

#include "robot.h"
#include "user_lib.h"

#define robot_lost_control (fabsf(robot->chassis->imu->Pitch) > 13.0f)  //todo: 12?

#define has_non_zero_data(data) \
    (data != NULL) && \
    (data->gimbal_receive.yaw != 0 || data->gimbal_receive.pitch != 0 || data->shoot_receive.fire_flag != 0)

#define CTRL_SPEED_COFF 2.5f
/**
 * @brief 抽象控制意图，隔离输入设备（遥控器/键鼠）与运动控制逻辑
 */
typedef struct {
    // === 底盘意图 ===
    float vx;               // 底盘 X 向期望速度 (左右)，归一化 [-1.0, 1.0]
    float vy;               // 底盘 Y 向期望速度 (前后)，归一化 [-1.0, 1.0]
    float leg_length_delta; // 腿长增量
    float roll_delta;       // Roll(横滚/pike) 增量
    float gimbal_yaw_ff;    // 云台 yaw 输入预判前馈 (deg), 叠加到 FOLLOW 的 target_yaw
    float rotate_scale;     // 小陀螺缩放系数 (-1..1), 符号决定转向; 0 = 不旋转
} Ctrl_Intent_s;

typedef enum {
  JUMP_IDLE = 0,
  JUMP_READY,    // CHASSIS_JUMP_READY 就位, 等待 V 起跳
  JUMP_ACTIVE,   // 起跳中, 等待 chassis->jump_state 回 IDLE 或超时
} JumpPhase_e;

typedef struct {
  uint8_t fire;   // 1 = 触发开火
  uint8_t burst;  // 1 = 连发 (压制单发)
} ShootReq_s;

typedef struct {
  JumpPhase_e phase;
  float       active_since;     // ACTIVE 进入时刻 (s), 用于超时退出
  uint8_t     observed_active;  // 已观测到 chassis->jump_state 离 IDLE
} JumpFsm_s;

typedef struct {
  int     index;    // [0..3] 四档腿长索引, 默认 1 (= initial_leg_length)
  uint8_t pending;  // 1 = 本帧待写一次绝对腿长 (CtrlSolve 消费后清零)
} LegPreset_s;

typedef struct {
  // ---- 帧末合并意图 (送入 RobotMotionSolve) ----
  Ctrl_Intent_s intent;

  // ---- JoyStickCtrl 写入 (摇杆原始映射, 不读键鼠输入) ----
  float      js_vx, js_vy;    // 摇杆映射后的运动输入
  float      js_yaw_ff;       // JS yaw 增量对应的底盘前馈
  float      js_rotate_scale; // 拨轮 -> 小陀螺缩放系数 (-1..1, 符号决定转向; 0 = 不旋转)
  ShootReq_s js_shoot;        // 扳机请求

  // ---- MouseKeyCtrl 写入 (键鼠原始映射, 不读摇杆输入) ----
  float      mk_vx, mk_vy;    // WASD 映射后的运动输入
  float      mk_yaw_ff;       // 鼠标 yaw 增量对应的底盘前馈
  float      mk_rotate_scale; // shift -> 小陀螺缩放系数 (按住 = 1, 否则 0)
  ShootReq_s mk_shoot;        // 鼠标左键请求
  uint8_t    mk_vision;       // 鼠标右键自瞄请求

  // ---- 帧间持久状态 (两路共写, 边沿驱动) ----
  JumpFsm_s   jump;
  LegPreset_s leg;
} CtrlInstance;

// 状态标志 (全部 is_xxx 风格):
//   is_stand / is_free:  持久 toggle (跨帧), 按钮边沿翻转, is_on=0 时锁定;
//   is_rotate:           CtrlSolve 帧末派生 (js_rotate_scale / mk_rotate_scale 任一非零);
//   is_on:            JS 帧首派生, 拨杆不在最左 (非急停姿态);
//   is_first_update:     上电首帧初始化用, 一次性置 0.
typedef struct {
  uint8_t is_stand;
  uint8_t is_free;
  uint8_t is_rotate;
  uint8_t is_on;
  uint8_t is_first_update;
} UpdateFlag_s;

/**
 * @brief Control logic for Remote Controller mode.
 * @param robot Pointer to the RobotInstance.
 */
void JoyStickCtrl(RobotInstance* robot);

/**
 * @brief Control logic for Mouse and Keyboard mode.
 * @param robot Pointer to the RobotInstance.
 */
void MouseKeyCtrl(RobotInstance* robot);

/**
 * @brief Emergency stop handler.
 * @param robot Pointer to the RobotInstance.
 */
void EmergencyHandler(RobotInstance* robot);

/**
 * @brief 合并 JoyStickCtrl/MouseKeyCtrl 输出 + 解算运动 (OCD 链路).
 * @note 必须在 JoyStickCtrl 与 MouseKeyCtrl 之后调用.
 */
void CtrlSolve(RobotInstance* robot);

#endif // INFANTRY_CTRL_H
