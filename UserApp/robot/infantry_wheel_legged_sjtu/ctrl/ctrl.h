#ifndef INFANTRY_CTRL_H
#define INFANTRY_CTRL_H

#include "robot.h"
#include "user_lib.h"

#define has_non_zero_data(data) \
    (data != NULL) && \
    (data->gimbal_receive.yaw != 0 || data->gimbal_receive.pitch != 0 || data->shoot_receive.fire_flag != 0)

#define REVERSE_FOLLOW_EXIT_DEG 160.0f
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
    uint8_t reverse_follow; // 1 = X 掉头反向跟随激活 (FOLLOW 内复用 rear_err 路径反 vx, 同时锁 chassis yaw)
} Ctrl_Intent_s;

typedef enum {
  JUMP_IDLE = 0,
  JUMP_READY,    // CHASSIS_JUMP_READY 就位, 等待 V 起跳
  JUMP_ACTIVE,   // 起跳中, 等待 chassis->jump_state 回 IDLE 或超时
} JumpPhase_e;

typedef enum {
  FIRE_VISION_PRIORITY = 0,  // 左键 + vision fire_flag 同时满足才开火
  FIRE_MOUSE_PRIORITY,       // 左键按下即开火
} FireMode_e;

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

// 速度常量 (内化到 CtrlInstance, 上电时由静态初始化设定).
// 单位已全姿态统一: vx = m/s, wz = rad/s. 趴下的 motor-rpm 换算在 chassis 层 (ChassisProstrate) 完成.
typedef struct {
  float vx;           // 默认平动 (WASD / 摇杆峰, stand 与 prostrate 通用)
  float wz;           // 小陀螺角速度上限 (与 rotate_scale 相乘, stand 与 prostrate 通用)
  float stair;  // 蹭台阶模式 (Z 切换, 慢速逼近台阶)
  float vault;        // 跳台阶模式 (Ctrl+V) 速度
} CtrlSpeed_s;

// CHASSIS_RECOVERY 触发阈值 (deg). 蹭台阶模式下放宽, 让倾翻更早进入恢复, 避免硬怼台阶.
typedef struct {
  float theta_default;  // 默认姿态 |theta| 阈值 (腿角偏离竖直方向)
  float theta_creep;    // 蹭台阶模式 |theta| 阈值 (更小, 更敏感)
  float pitch_default;  // 默认姿态 |pitch| 阈值 (机身俯仰角)
  float pitch_creep;    // 蹭台阶模式 |pitch| 阈值
} RecoveryThresh_s;

// 蹭台阶状态: 进入时把腿切到最高档并锁慢速, 退出恢复.
typedef struct {
  uint8_t active;         // 1 = 蹭台阶模式生效
  int     saved_leg_idx;  // 进入前 leg.index 快照, 退出时恢复
} Stair_s;

// X 掉头反向跟随: 云台 +180° 后底盘不跟随云台, 直到云台已转过 130° 再回到正常 FOLLOW.
typedef struct {
  uint8_t active;     // 1 = 反向跟随进行中
  float   start_yaw;  // 进入时云台 IMU YawTotalAngle 快照 (deg, 非缠绕)
} ReverseFollow_s;

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
  JumpFsm_s       jump;
  LegPreset_s     leg;
  Stair_s    stair;
  ReverseFollow_s reverse;
  FireMode_e      fire_mode;

  // ---- 上电常量 (静态初始化设定, 运行时不变) ----
  CtrlSpeed_s      speed;
  RecoveryThresh_s recovery;
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

FireMode_e GetFireMode(void);

#endif // INFANTRY_CTRL_H
