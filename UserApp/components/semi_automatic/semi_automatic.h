#ifndef SEMI_AUTOMATIC_H
#define SEMI_AUTOMATIC_H

#include "gantry.h"
#include "grab.h"
#include "chassis.h"
#include "remote_control.h"

// 半自动操作状态枚举
typedef enum
{
    SEMI_AUTO_IDLE = 0,           // 空闲状态
    SEMI_AUTO_RAISE_GANTRY,       // 第一步：抬升龙门架
    SEMI_AUTO_MOVE_CHASSIS,       // 第二步：移动底盘
    SEMI_AUTO_ARM_RAISE,          // 第三步：机械臂上抬
    SEMI_AUTO_ARM_FLIP,           // 第四步：掰把手
    SEMI_AUTO_ARM_ROTATE,         // 第五步：旋转
    SEMI_AUTO_COMPLETE,           // 完成状态
    SEMI_AUTO_ERROR               // 错误状态
} SemiAutoState_e;

// 半自动控制命令结构体
typedef struct {
    SemiAutoState_e state;        // 当前操作状态
    uint8_t is_running;           // 是否正在运行半自动操作
    uint32_t step_start_time;     // 当前步骤开始时间
    uint8_t manual_stop;          // 手动停止标志
} SemiAuto_Ctrl_Cmd_s;

// 半自动操作参数结构体
typedef struct {
    float gantry_lift_pos;        // 龙门架抬升目标位置
    float chassis_forward_speed;  // 底盘前移速度
    // 机械臂关节控制参数
    float base_joint_angle;       // 基座关节角度
    float elbow_pitch_angle;      // 肘部俯仰角度
    float elbow_roll_angle;       // 肘部滚动角度
    float wrist_roll_angle;       // 腕部滚动角度
    float wrist_pitch_angle;      // 腕部俯仰角度
    uint32_t arm_raise_delay_ms;  // 机械臂上抬动作延迟（毫秒）
    uint32_t handle_flip_delay_ms; // 把手掰动动作延迟（毫秒）
    uint32_t rotate_delay_ms;      // 旋转动作延迟（毫秒）
} SemiAuto_Param_s;

// 半自动操作初始化配置
typedef struct {
    SemiAuto_Param_s param;
} SemiAuto_Init_Config_s;

// 半自动操作实例
typedef struct {
    SemiAuto_Param_s param;           // 参数
    SemiAuto_Ctrl_Cmd_s ctrl_cmd;     // 控制命令
    GantryInstance* gantry;           // 龙门架实例
    GrabInstance* grab;               // 机械臂实例
    ChassisInstance* chassis;         // 底盘实例
} SemiAutoInstance;

// 函数声明
SemiAutoInstance* SemiAutoInit(SemiAuto_Init_Config_s* init_config);
void SemiAutoTask(void);
void StartSemiAutoOperation(void);    // 启动半自动操作（从抬升龙门架开始）

void StopSemiAutoOperation(void);     // 停止半自动操作
void ResetSemiAutoOperation(void);    // 重置半自动操作
void StartGantryLift(void);           // 启动龙门架抬升
void StartChassisMove(void);          // 启动底盘移动
void StartArmRaise(void);             // 启动机械臂上抬
void StartArmFlip(void);              // 启动掰把手
void StartArmRotate(void);            // 启动旋转
void StartArmSequence(void);          // 启动机械臂序列动作

#endif // SEMI_AUTOMATIC_H