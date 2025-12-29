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
    SEMI_AUTO_INSERT_MINERAL,     // 第一步：将矿物插入科技核心（原第二步）
    SEMI_AUTO_RAISE_ARM,          // 第二步：机械臂整体上抬（原第三步）
    SEMI_AUTO_FLIP_HANDLE,        // 第三步：掰科技核心把手（原第四步）
    SEMI_AUTO_ROTATE_RIGHT,       // 第四步：向右旋转5度（原第五步）
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
    float chassis_forward_speed;  // 底盘前移速度（用于插入操作）
    float arm_raise_angle;        // 机械臂上抬角度
    float handle_flip_angle;      // 把手掰动角度
    float rotate_angle;           // 旋转角度（5度）
    uint32_t step_delay_ms;       // 步骤间延时（毫秒）
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
void StartSemiAutoOperation(void);    // 启动半自动操作（从插入矿物开始）
void LiftGantryToTarget(void);        // 独立抬升龙门架功能
void StopSemiAutoOperation(void);     // 停止半自动操作
void ResetSemiAutoOperation(void);    // 重置半自动操作

#endif // SEMI_AUTOMATIC_H