#pragma once

#include "dmmotor.h"

/* 电机台数上限; 实际初始化台数由 Grab_Param_s.motor_num 决定(缺席的台不初始化不握手) */
#define GRAB_MOTOR_NUM 7u

/**
 * @brief 两态状态机, 表示机械臂执行状态
 *        GRAB_ESTOP  急停, 电机零力矩
 *        GRAB_RUN    正常运转(参考位置=锚定时刻的当前位置, 无回零动作)
 *        写入归属: robot 层写 ESTOP 与恢复(锚定→RUN 直接切), 上电首次锚定由组件在
 *        首次 GrabTask 完成(反馈帧此时才可用)
 */
typedef enum
{
    GRAB_ESTOP = 0,
    GRAB_RUN,
} Grab_Mode_e;

/**
 * @brief 命令层: robot 层接口.
 *        注意: cmd_angle[i] 的单位是【关节域】(第 i 轴的关节角),
 *        在指定电机PID_Ref 前按照 joint[i].reduction 换算为电机输出轴角度.
 */
typedef struct
{
    float cmd_angle[GRAB_MOTOR_NUM]; // 角度外环目标, rad, 相对上电/恢复锚点(锚定入口写入当前角)
    Grab_Mode_e grab_mode;
} Grab_Ctrl_Cmd_s;

typedef struct
{
    uint8_t motor_num;  // 实际使用的电机台数 (<= GRAB_MOTOR_NUM)
} Grab_Param_s;

/**
 * @brief 单关节机械参数包 (逐轴, config 侧).
 *        一个关节 = 一台驱动电机 + 机械参数; 将来的行程限位/契约角零点/重力补偿系数都住这里.
 */
typedef struct
{
    Motor_Init_Config_s motor_config; // 该关节驱动电机的框架初始化块
    float reduction;                  // θ电机/θ关节 换算比, 符号含方向; 直连=1.0, 带轮填实测比
    float limit;                      // 行程限幅 rad【关节域】, 以锚点为中心钳位; TODO: q_zero(契约角)
} Grab_Joint_Init_s;

typedef struct
{
    Grab_Joint_Init_s joint[GRAB_MOTOR_NUM];
    Grab_Param_s grab_param;
} Grab_Init_Config_s;

/**
 * @brief 单关节 (instance 侧) = 驱动电机 + 机械参数 + 运行时状态.
 */
typedef struct
{
    float reduction;
    float limit;    // 行程限幅 rad【关节域】, robot 层 CmdMap 钳位读这里
    /* 驱动电机实例 */
    DMMotorInstance *motor;
    /* 运行状态 */
    float q;        // 实际关节角 rad【关节域】

} Grab_Joint_s;

typedef struct
{
    Grab_Ctrl_Cmd_s grab_ctrl_cmd;
    Grab_Joint_s joint[GRAB_MOTOR_NUM];
    Grab_Param_s grab_param;
} GrabInstance;

/**
 * @brief 机械臂组件初始化: 注册 DM 电机, 初始化命令层(上电默认 GRAB_RUN, 首次 GrabTask 完成使能握手与锚定).
 *        只支持角度外环(速度环仅作级联内环), 传入其他 outer_loop_type 行为未定义
 *        在 RobotInit 中调用 (内部 DMMotorInit, 发送线程由 os_task.c 的 DMMotorTaskInit 创建)
 */
GrabInstance *GrabInit(Grab_Init_Config_s *config);

/**
 * @brief 机械臂任务: 两态状态机执行 + 电机下发.
 *        首次调用完成全部电机使能握手并锚定当前位置(非 ESTOP 时); 之后每周期刷新关节反馈 q, 再按 grab_mode 分支.
 *        在 RobotTask 中以固定频率调用.
 */
void GrabTask();

/**
 * @brief 锚定入口(上电首次运行 / 急停恢复共用): 以当前位置为参考, 无回零动作
 *        enable(翻回 MOTOR_ENALBED, 防 ESTOP 单向门) + cmd_angle := 当前关节角(MotorToJoint 换算)
 *        (初始误差 0 -> 进闭环瞬间无力矩跳变) + 清积分.
 *        调用后紧挨着写 grab_mode = GRAB_RUN (调用端显式调用, 顺序不可反).
 */
void GrabAnchorEntry();
