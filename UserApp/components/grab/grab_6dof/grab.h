#pragma once

#include "dmmotor.h"

/* 电机台数上限; 实际初始化台数由 Grab_Param_s.motor_num 决定(缺席的台不初始化不握手) */
#define GRAB_MOTOR_NUM 7u

/**
 * @brief 三态状态机, 表示机械臂执行状态
 *        GRAB_ESTOP  急停, 电机零力矩
 *        GRAB_HOMING 锚定ref + 受限斜坡回零
 *        GRAB_RUN    正常运转
 *        写入归属: robot 层写 ESTOP/HOMING(急停与恢复仪式), HOMING→RUN 由组件在到点时写回
 */
typedef enum
{
    GRAB_ESTOP = 0,
    GRAB_HOMING,
    GRAB_RUN,
} Grab_Mode_e;

/**
 * @brief 命令层: robot 层接口.
 *        注意: cmd_angle[i]/cmd_speed[i] 的单位是【关节域】(第 i 轴的关节角),
 *        在指定电机PID_Ref 前按照 joint[i].reduction 换算为电机输出轴角度.
 */
typedef struct
{
    float cmd_angle[GRAB_MOTOR_NUM]; // 角度外环目标, rad, 绝对值(回中=0)
    float cmd_speed[GRAB_MOTOR_NUM]; // 速度外环目标, rad/s, 绝对值
    Grab_Mode_e grab_mode;
} Grab_Ctrl_Cmd_s;

typedef struct
{
    uint8_t motor_num;  // 实际使用的电机台数 (<= GRAB_MOTOR_NUM)
    float homing_speed; // 回零参考斜率 rad/s【关节域】
    float homing_tol;   // 到点阈值 rad【关节域】, 全部关节到点才 HOMING->RUN
} Grab_Param_s;

/**
 * @brief 单关节机械参数包 (逐轴, config 侧).
 *        一个关节 = 一台驱动电机 + 机械参数; 将来的行程限位/契约角零点/重力补偿系数都住这里.
 */
typedef struct
{
    Motor_Init_Config_s motor_config; // 该关节驱动电机的框架初始化块
    float reduction;                  // θ电机/θ关节 换算比, 符号含方向; 直连=1.0, 带轮填实测比
                                      // TODO: limit(行程限幅, 迁自 CMD_ANGLE_MAX_x) / q_zero(契约角)
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
    /* 驱动电机实例 */
    DMMotorInstance *motor;
    /* 运行状态 */
    float q;        // 实际关节角 rad【关节域】
    float home_ref;

} Grab_Joint_s;

typedef struct
{
    Grab_Ctrl_Cmd_s grab_ctrl_cmd;
    Grab_Joint_s joint[GRAB_MOTOR_NUM];
    Grab_Param_s grab_param;
    uint32_t homing_dt_cnt;
} GrabInstance;

/**
 * @brief 机械臂组件初始化: 注册 DM 电机, 初始化命令层(上电默认进 GRAB_HOMING)
 *        在 RobotInit 中调用 (内部 DMMotorInit, 发送线程由 os_task.c 的 DMMotorTaskInit 创建)
 */
GrabInstance *GrabInit(Grab_Init_Config_s *config);

/**
 * @brief 机械臂任务: 三态状态机执行 + 电机下发.
 *        首次调用完成全部电机使能握手; 之后每周期刷新关节反馈 q, 再按 grab_mode 分支.
 *        在 RobotTask 中以固定频率调用.
 */
void GrabTask();

/**
 * @brief 进入 HOMING 的入口
 *        enable(翻回 MOTOR_ENALBED, 防 ESTOP 单向门) + ref := 当前关节角(MotorToJoint 换算)
 *        (初始误差 0 -> 进闭环瞬间无力矩跳变) + 清积分.
 */
void GrabHomingEntry();
