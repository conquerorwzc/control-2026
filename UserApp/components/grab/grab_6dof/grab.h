#pragma once

#include "dmmotor.h"

/* 电机台数上限; 实际初始化台数由 Grab_Init_Config_s.motor_num 决定(缺席的台不初始化不握手) */
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
 */
typedef struct
{
    float cmd_angle[GRAB_MOTOR_NUM]; // 角度外环目标, rad, 绝对值(回中=0)
    float cmd_speed[GRAB_MOTOR_NUM]; // 速度外环目标, rad/s, 绝对值
    Grab_Mode_e grab_mode;
} Grab_Ctrl_Cmd_s;

typedef struct
{
    float homing_speed; // 回零参考斜率 rad/s
    float homing_tol;   // 到点阈值 rad, 全部电机到点才 HOMING->RUN
} Grab_Param_s;

typedef struct
{
    uint8_t motor_num; // 实际使用的电机台数 (<= GRAB_MOTOR_NUM)
    Motor_Init_Config_s motor_config[GRAB_MOTOR_NUM];
    Grab_Param_s grab_param;
} Grab_Init_Config_s;

typedef struct
{
    Grab_Ctrl_Cmd_s grab_ctrl_cmd;
    DMMotorInstance *motor[GRAB_MOTOR_NUM];
    uint8_t motor_num;
    Grab_Param_s grab_param;

    /* homing 运行时状态 */
    float home_ref[GRAB_MOTOR_NUM]; // 每台电机的斜坡参考(由 GrabHomingEntry 锚定为当前 total_angle)
    uint32_t homing_dt_cnt;         // DWT 时间戳, 斜坡按真实 dt 推进, 抗调度抖动
} GrabInstance;

/**
 * @brief 机械臂组件初始化: 注册 DM 电机, 初始化命令层(上电默认进 GRAB_HOMING)
 *        在 RobotInit 中调用 (内部 DMMotorInit, 发送线程由 os_task.c 的 DMMotorTaskInit 创建)
 */
GrabInstance *GrabInit(Grab_Init_Config_s *config);

/**
 * @brief 机械臂任务: 三态状态机执行 + 电机下发.
 *        首次调用完成全部电机使能握手(有界阻塞); 之后每周期按 grab_mode 分支.
 *        在 RobotTask 中以固定频率调用.
 */
void GrabTask();

/**
 * @brief 进入 HOMING 的入口
 *        enable(翻回 MOTOR_ENALBED, 防 ESTOP 单向门) + ref := 当前 total_angle
 *        (初始误差 0 -> 进闭环瞬间无力矩跳变) + 清积分.
 */
void GrabHomingEntry();
