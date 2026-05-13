#pragma once
#include "bsp_gpio.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "general_def.h"
#include "stm32h7xx_hal.h"

typedef enum
{
    GRAB_POWER_OFF = 0, // 电流零输入
    GRAB_POWER_ON
} Grab_Mode_e;

typedef enum
{
    GRAB_PRE_CALI_MODE = 0,
    GRAB_CALI_MODE,
} Grab_Cali_Mode_e;

typedef enum
{
    GRAB_NO_ERROR = 0,
    GRAB_ERR_ROLL_OVERSPEED = 1, // Roll轴疯转
    GRAB_ERR_ROLL_OVERANGLE = 2  // Roll轴角度超限
} Grab_Error_e;

typedef enum
{
    GRIPPER_OPEN = 0, // 夹爪松开
    GRIPPER_CLOSE = 1 // 夹爪夹紧
} GripperState_e;
// =========================================================
// 面向对象的标定基类定义
// =========================================================
typedef enum
{
    CALI_RUNNING = 0, // 正在执行标定
    CALI_DONE = 1,    // 标定完成
    CALI_ERROR = 2    // 标定异常
} GeneralCaliState_e;

typedef struct Calibration_t Calibration_t;

struct Calibration_t
{
    GeneralCaliState_e state;                   // 对外暴露的状态
    uint32_t timeout_cnt;                       // 当前超时计数
    uint32_t max_timeout;                       // 最大允许超时时间
    uint8_t internal_step;                      // 内部子步骤 (用于记录特有状态机进度)
    void *host_ptr;                             // 宿主指针 (存放 GrabInstance 指针)
    void (*Execute_Logic)(Calibration_t *self); // 虚函数指针 (多态调用)
};

// =========================================================
// 机械臂全局参数配置 (宏定义全部转移至此)
// =========================================================
typedef struct
{
    // 1. 软件限位与键盘灵敏度 (原有)
    float wrist_roll_MAX;
    float wrist_roll_MIN;
    float wrist_pitch_MAX;
    float wrist_pitch_MIN;
    float base_joint_MAX;
    float base_joint_MIN;
    float elbow_roll_MAX;
    float elbow_roll_MIN;
    float elbow_pitch_MAX;
    float elbow_pitch_MIN;
    float arm_lift_MAX;
    float arm_lift_MIN;

    float base_joint_sens_keyboard;
    float elbow_roll_sens_keyboard;
    float elbow_pitch_sens_keyboard;
    float wrist_roll_sens_keyboard;
    float wrist_pitch_sens_keyboard;
    float arm_lift_sens_keyboard;
    float arm_extend_sens_keyboard;

    float elbow_pitch_max;
    float elbow_pitch_min;
    float base_joint_max;
    float base_joint_min;
    float elbow_roll_max;
    float elbow_roll_min;
    float arm_lift_max;

    float gripper_close_torque; // 夹紧时的扭矩 (如 2.0f)
    float gripper_open_torque;  // 松开时的扭矩 (如 -0.6f)

    // 2. 物理减速与传动比参数
    float pulley_gear_ratio;             // 带轮传动比
    float bevel_gear_ratio;              // 锥齿轮传动比
    float planar_gear_ratio;             // 平面齿轮传动比
    float motor2006_reduction_ratio;     // 2006减速比
    float motor3508_p51_reduction_ratio; // 3508抬升减速比
    float motor3508_p19_reduction_ratio; // 3508前伸减速比

    // 3. 标定超时与速度参数
    float dm_homing_tolerance;  // DM归零容差
    uint32_t dm_cali_max_ticks; // DM标定超时时间

    uint32_t wrist_cali_max_ticks;   // 腕部标定超时
    float wrist_cali_speed;          // 腕部标定速度
    uint32_t wrist_cali_check_ticks; // 腕部堵转检测周期
    float wrist_cali_tolerance;      // 腕部堵转容差
    float wrist_cali_stall_current;  // 腕部堵转电流阈值

    uint32_t extend_cali_max_ticks; // 前伸标定超时
    float extend_cali_speed;        // 前伸标定速度

    // 4. 硬件挂载开关与安全配置
    uint8_t use_wrist_stall_cali;  // 1: 开启自动堵转标定 0: 关闭
    uint8_t use_wrist_left_motor;  // 1: 启用左侧电机 0: 卸力断电
    uint8_t use_wrist_right_motor; // 1: 启用右侧电机 0: 卸力断电
    float wrist_soft_limit_margin; // 腕部软限位安全系数
} Grab_Param_s;

typedef struct
{
    Motor_Init_Config_s Grab_motor_config[9];
    Grab_Cali_Mode_e Grab_cali_mode;
    Grab_Param_s Grab_param;
} Grab_Init_Config_s;

typedef struct
{
    float wrist_roll;
    float wrist_pitch;
    float base_joint;
    float elbow_roll;
    float elbow_pitch;
    GripperState_e gripper_state;

    float arm_lift;
    float arm_lift_target;

    float arm_extend;
    float arm_extend_target;
    uint8_t wrist_roll_cali;
    uint8_t wrist_pitch_cali;

    uint8_t is_climb_mode;

    Grab_Mode_e grab_mode;
} Grab_Ctrl_Cmd_s;

typedef struct
{
    float wrist_roll;
    float wrist_pitch;
    float base_joint;
    float elbow_roll;
    float elbow_pitch;
    float torque;

    float arm_lift;
    float arm_extend;

    uint8_t micro_switch_state;
} Grab_Real_Measure_s;

// 执行器：腕部+夹爪
typedef struct
{
    DJIMotorInstance *grab_djimotor[3];
    DMMotorInstance *grab_dmmotor[1];
    float wrist_roll;
    float wrist_pitch;
    float gripper_joint;
    GripperState_e gripper_state;
    float L_target;
    float R_target;
    float M_target;
    float T_target;

    // OOP：腕部标定对象与软限位
    Calibration_t wrist_cali_obj;
    float max_pitch;
    float min_pitch;
} ActuatorInstance;

// 大臂：底盘+肘部+抬升+前伸
typedef struct
{
    DMMotorInstance *grab_dmmotor[3];
    DJIMotorInstance *arm_lift_motor;
    DJIMotorInstance *arm_extend_motor;
    GPIOInstance *micro_switch_gpio;

    // OOP：前伸标定对象与软限位
    Calibration_t extend_cali_obj;
    float max_extend;
    float min_extend;

    float base_joint;
    float elbow_roll;
    float elbow_pitch;
    float arm_lift;
    float arm_lift_max;
    float arm_lift_min;
} ArmInstance;

typedef struct
{
    Grab_Ctrl_Cmd_s grab_ctrl_cmd;
    Grab_Real_Measure_s grab_measure;
    ArmInstance *arm;
    ActuatorInstance *actuator;
    Grab_Error_e error_code;
} GrabInstance;

/* 外部函数声明 */
GrabInstance *GrabInit(Grab_Init_Config_s *Grab_init_config);
void GrabTask();
void Execute_Calibration(Calibration_t *cali_obj);