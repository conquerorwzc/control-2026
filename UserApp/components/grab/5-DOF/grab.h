#pragma once
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
    GRAB_ERR_ROLL_OVERSPEED   = 1, // Roll轴疯转
    GRAB_ERR_ROLL_OVERANGLE  = 2  // Roll轴角度超限
} Grab_Error_e;

typedef enum {
    CALI_STAGE_DM_WAIT_ZERO   = 0, // 0: 阶段一 - DM大臂等待物理归零
    CALI_STAGE_WRIST_FIND_MAX = 1, // 1: 阶段二 - Pitch 向上抬，寻找 90度 限位
    CALI_STAGE_WRIST_FIND_MIN = 2, // 2: 阶段三 - Pitch 向下压，寻找最低限位
    CALI_STAGE_DONE           = 3, // 3: 标定大功告成
    CALI_STAGE_ERROR          = 4  // 4: 标定超时或异常
} GrabCaliStage_e;
typedef struct {
    GrabCaliStage_e state;
    float max_pitch; // 自动生成的最高软件限位 (如 90 * 0.98 = 88.2度)
    float min_pitch; // 自动生成的最低软件限位
} Motor_Cali_Data_s;

typedef struct {
    float wrist_roll_MAX;    // 腕部关节旋转角度
    float wrist_roll_MIN;
    float wrist_pitch_MAX;   // 腕部关节俯仰角度
    float wrist_pitch_MIN;
    float base_joint_MAX;    // 基座旋转关节角度
    float base_joint_MIN;
    float elbow_roll_MAX;    // 肘部关节旋转角度
    float elbow_roll_MIN;
    float elbow_pitch_MAX;   // 肘部关节俯仰角度
    float elbow_pitch_MIN;
    // 👇 新增：3508 抬升电机的软限位
    float arm_lift_MAX;      // 机械臂整体抬升最大高度/角度
    float arm_lift_MIN;      // 机械臂整体抬升最小高度/角度

    float base_joint_sens_keyboard;    // 基座旋转关节灵敏度(键鼠)
    float elbow_roll_sens_keyboard;    // 肘部旋转关节灵敏度(键鼠)
    float elbow_pitch_sens_keyboard;   // 肘部俯仰关节灵敏度(键鼠)
    float wrist_roll_sens_keyboard;    // 腕部旋转关节灵敏度(键鼠)
    float wrist_pitch_sens_keyboard;   // 腕部俯仰关节灵敏度(键鼠)
    // 👇 新增：3508 抬升电机的键盘控制灵敏度
    float arm_lift_sens_keyboard;      // 抬升机构灵敏度(键鼠)

    float elbow_pitch_max;
    float elbow_pitch_min;
    float base_joint_max;
    float base_joint_min;
    float elbow_roll_max;
    float elbow_roll_min;
    float arm_lift_max;
} Grab_Param_s;

typedef struct
{
    // 👇 极其重要：从 [9] 改为 [10]！因为多了一个 3508 抬升电机
    Motor_Init_Config_s Grab_motor_config[8];
    Grab_Cali_Mode_e Grab_cali_mode;
    Grab_Param_s  Grab_param;
} Grab_Init_Config_s;

typedef struct
{
    float wrist_roll;    // 腕部关节旋转角度
    float wrist_pitch;   // 腕部关节俯仰角度
    float base_joint;    // 基座旋转关节角度
    float elbow_roll;    // 肘部关节旋转角度
    float elbow_pitch;   // 肘部关节俯仰角度
    float torque;        // 夹爪电机目标扭矩

    // 👇 新增：3508 抬升电机的目标指令
    float arm_lift;      // 机械臂整体抬升的目标角度/高度
    float arm_lift_target;
    uint8_t wrist_roll_cali;
    uint8_t wrist_pitch_cali;
    Grab_Mode_e grab_mode;
} Grab_Ctrl_Cmd_s;

typedef struct
{
    float wrist_roll;    // 实际：腕部关节旋转角度
    float wrist_pitch;   // 实际：腕部关节俯仰角度
    float base_joint;    // 实际：基座旋转关节角度
    float elbow_roll;    // 实际：肘部关节旋转角度
    float elbow_pitch;   // 实际：肘部关节俯仰角度
    float torque;        // 实际：夹爪电机当前扭矩

    // 👇 新增：3508 抬升电机的实际反馈
    float arm_lift;      // 实际：机械臂整体抬升的高度/角度
} Grab_Real_Measure_s;

typedef struct
{
    DJIMotorInstance *grab_djimotor[3];
    DMMotorInstance *grab_dmmotor[1];
    float wrist_roll;    // 腕部关节旋转角度
    float wrist_pitch;   // 腕部关节俯仰角度
    float gripper_joint; // 末端夹爪关节角度
    float torque;        // 夹爪电机目标扭矩
    float L_target;      // 左侧电机旋转角度
    float R_target;      // 右侧电机旋转角度
    float M_target;      // 中间电机旋转角度
    float T_target;      // 夹爪电机目标扭矩
    Motor_Cali_Data_s wrist_cali;
} ActuatorInstance;

typedef struct
{
    DMMotorInstance *grab_dmmotor[3];

    // 👇 新增：3508 大疆电机实例指针 (用于控制机械臂抬升)
    DJIMotorInstance *arm_lift_motor;

    float base_joint;  // 基座旋转关节角度
    float elbow_roll;  // 肘部关节旋转角度
    float elbow_pitch; // 肘部关节俯仰角度
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
    Grab_Error_e error_code;      // 当前实时错误码
} GrabInstance;

GrabInstance *GrabInit(Grab_Init_Config_s *Grab_init_config);

void GrabTask();