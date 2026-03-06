#pragma once
#include "dji_motor.h"
#include "dmmotor.h"
#include "general_def.h"

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

typedef enum {
    CALI_IDLE = 0,
    CALI_FIND_MIN,    // 寻找 0 度 (负向)
    CALI_FIND_MAX,    // 寻找 180 度 (正向)
    CALI_SUCCESS      // 校准成功
} Cali_State_e;


typedef enum {
    CALI_STAGE_DM_WAIT_ZERO = 0, // 0: 阶段一 - DM大臂等待物理归零
    CALI_STAGE_WRIST_STALL  = 1, // 1: 阶段二 - 2006腕部 Pitch 抬头堵转检测
    CALI_STAGE_DONE         = 2, // 2: 标定大功告成
    CALI_STAGE_ERROR        = 3  // 3: 标定超时或异常
} GrabCaliStage_e;

typedef struct {
    Cali_State_e state;
    float min_total_angle;
    float max_total_angle;
    float range;
} Motor_Cali_Data_s;

typedef struct {
    float wrist_roll_MAX;// 腕部关节旋转角度
    float wrist_roll_MIN;
    float wrist_pitch_MAX;   // 腕部关节俯仰角度
    float wrist_pitch_MIN;
    float base_joint_MAX;    // 基座旋转关节角度
    float base_joint_MIN;
    float elbow_roll_MAX;   // 肘部关节旋转角度
    float elbow_roll_MIN;
    float elbow_pitch_MAX;   // 肘部关节俯仰角度
    float elbow_pitch_MIN;
    float Video_forward_MAX;// 图传的前后移动距离
    float Video_forward_MIN;
    float Video_pitch_MAX;   // 图传的pitch旋转角度
    float Video_pitch_MIN;

    float base_joint_sens_keyboard;   // 基座旋转关节灵敏度(键鼠)
    float elbow_roll_sens_keyboard;   // 肘部旋转关节灵敏度(键鼠)
    float elbow_pitch_sens_keyboard;  // 肘部俯仰关节灵敏度(键鼠)
    float wrist_roll_sens_keyboard;   // 腕部旋转关节灵敏度(键鼠)
    float wrist_pitch_sens_keyboard;  // 腕部俯仰关节灵敏度(键鼠)
    float video_forward_sens_keyboard; // 图传前后移动灵敏度(键鼠)
    float video_pitch_sens_keyboard;   // 图传pitch旋转灵敏度(键鼠)


} Grab_Param_s;

typedef struct
{
    Motor_Init_Config_s Grab_motor_config[9]; // 修改为数组以支持多个电机
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
    float video_forward; // 图传的前后移动距离
    float video_pitch;   // 图传的pitch旋转角度
    float torque;        // 夹爪电机目标扭矩
    Grab_Mode_e grab_mode;
} Grab_Ctrl_Cmd_s;


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
    float base_joint;  // 基座旋转关节角度
    float elbow_roll;  // 肘部关节旋转角度
    float elbow_pitch; // 肘部关节俯仰角度
} ArmInstance;

typedef struct
{
    DJIMotorInstance *grab_djimotor[2];
    float Video_forward; // 图传的前后移动距离
    float Video_pitch;   // 图传的pitch旋转角度
    float F_target;      // 前后移动电机目标角度
    float P_target;      // pitch轴电机目标角度
} VideoInstance;

typedef struct
{
    Grab_Ctrl_Cmd_s grab_ctrl_cmd;
    ArmInstance *arm;
    ActuatorInstance *actuator;
    VideoInstance *video;
} GrabInstance;

GrabInstance *GrabInit(Grab_Init_Config_s *Grab_init_config);

void GrabTask();
