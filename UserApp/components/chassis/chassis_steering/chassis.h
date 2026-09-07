//
// 由 yang6 创建于 2025/11/17.
//

#ifndef CONTROL_2026_CHASSIS_H
#define CONTROL_2026_CHASSIS_H

#include <stdint.h>

#include "dji_motor.h"
#include "general_def.h"
#include "super_cap.h"

#define DEG2R(x) ((x) * PI / 180.0f)

/**
 * @brief 所有轮组/舵向数组统一使用的四个舵轮模块顺序。
 *
 * 该顺序需要和 robot_config.h 保持一致：
 * LF -> LB -> RB -> RF.
 */
typedef enum
{
    LF = 0,
    LB,
    RB,
    RF,
    WHEEL_COUNT,
} Chassis_Wheel_Index_e;

#define MAX_WHEEL_SPEED 40000.0f

typedef enum
{
    CHASSIS_POWER_OFF = 0,  // 停止轮电机和舵电机输出
    CHASSIS_ROTATE,         // 小陀螺模式，同时保留平移控制
    CHASSIS_FOLLOW,         // 跟随云台零位
    CHASSIS_FOLLOW_DIAGONAL, // 跟随云台并保持 45 度底盘偏置
    CHASSIS_FREE,           // 不做云台跟随补偿
} Chassis_Mode_e;

#pragma pack(1)
typedef struct
{
    float vx; // 云台坐标系下的前后速度指令
    float vy; // 云台坐标系下的左右速度指令
    float wz; // 偏航角速度指令，跟随 PID 会在此基础上叠加反馈
    Chassis_Mode_e chassis_mode;
    float offset_angle; // 底盘朝向相对云台朝向的偏角，单位：度
    int chassis_speed_buff;
    uint16_t max_power;  // 经过超电策略处理后的底盘功率预算
    uint8_t SuperCapBoost; // bit0：超电加速请求，bit1：UI 重新初始化触发
} Chassis_Ctrl_Cmd_s;
#pragma pack()

typedef enum
{
    SAFETY_MODE = 0,      // 电容电压过低，底盘保持保守输出
    PASSIVE_MODE,         // 常规模式，使用裁判系统功率限制
    ACTIVE_MODE,          // 主动使用超级电容能量
    CHARGING_MODE,        // 降低底盘功率以恢复电容电压
    FORCED_CHARGING_MODE, // 电容接近低压阈值时进一步降额
} SuperCapMode;

typedef struct
{
    float k0;
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
} Power_Param_3508_s;

typedef struct
{
    float k0;
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
} Power_Param_6020_s;

typedef struct
{
    float wheel_base;             // 纵向轴距，单位：mm
    float track_width;            // 横向轮距，单位：mm
    float center_gimbal_offset_x; // 云台中心相对底盘中心的 x 轴偏移，单位：mm
    float center_gimbal_offset_y; // 云台中心相对底盘中心的 y 轴偏移，单位：mm
    float wheel_radius;
    float wheel_reduction_ratio;
    Power_Param_3508_s power_param; // 3508 轮电机功率模型参数
    Power_Param_6020_s power_param_6020;
    uint16_t rudder_motor_offset[WHEEL_COUNT]; // 6020 舵电机零位编码器偏移
} Chassis_Param_s;

typedef struct
{
    Chassis_Param_s chassis_param;
    Motor_Init_Config_s rudder_motor_config[WHEEL_COUNT];
    Motor_Init_Config_s wheel_motor_config[WHEEL_COUNT];
    Motor_Init_Config_s yaw_motor_config;
    PID_Init_Config_s rudder_angle_pid_config;
    PID_Init_Config_s rudder_speed_pid_config;
    PID_Init_Config_s driver_speed_pid_config;
    PID_Init_Config_s follow_pid;
    PID_Init_Config_s planar_motion_pid_config;
    PID_Init_Config_s rotate_pid_config;
    SuperCap_Init_Config_s super_cap_config;
} Chassis_Init_Config_s;

typedef struct
{
    Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
    DJIMotorInstance *wheel_motor[WHEEL_COUNT];
    DJIMotorInstance *rudder_motor[WHEEL_COUNT];
    uint16_t rudder_offset[WHEEL_COUNT];
    SuperCapInstance *super_cap;
    SuperCapMode super_cap_mode;
} ChassisInstance;

/**
 * @brief 初始化舵轮底盘电机、跟随 PID、裁判系统指针和超电模块。
 *
 * @return 底盘运行时实例；内存分配失败时返回 NULL。
 */
ChassisInstance *ChassisInit(Chassis_Init_Config_s *chassis_init_config);

/**
 * @brief 周期性底盘控制入口，需要在 ChassisInit() 之后调用。
 */
void ChassisTask(void);

#endif // CONTROL_2026_CHASSIS_H
