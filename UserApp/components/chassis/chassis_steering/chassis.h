//
// 由 yang6 创建于 2025/11/17.
//
/**
 * @file    chassis.h
 * @author  YZC
 * @author  注释与修改：SRM-Control 2026
 * @date    2026/1/1
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   舵轮底盘模块
 */
#pragma once

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
    CHASSIS_POWER_OFF = 0,   // 停止轮电机和舵电机输出
    CHASSIS_ROTATE,          // 小陀螺模式，同时保留平移控制
    CHASSIS_FOLLOW,          // 跟随云台零位
    CHASSIS_FOLLOW_DIAGONAL, // 跟随云台并保持 45 度底盘偏置
    CHASSIS_FREE,            // 不做云台跟随补偿：平移与自转全部由摇杆直接给定
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

/**
 * @brief 底盘初始化配置。
 *
 * @note 超级电容是可选外设：只有当 `super_cap_config.can_config.can_handle` 非空时才会注册。
 *       台架调试(无超电)把该字段留空即可，底盘会自动退化为"固定的裁判系统功率上限"，
 *       既不注册 CAN 实例，也不把超电帧挂到总线上。
 */
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

/* ==========================================================================================
 *  坐标系约定 (解算与 robot 层共用, 推导过程见 chassis.c 里 kSteerPos 上方的注释)
 *
 *      +x = 向右平移        +y = 前进(车头方向)        +wz = 顺时针旋转(俯视)
 *
 *  @attention 这不是数学上常见的右手系(那套是 +x 前 / +y 左 / 逆时针为正),
 *             是本车按队伍习惯固定下来的约定。改任何解算公式前先读 chassis.c 里
 *             "坐标系与轮组位置约定" 那一段, 并按 v_i·r_i = 0 重新核对四个轮子。
 *
 *  @attention 舵机零偏必须在【舵轮指向机械正前方】的位置标定: 解算给出的舵角
 *             以正前方为 0°。若零偏标在"指向右侧"之类的位置, 整车会整体偏 90°,
 *             表现为"推前进却横向平移"。
 *
 *  @attention 【舵角正方向 = 顺时针】必须由舵机反馈的符号保证。本车实测
 *             "舵轮在地上逆时针旋转时 ecd 变大", 与解算约定相反, 因此
 *             RUDDER_MOTOR_CONFIG 里 feedback_reverse_flag 必须取
 *             FEEDBACK_DIRECTION_REVERSE, 把 total_angle 的符号翻过来
 *             (角度环闭环用的正是 total_angle, 见 dji_motor.c 的 DJIMotorControl)。
 *             注意 ForwardKinematicCal 读的是 angle_single_round, 它【不受】该 flag
 *             影响, 所以那里另外显式取了一次负号。
 * ========================================================================================== */

typedef struct
{
    Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
    DJIMotorInstance *wheel_motor[WHEEL_COUNT];
    DJIMotorInstance *rudder_motor[WHEEL_COUNT];
    uint16_t rudder_offset[WHEEL_COUNT];
    SuperCapInstance *super_cap; // 未配置超电时为 NULL
    SuperCapMode super_cap_mode;
} ChassisInstance;

/**
 * @brief 初始化舵轮底盘电机、跟随 PID、裁判系统指针和超电模块。
 *
 * @note 超电未配置(`super_cap_config.can_config.can_handle == NULL`)时跳过注册；
 *       四个 GM6020 的零偏必须全部非 0，否则 ChassisTask() 会拒绝使能舵电机
 *       (可用 ChassisIsRudderReady() 查询)。
 *
 * @return 底盘运行时实例；内存分配失败时返回 NULL。
 */
ChassisInstance *ChassisInit(Chassis_Init_Config_s *chassis_init_config);

/**
 * @brief 周期性底盘控制入口，需要在 ChassisInit() 之后调用。
 */
void ChassisTask(void);

/* ==========================================================================================
 *  只读调试导出 (由 robot 层调用, 不影响控制逻辑)
 *
 *  下标顺序固定为 LF / LB / RB / RF, 与 chassis.c 内部各变量一一对应。
 *  st  : 解算并做完零偏补偿后的【舵角目标】(度) —— 判断"解算算得对不对"的关键量
 *  vt  : 限幅并应用翻转方向后的【轮速参考】
 *  dir : 短弧翻转方向 (1 正转 / -1 反转)
 *
 *  【st 的读数速查】(st 以"舵轮指向机械正前方"为 0°, 指向右侧为 +90°)
 *      纯前进(+vy) -> 四轮全 0°      纯后退(-vy) -> 四轮全 ±180°
 *      纯右移(+vx) -> 四轮全 +90°    纯左移(-vx) -> 四轮全 -90°
 *      纯顺时针自转(+wz) -> LF 180°  LB -90°  RB 0°  RF +90°
 *      静止不动时 st 应保持在 0° 附近(播种值), 若明显偏离说明零偏标定朝向不对。
 * ========================================================================================== */
typedef struct
{
    float chassis_vx;        // 坐标变换后的 x 分量(本车 offset_angle 恒 0, 等于指令 vx)
    float chassis_vy;        // 坐标变换后的 y 分量
    float st[WHEEL_COUNT];   // 舵角目标(度), 顺序 LF LB RB RF
    float vt[WHEEL_COUNT];   // 轮速参考,     顺序 LF LB RB RF
    int8_t dir[WHEEL_COUNT]; // 翻转方向,     顺序 LF LB RB RF
    float cur[WHEEL_COUNT];  // 舵角实测(度, 已减零偏并归一到 ±180), 顺序 LF LB RB RF
    float ecd_deg[WHEEL_COUNT]; // 【原始】编码器角(度, 未减零偏, 0~360), 标定用
} Chassis_Debug_State_s;

/**
 * @brief 取一次解算中间量快照; 传 NULL 安全。
 */
void ChassisGetDebugState(Chassis_Debug_State_s *out);

/* ==========================================================================================
 *  只读状态导出 (由 robot 层调用, 用于上电自检/日志)
 * ========================================================================================== */

/**
 * @brief 查询底盘是否处于"可安全使能舵电机"状态。
 *
 * 以下任一条件不满足都会返回 0，此时 ChassisTask() 会让舵电机保持停机：
 *   - 尚未完成初始化；
 *   - 四个 GM6020 零偏中存在 0(未标定)。
 *
 * @return 1 = 可以驱动舵电机; 0 = 拒绝驱动。
 */
uint8_t ChassisIsRudderReady(void);

/**
 * @brief 设定底盘功率上限(W)。
 *
 * 仅在【没有裁判系统且没有超电】时作为兜底值生效；一旦裁判系统接入，功率预算
 * 就由裁判系统上报值(或超电状态机)决定，本设定值不再影响控制。
 *
 * 典型用法：台架调试时给一个安全的固定值；有裁判系统的整车上不用调用。
 */
void ChassisSetPowerLimit(uint16_t power_limit);

/**
 * @brief 当前生效的底盘功率上限(W)。
 *
 * 无裁判系统、无超电时返回 ChassisSetPowerLimit() 设定的值(默认 80W)，
 * 而不是 0 —— 否则轮电机会被功率环削到没有电流。
 */
uint16_t ChassisGetPowerLimit(void);
