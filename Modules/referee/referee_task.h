#ifndef REFEREE_H
#define REFEREE_H

#include "rm_referee.h"

//车辆示宽线
#define WIDTHLINE_UP                330  //上方间距
#define WIDTHLINE_DOWN              610  //下方间距

// 摩擦轮转速范围
#define FRIC_LOWER 30000
#define FRIC_UPPER 50000

// 电容电压范围
#define CAP_VOL_LOWER 16.0f
#define CAP_VOL_UPPER 23.0f

// 弹量上限
#define AMMO_UPPER 500

// #define QQ_SUPER_CAP
typedef enum
{
    LID_CLOSE = 0,
    LID_OPEN
} lid_mode_e;

typedef struct
{
    float chassis_power_mx; // 最大功率限制
    float chassis_power;    // 当前功率
} Chassis_Power_Data_s;
/**
 * @brief 初始化裁判系统交互任务(UI和多机通信)
 *
 */
typedef struct
{
    Referee_Interactive_Flag_t Referee_Interactive_Flag;
    // 为UI绘制以及交互数据所用
    Gimbal_Mode_e gimbal_mode;      // 云台模式
    Shoot_Mode_e shoot_mode;        // 发射模式设置
    Friction_Mode_e friction_mode;  // 摩擦轮关闭
    lid_mode_e lid_mode;            // 弹舱盖打开
    Chassis_Power_Data_s Chassis_Power_Data; // 功率控制
    float pitch_angle; // 云台俯仰角
    uint8_t Shoot_heat;
    float Shoot_rate;
    uint8_t autoaim_mode;           // 当前自瞄模式 (0/1/2)
    float cap_voltage;               // 电容电压 (V)
    uint8_t cap_mode;                // 电容模式 (0关/1开/2超级)
    uint16_t bullet_left_real;       // 实体弹丸剩余量
    uint16_t fric_speed_left;         // 左摩擦轮转速
    uint16_t fric_speed_right;        // 右摩擦轮转速
    float chassis_relative_angle;    // 底盘相对角度 (弧度)

    // 上一次的模式，用于flag判断
    Gimbal_Mode_e gimbal_last_mode;
    Shoot_Mode_e shoot_last_mode;
    Friction_Mode_e friction_last_mode;
    lid_mode_e lid_last_mode;
    Chassis_Power_Data_s Chassis_last_Power_Data;
    float last_pitch_angle; // 上一次的俯仰角
    uint8_t last_Shoot_heat;
    float last_Shoot_rate;
    uint8_t last_autoaim_mode;
    float last_cap_voltage;
    uint8_t last_cap_mode;
    uint16_t last_bullet_left_real;
    uint16_t last_fric_speed_left;
    uint16_t last_fric_speed_right;
    float last_chassis_relative_angle;
} Referee_Interactive_info_t;
/**
 * @brief 在referee task之前调用,添加在freertos.c中
 * 
 */
void MyUIInit();

/**
 * @brief 裁判系统交互任务(UI和多机通信)
 *
 */
void UITask();
Referee_Interactive_info_t* getUI();

#endif // REFEREE_H
