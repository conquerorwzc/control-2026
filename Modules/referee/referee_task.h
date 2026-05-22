#ifndef REFEREE_H
#define REFEREE_H

#include "gimbal.h"
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

// 自瞄相机画面框配置
// 图传相机参数 (已知)
#define TX_CAM_FOV_H        139.0f    // 图传相机水平FOV (度)
#define TX_CAM_FOCAL_LEN    12.0f     // 图传相机等效焦距 (mm)

// 自瞄相机参数 (需要根据实际配置修改)
// 焦距越小，FOV越大，框越大
#define AUTOAIM_CAM_FOCAL_LEN  35.0f  // 自瞄相机等效焦距 (mm)，需要根据实际配置修改
#define AUTOAIM_CAM_PITCH_DEG  0.0f  // 自瞄相机相对图传相机的pitch安装角 (度)，正值表示自瞄相机朝上

// 传感器假设 (16:9宽高比)
#define SENSOR_RATIO_W      16.0f
#define SENSOR_RATIO_H      9.0f

// 圆心
#define CENTER_X              960
#define CENTER_Y              540

// 辅助瞄准线相对于准心的偏移量
#define Aim_Line_1            -30
#define Aim_Line_2            -60
#define Aim_Line_3            -90
#define Aim_Line_4            -120

// #define QQ_SUPER_CAP
typedef enum
{
    LID_CLOSE = 0,
    LID_OPEN
} lid_mode_e;

typedef enum
{
    POWEROFF = 0,
    FOLLOW,
    FOLLOW_45,
    ROTATE,
} chassis_mode_e;

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
    uint16_t ui_init;
    Referee_Interactive_Flag_t Referee_Interactive_Flag;
    // 为UI绘制以及交互数据所用
    Chassis_Mode_e chassis_mode;    // 底盘模式
    Gimbal_Mode_e gimbal_mode;      // 云台模式
    Shoot_Mode_e shoot_mode;        // 发射模式设置
    Friction_Mode_e friction_mode;  // 摩擦轮关闭
    Vision_Mode_e vision_mode;
    lid_mode_e load_mode;            // 弹舱盖打开
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
    Chassis_Mode_e chassis_last_mode;
    Gimbal_Mode_e gimbal_last_mode;
    Shoot_Mode_e shoot_last_mode;
    Friction_Mode_e friction_last_mode;
    Vision_Mode_e vision_last_mode;
    lid_mode_e load_last_mode;
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
