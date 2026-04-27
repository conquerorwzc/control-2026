#ifndef INFANTRY_CTRL_H
#define INFANTRY_CTRL_H

#include "robot.h"
#include "user_lib.h"

#define robot_lost_control (abs(robot->chassis->imu->Pitch) > 13.0f)  //todo: 12?

#define has_non_zero_data(data) \
    (data != NULL) && \
    (data->gimbal_receive.yaw != 0 || data->gimbal_receive.pitch != 0 || data->shoot_receive.fire_flag != 0)

/**
 * @brief 抽象控制意图，隔离输入设备（遥控器/键鼠）与运动控制逻辑
 */
typedef struct {
    // === 底盘意图 ===
    float vx;               // 底盘 X 向期望速度 (左右)，归一化 [-1.0, 1.0]
    float vy;               // 底盘 Y 向期望速度 (前后)，归一化 [-1.0, 1.0]
    float rotate_coff;      // 小陀螺旋转系数
    float leg_length_delta; // 腿长增量
    float roll_delta;       // Roll(横滚/pike) 增量

    // === 云台意图 ===
    float yaw_delta;        // 云台 Yaw 轴期望增量
    float pitch_delta;      // 云台 Pitch 轴期望增量
    uint8_t use_absolute_angle; // 是否使用绝对角度（自瞄使用）
    float absolute_yaw;         // 云台 Yaw 绝对角度
    float absolute_pitch;       // 云台 Pitch 绝对角度

    // === 射击意图 ===
    uint8_t shoot_flag;     // 射击开火标志 (1: 单发, 2: 连发, 0: 停火)

    // 特殊指令标志
    uint8_t trigger_jump;     // 触发跳跃
    uint8_t toggle_recovery;  // 触发自起
    uint8_t toggle_supercap;  // 切换超级电容
    
} Ctrl_Intent_s;

/**
 * @brief Control logic for Remote Controller mode.
 * @param robot Pointer to the RobotInstance.
 */
void JoyStickCtrl(RobotInstance* robot);

/**
 * @brief Control logic for Mouse and Keyboard mode.
 * @param robot Pointer to the RobotInstance.
 */
void MouseKeyCtrl(RobotInstance* robot);

/**
 * @brief Emergency stop handler.
 * @param robot Pointer to the RobotInstance.
 */
void EmergencyHandler(RobotInstance* robot);

#endif // INFANTRY_CTRL_H
