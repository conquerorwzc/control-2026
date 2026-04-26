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
    float vx;           // 归一化底盘 X 向期望速度 (左右)
    float vy;           // 归一化底盘 Y 向期望速度 (前后)
    float yaw_delta;    // 云台 Yaw 轴期望增量
    float pitch_delta;  // 云台 Pitch 轴期望增量
    
    float rotate_coff;  // 小陀螺旋转系数
    
    uint8_t shoot_flag; // 射击开火标志 (1: 单发, 2: 连发, 0: 停火)
    uint8_t right_click;// 右键/自瞄标志
    
    float leg_length_delta; // 腿长增量
    float roll_delta;       // Roll(横滚/pike) 增量
    
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
