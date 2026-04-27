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
    float leg_length_delta; // 腿长增量
    float roll_delta;       // Roll(横滚/pike) 增量

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
