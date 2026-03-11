#ifndef INFANTRY_CTRL_H
#define INFANTRY_CTRL_H

#include "robot.h"
#include "user_lib.h"

// Ramp controller shared with other modules (e.g. for emergency stop)
extern Ramp_Controller_t chassis_ramp;

/**
 * @brief Initialize the control module.
 *        Allocates memory for rc_data_last and sets up internal pointers.
 * @param robot Pointer to the RobotInstance.
 */
void CtrlInit(RobotInstance* robot);

/**
 * @brief Control logic for Remote Controller mode.
 * @param robot Pointer to the RobotInstance.
 */
void RemoteControlSet(RobotInstance* robot);

/**
 * @brief Control logic for Mouse and Keyboard mode.
 * @param robot Pointer to the RobotInstance.
 */
void MouseKeySet(RobotInstance* robot);

#endif // INFANTRY_CTRL_H
