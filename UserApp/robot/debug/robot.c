#include "robot.h"
#include "dmmotor.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"

float target_torque = 0;
const float reduction_ratio = 268.0f / 17.0f;
float q2i_coeff = (3591.0f / 187.0f) / reduction_ratio / 0.3f;

// static DMMotorInstance* J8009P_instance;
static DJIMotorInstance *M3508_instance;
static DJIMotorInstance *M3508_instance_2;
static DJIMotorInstance *M2006_instance;
static DJIMotorInstance *M2006_instance_2;
static DJIMotorInstance *GM6020_instance;

float speed_ref = 0.0f;
float speed_ref_2= 0.0f;

void RobotInit()
{
    // J8009P_instance = DMMotorInit(&J8009P_config);
    // M3508_instance = DJIMotorInit(&M3508_config);
    // M3508_instance_2 = DJIMotorInit(&M3508_config_2);
    // M2006_instance = DJIMotorInit(&M2006_config);
    // M2006_instance_2 = DJIMotorInit(&M2006_config_2);
    GM6020_instance = DJIMotorInit(&GM6020_config);
}

void RobotTask()
{
    // DMMotorSetPIDRef(J8009P_instance, speed_ref);
    // M3508_instance->motor_controller.final_output = target_torque * q2i_coeff * (16384.0f / 20.0f);
    // DJIMotorSetPIDRef(M3508_instance, speed_ref);
    // DJIMotorSetPIDRef(M3508_instance_2, speed_ref);
    // DJIMotorSetPIDRef(M2006_instance, speed_ref * 49.1f);
    // DJIMotorSetPIDRef(M2006_instance_2, speed_ref * 49.1f);
    DJIMotorSetPIDRef(GM6020_instance, 0);
}
