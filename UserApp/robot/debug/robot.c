#include "robot.h"
#include "dmmotor.h"
#include "dji_motor.h"
#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"

static DMMotorInstance* DM4310_instance;
static DJIMotorInstance* M3508_instance_1;
static DJIMotorInstance* M3508_instance_2;
static DJIMotorInstance* M2006_instance;

void RobotInit() {
    // 初始化DM4310电机
    DM4310_instance = DMMotorInit(&DM4310_config);
    
    // 初始化第一个3508电机
    M3508_instance_1 = DJIMotorInit(&M3508_config_1);
    
    // 初始化第二个3508电机
    M3508_instance_2 = DJIMotorInit(&M3508_config_2);

    // 初始化2006电机
    M2006_instance = DJIMotorInit(&M2006_config);
}

void RobotTask() {

}
