#include "robot.h"
#include "robot_config.h"

void RobotInit(void)
{
    // FW0：不注册 DMMotorInit，现有实现会自动使能。
}

void RobotTask(void)
{
    // 激励数学内核已可测试；等采集、门控和实车限位接入后再连接电机。
}
