//
// Created by 99242 on 2026/11/14.
//

#ifndef CONTROL_2026_ROBOT_H
#define CONTROL_2026_ROBOT_H

#include "wing.h"

typedef struct {
    wing_typedef wing;   // 4 路舵面 + 混控
} RobotInstance;
//初始化实例
void RobotInit(void);
//舵机控制任务
void RobotTask(void);
#endif // CONTROL_2026_ROBOT_H
