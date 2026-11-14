//
// Created by CX330y on 2026/11/11.
//

#ifndef CONTROL_2026_SPRING_LOADER_H
#define CONTROL_2026_SPRING_LOADER_H

/**
 * @brief 注册左右两台 M3508 电机，使电机保持停机，并订阅动作指令。
 */
void SpringLoaderInit(void);

/**
 * @brief 处理上电找零、左摇杆上簧和手动回位。
 */
void SpringLoaderTask(void);



#endif // CONTROL_2026_SPRING_LOADER_H
