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
 * @brief 上电后驱动两侧找零，回零完成后接收左摇杆指令并执行下拉和回程。
 */
void SpringLoaderTask(void);



#endif // CONTROL_2026_SPRING_LOADER_H
