#pragma once

#include "chassis.h"
#include "bsp_usart.h"

typedef enum {
    CHASSIS_TUNING_MOTOR = 0,
    CHASSIS_TUNING_VX,
    CHASSIS_TUNING_VY,
    CHASSIS_TUNING_WZ,
    CHASSIS_TUNING_YAW,
    CHASSIS_TUNING_FOUR_MOTORS,
    CHASSIS_TUNING_TRANSLATION_WZ,
    CHASSIS_TUNING_THREE_AXES,
    CHASSIS_TUNING_START_STOP,
} ChassisTuningView_e;

// 两种采集共用一次初始化，由机器人应用固定传入 huart6，并停用裁判串口协议。
void ObserverVOFAInit(UART_HandleTypeDef *huart);
// 在 ChassisTask() 后调用；内部限制为 50 Hz，使用已有 VOFA JustFloat DMA 发送。
void ObserverVOFASend(const ChassisInstance *chassis, uint8_t motor_id);
// 调参页：0 单电机，1/2/3 三轴速度环，4 航向保持，5 四电机，6 平移/wz，7 三轴同步，8 起停诊断。
// 按实际波特率检查 >=20% 带宽余量；460800 下 4ms/250Hz，与噪声函数互斥调用。
void ChassisTuningVOFASend(const ChassisInstance *chassis, ChassisTuningView_e view,
                          uint8_t motor_id, uint32_t period_ms);
