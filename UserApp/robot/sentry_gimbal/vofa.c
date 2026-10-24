#include "vofa.h"

#include <stdint.h>

#include "robot.h"
#include "usart.h"
// 本目录已有同名vofa.h，模块层入口用相对路径引入，避开include搜索顺序歧义。
#include "../../../Modules/vofa/vofa.h"

#define VOFA_CHANNEL_NUM 16U

#if FRICTION_NUM != 3
#error "通道布局按三轮摩擦轮设计，FRICTION_NUM变动需同步修改vofa.h通道表"
#endif

void VofaInit(void)
{
    // 覆盖CubeMX默认115200；huart1当前无其他占用，日后接裁判系统时需挪到huart6。
    huart1.Init.BaudRate = VOFA_BAUDRATE;
    HAL_UART_Init(&huart1);
    VOFAInit(&huart1);
}

void VofaTick(void)
{
    ShootInstance *shoot = RobotGet()->shoot;
    if (shoot == NULL)
        return;
    static uint32_t frame_count;
    float ch[VOFA_CHANNEL_NUM];
    ch[0] = shoot->friction_set;
    for (uint8_t i = 0; i < FRICTION_NUM; ++i)
    {
        DJIMotorInstance *motor = shoot->friction_motor[i];
        ch[1 + i] = motor->measure.speed_aps;
        ch[4 + i] = motor->motor_controller.final_output * 0.001f; // 1000指令≈1A
        ch[7 + i] = motor->motor_controller.speed_PID.Iout * 0.001f;
        ch[10 + i] = (float)motor->measure.real_current;
    }
    ch[13] = shoot->loader_motor->measure.total_angle;
    ch[14] = shoot->loader_motor->measure.velocity;
    ch[15] = (float)(frame_count++);
    VOFAJustFloatSend(ch, VOFA_CHANNEL_NUM);
}
