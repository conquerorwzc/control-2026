#include "vofa.h"

#include <stdint.h>

#include "robot.h"
#include "usart.h"
// 本目录已有同名vofa.h，模块层入口用相对路径引入，避开include搜索顺序歧义。
#include "../../../Modules/vofa/vofa.h"

#if 0 // 保留原摩擦轮采集代码，当前使用下方的双 Pitch 采集。
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
#else

#include "bsp_dwt.h"
#include "cmsis_os.h"

void VofaInit(void)
{
    huart1.Init.BaudRate = VOFA_BAUDRATE;
    HAL_UART_Init(&huart1);
    VOFAInit(&huart1);
}

void VofaTick(void)
{
    static uint32_t last_tick;
    static uint32_t sample_count;
    uint32_t now = osKernelSysTick();
    if ((uint32_t)(now - last_tick) < VOFA_SAMPLE_PERIOD_MS)
        return;
    // 不补发积压帧；发送周期至少 2 ms，保持 RobotTask 的控制频率。
    last_tick = now;
    uint32_t sequence = sample_count++;

    RobotInstance *robot = RobotGet();
    GimbalInstance *gimbal = robot->gimbal;
    if (gimbal == NULL || robot->rc_data == NULL)
        return;
    // module 用同一静态缓冲发 DMA，必须在复制新数据前检查空闲。
    // 忙时丢弃本次采样，下一帧序号会跳变；不等待、不阻塞控制。
    if (huart1.gState != HAL_UART_STATE_READY)
        return;

    DMMotorInstance *big = gimbal->big_pitch_motor;
    DMMotorInstance *small = gimbal->small_pitch_motor;
    INS_t *imu = gimbal->gimbal_IMU_data;
    RC_ctrl_t rc = robot->rc_data[TEMP];
    float ch[VOFA_CHANNEL_NUM];
    ch[0] = DWT_GetTimeline_ms();
    ch[1] = (float)sequence;
    // 直接记录遥控值，不推断另一块板是否实际出弹。
    ch[2] = (float)rc.rc.switch_left;
    ch[3] = (float)rc.rc.switch_right;
    ch[4] = (float)rc.rc.dial;

    ch[5] = big->motor_controller.angle_PID.Ref;
    ch[6] = big->measure.total_angle;
    ch[7] = big->motor_controller.speed_PID.Ref;
    ch[8] = big->measure.velocity;
    ch[9] = big->motor_controller.final_output;
    ch[10] = big->motor_controller.speed_PID.Iout;
    ch[11] = gimbal->gravity_feedforward.big_pitch_feedforward;

    ch[12] = gimbal->gimbal_ctrl_cmd.pitch;
    ch[13] = imu->Pitch;
    ch[14] = small->measure.total_angle;
    ch[15] = small->motor_controller.speed_PID.Ref;
    ch[16] = imu->Gyro[0];
    ch[17] = small->measure.velocity;
    ch[18] = small->motor_controller.final_output;
    ch[19] = small->motor_controller.speed_PID.Iout;
    ch[20] = gimbal->gravity_feedforward.small_pitch_feedforward;
    ch[21] = imu->Accel[0];
    ch[22] = imu->Accel[1];
    ch[23] = imu->Accel[2];
    ch[24] = (float)rc.rc.rocker_r1;
    VOFAJustFloatSend(ch, VOFA_CHANNEL_NUM);
}
#endif
