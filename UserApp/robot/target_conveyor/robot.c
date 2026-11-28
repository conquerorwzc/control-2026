/**
 * @file robot.c
 * @brief target_conveyor: M3508 寻找左右挡板并自动往返。
 *
 * 左摇杆上推开始找零，再次上推开始往返；
 * 左摇杆下拨停止，下一次上推继续往返。
 */

#include "robot.h"

#include <math.h>
#include <stdbool.h>

#include "bsp_dwt.h"
#include "controller.h"
#include "daemon.h"
#include "dji_motor.h"
#include "remote_control.h"
#include "robot_config.h"
#include "usart.h"

enum {
    ST_IDLE,
    ST_HOME_LEFT,
    ST_HOME_RIGHT,
    ST_RUN,
};

static const float kHomeTimeoutMs = 30000.0f;
static const float kLowSpeedDps = 20.0f;

static DJIMotorInstance *motor;
static RC_ctrl_t *rc;

 uint8_t state;
 bool stick_mid;
 int8_t run_dir;
 float edge_l;
 float edge_r;
 float m_per_deg; // 有符号比例，单位m/转子°
 float speed_dps;
static float home_start_ms;

/**
 * @brief 处理堵转：找端时记录挡板位置，正常运行时停机。
 */
static void HandleBlock(float now_ms)
{
    PIDInstance *pid = &motor->motor_controller.speed_PID;
    if (pid->ERRORHandler.ERRORType != PID_MOTOR_BLOCKED_ERROR)
    {
        return;
    }

    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
    pid->ERRORHandler.ERRORCount = 0;
    PIDClear(pid);
    stick_mid = false;

    if (state == ST_HOME_LEFT)
    {
        edge_l = motor->measure.total_angle;
        home_start_ms = now_ms;
        state = ST_HOME_RIGHT;
        return;
    }

    if (state == ST_HOME_RIGHT)
    {
        edge_r = motor->measure.total_angle;

        float span = fabsf(edge_r - edge_l);
        if (CONVEYOR_STOP_TO_STOP_M > 0.0f &&
            CONVEYOR_SAFE_MARGIN_M * 2.0f < CONVEYOR_STOP_TO_STOP_M &&
            span > 1.0f)
        {
            m_per_deg = CONVEYOR_STOP_TO_STOP_M / (edge_r - edge_l);
        }

    }
    else
    {
      m_per_deg = 0.0f;
    }

    state = ST_IDLE;
}

/**
 * @brief 根据软限位计算正常往返速度。
 */
static void RunConveyor(float dt_s)
{
    float position = (motor->measure.total_angle - edge_l) * m_per_deg;
    if (position < 0.0f || position > CONVEYOR_STOP_TO_STOP_M)
    {
        m_per_deg = 0.0f;
        state = ST_IDLE;
        return;
    }

    // v=sqrt(2as)，离软限位越近，允许速度越小
    float remain = run_dir > 0
                       ? CONVEYOR_STOP_TO_STOP_M - CONVEYOR_SAFE_MARGIN_M - position
                       : position - CONVEYOR_SAFE_MARGIN_M;
    float target_mps = remain > 0.0f
                           ? run_dir * fminf(CONVEYOR_TARGET_MPS,
                           sqrtf(2.0f * CONVEYOR_ACCEL_MPS2 * remain)) : 0.0f;

    float delta = target_mps / m_per_deg - speed_dps;
    float step = CONVEYOR_ACCEL_MPS2 / fabsf(m_per_deg) * dt_s;
    speed_dps += fminf(fmaxf(delta, -step), step);

    // 到达软限位并基本停稳后换向
    if (remain <= 0.0f && fabsf(motor->measure.speed_aps) < kLowSpeedDps)
    {
        run_dir = -run_dir;
        speed_dps = 0.0f;
    }
}

void RobotInit(void)
{
    rc = RemoteControlInit(&huart3);
    motor = DJIMotorInit(&conveyor_motor_config);
    motor->motor_controller.speed_PID.Improve |= PID_ErrorHandle;
    DJIMotorStop(motor);

    state = ST_IDLE;
    m_per_deg = 0.0f;
}

void RobotTask(void)
{
    const bool control_ready = motor->feed_cnt != 0 &&
                               DaemonIsOnline(motor->daemon) &&
                               RemoteControlIsOnline();
    const int16_t stick = control_ready ? rc[TEMP].rc.rocker_l1 : 0;

    if (!control_ready)
    {
        m_per_deg = 0.0f;
        stick_mid = false;
        state = ST_IDLE;
    }
    else if (stick <= -400)
    {
        stick_mid = false;
        state = ST_IDLE;
    }
    else
    {
        const float now_ms = DWT_GetTimeline_ms();
        HandleBlock(now_ms);

        if (stick >= -100 && stick <= 100)
        {
            stick_mid = true;
        }

        const bool start = stick_mid && stick >= 400;
        if (start)
        {
            stick_mid = false;
        }

        if (state == ST_IDLE && start)
        {
            if (m_per_deg == 0.0f)
            {
                home_start_ms = now_ms;
                speed_dps = -CONVEYOR_HOME_SPEED_DPS;
                state = ST_HOME_LEFT;
            }
            else
            {
                float position = (motor->measure.total_angle - edge_l) * m_per_deg;
                run_dir = position < CONVEYOR_STOP_TO_STOP_M * 0.5f ? 1 : -1;
                speed_dps = 0.0f;
                state = ST_RUN;
            }
        }
        else if (state == ST_HOME_LEFT || state == ST_HOME_RIGHT)
        {
            if (now_ms - home_start_ms > kHomeTimeoutMs)
            {
                m_per_deg = 0.0f;
                stick_mid = false;
                state = ST_IDLE;
            }
            else
            {
                speed_dps = state == ST_HOME_LEFT ? -CONVEYOR_HOME_SPEED_DPS
                                                   : CONVEYOR_HOME_SPEED_DPS;
            }
        }
        else if (state == ST_RUN)
        {
            RunConveyor(motor->dt);
        }
    }

    if (state == ST_IDLE)
    {
        DJIMotorStop(motor);
        return;
    }

    if (motor->stop_flag == MOTOR_STOP)
    {
        PIDClear(&motor->motor_controller.speed_PID);
        DJIMotorEnable(motor);
    }
    DJIMotorSetPIDRef(motor, speed_dps);
}
