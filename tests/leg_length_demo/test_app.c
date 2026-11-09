#include "test_platform.h"
#include <stdio.h>
#include <string.h>

// 执行真实 RobotInit/RobotTask 和 PID；这里只替代硬件输入、RTOS 创建和发送端。
#include "../../UserApp/robot/leg_length_demo/robot.c"

UART_HandleTypeDef huart5;
int hcan1, hcan2;
static DMMotorInstance motors[4];
static RC_ctrl_t input;
static Publisher_t publisher;
static unsigned motor_count;
static uint32_t tick_ms;
static bool remote_online;

float DWT_GetDeltaT(uint32_t *count) { *count += 2000u; return 0.002f; }
uint32_t HAL_GetTick(void) { return tick_ms; }
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    (void)config;
    assert(motor_count < 4);
    motors[motor_count].stop_flag = MOTOR_ENALBED; // 原驱动 Init 会使能
    return &motors[motor_count++];
}
void DMMotorSetRef(DMMotorInstance *motor, float ref) { motor->command = ref; }
void DMMotorEnable(DMMotorInstance *motor) { motor->stop_flag = MOTOR_ENALBED; }
void DMMotorStop(DMMotorInstance *motor) { motor->stop_flag = MOTOR_STOP; }
void DMMotorTask(void const *argument) { (void)argument; }
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *uart) { assert(uart == &huart5); return &input; }
uint8_t RemoteControlIsOnline(void) { return remote_online; }
Publisher_t *PubRegister(char *name, uint8_t size)
{
    assert(strcmp(name, "leg_length_status") == 0 && size == 5 * sizeof(float));
    return &publisher;
}
uint8_t PubPushMessage(Publisher_t *pub, void *data) { (void)pub; (void)data; return 1; }
osThreadId osThreadCreate(const osThreadDef_t *definition, void *argument)
{
    assert(definition->entry == DMMotorTask);
    return argument; // 不运行异步硬件任务
}

static void Feed(void)
{
    const float q[2] = {0.841850453f, 2.299742201f};
    for (unsigned i = 0; i < 4; ++i)
    {
        motors[i].feed_cnt++;
        motors[i].measure.total_angle = kJointZeroRad[i] + q[i % 2];
        motors[i].measure.velocity = 0.0f;
        motors[i].measure.state = 1u;
    }
}

static void Step(bool feed)
{
    tick_ms += 2;
    if (feed) Feed();
    RobotTask();
}

static void CheckStopped(void)
{
    for (unsigned i = 0; i < 4; ++i)
    {
        assert(motors[i].command == 0.0f);
        assert(motors[i].stop_flag == MOTOR_STOP);
    }
}

static void Rearm(void)
{
    remote_online = true;
    input.rc.switch_left = 2; input.rc.switch_right = 3;
    Step(true); CheckStopped();
    input.rc.switch_left = 3;
    Step(true);
    assert(length_demo.mode == LENGTH_ACTIVE);
}

int main(void)
{
    RobotInit();
    if (!LEG_LENGTH_CALIBRATED)
    {
        remote_online = true; input.rc.switch_left = input.rc.switch_right = 3;
        RobotTask();
        assert(motor_count == 0);
        assert(length_demo.mode == LENGTH_STOPPED);
        puts("PASS: uncalibrated application never registers motors");
        return 0;
    }
    assert(motor_count == 4);
    CheckStopped();
    remote_online = true; input.rc.switch_left = input.rc.switch_right = 3;
    Step(true); CheckStopped(); // 上电拨杆已在运行位也不能直接接管
    assert(length_demo.mode == LENGTH_STOPPED);
    Rearm();
    assert(length_demo.leg[0].length_m > 0.18f);
    assert(motors[0].command < 0.0f && motors[1].command > 0.0f);
    assert(fabsf(motors[0].command + motors[1].command) < 1e-5f);
    assert(fabsf(motors[0].command - motors[2].command) < 1e-5f);
    const float target = length_demo.target_m[0];
    Step(true);
    assert(fabsf((target - length_demo.target_m[0]) - 0.0001f) < 1e-6f);
    remote_online = false; Step(true); CheckStopped();
    remote_online = true; Step(true); CheckStopped(); // 重连不能借用旧授权
    Rearm();
    for (unsigned i = 0; i < 11; ++i) Step(false);
    assert(length_demo.mode == LENGTH_FAULT); CheckStopped();
    Step(true); assert(length_demo.mode == LENGTH_FAULT); CheckStopped();
    Rearm();
    Feed(); motors[0].measure.state = 8; Step(false);
    assert(length_demo.mode == LENGTH_FAULT); CheckStopped();
    Rearm();
    Feed(); motors[1].measure.total_angle = NAN; Step(false);
    assert(length_demo.mode == LENGTH_FAULT); CheckStopped();
    Rearm();
    input.rc.switch_right = 0; Step(true); CheckStopped();
    // HAL tick 回卷时，持续更新反馈仍然有效。
    tick_ms = UINT32_MAX - 3; Rearm(); Step(true);
    assert(length_demo.mode == LENGTH_ACTIVE);
    puts("PASS: application startup, handover, paired torque, offline/reconnect, stale feedback, motor error, NaN, tick wrap");
    return 0;
}
