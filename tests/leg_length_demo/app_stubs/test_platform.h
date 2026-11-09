#pragma once
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include "controller.h"

typedef struct { unsigned unused; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart5;
extern int hcan1, hcan2;
enum { J8009P, MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_REVERSE,
       FEEDBACK_DIRECTION_NORMAL, FEEDBACK_DIRECTION_REVERSE, MOTOR_STOP, MOTOR_ENALBED };
typedef struct
{
    unsigned motor_type;
    struct { void *can_handle; unsigned tx_id, rx_id; } can_init_config;
    struct { unsigned motor_reverse_flag, feedback_reverse_flag; } controller_setting_init_config;
} Motor_Init_Config_s;
typedef struct { float total_angle, velocity; uint8_t state; } DM_Motor_Measure_s;
typedef struct
{
    DM_Motor_Measure_s measure;
    uint32_t feed_cnt;
    unsigned stop_flag;
    float command;
} DMMotorInstance;
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);
void DMMotorSetRef(DMMotorInstance *motor, float ref);
void DMMotorEnable(DMMotorInstance *motor);
void DMMotorStop(DMMotorInstance *motor);
void DMMotorTask(void const *argument);

typedef struct { struct { int16_t dial; uint8_t switch_left, switch_right; } rc; } RC_ctrl_t;
#define TEMP 0
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *uart);
uint8_t RemoteControlIsOnline(void);
typedef struct { unsigned unused; } Publisher_t;
Publisher_t *PubRegister(char *name, uint8_t size);
uint8_t PubPushMessage(Publisher_t *publisher, void *data);

typedef struct { void (*entry)(void const *); } osThreadDef_t;
typedef void *osThreadId;
#define osPriorityBelowNormal (-1)
#define osThreadDef(name, function, priority, instances, stack) const osThreadDef_t os_thread_def_##name = {function}
#define osThread(name) (&os_thread_def_##name)
osThreadId osThreadCreate(const osThreadDef_t *definition, void *argument);
#define configASSERT(value) assert(value)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
uint32_t HAL_GetTick(void);
