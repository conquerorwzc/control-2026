/**
 * @file bsp_pwm.h
 * @author your name (you@domain.com)
 * @brief * @version 0.1
 * @date 2023-02-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <stdint.h>

#include "tim.h"
#define PWM_DEVICE_CNT 16

typedef struct pwm_ins_temp {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t tclk;
    float period;
    float dutyratio;
    void (*callback)(struct pwm_ins_temp *);
    void *id;
} PWMInstance;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float period;
    float dutyratio;
    void (*callback)(PWMInstance *);
    void *id;
} PWM_Init_Config_s;

typedef enum {
    PWM_OK = 0,
    PWM_INVALID_ARGUMENT,
    PWM_NOT_READY,
    PWM_NO_RESOURCE,
    PWM_RESOURCE_CONFLICT,
    PWM_BUSY,
    PWM_OUTPUT_LATCHED,
    PWM_UNSUPPORTED,
    PWM_HAL_ERROR,
} PWMResult_e;

typedef enum { PWM_OUTPUT_PWM1 = 0, PWM_OUTPUT_INACTIVE, PWM_OUTPUT_ACTIVE } PWMOutputMode_e;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t frequency_millihz;
    uint32_t counter_hz;
    uint16_t duty_permille;
    uint8_t exclusive_timer;
    void (*callback)(PWMInstance *);
    void *id;
} PWM_Ext_Init_Config_s;

typedef struct {
    PWMInstance *instance;
    uint32_t generation;
    uint32_t auto_reload;
    uint32_t compare;
    PWMOutputMode_e mode;
    float period;
    float duty_ratio;
} PWM_Output_s;

typedef struct {
    uint8_t initialized;
    uint8_t started;
    uint8_t dma_active;
    uint8_t output_latched;
    uint8_t exclusive_timer;
    uint32_t timer_clock_hz;
    uint32_t counter_hz;
    uint32_t prescaler;
    uint32_t auto_reload;
    uint32_t compare;
    uint64_t frequency_millihz;
    uint16_t duty_permille;
    PWMOutputMode_e mode;
    PWMResult_e last_result;
} PWM_Status_s;

PWMInstance *PWMRegister(PWM_Init_Config_s *config);
void PWMStart(PWMInstance *pwm);
void PWMStop(PWMInstance *pwm);
void PWMSetPeriod(PWMInstance *pwm, float period);
void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio);
void PWMStartDMA(PWMInstance *pwm, uint32_t *buffer, uint32_t size);

PWMResult_e PWMRegisterEx(const PWM_Ext_Init_Config_s *config, PWMInstance **instance);
PWMResult_e PWMPrepareOutput(PWMInstance *instance, uint32_t frequency_millihz, uint16_t duty_permille,
                             PWM_Output_s *frame);
PWMResult_e PWMApplyOutput(PWMInstance *instance, const PWM_Output_s *frame);
PWMResult_e PWMSetOutput(PWMInstance *instance, uint32_t frequency_millihz, uint16_t duty_permille);
PWMResult_e PWMLatchOutput(PWMInstance *instance, const PWM_Output_s *frame);
PWMResult_e PWMGetStatus(PWMInstance *instance, PWM_Status_s *status);
