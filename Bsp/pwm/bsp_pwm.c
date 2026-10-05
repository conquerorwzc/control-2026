#include "bsp_pwm.h"

#include <stddef.h>
#include <string.h>

_Static_assert(__atomic_always_lock_free(sizeof(void *), 0), "PWM output latch requires lock-free pointers");

typedef enum { SLOT_FREE, SLOT_RESERVED, SLOT_READY } PWMSlotState_e;

typedef struct {
    PWMInstance instance;
    TIM_HandleTypeDef *timer;
    uint32_t channel;
    uint32_t timer_clock;
    uint32_t prescaler;
    uint32_t generation;
    PWMSlotState_e state;
    uint8_t exclusive;
    uint8_t started;
    uint8_t dma_active;
    uint8_t busy;
    PWMResult_e last_result;
    PWM_Output_s output;
    const PWM_Output_s *volatile latched_output;
} PWMSlot_s;

typedef struct {
    uint32_t control;
    uint32_t prescaler;
    uint32_t auto_reload;
    uint32_t compare;
    uint32_t capture_mode1;
    uint32_t capture_mode2;
    uint32_t channel_enable;
    uint32_t break_control;
    uint32_t counter;
    uint32_t status;
    uint32_t init_prescaler;
    uint32_t init_period;
    uint32_t init_preload;
    HAL_TIM_ChannelStateTypeDef channel_state;
} PWMRegisterSnapshot_s;

static PWMSlot_s slots[PWM_DEVICE_CNT];
static uint8_t registration_busy;
static uint32_t next_generation;
static PWMRegisterSnapshot_s registration_snapshot;

static uint32_t PWMLock(void) {
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return mask;
}

static void PWMUnlock(uint32_t mask) {
    __DMB();
    __set_PRIMASK(mask);
}

static const PWM_Output_s *PWMReadLatch(const PWMSlot_s *slot) {
    return __atomic_load_n(&slot->latched_output, __ATOMIC_ACQUIRE);
}

static PWMSlot_s *PWMFindSlot(PWMInstance *instance) {
    for (uint8_t index = 0; index < PWM_DEVICE_CNT; index++) {
        if (&slots[index].instance == instance && slots[index].state == SLOT_READY) return &slots[index];
    }
    return NULL;
}

static uint8_t PWMBindingValid(const PWMSlot_s *slot) {
    return slot->instance.htim == slot->timer && slot->instance.channel == slot->channel;
}

static PWMResult_e PWMRecord(PWMSlot_s *slot, PWMResult_e result) {
    uint32_t mask = PWMLock();
    if (slot != NULL) slot->last_result = result;
    PWMUnlock(mask);
    return result;
}

static uint32_t PWMFloatBits(const float *value) {
    uint32_t bits;
    memcpy(&bits, value, sizeof(bits));
    return bits;
}

static uint8_t PWMPositiveFloat(const float *value) {
    uint32_t bits = PWMFloatBits(value);
    return bits > 0 && bits < 0x7f800000UL;
}

static uint8_t PWMRatioValid(const float *value) {
    uint32_t bits = PWMFloatBits(value);
    return bits <= 0x3f800000UL || bits == 0x80000000UL;
}

static uint8_t PWMChannelValid(TIM_TypeDef *timer, uint32_t channel) {
    if (channel != TIM_CHANNEL_1 && channel != TIM_CHANNEL_2 && channel != TIM_CHANNEL_3 && channel != TIM_CHANNEL_4)
        return 0;
    return IS_TIM_CCX_INSTANCE(timer, channel);
}

static uint32_t PWMTimerClock(TIM_TypeDef *timer) {
    uint8_t apb2;
#if defined(STM32F407xx)
    if (timer == TIM1 || timer == TIM8 || timer == TIM9 || timer == TIM10 || timer == TIM11)
        apb2 = 1;
    else if (timer == TIM2 || timer == TIM3 || timer == TIM4 || timer == TIM5 || timer == TIM12 || timer == TIM13 ||
             timer == TIM14)
        apb2 = 0;
    else
        return 0;
    uint32_t peripheral = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
    uint32_t divider =
        apb2 ? (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos : (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    return divider < 4 ? peripheral : peripheral * 2;
#elif defined(STM32H723xx)
    if (timer == TIM1 || timer == TIM8 || timer == TIM15 || timer == TIM16 || timer == TIM17)
        apb2 = 1;
    else if (timer == TIM2 || timer == TIM3 || timer == TIM4 || timer == TIM5 || timer == TIM12 || timer == TIM13 ||
             timer == TIM14 || timer == TIM23 || timer == TIM24)
        apb2 = 0;
    else
        return 0;
    uint32_t peripheral = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
    uint32_t divider = apb2 ? (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) >> RCC_D2CFGR_D2PPRE2_Pos
                            : (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos;
    if (RCC->CFGR & RCC_CFGR_TIMPRE) return divider < 6 ? HAL_RCC_GetHCLKFreq() : peripheral * 4;
    return divider < 4 ? peripheral : peripheral * 2;
#else
#error "Unsupported PWM MCU"
#endif
}

static uint32_t PWMModeBits(PWMOutputMode_e mode) {
    if (mode == PWM_OUTPUT_INACTIVE) return TIM_OCMODE_FORCED_INACTIVE;
    if (mode == PWM_OUTPUT_ACTIVE) return TIM_OCMODE_FORCED_ACTIVE;
    return TIM_OCMODE_PWM1;
}

static volatile uint32_t *PWMModeRegister(const PWMSlot_s *slot) {
    return slot->channel < TIM_CHANNEL_3 ? &slot->timer->Instance->CCMR1 : &slot->timer->Instance->CCMR2;
}

static uint32_t PWMModeShift(const PWMSlot_s *slot) { return (slot->channel / 4 % 2) * 8; }

static void PWMWriteMode(const PWMSlot_s *slot, PWMOutputMode_e mode) {
    volatile uint32_t *capture_mode = PWMModeRegister(slot);
    uint32_t shift = PWMModeShift(slot);
    *capture_mode = (*capture_mode & ~(TIM_CCMR1_OC1M << shift)) | (PWMModeBits(mode) << shift);
}

static uint64_t PWMMaxCounts(const PWMSlot_s *slot) {
    return IS_TIM_32B_COUNTER_INSTANCE(slot->timer->Instance) ? 0x100000000ULL : 65536;
}

static PWMResult_e PWMHardwareValid(TIM_HandleTypeDef *timer, uint32_t channel) {
    if (timer == NULL || !PWMChannelValid(timer->Instance, channel)) return PWM_INVALID_ARGUMENT;
    if (TIM_CHANNEL_STATE_GET(timer, channel) != HAL_TIM_CHANNEL_STATE_READY) return PWM_NOT_READY;
    if (timer->Instance->CR1 & (TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_OPM)) return PWM_UNSUPPORTED;
    if (timer->Instance->CR2 & TIM_CR2_CCPC) return PWM_UNSUPPORTED;
    if (IS_TIM_SLAVE_INSTANCE(timer->Instance) && (timer->Instance->SMCR & (TIM_SMCR_SMS | TIM_SMCR_ECE))) {
        return PWM_UNSUPPORTED;
    }
    uint32_t capture = channel < TIM_CHANNEL_3 ? timer->Instance->CCMR1 : timer->Instance->CCMR2;
    uint32_t shift = (channel / 4 % 2) * 8;
    if (((capture >> shift) & TIM_CCMR1_CC1S) != 0 || ((capture >> shift) & TIM_CCMR1_OC1M) != TIM_OCMODE_PWM1)
        return PWM_UNSUPPORTED;
    return PWM_OK;
}

static PWMResult_e PWMTimerAvailable(const PWMSlot_s *slot, uint8_t change_period) {
    TIM_TypeDef *timer = slot->timer->Instance;
    uint32_t known_enable = 0;
    uint32_t known_requests = 0;
    uint8_t other = 0;
    for (uint8_t index = 0; index < PWM_DEVICE_CNT; index++) {
        const PWMSlot_s *candidate = &slots[index];
        if (candidate == slot || candidate->state == SLOT_FREE || candidate->timer->Instance != timer) continue;
        if (PWMReadLatch(candidate)) return PWM_OUTPUT_LATCHED;
        if (candidate->state == SLOT_RESERVED || candidate->busy) return PWM_BUSY;
        if (candidate->exclusive || slot->exclusive) return PWM_RESOURCE_CONFLICT;
        if (candidate->prescaler != slot->prescaler || candidate->timer_clock != slot->timer_clock) {
            return PWM_RESOURCE_CONFLICT;
        }
        known_enable |= 5UL << candidate->channel;
        if (candidate->dma_active) {
            known_requests |= (TIM_DMA_CC1 | TIM_IT_CC1) << (candidate->channel / 4);
        }
        other = 1;
    }
    if (slot->state == SLOT_READY) {
        known_enable |= 5UL << slot->channel;
        if (slot->dma_active) known_requests |= (TIM_DMA_CC1 | TIM_IT_CC1) << (slot->channel / 4);
    }
    if ((timer->CCER & 0x5555UL & ~known_enable) || (timer->DIER & ~known_requests)) return PWM_RESOURCE_CONFLICT;
    if (change_period && other) return PWM_RESOURCE_CONFLICT;
    if (slot->state == SLOT_RESERVED && !other && (timer->CR1 & TIM_CR1_CEN)) return PWM_RESOURCE_CONFLICT;
    if (timer->CR1 & TIM_CR1_UDIS) return PWM_BUSY;
    return PWM_OK;
}

static void PWMBuildFrame(const PWMSlot_s *slot, uint64_t counts, uint32_t compare, float duty, PWMOutputMode_e mode,
                          PWM_Output_s *frame) {
    *frame = (PWM_Output_s){.instance = (PWMInstance *)&slot->instance,
                            .generation = slot->generation,
                            .auto_reload = (uint32_t)(counts - 1),
                            .compare = compare,
                            .mode = mode,
                            .period = (float)((double)counts * (slot->prescaler + 1) / slot->timer_clock),
                            .duty_ratio = duty};
}

static PWMResult_e PWMBuildInteger(const PWMSlot_s *slot, uint32_t frequency, uint16_t duty, PWM_Output_s *frame) {
    if (duty > 1000 || (frequency == 0 && duty != 0)) return PWM_INVALID_ARGUMENT;
    uint64_t counts;
    if (frequency == 0) {
        uint32_t mask = PWMLock();
        counts = (uint64_t)slot->timer->Instance->ARR + 1;
        PWMUnlock(mask);
    } else {
        uint64_t denominator = (uint64_t)(slot->prescaler + 1) * frequency;
        counts = ((uint64_t)slot->timer_clock * 1000 + denominator / 2) / denominator;
    }
    if (counts < 2 || counts > PWMMaxCounts(slot)) return PWM_INVALID_ARGUMENT;
    PWMOutputMode_e mode = duty == 0 ? PWM_OUTPUT_INACTIVE : duty == 1000 ? PWM_OUTPUT_ACTIVE : PWM_OUTPUT_PWM1;
    uint32_t compare = mode == PWM_OUTPUT_PWM1 ? (uint32_t)(counts * duty / 1000) : 0;
    PWMBuildFrame(slot, counts, compare, (float)duty / 1000, mode, frame);
    return PWM_OK;
}

static PWMResult_e PWMBuildFloat(const PWMSlot_s *slot, const float *period, const float *duty, PWM_Output_s *frame) {
    if ((period && !PWMPositiveFloat(period)) || !PWMRatioValid(duty)) return PWM_INVALID_ARGUMENT;
    uint64_t counts;
    if (period) {
        double exact = (double)*period * slot->timer_clock / (slot->prescaler + 1);
        if (exact < 1.5 || exact >= (double)PWMMaxCounts(slot) + 0.5) return PWM_INVALID_ARGUMENT;
        counts = (uint64_t)(exact + 0.5);
    } else {
        uint32_t mask = PWMLock();
        counts = (uint64_t)slot->timer->Instance->ARR + 1;
        PWMUnlock(mask);
        if (counts < 2 || counts > PWMMaxCounts(slot)) return PWM_INVALID_ARGUMENT;
    }
    uint64_t compare = (uint64_t)((double)counts * *duty);
    PWMOutputMode_e mode = *duty == 0 ? PWM_OUTPUT_INACTIVE : compare >= counts ? PWM_OUTPUT_ACTIVE : PWM_OUTPUT_PWM1;
    PWMBuildFrame(slot, counts, mode == PWM_OUTPUT_PWM1 ? (uint32_t)compare : 0, *duty, mode, frame);
    return PWM_OK;
}

static uint8_t PWMFrameValid(const PWMSlot_s *slot, const PWM_Output_s *frame) {
    if (frame == NULL || frame->instance != &slot->instance || frame->generation != slot->generation ||
        (uint64_t)frame->auto_reload + 1 < 2 || (uint64_t)frame->auto_reload + 1 > PWMMaxCounts(slot) ||
        (uint64_t)frame->compare >= (uint64_t)frame->auto_reload + 1 || (uint32_t)frame->mode > PWM_OUTPUT_ACTIVE ||
        !PWMPositiveFloat(&frame->period) || !PWMRatioValid(&frame->duty_ratio))
        return 0;
    return frame->mode == PWM_OUTPUT_PWM1 || frame->compare == 0;
}

static void PWMCommit(const PWMSlot_s *slot, const PWM_Output_s *frame, uint8_t force) {
    TIM_TypeDef *timer = slot->timer->Instance;
    uint32_t mode = (*PWMModeRegister(slot) >> PWMModeShift(slot)) & TIM_CCMR1_OC1M;
    uint8_t restart = force || (slot->exclusive && (mode != TIM_OCMODE_PWM1 || frame->mode != PWM_OUTPUT_PWM1));
    if (!force && timer->ARR == frame->auto_reload &&
        __HAL_TIM_GET_COMPARE(slot->timer, slot->channel) == frame->compare && mode == PWMModeBits(frame->mode))
        return;
    uint32_t control = timer->CR1;
    timer->CR1 = (control | TIM_CR1_UDIS) & (restart ? ~TIM_CR1_CEN : UINT32_MAX);
    if (restart) PWMWriteMode(slot, PWM_OUTPUT_INACTIVE);
    timer->ARR = frame->auto_reload;
    __HAL_TIM_SET_COMPARE(slot->timer, slot->channel, frame->compare);
    if (restart) {
        timer->CR1 &= ~TIM_CR1_UDIS;
        timer->EGR = TIM_EGR_UG;
        timer->CNT = 0;
        timer->SR &= ~TIM_SR_UIF;
    }
    PWMWriteMode(slot, frame->mode);
    timer->CR1 = control;
    slot->timer->Init.Period = frame->auto_reload;
}

static uint8_t PWMRestoreLatch(PWMSlot_s *slot) {
    const PWM_Output_s *frame = PWMReadLatch(slot);
    if (frame == NULL) return 0;
    TIM_TypeDef *timer = slot->timer->Instance;
    timer->DIER = 0;
    timer->PSC = slot->prescaler;
    PWMCommit(slot, frame, 1);
    timer->CR1 &= ~TIM_CR1_UDIS;
    timer->CCER |= TIM_CCER_CC1E << slot->channel;
    if (IS_TIM_BREAK_INSTANCE(timer)) timer->BDTR |= TIM_BDTR_MOE;
    timer->CR1 |= TIM_CR1_CEN;
    slot->started = 1;
    return 1;
}

static void PWMSaveRegisters(const PWMSlot_s *slot) {
    TIM_TypeDef *timer = slot->timer->Instance;
    registration_snapshot = (PWMRegisterSnapshot_s){.control = timer->CR1,
                                                    .prescaler = timer->PSC,
                                                    .auto_reload = timer->ARR,
                                                    .compare = __HAL_TIM_GET_COMPARE(slot->timer, slot->channel),
                                                    .capture_mode1 = timer->CCMR1,
                                                    .capture_mode2 = timer->CCMR2,
                                                    .channel_enable = timer->CCER,
                                                    .break_control = IS_TIM_BREAK_INSTANCE(timer) ? timer->BDTR : 0,
                                                    .counter = timer->CNT,
                                                    .status = timer->SR,
                                                    .init_prescaler = slot->timer->Init.Prescaler,
                                                    .init_period = slot->timer->Init.Period,
                                                    .init_preload = slot->timer->Init.AutoReloadPreload,
                                                    .channel_state = TIM_CHANNEL_STATE_GET(slot->timer, slot->channel)};
}

static void PWMRollbackRegisters(PWMSlot_s *slot) {
    TIM_TypeDef *timer = slot->timer->Instance;
    timer->CR1 |= TIM_CR1_UDIS;
    timer->PSC = registration_snapshot.prescaler;
    timer->ARR = registration_snapshot.auto_reload;
    __HAL_TIM_SET_COMPARE(slot->timer, slot->channel, registration_snapshot.compare);
    timer->CCMR1 = registration_snapshot.capture_mode1;
    timer->CCMR2 = registration_snapshot.capture_mode2;
    timer->CCER = registration_snapshot.channel_enable;
    if (IS_TIM_BREAK_INSTANCE(timer)) timer->BDTR = registration_snapshot.break_control;
    if (!(registration_snapshot.control & TIM_CR1_CEN)) {
        timer->CR1 &= ~(TIM_CR1_CEN | TIM_CR1_UDIS);
        timer->EGR = TIM_EGR_UG;
        timer->CNT = registration_snapshot.counter;
        timer->SR = registration_snapshot.status;
    }
    timer->CR1 = registration_snapshot.control;
    slot->timer->Init.Prescaler = registration_snapshot.init_prescaler;
    slot->timer->Init.Period = registration_snapshot.init_period;
    slot->timer->Init.AutoReloadPreload = registration_snapshot.init_preload;
    TIM_CHANNEL_STATE_SET(slot->timer, slot->channel, registration_snapshot.channel_state);
}

static PWMResult_e PWMRegisterInternal(const PWM_Ext_Init_Config_s *config, const PWM_Init_Config_s *legacy,
                                       PWMInstance **instance) {
    if (instance == NULL) return PWM_INVALID_ARGUMENT;
    *instance = NULL;
    if (config == NULL || config->exclusive_timer > 1 || (config->counter_hz && !config->exclusive_timer)) {
        return PWM_INVALID_ARGUMENT;
    }
    if (config->htim == NULL || !PWMChannelValid(config->htim->Instance, config->channel)) {
        return PWM_INVALID_ARGUMENT;
    }
    uint32_t duplicate_mask = PWMLock();
    for (uint8_t index = 0; index < PWM_DEVICE_CNT; index++) {
        PWMSlot_s *candidate = &slots[index];
        if (candidate->state != SLOT_FREE && candidate->timer->Instance == config->htim->Instance &&
            (candidate->channel == config->channel || PWMReadLatch(candidate))) {
            PWMResult_e duplicate = PWMReadLatch(candidate) ? PWM_OUTPUT_LATCHED : PWM_RESOURCE_CONFLICT;
            PWMUnlock(duplicate_mask);
            return duplicate;
        }
    }
    PWMUnlock(duplicate_mask);
    PWMResult_e result = PWMHardwareValid(config->htim, config->channel);
    if (result != PWM_OK) return result;
    uint32_t timer_clock = PWMTimerClock(config->htim->Instance);
    uint32_t prescaler = config->htim->Instance->PSC;
    if (!timer_clock || prescaler > 65535) return PWM_NOT_READY;
    if (config->counter_hz) {
        uint64_t divider = ((uint64_t)timer_clock + config->counter_hz / 2) / config->counter_hz;
        if (divider == 0 || divider > 65536) return PWM_INVALID_ARGUMENT;
        prescaler = (uint32_t)divider - 1;
    }

    uint32_t mask = PWMLock();
    if (registration_busy) {
        PWMUnlock(mask);
        return PWM_BUSY;
    }
    PWMSlot_s *slot = NULL;
    for (uint8_t index = 0; index < PWM_DEVICE_CNT; index++) {
        PWMSlot_s *candidate = &slots[index];
        if (candidate->state == SLOT_FREE) {
            if (slot == NULL) slot = candidate;
        } else if (candidate->timer->Instance == config->htim->Instance && candidate->channel == config->channel) {
            result = PWMReadLatch(candidate) ? PWM_OUTPUT_LATCHED : PWM_RESOURCE_CONFLICT;
            PWMUnlock(mask);
            return result;
        }
    }
    if (slot == NULL) {
        PWMUnlock(mask);
        return PWM_NO_RESOURCE;
    }
    memset(slot, 0, sizeof(*slot));
    slot->timer = config->htim;
    slot->channel = config->channel;
    slot->timer_clock = timer_clock;
    slot->prescaler = prescaler;
    slot->exclusive = config->exclusive_timer;
    slot->generation = ++next_generation;
    if (slot->generation == 0) slot->generation = ++next_generation;
    slot->state = SLOT_RESERVED;
    result = PWMTimerAvailable(slot, 0);
    if (result != PWM_OK) {
        slot->state = SLOT_FREE;
        PWMUnlock(mask);
        return result;
    }
    registration_busy = 1;
    PWMUnlock(mask);

    PWM_Output_s frame;
    result = legacy ? PWMBuildFloat(slot, &legacy->period, &legacy->dutyratio, &frame)
                    : PWMBuildInteger(slot, config->frequency_millihz, config->duty_permille, &frame);
    mask = PWMLock();
    if (result == PWM_OK) result = PWMTimerAvailable(slot, config->htim->Instance->ARR != frame.auto_reload);
    if (result == PWM_OK) {
        PWMSaveRegisters(slot);
        TIM_TypeDef *timer = config->htim->Instance;
        uint8_t force = (timer->CR1 & TIM_CR1_CEN) == 0;
        timer->PSC = prescaler;
        slot->timer->Init.Prescaler = prescaler;
        timer->CR1 |= TIM_CR1_ARPE;
        timer->CR1 |= TIM_CR1_UDIS;
        slot->timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        *PWMModeRegister(slot) |= TIM_CCMR1_OC1PE << PWMModeShift(slot);
        PWMCommit(slot, &frame, force);
        PWMUnlock(mask);
        HAL_StatusTypeDef hal_result = HAL_TIM_PWM_Start(config->htim, config->channel);
        mask = PWMLock();
        if (hal_result != HAL_OK) {
            PWMRollbackRegisters(slot);
            result = PWM_HAL_ERROR;
        } else {
            timer->CR1 &= ~TIM_CR1_UDIS;
            slot->instance = (PWMInstance){.htim = config->htim,
                                           .channel = config->channel,
                                           .tclk = timer_clock,
                                           .period = frame.period,
                                           .dutyratio = frame.duty_ratio,
                                           .callback = config->callback,
                                           .id = config->id};
            slot->output = frame;
            slot->started = 1;
            slot->state = SLOT_READY;
            *instance = &slot->instance;
        }
    }
    if (result != PWM_OK) slot->state = SLOT_FREE;
    registration_busy = 0;
    PWMUnlock(mask);
    return result;
}

PWMResult_e PWMRegisterEx(const PWM_Ext_Init_Config_s *config, PWMInstance **instance) {
    if (__get_IPSR() != 0) {
        if (instance != NULL) *instance = NULL;
        return PWM_UNSUPPORTED;
    }
    return PWMRegisterInternal(config, NULL, instance);
}

PWMInstance *PWMRegister(PWM_Init_Config_s *config) {
    if (config == NULL || __get_IPSR() != 0) return NULL;
    PWM_Ext_Init_Config_s extended = {
        .htim = config->htim, .channel = config->channel, .callback = config->callback, .id = config->id};
    PWMInstance *instance;
    return PWMRegisterInternal(&extended, config, &instance) == PWM_OK ? instance : NULL;
}

static PWMResult_e PWMOperationReady(PWMSlot_s *slot) {
    if (slot == NULL) return PWM_NOT_READY;
    if (PWMReadLatch(slot)) return PWM_OUTPUT_LATCHED;
    if (!PWMBindingValid(slot)) return PWM_INVALID_ARGUMENT;
    if (slot->busy || slot->dma_active) return PWM_BUSY;
    if (slot->timer->Instance->PSC != slot->prescaler) return PWM_RESOURCE_CONFLICT;
    return PWMTimerAvailable(slot, 0);
}

PWMResult_e PWMPrepareOutput(PWMInstance *instance, uint32_t frequency_millihz, uint16_t duty_permille,
                             PWM_Output_s *frame) {
    if (frame == NULL) return PWM_INVALID_ARGUMENT;
    *frame = (PWM_Output_s){0};
    uint32_t mask = PWMLock();
    PWMSlot_s *slot = PWMFindSlot(instance);
    PWMResult_e result = PWMOperationReady(slot);
    PWMUnlock(mask);
    if (result == PWM_OK) result = PWMBuildInteger(slot, frequency_millihz, duty_permille, frame);
    return PWMRecord(slot, result);
}

PWMResult_e PWMApplyOutput(PWMInstance *instance, const PWM_Output_s *frame) {
    PWMSlot_s *slot = PWMFindSlot(instance);
    if (slot == NULL) return PWM_NOT_READY;
    if (!PWMFrameValid(slot, frame)) return PWMRecord(slot, PWM_INVALID_ARGUMENT);
    uint32_t mask = PWMLock();
    PWMResult_e result = PWMOperationReady(slot);
    if (result == PWM_OK) result = PWMTimerAvailable(slot, slot->timer->Instance->ARR != frame->auto_reload);
    if (result == PWM_OK) {
        PWMCommit(slot, frame, 0);
        if (PWMRestoreLatch(slot))
            result = PWM_OUTPUT_LATCHED;
        else {
            slot->output = *frame;
            slot->instance.period = frame->period;
            slot->instance.dutyratio = frame->duty_ratio;
        }
    }
    slot->last_result = result;
    PWMUnlock(mask);
    return result;
}

PWMResult_e PWMSetOutput(PWMInstance *instance, uint32_t frequency_millihz, uint16_t duty_permille) {
    PWM_Output_s frame;
    PWMResult_e result = PWMPrepareOutput(instance, frequency_millihz, duty_permille, &frame);
    return result == PWM_OK ? PWMApplyOutput(instance, &frame) : result;
}

PWMResult_e PWMLatchOutput(PWMInstance *instance, const PWM_Output_s *frame) {
    PWMSlot_s *slot = PWMFindSlot(instance);
    if (slot == NULL) return PWM_NOT_READY;
    if (!slot->exclusive) return PWM_UNSUPPORTED;
    if (!PWMBindingValid(slot)) return PWM_INVALID_ARGUMENT;
    if (!PWMFrameValid(slot, frame)) return PWM_INVALID_ARGUMENT;
    const PWM_Output_s *expected = NULL;
    if (!__atomic_compare_exchange_n(&slot->latched_output, &expected, frame, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE) &&
        expected != frame)
        return PWM_OUTPUT_LATCHED;
    uint32_t mask = PWMLock();
    PWMRestoreLatch(slot);
    slot->last_result = PWM_OK;
    PWMUnlock(mask);
    return PWM_OK;
}

PWMResult_e PWMGetStatus(PWMInstance *instance, PWM_Status_s *status) {
    if (status == NULL) return PWM_INVALID_ARGUMENT;
    *status = (PWM_Status_s){0};
    uint32_t mask = PWMLock();
    PWMSlot_s *slot = PWMFindSlot(instance);
    if (slot == NULL) {
        PWMUnlock(mask);
        return PWM_NOT_READY;
    }
    const PWM_Output_s *latched = PWMReadLatch(slot);
    PWM_Output_s frame = latched ? *latched : slot->output;
    *status = (PWM_Status_s){.initialized = 1,
                             .started = slot->started,
                             .dma_active = slot->dma_active,
                             .output_latched = latched != NULL,
                             .exclusive_timer = slot->exclusive,
                             .timer_clock_hz = slot->timer_clock,
                             .prescaler = slot->prescaler,
                             .auto_reload = frame.auto_reload,
                             .compare = frame.compare,
                             .mode = frame.mode,
                             .last_result = slot->last_result};
    PWMUnlock(mask);
    status->counter_hz = status->timer_clock_hz / (status->prescaler + 1);
    status->frequency_millihz = (uint64_t)status->timer_clock_hz * 1000 /
                                ((uint64_t)(status->prescaler + 1) * ((uint64_t)frame.auto_reload + 1));
    status->duty_permille = (uint16_t)(frame.duty_ratio * 1000 + 0.5f);
    return PWM_OK;
}

static void PWMSetFloat(PWMInstance *instance, const float *period, const float *duty) {
    uint32_t mask = PWMLock();
    PWMSlot_s *slot = PWMFindSlot(instance);
    PWMResult_e result = PWMOperationReady(slot);
    float local_duty = slot ? slot->output.duty_ratio : 0;
    PWMUnlock(mask);
    if (result == PWM_OK) {
        PWM_Output_s frame;
        result = PWMBuildFloat(slot, period, duty ? duty : &local_duty, &frame);
        if (result == PWM_OK) result = PWMApplyOutput(instance, &frame);
    }
    PWMRecord(slot, result);
}

void PWMSetPeriod(PWMInstance *pwm, float period) { PWMSetFloat(pwm, &period, NULL); }
void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio) { PWMSetFloat(pwm, NULL, &dutyratio); }

static uint32_t PWMDmaIndex(const PWMSlot_s *slot) { return TIM_DMA_ID_CC1 + slot->channel / 4; }

static void PWMControl(PWMInstance *instance, uint8_t start, uint32_t *buffer, uint32_t size) {
    uint32_t mask = PWMLock();
    PWMSlot_s *slot = PWMFindSlot(instance);
    PWMResult_e result = slot == NULL ? PWM_NOT_READY : PWM_OK;
    if (slot && PWMReadLatch(slot))
        result = PWM_OUTPUT_LATCHED;
    else if (slot && !PWMBindingValid(slot))
        result = PWM_INVALID_ARGUMENT;
    else if (slot && slot->busy)
        result = PWM_BUSY;
    else if (__get_IPSR() != 0)
        result = PWM_UNSUPPORTED;
    else if (slot && slot->timer->Instance->PSC != slot->prescaler)
        result = PWM_RESOURCE_CONFLICT;
    if (result == PWM_OK) result = PWMTimerAvailable(slot, 0);
    if (result == PWM_OK && size) {
        if (slot->exclusive)
            result = PWM_UNSUPPORTED;
        else if (slot->dma_active)
            result = PWM_BUSY;
        else if (buffer == NULL || (uintptr_t)buffer % sizeof(uint32_t) != 0 || size > UINT16_MAX ||
                 slot->timer->hdma[PWMDmaIndex(slot)] == NULL) {
            result = PWM_INVALID_ARGUMENT;
        } else if (HAL_DMA_GetState(slot->timer->hdma[PWMDmaIndex(slot)]) != HAL_DMA_STATE_READY) {
            result = PWM_BUSY;
        }
    }
    if (result != PWM_OK) {
        if (slot) slot->last_result = result;
        PWMUnlock(mask);
        return;
    }
    slot->busy = 1;
    uint8_t was_dma = slot->dma_active;
    uint8_t was_started = slot->started;
    uint32_t control = slot->timer->Instance->CR1;
    uint32_t enabled = slot->timer->Instance->CCER;
    uint32_t capture = *PWMModeRegister(slot);
    uint32_t compare = __HAL_TIM_GET_COMPARE(slot->timer, slot->channel);
    uint32_t requests = slot->timer->Instance->DIER;
    uint32_t break_control = IS_TIM_BREAK_INSTANCE(slot->timer->Instance) ? slot->timer->Instance->BDTR : 0;
    HAL_TIM_ChannelStateTypeDef channel_state = TIM_CHANNEL_STATE_GET(slot->timer, slot->channel);
    PWMUnlock(mask);

    HAL_StatusTypeDef hal_result = HAL_OK;
    if (!start) {
        if (was_dma) {
            mask = PWMLock();
            slot->timer->Instance->DIER &= ~(TIM_DMA_CC1 << (slot->channel / 4));
            PWMUnlock(mask);
            DMA_HandleTypeDef *dma = slot->timer->hdma[PWMDmaIndex(slot)];
            hal_result = HAL_DMA_GetState(dma) == HAL_DMA_STATE_READY ? HAL_OK : HAL_DMA_Abort(dma);
            if (hal_result == HAL_OK) hal_result = HAL_TIM_PWM_Stop_DMA(slot->timer, slot->channel);
        } else if (was_started) {
            hal_result = HAL_TIM_PWM_Stop(slot->timer, slot->channel);
        }
    } else if (size) {
        if (was_started) hal_result = HAL_TIM_PWM_Stop(slot->timer, slot->channel);
        if (hal_result == HAL_OK) {
            mask = PWMLock();
            PWMWriteMode(slot, PWM_OUTPUT_PWM1);
            slot->dma_active = 1;
            PWMUnlock(mask);
            hal_result = HAL_TIM_PWM_Start_DMA(slot->timer, slot->channel, buffer, (uint16_t)size);
        }
        if (hal_result != HAL_OK) {
            mask = PWMLock();
            slot->timer->Instance->DIER &= ~(TIM_DMA_CC1 << (slot->channel / 4));
            PWMUnlock(mask);
            HAL_DMA_Abort(slot->timer->hdma[PWMDmaIndex(slot)]);
            HAL_TIM_PWM_Stop_DMA(slot->timer, slot->channel);
        }
    } else if (was_dma) {
        result = PWM_BUSY;
    } else if (!was_started) {
        mask = PWMLock();
        PWMCommit(slot, &slot->output, 0);
        PWMUnlock(mask);
        hal_result = HAL_TIM_PWM_Start(slot->timer, slot->channel);
    }
    mask = PWMLock();
    if (PWMRestoreLatch(slot))
        result = PWM_OUTPUT_LATCHED;
    else if (hal_result != HAL_OK) {
        slot->timer->Instance->CR1 |= TIM_CR1_UDIS;
        __HAL_TIM_SET_COMPARE(slot->timer, slot->channel, compare);
        *PWMModeRegister(slot) = capture;
        slot->timer->Instance->CCER = enabled;
        if (IS_TIM_BREAK_INSTANCE(slot->timer->Instance)) slot->timer->Instance->BDTR = break_control;
        slot->timer->Instance->CR1 = control;
        TIM_CHANNEL_STATE_SET(slot->timer, slot->channel, channel_state);
        slot->dma_active = was_dma && HAL_DMA_GetState(slot->timer->hdma[PWMDmaIndex(slot)]) != HAL_DMA_STATE_READY;
        if (slot->dma_active) slot->timer->Instance->DIER = requests;
        result = PWMRestoreLatch(slot) ? PWM_OUTPUT_LATCHED : PWM_HAL_ERROR;
    } else if (result == PWM_OK) {
        slot->started = start;
        if (!start) slot->dma_active = 0;
    }
    slot->busy = 0;
    slot->last_result = result;
    PWMUnlock(mask);
}

void PWMStart(PWMInstance *pwm) { PWMControl(pwm, 1, NULL, 0); }
void PWMStop(PWMInstance *pwm) { PWMControl(pwm, 0, NULL, 0); }
void PWMStartDMA(PWMInstance *pwm, uint32_t *buffer, uint32_t size) {
    if (size == 0) {
        PWMRecord(PWMFindSlot(pwm), PWM_INVALID_ARGUMENT);
        return;
    }
    PWMControl(pwm, 1, buffer, size);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *timer) {
    if (timer == NULL) return;
    void (*callback)(PWMInstance *) = NULL;
    PWMInstance *instance = NULL;
    uint32_t mask = PWMLock();
    for (uint8_t index = 0; index < PWM_DEVICE_CNT; index++) {
        PWMSlot_s *slot = &slots[index];
        if (slot->state != SLOT_READY || slot->timer != timer || timer->Channel != (1UL << (slot->channel / 4)))
            continue;
        if (slot->dma_active && slot->timer->hdma[PWMDmaIndex(slot)]->Init.Mode != DMA_CIRCULAR) {
            slot->timer->Instance->DIER &= ~(TIM_DMA_CC1 << (slot->channel / 4));
            slot->dma_active = 0;
        }
        callback = slot->instance.callback;
        instance = &slot->instance;
        break;
    }
    PWMUnlock(mask);
    if (callback) callback(instance);
}
