/**
 * @file bsp_pwm.c
 * @brief 轻量 PWM BSP 实现: 单通道/单定时器, 支持固定与可变频率输出
 *
 * 设计取舍(相对旧版 700+ 行通用 PWM 驱动):
 *   - 只保留当前蜂鸣器(TIM4_CH3 / TIM12_CH2)和舵机(TIM1_CH1)用到的能力;
 *   - 一个 TIM 只允许一个通道, 去掉共享定时器/多通道仲裁;
 *   - 去掉 DMA、注册回滚、状态查询、prepare/apply 两段式帧;
 *   - 更新频率/占空比时不重置 CNT, 避免蜂鸣器换音时产生额外相位跳变.
 *
 * 频率计算:
 *   counter_hz = tclk / (PSC + 1)
 *   counts     = counter_hz * 1000 / frequency_millihz   (四舍五入)
 *   ARR        = counts - 1,  CCR = counts * duty_permille / 1000
 * counts 限制在 2..65536, 与 16 位 TIM 的 ARR 范围一致.
 */
#include "bsp_pwm.h"

#include <stddef.h>

/* ARR 为 16 位: 2..65536 个计数; 65536 时 ARR=65535 */
#define PWM_MAX_COUNTS 65536UL

/* 静态槽位: 不动态分配; 注册成功后实例指针一直有效, 直到硬件复位 */
typedef struct {
  PWMInstance instance;
  uint8_t used;
} PWMSlot_s;

static PWMSlot_s slots[PWM_DEVICE_CNT];

/* ---------------- 临界区 ---------------- */

/* PWM 更新与 latch 可能被任务/panic 并发调用; 用 PRIMASK 保存/恢复,
 * 临界区只包寄存器读写和槽位扫描, 不包含延时/阻塞操作. */
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

/* 占空比 -> CCR. 100% 用 ARR+1(始终有效), 但 counts=65536 时 CCR 只有 16 位,
 * 退化为 ARR(接近 100%), 避免写入 65536 被截断成 0 反而变成静音. */
static uint32_t PWMCompareValue(uint32_t counts, uint16_t duty_permille) {
  if (duty_permille == 1000) return counts >= PWM_MAX_COUNTS ? counts - 1 : counts;
  return (uint32_t)((uint64_t)counts * duty_permille / 1000);
}

/* ---------------- 基础校验 ---------------- */

/* 只支持 CH1~CH4; 同时用 HAL 宏确认该 TIM 确实有对应比较通道 */
static uint8_t PWMChannelValid(TIM_TypeDef *timer, uint32_t channel) {
  if (channel != TIM_CHANNEL_1 && channel != TIM_CHANNEL_2 && channel != TIM_CHANNEL_3 && channel != TIM_CHANNEL_4) {
    return 0;
  }
  return IS_TIM_CCX_INSTANCE(timer, channel);
}

/* TIM 时钟: F4 区分 APB1/APB2 分频, H7 处理 TIMPRE
 * 注意返回的是 TIM 计数时钟, 不是 APB 总线时钟:
 *   F4  APB 预分频 != 1 时, TIMxCLK = PCLKx * 2
 *   H7  还要看 RCC->CFGR.TIMPRE */
static uint32_t PWMTimerClock(TIM_TypeDef *timer) {
#if defined(STM32F407xx)
  uint8_t apb2;
  if (timer == TIM1 || timer == TIM8 || timer == TIM9 || timer == TIM10 || timer == TIM11) {
    apb2 = 1;
  } else if (timer == TIM2 || timer == TIM3 || timer == TIM4 || timer == TIM5 || timer == TIM12 || timer == TIM13 ||
             timer == TIM14) {
    apb2 = 0;
  } else {
    return 0;
  }
  uint32_t peripheral = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
  uint32_t divider =
      apb2 ? (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos : (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
  return divider < 4 ? peripheral : peripheral * 2;
#elif defined(STM32H723xx)
  uint8_t apb2;
  if (timer == TIM1 || timer == TIM8 || timer == TIM15 || timer == TIM16 || timer == TIM17) {
    apb2 = 1;
  } else if (timer == TIM2 || timer == TIM3 || timer == TIM4 || timer == TIM5 || timer == TIM12 || timer == TIM13 ||
             timer == TIM14 || timer == TIM23 || timer == TIM24) {
    apb2 = 0;
  } else {
    return 0;
  }
  uint32_t peripheral = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
  uint32_t divider = apb2 ? (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) >> RCC_D2CFGR_D2PPRE2_Pos
                          : (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos;
  if (RCC->CFGR & RCC_CFGR_TIMPRE) {
    return divider < 6 ? HAL_RCC_GetHCLKFreq() : peripheral * 4;
  }
  return divider < 4 ? peripheral : peripheral * 2;
#else
#error "Unsupported PWM MCU"
#endif
}

/* 实例指针必须仍是当前 used 槽位, 防止传入陈旧/伪造指针 */
static PWMInstance *PWMFind(PWMInstance *instance) {
  for (uint32_t index = 0; index < PWM_DEVICE_CNT; index++) {
    if (slots[index].used && &slots[index].instance == instance) return instance;
  }
  return NULL;
}

/* ---------------- 寄存器操作 ---------------- */

/* 写 PSC/ARR/CCR; reload=1 时产生更新事件, 用于首次注册或改 PSC.
 * 普通换频/换占空比(reload=0)不触发 EGR, 因此不会把 CNT 清零,
 * 蜂鸣器连续换音时不会引入额外的 100 Hz 相位跳变. */
static void PWMWriteTiming(PWMInstance *instance, uint32_t prescaler, uint32_t auto_reload, uint32_t compare,
                           uint8_t reload) {
  TIM_TypeDef *timer = instance->htim->Instance;
  uint32_t control = timer->CR1;
  timer->CR1 = control | TIM_CR1_UDIS;  // 更新期间不让影子寄存器生效
  if (timer->PSC != prescaler) {
    timer->PSC = prescaler;             // PSC 改动必须靠 UG 生效
    reload = 1;
  }
  timer->ARR = auto_reload;
  __HAL_TIM_SET_COMPARE(instance->htim, instance->channel, compare);
  if (reload) timer->EGR = TIM_EGR_UG;
  timer->CR1 = control;
  /* 同步 HAL 句柄, 避免后续 HAL 调用看到旧的 Prescaler/Period */
  instance->htim->Init.Prescaler = prescaler;
  instance->htim->Init.Period = auto_reload;
  instance->prescaler = prescaler;
  instance->auto_reload = auto_reload;
}

/* 调用方需持有 PWMLock.
 * 0 Hz 表示静音: 只把 CCR 清 0, 保留 ARR, 这样恢复发声时不需要重新算周期. */
static PWMResult_e PWMApplyLocked(PWMInstance *instance, uint32_t frequency_millihz, uint16_t duty_permille) {
  if (frequency_millihz == 0) {
    __HAL_TIM_SET_COMPARE(instance->htim, instance->channel, 0);
    instance->dutyratio = 0.0f;
    return PWM_OK;
  }
  if (duty_permille > 1000) return PWM_INVALID_ARGUMENT;

  /* counts 用 64 位中间量, 先乘 1000 再除, 避免 mHz 单位下溢出 */
  uint32_t counter_hz = instance->tclk / (instance->prescaler + 1);
  if (counter_hz == 0) return PWM_UNSUPPORTED;
  uint64_t counts = ((uint64_t)counter_hz * 1000 + frequency_millihz / 2) / frequency_millihz;
  if (counts < 2 || counts > PWM_MAX_COUNTS) return PWM_INVALID_ARGUMENT;

  uint32_t auto_reload = (uint32_t)counts - 1;
  uint32_t compare = PWMCompareValue((uint32_t)counts, duty_permille);
  PWMWriteTiming(instance, instance->prescaler, auto_reload, compare, 0);
  instance->period = (float)((double)counts * (instance->prescaler + 1) / instance->tclk);
  instance->dutyratio = (float)duty_permille / 1000.0f;
  return PWM_OK;
}

/* ---------------- 注册 ---------------- */

/* 注册公共实现:
 *   - 先校验通道/counts/占空比, 再检查该 TIM 是否已被占用(一 TIM 一通道);
 *   - 从静态槽位取空位, 写入实例后配置 PSC/ARR/CCR, 最后 HAL 启动;
 *   - HAL 启动失败时释放槽位, 但不会回滚已经写入的寄存器(调用方很少重试).
 * 调用方负责把 counts(ARR+1) 按自己的周期/频率算好. */
static PWMResult_e PWMInstall(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t prescaler, uint32_t counts,
                              uint16_t duty_permille, void (*callback)(PWMInstance *), void *id,
                              PWMInstance **instance) {
  if (instance == NULL) return PWM_INVALID_ARGUMENT;
  *instance = NULL;
  if (htim == NULL || !PWMChannelValid(htim->Instance, channel) || prescaler > 65535 ||
      counts < 2 || counts > PWM_MAX_COUNTS || duty_permille > 1000) {
    return PWM_INVALID_ARGUMENT;
  }
  uint32_t timer_clock = PWMTimerClock(htim->Instance);
  if (timer_clock == 0) return PWM_UNSUPPORTED;
  if (timer_clock / (prescaler + 1) == 0) return PWM_INVALID_ARGUMENT;

  uint32_t mask = PWMLock();
  for (uint32_t index = 0; index < PWM_DEVICE_CNT; index++) {
    if (slots[index].used && slots[index].instance.htim->Instance == htim->Instance) {
      PWMUnlock(mask);
      return PWM_BUSY;  // 一个 TIM 只允许一个 PWM 实例
    }
  }

  PWMSlot_s *slot = NULL;
  for (uint32_t index = 0; index < PWM_DEVICE_CNT; index++) {
    if (!slots[index].used) {
      slot = &slots[index];
      break;
    }
  }
  if (slot == NULL) {
    PWMUnlock(mask);
    return PWM_BUSY;
  }

  slot->used = 1;
  PWMInstance *output = &slot->instance;
  output->htim = htim;
  output->channel = channel;
  output->tclk = timer_clock;
  output->period = (float)((double)counts * (prescaler + 1) / timer_clock);
  output->dutyratio = (float)duty_permille / 1000.0f;
  output->callback = callback;
  output->id = id;
  output->prescaler = prescaler;
  output->auto_reload = counts - 1;
  output->started = 0;
  output->latched = 0;

  uint32_t compare = PWMCompareValue(counts, duty_permille);
  PWMWriteTiming(output, prescaler, counts - 1, compare, 1);
  if (HAL_TIM_PWM_Start(htim, channel) != HAL_OK) {
    slot->used = 0;
    PWMUnlock(mask);
    return PWM_HAL_ERROR;
  }
  output->started = 1;
  PWMUnlock(mask);
  *instance = output;
  return PWM_OK;
}

/* 旧接口: period 秒 + dutyratio 0~1, 沿用 CubeMX 当前 PSC.
 * 例: TIM1 PSC=167, tclk=168 MHz -> counter=1 MHz, period=0.02 s -> counts=20000. */
PWMInstance *PWMRegister(PWM_Init_Config_s *config) {
  if (config == NULL || config->htim == NULL || !(config->period > 0.0f) ||
      !(config->dutyratio >= 0.0f && config->dutyratio <= 1.0f)) {
    return NULL;
  }
  uint32_t timer_clock = PWMTimerClock(config->htim->Instance);
  uint32_t prescaler = config->htim->Instance->PSC;
  if (timer_clock == 0 || prescaler > 65535) return NULL;
  uint32_t counter_hz = timer_clock / (prescaler + 1);
  if (counter_hz == 0) return NULL;

  double exact_counts = (double)config->period * counter_hz;
  if (exact_counts < 2.0 || exact_counts > (double)PWM_MAX_COUNTS) return NULL;
  uint32_t counts = (uint32_t)(exact_counts + 0.5);
  uint16_t duty = (uint16_t)(config->dutyratio * 1000.0f + 0.5f);
  PWMInstance *instance = NULL;
  return PWMInstall(config->htim, config->channel, prescaler, counts, duty, config->callback, config->id, &instance) ==
                 PWM_OK
             ? instance
             : NULL;
}

/* 可变频率接口: counter_hz 非 0 时按目标计数频率反算 PSC(蜂鸣器固定 1 MHz);
 * counter_hz=0 时沿用板级 PSC. 例: tclk=84 MHz, counter_hz=1 MHz -> PSC=83,
 * frequency=1 kHz -> counts=1000. */
PWMResult_e PWMRegisterEx(const PWM_Ext_Init_Config_s *config, PWMInstance **instance) {
  if (instance == NULL) return PWM_INVALID_ARGUMENT;
  *instance = NULL;
  if (config == NULL || config->htim == NULL || config->frequency_millihz == 0 || config->duty_permille > 1000) {
    return PWM_INVALID_ARGUMENT;
  }
  uint32_t timer_clock = PWMTimerClock(config->htim->Instance);
  if (timer_clock == 0) return PWM_UNSUPPORTED;

  uint32_t prescaler;
  if (config->counter_hz != 0) {
    uint64_t divider = ((uint64_t)timer_clock + config->counter_hz / 2) / config->counter_hz;
    if (divider == 0 || divider > 65536) return PWM_INVALID_ARGUMENT;
    prescaler = (uint32_t)divider - 1;
  } else {
    prescaler = config->htim->Instance->PSC;
    if (prescaler > 65535) return PWM_INVALID_ARGUMENT;
  }
  uint32_t counter_hz = timer_clock / (prescaler + 1);
  if (counter_hz == 0) return PWM_INVALID_ARGUMENT;
  uint64_t counts = ((uint64_t)counter_hz * 1000 + config->frequency_millihz / 2) / config->frequency_millihz;
  if (counts < 2 || counts > PWM_MAX_COUNTS) return PWM_INVALID_ARGUMENT;

  return PWMInstall(config->htim, config->channel, prescaler, (uint32_t)counts, config->duty_permille,
                    config->callback, config->id, instance);
}

/* ---------------- 输出更新 ---------------- */

/* 更新频率/占空比; 用短临界区包住查找+寄存器写入, 允许任务周期调用.
 * 0 Hz 时静音; 已 latch 的实例返回 PWM_OUTPUT_LATCHED, 不再覆盖 panic 输出. */
PWMResult_e PWMSetOutput(PWMInstance *instance, uint32_t frequency_millihz, uint16_t duty_permille) {
  if (instance == NULL) return PWM_NOT_READY;
  uint32_t mask = PWMLock();
  PWMInstance *found = PWMFind(instance);
  PWMResult_e result;
  if (found == NULL) {
    result = PWM_NOT_READY;
  } else if (found->latched) {
    result = PWM_OUTPUT_LATCHED;
  } else {
    result = PWMApplyLocked(found, frequency_millihz, duty_permille);
  }
  PWMUnlock(mask);
  return result;
}

/* panic 用: 先写一次目标输出, 成功后置 latched, 之后所有 PWMSetOutput 都被拒绝.
 * 锁存没有清除接口, 只能靠硬件复位解除. */
PWMResult_e PWMLatchOutput(PWMInstance *instance, uint32_t frequency_millihz, uint16_t duty_permille) {
  if (instance == NULL) return PWM_NOT_READY;
  uint32_t mask = PWMLock();
  PWMInstance *found = PWMFind(instance);
  PWMResult_e result;
  if (found == NULL) {
    result = PWM_NOT_READY;
  } else if (found->latched) {
    result = PWM_OUTPUT_LATCHED;
  } else {
    result = PWMApplyLocked(found, frequency_millihz, duty_permille);
    if (result == PWM_OK) found->latched = 1;
  }
  PWMUnlock(mask);
  return result;
}

/* 旧接口: 只改占空比, 保留当前 ARR; 适合舵机这类固定 50 Hz 输出 */
void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio) {
  if (pwm == NULL || !(dutyratio >= 0.0f && dutyratio <= 1.0f)) return;
  uint32_t mask = PWMLock();
  PWMInstance *instance = PWMFind(pwm);
  if (instance != NULL && !instance->latched) {
    uint32_t counts = instance->auto_reload + 1;
    uint16_t duty = (uint16_t)(dutyratio * 1000.0f + 0.5f);
    uint32_t compare = PWMCompareValue(counts, duty);
    __HAL_TIM_SET_COMPARE(instance->htim, instance->channel, compare);
    instance->dutyratio = dutyratio;
  }
  PWMUnlock(mask);
}

/* 旧接口: 只改周期(ARR), PSC 不变; 占空比按当前 dutyratio 重算 CCR */
void PWMSetPeriod(PWMInstance *pwm, float period) {
  if (pwm == NULL || !(period > 0.0f)) return;
  uint32_t mask = PWMLock();
  PWMInstance *instance = PWMFind(pwm);
  if (instance != NULL && !instance->latched) {
    uint32_t counter_hz = instance->tclk / (instance->prescaler + 1);
    double exact_counts = counter_hz > 0 ? (double)period * counter_hz : 0.0;
    if (exact_counts >= 2.0 && exact_counts <= (double)PWM_MAX_COUNTS) {
      uint32_t counts = (uint32_t)(exact_counts + 0.5);
      uint16_t duty = (uint16_t)(instance->dutyratio * 1000.0f + 0.5f);
      uint32_t compare = PWMCompareValue(counts, duty);
      PWMWriteTiming(instance, instance->prescaler, counts - 1, compare, 0);
      instance->period = period;
    }
  }
  PWMUnlock(mask);
}

/* HAL 启停封装: latch 后禁止再启动/停止, 保证 panic 输出不会被误关掉 */
void PWMStart(PWMInstance *pwm) {
  if (pwm == NULL) return;
  uint32_t mask = PWMLock();
  PWMInstance *instance = PWMFind(pwm);
  if (instance != NULL && !instance->latched && !instance->started) {
    if (HAL_TIM_PWM_Start(instance->htim, instance->channel) == HAL_OK) instance->started = 1;
  }
  PWMUnlock(mask);
}

void PWMStop(PWMInstance *pwm) {
  if (pwm == NULL) return;
  uint32_t mask = PWMLock();
  PWMInstance *instance = PWMFind(pwm);
  if (instance != NULL && !instance->latched && instance->started) {
    HAL_TIM_PWM_Stop(instance->htim, instance->channel);
    instance->started = 0;
  }
  PWMUnlock(mask);
}
