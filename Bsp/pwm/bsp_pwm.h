/**
 * @file bsp_pwm.h
 * @brief 轻量 PWM BSP: 固定/可变频率占空比输出
 *
 * 按当前仓库实际使用裁剪:
 *   - 一个 TIM 只注册一个 PWM 通道(蜂鸣器 TIM4/TIM12、舵机 TIM1 均独占);
 *   - 不使用 DMA, 不动态分配, 不支持同一 TIM 多通道共享;
 *   - 支持 F4/H7 的 16 位计数器、PWM1 模式;
 *   - 频率单位 mHz, 占空比单位 0~1000‰, 便于蜂鸣器直接换频.
 *
 * 旧接口 PWMRegister/PWMSetPeriod/PWMSetDutyRatio/PWMStart/PWMStop 保留.
 */
#pragma once

#include <stdint.h>

#include "tim.h"

#define PWM_DEVICE_CNT 8

typedef struct pwm_ins_temp {
  /* 公开字段: 兼容旧 PWM BSP/调用方, 只读使用 */
  TIM_HandleTypeDef *htim;              // 定时器 HAL 句柄
  uint32_t channel;                     // TIM_CHANNEL_x
  uint32_t tclk;                        // 定时器输入时钟(Hz), 非 APB 总线时钟
  float period;                         // 当前周期(s), 仅用于旧接口回读
  float dutyratio;                      // 当前占空比(0~1), 仅用于旧接口回读
  void (*callback)(struct pwm_ins_temp *);  // 兼容旧字段, 本实现不触发
  void *id;                             // 调用方私有指针

  /* private: 由 BSP 维护, 调用方不要直接修改 */
  uint32_t prescaler;                   // 当前 PSC
  uint32_t auto_reload;                 // 当前 ARR
  uint8_t started;                      // 通道是否已启动
  uint8_t latched;                      // 是否已锁存(panic), 锁存后不可恢复
  uint8_t used;                         // 静态槽位是否被占用
} PWMInstance;

typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  float period;                         // 周期(s)
  float dutyratio;                      // 占空比 0~1
  void (*callback)(PWMInstance *);
  void *id;
} PWM_Init_Config_s;

typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  uint32_t frequency_millihz;           // 输出频率(mHz), 如 1000000 = 1 kHz
  uint32_t counter_hz;                  // 计数频率(Hz); 0 = 沿用当前 PSC
  uint16_t duty_permille;               // 占空比 0~1000‰
  void (*callback)(PWMInstance *);
  void *id;
} PWM_Ext_Init_Config_s;

typedef enum {
  PWM_OK = 0,              // 成功
  PWM_INVALID_ARGUMENT,   // 参数/范围非法
  PWM_NOT_READY,          // 未注册或 slot 失效
  PWM_BUSY,               // 定时器已被占用/无空槽
  PWM_OUTPUT_LATCHED,     // 已锁存, 普通更新被拒绝
  PWM_UNSUPPORTED,        // 不支持的 MCU/定时器
  PWM_HAL_ERROR,          // HAL 启动失败
} PWMResult_e;

/**
 * @brief 注册固定周期/占空比的 PWM(旧接口), 自动启动
 * @param config period 单位 s, dutyratio 0~1
 * @return 实例指针, 失败返回 NULL
 */
PWMInstance *PWMRegister(PWM_Init_Config_s *config);

/**
 * @brief 注册可变频率/占空比的 PWM, 自动启动
 * @param config frequency_millihz 单位 mHz, duty_permille 0~1000
 * @param instance 输出实例指针, 失败置 NULL
 */
PWMResult_e PWMRegisterEx(const PWM_Ext_Init_Config_s *config, PWMInstance **instance);

/**
 * @brief 更新频率和占空比; frequency_millihz = 0 表示静音(占空比置 0, 保持 ARR)
 * @note 可周期性调用(蜂鸣器 100 Hz); 内部只持有很短的临界区.
 */
PWMResult_e PWMSetOutput(PWMInstance *instance, uint32_t frequency_millihz, uint16_t duty_permille);

/**
 * @brief 锁存固定输出(用于 panic), 锁存后 PWMSetOutput 不再覆盖, 直到硬件复位
 * @note 锁存会立即写一次输出, 并把 latched 置位; 主要用于 BuzzerPanicTone().
 */
PWMResult_e PWMLatchOutput(PWMInstance *instance, uint32_t frequency_millihz, uint16_t duty_permille);

/* 旧接口: 保留给 servo_motor 等既有调用方 */
void PWMStart(PWMInstance *pwm);
void PWMStop(PWMInstance *pwm);
void PWMSetPeriod(PWMInstance *pwm, float period);   // period 单位 s
void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio);  // dutyratio 0~1
