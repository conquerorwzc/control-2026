# 轻量 PWM BSP

`Bsp/pwm` 只做 TIM/PWM 通道的薄封装，不包含蜂鸣器、音乐或报警逻辑。

## 约束

- 板级代码先完成 TIM/GPIO/时钟初始化；本模块不重复初始化外设，也不调用 `HAL_TIM_PWM_Init()`。
- **一个 TIM 只注册一个通道**。当前蜂鸣器（F4 TIM4_CH3 / H7 TIM12_CH2）和舵机（TIM1_CH1）均独占各自的定时器。
- 不使用 DMA，不动态分配；实例由 `PWM_DEVICE_CNT` 个静态槽位管理，注册成功后一直有效。
- 支持 F4/H7 的 16 位计数器、PWM1 模式；频率计数范围 2~65536。
- 频率单位 **mHz**，占空比单位 **0~1000‰**。

## 接口

| 接口 | 作用 |
| --- | --- |
| `PWMRegister(config)` | 旧接口：固定周期(s)/占空比(0~1)，自动启动 |
| `PWMRegisterEx(config, &instance)` | 注册可变频率(mHz)/占空比(‰)，自动启动；`counter_hz` 为 0 时沿用当前 PSC |
| `PWMSetOutput(instance, frequency_millihz, duty_permille)` | 更新频率/占空比；频率为 0 时静音（CCR=0，ARR 不变） |
| `PWMLatchOutput(instance, frequency_millihz, duty_permille)` | 锁存固定输出，之后 `PWMSetOutput` 不再覆盖，直到复位 |
| `PWMStart/PWMStop` | 启动/停止通道；锁存后不受影响 |
| `PWMSetPeriod/PWMSetDutyRatio` | 旧接口，单位分别为 s 和 0~1 |

## 行为

- 注册时按 `counter_hz`（或当前 PSC）计算计数频率，再按频率计算 `ARR`、按占空比计算 `CCR`。
- 正常换频/换占空比只写 `ARR/CCR`，不重置计数器，避免音乐换音时产生额外的相位跳变。
- 100% 占空比使用 `CCR = ARR`（PWM1 下接近满占空），0% 使用 `CCR = 0`。
- 相同 TIM 重复注册返回 `PWM_BUSY`；非法参数返回 `PWM_INVALID_ARGUMENT`；HAL 启动失败返回 `PWM_HAL_ERROR`。
- 锁存用于 `BuzzerPanicTone()`，锁存后普通输出更新返回 `PWM_OUTPUT_LATCHED`。

## 验证

- 仓库内 `infantry_mecanum`、`infantry_wheel_legged_sjtu` 和飞镖 F4 构建通过。
- 蜂鸣器 `BuzzerInit/BuzzerTask/BuzzerPanicTone` 使用新接口；舵机沿用旧 `PWMRegister/PWMSetDutyRatio`。
- 未做实车听感/示波器验证；H7 的 TIM12_CH2 路径需上板确认。
