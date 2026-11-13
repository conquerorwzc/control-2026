# 通用 PWM BSP

本模块只封装 MCU 的 TIM/PWM、DMA 和固定输出锁存，不包含蜂鸣器、音乐、报警或故障分类。功能模块自行选择定时器、输出参数及业务策略。

## 初始化与资源约定

- 板级代码必须先初始化外设时钟、GPIO 复用、TIM 和对应 PWM 通道；本模块不重复初始化引脚，也不调用 `HAL_TIM_PWM_Init()`。
- 所有新旧注册入口共用 `PWM_DEVICE_CNT = 16` 个静态实例。没有运行时分配、注销或池满死循环；注册成功的实例一直有效到硬件复位。
- `PWMRegisterEx()` 和旧 `PWMRegister()` 都会自动启动输出；失败不占用槽位，并恢复本次改动的定时器配置和 HAL 通道状态，可以修正条件后重试。
- 同一物理 TIM/通道不得重复登记，即使传入不同 HAL 句柄也会拒绝。实例的公开字段保留源代码兼容性，但绑定和输出缓存应视为只读，不得直接修改寄存器、重新初始化 TIM 或变更时钟树。
- 支持当前 F4/H7 器件实际存在的通道 1～4，区分 16/32 位计数器。同步输出要求内部时钟、向上边沿计数、非单脉冲、PWM1 基准模式和非换相预装载；不支持的配置返回失败，不擅自接管。
- 一个 TIM 可登记多个非独占通道，但必须接受同一计数基准和 ARR。共享期间可以分别改占空比，不能通过任何新旧接口改变 ARR/PSC；停止另一通道也不解除其登记。
- `exclusive_timer = 1` 与已有其他通道登记互斥，也拒绝接管发现已启动的未知通道、定时器中断或 DMA 资源。非零 `counter_hz` 只允许独占注册，选择最接近目标的整数 PSC；注册后没有运行时 PSC 修改接口。目标为 0 则沿用板级 PSC。

PSC 根据实际 APB1/APB2 TIM 时钟计算，处理 F4 APB2 分频及 H7 TIMPRE。周期计数以 64 位中间量舍入，`ARR = 周期计数 - 1`；实际频率受计数分辨率约束。最短允许周期为 2 个计数。占空比 0～1000 通用可用，BSP 没有 50% 限制。

## 整数接口

```c
static PWMInstance *output;

PWMResult_e OutputInit(void) {
    PWM_Ext_Init_Config_s config = {
        .htim = &htim4,
        .channel = TIM_CHANNEL_3,
        .frequency_millihz = 1000000,
        .duty_permille = 0,
        .counter_hz = 1000000,
        .exclusive_timer = 1,
    };
    return PWMRegisterEx(&config, &output);
}

PWMResult_e OutputChange(void) {
    PWM_Output_s frame;
    PWMResult_e result = PWMPrepareOutput(output, 1500000, 375, &frame);
    return result == PWM_OK ? PWMApplyOutput(output, &frame) : result;
}
```

频率单位为整数毫赫兹，例如 `1000000` 表示 1 kHz。`PWMSetOutput(output, frequency, duty)` 是准备与提交的便利接口。

| 接口 | 行为 |
| --- | --- |
| `PWMRegisterEx(config, out_instance)` | 返回操作结果；失败时将输出指针置为 `NULL`，参数和回调/用户 ID 复制到静态实例 |
| `PWMPrepareOutput(instance, frequency, duty, frame)` | 只校验并计算，不写硬件；频率为 0 必须同时占空比为 0，保留当前周期并静音 |
| `PWMApplyOutput(instance, frame)` | 重新检查绑定、资源、DMA 和锁存状态，提交周期/比较值；不会自动重启已经停止的实例 |
| `PWMSetOutput(instance, frequency, duty)` | 准备和提交；同样会在共享周期冲突时失败 |
| `PWMLatchOutput(instance, frame)` | 对独占、非 DMA 实例锁存指定固定输出，只能硬件复位解除 |
| `PWMGetStatus(instance, status)` | 返回启动、DMA、锁存、计数基准、最后提交帧参数和最近操作结果 |

准备成功不代表提交成功：期间可能出现其他通道注册、DMA 启动或锁存。帧绑定实例和登记代数，不能跨实例提交，不能手工修改或超出计数范围。普通提交会复制帧，帧可以是局部变量；准备与使用期间实例必须保持有效。

正常输出用 ARR/CCR 预装载和短暂 UDIS 屏蔽保证周期与比较值同步。在正在输出的 PWM1 上连续换频不反复清零 CNT，参数在自然更新事件生效，低频时可能等待一个完整周期。共享模式不强制更新、不重置其他通道。独占静音、起音以及固定输出锁存可立即同步提交。

0%/100% 通过目标通道的强制无效/有效模式保证端点，其他占空比恢复 PWM1；非端点 CCR 向下取整。这里的有效/无效是相对配置的输出极性，不保证对应物理引脚高/低电平。强制模式只作用于本通道。

## 返回值与旧 API

`PWMResult_e` 区分：

- `PWM_OK`：操作成功。
- `PWM_INVALID_ARGUMENT`：空参数、非法通道、频率/占空比越界、错误绑定或帧等。
- `PWM_NOT_READY`：没有有效登记实例，或板级 HAL 通道尚未就绪。
- `PWM_NO_RESOURCE`：16 个静态槽位耗尽。
- `PWM_RESOURCE_CONFLICT`：重复物理通道、独占冲突、共享周期/计数基准冲突，或未知硬件资源占用。
- `PWM_BUSY`：同一资源正在处理其他操作，或 DMA 正在控制输出。
- `PWM_OUTPUT_LATCHED`：该定时器已经锁存，普通控制不能覆盖。
- `PWM_UNSUPPORTED`：不支持的时钟/计数/PWM 模式、独占实例 DMA 或不支持的调用上下文。
- `PWM_HAL_ERROR`：HAL 启动、停止或 DMA 操作失败。

保留 `PWMInstance` 所有旧公开字段、`PWM_Init_Config_s` 及 `PWMRegister/PWMStart/PWMStop/PWMSetPeriod/PWMSetDutyRatio/PWMStartDMA` 的签名。`PWMRegister()` 失败返回 `NULL`，不死循环；有效实例的旧 void 控制接口失败时不改变正常输出，原因通过 `PWMGetStatus().last_result` 查询。空或伪造实例安全拒绝，没有对应实例可查询。

旧 `period` 单位仍为秒、`dutyratio` 为 0～1。浮点值通过读取原始对象的整数位表示验证，拒绝 NaN/Inf，快速浮点优化下仍有效。周期与占空比直接换算到硬件计数精度，不先压缩到千分比；单独调整占空比保留现有 ARR，避免大周期因浮点缓存舍入而改变。

## DMA 与调用上下文

注册、启动、停止和 DMA 控制只允许任务上下文。准备、普通提交和状态查询不依赖 RTOS，使用保存/恢复 PRIMASK 的短临界区；计算在临界区外执行。用户回调在临界区外分发，但仍处于 HAL 回调的中断上下文，不得阻塞或调用只允许任务上下文的接口。

`PWMStartDMA()` 保留旧入口，检查非空、4 字节对齐缓冲、1～65535 长度、已配置且就绪的 DMA 句柄；长度单位为传输元素。调用方负责缓冲区生命周期、DMA 可访问内存、Cache 一致性以及 DMA 数据宽度/样本值与硬件匹配。DMA 仅适用于非独占、固定计数基准实例，运行时手工提交及再次启动返回忙。

正常 DMA 完成按 HAL TIM 句柄及活动通道匹配回调，清除该通道 DMA 请求并解除忙状态；循环 DMA 保持运行，直到 `PWMStop()` 同步中止 DMA 并停止通道。完成回调不会自动恢复最后一个普通帧；状态中的帧参数是最近的普通提交，不表示 DMA 当前样本。后续普通提交或重启会恢复正常输出。

## 裸机固定输出锁存

```c
static PWM_Output_s emergency_output;

PWMResult_e PrepareEmergencyOutput(void) {
    return PWMPrepareOutput(output, 1000000, 500, &emergency_output);
}

void EmergencyHandler(void) {
    PWMLatchOutput(output, &emergency_output);
}
```

必须先完成独占实例注册并预计算帧。**锁存帧必须在 RAM 中具有直到硬件复位的静态生命周期，且准备完成后不可修改**；不要传栈变量。实现原子发布这个永久帧，第一帧胜出，重复提交同一个永久帧幂等，不允许换帧。

`PWMLatchOutput()` 不调用 HAL、RTOS、堆或延时，可用于关闭普通中断、停止调度或致命异常路径。锁存后普通输出、启动、停止、DMA 和重新注册均不能覆盖；普通寄存器提交或 HAL 控制被异常中断时，返回后重新恢复锁存输出。锁存需要 MCU 时钟、电源、TIM、GPIO 和 RAM 仍正常，不能替代硬件安全电路。

## 验证与资源

在仓库外隔离夹具直接编译真实 PWM BSP 和蜂鸣器源码，未新增仓库测试框架。F4/H7 模型分别在 ASan/UBSan 常规优化和快速浮点优化下验证 40 个 PWM 场景（共 160 次），覆盖容量/重复登记、非法参数和伪造实例、NaN/Inf、回滚、APB1/APB2/TIMPRE、16/32 位计数边界、端点、旧浮点精度、共享/独占冲突、连续低频换频、DMA 普通/循环/即时完成/启动失败/中止失败，以及提交/启动/停止中触发锁存。蜂鸣器原有 24 场景另外重跑 96 次。

当前 F4 配置 Debug/Release 整机编译，以及 F4/H7 两个模块 Debug/Release 独立交叉编译均通过；独立编译使用 `-Wall -Wextra -Werror`。Cortex-M4/M7 的原子锁存未引入外部原子运行时库。

当前交叉编译 PWM 静态 RAM 为 1472 字节，包括全部 16 槽和登记回滚快照，无 PWM 堆分配。`PWMPrepareOutput()` 自身栈帧 32 字节，`PWMApplyOutput()` 为 Debug 24 / Release 32 字节，锁存入口为 Debug 16 / Release 24 字节；这些不是完整调用链或异常上下文的栈上限。

**尚未验证真实硬件。** 上板检查 APB 时钟与实际频率、0%/100% 和极性、ARR/CCR 同步、低频调制连续性、共享通道隔离、DMA 中止、异常锁存，以及守护任务最小剩余栈。
