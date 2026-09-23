/**
 * @file    arm_traj.h
 * @brief   辨识用位姿扫描状态机（STM32H7 / control2026）。
 *
 * 为什么在下位机里而不是上位机流下来：轨迹决定辨识结果的成败，写成固件常量
 * 表可以保证
 *   - 时序确定（不依赖 USART 吞吐与抖动，掉帧不会让机械臂顿一下）；
 *   - 一份定义（scripts/emit_pose_table.py 从 sim/trajectory.py 生成，
 *     仿真与固件不可能不一致）。
 * 上位机只收数据，不发任何命令。
 *
 * 时序（每个位姿做两遍，dir = +1 / -1）：
 *
 *   MOVE    : PD 驱动到目标位姿                        （一直持续到到位或超时）
 *   SETTLE  : |dq| < ARM_TRAJ_STILL_RADPS 持续 100 ms  （等真静止）
 *   AVERAGE : 0.4 s 内每周期调一次回调，由回调负责采样   （移动段样本丢弃）
 *   -> 下一个位姿；全表 28 个位姿跑完后 dir 取反再跑一遍
 *
 * **PD 是纯 PD，没有任何重力前馈** —— 辨识期本来就没有系数可用，而且一旦
 * 加了前馈，记录量里就含了模型，回归它就是循环论证（这个错误犯过一次）。
 *
 * 每个周期的力矩：
 *     tau = ARM_TRAJ_KP * (q_pose - q) - ARM_TRAJ_KD * dq
 *
 * 静止等待的判据不可省：只有 |dq| ≈ 0 时粘滞项 b*dq 才消失，静摩擦才由
 * 双向平均抵消掉。超时（ARM_TRAJ_SETTLE_TIMEOUT_MS）则跳过该点并继续，
 * 不卡死——坏点由上位机的双向一致性门禁剔除。
 */
#ifndef ARM_TRAJ_H
#define ARM_TRAJ_H

#include <stdbool.h>
#include <stdint.h>

/** 写力矩回调。ArmTraj 每个周期算完 PD 后调用，实现方负责写进电机。 */
typedef void (*ArmTraj_OutputFn)(float tau1, float tau2);

/** 读反馈回调。**必须返回实测值**，不能是参考值。
 *  @param q1,q2   实测角（rad，**电机原始坐标系**，即 measure.total_angle。
 *                 不要在这里减零位/乘符号 —— ArmTraj 内部统一做一次）
 *  @param dq1,dq2 实测角速度（rad/s，电机原始坐标系） */
typedef void (*ArmTraj_ReadFn)(float *q1, float *q2, float *dq1, float *dq2);

/** 采样回调。仅在 AVERAGE 阶段每周期调用一次。
 *  @param q1,q2   实测角（rad，**电机原始坐标系**，与读回调同一坐标系。
 *                 换算成契约角由 ArmIdent_Feed() 负责，不要在别处重复做）
 *  @param dq1,dq2 实测角速度（rad/s，电机原始坐标系）
 *  @param tau1,tau2 本周期算出的指令力矩（**下发值**，不是读回的 torque）
 *  @param pose_id 当前位姿编号
 *  @param dir     +1 / -1，第几遍
 */
typedef void (*ArmTraj_SampleFn)(float q1, float q2, float dq1, float dq2,
                                 float tau1, float tau2,
                                 uint8_t pose_id, int8_t dir);

/** @brief 初始化。在电机和串口都就绪之后调用一次。 */
void ArmTraj_Init(ArmTraj_ReadFn read_fn, ArmTraj_OutputFn out_fn,
                  ArmTraj_SampleFn sample_fn);

/** @brief 设置电机原始角 -> 契约角的换算，以及关节角限位。
 *
 *  契约角 = (total_angle - zero) * sign，0 = 连杆水平、+ 为抬起。
 *  **位姿表就在契约系里**（emit_pose_table.py 从 sim/trajectory.py 的 workspace
 *  生成），所以 PD 的误差必须两边同系；而限位宏 ARM_LIM_* 也是契约角。
 *
 *  零位不精确不影响辨识（S 列会吸收），但**符号必须实测正确**：符号传错则
 *  PD 把臂往反方向推，一路顶到限位才停。
 *
 *  @param q1_zero,q2_zero 电机零位（rad）
 *  @param q1_sign,q2_sign 必须为 +1.0f 或 -1.0f
 *  @param q1_min,q1_max   契约系 q1 限位（rad）
 *  @param q2_min,q2_max   契约系 q2 限位（rad）
 *  @param trip_margin     越界判定余量（rad）：超出「限位 + 该余量」才判定跑飞。
 *                         留余量是为了避开"刚好在边界抖动 → 反复 abort"。
 *
 *  必须在 ArmTraj_Init() 之后、ArmTraj_Tick() 之前调用。不调用则退化为
 *  恒等换算且**不启用限位**（仅用于单元测试）。 */
void ArmTraj_SetJointTransform(float q1_zero, float q2_zero,
                               float q1_sign, float q2_sign,
                               float q1_min, float q1_max,
                               float q2_min, float q2_max,
                               float trip_margin);

/** @brief 关节角是否越过限位（契约角超限）。true 表示扫描已被安全中止。 */
bool ArmTraj_LimitTripped(void);

/** @brief 双向逼近的起始偏置（rad）。第一遍取 +bias，第二遍取 -bias。
 *
 *  为什么需要它：静摩擦残留的方向由**逼近方向**决定。如果两遍都从同一边
 *  逼近同一个位姿，静摩擦残留符号相同，取均值无法抵消——那双向平均就是
 *  白做的。这个偏置保证两遍从相反方向逼近，残差符号相反，均值里消掉。
 *  （等价于仿真里的 MOVE_BIAS_DEG，见 scripts/pose_sweep_experiment.py。）
 *
 *  0 表示关闭偏置。默认值由 emit_pose_table.py 写进 ARM_TRAJ_MOVE_BIAS_RAD。 */
void ArmTraj_SetMoveBias(float bias_rad);

/** @brief 在 500 Hz 控制任务里调用。dt 用 DWT 实测值（不是 osDelay 次数）。
 *  @return true 表示整趟扫掠（含双向）已完成。
 */
bool ArmTraj_Tick(float dt);

/** @brief 当前进展，用于调试/亮灯。 */
void ArmTraj_Progress(uint8_t *pose_id, int8_t *dir, uint8_t *state);

/** @brief 中止扫描，输出零力矩（安全态）。 */
void ArmTraj_Abort(void);

/** @brief 中止状态查询；true = 已中止，调用方应给零力矩。 */
bool ArmTraj_Aborted(void);

/** @brief 因未能在超时内静止而跳过的停留次数。>0 说明有坏点（会被上位机
 *         的双向配对剔除）；数量多则要复查静止阈值或机械摩擦。 */
uint16_t ArmTraj_Skipped(void);

#endif /* ARM_TRAJ_H */
