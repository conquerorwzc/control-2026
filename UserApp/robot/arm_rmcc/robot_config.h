/**
******************************************************************************
* @file    robot_config.h
* @brief   arm_rmcc 的全部可调参数集中在此
*
* 改动本文件之外的东西之前请先想清楚：本机器人被要求
*   - 所有业务逻辑只在 RobotTask() 内执行
*   - 不修改 module / bsp / components 等其它层
******************************************************************************
*/
#pragma once

/* robot.h 已经引入了 dmmotor.h / bsp_usart.h / general_def.h，
 * 本文件依赖的 Motor_Init_Config_s / J4310 / OPEN_LOOP / hcan1 / huart1
 * 都由那条链带进来，保持与其它 robot 相同的写法（只 include robot.h）。 */
#include "robot.h"

/* ==========================================================================
 * 1. 工作模式 —— 编译期二选一
 *
 * 两种模式的控制器是**互相矛盾**的，所以不能同时存在：
 *   辨识模式：外面套一层纯 PD 把机械臂驱动到各个位姿（要硬，才跟得动）
 *   部署模式：纯力矩前馈，可被参赛者的机械臂反向推动（要软，才推得动）
 * ======================================================================== */
#define ARM_MODE_IDENT      /* 辨识：跑位姿扫描 + J-Link 上报 */
// #define ARM_MODE_DEPLOY  /* 部署：重力前馈补偿 */

#if defined(ARM_MODE_IDENT) && defined(ARM_MODE_DEPLOY)
#error "ARM_MODE_IDENT 与 ARM_MODE_DEPLOY 只能开一个"
#endif
#if !defined(ARM_MODE_IDENT) && !defined(ARM_MODE_DEPLOY)
#error "必须开启 ARM_MODE_IDENT 或 ARM_MODE_DEPLOY 之一"
#endif

/* ==========================================================================
 * 2. 电机：两个 pitch 关节，各一个 DM-J4310
 *
 * CAN ID -> 接收 ID 的映射由 dmmotor.c 的 MotorSenderGrouping 决定：
 *   tx_id 1..4 -> rx_id 0x205..0x208
 * 两个电机挂在同一条总线上，ID 不能重复。
 * ======================================================================== */
#define ARM_J1_CAN_HANDLE   hcan1
#define ARM_J1_TX_ID        1
#define ARM_J1_RX_ID        0x205   /* DM 反馈 ID：tx_id 1..4 -> 0x205..0x208 */
#define ARM_J2_CAN_HANDLE   hcan1
#define ARM_J2_TX_ID        2
#define ARM_J2_RX_ID        0x206

/* 力矩上限。本仓库的 dmmotor.c 按型号分发，用的是型号专属宏
 * （DM_T_MIN_J4310 / DM_T_MAX_J4310），没有通用的 DM_T_MAX。
 * 这里只做**改名**，不定义 ARM_MOTOR_TYPE 之类的型号宏 —— 那会污染
 * 下面 `.motor_type = J4310` 的枚举名。换电机时改这里和那两处 .motor_type。
 *
 * 底部大 pitch（J1）已换 DM-J4340：±28 N*m / ±10 rad/s。
 * 注意 J4340 的速度上限只有 J4310（±30 rad/s）的三分之一，而位置量程同样是
 * ±12.5 rad —— 10 rad/s 下 1.25 s 就能跑遍整个量程，所以下面的角度限位不是
 * 可选项。J4340 力矩余量很大（静态载荷峰值约 3.5 N*m，只占 13%），
 * 但 J2 仍是 J4310（±10 N*m）。 */
#define ARM_J1_T_MIN        DM_T_MIN_J4340
#define ARM_J1_T_MAX        DM_T_MAX_J4340
#define ARM_J2_T_MIN        DM_T_MIN_J4310
#define ARM_J2_T_MAX        DM_T_MAX_J4310

/* ==========================================================================
 * 2b. 关节角限位（安全）
 *
 * 角度是**契约角**：0 = 连杆水平，+ 为抬起。与位姿表、与辨识上报同一坐标系。
 *
 * 为什么必须有：PD 是纯 PD、没有前馈，而 `DMMotorTask` 在饱和时只把力矩截到
 * 额定值、**不会停**。编码器漂移、零位设错、被测臂顶住、或逼近偏置把目标顶
 * 出包络，控制器都会一直顶着额定力矩推。限位是这条链上唯一会"停"的东西。
 *
 * ⚠️ 下面四个 TRIP 限位是**推断值，不是实测值**。依据是
 *    RMCC/docs/mechanical-interface.md 里那张表的空白项——真实机械硬限位标定
 *    完请回来改，并保证 TRIP 落在硬限位**以内**（要留制动距离，J4340 在
 *    10 rad/s 下的制动距离不是零）。
 *
 * 辨识包络必须覆盖「位姿表 + 逼近偏置」：位姿表用满设计包络
 * q1 ∈ [35,140]°、q2 ∈ [-80,30]°，而 MOVE 逼近偏置会再超出 2°，故取
 * [30,145]° / [-85,35]°，两侧各留 5° 余量。
 * ======================================================================== */
#define ARM_LIM_Q1_MIN      (0.523599f)   /*  30 deg */
#define ARM_LIM_Q1_MAX      (2.530727f)   /* 145 deg */
#define ARM_LIM_Q2_MIN      (-1.483530f)  /* -85 deg */
#define ARM_LIM_Q2_MAX      (0.610865f)   /*  35 deg */

/* 部署（重力补偿 + 保持）包络。当前取与辨识相同；对接任务若需要更大范围，
 * 在这里放宽——但**不能超过机械硬限位**。 */
#define ARM_DEPLOY_Q1_MIN   ARM_LIM_Q1_MIN
#define ARM_DEPLOY_Q1_MAX   ARM_LIM_Q1_MAX
#define ARM_DEPLOY_Q2_MIN   ARM_LIM_Q2_MIN
#define ARM_DEPLOY_Q2_MAX   ARM_LIM_Q2_MAX

/* 越界判定点：超出「限位 + 该余量」才判跑飞并卸力。
 * 留这 3° 是为了避开"刚好在边界抖动 → 反复 abort"。 */
#define ARM_LIM_TRIP_MARGIN (0.0523599f)  /* 3 deg */

/* ==========================================================================
 * 3. 零位与方向
 *
 * 零位**不需要精确**。补偿律的系数会吸收零位偏移：
 *   cos(q+alpha) 展开后多出来的分量正好落在 S 列上，最小二乘会解出来。
 *   实测偏移 0~180° 力矩误差均为机器精度（见 docs/zero-point.md）。
 *   所以 q_zero 填 0 也能跑，只是解出的 C 列不再正好等于物理静矩。
 *
 * 方向**必须实测正确**。搞反不是小误差，是整条方程反号：
 *   辨识会解出负静矩，补偿方向反了，机械臂跑到镜像位姿。
 *   自检：把连杆摆在水平附近，手推它**向上**，看 measure.total_angle
 *   是增大（+1）还是减小（-1）。
 * ======================================================================== */
#define ARM_J1_ZERO         (0.0f)
#define ARM_J2_ZERO         (0.0f)
#define ARM_J1_SIGN         (+1.0f)
#define ARM_J2_SIGN         (+1.0f)

/* ==========================================================================
 * 4. 数据上报串口（J-Link 虚拟串口）
 *
 * 走的是目标板上一路**真实 UART** 桥接到 PC，不是 STM32 自己的 USB 外设。
 *   - 波特率两头必须一致（J-Link 设置里也要改）
 *   - 必须是 8N1：UART5 是 9 位字长（SBUS 用），**不能用来传二进制**
 *   - 本协议是单向的：下位机只发，上位机只收，所以 recv_buff_size 设为 1
 * 带宽：40 字节/帧 * 100 Hz = 40000 bit/s（含起止位）
 *   115200 占 34.7%（单帧 3.47 ms / 间隔 10 ms，够但余量不大）
 *   460800 占  8.7%（建议值）
 * ======================================================================== */
#define ARM_IDENT_UART_HANDLE   huart1
#define ARM_IDENT_BAUD_HINT     460800U  /* 仅注释用：真实波特率在 usart.c 里设 */

/* ==========================================================================
 * 5. 辨识期 PD（纯 PD，**没有任何重力前馈**）
 *
 * 为什么不能加前馈：辨识期本来就没有系数可用；而且一旦加了前馈，记录的
 * 指令力矩里就含了模型，再拿它回归模型就是循环论证（这个错误犯过一次）。
 * 参考 c/arm_traj.c 与 docs/identification-procedure.md。
 *
 * KP 的取舍（实测）：
 *   KP=10  下垂 23.4°  辨识云图 RMS 0.0093 N*m
 *   KP=25  下垂  8.6°  辨识云图 RMS 0.0001 N*m   <- 默认
 *   KP=50  下垂  4.2°  辨识云图 RMS 0.0001 N*m
 *   KP=100 下垂  2.1°  辨识云图 RMS 0.0001 N*m
 * 加大 KP 收益很小，但会让移动段峰值力矩变大、并放大速度噪声。
 * ======================================================================== */
#define ARM_TRAJ_KP_DEFAULT  (25.0f)
#define ARM_TRAJ_KD_DEFAULT  (2.0f)

/* ==========================================================================
 * 6. 部署期：重力前馈 + 可选低刚度阻抗
 *
 * 场地臂要能被参赛者的机械臂推动，所以刚度必须低。
 * 静止时的稳态误差 ≈ 力矩残差 / Kp：Kp=2 且残差 0.015 N*m 时约 0.0075 rad。
 * ======================================================================== */
#define ARM_HOLD_KP         (2.0f)
#define ARM_HOLD_KD         (0.15f)
#define ARM_GRAV_SCALE      (1.0f)   /* 调手感用；不影响辨识结果 */

/* 部署期的保持目标（**契约角**，rad：0 = 连杆水平，+ 为抬起）。
 * 取位姿表首点，即设计包络的 q1 下限 / q2 下限那一角。
 * 留给后续接遥控/上位机改成可变值。 */
#define ARM_HOLD_Q1         (0.610865f)   /*  35 deg */
#define ARM_HOLD_Q2         (-1.396263f)  /* -80 deg */

/* ==========================================================================
 * 7. 安全
 * ======================================================================== */
/* >0 表示电机反馈长时间不变就卸力（判为通信丢失或卡死）。单位 ms。 */
#define ARM_FEEDBACK_TIMEOUT_MS  (200U)
/* 预测峰值力矩超过这个比例就在启动时报警。额定值取两轴中较小的那个
 * （J2 仍是 J4310 的 10 N*m），因为静态载荷是由 J1 承担、但误差会摊到两轴。 */
#define ARM_TORQUE_BUDGET_RATIO  (0.8f)

/* ==========================================================================
 * 8. 电机配置
 *
 * 三个环全部不闭合：DMMotorSetRef() 直接写 final_output，而 DMMotorTask()
 * 只读 final_output 并发出去 —— 这就是纯力矩模式。
 * 闭合任何环都会让"下发的力矩 ≠ 实际力矩"，辨识的前提就没了。
 * 而且位置环会让关节变硬，与"可被推动"的要求相反。
 * ======================================================================== */
/* __attribute__((unused)) 只影响编译：os_task.c 也会 include robot.h，
 * 于是这两份 static 配置在它的编译单元里被定义却用不到，-Wall 会报
 * -Wunused-variable。属性只抑制那个告警，不改语义、不改任何其它层。 */
static Motor_Init_Config_s arm_j1_motor_config __attribute__((unused)) = {
    .can_init_config =
        {
            /* hcan1 会被 bsp_can.h 展开成实例 hfdcan1，需要取地址 */
            .can_handle = &ARM_J1_CAN_HANDLE,
            .tx_id = ARM_J1_TX_ID,
            /* 本仓库的 CAN 过滤器是"每电机一条精确匹配"（bsp_can.c 用
             * FilterID1=rx_id, mask=0x7FF），rx_id 不写就默认 0，
             * 反馈帧永远进不来 —— 编译期查不出，只在真机上表现为电机不动。 */
            .rx_id = ARM_J1_RX_ID,
        },
    .controller_setting_init_config =
        {
            .outer_loop_type = OPEN_LOOP,
            .close_loop_type = OPEN_LOOP,
            .feedforward_flag = FEEDFORWARD_NONE,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
    .motor_type = J4340,   /* 底部大 pitch：DM-J4340, ±28 N*m */
};

static Motor_Init_Config_s arm_j2_motor_config __attribute__((unused)) = {
    .can_init_config =
        {
            .can_handle = &ARM_J2_CAN_HANDLE,
            .tx_id = ARM_J2_TX_ID,
            .rx_id = ARM_J2_RX_ID,
        },
    .controller_setting_init_config =
        {
            .outer_loop_type = OPEN_LOOP,
            .close_loop_type = OPEN_LOOP,
            .feedforward_flag = FEEDFORWARD_NONE,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
    .motor_type = J4310,
};
