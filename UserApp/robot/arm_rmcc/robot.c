/**
******************************************************************************
* @file    robot.c
* @brief   arm_rmcc 业务逻辑
*
* 约束（来自项目要求）：
*   - 所有业务逻辑只在 RobotTask() 内执行
*   - 不新增任务/线程，不动 RoborTask 的调度
*   - 不修改 module / bsp / components 等其它层
*
* 硬件路径：
*   两个 DM-J4310 挂在 hcan1（各自一个 CAN ID）
*   DMMotorTask() 由框架在 StartROBOTTASK() 里通过 DMMotorTaskInit() 创建，
*   每个电机一个线程，每 2 ms 发一次 final_output。本文件只负责每周期填值。
*
* 两个模式见 robot_config.h 的说明，控制器互相矛盾，编译期二选一。
******************************************************************************
*/

#include "robot.h"

#include "arm_ident.h"
#include "arm_pose_table.h"
#include "arm_traj.h"
#include "bsp_dwt.h"
#include "gravity_comp.h"
#include "user_lib.h"

#ifndef ARM_MODE_IDENT
#ifndef ARM_MODE_DEPLOY
#error "robot_config.h 必须选择 ARM_MODE_IDENT 或 ARM_MODE_DEPLOY"
#endif
#endif

static RobotInstance *robot;

#ifdef ARM_MODE_IDENT
/* 辨识期的两个静态中介量，避免参数传递开销（与其它 robot 的写法一致） */
static uint32_t ident_tick;
static uint32_t ident_frames_sent_last;
#endif

#ifdef ARM_MODE_DEPLOY
/* 保持目标（**契约角**：0 = 连杆水平，+ 为抬起）。
 * 阻抗项就在契约系里算，所以这里必须是契约角 —— 不能拿它去减 total_angle。
 * 留给后续接遥控/上位机改成可变值。 */
static float hold_ref[2] = {ARM_HOLD_Q1, ARM_HOLD_Q2};
#endif

/* ==========================================================================
 * 回调：全部无堆分配、无阻塞
 * ======================================================================== */

/**
 * @brief 输出力矩。这是**唯一**让关节出力的路径。
 *
 * DMMotorSetRef() 直接写 final_output，绕过 motor_def.h 里的三环 PID
 * （配置里 outer/close_loop_type 都是 OPEN_LOOP，所以即使不绕过也没环）。
 * DMMotorTask() 会读 final_output，做 LIMIT_MIN_MAX(±ARM_J4310_T_MAX) 后发出。
 */
static void ArmOutputTorque(float tau1, float tau2) {
  DMMotorSetRef(robot->joint_motor[0], tau1);
  DMMotorSetRef(robot->joint_motor[1], tau2);
}

/* 以下三个回调只有辨识模式用得上；部署模式下不引用它们，
 * 所以整段按模式裁剪，免得 -Wunused-function。 */
#ifdef ARM_MODE_IDENT
/**
 * @brief 读实测反馈。给扫描状态机用。
 *
 * **必须是实测值，不能是参考值** —— 拟合用的是实测角度，所以 PD 跟得准
 * 不准都不影响辨识；但拿参考值当实测值就等于伪造数据。
 */
static void ArmReadFeedback(float *q1, float *q2, float *dq1, float *dq2) {
  *q1 = robot->joint_motor[0]->measure.total_angle;
  *q2 = robot->joint_motor[1]->measure.total_angle;
  *dq1 = robot->joint_motor[0]->measure.velocity;
  *dq2 = robot->joint_motor[1]->measure.velocity;
}

/**
 * @brief 静止窗采样回调。逐帧进环形缓冲，不在这里做协议/发送。
 *
 * 收的是 tau1/tau2 —— **我们算出的指令力矩**，不是 measure.torque。
 * 理由：要在能控制的量里标定。measure.torque 是转子侧测量，看不到输出端
 * 减速器/轴承/线缆的摩擦，用它辨识得到的补偿量纲对不上。
 */
static void ArmSampleCallback(float q1, float q2, float dq1, float dq2,
                              float tau1, float tau2,
                              uint8_t pose_id, int8_t dir) {
  (void)pose_id;
  (void)dir;
  ArmIdent_Feed(q1, q2, dq1, dq2, tau1, tau2);
}

/**
 * @brief USART 发送就绪判据。
 *
 * bsp_usart 不暴露 TX 完成回调，而本工程不允许改 bsp。好在
 * USARTInstance 已经暴露了 usart_handle，直接查 HAL 的发送状态即可，
 * 一行搞定。
 *
 * 没有这个判据也能跑（靠 100 Hz 发送节奏兜底：一帧 40 B 在 115200 下
 * 占 3.47 ms，而发送间隔 10 ms），但接上更稳 —— 尤其波特率调低时。
 */
static bool ArmUartTxReady(void *usart_handle) {
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)usart_handle;
  if (huart == NULL) return true;
  return (huart->gState == HAL_UART_STATE_READY) &&
         (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) != 0U);
}
#endif /* ARM_MODE_IDENT */

/* ==========================================================================
 * 初始化
 * ======================================================================== */

void RobotInit() {
  /* 这里**不要**调用 BSPInit()：
   *   - 它已经由 os_task.c 的 OSTaskInit() 在启动 RTOS 之前调用过了；
   *   - 而且 bsp_init.h 里 BSPInit 是 header-only 的非 static 定义，
   *     os_task.c 已经 include 了它，这里再 include 一次会触发
   *     "multiple definition of BSPInit" 链接错误。
   * CAN/USART 外设由 main.c 的 MX_*_Init() 初始化，DWT 见下。 */

  /* DWT 在 H7 上是前置条件，必须显式初始化。
   *
   * bsp_init.h 里 DWT_Init 的条件是 #elifdef STM32H7，但本工程编译时定义的
   * 是 STM32H723xx —— 两个宏不匹配，所以 H7 上那个分支从来不执行。后果：
   *   CPU_FREQ_Hz 保持 0  -> DWT_GetDeltaT() 变成 0/0 = NaN
   *   DWT->CTRL 未使能    -> CYCCNT 根本不计数，时间戳恒为 0
   * 而 StartROBOTTASK 与本文件的 RobotTask 都依赖 DWT，所以必须在这里补。
   *
   * 480 与 bsp_init.h 里 H7 分支写的值一致。重复调用 DWT_Init 只是把
   * CYCCNT 清一次，无副作用，所以即使将来框架修了这里也不会冲突。
   * 不改 bsp_init.h：那会影响所有机器人，超出本次改动范围。 */
  DWT_Init(480);



  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

  robot->q_zero[0] = ARM_J1_ZERO;
  robot->q_zero[1] = ARM_J2_ZERO;
  robot->q_sign[0] = ARM_J1_SIGN;
  robot->q_sign[1] = ARM_J2_SIGN;

  /* --- 电机的注册必须在这里完成：StartROBOTTASK 紧接着就调
   *     DMMotorTaskInit()，它按注册数量创建线程（idx==0 则不创建）。 --- */
  robot->joint_motor[0] = DMMotorInit(&arm_j1_motor_config);
  robot->joint_motor[1] = DMMotorInit(&arm_j2_motor_config);

  /* --- 补偿器：零位与方向在这里注入，不参与辨识（见 docs/zero-point.md） --- */
  GravityComp_Init(robot->q_zero[0], robot->q_zero[1],
                   robot->q_sign[0], robot->q_sign[1]);

  /* 上电自检：静态载荷超过**两轴中较小**额定的 80% 就报错，别等它烧了才发现。
   * 载荷由 J1 承担，但误差会摊到两轴，所以取 min(J1_T_MAX, J2_T_MAX) 作基准；
   * J1 换成 J4340（28 N*m）后瓶颈就是 J2 的 J4310（10 N*m）。 */
  {
    const float t_budget = (ARM_J1_T_MAX < ARM_J2_T_MAX) ? ARM_J1_T_MAX
                                                         : ARM_J2_T_MAX;
    if (GravityComp_MaxAbsTorque() > ARM_TORQUE_BUDGET_RATIO * t_budget) {
      LOGERROR("[arm_rmcc] gravity torque %.2f N*m exceeds %.0f%% of %.1f N*m!",
               GravityComp_MaxAbsTorque(), ARM_TORQUE_BUDGET_RATIO * 100.0f,
               t_budget);
    }
  }

#ifdef ARM_MODE_IDENT
  /* --- 上报串口。USARTRegister 内部已经调过 USARTServiceInit，
   *     不要再单独调一次。本协议单向，不需要接收解析。 --- */
  robot->ident_usart = USARTRegister(&(USART_Init_Config_s){
      .usart_handle = &ARM_IDENT_UART_HANDLE,
      .recv_buff_size = 1,
      .module_callback = NULL,
  });
  ArmIdent_Init(robot->ident_usart, robot->q_sign[0], robot->q_sign[1]);
  ArmIdent_SetZero(robot->q_zero[0], robot->q_zero[1]);
  ArmIdent_SetTxReadyFn(ArmUartTxReady);

  /* --- 位姿扫描状态机。三个回调都在本文件内，不跨层。 --- */
  ArmTraj_SetMoveBias(ARM_TRAJ_MOVE_BIAS_RAD);
  ArmTraj_Init(ArmReadFeedback, ArmOutputTorque, ArmSampleCallback);
  /* 坐标系换算 + 关节角限位。位姿表在契约系里，而读回调给的是电机原始角，
   * 这个换算就是两者之间的桥；限位是这条控制链上唯一会"停"的东西。 */
  ArmTraj_SetJointTransform(robot->q_zero[0], robot->q_zero[1],
                            robot->q_sign[0], robot->q_sign[1],
                            ARM_LIM_Q1_MIN, ARM_LIM_Q1_MAX,
                            ARM_LIM_Q2_MIN, ARM_LIM_Q2_MAX,
                            ARM_LIM_TRIP_MARGIN);
  LOGINFO("[arm_rmcc] joint limits q1[%.1f,%.1f] q2[%.1f,%.1f] deg",
          ARM_LIM_Q1_MIN * 57.29578f, ARM_LIM_Q1_MAX * 57.29578f,
          ARM_LIM_Q2_MIN * 57.29578f, ARM_LIM_Q2_MAX * 57.29578f);

  robot->mode = ARM_IDENT;
  ident_tick = 0;
  ident_frames_sent_last = 0;
  LOGINFO("[arm_rmcc] IDENT mode: %d poses x 2 dir = %d dwells, about %d s",
          ARM_POSE_COUNT, ARM_POSE_DWELLS,
          (int)(ARM_POSE_DWELLS * (ARM_TRAJ_SETTLE_S + ARM_TRAJ_AVG_S)));
#else
  robot->mode = ARM_HOLD;
  hold_ref[0] = ARM_POSE_TABLE[0].q1;
  hold_ref[1] = ARM_POSE_TABLE[0].q2;
  LOGINFO("[arm_rmcc] DEPLOY mode: gravity feed-forward, k_grav=%.2f Kp=%.2f",
          ARM_GRAV_SCALE, ARM_HOLD_KP);
#endif

  DMMotorEnable(robot->joint_motor[0]);
  DMMotorEnable(robot->joint_motor[1]);
}

/* ==========================================================================
 * 业务：全部在 RobotTask 内，500 Hz
 * ======================================================================== */

void RobotTask() {
#ifdef ARM_MODE_IDENT
  /* 用 DWT 实测周期，不要写死 0.002 —— osDelay(2) 的实际周期有抖动，
   * 而静止判据与静止窗时长都是按时间积分的。 */
  const float dt = DWT_GetDeltaT(&ident_tick);

  /* 扫描：状态机自己管 MOVE / SETTLE / AVERAGE / 双向 / 超时。
   * 返回 true 表示整趟（含双向）跑完 —— 该返回是幂等的，不会漏掉。 */
  if (ArmTraj_Tick(dt)) {
    ArmOutputTorque(0.0f, 0.0f); /* 跑完立刻卸力，别一直顶着 */
    if (robot->mode != ARM_IDLE) {
      robot->mode = ARM_IDLE;
      uint32_t dropped = 0, sent = 0;
      ArmIdent_Stats(&dropped, &sent);
      LOGINFO("[arm_rmcc] scan DONE. dwells skipped=%d, usart frames=%lu "
              "dropped=%lu",
              (int)ArmTraj_Skipped(), (unsigned long)sent,
              (unsigned long)dropped);
    }
  } else if (ArmTraj_Aborted()) {
    ArmOutputTorque(0.0f, 0.0f);
    robot->mode = ARM_IDLE;
  }

  /* 上报：把环形缓冲里的整批推给 USART。非阻塞，拥塞就下次再来。 */
  ArmIdent_Poll();
#else
  /* ---- 部署模式：重力前馈 + 低刚度阻抗 ---- */
  static uint32_t hold_tick;
  (void)DWT_GetDeltaT(&hold_tick); /* 维持计时器，避免复位后第一次 dt 巨大 */

  const DM_Motor_Measure_s m1 = robot->joint_motor[0]->measure;
  const DM_Motor_Measure_s m2 = robot->joint_motor[1]->measure;
  /* 电机原始角 -> 契约角。**只有重力前馈吃原始角**（GravityComp_Calc
   * 内部自己再换算一次），阻抗项和限位都用契约角，避免两边不同系。 */
  const float q1 = (m1.total_angle - robot->q_zero[0]) * robot->q_sign[0];
  const float q2 = (m2.total_angle - robot->q_zero[1]) * robot->q_sign[1];

  /* 关节角限位：超限则卸力。这里没有"跳过并继续"的余地 —— 部署期臂是被
   * 参赛者推的，越界说明机械上出事了，安全态就是零力矩。 */
  if (q1 < ARM_DEPLOY_Q1_MIN - ARM_LIM_TRIP_MARGIN ||
      q1 > ARM_DEPLOY_Q1_MAX + ARM_LIM_TRIP_MARGIN ||
      q2 < ARM_DEPLOY_Q2_MIN - ARM_LIM_TRIP_MARGIN ||
      q2 > ARM_DEPLOY_Q2_MAX + ARM_LIM_TRIP_MARGIN) {
    ArmOutputTorque(0.0f, 0.0f);
    return;
  }

  float ff1 = 0.0f, ff2 = 0.0f;
  GravityComp_Calc(m1.total_angle, m2.total_angle, &ff1, &ff2);

  /* k_grav 只缩放重力项；阻抗项独立，便于调"手感"而不动补偿模型。
   * ff 是电机系力矩，阻抗项在契约系算完要乘回符号才同系。 */
  ff1 = ff1 * ARM_GRAV_SCALE +
        robot->q_sign[0] * (ARM_HOLD_KP * (hold_ref[0] - q1) -
                            ARM_HOLD_KD * m1.velocity * robot->q_sign[0]);
  ff2 = ff2 * ARM_GRAV_SCALE +
        robot->q_sign[1] * (ARM_HOLD_KP * (hold_ref[1] - q2) -
                            ARM_HOLD_KD * m2.velocity * robot->q_sign[1]);

  ArmOutputTorque(ff1, ff2);
#endif
}
