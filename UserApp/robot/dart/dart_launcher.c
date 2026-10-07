/**
 * @file dart_launcher.c
 * @brief dart 发射架执行层实现: 零位校准 / 储能 / 发射(舵机开环) / 射力调节 / yaw 角度环
 */
#include "dart_launcher.h"

#include "dart_buzzer.h"

#include <math.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "robot_config.h"
#include "user_lib.h"

/* ==================== 内部工具 ==================== */

/* 校准完成前严格限制同步带电机力矩(所有模式生效), 防止卡死损坏 */
static void ApplyBeltTorqueLimit(DartLauncherInstance* inst) {
  float max_out = inst->is_calibrated ? DART_BELT_MAX_OUT : DART_BELT_CALI_MAX_OUT;
  float integral = inst->is_calibrated ? DART_BELT_INTEGRAL_LIMIT : DART_BELT_CALI_INTEGRAL_LIMIT;
  for (int i = 0; i < 2; i++) {
    inst->belt_motor[i]->motor_controller.speed_PID.MaxOut = max_out;
    inst->belt_motor[i]->motor_controller.speed_PID.IntegralLimit = integral;
  }
}

/* 校准完成前严格限制扳机丝杆力矩(顶硬限位时防止卡死损坏) */
static void ApplyScrewTorqueLimit(DartLauncherInstance* inst) {
  float max_out = inst->is_calibrated ? DART_SCREW_MAX_OUT : DART_SCREW_CALI_MAX_OUT;
  float integral = inst->is_calibrated ? DART_SCREW_INTEGRAL_LIMIT : DART_SCREW_CALI_INTEGRAL_LIMIT;
  inst->screw_motor->motor_controller.speed_PID.MaxOut = max_out;
  inst->screw_motor->motor_controller.speed_PID.IntegralLimit = integral;
}

/* 双带速度环积分互融 + 差模力矩偏置注入(每周期)
 *   shared = (I0+I1)/2 —— 差模偏置在求和中抵消, 即"共模积分";
 *   两台 Iout 向 shared ± belt_diff_torque 靠拢(系数 DART_BELT_INTEGRAL_BLEND):
 *   - 互融: 摩擦差异/起动不对称不再被各自积分"记住"固化(修稳态力矩差);
 *   - 差模: 位置纠偏以力矩偏置 ±Ib 注入, 两台速度给定保持相同 -> P 项一致, 电流差只剩 2*Ib(有上限)。 */
static void BlendBeltIntegrals(DartLauncherInstance* inst) {
  PIDInstance* pid0 = &inst->belt_motor[0]->motor_controller.speed_PID;
  PIDInstance* pid1 = &inst->belt_motor[1]->motor_controller.speed_PID;
  float shared = 0.5f * (pid0->Iout + pid1->Iout);
  float bias = inst->belt_zero_valid ? inst->belt_diff_torque : 0.0f;
  float blend = DART_BELT_INTEGRAL_BLEND;
  float target0 = shared + bias;
  float target1 = shared - bias;
  pid0->Iout += blend * (target0 - pid0->Iout);
  pid1->Iout += blend * (target1 - pid1->Iout);
}

/* 双同步带电机同速控制(速度环), 逻辑正方向 = 储能方向 */
static void SetBeltSpeed(DartLauncherInstance* inst, float speed_dps) {
  for (int i = 0; i < 2; i++) {
    DJIMotorOuterLoop(inst->belt_motor[i], SPEED_LOOP);
    DJIMotorSetPIDRef(inst->belt_motor[i], speed_dps);
  }
}

/* 双带共模/差模解耦(每周期一次, 同步带位置控制的唯一出口) —— 差模在**力矩级**
 *   共模: 两侧平均位置跟目标 -> 速度给定 v_common, 两台**完全相同**(P 项一致, 稳态出力均分)
 *   差模: 位置差收敛到基准 -> 输出**力矩偏置**(经积分互融注入两台速度环 Iout, ±Ib, 有上限)
 *   @note 差模不能走速度给定: v_ref 差经速度环 Kp 放大成 P 项差(2*Kp_s*v_diff),
 *         会持续制造几千的 real_current 差 —— 积分互融压不掉 P 项。力矩级注入后,
 *         稳态电流差 = 2*Ib, 仅在需要对齐时存在且被 DART_BELT_SYNC_TORQUE_MAX 钳幅。 */
static void BeltSyncUpdate(DartLauncherInstance* inst) {
  float p0 = inst->belt_motor[0]->measure.total_angle;
  float p1 = inst->belt_motor[1]->measure.total_angle;
  float dev = p0 - p1;
  /* 负载感知模式切换(判据滤波 + 回差 + 最短驻留三重防抖), 判据 = 两台 real_current **绝对值之和**:
   *   > DART_BELT_SYNC_LOAD_ON  -> **力矩同步**(带载: 拉拽/保持): 负载路径不对称(皮带张紧/摩擦)是
   *     机械事实, 硬锁偏差=持续较劲=电流差; 软基准放行慢漂移, 只纠窜动 -> 均流;
   *   < DART_BELT_SYNC_LOAD_OFF -> **位置同步**(空载移动): 硬基准锁死偏差值 -> 两侧走位齐。
   *   ON/OFF 之间保持原模式(回差); 判据先低通(real_current 噪声大, 不滤会在阈值附近反复横跳),
   *   切换后最短驻留一段时间(切换本身有扰动: 清差模积分/基准跳变), 不许连续翻转。 */
  float load_raw = fabsf(inst->belt_motor[0]->measure.real_current) + fabsf(inst->belt_motor[1]->measure.real_current);
  inst->belt_load_filt += (load_raw - inst->belt_load_filt) * (DART_TASK_DT_S / DART_BELT_SYNC_LOAD_LPF_TAU_S);
  bool torque_sync = inst->belt_torque_sync;
  if (inst->belt_load_filt > DART_BELT_SYNC_LOAD_ON) {
    torque_sync = true;
  } else if (inst->belt_load_filt < DART_BELT_SYNC_LOAD_OFF) {
    torque_sync = false;
  }
  uint32_t mode_ms = (uint32_t)DWT_GetTimeline_ms();
  if (torque_sync != inst->belt_torque_sync && (mode_ms - inst->belt_mode_switch_ms) < DART_BELT_SYNC_MODE_DWELL_MS) {
    torque_sync = inst->belt_torque_sync;  // 驻留期内不切换
  }
  if (torque_sync != inst->belt_torque_sync) {
    inst->belt_diff_pid.Iout = 0.0f;
    inst->belt_torque_sync = torque_sync;
    inst->belt_mode_switch_ms = mode_ms;
    LOGINFO("[dart] belt sync mode -> %s (load %d)", torque_sync ? "torque-balance" : "position-lock",
            (int)inst->belt_load_filt);
  }

  if (inst->belt_torque_sync && DART_BELT_SYNC_BASELINE_TAU_S > 0.0f) {
    inst->belt_diff_ref += (dev - inst->belt_diff_ref) * (DART_TASK_DT_S / DART_BELT_SYNC_BASELINE_TAU_S);
  } else {
    inst->belt_diff_ref = inst->belt_diff_base;
  }
  float v_common = PIDCalculate(&inst->belt_common_pid, 0.5f * (p0 + p1), inst->belt_common_ref);
  inst->belt_diff_torque = PIDCalculate(&inst->belt_diff_pid, dev - inst->belt_diff_ref, 0.0f);
  VAL_LIMIT(inst->belt_diff_torque, -DART_BELT_SYNC_TORQUE_MAX, DART_BELT_SYNC_TORQUE_MAX);
  VAL_LIMIT(v_common, -DART_BELT_MAX_SPEED_DPS, DART_BELT_MAX_SPEED_DPS);
  for (int i = 0; i < 2; i++) {
    DJIMotorOuterLoop(inst->belt_motor[i], SPEED_LOOP);
    DJIMotorSetPIDRef(inst->belt_motor[i], v_common);  // 两机同一速度给定
  }
}

/* 双同步带电机同一逻辑位置目标(共模/差模解耦), 保证均载与严格同步
 * speed_limit_dps: 本阶段限速, 通过共模环 MaxOut 实现, 再被 DART_BELT_MAX_SPEED_DPS 钳一次 */
static void SetBeltPosition(DartLauncherInstance* inst, float pos_deg, float speed_limit_dps) {
  inst->belt_pos_target = pos_deg;
  if (!inst->belt_zero_valid) {
    SetBeltSpeed(inst, 0.0f);  // 零点无效时只能速度环抱死, 禁止位置闭环
    return;
  }
  float limit = speed_limit_dps < DART_BELT_MAX_SPEED_DPS ? speed_limit_dps : DART_BELT_MAX_SPEED_DPS;
  inst->belt_common_pid.MaxOut = limit;
  // 共模目标 = 逻辑目标折算到两侧平均坐标; 差模基准 = 两侧"应有"的位置差(零点之差, 即逻辑差为 0)
  inst->belt_common_ref = 0.5f * (inst->belt_zero_offset[0] + inst->belt_zero_offset[1]) + pos_deg;
  inst->belt_diff_base = inst->belt_zero_offset[0] - inst->belt_zero_offset[1];
  BeltSyncUpdate(inst);
}

/* 双带是否到达逻辑目标位置: 以**两侧平均位置**判到位(挡块位置 ≈ 两侧平均)。
 * @warning 不能按"每侧各自 ±容差"判: 力矩同步模式下差模有意放行两侧变形差(均流),
 *          单侧误差可达上千度, 按单侧判会永远"没到位" -> 堵转/超时误报故障
 *          (踩过: 拉到位仍进 FAULT, 把容差改到 3000 才不报 —— 那是在掩盖判据错误)。
 *          单侧只留宽松健全性上限(END_TOL): 防"一侧到位另一侧卡死远端"的真异常。 */
static bool BeltPositionReached(DartLauncherInstance* inst, float pos_deg) {
  if (!inst->belt_zero_valid) return false;
  float e0 = (inst->belt_zero_offset[0] + pos_deg) - inst->belt_motor[0]->measure.total_angle;
  float e1 = (inst->belt_zero_offset[1] + pos_deg) - inst->belt_motor[1]->measure.total_angle;
  if (fabsf(0.5f * (e0 + e1)) > DART_BELT_POS_TOL_DEG) return false;
  if (fabsf(e0) > DART_BELT_END_TOL_DEG || fabsf(e1) > DART_BELT_END_TOL_DEG) return false;
  return true;
}

/* 校准回撤步的时间预算(回撤距离/限速 + 2s 余量), 超时判定与日志共用 */
static uint32_t BeltBackoffBudgetMs(void) {
  return (uint32_t)(DART_BELT_CALI_BACKOFF_DEG / DART_BELT_CALI_BACKOFF_SPEED_DPS * 1000.0f) + 4000u;
}

/* 扳机丝杆是否到达目标位置(射力位置, 相对丝杆零点) */
static bool ScrewPositionReached(DartLauncherInstance* inst) {
  if (!inst->screw_zero_valid) return false;
  float err = (inst->screw_zero_offset + inst->screw_pos_deg) - inst->screw_motor->measure.total_angle;
  return fabsf(err) < DART_SCREW_POS_TOL_DEG;
}

/* 扳机丝杆位置环输出(未校准时只能速度环抱死, 禁止位置闭环) */
static void ApplyScrewPosition(DartLauncherInstance* inst) {
  if (!inst->screw_zero_valid) {
    DJIMotorOuterLoop(inst->screw_motor, SPEED_LOOP);
    DJIMotorSetPIDRef(inst->screw_motor, 0.0f);
    return;
  }
  DJIMotorOuterLoop(inst->screw_motor, ANGLE_LOOP);
  DJIMotorSetPIDRef(inst->screw_motor, inst->screw_zero_offset + inst->screw_pos_deg);
}

/* 舵机角度 -> 占空比: 角度线性映射到脉宽, 脉宽/周期即占空比 */
static float ServoAngleToDuty(float angle_deg) {
  float span = DART_SERVO_ANGLE_MAX_DEG - DART_SERVO_ANGLE_MIN_DEG;
  float pulse_us = DART_SERVO_PULSE_MIN_US +
                   (angle_deg - DART_SERVO_ANGLE_MIN_DEG) * (DART_SERVO_PULSE_MAX_US - DART_SERVO_PULSE_MIN_US) / span;
  return (pulse_us * 1e-6f) / DART_SERVO_PWM_PERIOD_S;
}

/* 设定舵机角度(开环, 限幅到机械范围) */
static void SetServoAngle(DartLauncherInstance* inst, float angle_deg) {
  if (inst->servo_pwm == NULL) return;
  VAL_LIMIT(angle_deg, DART_SERVO_ANGLE_MIN_DEG, DART_SERVO_ANGLE_MAX_DEG);
  inst->servo_angle_deg = angle_deg;
  PWMSetDutyRatio(inst->servo_pwm, ServoAngleToDuty(angle_deg));
}

/* 堵转判据(唯一来源): 速度阈值。
 * 实测角速度(measure.speed_aps, 单位 deg/s)持续低于 DART_CALI_STALL_SPEED_DPS 且累计
 * 达到 DART_CALI_STALL_MS -> 判定堵转。调用侧负责起步宽限(DART_CALI_START_GRACE_MS)。
 * @note 不使用 PID 的 PID_ErrorHandle 堵转标志位: 它一旦置位(ERRORType)不手动清就一直挂着,
 *       会把"已经过去的堵转"带到后续阶段造成误故障; 速度阈值判据没有这种状态残留。 */
static bool CheckStall(StallDetector_s* det, DJIMotorInstance* motor, uint32_t now) {
  if (fabsf(motor->measure.speed_aps) > DART_CALI_STALL_SPEED_DPS) {
    det->tracking = false;
    return false;
  }
  if (!det->tracking) {
    det->tracking = true;
    det->start_ms = now;
    return false;
  }
  return (now - det->start_ms) >= DART_CALI_STALL_MS;
}

/* 进入新阶段/新步骤前清掉堵转记录 */
static void ResetStallState(DartLauncherInstance* inst) {
  for (int i = 0; i < 2; i++) {
    inst->stalled_flag[i] = false;
    inst->stall[i].tracking = false;
  }
  inst->screw_stall.tracking = false;
}

static void StopAllMotors(DartLauncherInstance* inst) {
  DJIMotorStop(inst->yaw_motor);
  for (int i = 0; i < 2; i++) DJIMotorStop(inst->belt_motor[i]);
  DJIMotorStop(inst->screw_motor);
}

static void EnableAllMotors(DartLauncherInstance* inst) {
  DJIMotorEnable(inst->yaw_motor);
  for (int i = 0; i < 2; i++) DJIMotorEnable(inst->belt_motor[i]);
  DJIMotorEnable(inst->screw_motor);
}

static void ClearBeltPid(DartLauncherInstance* inst) {
  for (int i = 0; i < 2; i++) {
    PIDClear(&inst->belt_motor[i]->motor_controller.speed_PID);
    PIDClear(&inst->belt_motor[i]->motor_controller.angle_PID);
  }
  PIDClear(&inst->belt_common_pid);
  PIDClear(&inst->belt_diff_pid);
}

static void ClearScrewPid(DartLauncherInstance* inst) {
  PIDClear(&inst->screw_motor->motor_controller.speed_PID);
  PIDClear(&inst->screw_motor->motor_controller.angle_PID);
}

static void EnterFault(DartLauncherInstance* inst) {
  /* @warning 故障**不断力矩**: 弹簧负载下断力矩=机构被拉簧带飞(失控加速+反拖发电,
   * 反电动势过压, 实车险些打坏硬件, 靠扳机卡住才没出事)。故障=保持当前位置(限力),
   * 由操作手排查; 只有失能(右开关下档)才真正断输出。 */
  inst->state = DART_STATE_FAULT;
  inst->cali_step = CALI_STEP_BELT_DRIVE_TO_STOP;
  inst->charge_step = CHARGE_STEP_PREP_SCREW;
  inst->fire_step = FIRE_STEP_RELEASE;
  SetServoAngle(inst, DART_SERVO_CATCH_DEG);  // 故障时舵机回卡位角(保持扣住)
  // 记录故障时刻位置为保持目标(逻辑坐标); 零点无效时退化为速度环 0 抱死
  if (inst->belt_zero_valid) {
    inst->fault_hold_pos = 0.5f * ((inst->belt_motor[0]->measure.total_angle - inst->belt_zero_offset[0]) +
                                   (inst->belt_motor[1]->measure.total_angle - inst->belt_zero_offset[1]));
  } else {
    inst->fault_hold_pos = 0.0f;
  }
  DartBuzzerFault();
}

/* 开始一次零位校准(当前只做同步带; 丝杆不自动校零) */
static void StartCalibration(DartLauncherInstance* inst, uint32_t now) {
  inst->is_calibrated = false;
  inst->belt_zero_valid = false;
  // 丝杆不参与自动校准: screw_zero_valid 保持有效(零点=开机位置), 否则位置环会一直抱死
  inst->recovery_retract = false;
  inst->cali_step = CALI_STEP_BELT_DRIVE_TO_STOP;
  inst->cali_start_ms = now;
  ResetStallState(inst);
  ClearBeltPid(inst);
  ClearScrewPid(inst);
  EnableAllMotors(inst);  // 故障恢复路径上电机可能处于停机状态
  inst->state = DART_STATE_CALIBRATING;
  DartBuzzerFaultCleared();
  DartBuzzerCaliStart();
  DartBuzzerCaliBgmStart();  // 校准 BGM(无刺有刺), 校准完成/故障/中止/失能即停
  LOGINFO("[dart] start zero calibration (belt only)");
}

/* ==================== 状态处理 ==================== */

static void HandleCalibrating(DartLauncherInstance* inst, uint32_t now) {
  switch (inst->cali_step) {
    case CALI_STEP_BELT_DRIVE_TO_STOP:
      ApplyScrewPosition(inst);  // 丝杆零点未标定前保持不动(速度环抱死)
      SetBeltSpeed(inst, DART_BELT_CALI_DIRECTION * DART_BELT_CALI_SPEED_DPS);
      // 堵转判据: 速度阈值(不用 PID 堵转标志位); 起步先宽限, 防速度还没上来就误判
      if (now - inst->step_start_ms > DART_CALI_START_GRACE_MS) {
        for (int i = 0; i < 2; i++) {
          if (CheckStall(&inst->stall[i], inst->belt_motor[i], now)) inst->stalled_flag[i] = true;
        }
      }
      if (inst->stalled_flag[0] && inst->stalled_flag[1]) {
        // 双电机各记硬限位处编码器值, 零点即完成同步
        for (int i = 0; i < 2; i++) {
          inst->belt_zero_offset[i] = inst->belt_motor[i]->measure.total_angle;
        }
        inst->belt_zero_valid = true;
        inst->belt_pos_target = DART_BELT_HOME_DEG;
        ClearBeltPid(inst);
        inst->cali_step = CALI_STEP_BELT_BACKOFF;
        inst->step_start_ms = now;
        inst->cali_log_ms = now;
        LOGINFO("[dart] belt zero found (L %d, R %d, diff %d), step 1/2 done; back off %d deg at %d deg/s (budget %d ms)",
                (int)inst->belt_zero_offset[0], (int)inst->belt_zero_offset[1],
                (int)(inst->belt_zero_offset[0] - inst->belt_zero_offset[1]), (int)DART_BELT_CALI_BACKOFF_DEG,
                (int)DART_BELT_CALI_BACKOFF_SPEED_DPS, (int)BeltBackoffBudgetMs());
      } else if (now - inst->cali_start_ms > DART_BELT_CALI_TIMEOUT_MS) {
        LOGERROR("[dart] fault: belt calibration stall timeout");
        EnterFault(inst);
      }
      break;

    case CALI_STEP_BELT_BACKOFF:
      ApplyScrewPosition(inst);
      SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_CALI_BACKOFF_SPEED_DPS);
      if (BeltPositionReached(inst, DART_BELT_HOME_DEG)) {
        // 自动校准到此结束(只做同步带): 同步带两侧零点已同步;
        // 丝杆不做自动校零, 直接以"开机位置"为零点(见 DartLauncherInit 里的 screw_zero_valid)
        ResetStallState(inst);
        ClearScrewPid(inst);
        inst->is_calibrated = true;
        inst->state = DART_STATE_IDLE;
        inst->cali_step = CALI_STEP_BELT_DRIVE_TO_STOP;
        DartBuzzerCaliDone();
        LOGINFO("[dart] calibration done (belt only): is_calibrated=1, belt_zero_valid=%d",
                inst->belt_zero_valid ? 1 : 0);
      } else {
        // 回撤超时按"回撤距离 / 回撤限速"动态算, 不再用为顶限位步设的通用超时:
        // 例: BACKOFF=3600° @20°/s 需要 180s, 而 DART_BELT_CALI_TIMEOUT_MS=8s -> 必然误报故障
        uint32_t budget_ms = BeltBackoffBudgetMs();
        // 1Hz 打印回撤进度(离目标还差多少): 判据是 |两带各自的位置误差| <= DART_BELT_POS_TOL_DEG
        if (now - inst->cali_log_ms >= 1000u) {
          inst->cali_log_ms = now;
          LOGINFO("[dart] backoff %d/%d ms, L err %d, R err %d, tol %d", (int)(now - inst->step_start_ms),
                  (int)budget_ms,
                  (int)((inst->belt_zero_offset[0] + DART_BELT_HOME_DEG) - inst->belt_motor[0]->measure.total_angle),
                  (int)((inst->belt_zero_offset[1] + DART_BELT_HOME_DEG) - inst->belt_motor[1]->measure.total_angle),
                  (int)DART_BELT_POS_TOL_DEG);
        }
        if (now - inst->step_start_ms > budget_ms) {
          LOGERROR("[dart] fault: belt calibration backoff timeout");
          EnterFault(inst);
        }
      }
      break;

#if 0 /* ===== 扳机丝杆零位校准: 自动校准中已停用(按要求注释保留, 未编译) =====
       * 恢复方法:
       *   1. 本段 #if 0 改回 #if 1;
       *   2. 上面 BELT_BACKOFF 完成分支改回: ResetStallState/ClearScrewPid 后
       *      cali_step = CALI_STEP_SCREW_DRIVE_TO_STOP, 并去掉 is_calibrated/state/cali_step 三行;
       *   3. StartCalibration()、失能中止(DartLauncherSetEnable)、调试档中止(DartLauncherTask)
       *      三处恢复 inst->screw_zero_valid = false;
       *   4. DartLauncherInit() 去掉 screw_zero_valid = true; (改为由校准置位) */
    case CALI_STEP_SCREW_DRIVE_TO_STOP:
      SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_HOME_SPEED_DPS);  // 带保持释放位置
      // 丝杆低速顶硬限位(力矩由 ApplyScrewTorqueLimit 严格限制)
      DJIMotorOuterLoop(inst->screw_motor, SPEED_LOOP);
      DJIMotorSetPIDRef(inst->screw_motor, DART_SCREW_CALI_DIRECTION * DART_SCREW_CALI_SPEED_DPS);
      // 同上: 速度阈值判据 + 起步宽限
      if (now - inst->step_start_ms > DART_CALI_START_GRACE_MS &&
          CheckStall(&inst->screw_stall, inst->screw_motor, now)) {
        // 记丝杆零点, 并退开限位到默认射力位置
        inst->screw_zero_offset = inst->screw_motor->measure.total_angle;
        inst->screw_zero_valid = true;
        ClearScrewPid(inst);
        float target = DART_SCREW_POS_DEFAULT_DEG > DART_SCREW_CALI_BACKOFF_DEG ? DART_SCREW_POS_DEFAULT_DEG
                                                                                : DART_SCREW_CALI_BACKOFF_DEG;
        inst->screw_pos_deg = target;
        inst->cali_step = CALI_STEP_SCREW_BACKOFF;
        inst->step_start_ms = now;
        LOGINFO("[dart] screw zero found, move to default power position");
      } else if (now - inst->step_start_ms > DART_SCREW_CALI_TIMEOUT_MS) {
        LOGERROR("[dart] fault: screw calibration stall timeout");
        EnterFault(inst);
      }
      break;

    case CALI_STEP_SCREW_BACKOFF:
      SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_HOME_SPEED_DPS);
      ApplyScrewPosition(inst);
      if (ScrewPositionReached(inst)) {
        inst->is_calibrated = true;
        inst->state = DART_STATE_IDLE;
        inst->cali_step = CALI_STEP_BELT_DRIVE_TO_STOP;
        DartBuzzerCaliDone();
        LOGINFO("[dart] calibration done (belt + screw)");
      } else if (now - inst->step_start_ms > DART_SCREW_CALI_TIMEOUT_MS) {
        LOGERROR("[dart] fault: screw calibration backoff timeout");
        EnterFault(inst);
      }
      break;
#endif

    default:
      inst->cali_step = CALI_STEP_BELT_DRIVE_TO_STOP;
      break;
  }
}

static void HandleIdle(DartLauncherInstance* inst, uint32_t now) {
  (void)now;
  // @note 失能中止储能后的复位(recovery_retract)已提到 DartLauncherTask 里统一处理;
  //       放在这里会被调试档(跳过 HandleIdle)漏掉 -> 标志永不清, 之后储能命令一直被拒
  SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_HOME_SPEED_DPS);
}

static void HandleCharging(DartLauncherInstance* inst, uint32_t now) {
  switch (inst->charge_step) {
    case CHARGE_STEP_PREP_SCREW:
      // 拉伸量由扳机丝杆位置(射力)决定, 必须先就位再拉拽
      SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_HOME_SPEED_DPS);
      if (ScrewPositionReached(inst)) {
        ResetStallState(inst);
        inst->charge_step = CHARGE_STEP_DRIVE;
        inst->step_start_ms = now;
      } else if (now - inst->step_start_ms > DART_SCREW_SETTLE_TIMEOUT_MS) {
        // 把目标/实测/容差打出来: 是"没走到"还是"容差太紧"一眼可辨
        LOGERROR("[dart] fault: screw not in position before charge (tgt %d, cur %d, tol %d, zero_valid %d)",
                 (int)(inst->screw_zero_offset + inst->screw_pos_deg), (int)inst->screw_motor->measure.total_angle,
                 (int)DART_SCREW_POS_TOL_DEG, inst->screw_zero_valid ? 1 : 0);
        EnterFault(inst);
      }
      break;

    case CHARGE_STEP_DRIVE: {
      // 扳机锁定态是单向通道, 平台会滑过扳机被继续后拉, 储能不会堵转, 按行程判完成
      SetBeltPosition(inst, DART_BELT_CHARGE_DEG, DART_BELT_CHARGE_SPEED_DPS);
      if (BeltPositionReached(inst, DART_BELT_CHARGE_DEG)) {
        inst->charge_step = CHARGE_STEP_RETRACT;
        inst->step_start_ms = now;
        LOGINFO("[dart] charge stroke done, reset to release position");
      } else {
        // 堵转看门狗: 同步带机械耦合, **两侧同时转不动**才算顶死(超滑台行程等异常);
        // 单侧慢=负载不均/偏差, 由差模环自动纠偏处理, 不停机不故障
        bool stalled = false;
        if (now - inst->step_start_ms > DART_CALI_START_GRACE_MS) {
          stalled = true;
          for (int i = 0; i < 2; i++) {
            if (!CheckStall(&inst->stall[i], inst->belt_motor[i], now)) stalled = false;
          }
        }
        if (stalled) {
          // 两侧同时顶死: 离目标近 = 拉到机械末端(正常完成!); 离目标远 = 真卡滞/超行程(故障)
          // (踩过: 目标恰好在机械末端 -> 到位容差永远判不到 -> 误报故障 -> 断力矩被拉簧带飞)
          float err0 = fabsf((inst->belt_zero_offset[0] + DART_BELT_CHARGE_DEG) - inst->belt_motor[0]->measure.total_angle);
          float err1 = fabsf((inst->belt_zero_offset[1] + DART_BELT_CHARGE_DEG) - inst->belt_motor[1]->measure.total_angle);
          float err_max = err0 > err1 ? err0 : err1;
          if (err_max < DART_BELT_END_TOL_DEG) {
            ResetStallState(inst);
            inst->charge_step = CHARGE_STEP_RETRACT;
            inst->step_start_ms = now;
            LOGINFO("[dart] charge stroke done at hard stop (err %d deg), reset to release position", (int)err_max);
          } else {
            LOGERROR("[dart] fault: belt stalled during charge far from target (err %d deg, check charge travel)", (int)err_max);
            EnterFault(inst);
          }
        } else if (now - inst->step_start_ms > DART_CHARGE_TIMEOUT_MS) {
          LOGERROR("[dart] fault: charge timeout");
          EnterFault(inst);
        }
      }
      break;
    }

    case CHARGE_STEP_RETRACT:
      // 限速复位到释放位置: 挡块脱离发射平台活动范围, 避免阻挡发射体
      SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_HOME_SPEED_DPS);
      if (BeltPositionReached(inst, DART_BELT_HOME_DEG)) {
        inst->state = DART_STATE_READY;
        DartBuzzerChargeDone();
        LOGINFO("[dart] charged, ready to fire");
      } else if (now - inst->step_start_ms > DART_RETRACT_TIMEOUT_MS) {
        LOGERROR("[dart] fault: retract timeout");
        EnterFault(inst);
      }
      break;

    default:
      inst->charge_step = CHARGE_STEP_PREP_SCREW;
      break;
  }
}

static void HandleReady(DartLauncherInstance* inst, uint32_t now) {
  (void)now;
  SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_HOME_SPEED_DPS);
}

static void HandleFiring(DartLauncherInstance* inst, uint32_t now) {
  // 舵机无位置反馈, 发射序列全部按时间推进
  switch (inst->fire_step) {
    case FIRE_STEP_RELEASE:
      SetServoAngle(inst, DART_SERVO_RELEASE_DEG);
      if (now - inst->step_start_ms >= DART_SERVO_SETTLE_MS) {
        inst->fire_step = FIRE_STEP_DWELL;
        inst->step_start_ms = now;
      }
      break;

    case FIRE_STEP_DWELL:
      if (now - inst->step_start_ms >= DART_FIRE_DWELL_MS) {
        inst->fire_step = FIRE_STEP_RESET;
        inst->step_start_ms = now;
      }
      break;

    case FIRE_STEP_RESET:
      SetServoAngle(inst, DART_SERVO_CATCH_DEG);
      if (now - inst->step_start_ms >= DART_SERVO_SETTLE_MS) {
        inst->fire_step = FIRE_STEP_RELEASE;
        inst->state = DART_STATE_IDLE;
        DartBuzzerFireDone();
        LOGINFO("[dart] fire sequence done, back to idle");
      }
      break;

    default:
      inst->fire_step = FIRE_STEP_RELEASE;
      break;
  }
}

/* 消费一次性命令 */
static void HandleCommand(DartLauncherInstance* inst, uint32_t now) {
  Dart_Cmd_e cmd = inst->pending_cmd;
  inst->pending_cmd = DART_CMD_NONE;
  if (cmd == DART_CMD_NONE) return;

  switch (cmd) {
    case DART_CMD_CALIBRATE:
      if (inst->debug_jog) {
        // 调试档 = 纯手动点动: 校准/储能/发射命令一律拒绝(否则状态机会抢占点动); 请拨回中档再操作
        LOGWARNING("[dart] calibrate command ignored in debug mode (switch to normal mode)");
        DartBuzzerCmdRejected();
      } else if (inst->state == DART_STATE_IDLE || inst->state == DART_STATE_FAULT) {
        StartCalibration(inst, now);
      } else {
        LOGWARNING("[dart] calibrate command ignored");
        DartBuzzerCmdRejected();
      }
      break;

    case DART_CMD_CHARGE:
      if (inst->debug_jog) {
        // 调试档 = 纯手动: 校准/储能/发射命令一律拒绝(否则状态机会从点动手里抢占机构)
        LOGWARNING("[dart] charge command ignored in debug mode (switch to normal mode)");
        DartBuzzerCmdRejected();
      } else if (inst->state == DART_STATE_IDLE && inst->is_calibrated && !inst->recovery_retract) {
        inst->charge_step = CHARGE_STEP_PREP_SCREW;
        inst->step_start_ms = now;
        inst->state = DART_STATE_CHARGING;
        DartBuzzerChargeStart();
        LOGINFO("[dart] charge sequence start (belt target %d deg, screw pos %d deg)", (int)DART_BELT_CHARGE_DEG,
                (int)inst->screw_pos_deg);
      } else if (inst->recovery_retract) {
        // 上一次储能被失能中止, 挡块还没回到释放位置: 正在自动复位, 复位完成后重新拨一次即可
        LOGWARNING("[dart] charge ignored: recovery retract in progress (block not at release position yet)");
        DartBuzzerCmdRejected();
      } else {
        // 三个门槛: 必须在 IDLE / 已完成校准 / 无待复位动作。日志把实际值打出来, 便于定位被拒原因
        LOGWARNING("[dart] charge ignored: state=%d (need IDLE=%d), is_calibrated=%d, recovery_retract=%d",
                   (int)inst->state, (int)DART_STATE_IDLE, inst->is_calibrated ? 1 : 0,
                   inst->recovery_retract ? 1 : 0);
        DartBuzzerCmdRejected();
      }
      break;

    case DART_CMD_FIRE:
      if (inst->debug_jog) {
        LOGWARNING("[dart] fire command ignored in debug mode (switch to normal mode)");
        DartBuzzerCmdRejected();
      } else if (inst->state == DART_STATE_READY) {
        inst->fire_step = FIRE_STEP_RELEASE;
        inst->step_start_ms = now;
        inst->state = DART_STATE_FIRING;
        DartBuzzerFire();
        LOGINFO("[dart] fire sequence start");
      } else {
        LOGWARNING("[dart] fire command ignored");
        DartBuzzerCmdRejected();
      }
      break;

    default:
      break;
  }
}

/* yaw: 角速度指令积分成角度目标, 角度环串级速度环输出 */
static void ApplyYaw(DartLauncherInstance* inst, float dt_s) {
  inst->yaw_angle_target += inst->yaw_rate_cmd_dps * dt_s;
  // 超前限幅防目标跑飞, 软限位限制机械行程
  float cur = inst->yaw_motor->measure.total_angle;
  VAL_LIMIT(inst->yaw_angle_target, cur - DART_YAW_LEAD_LIMIT_DEG, cur + DART_YAW_LEAD_LIMIT_DEG);
  VAL_LIMIT(inst->yaw_angle_target, inst->yaw_boot_angle - DART_YAW_SOFT_LIMIT_DEG,
            inst->yaw_boot_angle + DART_YAW_SOFT_LIMIT_DEG);
  DJIMotorOuterLoop(inst->yaw_motor, ANGLE_LOOP);
  DJIMotorSetPIDRef(inst->yaw_motor, inst->yaw_angle_target);
}

/* 双带电机同步偏差监测
 * @note 只在"需要同步"的阶段检查: 校准回撤时零点刚建立、两条带起步有先后(静摩擦释放),
 *       此时报偏差没有意义(实测会立刻误报); 储能/就绪/发射才是必须同步的窗口。
 *       偏差由差模环(BeltSyncUpdate)自动纠偏, 这里**只告警不停机**——不同步是被"解决"的,
 *       不作为故障条件(阈值 DART_BELT_SYNC_WARN_DEG)。 */
static void MonitorBeltSync(DartLauncherInstance* inst, uint32_t now) {
  if (!inst->belt_zero_valid) return;
  if (inst->state == DART_STATE_CALIBRATING) return;
  float pos0 = inst->belt_motor[0]->measure.total_angle - inst->belt_zero_offset[0];
  float pos1 = inst->belt_motor[1]->measure.total_angle - inst->belt_zero_offset[1];
  if (fabsf(pos0 - pos1) > DART_BELT_SYNC_WARN_DEG && now - inst->sync_warn_ms > DART_SYNC_WARN_PERIOD_MS) {
    inst->sync_warn_ms = now;
    LOGERROR("[dart] belt motors sync deviation exceeded: %d deg (L %d, R %d, thr %d)",
             (int)(pos0 - pos1), (int)pos0, (int)pos1, (int)DART_BELT_SYNC_WARN_DEG);
  }
}

/* 调试点动: 同步带走共模/差模(两侧共用虚拟目标 + 均载 + 自动纠偏), 丝杆走虚拟位置目标 + 位置环
 * @note 之前两侧各自跑速度环, 负载/摩擦不同会导致位置差累计、出力不均(一侧扛载一侧被拖着走);
 *       现在摇杆只给速率, 目标积分推进, 共模给两侧**相同**速度给定(均载), 差模自动收敛偏差。
 *       顶到机械限位/跟不上时暂停目标积分, 反向打杆先重同步, 不会出现"先往原方向走一段才反向"。 */
static void ApplyDebugJog(DartLauncherInstance* inst, float dt_s, uint32_t now) {
  DJIMotorInstance* belt_l = inst->belt_motor[0];
  DJIMotorInstance* belt_r = inst->belt_motor[1];

  if (!inst->debug_belt_synced) {
    // 进入点动/重新取得控制权: 目标取两侧当前位置平均(两侧已有偏差时会被差模收敛)
    inst->debug_belt_target = 0.5f * (belt_l->measure.total_angle + belt_r->measure.total_angle);
    inst->debug_belt_dev_base = belt_l->measure.total_angle - belt_r->measure.total_angle;
    inst->belt_diff_base = inst->debug_belt_dev_base;
    inst->belt_diff_ref = inst->debug_belt_dev_base;
    inst->debug_belt_synced = true;
  }
  if (!inst->debug_screw_synced) {
    inst->debug_screw_target = inst->screw_motor->measure.total_angle;
    inst->debug_screw_synced = true;
  }

  // ---- 同步带: 摇杆给速率 -> 积分出两侧共用的虚拟位置目标 ----
  float actual = 0.5f * (belt_l->measure.total_angle + belt_r->measure.total_angle);
  float rate = inst->debug_belt_speed_dps;
  float follow_err = inst->debug_belt_target - actual;
  // 反向指令: 先把目标重同步到实际位置。否则目标还停在上次的位置(超前), 会先"回绕"误差
  if (rate != 0.0f && follow_err != 0.0f && ((rate > 0.0f) != (follow_err > 0.0f))) {
    inst->debug_belt_target = actual;
    follow_err = 0.0f;
  }
  // 摇杆松手沿: 点动中目标领跑实际最多 MAX_ERR(追杆滞后), 松手若只冻结目标, 机构会继续
  // 追这段滞后量("松手后飘一段才停", 回中限速小追得还慢) -> 把目标一次性收回实际, 立即停住,
  // 之后目标保持(位置环顶住, 不会被手推走)
  if (rate == 0.0f && inst->debug_belt_rate_last != 0.0f) {
    inst->debug_belt_target = actual;
    follow_err = 0.0f;
  }
  inst->debug_belt_rate_last = rate;
  // 目标超前实测过多(顶到机械限位/电机跟不上)时暂停积分, 避免摇杆反向时要先"回绕"误差
  if (fabsf(follow_err) < DART_DEBUG_BELT_MAX_ERR_DEG) {
    inst->debug_belt_target += rate * dt_s;
  }
  // 共模/差模: 限速放共模环(摇杆回中后仍保留较小限速用于收敛); 差模基准由 BeltSyncUpdate 软基准维护
  float limit = fabsf(inst->debug_belt_speed_dps);
  if (limit < DART_DEBUG_BELT_SYNC_SPEED_DPS) limit = DART_DEBUG_BELT_SYNC_SPEED_DPS;
  inst->belt_common_pid.MaxOut = limit;
  inst->belt_common_ref = inst->debug_belt_target;
  BeltSyncUpdate(inst);

  // ---- 丝杆: 速度前馈 + P 跟踪(虚拟位置目标) ----
  // 纯位置环追杆的速度上限 = 角度环Kp × 跟踪误差上限(15×200°=3000°/s), 带载更慢 -> 点动发肉;
  // 前馈把摇杆速率**直接**给速度环, P 只负责收敛误差: 响应即时(不再靠拉大角度环Kp硬抬),
  // 松手后仍由虚拟目标顶住不滑走, 顶限位暂停积分/反向重同步保护不变。
  // READY 态不许动丝杆: 平台被扳机锁住、拉簧带载, 移动丝杆=带载推棘爪(误释放/卡滞风险),
  // 保持射力位置, 点动只给同步带; IDLE 态丝杆照常点动
  if (inst->state != DART_STATE_READY) {
  float s_actual = inst->screw_motor->measure.total_angle;
  float s_rate = inst->debug_screw_speed_dps;
  float s_follow_err = inst->debug_screw_target - s_actual;
  if (s_rate != 0.0f && s_follow_err != 0.0f && ((s_rate > 0.0f) != (s_follow_err > 0.0f))) {
    inst->debug_screw_target = s_actual;
    s_follow_err = 0.0f;
  }
  // 松手沿同同步带: 目标收回实际, 立即停住(否则追杆滞后量会"松手后飘一段")
  if (s_rate == 0.0f && inst->debug_screw_rate_last != 0.0f) {
    inst->debug_screw_target = s_actual;
    s_follow_err = 0.0f;
  }
  inst->debug_screw_rate_last = s_rate;
  if (fabsf(s_follow_err) < DART_DEBUG_SCREW_MAX_ERR_DEG) {
    inst->debug_screw_target += s_rate * dt_s;
  }
  float s_v_cmd = s_rate + DART_DEBUG_SCREW_KP * s_follow_err;
  float s_cap = (s_rate == 0.0f) ? DART_DEBUG_SCREW_HOLD_SPEED_DPS : DART_SCREW_MAX_SPEED_DPS;
  VAL_LIMIT(s_v_cmd, -s_cap, s_cap);
  DJIMotorOuterLoop(inst->screw_motor, SPEED_LOOP);
  DJIMotorSetPIDRef(inst->screw_motor, s_v_cmd);
  } else {
    ApplyScrewPosition(inst);  // 丝杆保持射力位置不动
  }

  // 两侧偏差监测(以进入点动时的位置差为基准) + 限频状态日志, 便于实机排查同步问题
  float dev = (belt_l->measure.total_angle - belt_r->measure.total_angle) - inst->debug_belt_dev_base;
  if (now - inst->debug_log_ms >= DART_DEBUG_LOG_PERIOD_MS) {
    inst->debug_log_ms = now;
    if (fabsf(dev) > DART_DEBUG_BELT_SYNC_WARN_DEG) {
      LOGERROR("[dart] debug jog belt sync dev %d deg (L %d, R %d, tgt %d)", (int)dev,
               (int)belt_l->measure.total_angle, (int)belt_r->measure.total_angle, (int)inst->debug_belt_target);
    } else {
      // cur = 实际电流, iout = 速度环积分项(f_Integral_Limit 硬钳在 DART_BELT_INTEGRAL_LIMIT):
      // 松手后若 iout 贴着积分限幅、位置还在往释放方向退, 说明保持力矩不够 -> 调大该宏
      LOGINFO("[dart] debug jog spd %d/%d pos %d/%d dev %d err %d cur %d/%d iout %d", (int)belt_l->measure.speed_aps,
              (int)belt_r->measure.speed_aps, (int)belt_l->measure.total_angle, (int)belt_r->measure.total_angle,
              (int)dev, (int)follow_err, (int)belt_l->measure.real_current, (int)belt_r->measure.real_current,
              (int)belt_l->motor_controller.speed_PID.Iout);
    }
  }
}

/* ==================== 对外接口 ==================== */

DartLauncherInstance* DartLauncherInit(void) {
  DartLauncherInstance* inst = (DartLauncherInstance*)zmalloc(sizeof(DartLauncherInstance));

  Motor_Init_Config_s yaw_conf = DART_YAW_MOTOR_CONFIG(DART_YAW_CAN, DART_YAW_CAN_ID);
  inst->yaw_motor = DJIMotorInit(&yaw_conf);

  Motor_Init_Config_s belt_l_conf = DART_BELT_MOTOR_CONFIG(DART_BELT_CAN, DART_BELT_L_CAN_ID, DART_BELT_L_REVERSE);
  inst->belt_motor[0] = DJIMotorInit(&belt_l_conf);
  Motor_Init_Config_s belt_r_conf = DART_BELT_MOTOR_CONFIG(DART_BELT_CAN, DART_BELT_R_CAN_ID, DART_BELT_R_REVERSE);
  inst->belt_motor[1] = DJIMotorInit(&belt_r_conf);

  Motor_Init_Config_s screw_conf = DART_SCREW_MOTOR_CONFIG(DART_SCREW_CAN, DART_SCREW_CAN_ID, DART_SCREW_REVERSE);
  inst->screw_motor = DJIMotorInit(&screw_conf);

  // 扳机舵机(PWM): 上电即输出卡位角(保持扣住发射平台), 占空比映射见 robot_config.h
  PWM_Init_Config_s servo_conf = {
      .htim = DART_SERVO_PWM_TIM,
      .channel = DART_SERVO_PWM_CHANNEL,
      .period = DART_SERVO_PWM_PERIOD_S,
      .dutyratio = ServoAngleToDuty(DART_SERVO_CATCH_DEG),
      .callback = NULL,
      .id = inst,
  };
  inst->servo_pwm = PWMRegister(&servo_conf);
  inst->servo_angle_deg = DART_SERVO_CATCH_DEG;

  inst->yaw_boot_angle = inst->yaw_motor->measure.total_angle;
  inst->yaw_angle_target = inst->yaw_boot_angle;
  // 丝杆不参与自动校准: 直接把零点定为开机位置(offset=0), 上电即允许位置环工作;
  // DART_SCREW_POS_* 因此是"相对开机位置"的角度(不同上电位置会差一个常量, 需要绝对重复性时再恢复丝杆校零)
  inst->screw_zero_offset = 0.0f;
  inst->screw_zero_valid = true;
  // 上电目标 = 0: 保持开机位置不动(不让扳机自己走到默认射力位置), 射力由操作手用右摇杆竖直调整;
  // 若希望上电就到默认射力位置, 把下面一行改成 DART_SCREW_POS_DEFAULT_DEG
  inst->screw_pos_deg = 0.0f;
  inst->belt_pos_target = DART_BELT_HOME_DEG;
  inst->cali_log_ms = 0;
  ResetStallState(inst);  // 堵转检测器/记录清零(速度阈值判据, 无残留状态)
  // 双带共模/差模解耦控制器(均载 + 自动纠偏), 参数见 robot_config.h DART_BELT_POS_* / DART_BELT_SYNC_*
  PID_Init_Config_s belt_common_cfg = {
      .Kp = DART_BELT_POS_KP,
      .Ki = DART_BELT_POS_KI,
      .Kd = 0.0f,
      .MaxOut = DART_BELT_MAX_SPEED_DPS,
      .IntegralLimit = 0.0f,
      .DeadBand = 0.0f,
      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,
  };
  PIDInit(&inst->belt_common_pid, &belt_common_cfg);
  PID_Init_Config_s belt_diff_cfg = {
      .Kp = DART_BELT_SYNC_KP,
      .Ki = DART_BELT_SYNC_KI,
      .Kd = DART_BELT_SYNC_KD,
      .MaxOut = DART_BELT_SYNC_TORQUE_MAX,
      .IntegralLimit = DART_BELT_SYNC_KI_LIMIT,  // 积分限幅 << 输出限幅, 防灌满卡死
      .DeadBand = 0.0f,
      .Derivative_LPF_RC = DART_BELT_SYNC_KD_LPF_RC,
      // 微分取测量值(=两侧位置差): D 项正比于两侧相对速度, 是"阻尼器"(修运动中一快一慢)
      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_Derivative_On_Measurement | PID_DerivativeFilter,
  };
  PIDInit(&inst->belt_diff_pid, &belt_diff_cfg);
  inst->state = DART_STATE_IDLE;
  inst->last_ms = (uint32_t)DWT_GetTimeline_ms();
  return inst;
}

void DartLauncherSetEnable(DartLauncherInstance* inst, bool enable, bool auto_cali) {
  if (inst == NULL) return;
  uint32_t now = (uint32_t)DWT_GetTimeline_ms();

  if (enable && !inst->enabled) {
    inst->enabled = true;
    inst->pending_cmd = DART_CMD_NONE;  // 清掉失能前潜伏的命令, 防止使能瞬间自动执行
    EnableAllMotors(inst);
    DartBuzzerEnableOk();
    // 重同步(实车教训): 失能期间机构可能被手推动, 重新使能时把 yaw 目标同步到当前位置并清 PID,
    // 避免使能瞬间输出跳变; 正常档位切换(持续使能)不做清零, 否则积分清零会丢保持力矩引起抽动
    inst->yaw_angle_target = inst->yaw_motor->measure.total_angle;
    PIDClear(&inst->yaw_motor->motor_controller.angle_PID);
    PIDClear(&inst->yaw_motor->motor_controller.speed_PID);
    ClearBeltPid(inst);
    ClearScrewPid(inst);
    // 失能时被中止的序列统一回 IDLE 兜底
    if (inst->state == DART_STATE_CALIBRATING || inst->state == DART_STATE_CHARGING ||
        inst->state == DART_STATE_FIRING) {
      if (inst->state == DART_STATE_CHARGING && inst->is_calibrated) inst->recovery_retract = true;
      inst->state = DART_STATE_IDLE;
    }
    // 上电后开始工作时必须先完成一次零位校准(调试档传 auto_cali=false: 先点动验方向, 手动校准)
    if (auto_cali && !inst->is_calibrated && inst->state == DART_STATE_IDLE && !inst->recovery_retract) {
      StartCalibration(inst, now);
    }
  } else if (!enable && inst->enabled) {
    switch (inst->state) {
      case DART_STATE_CALIBRATING:
        inst->state = DART_STATE_IDLE;
        inst->is_calibrated = false;
        inst->belt_zero_valid = false;
        break;
      case DART_STATE_CHARGING:
        inst->state = DART_STATE_IDLE;
        inst->charge_step = CHARGE_STEP_PREP_SCREW;
        if (inst->is_calibrated) inst->recovery_retract = true;
        break;
      case DART_STATE_FIRING:
        inst->state = DART_STATE_IDLE;
        inst->fire_step = FIRE_STEP_RELEASE;
        break;
      default:
        break;
    }
    inst->enabled = false;
    inst->pending_cmd = DART_CMD_NONE;  // 失能时清命令队列: 存在的命令不允许潜伏到下次使能后才执行
    StopAllMotors(inst);
    DartBuzzerDisable();
  }
}

void DartLauncherSetCommand(DartLauncherInstance* inst, Dart_Cmd_e cmd) {
  if (inst == NULL) return;
  inst->pending_cmd = cmd;
}

void DartLauncherAdjustScrewPos(DartLauncherInstance* inst, float delta_deg) {
  if (inst == NULL) return;
  // 仅 IDLE 生效: READY 时平台被扳机锁住、拉簧带载, 移动丝杆=带载推棘爪, 有误释放/卡滞风险;
  // 射力在待机调好后再储能(READY 也无法重新储能, 须发射回待机)
  if (inst->state != DART_STATE_IDLE) return;
  inst->screw_pos_deg += delta_deg;
  VAL_LIMIT(inst->screw_pos_deg, DART_SCREW_POS_MIN_DEG, DART_SCREW_POS_MAX_DEG);
}

void DartLauncherSetServoAngle(DartLauncherInstance* inst, float angle_deg) {
  if (inst == NULL) return;
  SetServoAngle(inst, angle_deg);
}

void DartLauncherAdjustServoAngle(DartLauncherInstance* inst, float delta_deg) {
  if (inst == NULL) return;
  // 仅 IDLE 生效: 序列中舵机由状态机接管, 防止摇杆误改卡位/释放角
  if (inst->state != DART_STATE_IDLE) return;
  SetServoAngle(inst, inst->servo_angle_deg + delta_deg);
}

void DartLauncherSetYawRate(DartLauncherInstance* inst, float rate_dps) {
  if (inst == NULL) return;
  inst->yaw_rate_cmd_dps = rate_dps;
}

void DartLauncherSetDebugJog(DartLauncherInstance* inst, bool enable, float belt_speed_dps, float screw_speed_dps) {
  if (inst == NULL) return;
  inst->debug_jog = enable;
  inst->debug_belt_speed_dps = belt_speed_dps;
  inst->debug_screw_speed_dps = screw_speed_dps;
}

void DartLauncherTask(DartLauncherInstance* inst) {
  if (inst == NULL || !inst->enabled) return;

  uint32_t now = (uint32_t)DWT_GetTimeline_ms();
  float dt_s = (float)(now - inst->last_ms) * 0.001f;
  if (dt_s <= 0.0f || dt_s > 0.05f) dt_s = DART_TASK_DT_S;
  inst->last_ms = now;

  // 校准完成前严格限制同步带/丝杆力矩(所有模式生效)
  ApplyBeltTorqueLimit(inst);
  ApplyScrewTorqueLimit(inst);
  BlendBeltIntegrals(inst);  // 双带速度环积分互融: 稳态力矩均分(修 real_current 一大一小)

  HandleCommand(inst, now);

  if (inst->state == DART_STATE_FAULT) {
    // 故障保持(不断力矩, 见 EnterFault 注释): 双带保持故障位置、丝杆保持、yaw 断力矩(无弹簧负载)
    SetBeltPosition(inst, inst->fault_hold_pos, DART_BELT_HOME_SPEED_DPS);
    ApplyScrewPosition(inst);
    DJIMotorStop(inst->yaw_motor);
    return;
  }

  // 调试档 = 纯手动: 拨到调试档(右开关上)时, 正在进行的自动校准/储能立即中止, 机构交还手动点动
  // (使能时右开关还没拨到上档/先在中档使能过, 都会遇到这种情况)
  if (inst->debug_jog && (inst->state == DART_STATE_CALIBRATING || inst->state == DART_STATE_CHARGING)) {
    if (inst->state == DART_STATE_CALIBRATING) {
      inst->is_calibrated = false;
      inst->belt_zero_valid = false;
      DartBuzzerCaliBgmStop();  // 校准中止即停 BGM
      LOGWARNING("[dart] auto calibration aborted: debug mode (jog only, commands rejected in debug)");
    } else {
      // 储能中止: 挡块可能已被拉离释放位置 -> 挂回撤标志, 回撤完成前点动不接管(复用失能中止的恢复逻辑)
      if (inst->is_calibrated && inst->charge_step != CHARGE_STEP_PREP_SCREW) inst->recovery_retract = true;
      inst->charge_step = CHARGE_STEP_PREP_SCREW;
      LOGWARNING("[dart] charge sequence aborted: debug mode");
    }
    inst->state = DART_STATE_IDLE;
    inst->debug_belt_synced = false;
    inst->debug_screw_synced = false;
  }

  // 失能中止储能后的复位: 优先于调试档点动和 IDLE 保持执行。
  // (原先只在 HandleIdle 里做, 调试档会跳过 HandleIdle -> recovery_retract 永不清, 储能命令一直被拒)
  if (inst->state == DART_STATE_IDLE && inst->recovery_retract) {
    SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_HOME_SPEED_DPS);
    if (BeltPositionReached(inst, DART_BELT_HOME_DEG)) {
      inst->recovery_retract = false;
      LOGINFO("[dart] recovery retract done (block back at release position)");
    }
  } else if (inst->debug_jog && (inst->state == DART_STATE_IDLE || inst->state == DART_STATE_READY)) {
    // 调试点动: IDLE 全部可动; READY 只动同步带(平台被扳机锁住, 丝杆/舵机锁定, 见 ApplyDebugJog)
    ApplyDebugJog(inst, dt_s, now);
  } else {
    // 点动结束/状态机接管: 下次点动重新同步虚拟目标
    inst->debug_belt_synced = false;
    inst->debug_screw_synced = false;
    switch (inst->state) {
      case DART_STATE_IDLE:
        HandleIdle(inst, now);
        break;
      case DART_STATE_CALIBRATING:
        HandleCalibrating(inst, now);
        break;
      case DART_STATE_CHARGING:
        HandleCharging(inst, now);
        break;
      case DART_STATE_READY:
        HandleReady(inst, now);
        break;
      case DART_STATE_FIRING:
        HandleFiring(inst, now);
        break;
      default:
        break;
    }
    // 发射序列中舵机由 HandleFiring 控制, 其余状态舵机保持卡位角
    if (inst->state != DART_STATE_FIRING) SetServoAngle(inst, DART_SERVO_CATCH_DEG);
    // 校准过程中丝杆由 HandleCalibrating 自己控制(顶限位/退回默认位置), 其余状态保持射力位置
    if (inst->state != DART_STATE_CALIBRATING) ApplyScrewPosition(inst);
  }

  ApplyYaw(inst, dt_s);
  MonitorBeltSync(inst, now);
}
