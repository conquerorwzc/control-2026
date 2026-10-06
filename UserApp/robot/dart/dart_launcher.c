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

/* 双同步带电机同速控制(速度环), 逻辑正方向 = 储能方向 */
static void SetBeltSpeed(DartLauncherInstance* inst, float speed_dps) {
  for (int i = 0; i < 2; i++) {
    DJIMotorOuterLoop(inst->belt_motor[i], SPEED_LOOP);
    DJIMotorSetPIDRef(inst->belt_motor[i], speed_dps);
  }
}

/* 双同步带电机同一逻辑位置目标(位置环串级), 保证严格同步
 * speed_limit_dps: 本阶段限速, 通过角度环 MaxOut 实现, 再被 DART_BELT_MAX_SPEED_DPS 钳一次 */
static void SetBeltPosition(DartLauncherInstance* inst, float pos_deg, float speed_limit_dps) {
  inst->belt_pos_target = pos_deg;
  if (!inst->belt_zero_valid) {
    SetBeltSpeed(inst, 0.0f);  // 零点无效时只能速度环抱死, 禁止位置闭环
    return;
  }
  float limit = speed_limit_dps < DART_BELT_MAX_SPEED_DPS ? speed_limit_dps : DART_BELT_MAX_SPEED_DPS;
  for (int i = 0; i < 2; i++) {
    inst->belt_motor[i]->motor_controller.angle_PID.MaxOut = limit;  // 限速(位置环输出=速度给定)
    DJIMotorOuterLoop(inst->belt_motor[i], ANGLE_LOOP);
    DJIMotorSetPIDRef(inst->belt_motor[i], inst->belt_zero_offset[i] + pos_deg);
  }
}

/* 双带电机是否都到达逻辑目标位置 */
static bool BeltPositionReached(DartLauncherInstance* inst, float pos_deg) {
  for (int i = 0; i < 2; i++) {
    float err = (inst->belt_zero_offset[i] + pos_deg) - inst->belt_motor[i]->measure.total_angle;
    if (fabsf(err) > DART_BELT_POS_TOL_DEG) return false;
  }
  return true;
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

/* 堵转检测: 速度低于阈值持续 need_ms 判定堵转 */
static bool CheckStall(StallDetector_s* det, DJIMotorInstance* motor, float speed_thresh_dps, uint32_t need_ms,
                       uint32_t now) {
  if (fabsf(motor->measure.speed_aps) > speed_thresh_dps) {
    det->tracking = false;
    return false;
  }
  if (!det->tracking) {
    det->tracking = true;
    det->start_ms = now;
    return false;
  }
  return (now - det->start_ms) >= need_ms;
}

/* PID 自带堵转检测(PID_ErrorHandle)作为备份判据, 读后清标志防死锁 */
static bool ConsumePidBlocked(DJIMotorInstance* motor) {
  PID_ErrorHandler_t* handler = &motor->motor_controller.speed_PID.ERRORHandler;
  if (handler->ERRORType == PID_MOTOR_BLOCKED_ERROR) {
    handler->ERRORType = PID_ERROR_NONE;
    handler->ERRORCount = 0;
    return true;
  }
  return false;
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

static void ResetStallDetectors(DartLauncherInstance* inst) {
  for (int i = 0; i < 2; i++) {
    inst->stall[i].tracking = false;
    inst->stall[i].start_ms = 0;
    inst->stalled_flag[i] = false;
  }
}

static void ClearBeltPid(DartLauncherInstance* inst) {
  for (int i = 0; i < 2; i++) {
    PIDClear(&inst->belt_motor[i]->motor_controller.speed_PID);
    PIDClear(&inst->belt_motor[i]->motor_controller.angle_PID);
  }
}

static void ClearScrewPid(DartLauncherInstance* inst) {
  PIDClear(&inst->screw_motor->motor_controller.speed_PID);
  PIDClear(&inst->screw_motor->motor_controller.angle_PID);
}

static void EnterFault(DartLauncherInstance* inst) {
  inst->state = DART_STATE_FAULT;
  inst->cali_step = CALI_STEP_BELT_DRIVE_TO_STOP;
  inst->charge_step = CHARGE_STEP_PREP_SCREW;
  inst->fire_step = FIRE_STEP_RELEASE;
  SetServoAngle(inst, DART_SERVO_CATCH_DEG);  // 故障时舵机回卡位角(保持扣住)
  StopAllMotors(inst);
  DartBuzzerFault();
}

/* 开始一次零位校准(同步带 -> 扳机丝杆) */
static void StartCalibration(DartLauncherInstance* inst, uint32_t now) {
  inst->is_calibrated = false;
  inst->belt_zero_valid = false;
  inst->screw_zero_valid = false;
  inst->recovery_retract = false;
  inst->cali_step = CALI_STEP_BELT_DRIVE_TO_STOP;
  inst->cali_start_ms = now;
  inst->screw_stall.tracking = false;
  ResetStallDetectors(inst);
  ClearBeltPid(inst);
  ClearScrewPid(inst);
  EnableAllMotors(inst);  // 故障恢复路径上电机可能处于停机状态
  inst->state = DART_STATE_CALIBRATING;
  DartBuzzerFaultCleared();
  DartBuzzerCaliStart();
  LOGINFO("[dart] start zero calibration (belt + screw)");
}

/* ==================== 状态处理 ==================== */

static void HandleCalibrating(DartLauncherInstance* inst, uint32_t now) {
  switch (inst->cali_step) {
    case CALI_STEP_BELT_DRIVE_TO_STOP:
      ApplyScrewPosition(inst);  // 丝杆零点未标定前保持不动(速度环抱死)
      SetBeltSpeed(inst, DART_BELT_CALI_DIRECTION * DART_BELT_CALI_SPEED_DPS);
      if (now - inst->step_start_ms > DART_CALI_START_GRACE_MS) {  // 起步宽限, 防起步误判堵转
        for (int i = 0; i < 2; i++) {
          bool stalled = CheckStall(&inst->stall[i], inst->belt_motor[i], DART_CALI_STALL_SPEED_DPS,
                                    DART_CALI_STALL_MS, now) ||
                         ConsumePidBlocked(inst->belt_motor[i]);
          if (stalled) inst->stalled_flag[i] = true;
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
        LOGINFO("[dart] belt zero found, back off to release position");
      } else if (now - inst->cali_start_ms > DART_BELT_CALI_TIMEOUT_MS) {
        LOGERROR("[dart] fault: belt calibration stall timeout");
        EnterFault(inst);
      }
      break;

    case CALI_STEP_BELT_BACKOFF:
      ApplyScrewPosition(inst);
      SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_CALI_BACKOFF_SPEED_DPS);
      if (BeltPositionReached(inst, DART_BELT_HOME_DEG)) {
        // 同步带完成, 接着校准扳机丝杆零点
        inst->screw_stall.tracking = false;
        ClearScrewPid(inst);
        inst->cali_step = CALI_STEP_SCREW_DRIVE_TO_STOP;
        inst->step_start_ms = now;
        LOGINFO("[dart] belt zero done, start screw zero calibration");
      } else if (now - inst->step_start_ms > DART_BELT_CALI_TIMEOUT_MS) {
        LOGERROR("[dart] fault: belt calibration backoff timeout");
        EnterFault(inst);
      }
      break;

    case CALI_STEP_SCREW_DRIVE_TO_STOP:
      SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_HOME_SPEED_DPS);  // 带保持释放位置
      // 丝杆低速顶硬限位(力矩由 ApplyScrewTorqueLimit 严格限制)
      DJIMotorOuterLoop(inst->screw_motor, SPEED_LOOP);
      DJIMotorSetPIDRef(inst->screw_motor, DART_SCREW_CALI_DIRECTION * DART_SCREW_CALI_SPEED_DPS);
      if (now - inst->step_start_ms > DART_CALI_START_GRACE_MS &&
          (CheckStall(&inst->screw_stall, inst->screw_motor, DART_CALI_STALL_SPEED_DPS, DART_CALI_STALL_MS, now) ||
           ConsumePidBlocked(inst->screw_motor))) {
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
        DartBuzzerCaliDone();
        LOGINFO("[dart] calibration done (belt + screw)");
      } else if (now - inst->step_start_ms > DART_SCREW_CALI_TIMEOUT_MS) {
        LOGERROR("[dart] fault: screw calibration backoff timeout");
        EnterFault(inst);
      }
      break;

    default:
      inst->cali_step = CALI_STEP_BELT_DRIVE_TO_STOP;
      break;
  }
}

static void HandleIdle(DartLauncherInstance* inst, uint32_t now) {
  (void)now;
  SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_HOME_SPEED_DPS);
  if (inst->recovery_retract) {
    if (BeltPositionReached(inst, DART_BELT_HOME_DEG)) {
      inst->recovery_retract = false;
      LOGINFO("[dart] recovery retract done");
    }
  }
}

static void HandleCharging(DartLauncherInstance* inst, uint32_t now) {
  switch (inst->charge_step) {
    case CHARGE_STEP_PREP_SCREW:
      // 拉伸量由扳机丝杆位置(射力)决定, 必须先就位再拉拽
      SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_HOME_SPEED_DPS);
      if (ScrewPositionReached(inst)) {
        ResetStallDetectors(inst);
        inst->charge_step = CHARGE_STEP_DRIVE;
        inst->step_start_ms = now;
      } else if (now - inst->step_start_ms > DART_SCREW_SETTLE_TIMEOUT_MS) {
        LOGERROR("[dart] fault: screw not in position before charge");
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
        // 堵转看门狗: 单向扳机不会阻挡平台, 顶死只可能是超过滑台行程等异常, 直接故障停机
        bool stalled = false;
        for (int i = 0; i < 2; i++) {
          bool motor_stalled = CheckStall(&inst->stall[i], inst->belt_motor[i], DART_CHARGE_STALL_SPEED_DPS,
                                          DART_CHARGE_STALL_MS, now) ||
                               ConsumePidBlocked(inst->belt_motor[i]);
          if (motor_stalled) stalled = true;
        }
        if (stalled) {
          LOGERROR("[dart] fault: belt stalled during charge (check charge travel vs slide range)");
          EnterFault(inst);
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
      if (inst->state == DART_STATE_IDLE || inst->state == DART_STATE_FAULT) {
        StartCalibration(inst, now);
      } else {
        LOGWARNING("[dart] calibrate command ignored");
        DartBuzzerCmdRejected();
      }
      break;

    case DART_CMD_CHARGE:
      if (inst->state == DART_STATE_IDLE && inst->is_calibrated && !inst->recovery_retract) {
        inst->charge_step = CHARGE_STEP_PREP_SCREW;
        inst->step_start_ms = now;
        inst->state = DART_STATE_CHARGING;
        DartBuzzerChargeStart();
        LOGINFO("[dart] charge sequence start");
      } else {
        LOGWARNING("[dart] charge command ignored");
        DartBuzzerCmdRejected();
      }
      break;

    case DART_CMD_FIRE:
      if (inst->state == DART_STATE_READY) {
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

/* 双带电机同步偏差监测 */
static void MonitorBeltSync(DartLauncherInstance* inst, uint32_t now) {
  if (!inst->belt_zero_valid) return;
  float pos0 = inst->belt_motor[0]->measure.total_angle - inst->belt_zero_offset[0];
  float pos1 = inst->belt_motor[1]->measure.total_angle - inst->belt_zero_offset[1];
  if (fabsf(pos0 - pos1) > DART_BELT_SYNC_WARN_DEG && now - inst->sync_warn_ms > DART_SYNC_WARN_PERIOD_MS) {
    inst->sync_warn_ms = now;
    LOGERROR("[dart] belt motors sync deviation exceeded");
  }
}

/* 调试点动: 同步带两侧共用同一"虚拟位置目标"(位置环)消除速度差累计, 丝杆速度环点动
 * @note 之前两侧各自跑速度环, 负载/摩擦不同会导致实际速度不一致, 位置差会一直累计;
 *       现在摇杆只给速率, 目标积分推进, 两侧位置环跟踪同一目标 -> 严格同步 */
static void ApplyDebugJog(DartLauncherInstance* inst, float dt_s, uint32_t now) {
  DJIMotorInstance* belt_l = inst->belt_motor[0];
  DJIMotorInstance* belt_r = inst->belt_motor[1];

  if (!inst->debug_belt_synced) {
    // 进入点动/重新取得控制权: 目标取两侧当前位置平均(两侧已有偏差时会被收敛到同一目标)
    inst->debug_belt_target = 0.5f * (belt_l->measure.total_angle + belt_r->measure.total_angle);
    inst->debug_belt_dev_base = belt_l->measure.total_angle - belt_r->measure.total_angle;
    inst->debug_belt_synced = true;
  }

  // 摇杆给速率 -> 积分出两侧共用的位置目标
  // 目标超前实测过多(顶到机械限位/电机跟不上)时暂停积分, 避免摇杆反向时要先"回绕"误差
  float follow_err = inst->debug_belt_target - 0.5f * (belt_l->measure.total_angle + belt_r->measure.total_angle);
  if (fabsf(follow_err) < DART_DEBUG_BELT_MAX_ERR_DEG) {
    inst->debug_belt_target += inst->debug_belt_speed_dps * dt_s;
  }

  // 位置环限速: 跟随摇杆速率; 摇杆回中后仍保留较小限速用于收敛两侧已有偏差
  float limit = fabsf(inst->debug_belt_speed_dps);
  if (limit < DART_DEBUG_BELT_SYNC_SPEED_DPS) limit = DART_DEBUG_BELT_SYNC_SPEED_DPS;
  for (int i = 0; i < 2; i++) {
    inst->belt_motor[i]->motor_controller.angle_PID.MaxOut = limit;
    DJIMotorOuterLoop(inst->belt_motor[i], ANGLE_LOOP);
    DJIMotorSetPIDRef(inst->belt_motor[i], inst->debug_belt_target);
  }

  // 丝杆点动: 单电机, 速度环即可
  DJIMotorOuterLoop(inst->screw_motor, SPEED_LOOP);
  DJIMotorSetPIDRef(inst->screw_motor, inst->debug_screw_speed_dps);

  // 两侧偏差监测(以进入点动时的位置差为基准) + 限频状态日志, 便于实机排查同步问题
  float dev = (belt_l->measure.total_angle - belt_r->measure.total_angle) - inst->debug_belt_dev_base;
  if (now - inst->debug_log_ms >= DART_DEBUG_LOG_PERIOD_MS) {
    inst->debug_log_ms = now;
    if (fabsf(dev) > DART_DEBUG_BELT_SYNC_WARN_DEG) {
      LOGERROR("[dart] debug jog belt sync dev %d deg (L %d, R %d, tgt %d)", (int)dev,
               (int)belt_l->measure.total_angle, (int)belt_r->measure.total_angle, (int)inst->debug_belt_target);
    } else {
      LOGINFO("[dart] debug jog spd %d/%d deg/s, pos %d/%d, dev %d", (int)belt_l->measure.speed_aps,
              (int)belt_r->measure.speed_aps, (int)belt_l->measure.total_angle, (int)belt_r->measure.total_angle,
              (int)dev);
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
  inst->screw_pos_deg = DART_SCREW_POS_DEFAULT_DEG;
  inst->belt_pos_target = DART_BELT_HOME_DEG;
  inst->state = DART_STATE_IDLE;
  inst->last_ms = (uint32_t)DWT_GetTimeline_ms();
  return inst;
}

void DartLauncherSetEnable(DartLauncherInstance* inst, bool enable, bool auto_cali) {
  if (inst == NULL) return;
  uint32_t now = (uint32_t)DWT_GetTimeline_ms();

  if (enable && !inst->enabled) {
    inst->enabled = true;
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
        inst->screw_zero_valid = false;
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
  if (inst->state != DART_STATE_IDLE && inst->state != DART_STATE_READY) return;
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

  HandleCommand(inst, now);

  if (inst->state == DART_STATE_FAULT) {
    StopAllMotors(inst);
    return;
  }

  if (inst->debug_jog && inst->state == DART_STATE_IDLE) {
    ApplyDebugJog(inst, dt_s, now);
  } else {
    inst->debug_belt_synced = false;  // 点动结束/状态机接管: 下次点动重新同步虚拟目标
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
