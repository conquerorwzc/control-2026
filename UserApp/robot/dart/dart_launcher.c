/**
 * @file dart_launcher.c
 * @brief dart 发射架执行层实现: 零位校准 / 储能 / 发射 / 扳机调力 / yaw 角度环
 */
#include "dart_launcher.h"

#include <math.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "robot_config.h"
#include "user_lib.h"

/* ==================== 内部工具 ==================== */

/* 校准完成前严格限制同步带电机力矩(所有模式生效), 防止卡死损坏 */
static void ApplyBeltTorqueLimit(DartLauncherInstance* inst) {
  float max_out = inst->is_calibrated ? DART_BELT_MAX_OUT : DART_CALI_MAX_OUT;
  float integral = inst->is_calibrated ? DART_BELT_INTEGRAL_LIMIT : DART_CALI_INTEGRAL_LIMIT;
  for (int i = 0; i < 2; i++) {
    inst->belt_motor[i]->motor_controller.speed_PID.MaxOut = max_out;
    inst->belt_motor[i]->motor_controller.speed_PID.IntegralLimit = integral;
  }
}

/* 双同步带电机同速控制(速度环), 逻辑正方向 = 储能方向 */
static void SetBeltSpeed(DartLauncherInstance* inst, float speed_dps) {
  for (int i = 0; i < 2; i++) {
    DJIMotorOuterLoop(inst->belt_motor[i], SPEED_LOOP);
    DJIMotorSetPIDRef(inst->belt_motor[i], speed_dps);
  }
}

/* 双同步带电机同一逻辑位置目标(位置环串级), 保证严格同步 */
static void SetBeltPosition(DartLauncherInstance* inst, float pos_deg) {
  inst->belt_pos_target = pos_deg;
  if (!inst->zero_valid) {
    SetBeltSpeed(inst, 0.0f);  // 零点无效时只能速度环抱死, 禁止位置闭环
    return;
  }
  for (int i = 0; i < 2; i++) {
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

/* 扳机是否到达当前目标位置 */
static bool TriggerPositionReached(DartLauncherInstance* inst) {
  float err = (inst->trigger_boot_angle + inst->trigger_target_deg) - inst->trigger_motor->measure.total_angle;
  return fabsf(err) < DART_TRIGGER_POS_TOL_DEG;
}

/* 扳机位置环输出 */
static void ApplyTriggerPosition(DartLauncherInstance* inst) {
  DJIMotorOuterLoop(inst->trigger_motor, ANGLE_LOOP);
  DJIMotorSetPIDRef(inst->trigger_motor, inst->trigger_boot_angle + inst->trigger_target_deg);
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
  DJIMotorStop(inst->trigger_motor);
}

static void EnableAllMotors(DartLauncherInstance* inst) {
  DJIMotorEnable(inst->yaw_motor);
  for (int i = 0; i < 2; i++) DJIMotorEnable(inst->belt_motor[i]);
  DJIMotorEnable(inst->trigger_motor);
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

static void EnterFault(DartLauncherInstance* inst) {
  inst->state = DART_STATE_FAULT;
  inst->cali_step = CALI_STEP_DRIVE_TO_STOP;
  inst->charge_step = CHARGE_STEP_PREP_TRIGGER;
  inst->fire_step = FIRE_STEP_RELEASE;
  StopAllMotors(inst);
}

/* 开始一次零位校准 */
static void StartCalibration(DartLauncherInstance* inst, uint32_t now) {
  inst->is_calibrated = false;
  inst->zero_valid = false;
  inst->recovery_retract = false;
  inst->cali_step = CALI_STEP_DRIVE_TO_STOP;
  inst->cali_start_ms = now;
  ResetStallDetectors(inst);
  ClearBeltPid(inst);
  EnableAllMotors(inst);  // 故障恢复路径上电机可能处于停机状态
  inst->state = DART_STATE_CALIBRATING;
  LOGINFO("[dart] start zero calibration");
}

/* ==================== 状态处理 ==================== */

static void HandleCalibrating(DartLauncherInstance* inst, uint32_t now) {
  switch (inst->cali_step) {
    case CALI_STEP_DRIVE_TO_STOP:
      SetBeltSpeed(inst, DART_CALI_DIRECTION * DART_CALI_SPEED_DPS);
      for (int i = 0; i < 2; i++) {
        bool stalled = CheckStall(&inst->stall[i], inst->belt_motor[i], DART_CALI_STALL_SPEED_DPS, DART_CALI_STALL_MS,
                                  now) ||
                       ConsumePidBlocked(inst->belt_motor[i]);
        if (stalled) inst->stalled_flag[i] = true;
      }
      if (inst->stalled_flag[0] && inst->stalled_flag[1]) {
        // 双电机各记硬限位处编码器值, 零点即完成同步
        for (int i = 0; i < 2; i++) {
          inst->belt_zero_offset[i] = inst->belt_motor[i]->measure.total_angle;
        }
        inst->zero_valid = true;
        inst->belt_pos_target = DART_BELT_HOME_DEG;
        ClearBeltPid(inst);
        inst->cali_step = CALI_STEP_BACKOFF;
        inst->step_start_ms = now;
        LOGINFO("[dart] zero found, back off to home");
      } else if (now - inst->cali_start_ms > DART_CALI_TIMEOUT_MS) {
        LOGERROR("[dart] fault: calibration stall timeout");
        EnterFault(inst);
      }
      break;

    case CALI_STEP_BACKOFF:
      SetBeltPosition(inst, DART_BELT_HOME_DEG);
      if (BeltPositionReached(inst, DART_BELT_HOME_DEG)) {
        inst->is_calibrated = true;
        inst->state = DART_STATE_IDLE;
        LOGINFO("[dart] calibration done");
      } else if (now - inst->step_start_ms > DART_CALI_TIMEOUT_MS) {
        LOGERROR("[dart] fault: calibration backoff timeout");
        EnterFault(inst);
      }
      break;

    default:
      inst->cali_step = CALI_STEP_DRIVE_TO_STOP;
      break;
  }
}

static void HandleIdle(DartLauncherInstance* inst, uint32_t now) {
  (void)now;
  inst->trigger_target_deg = inst->trigger_catch_deg;
  SetBeltPosition(inst, DART_BELT_HOME_DEG);
  if (inst->recovery_retract) {
    if (BeltPositionReached(inst, DART_BELT_HOME_DEG)) {
      inst->recovery_retract = false;
      LOGINFO("[dart] recovery retract done");
    }
  }
}

static void HandleCharging(DartLauncherInstance* inst, uint32_t now) {
  switch (inst->charge_step) {
    case CHARGE_STEP_PREP_TRIGGER:
      // 拉伸量由扳机卡位决定, 必须先就位再拉拽
      inst->trigger_target_deg = inst->trigger_catch_deg;
      SetBeltPosition(inst, DART_BELT_HOME_DEG);
      if (TriggerPositionReached(inst)) {
        ResetStallDetectors(inst);
        inst->charge_step = CHARGE_STEP_DRIVE;
        inst->step_start_ms = now;
      } else if (now - inst->step_start_ms > DART_TRIGGER_SETTLE_TIMEOUT_MS) {
        LOGERROR("[dart] fault: trigger not in position before charge");
        EnterFault(inst);
      }
      break;

    case CHARGE_STEP_DRIVE: {
      SetBeltPosition(inst, DART_BELT_CHARGE_DEG);
      bool done = BeltPositionReached(inst, DART_BELT_CHARGE_DEG);
      if (!done && DART_CHARGE_COMPLETE_ON_STALL) {
        done = true;
        for (int i = 0; i < 2; i++) {
          bool stalled = CheckStall(&inst->stall[i], inst->belt_motor[i], DART_CHARGE_STALL_SPEED_DPS,
                                    DART_CHARGE_STALL_MS, now) ||
                         ConsumePidBlocked(inst->belt_motor[i]);
          if (stalled) inst->stalled_flag[i] = true;
          if (!inst->stalled_flag[i]) done = false;
        }
      }
      if (done) {
        inst->charge_step = CHARGE_STEP_HOLD;
        inst->step_start_ms = now;
      } else if (now - inst->step_start_ms > DART_CHARGE_TIMEOUT_MS) {
        LOGERROR("[dart] fault: charge timeout");
        EnterFault(inst);
      }
      break;
    }

    case CHARGE_STEP_HOLD:
      SetBeltPosition(inst, DART_BELT_CHARGE_DEG);
      if (now - inst->step_start_ms >= DART_CHARGE_HOLD_MS) {
        inst->charge_step = CHARGE_STEP_RETRACT;
        inst->step_start_ms = now;
      }
      break;

    case CHARGE_STEP_RETRACT:
      // 挡块脱离发射平台活动范围, 避免阻挡发射体
      SetBeltPosition(inst, DART_BELT_HOME_DEG);
      if (BeltPositionReached(inst, DART_BELT_HOME_DEG)) {
        inst->state = DART_STATE_READY;
        LOGINFO("[dart] charged, ready to fire");
      } else if (now - inst->step_start_ms > DART_RETRACT_TIMEOUT_MS) {
        LOGERROR("[dart] fault: retract timeout");
        EnterFault(inst);
      }
      break;

    default:
      inst->charge_step = CHARGE_STEP_PREP_TRIGGER;
      break;
  }
}

static void HandleReady(DartLauncherInstance* inst, uint32_t now) {
  (void)now;
  inst->trigger_target_deg = inst->trigger_catch_deg;
  SetBeltPosition(inst, DART_BELT_HOME_DEG);
}

static void HandleFiring(DartLauncherInstance* inst, uint32_t now) {
  switch (inst->fire_step) {
    case FIRE_STEP_RELEASE:
      inst->trigger_target_deg = DART_TRIGGER_RELEASE_DEG;
      if (TriggerPositionReached(inst)) {
        inst->fire_step = FIRE_STEP_DWELL;
        inst->step_start_ms = now;
      } else if (now - inst->step_start_ms > DART_FIRE_TIMEOUT_MS) {
        LOGERROR("[dart] fault: trigger release timeout");
        EnterFault(inst);
      }
      break;

    case FIRE_STEP_DWELL:
      if (now - inst->step_start_ms >= DART_FIRE_DWELL_MS) {
        inst->fire_step = FIRE_STEP_RESET;
        inst->step_start_ms = now;
      }
      break;

    case FIRE_STEP_RESET:
      inst->trigger_target_deg = inst->trigger_catch_deg;
      if (TriggerPositionReached(inst)) {
        inst->fire_step = FIRE_STEP_RELEASE;
        inst->state = DART_STATE_IDLE;
        LOGINFO("[dart] fire sequence done, back to idle");
      } else if (now - inst->step_start_ms > DART_FIRE_TIMEOUT_MS) {
        LOGERROR("[dart] fault: trigger reset timeout");
        EnterFault(inst);
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
      }
      break;

    case DART_CMD_CHARGE:
      if (inst->state == DART_STATE_IDLE && inst->is_calibrated && !inst->recovery_retract) {
        inst->charge_step = CHARGE_STEP_PREP_TRIGGER;
        inst->step_start_ms = now;
        inst->state = DART_STATE_CHARGING;
        LOGINFO("[dart] charge sequence start");
      } else {
        LOGWARNING("[dart] charge command ignored");
      }
      break;

    case DART_CMD_FIRE:
      if (inst->state == DART_STATE_READY) {
        inst->fire_step = FIRE_STEP_RELEASE;
        inst->step_start_ms = now;
        inst->state = DART_STATE_FIRING;
        LOGINFO("[dart] fire sequence start");
      } else {
        LOGWARNING("[dart] fire command ignored");
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
  if (!inst->zero_valid) return;
  float pos0 = inst->belt_motor[0]->measure.total_angle - inst->belt_zero_offset[0];
  float pos1 = inst->belt_motor[1]->measure.total_angle - inst->belt_zero_offset[1];
  if (fabsf(pos0 - pos1) > DART_BELT_SYNC_WARN_DEG && now - inst->sync_warn_ms > DART_SYNC_WARN_PERIOD_MS) {
    inst->sync_warn_ms = now;
    LOGERROR("[dart] belt motors sync deviation exceeded");
  }
}

/* 调试点动: 双带电机同速 + 扳机点动 */
static void ApplyDebugJog(DartLauncherInstance* inst) {
  SetBeltSpeed(inst, inst->debug_belt_speed_dps);
  DJIMotorOuterLoop(inst->trigger_motor, SPEED_LOOP);
  DJIMotorSetPIDRef(inst->trigger_motor, inst->debug_trigger_speed_dps);
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

  Motor_Init_Config_s trigger_conf = DART_TRIGGER_MOTOR_CONFIG(DART_TRIGGER_CAN, DART_TRIGGER_CAN_ID, DART_TRIGGER_REVERSE);
  inst->trigger_motor = DJIMotorInit(&trigger_conf);

  inst->yaw_boot_angle = inst->yaw_motor->measure.total_angle;
  inst->trigger_boot_angle = inst->trigger_motor->measure.total_angle;
  inst->yaw_angle_target = inst->yaw_boot_angle;
  inst->trigger_catch_deg = DART_TRIGGER_CATCH_DEFAULT_DEG;
  inst->trigger_target_deg = DART_TRIGGER_CATCH_DEFAULT_DEG;
  inst->belt_pos_target = DART_BELT_HOME_DEG;
  inst->state = DART_STATE_IDLE;
  inst->last_ms = (uint32_t)DWT_GetTimeline_ms();
  return inst;
}

void DartLauncherSetEnable(DartLauncherInstance* inst, bool enable) {
  if (inst == NULL) return;
  uint32_t now = (uint32_t)DWT_GetTimeline_ms();

  if (enable && !inst->enabled) {
    inst->enabled = true;
    EnableAllMotors(inst);
    // 失能时被中止的序列统一回 IDLE 兜底
    if (inst->state == DART_STATE_CALIBRATING || inst->state == DART_STATE_CHARGING ||
        inst->state == DART_STATE_FIRING) {
      if (inst->state == DART_STATE_CHARGING && inst->is_calibrated) inst->recovery_retract = true;
      inst->state = DART_STATE_IDLE;
    }
    // 上电后开始工作时必须先完成一次零位校准
    if (!inst->is_calibrated && inst->state == DART_STATE_IDLE && !inst->recovery_retract) {
      StartCalibration(inst, now);
    }
  } else if (!enable && inst->enabled) {
    switch (inst->state) {
      case DART_STATE_CALIBRATING:
        inst->state = DART_STATE_IDLE;
        inst->is_calibrated = false;
        inst->zero_valid = false;
        break;
      case DART_STATE_CHARGING:
        inst->state = DART_STATE_IDLE;
        inst->charge_step = CHARGE_STEP_PREP_TRIGGER;
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
  }
}

void DartLauncherSetCommand(DartLauncherInstance* inst, Dart_Cmd_e cmd) {
  if (inst == NULL) return;
  inst->pending_cmd = cmd;
}

void DartLauncherAdjustTrigger(DartLauncherInstance* inst, float delta_deg) {
  if (inst == NULL) return;
  if (inst->state != DART_STATE_IDLE && inst->state != DART_STATE_READY) return;
  inst->trigger_catch_deg += delta_deg;
  VAL_LIMIT(inst->trigger_catch_deg, DART_TRIGGER_MIN_DEG, DART_TRIGGER_MAX_DEG);
}

void DartLauncherSetYawRate(DartLauncherInstance* inst, float rate_dps) {
  if (inst == NULL) return;
  inst->yaw_rate_cmd_dps = rate_dps;
}

void DartLauncherSetDebugJog(DartLauncherInstance* inst, bool enable, float belt_speed_dps, float trigger_speed_dps) {
  if (inst == NULL) return;
  inst->debug_jog = enable;
  inst->debug_belt_speed_dps = belt_speed_dps;
  inst->debug_trigger_speed_dps = trigger_speed_dps;
}

void DartLauncherTask(DartLauncherInstance* inst) {
  if (inst == NULL || !inst->enabled) return;

  uint32_t now = (uint32_t)DWT_GetTimeline_ms();
  float dt_s = (float)(now - inst->last_ms) * 0.001f;
  if (dt_s <= 0.0f || dt_s > 0.05f) dt_s = DART_TASK_DT_S;
  inst->last_ms = now;

  // 校准完成前严格限制同步带电机力矩(所有模式生效)
  ApplyBeltTorqueLimit(inst);

  HandleCommand(inst, now);

  if (inst->state == DART_STATE_FAULT) {
    StopAllMotors(inst);
    return;
  }

  if (inst->debug_jog && inst->state == DART_STATE_IDLE) {
    ApplyDebugJog(inst);
  } else {
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
    ApplyTriggerPosition(inst);
  }

  ApplyYaw(inst, dt_s);
  MonitorBeltSync(inst, now);
}
