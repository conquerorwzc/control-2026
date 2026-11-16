/**
 * @file shoot.c
 * @brief dart 发射机构(shoot_dart)实现: 零位校准 / 储能 / 发射(舵机开环) / 射力调节
 */
#include "shoot.h"

#include "dart_buzzer.h"

#include <math.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "robot_config.h"
#include "user_lib.h"

/* ==================== 工具函数 ==================== */

/* 舵机角度 -> 占空比: 角度线性映射到脉宽, 脉宽/周期即占空比 */
static float ServoAngleToDuty(float angle_deg) {
  float span = DART_SERVO_ANGLE_MAX_DEG - DART_SERVO_ANGLE_MIN_DEG;
  float pulse_us = DART_SERVO_PULSE_MIN_US +
                   (angle_deg - DART_SERVO_ANGLE_MIN_DEG) * (DART_SERVO_PULSE_MAX_US - DART_SERVO_PULSE_MIN_US) / span;
  return (pulse_us * 1e-6f) / DART_SERVO_PWM_PERIOD_S;
}

/* 设定舵机角度(开环, 限幅到机械范围) */
static void SetServoAngle(ShootDartInstance* inst, float angle_deg) {
  VAL_LIMIT(angle_deg, DART_SERVO_ANGLE_MIN_DEG, DART_SERVO_ANGLE_MAX_DEG);
  inst->servo.angle_deg = angle_deg;
  PWMSetDutyRatio(inst->servo.pwm, ServoAngleToDuty(angle_deg));
}

/* 堵转判据: 实测角速度持续低于阈值(deg/s)达 stall_ms 即判堵转
 * @param speed_thr 速度阈值, @param stall_ms 持续时间; 校准与储能"拉到底"用不同参数
 * (校准要慢判防误判, 储能拉到底后要尽快确认以缩短等待) */
static bool CheckStall(StallDetector_s* det, DJIMotorInstance* motor, uint32_t now, float speed_thr,
                       uint32_t stall_ms) {
  if (fabsf(motor->measure.speed_aps) > speed_thr) {
    det->tracking = false;
    return false;
  }
  if (!det->tracking) {
    det->tracking = true;
    det->start_ms = now;
    return false;
  }
  return (now - det->start_ms) >= stall_ms;
}

/* 清同步带控制状态(校准/使能/零点重建后调用): 清 PID 与差模积分/偏置, 避免残留造成初始偏置 */
static void ClearBeltControl(ShootDartInstance* inst) {
  for (int i = 0; i < 2; i++) {
    PIDClear(&inst->belt.motor[i]->motor_controller.speed_PID);
    PIDClear(&inst->belt.motor[i]->motor_controller.angle_PID);
  }
  PIDClear(&inst->belt.common_pid);
  inst->belt.sync_i = 0.0f;
  inst->belt.sync_u = 0.0f;
  inst->belt.add_current[0] = 0.0f;
  inst->belt.add_current[1] = 0.0f;
  inst->belt.pos_diff = inst->belt.motor[0]->measure.total_angle - inst->belt.motor[1]->measure.total_angle;
  inst->belt.ff_dps = 0.0f;
  inst->belt.v_ff_dps = 0.0f;
  inst->belt.v_common_dps = 0.0f;
}

/* 进入新阶段/新步骤前清掉堵转记录 */
static void ResetStallState(ShootDartInstance* inst) {
  for (int i = 0; i < 2; i++) {
    inst->belt.stalled[i] = false;
    inst->belt.stall[i].tracking = false;
  }
}

/* 同步带位置移动的类型: 决定共模速度前馈用哪种规划 */
typedef enum {
  BELT_MOVE_NORMAL = 0, // 普通移动(故障保持等): 正常梯形规划
  BELT_MOVE_HOME,       // 回撤到释放位: 靠近行程极限, 末段提前减速 + 低速爬行
  BELT_MOVE_CHARGE,     // 储能拉拽: 经过"推块咬合区"时限速, 避免高速撞击
} Dart_Belt_Move_e;

/* 前馈目标速度 -> 按加速度限幅平滑输出(梯形规划的"斜坡"环节)
 * 规划状态跨周期保留: 起步不阶跃、换目标/换向时自然经过 0 过渡。 */
static float BeltRampSpeed(ShootDartInstance* inst, float v_target) {
  float dv = DART_BELT_TRACK_ACC_DPS2 * inst->task_dt_s;
  float v = inst->belt.ff_dps;
  if (v_target > v + dv) {
    v += dv;
  } else if (v_target < v - dv) {
    v -= dv;
  } else {
    v = v_target;
  }
  inst->belt.ff_dps = v;
  return v;
}

/* 正常自动动作的速度前馈(梯形): 剩余距离大 -> 巡航限速; 接近目标 -> sqrt 减速曲线;
 * 再按加速度限幅平滑成斜坡。
 * @note 减速曲线近端线性化: v = sqrt(2a|e| + v0²) - v0。纯 sqrt 在 e->0 处斜率无穷,
 *       离散下每周期 v*dt > e 会在 0 附近抖(~1 编码器计数); 线性化后近端速度 ∝ 误差, 静止保持不颤。 */
static float BeltTrackFeedforward(ShootDartInstance* inst, float pos_err, float v_lim) {
  float a = DART_BELT_TRACK_ACC_DPS2;
  float v_brake = sqrtf(2.0f * a * fabsf(pos_err) + DART_BELT_TRACK_FF_V0_DPS * DART_BELT_TRACK_FF_V0_DPS) -
                  DART_BELT_TRACK_FF_V0_DPS;
  if (v_brake > v_lim) v_brake = v_lim;
  return BeltRampSpeed(inst, (pos_err >= 0.0f) ? v_brake : -v_brake);
}

/* 回撤到释放位的速度前馈(专用保守规划):
 * 释放位离行程硬限位只有 DART_BELT_CALI_BACKOFF_DEG(≈4.6mm), 从储能端高速回撤时若按正常
 * 减速曲线走, 机构/控制链滞后会让滑台冲过释放位撞到同步带轮或硬限位, 所以末段单独规划:
 *   ① 距目标 DART_BELT_HOME_APPROACH_DIST_DEG 内, 巡航限速降到 DART_BELT_HOME_APPROACH_SPEED_DPS;
 *   ② 减速曲线用更保守的 DART_BELT_HOME_APPROACH_DECEL_DPS2(比正常 TRACK_ACC 小 -> 提前刹车);
 *   ③ 最后 DART_BELT_HOME_CREEP_DIST_DEG 内以 DART_BELT_HOME_CREEP_SPEED_DPS 低速爬行到位,
 *      到点时动量足够小(滞后造成的冲过量只有几度, 远小于释放位到限位的 300° 余量)。
 * 三者叠加后再经 BeltRampSpeed 加速度限幅平滑, 不会出现速度阶跃。
 * @note 只用于"从储能端朝释放位方向"的运动; 从硬限位往外退(校准回撤)方向安全, 走正常曲线。 */
static float BeltHomeFeedforward(ShootDartInstance* inst, float pos_err, float v_lim) {
  float dist = fabsf(pos_err);
  float v = v_lim;
  if (dist < DART_BELT_HOME_APPROACH_DIST_DEG) {
    float v_curve;
    if (dist <= DART_BELT_HOME_CREEP_DIST_DEG) {
      // 最后一段爬行: 速度随剩余距离线性收敛(到点速度为 0), 避免在释放位附近来回过冲
      v_curve = DART_BELT_HOME_CREEP_SPEED_DPS * dist / DART_BELT_HOME_CREEP_TAPER_DEG;
      if (v_curve > DART_BELT_HOME_CREEP_SPEED_DPS) v_curve = DART_BELT_HOME_CREEP_SPEED_DPS;
    } else {
      // 减速曲线: 比正常 TRACK_ACC 保守(提前刹车), 并限到末段巡航速度
      v_curve = sqrtf(2.0f * DART_BELT_HOME_APPROACH_DECEL_DPS2 * (dist - DART_BELT_HOME_CREEP_DIST_DEG) +
                      DART_BELT_HOME_CREEP_SPEED_DPS * DART_BELT_HOME_CREEP_SPEED_DPS);
      if (v_curve > DART_BELT_HOME_APPROACH_SPEED_DPS) v_curve = DART_BELT_HOME_APPROACH_SPEED_DPS;
    }
    if (v_curve < v) v = v_curve;
  }
  return BeltRampSpeed(inst, (pos_err >= 0.0f) ? v : -v);
}

/* 同步带共模/差模控制(每周期唯一出口)
 *   差模: 两侧位置差 θ0−θ1 ∝ 两侧皮带张力差(见 robot_config.h), 低通+死区后 PD+I,
 *         输出电流偏置 u -> 机0 前馈 −u / 机1 前馈 +u(叠加在速度环输出之后, 不写速度环积分,
 *         因此不会被积分限幅单边裁掉); 阻尼取实测相对速度, 不对位置求导。
 *   共模: v_common = 速度前馈 v_ff + 位置修正 P(只补小误差), 两台同一速度给定 -> 出力均担。
 *   均流: 两台速度环积分以 DART_BELT_INTEGRAL_BLEND 向均值互融, 摩擦差异不被积分固化成力矩差。
 * @param v_ff          本周期速度前馈(°/s)
 * @param v_total_limit 速度给定总限幅(°/s), 前馈叠加后也不许超过 */
static void BeltSyncUpdate(ShootDartInstance* inst, float v_ff, float v_total_limit) {
  DJIMotorInstance* belt_l = inst->belt.motor[0];
  DJIMotorInstance* belt_r = inst->belt.motor[1];
  float p0 = belt_l->measure.total_angle;
  float p1 = belt_r->measure.total_angle;
  float dt = inst->task_dt_s;

  // 1) 速度环积分互融(均流): Iout 向两机均值靠拢 -> 稳态出力均分
  PIDInstance* sp0 = &belt_l->motor_controller.speed_PID;
  PIDInstance* sp1 = &belt_r->motor_controller.speed_PID;
  float shared = 0.5f * (sp0->Iout + sp1->Iout);
  sp0->Iout += DART_BELT_INTEGRAL_BLEND * (shared - sp0->Iout);
  sp1->Iout += DART_BELT_INTEGRAL_BLEND * (shared - sp1->Iout);

  // 2) 差模(张力差)环: 目标是保持位置差在标定基准上, 而不是让它 -> 0
  inst->belt.pos_diff += (p0 - p1 - inst->belt.pos_diff) * (dt / (DART_BELT_SYNC_DIFF_LPF_TAU_S + dt));
  float err = inst->belt.pos_diff - inst->belt.diff_ref;
  if (fabsf(err) < DART_BELT_SYNC_DEADBAND_DEG) err = 0.0f;  // 跨过减速箱回差/皮带齿隙造成的假误差
  float rel_speed = belt_l->measure.speed_aps - belt_r->measure.speed_aps;
  inst->belt.sync_i += DART_BELT_SYNC_KI * err * dt;
  VAL_LIMIT(inst->belt.sync_i, -DART_BELT_SYNC_KI_LIMIT, DART_BELT_SYNC_KI_LIMIT);
  float u = DART_BELT_SYNC_KP * err + DART_BELT_SYNC_KD_VEL * rel_speed + inst->belt.sync_i;
  VAL_LIMIT(u, -DART_BELT_SYNC_TORQUE_MAX, DART_BELT_SYNC_TORQUE_MAX);
  inst->belt.sync_u = u;
  inst->belt.add_current[0] = -u;  // u>0 = 机0 超前/更快 -> 减机0 电流、加机1 电流
  inst->belt.add_current[1] = u;

  // 3) 共模: 速度前馈 + 位置修正, 两台同一速度给定
  float v_common = PIDCalculate(&inst->belt.common_pid, 0.5f * (p0 + p1), inst->belt.common_ref) + v_ff;
  VAL_LIMIT(v_common, -DART_BELT_MAX_SPEED_DPS, DART_BELT_MAX_SPEED_DPS);
  VAL_LIMIT(v_common, -v_total_limit, v_total_limit);
  inst->belt.v_ff_dps = v_ff;
  inst->belt.v_common_dps = v_common;
  for (int i = 0; i < 2; i++) {
    DJIMotorOuterLoop(inst->belt.motor[i], SPEED_LOOP);
    DJIMotorSetPIDRef(inst->belt.motor[i], v_common);
    // 电流前馈叠加后仍对称钳在工作限幅(校准前=低限幅): 偏置只用剩余电流裕量
    float lim = inst->belt.motor[i]->motor_controller.speed_PID.MaxOut;
    VAL_LIMIT(inst->belt.motor[i]->motor_controller.final_output, -lim, lim);
  }
}

/* 双带纯速度控制(校准顶限位/零点无效): 清同步环状态后两机同速 */
static void SetBeltSpeed(ShootDartInstance* inst, float speed_dps) {
  inst->belt.sync_i = 0.0f;
  inst->belt.sync_u = 0.0f;
  inst->belt.add_current[0] = 0.0f;
  inst->belt.add_current[1] = 0.0f;
  inst->belt.ff_dps = 0.0f;
  inst->belt.v_ff_dps = 0.0f;
  inst->belt.v_common_dps = 0.0f;
  for (int i = 0; i < 2; i++) {
    DJIMotorOuterLoop(inst->belt.motor[i], SPEED_LOOP);
    DJIMotorSetPIDRef(inst->belt.motor[i], speed_dps);
  }
}

/* 储能拉拽"推块咬合区"的位置限速(返回该位置允许的最大速度, 同时用于前馈与总给定钳制)
 * 推块在 DART_BELT_CHARGE_ENGAGE_POS_DEG(当前 8000°)附近咬住滑台: 从释放位高速冲过来时,
 * 咬合瞬间接近撞击, 有损坏风险 -> 该区间(±HALF_DEG)限速到 ENGAGE_SPEED;
 * 并且提前 MARGIN_DEG 就开始按减速度曲线减速, 补偿控制链滞后, 保证到区间口时实际速度已降下来
 * (不能等进了区间才刹, 那样已经撞上了)。咬合完成(过了区间)后恢复正常储能速度。
 * @note 只作用于沿储能方向(正方向)运动; 回撤反向经过该区间不受影响(此时推块已脱离滑台)。 */
static float BeltChargeSpeedLimit(ShootDartInstance* inst, float v_lim) {
  float pos = 0.5f * (inst->belt.pos_rel[0] + inst->belt.pos_rel[1]);  // 两侧相对零点的平均位置
  float brake_start = DART_BELT_CHARGE_ENGAGE_POS_DEG - DART_BELT_CHARGE_ENGAGE_HALF_DEG -
                      DART_BELT_CHARGE_ENGAGE_MARGIN_DEG;
  float zone_hi = DART_BELT_CHARGE_ENGAGE_POS_DEG + DART_BELT_CHARGE_ENGAGE_HALF_DEG;
  float cap;
  if (pos < brake_start) {
    // 区间之前: 按"能在 brake_start 处降到 ENGAGE_SPEED"的减速曲线提前限速
    cap = sqrtf(DART_BELT_CHARGE_ENGAGE_SPEED_DPS * DART_BELT_CHARGE_ENGAGE_SPEED_DPS +
                2.0f * DART_BELT_TRACK_ACC_DPS2 * (brake_start - pos));
  } else if (pos <= zone_hi) {
    cap = DART_BELT_CHARGE_ENGAGE_SPEED_DPS;  // 咬合区: 限速
  } else {
    cap = v_lim;  // 咬合完成: 恢复正常储能速度
  }
  return cap < v_lim ? cap : v_lim;
}

/* 同步带位置目标(逻辑坐标): 差模基准=两零点之差, 共模=平均零点+目标; 速度前馈按梯形规划
 * @param move 移动类型: HOME=回撤到释放位(末段提前减速+爬行, 靠近行程极限),
 *             CHARGE=储能拉拽(经过推块咬合区时限速), NORMAL=普通梯形规划 */
static void SetBeltPosition(ShootDartInstance* inst, float pos_deg, float speed_limit_dps, Dart_Belt_Move_e move) {
  if (!inst->belt.zero_valid) {
    SetBeltSpeed(inst, 0.0f);  // 零点无效: 只能速度环抱死, 禁止位置闭环
    return;
  }
  float limit = speed_limit_dps < DART_BELT_MAX_SPEED_DPS ? speed_limit_dps : DART_BELT_MAX_SPEED_DPS;
  inst->belt.common_ref = 0.5f * (inst->belt.zero_offset[0] + inst->belt.zero_offset[1]) + pos_deg;
  inst->belt.diff_ref = inst->belt.zero_offset[0] - inst->belt.zero_offset[1];
  inst->belt.common_pid.MaxOut = DART_BELT_TRACK_P_CORR_DPS;  // P 只做小误差修正, 稳态速度由前馈给
  float pos_err = inst->belt.common_ref -
                  0.5f * (inst->belt.motor[0]->measure.total_angle + inst->belt.motor[1]->measure.total_angle);
  // 储能拉拽沿正方向经过"推块咬合区"时, 本段速度上限由位置决定(提前减速 + 区间限速)
  float v_limit = limit;
  if (move == BELT_MOVE_CHARGE && pos_err > 0.0f) v_limit = BeltChargeSpeedLimit(inst, limit);
  float v_ff;
  if (move == BELT_MOVE_HOME && pos_err < 0.0f) {
    v_ff = BeltHomeFeedforward(inst, pos_err, limit);  // 朝释放位方向(也是朝硬限位方向): 提前减速
  } else {
    v_ff = BeltTrackFeedforward(inst, pos_err, v_limit);
  }
  BeltSyncUpdate(inst, v_ff, v_limit);
}

/* 是否到达逻辑目标: 以两侧平均位置判(挡块位置); 单侧只留宽松健全性上限 END_TOL
 * @warning 不能按单侧各自容差判: 差模有意放行两侧变形差, 单侧误差可上千度, 按单侧判会永远"没到位" */
static bool BeltPositionReached(ShootDartInstance* inst, float pos_deg, float tol_deg) {
  if (!inst->belt.zero_valid) return false;
  float e0 = (inst->belt.zero_offset[0] + pos_deg) - inst->belt.motor[0]->measure.total_angle;
  float e1 = (inst->belt.zero_offset[1] + pos_deg) - inst->belt.motor[1]->measure.total_angle;
  if (fabsf(0.5f * (e0 + e1)) > tol_deg) return false;
  if (fabsf(e0) > DART_BELT_END_TOL_DEG || fabsf(e1) > DART_BELT_END_TOL_DEG) return false;
  return true;
}

/* 扳机丝杆位置环输出: 目标 = 射力位置(相对开机位置) */
static void ApplyScrewPosition(ShootDartInstance* inst) {
  DJIMotorOuterLoop(inst->screw.motor, ANGLE_LOOP);
  DJIMotorSetPIDRef(inst->screw.motor, inst->screw.pos_deg);
}

/* 故障: 保持当前位置(不断力矩)
 * @warning 弹簧负载下断力矩 = 机构被拉簧带飞(失控加速 + 反拖发电过压), 只有失能才真正断输出 */
static void EnterFault(ShootDartInstance* inst) {
  inst->state = DART_STATE_FAULT;
  inst->cali.step = CALI_STEP_BELT_DRIVE_TO_STOP;
  inst->charge.step = CHARGE_STEP_PREP_SCREW;
  inst->fire.step = FIRE_STEP_RELEASE;
  SetServoAngle(inst, DART_SERVO_CATCH_DEG);  // 回卡位角保持扣住
  if (inst->belt.zero_valid) {
    inst->belt.hold_pos = 0.5f * ((inst->belt.motor[0]->measure.total_angle - inst->belt.zero_offset[0]) +
                                   (inst->belt.motor[1]->measure.total_angle - inst->belt.zero_offset[1]));
  } else {
    inst->belt.hold_pos = 0.0f;
  }
  inst->belt.ff_dps = 0.0f;  // 清运动前馈, 否则故障后还会按 a*dt 衰减着"滑一段"
  DartBuzzerFault();
}

/* 开始一次零位校准(只做同步带) */
static void StartCalibration(ShootDartInstance* inst, uint32_t now) {
  inst->is_calibrated = false;
  inst->belt.zero_valid = false;
  inst->recovery_retract = false;
  inst->cali.step = CALI_STEP_BELT_DRIVE_TO_STOP;
  inst->cali.start_ms = now;
  ResetStallState(inst);
  ClearBeltControl(inst);
  PIDClear(&inst->screw.motor->motor_controller.speed_PID);
  PIDClear(&inst->screw.motor->motor_controller.angle_PID);
  for (int i = 0; i < 2; i++) DJIMotorEnable(inst->belt.motor[i]);  // 故障恢复路径上可能处于停机
  DJIMotorEnable(inst->screw.motor);
  inst->state = DART_STATE_CALIBRATING;
  DartBuzzerFaultCleared();
  DartBuzzerCaliStart();
  DartBuzzerCaliBgmStart();
  LOGINFO("[dart] start zero calibration (belt only)");
}

/* ==================== 状态处理 ==================== */

static void HandleCalibrating(ShootDartInstance* inst, uint32_t now) {
  switch (inst->cali.step) {
    case CALI_STEP_BELT_DRIVE_TO_STOP:
      ApplyScrewPosition(inst);  // 丝杆保持不动
      SetBeltSpeed(inst, DART_BELT_CALI_DIRECTION * DART_BELT_CALI_SPEED_DPS);
      if (now - inst->step_start_ms > DART_CALI_START_GRACE_MS) {  // 起步宽限, 防速度没上来就误判
        for (int i = 0; i < 2; i++) {
          if (CheckStall(&inst->belt.stall[i], inst->belt.motor[i], now, DART_CALI_STALL_SPEED_DPS,
                        DART_CALI_STALL_MS))
            inst->belt.stalled[i] = true;
        }
      }
      if (inst->belt.stalled[0] && inst->belt.stalled[1]) {
        for (int i = 0; i < 2; i++) inst->belt.zero_offset[i] = inst->belt.motor[i]->measure.total_angle;
        inst->belt.zero_valid = true;  // 两侧零点同时建立(=两侧零点同步)
        ClearBeltControl(inst);
        inst->cali.step = CALI_STEP_BELT_BACKOFF;
        inst->step_start_ms = now;
        inst->cali.log_ms = now;
        LOGINFO("[dart] belt zero found (L %d, R %d, diff %d), back off %d deg at %d deg/s",
                (int)inst->belt.zero_offset[0], (int)inst->belt.zero_offset[1],
                (int)(inst->belt.zero_offset[0] - inst->belt.zero_offset[1]), (int)DART_BELT_CALI_BACKOFF_DEG,
                (int)DART_BELT_CALI_BACKOFF_SPEED_DPS);
      } else if (now - inst->cali.start_ms > DART_BELT_CALI_TIMEOUT_MS) {
        LOGERROR("[dart] fault: belt calibration stall timeout");
        EnterFault(inst);
      }
      break;

    case CALI_STEP_BELT_BACKOFF: {
      ApplyScrewPosition(inst);
      SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_CALI_BACKOFF_SPEED_DPS, BELT_MOVE_HOME);
      // 超时按"回撤距离/回撤限速 + 4s 余量"算(不能用顶限位步的超时, 否则必误报)
      uint32_t budget_ms = (uint32_t)(DART_BELT_CALI_BACKOFF_DEG / DART_BELT_CALI_BACKOFF_SPEED_DPS * 1000.0f) + 4000u;
      if (BeltPositionReached(inst, DART_BELT_HOME_DEG, DART_BELT_POS_TOL_DEG)) {
        ResetStallState(inst);
        PIDClear(&inst->screw.motor->motor_controller.speed_PID);
        PIDClear(&inst->screw.motor->motor_controller.angle_PID);
        inst->is_calibrated = true;
        inst->state = DART_STATE_IDLE;
        inst->cali.step = CALI_STEP_BELT_DRIVE_TO_STOP;
        DartBuzzerCaliDone();
        LOGINFO("[dart] calibration done (belt only): is_calibrated=1, zero_valid=%d",
                inst->belt.zero_valid ? 1 : 0);
      } else {
        if (now - inst->cali.log_ms >= 1000u) {  // 1Hz 打印回撤进度
          inst->cali.log_ms = now;
          LOGINFO("[dart] backoff %d/%d ms, L err %d, R err %d, tol %d", (int)(now - inst->step_start_ms),
                  (int)budget_ms,
                  (int)((inst->belt.zero_offset[0] + DART_BELT_HOME_DEG) - inst->belt.motor[0]->measure.total_angle),
                  (int)((inst->belt.zero_offset[1] + DART_BELT_HOME_DEG) - inst->belt.motor[1]->measure.total_angle),
                  (int)DART_BELT_POS_TOL_DEG);
        }
        if (now - inst->step_start_ms > budget_ms) {
          LOGERROR("[dart] fault: belt calibration backoff timeout");
          EnterFault(inst);
        }
      }
      break;
    }

    default:
      inst->cali.step = CALI_STEP_BELT_DRIVE_TO_STOP;
      break;
  }
}

static void HandleCharging(ShootDartInstance* inst, uint32_t now) {
  switch (inst->charge.step) {
    case CHARGE_STEP_PREP_SCREW: {
      // 拉伸量由丝杆位置(射力)决定, 必须先就位再拉拽
      SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_HOME_SPEED_DPS, BELT_MOVE_HOME);
      float screw_err = inst->screw.pos_deg - inst->screw.motor->measure.total_angle;
      if (fabsf(screw_err) < DART_SCREW_POS_TOL_DEG) {
        ResetStallState(inst);
        inst->charge.step = CHARGE_STEP_DRIVE;
        inst->step_start_ms = now;
      } else if (now - inst->step_start_ms > DART_SCREW_SETTLE_TIMEOUT_MS) {
        LOGERROR("[dart] fault: screw not in position before charge (tgt %d, cur %d, tol %d)",
                 (int)inst->screw.pos_deg, (int)inst->screw.motor->measure.total_angle, (int)DART_SCREW_POS_TOL_DEG);
        EnterFault(inst);
      }
      break;
    }

    case CHARGE_STEP_DRIVE: {
      // 扳机锁定态是单向通道: 平台会滑过扳机继续被后拉, 储能不会堵转, 按行程判完成
      SetBeltPosition(inst, DART_BELT_CHARGE_DEG, DART_BELT_CHARGE_SPEED_DPS, BELT_MOVE_CHARGE);
      if (BeltPositionReached(inst, DART_BELT_CHARGE_DEG, DART_BELT_CHARGE_TOL_DEG)) {
        uint32_t elapsed_ms = now - inst->step_start_ms;
        inst->charge.step = CHARGE_STEP_RETRACT;
        inst->step_start_ms = now;
        LOGINFO("[dart] charge stroke done in %d ms (moving), reset to release position", (int)elapsed_ms);
      } else {
        // 堵转看门狗: 两侧同时转不动才算顶死; 单侧慢由差模环自动纠偏(不停机不故障)
        bool stalled = false;
        if (now - inst->step_start_ms > DART_CALI_START_GRACE_MS) {
          stalled = true;
          for (int i = 0; i < 2; i++) {
            if (!CheckStall(&inst->belt.stall[i], inst->belt.motor[i], now, DART_CHARGE_STALL_SPEED_DPS,
                            DART_CHARGE_STALL_MS))
              stalled = false;
          }
        }
        if (stalled) {
          // 两侧同时顶死: 离目标近 = 拉到机械末端(正常完成); 离目标远 = 真卡滞/超行程(故障)
          float err0 = fabsf((inst->belt.zero_offset[0] + DART_BELT_CHARGE_DEG) - inst->belt.motor[0]->measure.total_angle);
          float err1 = fabsf((inst->belt.zero_offset[1] + DART_BELT_CHARGE_DEG) - inst->belt.motor[1]->measure.total_angle);
          float err_max = err0 > err1 ? err0 : err1;
          if (err_max < DART_BELT_END_TOL_DEG) {
            uint32_t elapsed_ms = now - inst->step_start_ms;
            ResetStallState(inst);
            inst->charge.step = CHARGE_STEP_RETRACT;
            inst->step_start_ms = now;
            inst->belt.ff_dps = 0.0f;  // 同上: 顶死后立即开始回撤, 不从前馈高值慢慢降
            LOGINFO("[dart] charge stroke done at hard stop (err %d deg) in %d ms, reset to release position",
                    (int)err_max, (int)elapsed_ms);
          } else {
            LOGERROR("[dart] fault: belt stalled during charge far from target (err %d deg, check charge travel)",
                     (int)err_max);
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
      SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_HOME_SPEED_DPS, BELT_MOVE_HOME);
      if (BeltPositionReached(inst, DART_BELT_HOME_DEG, DART_BELT_POS_TOL_DEG)) {
        inst->state = DART_STATE_READY;
        DartBuzzerChargeDone();
        LOGINFO("[dart] charged, ready to fire");
      } else if (now - inst->step_start_ms > DART_RETRACT_TIMEOUT_MS) {
        LOGERROR("[dart] fault: retract timeout");
        EnterFault(inst);
      }
      break;

    default:
      inst->charge.step = CHARGE_STEP_PREP_SCREW;
      break;
  }
}

static void HandleFiring(ShootDartInstance* inst, uint32_t now) {
  // 发射只动舵机, 但同步带必须主动保持在释放位(不调 SetPIDRef 会冻结上一拍电流指令 -> 可能漂移)
  SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_HOME_SPEED_DPS, BELT_MOVE_HOME);
  switch (inst->fire.step) {
    case FIRE_STEP_RELEASE:
      SetServoAngle(inst, DART_SERVO_RELEASE_DEG);
      if (now - inst->step_start_ms >= DART_SERVO_SETTLE_MS) {
        inst->fire.step = FIRE_STEP_DWELL;
        inst->step_start_ms = now;
      }
      break;

    case FIRE_STEP_DWELL:
      if (now - inst->step_start_ms >= DART_FIRE_DWELL_MS) {
        inst->fire.step = FIRE_STEP_RESET;
        inst->step_start_ms = now;
      }
      break;

    case FIRE_STEP_RESET:
      SetServoAngle(inst, DART_SERVO_CATCH_DEG);
      if (now - inst->step_start_ms >= DART_SERVO_SETTLE_MS) {
        inst->fire.step = FIRE_STEP_RELEASE;
        inst->state = DART_STATE_IDLE;
        DartBuzzerFireDone();
        LOGINFO("[dart] fire sequence done, back to idle");
      }
      break;

    default:
      inst->fire.step = FIRE_STEP_RELEASE;
      break;
  }
}

/* 消费一次性命令 */
static void HandleCommand(ShootDartInstance* inst, uint32_t now) {
  Dart_Cmd_e cmd = inst->pending_cmd;
  inst->pending_cmd = DART_CMD_NONE;
  if (cmd == DART_CMD_NONE) return;

  switch (cmd) {
    case DART_CMD_CALIBRATE:
      if (inst->debug.enabled) {
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
      if (inst->debug.enabled) {
        LOGWARNING("[dart] charge command ignored in debug mode (switch to normal mode)");
        DartBuzzerCmdRejected();
      } else if (inst->state == DART_STATE_IDLE && inst->is_calibrated && !inst->recovery_retract) {
        inst->charge.step = CHARGE_STEP_PREP_SCREW;
        inst->step_start_ms = now;
        inst->state = DART_STATE_CHARGING;
        DartBuzzerChargeStart();
        LOGINFO("[dart] charge sequence start (belt target %d deg, screw pos %d deg)", (int)DART_BELT_CHARGE_DEG,
                (int)inst->screw.pos_deg);
      } else if (inst->recovery_retract) {
        LOGWARNING("[dart] charge ignored: recovery retract in progress (block not at release position yet)");
        DartBuzzerCmdRejected();
      } else {
        LOGWARNING("[dart] charge ignored: state=%d (need IDLE=%d), is_calibrated=%d, recovery_retract=%d",
                   (int)inst->state, (int)DART_STATE_IDLE, inst->is_calibrated ? 1 : 0,
                   inst->recovery_retract ? 1 : 0);
        DartBuzzerCmdRejected();
      }
      break;

    case DART_CMD_FIRE:
      if (inst->debug.enabled) {
        LOGWARNING("[dart] fire command ignored in debug mode (switch to normal mode)");
        DartBuzzerCmdRejected();
      } else if (inst->state == DART_STATE_READY) {
        inst->fire.step = FIRE_STEP_RELEASE;
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

/* 同步偏差监测与状态日志(只告警不停机: 偏差由差模环自动纠偏)
 * @note 校准中不检查(回撤时零点刚建立, 两侧起步有先后, 报偏差没有意义) */
static void MonitorBeltSync(ShootDartInstance* inst, uint32_t now) {
  if (!inst->belt.zero_valid || inst->state == DART_STATE_CALIBRATING) return;
  float pos0 = inst->belt.pos_rel[0];
  float pos1 = inst->belt.pos_rel[1];
  if (fabsf(pos0 - pos1) > DART_BELT_SYNC_WARN_DEG && now - inst->belt.sync_warn_ms > DART_SYNC_WARN_PERIOD_MS) {
    inst->belt.sync_warn_ms = now;
    LOGERROR("[dart] belt motors sync deviation exceeded: %d deg (L %d, R %d, thr %d)", (int)(pos0 - pos1),
             (int)pos0, (int)pos1, (int)DART_BELT_SYNC_WARN_DEG);
  }
  // 运动时 1Hz 打同步状态: ff/v=前馈与共模给定, posdiff=扣零点位置差(∝张力差), err/u/si=差模环
  if (fabsf(inst->belt.v_common_dps) > DART_BELT_SYNC_LOG_MIN_SPEED_DPS &&
      now - inst->belt.sync_log_ms >= DART_SYNC_WARN_PERIOD_MS) {
    inst->belt.sync_log_ms = now;
    LOGINFO("[dart] belt sync ff %d v %d posdiff %d err %d u %d si %d cur %d/%d spd %d/%d", (int)inst->belt.v_ff_dps,
            (int)inst->belt.v_common_dps, (int)(pos0 - pos1), (int)(inst->belt.pos_diff - inst->belt.diff_ref),
            (int)inst->belt.sync_u, (int)inst->belt.sync_i, (int)inst->belt.motor[0]->measure.real_current,
            (int)inst->belt.motor[1]->measure.real_current, (int)inst->belt.motor[0]->measure.speed_aps,
            (int)inst->belt.motor[1]->measure.speed_aps);
  }
}

/* 调试点动: 摇杆速率积分出两侧共用的虚拟位置目标, 共模给两侧同一速度给定, 差模自动纠偏
 * @note 反向打杆/松手时目标重同步到实际位置, 避免"先回绕误差"/"松手后飘一段";
 *       目标超前实测超过 MAX_ERR 时暂停积分, 顶到机械限位不跑飞 */
static void ApplyDebugMove(ShootDartInstance* inst, float dt_s, uint32_t now) {
  DJIMotorInstance* belt_l = inst->belt.motor[0];
  DJIMotorInstance* belt_r = inst->belt.motor[1];

  if (!inst->debug.belt_synced) {
    // 进入点动: 目标取两侧当前位置平均, 以当前位置差为张力差基准, 同步环状态就地重置
    inst->debug.belt_target = 0.5f * (belt_l->measure.total_angle + belt_r->measure.total_angle);
    inst->debug.belt_diff_base = belt_l->measure.total_angle - belt_r->measure.total_angle;
    inst->belt.diff_ref = inst->debug.belt_diff_base;
    inst->belt.pos_diff = inst->debug.belt_diff_base;
    inst->belt.sync_i = 0.0f;
    inst->belt.sync_u = 0.0f;
    inst->belt.add_current[0] = 0.0f;
    inst->belt.add_current[1] = 0.0f;
    inst->belt.ff_dps = 0.0f;
    inst->debug.belt_synced = true;
  }
  if (!inst->debug.screw_synced) {
    inst->debug.screw_target = inst->screw.motor->measure.total_angle;
    inst->debug.screw_synced = true;
  }

  // ---- 同步带: 摇杆速率 -> 双侧共用虚拟位置目标 ----
  float actual = 0.5f * (belt_l->measure.total_angle + belt_r->measure.total_angle);
  float rate = inst->debug.belt_speed_dps;
  float follow_err = inst->debug.belt_target - actual;
  if (rate != 0.0f && follow_err != 0.0f && ((rate > 0.0f) != (follow_err > 0.0f))) {
    inst->debug.belt_target = actual;  // 反向: 先重同步目标, 否则要先"回绕"滞后误差
    follow_err = 0.0f;
  }
  if (rate == 0.0f && inst->debug.belt_rate_last != 0.0f) {
    inst->debug.belt_target = actual;  // 松手: 收回目标领跑量, 立即停住
    follow_err = 0.0f;
  }
  inst->debug.belt_rate_last = rate;
  if (fabsf(follow_err) < DART_DEBUG_BELT_MAX_ERR_DEG) inst->debug.belt_target += rate * dt_s;
  float limit = fabsf(rate);
  if (limit < DART_DEBUG_BELT_SYNC_SPEED_DPS) limit = DART_DEBUG_BELT_SYNC_SPEED_DPS;
  inst->belt.common_pid.MaxOut = limit;
  inst->belt.common_ref = inst->debug.belt_target;
  BeltSyncUpdate(inst, rate, DART_BELT_MAX_SPEED_DPS);  // 点动速度前馈 = 摇杆速率

  // ---- 丝杆: 速度前馈 + P 跟踪虚拟位置目标; READY 时平台被扳机锁住, 禁止动丝杆 ----
  if (inst->state != DART_STATE_READY) {
    float s_actual = inst->screw.motor->measure.total_angle;
    float s_rate = inst->debug.screw_speed_dps;
    float s_follow_err = inst->debug.screw_target - s_actual;
    if (s_rate != 0.0f && s_follow_err != 0.0f && ((s_rate > 0.0f) != (s_follow_err > 0.0f))) {
      inst->debug.screw_target = s_actual;
      s_follow_err = 0.0f;
    }
    if (s_rate == 0.0f && inst->debug.screw_rate_last != 0.0f) {
      inst->debug.screw_target = s_actual;
      s_follow_err = 0.0f;
    }
    inst->debug.screw_rate_last = s_rate;
    if (fabsf(s_follow_err) < DART_DEBUG_SCREW_MAX_ERR_DEG) inst->debug.screw_target += s_rate * dt_s;
    float s_v_cmd = s_rate + DART_DEBUG_SCREW_KP * s_follow_err;
    float s_cap = (s_rate == 0.0f) ? DART_DEBUG_SCREW_HOLD_SPEED_DPS : DART_SCREW_MAX_SPEED_DPS;
    VAL_LIMIT(s_v_cmd, -s_cap, s_cap);
    DJIMotorOuterLoop(inst->screw.motor, SPEED_LOOP);
    DJIMotorSetPIDRef(inst->screw.motor, s_v_cmd);
  } else {
    ApplyScrewPosition(inst);  // 保持射力位置不动
  }

  // ---- 状态日志(500ms): 位置差偏差大时告警, 否则正常状态 ----
  float pos_diff = (belt_l->measure.total_angle - belt_r->measure.total_angle) - inst->debug.belt_diff_base;
  if (now - inst->debug.log_ms >= DART_DEBUG_LOG_PERIOD_MS) {
    inst->debug.log_ms = now;
    if (fabsf(pos_diff) > DART_DEBUG_BELT_SYNC_WARN_DEG) {
      LOGERROR("[dart] debug belt sync posdiff %d deg (L %d, R %d, tgt %d)", (int)pos_diff,
               (int)belt_l->measure.total_angle, (int)belt_r->measure.total_angle, (int)inst->debug.belt_target);
    } else {
      // pos: 已校准时=相对零点的位置(0=释放方向硬限位); 未校准(调试档直接上电)=上电累计原始角
      int pos_l = (int)(inst->belt.zero_valid ? inst->belt.pos_rel[0] : belt_l->measure.total_angle);
      int pos_r = (int)(inst->belt.zero_valid ? inst->belt.pos_rel[1] : belt_r->measure.total_angle);
      LOGINFO("[dart] debug spd %d/%d pos %d/%d posdiff %d err %d cur %d/%d iout %d", (int)belt_l->measure.speed_aps,
              (int)belt_r->measure.speed_aps, pos_l, pos_r,
              (int)pos_diff, (int)follow_err, (int)belt_l->measure.real_current, (int)belt_r->measure.real_current,
              (int)belt_l->motor_controller.speed_PID.Iout);
      LOGINFO("[dart] debug sync ff %d v %d posdiff %d ref %d u %d si %d add %d/%d rel_speed %d", (int)inst->belt.v_ff_dps,
              (int)inst->belt.v_common_dps, (int)inst->belt.pos_diff, (int)inst->belt.diff_ref, (int)inst->belt.sync_u,
              (int)inst->belt.sync_i, (int)inst->belt.add_current[0], (int)inst->belt.add_current[1],
              (int)(belt_l->measure.speed_aps - belt_r->measure.speed_aps));
    }
  }
}

/* ==================== 对外接口 ==================== */

ShootDartInstance* ShootDartInit(void) {
  ShootDartInstance* inst = (ShootDartInstance*)zmalloc(sizeof(ShootDartInstance));

  // 电流前馈指针指向实例内的 belt_add_current[]: 差模偏置从这里叠加到速度环输出之后
  Motor_Init_Config_s belt_l_conf = DART_BELT_MOTOR_CONFIG(DART_BELT_CAN, DART_BELT_L_CAN_ID, DART_BELT_L_REVERSE,
                                                          &inst->belt.add_current[0]);
  inst->belt.motor[0] = DJIMotorInit(&belt_l_conf);
  Motor_Init_Config_s belt_r_conf = DART_BELT_MOTOR_CONFIG(DART_BELT_CAN, DART_BELT_R_CAN_ID, DART_BELT_R_REVERSE,
                                                          &inst->belt.add_current[1]);
  inst->belt.motor[1] = DJIMotorInit(&belt_r_conf);

  Motor_Init_Config_s screw_conf = DART_SCREW_MOTOR_CONFIG(DART_SCREW_CAN, DART_SCREW_CAN_ID, DART_SCREW_REVERSE);
  inst->screw.motor = DJIMotorInit(&screw_conf);

  // 扳机舵机: 上电即输出卡位角(保持扣住发射平台)
  PWM_Init_Config_s servo_conf = {
      .htim = DART_SERVO_PWM_TIM,
      .channel = DART_SERVO_PWM_CHANNEL,
      .period = DART_SERVO_PWM_PERIOD_S,
      .dutyratio = ServoAngleToDuty(DART_SERVO_CATCH_DEG),
      .callback = NULL,
      .id = inst,
  };
  inst->servo.pwm = PWMRegister(&servo_conf);
  inst->servo.angle_deg = DART_SERVO_CATCH_DEG;

  // 丝杆不参与自动校准: 零点 = 开机位置, 因此 DART_SCREW_POS_* 是相对上电位置的角度
  // 上电目标 = 0(保持开机位置不动), 射力由操作手调整
  inst->screw.pos_deg = 0.0f;

  // 共模位置环(pi 输出 = 速度给定)。Iout 不参与(纯 P), DeadBand = 0
  PID_Init_Config_s belt_common_cfg = {
      .Kp = DART_BELT_POS_KP,
      .Ki = 0.0f,
      .Kd = 0.0f,
      .MaxOut = DART_BELT_MAX_SPEED_DPS,
      .IntegralLimit = 0.0f,
      .DeadBand = 0.0f,
      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,
  };
  PIDInit(&inst->belt.common_pid, &belt_common_cfg);

  inst->state = DART_STATE_IDLE;
  inst->last_ms = (uint32_t)DWT_GetTimeline_ms();
  inst->task_dt_s = DART_TASK_DT_S;
  return inst;
}

void ShootDartSetEnable(ShootDartInstance* inst, bool enable, bool auto_cali) {
  uint32_t now = (uint32_t)DWT_GetTimeline_ms();

  if (enable && !inst->enabled) {
    inst->enabled = true;
    inst->pending_cmd = DART_CMD_NONE;  // 清掉失能前潜伏的命令
    for (int i = 0; i < 2; i++) DJIMotorEnable(inst->belt.motor[i]);
    DJIMotorEnable(inst->screw.motor);
    DartBuzzerEnableOk();
    ClearBeltControl(inst);
    PIDClear(&inst->screw.motor->motor_controller.speed_PID);
    PIDClear(&inst->screw.motor->motor_controller.angle_PID);
    // 失能时被中止的序列统一回 IDLE 兜底
    if (inst->state == DART_STATE_CALIBRATING || inst->state == DART_STATE_CHARGING ||
        inst->state == DART_STATE_FIRING) {
      if (inst->state == DART_STATE_CHARGING && inst->is_calibrated) inst->recovery_retract = true;
      inst->state = DART_STATE_IDLE;
    }
    // 上电后必须先完成一次零位校准(调试档传 auto_cali=false: 先点动验方向)
    if (auto_cali && !inst->is_calibrated && inst->state == DART_STATE_IDLE && !inst->recovery_retract) {
      StartCalibration(inst, now);
    }
  } else if (!enable && inst->enabled) {
    switch (inst->state) {
      case DART_STATE_CALIBRATING:
        inst->state = DART_STATE_IDLE;
        inst->is_calibrated = false;
        inst->belt.zero_valid = false;
        break;
      case DART_STATE_CHARGING:
        inst->state = DART_STATE_IDLE;
        inst->charge.step = CHARGE_STEP_PREP_SCREW;
        if (inst->is_calibrated) inst->recovery_retract = true;
        break;
      case DART_STATE_FIRING:
        inst->state = DART_STATE_IDLE;
        inst->fire.step = FIRE_STEP_RELEASE;
        break;
      default:
        break;
    }
    inst->enabled = false;
    inst->pending_cmd = DART_CMD_NONE;  // 命令不允许潜伏到下次使能后才执行
    for (int i = 0; i < 2; i++) DJIMotorStop(inst->belt.motor[i]);
    DJIMotorStop(inst->screw.motor);
    DartBuzzerDisable();
  }
}

void ShootDartSetCommand(ShootDartInstance* inst, Dart_Cmd_e cmd) { inst->pending_cmd = cmd; }

void ShootDartAdjustScrewPos(ShootDartInstance* inst, float delta_deg) {
  // 仅 IDLE 生效: READY 时平台被扳机锁住、拉簧带载, 动丝杆=带载推棘爪, 有误释放/卡滞风险
  if (inst->state != DART_STATE_IDLE) return;
  inst->screw.pos_deg += delta_deg;
  VAL_LIMIT(inst->screw.pos_deg, DART_SCREW_POS_MIN_DEG, DART_SCREW_POS_MAX_DEG);
}

void ShootDartAdjustServoAngle(ShootDartInstance* inst, float delta_deg) {
  // 仅 IDLE 生效: 序列中舵机由状态机接管, 防止摇杆误改卡位/释放角
  if (inst->state != DART_STATE_IDLE) return;
  SetServoAngle(inst, inst->servo.angle_deg + delta_deg);
}

void ShootDartSetDebug(ShootDartInstance* inst, bool enable, float belt_speed_dps, float screw_speed_dps) {
  inst->debug.enabled = enable;
  inst->debug.belt_speed_dps = belt_speed_dps;
  inst->debug.screw_speed_dps = screw_speed_dps;
}

void ShootDartTask(ShootDartInstance* inst) {
  if (!inst->enabled) return;

  uint32_t now = (uint32_t)DWT_GetTimeline_ms();
  float dt_s = (float)(now - inst->last_ms) * 0.001f;
  if (dt_s <= 0.0f || dt_s > 0.05f) dt_s = DART_TASK_DT_S;
  inst->last_ms = now;
  inst->task_dt_s = dt_s;
  // 两侧相对零点的位置(0=释放方向硬限位, 正=储能方向); 未校准时为 0
  for (int i = 0; i < 2; i++) {
    inst->belt.pos_rel[i] = inst->belt.zero_valid ? inst->belt.motor[i]->measure.total_angle - inst->belt.zero_offset[i]
                                                  : 0.0f;
  }

  // 校准完成前严格限制同步带/丝杆力矩(所有模式生效), 防止卡死损坏
  float belt_max = inst->is_calibrated ? DART_BELT_MAX_OUT : DART_BELT_CALI_MAX_OUT;
  float belt_ilim = inst->is_calibrated ? DART_BELT_INTEGRAL_LIMIT : DART_BELT_CALI_INTEGRAL_LIMIT;
  for (int i = 0; i < 2; i++) {
    inst->belt.motor[i]->motor_controller.speed_PID.MaxOut = belt_max;
    inst->belt.motor[i]->motor_controller.speed_PID.IntegralLimit = belt_ilim;
  }
  inst->screw.motor->motor_controller.speed_PID.MaxOut =
      inst->is_calibrated ? DART_SCREW_MAX_OUT : DART_SCREW_CALI_MAX_OUT;
  inst->screw.motor->motor_controller.speed_PID.IntegralLimit =
      inst->is_calibrated ? DART_SCREW_INTEGRAL_LIMIT : DART_SCREW_CALI_INTEGRAL_LIMIT;

  HandleCommand(inst, now);

  if (inst->state == DART_STATE_FAULT) {
    // 故障保持(不断力矩): 双带保持故障位置, 丝杆保持; yaw 由 RobotTask 断力矩
    SetBeltPosition(inst, inst->belt.hold_pos, DART_BELT_HOME_SPEED_DPS, BELT_MOVE_NORMAL);
    ApplyScrewPosition(inst);
    return;
  }

  // 调试档 = 纯手动: 拨到调试档时正在进行的自动校准/储能立即中止, 机构交还手动点动
  if (inst->debug.enabled && (inst->state == DART_STATE_CALIBRATING || inst->state == DART_STATE_CHARGING)) {
    if (inst->state == DART_STATE_CALIBRATING) {
      inst->is_calibrated = false;
      inst->belt.zero_valid = false;
      DartBuzzerCaliBgmStop();
      LOGWARNING("[dart] auto calibration aborted: debug mode (manual move only, commands rejected)");
    } else {
      if (inst->is_calibrated && inst->charge.step != CHARGE_STEP_PREP_SCREW) inst->recovery_retract = true;
      inst->charge.step = CHARGE_STEP_PREP_SCREW;
      LOGWARNING("[dart] charge sequence aborted: debug mode");
    }
    inst->state = DART_STATE_IDLE;
    inst->debug.belt_synced = false;
    inst->debug.screw_synced = false;
  }

  // 失能中止储能后的复位: 优先于点动和 IDLE 保持(调试档会跳过 HandleIdle, 否则标志永不清)
  if (inst->state == DART_STATE_IDLE && inst->recovery_retract) {
    SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_HOME_SPEED_DPS, BELT_MOVE_HOME);
    if (BeltPositionReached(inst, DART_BELT_HOME_DEG, DART_BELT_POS_TOL_DEG)) {
      inst->recovery_retract = false;
      LOGINFO("[dart] recovery retract done (block back at release position)");
    }
  } else if (inst->debug.enabled && (inst->state == DART_STATE_IDLE || inst->state == DART_STATE_READY)) {
    ApplyDebugMove(inst, dt_s, now);  // IDLE 全部可动; READY 只动同步带
  } else {
    inst->debug.belt_synced = false;  // 点动结束/状态机接管: 下次点动重新同步虚拟目标
    inst->debug.screw_synced = false;
    switch (inst->state) {
      case DART_STATE_IDLE:
      case DART_STATE_READY:
        SetBeltPosition(inst, DART_BELT_HOME_DEG, DART_BELT_HOME_SPEED_DPS, BELT_MOVE_HOME);
        break;
      case DART_STATE_CALIBRATING:
        HandleCalibrating(inst, now);
        break;
      case DART_STATE_CHARGING:
        HandleCharging(inst, now);
        break;
      case DART_STATE_FIRING:
        HandleFiring(inst, now);
        break;
      default:
        break;
    }
    if (inst->state != DART_STATE_FIRING) SetServoAngle(inst, DART_SERVO_CATCH_DEG);  // 非发射时舵机保持卡位
    if (inst->state != DART_STATE_CALIBRATING) ApplyScrewPosition(inst);  // 校准中由状态机控制丝杆
  }

  MonitorBeltSync(inst, now);
}
