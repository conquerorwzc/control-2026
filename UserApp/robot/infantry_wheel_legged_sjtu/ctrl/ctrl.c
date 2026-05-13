#include "ctrl.h"

#include <math.h>

#include "general_def.h"
#include "robot_config.h"
#include "ui.h"

// Static variables for control state
#ifdef USE_RC_CTRL
static RC_ctrl_t* rc_data;
static RC_ctrl_t rc_data_last;
#elifdef USE_OCD_CTRL
static VT13_RC_t* rc_data;
static VT13_RC_t rc_data_last;
#endif

// 共享状态标志: RC 路径只用 is_first_update, OCD 路径全部使用
static UpdateFlag_s update_flag = {.is_first_update = 1};

static CtrlInstance ocd = {
    .leg.index = 1,
    // 速度常量: vx = m/s (默认平动, stand + prostrate 通用), wz = rad/s (小陀螺角速度上限);
    //           stair / vault 仅用于键鼠 WASD 覆盖; 趴下由 chassis.ChassisProstrate 做 m/s → motor rpm 换算.
    .speed = {.vx = 2.5f, .wz = 15.0f, .stair = 2.2f, .vault = 1.8f},
    // CHASSIS_RECOVERY 触发阈值: 默认 13°, 蹭台阶模式更敏感 (7°)
    .recovery = {.pitch_default = 13.0f, .pitch_creep = 13.0f},
};
// 四档腿长 (m): MIN, 默认 (与 chassis 上电一致), MID, MAX
static const float LEG_TABLE[4] = {0.117f, 0.20f, 0.285f, 0.370f};

// 倾翻判定: |Pitch| > 当前模式阈值即视为失控, 触发 CHASSIS_RECOVERY. 蹭台阶模式下用更小阈值更早介入.
static uint8_t IsRobotLostControl(RobotInstance* robot) {
  const float thresh = ocd.stair.active ? ocd.recovery.pitch_creep : ocd.recovery.pitch_default;
  return fabsf(robot->chassis->imu->Pitch) > thresh;
}

static void RequestUiForceRefresh(RobotInstance* robot) {
  if (robot == NULL) return;

#if !defined(ONE_BOARD)
  if (robot->chassis_fetch_data != NULL) {
    robot->chassis_fetch_data->main.force_refresh_ui = 1;
  }
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  Referee_Interactive_info_t* ui_data = getUI();
  if (ui_data != NULL) {
    ui_data->force_refresh_ui = 1;
  }
#endif
}

// Ramp / yaw planner 已迁移到 chassis 层 (chassis.c::ChassisPlannerUpdate),
// 在 chassis 时基 (200Hz) 平滑 cmd, 避免双板 50Hz CAN ZOH 把 LQR 参考切阶梯.
// ctrl 层只负责生成 raw 期望 (target_yaw / vx / wz), 直写 chassis_ctrl_cmd.

static void ResetChassisMotionMemory(RobotInstance* robot) {
  if (robot == NULL || robot->chassis == NULL) return;

  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD;
  chassis_ctrl_cmd->vx = 0.0f;
  chassis_ctrl_cmd->wz = 0.0f;
  chassis_ctrl_cmd->theta_ff = 0.0f;
  chassis_ctrl_cmd->is_rotate = 0;
  robot->chassis->state_var.x_b = 0.0f;
  robot->chassis->last_state_var.x_b = 0.0f;
}

static uint8_t ResetMotionMemoryOnPostureEdge(RobotInstance* robot) {
  static uint8_t has_last_mode = 0;
  static uint8_t was_prostrate = 0;
  static uint8_t was_power_off = 1;
  uint8_t did_reset = 0;

  uint8_t is_prostrate =
      (robot->robot_mode == ROBOT_CHASSIS_PROSTRATE_FOLLOW) || (robot->robot_mode == ROBOT_CHASSIS_PROSTRATE_ROTATE);
  uint8_t is_power_off = robot->robot_mode == ROBOT_POWER_OFF;
  if (!has_last_mode || is_prostrate != was_prostrate || is_power_off || was_power_off) {
    ResetChassisMotionMemory(robot);
    did_reset = 1;
  }

  was_prostrate = is_prostrate;
  was_power_off = is_power_off;
  has_last_mode = 1;
  return did_reset;
}

// ==========================================
// 统一的机器人运动解算层
// ==========================================
static void RobotMotionSolve(RobotInstance* robot, Ctrl_Intent_s* intent) {
  static float yaw_ref = 0.0f;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  if (ResetMotionMemoryOnPostureEdge(robot)) {
    yaw_ref = chassis_ctrl_cmd->target_yaw;
  }
  float input_mag = sqrtf(intent->vx * intent->vx + intent->vy * intent->vy);

  // 小陀螺标志: 仅 LQR 平衡态下的 ROTATE 才使能 (PROSTRATE_ROTATE 走 ChassisProstrateMode, 不进 LQR)
  chassis_ctrl_cmd->is_rotate = (robot->robot_mode == ROBOT_CHASSIS_ROTATE) ? 1 : 0;

  chassis_ctrl_cmd->roll = 0.0f;
  // roll 前馈 = 底盘恒定偏置 + 云台 CoM 旋转分量. offset_angle 俯视 CW 为正 → 底盘系下 CoM 角 = α_g - offset_angle.
  chassis_ctrl_cmd->roll_ff =
      ROLL_FF_BIAS + ROLL_FF_AMP * sinf((GIMBAL_COM_ANGLE_DEG - robot->offset_angle) * DEGREE_2_RAD);

  switch (robot->robot_mode) {
    case ROBOT_CHASSIS_ROTATE: {
      // target_yaw 跟当前 imu, planner 见 is_rotate=1 后忽略 yaw_err, 单纯靠 cmd.wz FF 推进 target_yaw.
      // 退出 ROTATE 瞬间 raw 即对齐当前姿态, 无 lurch.
      chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD;
      chassis_ctrl_cmd->wz = intent->rotate_scale * ocd.speed.wz;  // 拨轮/shift 缩放 × ocd.speed.wz 上限
      chassis_ctrl_cmd->vx = 0.0f;
      break;
    }
    case ROBOT_CHASSIS_FOLLOW: {
#if (!defined(ONE_BOARD))
      const uint8_t has_move_input = input_mag > 0.0005f;
      float move_angle_deg = has_move_input ? (atan2f(intent->vy, intent->vx) - PI / 2.0f) * RAD_2_DEGREE : 0.0f;
      float follow_err = wrap180(move_angle_deg - robot->offset_angle);
      float rear_err = wrap180(follow_err - 180.0f);
      // X 掉头反向跟随: 强制走 rear 分支 (复用 input_mag 取反, 让 W = 后退);
      // 同时把 follow_err 清零, chassis target_yaw 锁在当前姿态, 不跟随云台 — 直到 CtrlSolve 监测到云台已转过 130° 后清除标志.
      if (intent->reverse_follow || fabsf(rear_err) < fabsf(follow_err)) {
        follow_err = rear_err;
        if (has_move_input) input_mag = -input_mag;
      }
      if (intent->reverse_follow) {
        follow_err = 0.0f;
      }
      // intent->gimbal_yaw_ff = 0.0f;
      yaw_ref = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + (follow_err + intent->gimbal_yaw_ff) * DEGREE_2_RAD;
      chassis_ctrl_cmd->target_yaw = yaw_ref;  // raw, chassis planner 平滑
      chassis_ctrl_cmd->wz = 0.0f;

      float align_attenuation = cosf(follow_err * DEGREE_2_RAD);
      if (align_attenuation < 0) align_attenuation = 0;
      input_mag *= align_attenuation * align_attenuation * align_attenuation;

      chassis_ctrl_cmd->vx = input_mag;  // raw vx, chassis vx_ramp 平滑
      chassis_ctrl_cmd->theta_ff = 0.0f;
      break;
#endif
    }
    case ROBOT_CHASSIS_FREE: {
      float input_vy = intent->vy;
#if defined(ONE_BOARD)
      yaw_ref = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD;
#else
      float follow_err = wrap180(-robot->offset_angle);
      float rear_err = wrap180(follow_err - 180.0f);
      if (fabsf(rear_err) < fabsf(follow_err)) {
        follow_err = rear_err;
        input_vy = -input_vy;
      }
      // intent->gimbal_yaw_ff = 0.0f;
      yaw_ref = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + (follow_err + intent->gimbal_yaw_ff) * DEGREE_2_RAD;
#endif
      chassis_ctrl_cmd->target_yaw = yaw_ref;  // raw, chassis planner 平滑
      // 与 FOLLOW 一致: FREE 只用 target_yaw 位置误差收敛, 不叠加当前 gyro 前馈.
      chassis_ctrl_cmd->wz = 0.0f;
      chassis_ctrl_cmd->vx = input_vy;  // raw vx, chassis vx_ramp 平滑
      chassis_ctrl_cmd->theta_ff = 0.0f;
      chassis_ctrl_cmd->roll += intent->roll_delta;
      chassis_ctrl_cmd->leg_length += intent->leg_length_delta;

      if (chassis_ctrl_cmd->leg_length > LEG_MAX_LENGTH)
        chassis_ctrl_cmd->leg_length = LEG_MAX_LENGTH;
      else if (chassis_ctrl_cmd->leg_length < LEG_MIN_LENGTH)
        chassis_ctrl_cmd->leg_length = LEG_MIN_LENGTH;

      break;
    }
    case ROBOT_CHASSIS_PROSTRATE_ROTATE: {
      chassis_ctrl_cmd->roll = 0.0f;
      chassis_ctrl_cmd->vx = 0.0f;
      // yaw_prostrate_PID 不参与: target_yaw 跟踪当前 yaw, 仅靠 wz 前馈驱动旋转.
      // 否则 target_yaw 不变, yaw 误差累积后 PID 会反向拽住, 转到一定角度就停.
      chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD;
      yaw_ref = chassis_ctrl_cmd->target_yaw;
      chassis_ctrl_cmd->wz = intent->rotate_scale * ocd.speed.wz;  // rad/s, chassis 层换算
      break;
    }
    case ROBOT_CHASSIS_PROSTRATE_FOLLOW: {
#if (!defined(ONE_BOARD))
      chassis_ctrl_cmd->roll = 0.0f;
      chassis_ctrl_cmd->wz = 0.0f;
      // 与 FOLLOW 一致的 m/s 判定阈值 (之前的 5.0 是原始摇杆量级残留).
      const uint8_t has_move_input = input_mag > 0.0005f;
      float move_angle_deg = has_move_input ? (atan2f(intent->vy, intent->vx) - PI / 2.0f) * RAD_2_DEGREE : 0.0f;
      float follow_err = wrap180(move_angle_deg - robot->offset_angle);
      float rear_err = wrap180(follow_err - 180.0f);
      if (fabsf(rear_err) < fabsf(follow_err)) {
        follow_err = rear_err;
        if (has_move_input) input_mag = -input_mag;
      }
      // intent->gimbal_yaw_ff = 0.0f;
      yaw_ref = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + (follow_err + intent->gimbal_yaw_ff) * DEGREE_2_RAD;
      chassis_ctrl_cmd->target_yaw = yaw_ref;

      // m/s 量级, 不再做原始摇杆 ±800 截断 (原数值对应 ~800/660 = 1.2×peak).
      float align_attenuation = cosf(follow_err * DEGREE_2_RAD);
      if (align_attenuation < 0) align_attenuation = 0;
      input_mag *= align_attenuation * align_attenuation * align_attenuation;
      chassis_ctrl_cmd->vx = input_mag;
      break;
#endif
    }

    default:
      break;
  }

  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  PIDSwitchConfig(&robot->gimbal->yaw_motor->motor_controller.angle_PID, gimbal_ctrl_cmd->gimbal_mode == GIMBAL_VISION
                                                                             ? &yaw_angle_PID_vision_config
                                                                             : &yaw_angle_PID_manual_config);

  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE)
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE)
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
}

// ==========================================
// 逻辑层与输入层 (OCD CTRL)
// ==========================================
#ifdef USE_OCD_CTRL

// ==========================================
// 模式推导 (CtrlSolve 调用一次, 是 robot_mode/chassis_mode 的唯一写入点)
// RECOVERY 期间不覆盖任何模式字段, 让 GimbalAlign 走完对齐流程后置回 CHASSIS_ON.
// ==========================================
static void ApplyOcdMode(RobotInstance* robot) {
  Chassis_Ctrl_Cmd_s* cmd = &robot->chassis->chassis_ctrl_cmd;
  if (cmd->chassis_mode == CHASSIS_RECOVERY) return;

  switch (ocd.jump.phase) {
    case JUMP_ACTIVE:
      cmd->chassis_mode = CHASSIS_JUMP_START;
      cmd->jump_force = JUMP_FORCE;
      return;
    case JUMP_READY:
      cmd->chassis_mode = CHASSIS_JUMP_READY;
      return;
    case JUMP_IDLE:
      break;
  }

  cmd->chassis_mode = update_flag.is_stand ? CHASSIS_ON : CHASSIS_PROSTRATE;

  // 优先级: rotate > free > follow; stand 轴决定 stand / prostrate 变体.
  // 趴下时没有 LQR 平衡, FREE (解耦云台跟随 + roll/腿长微调) 没有实际意义;
  // 且 ChassisProstrate 期望 vx 为原始摇杆量级 (±660), ROBOT_CHASSIS_FREE 的 vx_ramp
  // 输出是 m/s 量级 (±2.97), 量纲错配会让前后驱动接近 0, 故 prostrate 强制走 FOLLOW.
  if (update_flag.is_rotate) {
    robot->robot_mode =
        update_flag.is_stand || update_flag.is_free ? ROBOT_CHASSIS_ROTATE : ROBOT_CHASSIS_PROSTRATE_ROTATE;
  } else if (update_flag.is_free && update_flag.is_stand) {
    robot->robot_mode = ROBOT_CHASSIS_FREE;
  } else {
    robot->robot_mode = update_flag.is_stand ? ROBOT_CHASSIS_FOLLOW : ROBOT_CHASSIS_PROSTRATE_FOLLOW;
  }
}

// ==========================================
// JoyStickCtrl: 摇杆 -> ocd.js_* / ocd.intent.{rotate_scale, roll_delta, leg_length_delta}
//             + 直接积分 gimbal yaw/pitch 位置目标
//             + 每帧刷新 update_flag (is_rotate 派生; is_stand/is_free 边沿翻转, 急停姿态锁定)
// ==========================================
void JoyStickCtrl(RobotInstance* robot) {
  rc_data = robot->rc_data;
  if (update_flag.is_first_update) {
    rc_data_last = *rc_data;
    update_flag.is_first_update = 0;
  }

  // 帧起始: JS 输入字段清零 (持久字段 jump/leg 保留)
  ocd.intent = (Ctrl_Intent_s){0};
  ocd.js_vx = 0.0f;
  ocd.js_vy = 0.0f;
  ocd.js_yaw_ff = 0.0f;
  ocd.js_rotate_scale = 0.0f;
  ocd.js_shoot = (ShootReq_s){0};

  Chassis_Ctrl_Cmd_s* chassis_cmd = &robot->chassis->chassis_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_cmd = &robot->shoot->shoot_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_cmd = &robot->gimbal->gimbal_ctrl_cmd;

  // is_on: 拨杆不在最左 = 非急停姿态 (mode_switch 是 JS 域输入, 由 JS 派生供 MK 读)
  update_flag.is_on = !switch_left(rc_data->rc.mode_switch);

  // fn_1 边沿: 站立 <-> 趴下 (跳跃 ACTIVE / RECOVERY / 急停姿态时锁定)
  const uint8_t fn_1_edge = rc_data->rc.fn_1 && !rc_data_last.rc.fn_1;
  if (fn_1_edge && update_flag.is_on && chassis_cmd->chassis_mode != CHASSIS_RECOVERY &&
      ocd.jump.phase != JUMP_ACTIVE) {
    ocd.jump.phase = JUMP_IDLE;
    ocd.jump.observed_active = 0;
    update_flag.is_stand = !update_flag.is_stand;
  }

  // pause 按钮边沿: FREE <-> 非 FREE (急停姿态时锁定)
  const uint8_t pause_edge = rc_data->rc.pause && !rc_data_last.rc.pause;
  if (pause_edge && update_flag.is_on) {
    update_flag.is_free = !update_flag.is_free;
  }

  // RECOVERY 期间清空跳跃 FSM (避免对齐后残留 READY/ACTIVE 状态)
  if (chassis_cmd->chassis_mode == CHASSIS_RECOVERY) {
    ocd.jump.phase = JUMP_IDLE;
    ocd.jump.observed_active = 0;
  }

  // 拨轮 -> JS 小陀螺缩放系数 (dial/660, -1..1, 符号决定转向; MK 的 shift 由 MouseKeyCtrl 独立处理)
  if (abs(rc_data->rc.dial) > 20) {
    ocd.js_rotate_scale = (float)rc_data->rc.dial / 660.0f;
  }

  // 右拨杆 -> 摩擦轮 / 扳机请求
  static float trigger_t0 = 0;
  static uint8_t trigger_last = 0;
  if (switch_right(rc_data->rc.mode_switch)) {
    shoot_cmd->shoot_mode = SHOOT_ON;
    shoot_cmd->friction_mode = FRICTION_ON;
    if (!trigger_last && rc_data->rc.trigger) {
      trigger_t0 = DWT_GetTimeline_s();
    }
    trigger_last = rc_data->rc.trigger;
    if (rc_data->rc.trigger) {
      ocd.js_shoot.fire = 1;
      ocd.js_shoot.burst = (DWT_GetTimeline_s() - trigger_t0 > 1.0f) ? 1 : 0;
    }
  } else {
    shoot_cmd->shoot_mode = SHOOT_OFF;
    shoot_cmd->friction_mode = FRICTION_OFF;
    shoot_cmd->load_mode = LOAD_STOP;
  }

  // 云台 yaw / pitch (CtrlSolve 中自瞄请求生效时会覆盖)
  gimbal_cmd->gimbal_mode = GIMBAL_ON;
  const float yaw_delta = -0.00035f * (float)rc_data->rc.rocker_r_;
  gimbal_cmd->yaw += yaw_delta;
  ocd.js_yaw_ff = yaw_delta * 30.0f;

  if (!(update_flag.is_stand && update_flag.is_free)) {
    gimbal_cmd->pitch += -0.00006f * (float)rc_data->rc.rocker_r1;
  }

  // 摇杆 -> 运动输入 (全姿态 m/s, 趴下由 chassis 层换算到电机量)
  if (update_flag.is_stand && update_flag.is_free) {
    // 站立 + FREE: 右摇杆纵向 = 前进; 左摇杆 = roll / 腿长 微调
    ocd.js_vy = 0.003f * (float)rc_data->rc.rocker_r1;
    ocd.intent.roll_delta = 0.0003f * (float)rc_data->rc.rocker_l_ * (abs(rc_data->rc.rocker_l_) > 10);
    ocd.intent.leg_length_delta = 0.0000005f * (float)rc_data->rc.rocker_l1;
  } else {
    // 站立 (FOLLOW / ROTATE) / 趴下 (PROSTRATE_FOLLOW / PROSTRATE_ROTATE / !stand+FREE): m/s 量级
    // ROTATE 模式下 vx 不读, 写无害.
    ocd.js_vx = 0.003f * (float)rc_data->rc.rocker_l_;
    ocd.js_vy = 0.003f * (float)rc_data->rc.rocker_l1;
  }
}

// ==========================================
// MouseKeyCtrl: 键鼠 -> ocd.mk_* (运动/开火/自瞄)
//             + 边沿驱动 update_flag.is_stand (Ctrl+G, 急停姿态锁定) / ocd.jump / ocd.leg
//             + 直接积分 gimbal yaw/pitch 位置目标
// ==========================================
void MouseKeyCtrl(RobotInstance* robot) {
  rc_data = robot->rc_data;

  // 帧起始: MK 输入字段清零 (持久字段保留)
  ocd.mk_vx = 0.0f;
  ocd.mk_vy = 0.0f;
  ocd.mk_yaw_ff = 0.0f;
  ocd.mk_rotate_scale = 0.0f;
  ocd.mk_shoot = (ShootReq_s){0};
  ocd.mk_vision = 0;

  Chassis_Ctrl_Cmd_s* chassis_cmd = &robot->chassis->chassis_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_cmd = &robot->shoot->shoot_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Vision_Receive_s* vision = robot->vision_recv_data;
  const VT13_MouseKey_t* mk = &rc_data->mouse_key;
  const VT13_MouseKey_t* mk_last = &rc_data_last.mouse_key;

  // shift 持续按住 -> MK 小陀螺缩放系数 = 1 (满速; JS 的拨轮独立处理, CtrlSolve 仲裁)
  if (mk->keyboard.shift) {
    ocd.mk_rotate_scale = 1.0f;
  }

  // 鼠标左键 -> 开火请求 (按住超过 0.3s 转连发)
  static float mouse_trigger_t0 = 0;
  if (mk->mouse.press_l) {
    if (shoot_cmd->friction_mode == FRICTION_ON) {
      ocd.mk_shoot.fire = 1;
      ocd.mk_shoot.burst = (DWT_GetTimeline_s() - mouse_trigger_t0 > 0.3f) ? 1 : 0;
    }
  } else {
    mouse_trigger_t0 = DWT_GetTimeline_s();
  }

  // 鼠标右键 + vision 有数据 -> 自瞄请求 (CtrlSolve 中应用)
  if (mk->mouse.press_r && has_non_zero_data(vision)) {
    ocd.mk_vision = 1;
  }

  // 鼠标增量 -> 云台 yaw / pitch (自瞄请求时 yaw_ff 由 CtrlSolve 统一清零)
  const float yaw_delta = -(float)mk->mouse.x * 0.002f;
  gimbal_cmd->yaw += yaw_delta;
  ocd.mk_yaw_ff = yaw_delta * 10.0f;
  gimbal_cmd->pitch += -(float)mk->mouse.y * 0.002f;

  // X 边沿: 云台 +180° 掉头 + 启动反向跟随 (chassis 不旋转, W 反向, 待云台转过 REVERSE_FOLLOW_EXIT_DEG 后 CtrlSolve 清零)
  if (mk->keyboard.x && !mk_last->keyboard.x) {
    gimbal_cmd->yaw += 180.0f;
    ocd.reverse.active = 1;
    ocd.reverse.start_yaw = robot->gimbal->gimbal_IMU_data->YawTotalAngle;
  }

  // Z 边沿: 蹭台阶模式切换 (仅 stand). 进入: 快照腿档 + 切最高档 + 慢速; 退出: 恢复腿档 + 默认速度.
  if (mk->keyboard.z && !mk_last->keyboard.z && update_flag.is_stand) {
    if (!ocd.stair.active) {
      ocd.stair.active = 1;
      ocd.stair.saved_leg_idx = ocd.leg.index;
      ocd.leg.index = 3;
    } else {
      ocd.stair.active = 0;
      ocd.leg.index = ocd.stair.saved_leg_idx;
    }
    ocd.leg.pending = 1;
  }

  // B 边沿: 强制 UI 刷新
  if (mk->keyboard.b && !mk_last->keyboard.b) {
    RequestUiForceRefresh(robot);
  }

  // C 边沿: 超级电容 BOOST <-> NORMAL
  if (mk->keyboard.c && !mk_last->keyboard.c) {
    robot->chassis->super_cap->super_cap_ctrl_cmd =
        (robot->chassis->super_cap->super_cap_ctrl_cmd == BOOST) ? NORMAL : BOOST;
  }

  // Q / E 持续按住 -> roll 增量 (与 JS rocker_l_ 加性叠加, 同速度量纲)
  if (mk->keyboard.q)
    ocd.intent.roll_delta -= 0.4f;
  else if (mk->keyboard.e)
    ocd.intent.roll_delta += 0.4f;

  // R / F 边沿: 四档腿长升降 (绝对写入挂起, CtrlSolve 消费)
  if (mk->keyboard.r && !mk_last->keyboard.r) {
    if (ocd.leg.index < 3) ocd.leg.index++;
    ocd.leg.pending = 1;
  }
  if (mk->keyboard.f && !mk_last->keyboard.f) {
    if (ocd.leg.index > 0) ocd.leg.index--;
    ocd.leg.pending = 1;
  }

  // Ctrl+G 边沿: 站立 <-> 趴下 (跳跃 ACTIVE / RECOVERY / 急停姿态时锁定)
  const uint8_t ctrl_g = mk->keyboard.ctrl && mk->keyboard.g;
  const uint8_t ctrl_g_last = mk_last->keyboard.ctrl && mk_last->keyboard.g;
  if (ctrl_g && !ctrl_g_last && update_flag.is_on && chassis_cmd->chassis_mode != CHASSIS_RECOVERY &&
      ocd.jump.phase != JUMP_ACTIVE) {
    ocd.jump.phase = JUMP_IDLE;
    ocd.jump.observed_active = 0;
    update_flag.is_stand = !update_flag.is_stand;
  }

  // Ctrl+V 边沿: jump.phase IDLE <-> READY
  // V (无 Ctrl) 边沿: READY -> ACTIVE 起跳
  const uint8_t ctrl_v = mk->keyboard.ctrl && mk->keyboard.v;
  const uint8_t ctrl_v_last = mk_last->keyboard.ctrl && mk_last->keyboard.v;
  const uint8_t v_edge = mk->keyboard.v && !mk_last->keyboard.v;
  if (ctrl_v && !ctrl_v_last && chassis_cmd->chassis_mode != CHASSIS_RECOVERY && ocd.jump.phase != JUMP_ACTIVE) {
    ocd.jump.phase = (ocd.jump.phase == JUMP_READY) ? JUMP_IDLE : JUMP_READY;
    ocd.jump.observed_active = 0;
  } else if (v_edge && !mk->keyboard.ctrl && ocd.jump.phase == JUMP_READY) {
    ocd.jump.phase = JUMP_ACTIVE;
    ocd.jump.active_since = DWT_GetTimeline_s();
    ocd.jump.observed_active = 0;
  }

  // WASD -> ocd.mk_vx/mk_vy (全姿态 m/s 单位, 趴下由 chassis 层换算到电机量)
  // 速度优先级 (高到低): vault (跳台阶) > stair (蹭台阶) > 默认 vx.
  float wasd_scale;
  const uint8_t in_vault = (ocd.jump.phase == JUMP_READY) || (ocd.jump.phase == JUMP_ACTIVE);
  if (in_vault) {
    wasd_scale = ocd.speed.vault;
  } else if (update_flag.is_stand && ocd.stair.active) {
    wasd_scale = ocd.speed.stair;
  } else {
    wasd_scale = ocd.speed.vx;
  }
  if (mk->keyboard.w) ocd.mk_vy += wasd_scale;
  if (mk->keyboard.s) ocd.mk_vy -= wasd_scale;
  if (mk->keyboard.d) ocd.mk_vx += wasd_scale;
  if (mk->keyboard.a) ocd.mk_vx -= wasd_scale;
}

// ==========================================
// CtrlSolve: 仲裁 -> ApplyOcdMode -> RobotMotionSolve -> Jump FSM 推进
// 仲裁策略:
//   运动 (vx/vy):  JS 离中位时优先, 否则 MK 接管 (避免叠加爆冲)
//   yaw 前馈:       JS + MK 加性 (velocity 性质, 两手并用更快)
//   开火:           OR 合并 (任一路触发都射)
//   自瞄/跳跃/腿长:  单源 (MK 写, 无冲突)
// ==========================================
void CtrlSolve(RobotInstance* robot) {
  Chassis_Ctrl_Cmd_s* chassis_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_cmd = &robot->shoot->shoot_ctrl_cmd;

  // 1. 运动轴: JS 优先 / MK 兜底
  const uint8_t js_owns = (fabsf(ocd.js_vx) > 1e-3f) || (fabsf(ocd.js_vy) > 1e-3f);
  ocd.intent.vx = js_owns ? ocd.js_vx : ocd.mk_vx;
  ocd.intent.vy = js_owns ? ocd.js_vy : ocd.mk_vy;

  // 2. 小陀螺缩放系数: JS 拨轮带符号 (-1..1), MK shift 仅正向 (0 或 +1).
  //    JS 拨轮活跃时优先 (保留转向); 否则 MK shift 接管, 避免符号被丢失.
  const uint8_t js_rotate_active = fabsf(ocd.js_rotate_scale) > 0.0f;
  ocd.intent.rotate_scale = js_rotate_active ? ocd.js_rotate_scale : ocd.mk_rotate_scale;
  update_flag.is_rotate = (fabsf(ocd.intent.rotate_scale) > 0.0f);

  // 3. 云台 yaw 前馈: 加性合并; 自瞄请求生效时覆盖云台目标, 清空底盘前馈
  ocd.intent.gimbal_yaw_ff = ocd.js_yaw_ff + ocd.mk_yaw_ff;
  if (ocd.mk_vision) {
    gimbal_cmd->gimbal_mode = GIMBAL_VISION;
    gimbal_cmd->yaw = robot->vision_recv_data->gimbal_receive.yaw;
    gimbal_cmd->pitch = robot->vision_recv_data->gimbal_receive.pitch;
    ocd.intent.gimbal_yaw_ff = 0.0f;
  }

  // 4. 开火: OR 合并 (任一路触发都射)
  if (shoot_cmd->shoot_mode == SHOOT_ON && shoot_cmd->friction_mode == FRICTION_ON) {
    const uint8_t burst = ocd.js_shoot.burst || ocd.mk_shoot.burst;
    const uint8_t fire = ocd.js_shoot.fire || ocd.mk_shoot.fire;
    shoot_cmd->load_mode = burst ? LOAD_BURSTFIRE : fire ? LOAD_1_BULLET : LOAD_STOP;
  }

  // 5. 模式推导 (含 Jump FSM 状态映射, 读 update_flag.is_rotate)
  ApplyOcdMode(robot);

  // 5. 腿长预设: R/F 边沿挂起的绝对写入 (JS leg_length_delta 由 RobotMotionSolve 叠加)
  if (ocd.leg.pending) {
    chassis_cmd->leg_length = LEG_TABLE[ocd.leg.index];
    ocd.leg.pending = 0;
  }

  // 5b. 跳台阶模式 (Ctrl+V 后 JUMP_READY / JUMP_ACTIVE): 强制云台 pitch = 0 (水平),
  //     覆盖 MK 鼠标增量 / JS 摇杆增量 / 自瞄写入, 在 RobotMotionSolve 钳位之前生效.
  if (ocd.jump.phase == JUMP_READY || ocd.jump.phase == JUMP_ACTIVE) {
    gimbal_cmd->pitch = 0.0f;
  }

  // 5c. X 掉头反向跟随: 监视云台已转过角度, 达 REVERSE_FOLLOW_EXIT_DEG 后清除标志, 自动回正常 FOLLOW.
  if (ocd.reverse.active) {
    const float gimbal_delta = fabsf(robot->gimbal->gimbal_IMU_data->YawTotalAngle - ocd.reverse.start_yaw);
    if (gimbal_delta >= REVERSE_FOLLOW_EXIT_DEG) {
      ocd.reverse.active = 0;
    }
  }
  ocd.intent.reverse_follow = ocd.reverse.active;

  // 6. 解算运动 (rotate_scale 只推进 yaw 轨迹, 不再直写 wz)
  RobotMotionSolve(robot, &ocd.intent);

  // 7. Jump FSM 推进: ACTIVE 状态等 chassis->jump_state 回 IDLE 或 1.2s 超时
  if (ocd.jump.phase == JUMP_ACTIVE) {
    if (robot->chassis->jump_state != JUMP_STATE_IDLE) {
      ocd.jump.observed_active = 1;
    }
    const uint8_t state_done = ocd.jump.observed_active && robot->chassis->jump_state == JUMP_STATE_IDLE;
    const uint8_t timeout = DWT_GetTimeline_s() - ocd.jump.active_since > 1.2f;
    if (state_done || timeout) {
      ocd.jump.phase = JUMP_IDLE;
      ocd.jump.observed_active = 0;
      ApplyOcdMode(robot);  // 立即回正常模式, 不等下一帧
    }
  }

  // 8. 帧末快照: 供下一帧边沿检测
  rc_data_last = *rc_data;
}

void EmergencyHandler(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_cmd = &robot->shoot->shoot_ctrl_cmd;

  if (rc_data == NULL || switch_left(rc_data->rc.mode_switch)) {
    robot->robot_mode = ROBOT_POWER_OFF;
    gimbal_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    chassis_cmd->chassis_mode = CHASSIS_POWER_OFF;
    shoot_cmd->shoot_mode = SHOOT_OFF;
    shoot_cmd->friction_mode = FRICTION_OFF;
    shoot_cmd->load_mode = LOAD_STOP;
    ocd.jump.phase = JUMP_IDLE;
    ocd.jump.observed_active = 0;
    ResetChassisMotionMemory(robot);
    LOGERROR("[CMD] emergency stop!");
  } else if (IsRobotLostControl(robot) && chassis_cmd->chassis_mode != CHASSIS_PROSTRATE) {
    chassis_cmd->chassis_mode = CHASSIS_RECOVERY;
  }

  if (rc_data != NULL && switch_middle(rc_data->rc.mode_switch)) {
    shoot_cmd->shoot_mode = SHOOT_OFF;
    shoot_cmd->friction_mode = FRICTION_OFF;
    shoot_cmd->load_mode = LOAD_STOP;
  }
}
#endif

// ==========================================
// 逻辑层与输入层 (RC CTRL)
// ==========================================
#ifdef USE_RC_CTRL
void JoyStickCtrl(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  static float trigger_time = 0;

  if (update_flag.is_first_update) {
    rc_data_last = rc_data[TEMP];
    update_flag.is_first_update = 0;
  }

  Ctrl_Intent_s intent = {0};
  // 拨轮活跃时缩放系数 = dial/660 (保留符号, 决定转向); 仅按 shift 时取 +1
  const int16_t dial = rc_data[TEMP].rc.dial;
  const uint8_t is_rotate_mode = (abs(dial) > 20 || rc_data[TEMP].key[KEY_PRESS].shift);
  if (is_rotate_mode) {
    intent.rotate_scale = (abs(dial) > 20) ? (float)dial / 660.0f : 1.0f;
  }

  if (switch_is_mid(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_ON;
    if (is_rotate_mode) {
      robot->robot_mode = ROBOT_CHASSIS_ROTATE;
    } else {
      robot->robot_mode = ROBOT_CHASSIS_FOLLOW;
    }
  } else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_ON;
    if (is_rotate_mode) {
      robot->robot_mode = ROBOT_CHASSIS_ROTATE;
    } else {
      robot->robot_mode = ROBOT_CHASSIS_FREE;
    }

    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_JUMP_READY;
    }
    if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_JUMP_START;
      chassis_ctrl_cmd->jump_force = JUMP_FORCE;
    }
  } else {
    // Other mode?
  }

  if (!switch_is_up(rc_data[TEMP].rc.switch_right)) {
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
      shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
      shoot_ctrl_cmd->friction_mode = FRICTION_ON;
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
    } else if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
      shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
      shoot_ctrl_cmd->friction_mode = FRICTION_ON;
      if (switch_is_mid(rc_data_last.rc.switch_left)) {
        trigger_time = DWT_GetTimeline_s();
      }
      if (DWT_GetTimeline_s() - trigger_time > 0.5f) {
        shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
      } else {
        shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
      }
    }
  } else {
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }

  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
  float gimbal_yaw_delta = -0.00035f * (float)rc_data[TEMP].rc.rocker_r_;
  gimbal_ctrl_cmd->yaw += gimbal_yaw_delta;
  intent.gimbal_yaw_ff += CalcFollowTargetYawFeedforward(gimbal_yaw_delta);
  gimbal_ctrl_cmd->pitch += -0.00006f * (float)rc_data[TEMP].rc.rocker_r1;

  if (robot->robot_mode == ROBOT_CHASSIS_ROTATE) {
    intent.vx = 0.0025f * (float)rc_data[TEMP].rc.rocker_l_;
    intent.vy = 0.0025f * (float)rc_data[TEMP].rc.rocker_l1;
  } else if (robot->robot_mode == ROBOT_CHASSIS_FOLLOW) {
    intent.vx = 0.003f * (float)rc_data[TEMP].rc.rocker_l_;
    intent.vy = 0.003f * (float)rc_data[TEMP].rc.rocker_l1;
  } else if (robot->robot_mode == ROBOT_CHASSIS_FREE) {
    gimbal_ctrl_cmd->yaw += (-0.25f) * (float)rc_data[TEMP].rc.rocker_r_ * robot->dt * DEGREE_2_RAD;
    intent.vy = (0.004f) * (float)rc_data[TEMP].rc.rocker_r1;
    intent.roll_delta = 0.0004f * (float)rc_data[TEMP].rc.rocker_l_ * (abs(rc_data[TEMP].rc.rocker_l_) > 10);
    intent.leg_length_delta = 0.0000005f * (float)rc_data[TEMP].rc.rocker_l1;
  }

  RobotMotionSolve(robot, &intent);
  rc_data_last = rc_data[TEMP];
}

void MouseKeyCtrl(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  Vision_Receive_s* vision_recv_data = robot->vision_recv_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;

  if (update_flag.is_first_update) {
    rc_data_last = rc_data[TEMP];
    update_flag.is_first_update = 0;
  }

  static float trigger_time = 0;

  static uint8_t x_key_last = 0;
  static uint8_t f_key_last = 0;
  static uint8_t ctrl_g_key_last = 0;
  static uint8_t c_key_last = 0;

  Ctrl_Intent_s intent = {0};

  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;

  if (rc_data[TEMP].mouse.press_l) {
    if (shoot_ctrl_cmd->friction_mode == FRICTION_ON &&
        (vision_recv_data->shoot_receive.fire_flag || rc_data[TEMP].mouse.press_r % 2 == 0)) {
      if (DWT_GetTimeline_s() - trigger_time > 0.2f) {
        shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
      } else {
        shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
      }
    }
  } else {
    if (!switch_is_up(rc_data[TEMP].rc.switch_left)) {
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
      trigger_time = DWT_GetTimeline_s();
    }
  }

  if (rc_data[TEMP].mouse.press_r && has_non_zero_data(vision_recv_data)) {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_VISION;
    gimbal_ctrl_cmd->yaw = vision_recv_data->gimbal_receive.yaw;
    gimbal_ctrl_cmd->pitch = vision_recv_data->gimbal_receive.pitch;
  } else {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    float gimbal_yaw_delta = -(float)rc_data[TEMP].mouse.x * 0.002f;
    gimbal_ctrl_cmd->yaw += gimbal_yaw_delta;
    intent.gimbal_yaw_ff += CalcFollowTargetYawFeedforward(gimbal_yaw_delta);
    gimbal_ctrl_cmd->pitch += -(float)rc_data[TEMP].mouse.y * 0.002f;

    if (rc_data[TEMP].key[KEY_PRESS].x != x_key_last) {
      if (rc_data[TEMP].key[KEY_PRESS].x) {
        gimbal_ctrl_cmd->yaw += 180.0f;
      }
      x_key_last = rc_data[TEMP].key[KEY_PRESS].x;
    }
  }

  if (rc_data[TEMP].key[KEY_PRESS].ctrl && !rc_data_last.key[KEY_PRESS].ctrl) {
    RequestUiForceRefresh(robot);
  }

  if (abs(rc_data[TEMP].rc.dial) > 20 || rc_data[TEMP].key[KEY_PRESS].shift) {
    robot->robot_mode = ROBOT_CHASSIS_ROTATE;
  } else {
    robot->robot_mode = ROBOT_CHASSIS_FOLLOW;
  }

  if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].g != ctrl_g_key_last) {
    if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].g) {
      if (chassis_ctrl_cmd->chassis_mode != CHASSIS_RECOVERY) {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_RECOVERY;
      } else {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_ON;
      }
    }
    ctrl_g_key_last = rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].g;
  }

  if (rc_data[TEMP].key[KEY_PRESS].f != f_key_last) {
    if (rc_data[TEMP].key[KEY_PRESS].f) {
      // 切换飞坡参数 待添加...
    }
    f_key_last = rc_data[TEMP].key[KEY_PRESS].f;
  }

  if (rc_data[TEMP].key[KEY_PRESS].c != c_key_last) {
    if (rc_data[TEMP].key[KEY_PRESS].c) {
      if (robot->chassis->super_cap->super_cap_ctrl_cmd == BOOST) {
        robot->chassis->super_cap->super_cap_ctrl_cmd = NORMAL;
      } else {
        robot->chassis->super_cap->super_cap_ctrl_cmd = BOOST;
      }
    }
    c_key_last = rc_data[TEMP].key[KEY_PRESS].c;
  }

  if (rc_data[TEMP].key[KEY_PRESS].q) {
    intent.roll_delta = -0.4f;
  } else if (rc_data[TEMP].key[KEY_PRESS].e) {
    intent.roll_delta = 0.4f;
  } else {
    intent.roll_delta = 0.0f;
  }

  if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].r) {
    intent.leg_length_delta = 0.0000005f * 630.0f;
  } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].f) {
    intent.leg_length_delta = -0.0000005f * 630.0f;
  }

  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_RECOVERY || chassis_ctrl_cmd->chassis_mode == CHASSIS_JUMP_READY ||
      chassis_ctrl_cmd->chassis_mode == CHASSIS_JUMP_START) {
    robot->robot_mode = ROBOT_CHASSIS_FREE;

    if (robot->robot_mode == ROBOT_CHASSIS_FREE) {
      if (rc_data[TEMP].key[KEY_PRESS].w)
        intent.vy = (0.99f) * CTRL_SPEED_COFF;
      else if (rc_data[TEMP].key[KEY_PRESS].s)
        intent.vy = (-0.99f) * CTRL_SPEED_COFF;
      else
        intent.vy = 0.0f;
    } else {
      if (rc_data[TEMP].key[KEY_PRESS].w)
        intent.vy = 0.5f * CTRL_SPEED_COFF;
      else if (rc_data[TEMP].key[KEY_PRESS].s)
        intent.vy = -0.5f * CTRL_SPEED_COFF;
      else
        intent.vy = 0.0f;

      if (rc_data[TEMP].key[KEY_PRESS].d)
        intent.vx = 0.5f * CTRL_SPEED_COFF;
      else if (rc_data[TEMP].key[KEY_PRESS].a)
        intent.vx = -0.5f * CTRL_SPEED_COFF;
      else
        intent.vx = 0.0f;
    }

    float target_speed =
        (robot->robot_mode == ROBOT_CHASSIS_FREE) ? (0.99f * CTRL_SPEED_COFF) : (0.5f * CTRL_SPEED_COFF);
    float input_mag = sqrtf(intent.vx * intent.vx + intent.vy * intent.vy);
    if (input_mag > 1e-3f) {
      intent.vx = intent.vx / input_mag * target_speed;
      intent.vy = intent.vy / input_mag * target_speed;
    }
  }
  RobotMotionSolve(robot, &intent);
  rc_data_last = rc_data[TEMP];
}

void EmergencyHandler(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;

  if (IsRobotLostControl(robot)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_RECOVERY;
  }

  if ((switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left)) |
      switch_is_off(rc_data[TEMP].rc.switch_right)) {
    robot->robot_mode = ROBOT_POWER_OFF;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    ResetChassisMotionMemory(robot);
    LOGERROR("[CMD] emergency stop!");
  } else {
    LOGINFO("[CMD] reinstate, robot ready");
  }
  if (switch_is_down(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
  }
  if (switch_is_down(rc_data[TEMP].rc.switch_left)) {
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }
}

#endif