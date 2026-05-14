/**
 * @file ctrl.c
 * @brief 轮腿步兵的上层输入仲裁与模式控制。
 *
 * 控制链路分为三层：
 * - 输入采样：JoyStickCtrl / MouseKeyCtrl 只把摇杆、键鼠转换成临时意图；
 * - 意图仲裁：CtrlSolve 合并两路输入，并推进射击、自瞄、跳跃、腿长预设等离散状态；
 * - 指令落地：RobotMotionSolve 根据 robot_mode 写入 chassis_ctrl_cmd / gimbal_ctrl_cmd。
 *
 * 这里不做底盘速度斜坡、yaw 轨迹规划或 LQR 参考滤波；这些连续控制逻辑在 chassis 层
 * 按底盘本地时基执行。本文件输出的是 raw 目标，便于双板通信后由底盘侧统一平滑。
 */

#include "ctrl.h"

#include <math.h>

#include "general_def.h"
#include "robot_config.h"
#include "ui.h"

/* 当前编译只会启用一种输入链路。rc_data_last 用于按键边沿检测。 */
#ifdef USE_RC_CTRL
static RC_ctrl_t* rc_data;
static RC_ctrl_t rc_data_last;
#elifdef USE_OCD_CTRL
static VT13_RC_t* rc_data;
static VT13_RC_t rc_data_last;
#endif

/* 共享控制状态。RC 链路只使用 is_first_update，OCD 链路会使用全部字段。 */
static UpdateFlag_s update_flag = {.is_first_update = 1};

static CtrlInstance ocd = {
    .leg.index = 1,
    /* 速度配置。vx/stair/vault 为平移速度 m/s，wz 为小陀螺角速度 rad/s。 */
    .speed = {.vx = 2.5f, .wz = 15.0f, .stair = 2.2f, .vault = 1.8f},
    /* 倾倒恢复阈值。保留 default/creep 两套配置，便于后续按场景单独调参。 */
    .recovery = {.pitch_default = 13.0f, .pitch_creep = 13.0f},
};
/* 四档腿长预设：最低、上电默认、中档、最高。单位 m。 */
static const float LEG_TABLE[4] = {0.117f, 0.20f, 0.285f, 0.370f};

/** @brief 根据当前场景选择 Pitch 阈值，判断是否需要进入 CHASSIS_RECOVERY。 */
static uint8_t IsRobotLostControl(RobotInstance* robot) {
  const float thresh = robot->chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_STAIR ? ocd.recovery.pitch_creep
                                                                                      : ocd.recovery.pitch_default;
  return fabsf(robot->chassis->imu->Pitch) > thresh;
}

/** @brief 请求裁判 UI 做一次全量重绘。双板时通过通信字段转发到底盘侧。 */
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

/**
 * @brief 清空底盘运动相关的历史量。
 *
 * 用在急停、上下姿态切换和从 POWER_OFF 恢复等边沿。target_yaw 必须对齐当前 IMU yaw，
 * 否则 LQR 恢复时会吃到历史 yaw 误差并瞬间给出大输出。
 */
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

/**
 * @brief 在姿态大状态边沿自动清空运动历史。
 * @return 1 表示本帧执行过重置，调用方需要同步自己的局部参考量。
 */
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

/* ------------------------- 运动指令生成 ------------------------- */

/**
 * @brief 将最终控制意图转换为底盘 raw 指令。
 *
 * 本函数只根据当前 robot_mode 写入 target_yaw / vx / wz / roll / leg_length 等目标。
 * 连续量的平滑和限幅由 chassis 层完成。
 */
static void RobotMotionSolve(RobotInstance* robot, Ctrl_Intent_s* intent) {
  static float yaw_ref = 0.0f;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  if (ResetMotionMemoryOnPostureEdge(robot)) {
    yaw_ref = chassis_ctrl_cmd->target_yaw;
  }
  float input_mag = sqrtf(intent->vx * intent->vx + intent->vy * intent->vy);

  /* 只有站立平衡小陀螺需要通知 LQR 使用旋转状态估计；趴下旋转走 ChassisProstrate。 */
  chassis_ctrl_cmd->is_rotate = (robot->robot_mode == ROBOT_CHASSIS_ROTATE) ? 1 : 0;

  chassis_ctrl_cmd->roll = 0.0f;
  /* roll 前馈补偿底盘固有偏置和云台质心旋转带来的侧倾力矩。 */
  chassis_ctrl_cmd->roll_ff =
      ROLL_FF_BIAS + ROLL_FF_AMP * sinf((GIMBAL_COM_ANGLE_DEG - robot->offset_angle) * DEGREE_2_RAD);

  switch (robot->robot_mode) {
    case ROBOT_CHASSIS_ROTATE: {
      /* 小陀螺模式下 target_yaw 跟随当前 yaw，旋转只由 wz 前馈推进，退出时不会残留 yaw 误差。 */
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
      /* 掉头反向跟随时底盘不转身，只把前进方向反过来，等待云台完成 180 度转向。 */
      if (intent->reverse_follow || fabsf(rear_err) < fabsf(follow_err)) {
        follow_err = rear_err;
        if (has_move_input) input_mag = -input_mag;
      }
      if (intent->reverse_follow) {
        follow_err = 0.0f;
      }
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
      yaw_ref = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + (follow_err + intent->gimbal_yaw_ff) * DEGREE_2_RAD;
#endif
      chassis_ctrl_cmd->target_yaw = yaw_ref;  // raw, chassis planner 平滑
      /* FREE 只给 yaw 位置目标，不额外叠加当前 gyro 前馈。 */
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
      /* 趴下小陀螺禁用 yaw 闭环位置误差，避免 PID 抵消持续旋转前馈。 */
      chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD;
      yaw_ref = chassis_ctrl_cmd->target_yaw;
      chassis_ctrl_cmd->wz = intent->rotate_scale * ocd.speed.wz;  // rad/s, chassis 层换算
      break;
    }
    case ROBOT_CHASSIS_PROSTRATE_FOLLOW: {
#if (!defined(ONE_BOARD))
      chassis_ctrl_cmd->roll = 0.0f;
      chassis_ctrl_cmd->wz = 0.0f;
      /* 趴下跟随仍使用 m/s 量级输入，底盘层再换算到卧倒轮速。 */
      const uint8_t has_move_input = input_mag > 0.0005f;
      float move_angle_deg = has_move_input ? (atan2f(intent->vy, intent->vx) - PI / 2.0f) * RAD_2_DEGREE : 0.0f;
      float follow_err = wrap180(move_angle_deg - robot->offset_angle);
      float rear_err = wrap180(follow_err - 180.0f);
      if (fabsf(rear_err) < fabsf(follow_err)) {
        follow_err = rear_err;
        if (has_move_input) input_mag = -input_mag;
      }
      yaw_ref = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + (follow_err + intent->gimbal_yaw_ff) * DEGREE_2_RAD;
      chassis_ctrl_cmd->target_yaw = yaw_ref;

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
  if (robot->robot_mode == ROBOT_CHASSIS_ROTATE || robot->robot_mode == ROBOT_CHASSIS_PROSTRATE_ROTATE) {
    gimbal_ctrl_cmd->chassis_rotate_wz = -0.25f * robot->chassis->imu->Gyro[2];
  } else {
    gimbal_ctrl_cmd->chassis_rotate_wz = 0.0f;
  }

  PIDSwitchConfig(&robot->gimbal->yaw_motor->motor_controller.angle_PID, gimbal_ctrl_cmd->gimbal_mode == GIMBAL_VISION
                                                                             ? &yaw_angle_PID_vision_config
                                                                             : &yaw_angle_PID_manual_config);

  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE)
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE)
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
}

/* ------------------------- OCD 图传链路 ------------------------- */

#ifdef USE_OCD_CTRL

/**
 * @brief 根据持久状态和跳跃状态机推导 robot_mode / chassis_mode。
 *
 * OCD 链路下模式集中在这里写入。RECOVERY 期间直接返回，避免覆盖云台对齐流程写入的
 * CHASSIS_RECOVERY / CHASSIS_ON。
 */
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

  cmd->chassis_mode = update_flag.is_stand ? (ocd.stair.active ? CHASSIS_STAIR : CHASSIS_ON) : CHASSIS_PROSTRATE;

  /*
   * 模式优先级：rotate > free > follow。
   * 趴下没有 LQR 平衡和腿长/roll 微调需求，因此不进入 FREE，统一走 PROSTRATE_FOLLOW。
   */
  if (update_flag.is_rotate) {
    robot->robot_mode =
        update_flag.is_stand || update_flag.is_free ? ROBOT_CHASSIS_ROTATE : ROBOT_CHASSIS_PROSTRATE_ROTATE;
  } else if (update_flag.is_free && update_flag.is_stand) {
    robot->robot_mode = ROBOT_CHASSIS_FREE;
  } else {
    robot->robot_mode = update_flag.is_stand ? ROBOT_CHASSIS_FOLLOW : ROBOT_CHASSIS_PROSTRATE_FOLLOW;
  }
}

/**
 * @brief 采样图传遥控器摇杆域输入。
 *
 * 本函数只写 ocd.js_* 和少量持久状态，不直接做最终模式仲裁。键鼠域会在
 * MouseKeyCtrl 中独立采样，最后由 CtrlSolve 合并。
 */
void JoyStickCtrl(RobotInstance* robot) {
  rc_data = robot->rc_data;
  if (update_flag.is_first_update) {
    rc_data_last = *rc_data;
    update_flag.is_first_update = 0;
  }

  /* 每帧清空瞬时摇杆意图，跳跃和腿长预设等持久状态保留。 */
  ocd.intent = (Ctrl_Intent_s){0};
  ocd.js_vx = 0.0f;
  ocd.js_vy = 0.0f;
  ocd.js_yaw_ff = 0.0f;
  ocd.js_rotate_scale = 0.0f;
  ocd.js_shoot = (ShootReq_s){0};

  Chassis_Ctrl_Cmd_s* chassis_cmd = &robot->chassis->chassis_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_cmd = &robot->shoot->shoot_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_cmd = &robot->gimbal->gimbal_ctrl_cmd;

  /* 左拨到底视为急停姿态；该状态也会约束键鼠侧的 toggle。 */
  update_flag.is_on = !switch_left(rc_data->rc.mode_switch);

  /* fn_1 边沿切换站立/趴下；跳跃、恢复、急停期间禁止切换。 */
  const uint8_t fn_1_edge = rc_data->rc.fn_1 && !rc_data_last.rc.fn_1;
  if (fn_1_edge && update_flag.is_on && chassis_cmd->chassis_mode != CHASSIS_RECOVERY &&
      ocd.jump.phase != JUMP_ACTIVE) {
    ocd.jump.phase = JUMP_IDLE;
    ocd.jump.observed_active = 0;
    update_flag.is_stand = !update_flag.is_stand;
  }

  /* pause 边沿切换 FREE 模式。 */
  const uint8_t pause_edge = rc_data->rc.pause && !rc_data_last.rc.pause;
  if (pause_edge && update_flag.is_on) {
    update_flag.is_free = !update_flag.is_free;
  }

  /* 恢复流程期间清空跳跃状态，避免对齐完成后继续执行旧的跳跃请求。 */
  if (chassis_cmd->chassis_mode == CHASSIS_RECOVERY) {
    ocd.jump.phase = JUMP_IDLE;
    ocd.jump.observed_active = 0;
  }

  /* 拨轮给出带符号的小陀螺缩放系数。 */
  if (abs(rc_data->rc.dial) > 20) {
    ocd.js_rotate_scale = (float)rc_data->rc.dial / 660.0f;
  }

  /* 右拨杆开启发射系统；扳机短按单发，长按连发。 */
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

  /* 手动云台增量；如果本帧进入自瞄，CtrlSolve 会覆盖 yaw/pitch。 */
  gimbal_cmd->gimbal_mode = GIMBAL_ON;
  const float yaw_delta = -0.00035f * (float)rc_data->rc.rocker_r_;
  gimbal_cmd->yaw += yaw_delta;
  ocd.js_yaw_ff = yaw_delta * 30.0f;

  if (!(update_flag.is_stand && update_flag.is_free)) {
    gimbal_cmd->pitch += -0.00006f * (float)rc_data->rc.rocker_r1;
  }

  /* 摇杆平移统一输出 m/s，趴下模式由 chassis 层换算为轮速指令。 */
  if (update_flag.is_stand && update_flag.is_free) {
    /* FREE 模式保留右摇杆前进，左摇杆用于 roll / 腿长微调。 */
    ocd.js_vy = 0.003f * (float)rc_data->rc.rocker_r1;
    ocd.intent.roll_delta = 0.0003f * (float)rc_data->rc.rocker_l_ * (abs(rc_data->rc.rocker_l_) > 10);
    ocd.intent.leg_length_delta = 0.0000005f * (float)rc_data->rc.rocker_l1;
  } else {
    /* FOLLOW/ROTATE 使用左摇杆平移；ROTATE 下 vx/vy 最终会被忽略。 */
    ocd.js_vx = 0.003f * (float)rc_data->rc.rocker_l_;
    ocd.js_vy = 0.003f * (float)rc_data->rc.rocker_l1;
  }
}

/**
 * @brief 采样图传键鼠域输入。
 *
 * 键鼠域负责 WASD、鼠标云台、开火、自瞄、跳跃、腿长预设和 UI 刷新等事件。
 * 这里只记录请求，不直接决定最终底盘模式。
 */
void MouseKeyCtrl(RobotInstance* robot) {
  rc_data = robot->rc_data;

  /* 每帧清空瞬时键鼠意图，跳跃/腿长/掉头等持久状态保留。 */
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

  /* Shift 按住请求正向小陀螺；若拨轮也活跃，CtrlSolve 优先使用拨轮方向。 */
  if (mk->keyboard.shift) {
    ocd.mk_rotate_scale = 1.0f;
  }

  /* 鼠标左键发射：短按单发，按住超过 0.3s 连发。 */
  static float mouse_trigger_t0 = 0;
  if (mk->mouse.press_l) {
    if (shoot_cmd->friction_mode == FRICTION_ON) {
      ocd.mk_shoot.fire = 1;
      ocd.mk_shoot.burst = (DWT_GetTimeline_s() - mouse_trigger_t0 > 0.3f) ? 1 : 0;
    }
  } else {
    mouse_trigger_t0 = DWT_GetTimeline_s();
  }

  /* 右键且视觉数据有效时请求自瞄。 */
  if (mk->mouse.press_r && has_non_zero_data(vision)) {
    ocd.mk_vision = 1;
  }

  /* 鼠标增量直接积分到云台目标；自瞄生效时 CtrlSolve 会覆盖。 */
  const float yaw_delta = -(float)mk->mouse.x * 0.002f;
  gimbal_cmd->yaw += yaw_delta;
  ocd.mk_yaw_ff = yaw_delta * 10.0f;
  gimbal_cmd->pitch += -(float)mk->mouse.y * 0.002f;

  /* X：云台 180 度掉头，同时临时启用反向跟随。 */
  if (mk->keyboard.x && !mk_last->keyboard.x) {
    gimbal_cmd->yaw += 180.0f;
    ocd.reverse.active = 1;
    ocd.reverse.start_yaw = robot->gimbal->gimbal_IMU_data->YawTotalAngle;
  }

  /* Z：切换蹭台阶模式。进入时升到最高腿长并降低 WASD 速度，退出时恢复原腿长档。 */
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

  /* B：强制刷新裁判 UI。 */
  if (mk->keyboard.b && !mk_last->keyboard.b) {
    RequestUiForceRefresh(robot);
  }

  /* C：切换超级电容控制命令。 */
  if (mk->keyboard.c && !mk_last->keyboard.c) {
    robot->chassis->super_cap->super_cap_ctrl_cmd =
        (robot->chassis->super_cap->super_cap_ctrl_cmd == BOOST) ? NORMAL : BOOST;
  }

  /* Q/E：持续微调 roll。 */
  if (mk->keyboard.q)
    ocd.intent.roll_delta -= 0.4f;
  else if (mk->keyboard.e)
    ocd.intent.roll_delta += 0.4f;

  /* R/F：切换四档腿长预设，实际写入由 CtrlSolve 消费 pending 标志。 */
  if (mk->keyboard.r && !mk_last->keyboard.r) {
    if (ocd.leg.index < 3) ocd.leg.index++;
    ocd.leg.pending = 1;
  }
  if (mk->keyboard.f && !mk_last->keyboard.f) {
    if (ocd.leg.index > 0) ocd.leg.index--;
    ocd.leg.pending = 1;
  }

  /* Ctrl+G：切换站立/趴下；跳跃、恢复、急停期间锁定。 */
  const uint8_t ctrl_g = mk->keyboard.ctrl && mk->keyboard.g;
  const uint8_t ctrl_g_last = mk_last->keyboard.ctrl && mk_last->keyboard.g;
  if (ctrl_g && !ctrl_g_last && update_flag.is_on && chassis_cmd->chassis_mode != CHASSIS_RECOVERY &&
      ocd.jump.phase != JUMP_ACTIVE) {
    ocd.jump.phase = JUMP_IDLE;
    ocd.jump.observed_active = 0;
    update_flag.is_stand = !update_flag.is_stand;
  }

  /* Ctrl+V 进入/退出跳跃准备；准备状态下单按 V 起跳。 */
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

  /* WASD 平移速度优先级：跳台阶限速 > 蹭台阶限速 > 默认速度。 */
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

/**
 * @brief 合并摇杆和键鼠意图，并推进一次上层控制状态。
 *
 * 仲裁原则：
 * - 平移输入：摇杆优先，摇杆回中后键鼠接管；
 * - yaw 前馈：摇杆和鼠标可以叠加；
 * - 发射：任一路触发都有效；
 * - 自瞄、跳跃、腿长预设：只由键鼠事件驱动，避免重复触发。
 */
void CtrlSolve(RobotInstance* robot) {
  Chassis_Ctrl_Cmd_s* chassis_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_cmd = &robot->shoot->shoot_ctrl_cmd;

  /* 1. 平移输入仲裁。 */
  const uint8_t js_owns = (fabsf(ocd.js_vx) > 1e-3f) || (fabsf(ocd.js_vy) > 1e-3f);
  ocd.intent.vx = js_owns ? ocd.js_vx : ocd.mk_vx;
  ocd.intent.vy = js_owns ? ocd.js_vy : ocd.mk_vy;

  /* 2. 小陀螺仲裁。拨轮带方向，优先级高于键盘 Shift。 */
  const uint8_t js_rotate_active = fabsf(ocd.js_rotate_scale) > 0.0f;
  ocd.intent.rotate_scale = js_rotate_active ? ocd.js_rotate_scale : ocd.mk_rotate_scale;
  update_flag.is_rotate = (fabsf(ocd.intent.rotate_scale) > 0.0f);

  /* 3. 云台 yaw 前馈。自瞄生效时覆盖云台目标并清空底盘前馈。 */
  ocd.intent.gimbal_yaw_ff = ocd.js_yaw_ff + ocd.mk_yaw_ff;
  if (ocd.mk_vision) {
    gimbal_cmd->gimbal_mode = GIMBAL_VISION;
    gimbal_cmd->yaw = robot->vision_recv_data->gimbal_receive.yaw;
    gimbal_cmd->pitch = robot->vision_recv_data->gimbal_receive.pitch;
    ocd.intent.gimbal_yaw_ff = 0.0f;
  }

  /* 4. 发射请求合并。 */
  if (shoot_cmd->shoot_mode == SHOOT_ON && shoot_cmd->friction_mode == FRICTION_ON) {
    const uint8_t burst = ocd.js_shoot.burst || ocd.mk_shoot.burst;
    const uint8_t fire = ocd.js_shoot.fire || ocd.mk_shoot.fire;
    shoot_cmd->load_mode = burst ? LOAD_BURSTFIRE : fire ? LOAD_1_BULLET : LOAD_STOP;
  }

  /* 5. 根据持久状态和跳跃 FSM 写入 robot_mode / chassis_mode。 */
  ApplyOcdMode(robot);

  /* 6. 消费腿长预设事件。摇杆腿长微调稍后在 RobotMotionSolve 中叠加。 */
  if (ocd.leg.pending) {
    chassis_cmd->leg_length = LEG_TABLE[ocd.leg.index];
    ocd.leg.pending = 0;
  }

  /* 7. 跳跃准备/执行期间强制云台水平，避免鼠标或自瞄覆盖 pitch。 */
  if (ocd.jump.phase == JUMP_READY || ocd.jump.phase == JUMP_ACTIVE) {
    gimbal_cmd->pitch = 0.0f;
  }

  /* 8. 掉头反向跟随到角度后自动退出。 */
  if (ocd.reverse.active) {
    const float gimbal_delta = fabsf(robot->gimbal->gimbal_IMU_data->YawTotalAngle - ocd.reverse.start_yaw);
    if (gimbal_delta >= REVERSE_FOLLOW_EXIT_DEG) {
      ocd.reverse.active = 0;
    }
  }
  ocd.intent.reverse_follow = ocd.reverse.active;

  /* 9. 将最终意图落到底盘 raw 指令。 */
  RobotMotionSolve(robot, &ocd.intent);

  /* 10. 跳跃 FSM 收尾：等待底盘跳跃状态回 IDLE，或超时保护退出。 */
  if (ocd.jump.phase == JUMP_ACTIVE) {
    if (robot->chassis->jump_state != JUMP_STATE_IDLE) {
      ocd.jump.observed_active = 1;
    }
    const uint8_t state_done = ocd.jump.observed_active && robot->chassis->jump_state == JUMP_STATE_IDLE;
    const uint8_t timeout = DWT_GetTimeline_s() - ocd.jump.active_since > 1.2f;
    if (state_done || timeout) {
      ocd.jump.phase = JUMP_IDLE;
      ocd.jump.observed_active = 0;
      ApplyOcdMode(robot);  // 立即回正常模式，不等下一帧。
    }
  }

  /* 11. 保存输入快照，供下一帧边沿检测。 */
  rc_data_last = *rc_data;
}

/**
 * @brief 处理急停、倾倒恢复和发射禁用。
 *
 * 急停会直接断开底盘、云台、发射输出并清空跳跃状态。倾倒检测只把底盘切到
 * CHASSIS_RECOVERY，让下层恢复流程接管。
 */
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

/* ------------------------- 传统遥控器链路 ------------------------- */

#ifdef USE_RC_CTRL

/**
 * @brief 传统遥控器链路的摇杆控制。
 *
 * 该分支保留旧遥控器输入方式，当前工程默认使用 OCD 图传链路。
 */
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
  /* 拨轮给出带方向的小陀螺缩放；仅按 Shift 时使用正向满速。 */
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

/**
 * @brief 传统遥控器链路下的键鼠辅助控制。
 */
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
      /* 预留：切换飞坡参数。 */
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

/**
 * @brief 传统遥控器链路下的急停与恢复处理。
 */
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
