#include "ctrl.h"

#include <math.h>

#include "general_def.h"
#include "robot_config.h"

// Static variables for control state
#ifdef USE_RC_CTRL
static RC_ctrl_t* rc_data;
static RC_ctrl_t rc_data_last;
#elifdef USE_OCD_CTRL
static VT13_RC_t* rc_data;
static VT13_RC_t rc_data_last;
#endif

static uint8_t is_first_update = 1;

Ramp_Controller_t vx_ramp = {
    .planning_v = 0.0f,
    .max_v = 15.0f,
    .max_accel = 1.0f,
    .min_accel = 0.05f,
    .accel_base_speed = 0.3f,
    .max_decel = 3.7f,
    .min_decel = 1.0f,
    .decel_base_speed = 0.3f,
    .k_p_vel = 0.35f,
};

Ramp_Controller_t wz_ramp = {
    .planning_v = 0.0f,
    .max_v = 15.0f,
    .max_accel = 3.0f,
    .min_accel = 3.0f,
    .accel_base_speed = 1.3f,
    .max_decel = 3.0f,
    .min_decel = 3.0f,
    .decel_base_speed = 1.3f,
    .k_p_vel = 0.35f,
};

// ==========================================
// 统一的机器人运动解算层
// ==========================================
static void RobotMotionSolve(RobotInstance* robot, Ctrl_Intent_s* intent) {
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  float input_mag = sqrtf(intent->vx * intent->vx + intent->vy * intent->vy);

  // 小陀螺标志: 仅 LQR 平衡态下的 ROTATE 才使能 (PROSTRATE_ROTATE 走 ChassisProstrateMode, 不进 LQR)
  chassis_ctrl_cmd->is_rotate = (robot->robot_mode == ROBOT_CHASSIS_ROTATE) ? 1 : 0;

  chassis_ctrl_cmd->roll = 0.0f;
  // roll 前馈 = 底盘恒定偏置 + 云台 CoM 旋转分量. offset_angle 俯视 CW 为正 → 底盘系下 CoM 角 = α_g - offset_angle.
  chassis_ctrl_cmd->roll_ff =
      ROLL_FF_BIAS + ROLL_FF_AMP * sinf((GIMBAL_COM_ANGLE_DEG - robot->offset_angle) * DEGREE_2_RAD);

  switch (robot->robot_mode) {
    case ROBOT_CHASSIS_ROTATE: {
      // 小陀螺频率设置
      float rotate_frequency = 6.0f;
      // 小陀螺原地旋转
      float rotate_omega = rotate_frequency * 2.0f * PI;
      // chassis_ctrl_cmd->target_yaw += rotate_omega * robot->dt;
      chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD;
      // chassis_ctrl_cmd->wz = ramp_controller_update(&wz_ramp, rotate_omega, robot->chassis->imu->Gyro[2], robot->dt);
      chassis_ctrl_cmd->wz = rotate_omega;;
      chassis_ctrl_cmd->vx = 0.0f;
      break;
    }
    case ROBOT_CHASSIS_FOLLOW: {
#if (!defined(ONE_BOARD))
      const uint8_t has_move_input = input_mag > 0.0005f;
      float move_angle_deg = has_move_input ? (atan2f(intent->vy, intent->vx) - PI / 2.0f) * RAD_2_DEGREE : 0.0f;
      float follow_err = wrap180(move_angle_deg - robot->offset_angle);
      float rear_err = wrap180(follow_err - 180.0f);
      if (fabsf(rear_err) < fabsf(follow_err)) {
        follow_err = rear_err;
        if (has_move_input) input_mag = -input_mag;
      }
      chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + follow_err * DEGREE_2_RAD;
      // 跟随当前机体角速度 → state_err[3]=0, LQR d_phi 项不参与输出, 仅由 yaw 位置误差驱动转向
      // chassis_ctrl_cmd->wz = robot->chassis->imu->Gyro[2];
      chassis_ctrl_cmd->wz = 0.0f;

      VAL_LIMIT(input_mag, -2.97f, 2.97f);

      float align_attenuation = cosf(follow_err * DEGREE_2_RAD);
      if (align_attenuation < 0) align_attenuation = 0;
      input_mag *= align_attenuation * align_attenuation * align_attenuation;

      chassis_ctrl_cmd->vx =
          ramp_controller_update(&vx_ramp, input_mag, robot->chassis->state_var.x_b_d, robot->dt);
      chassis_ctrl_cmd->theta_ff = 0.0f;
      break;
#endif
    }
    case ROBOT_CHASSIS_FREE: {
      float input_vy = intent->vy;
#if defined(ONE_BOARD)
      chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD;
#else
      float follow_err = wrap180(-robot->offset_angle);
      float rear_err = wrap180(follow_err - 180.0f);
      if (fabsf(rear_err) < fabsf(follow_err)) {
        follow_err = rear_err;
        input_vy = -input_vy;
      }
      chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + follow_err * DEGREE_2_RAD;
#endif
      // 与 FOLLOW 一致: FREE 只用 target_yaw 位置误差收敛, 不叠加当前 gyro 前馈.
      chassis_ctrl_cmd->wz = 0.0f;
      chassis_ctrl_cmd->vx =
          ramp_controller_update(&vx_ramp, input_vy, robot->chassis->state_var.x_b_d, robot->dt);
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
      chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD;
      chassis_ctrl_cmd->vx = 0.0f;
      chassis_ctrl_cmd->wz = 800.0f;
      break;
    }
    case ROBOT_CHASSIS_PROSTRATE_FOLLOW: {
#if (!defined(ONE_BOARD))
      chassis_ctrl_cmd->roll = 0.0f;
      chassis_ctrl_cmd->wz = 0.0f;
      const uint8_t has_move_input = input_mag > 5.0f;
      float move_angle_deg = has_move_input ? (atan2f(intent->vy, intent->vx) - PI / 2.0f) * RAD_2_DEGREE : 0.0f;
      float follow_err = wrap180(move_angle_deg - robot->offset_angle);
      float rear_err = wrap180(follow_err - 180.0f);
      if (fabsf(rear_err) < fabsf(follow_err)) {
        follow_err = rear_err;
        if (has_move_input) input_mag = -input_mag;
      }
      chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + follow_err * DEGREE_2_RAD;
      if (has_move_input) chassis_ctrl_cmd->wz = follow_err;  // 目标 yaw 角速度(rad/s) 前馈

      VAL_LIMIT(input_mag, -800.f, 800.f);

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
// 逻辑层与输入层 (RC CTRL)
// ==========================================
#ifdef USE_RC_CTRL
void JoyStickCtrl(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  static float trigger_time = 0;

  if (is_first_update) {
    rc_data_last = rc_data[TEMP];
    is_first_update = 0;
  }

  Ctrl_Intent_s intent = {0};

  if (switch_is_mid(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_ON;
    if (abs(rc_data[TEMP].rc.dial) > 20 || rc_data[TEMP].key[KEY_PRESS].shift) {
      robot->robot_mode = ROBOT_CHASSIS_ROTATE;
    } else {
      robot->robot_mode = ROBOT_CHASSIS_FOLLOW;
    }
  } else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_ON;
    if (abs(rc_data[TEMP].rc.dial) > 20 || rc_data[TEMP].key[KEY_PRESS].shift) {
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
  gimbal_ctrl_cmd->yaw += -0.00035f * (float)rc_data[TEMP].rc.rocker_r_;
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

  if (is_first_update) {
    rc_data_last = rc_data[TEMP];
    is_first_update = 0;
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
    gimbal_ctrl_cmd->yaw += -(float)rc_data[TEMP].mouse.x * 0.002f;
    gimbal_ctrl_cmd->pitch += -(float)rc_data[TEMP].mouse.y * 0.002f;

    if (rc_data[TEMP].key[KEY_PRESS].x != x_key_last) {
      if (rc_data[TEMP].key[KEY_PRESS].x) {
        gimbal_ctrl_cmd->yaw += 180.0f;
      }
      x_key_last = rc_data[TEMP].key[KEY_PRESS].x;
    }
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

  if (robot_lost_control) {
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
    vx_ramp.planning_v = 0.0f;
    vx_ramp.expected_a = 0.0f;
    wz_ramp.planning_v = 0.0f;
    wz_ramp.expected_a = 0.0f;
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

// ==========================================
// 逻辑层与输入层 (OCD CTRL)
// ==========================================
#elifdef USE_OCD_CTRL

// ---------- 两路 Ctrl 共享状态 ----------
// JS 与 MK 各自往 intent_shared / *_fire / *_burst 写, 由 CtrlSolve 合并.
static Ctrl_Intent_s intent_shared;
static uint8_t joystick_fire = 0, joystick_burst = 0;
static uint8_t mouse_fire = 0, mouse_burst = 0;
static uint8_t mk_request_vision = 0;
static uint8_t mk_set_leg_length = 0;
static float   mk_leg_length_target = 0.0f;
static uint8_t mk_jump_ready = 0;
static uint8_t mk_jump_started = 0;
static uint8_t mk_jump_started_seen_active = 0;
static float mk_jump_started_time = 0.0f;
static uint8_t mk_stand_mode = 0;

// 三档腿长: LOW=initial_leg_length, MID=(MAX+initial)/2, HIGH=LEG_MAX_LENGTH
static const float LEG_TABLE[3] = {0.20f, 0.285f, 0.370f};
static int leg_step = 0;

static uint8_t OcdIsStand(void) {
  return mk_stand_mode;
}

static void ApplyOcdNormalMode(RobotInstance* robot) {
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  const uint8_t is_stand = OcdIsStand();
  const uint8_t is_rotate = (abs(rc_data->rc.dial) > 20 || rc_data->mouse_key.keyboard.shift);
  const uint8_t is_free = (rc_data->button_status.pause_flag == 1);

  chassis_ctrl_cmd->chassis_mode = is_stand ? CHASSIS_ON : CHASSIS_PROSTRATE;

  if (is_free) {
    robot->robot_mode = ROBOT_CHASSIS_FREE;
  } else if (is_stand) {
    robot->robot_mode = is_rotate ? ROBOT_CHASSIS_ROTATE : ROBOT_CHASSIS_FOLLOW;
  } else {
    robot->robot_mode = is_rotate ? ROBOT_CHASSIS_PROSTRATE_ROTATE : ROBOT_CHASSIS_PROSTRATE_FOLLOW;
  }
}

void JoyStickCtrl(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  static float trigger_time = 0;
  if (is_first_update) {
    rc_data_last = *rc_data;
    is_first_update = 0;
  }

  // 帧起始: 共享 intent 与 fire flag 复位 (rc_data_last 在 CtrlSolve 末尾更新)
  intent_shared = (Ctrl_Intent_s){0};
  joystick_fire = 0;
  joystick_burst = 0;

  // 状态机 (按当前输入纯映射):
  //   fn_1       (rising edge): toggles local stand/prostrate mode, initial=prostrate.
  //   pause_flag (toggle):           1=FREE 模式
  //   dial / shift:                  ROTATE
  // RECOVERY 期间不覆盖 chassis_mode 也不刷新 robot_mode,
  // 让 GimbalAlignToChassisForward() 走完对齐流程, 避免与正常控制冲突;
  // 对齐完成后由 GimbalAlign 一次性置为 CHASSIS_ON, 此处恢复正常接管.
  const uint8_t fn_1_pressed = rc_data->rc.fn_1 && !rc_data_last.rc.fn_1;
  if (fn_1_pressed && chassis_ctrl_cmd->chassis_mode != CHASSIS_RECOVERY && !mk_jump_started) {
    mk_jump_ready = 0;
    mk_jump_started_seen_active = 0;
    mk_stand_mode = !mk_stand_mode;
  }
  const uint8_t is_stand = OcdIsStand();
  const uint8_t is_free = (rc_data->button_status.pause_flag == 1);

  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_RECOVERY) {
    mk_jump_ready = 0;
    mk_jump_started = 0;
    mk_jump_started_seen_active = 0;
  } else {
    if (mk_jump_started) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_JUMP_START;
      chassis_ctrl_cmd->jump_force = JUMP_FORCE;
      robot->robot_mode = ROBOT_CHASSIS_FREE;
    } else if (mk_jump_ready) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_JUMP_READY;
      robot->robot_mode = ROBOT_CHASSIS_FOLLOW;
    } else {
      ApplyOcdNormalMode(robot);
    }
  }

  if (switch_right(rc_data->rc.mode_switch)) {
    shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
    shoot_ctrl_cmd->friction_mode = FRICTION_ON;
    static uint8_t trigger_last = 0;
    if (trigger_last == 0 && rc_data->rc.trigger == 1) {
      trigger_time = DWT_GetTimeline_s();
    }
    trigger_last = rc_data->rc.trigger;
    if (rc_data->rc.trigger == 1) {
      joystick_fire = 1;
      joystick_burst = (DWT_GetTimeline_s() - trigger_time > 1.0f) ? 1 : 0;
    }
  } else {
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }

  // 设置控制量
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
  gimbal_ctrl_cmd->yaw += -0.00035f * (float)rc_data->rc.rocker_r_;
  if (!(is_stand && is_free)) {
    gimbal_ctrl_cmd->pitch += -0.00006f * (float)rc_data->rc.rocker_r1;
  }

  if (is_stand && is_free) {
    intent_shared.vy = 0.003f * (float)rc_data->rc.rocker_r1;
    intent_shared.roll_delta = 0.0003f * (float)rc_data->rc.rocker_l_ * (abs(rc_data->rc.rocker_l_) > 10);
    intent_shared.leg_length_delta = 0.0000005f * (float)rc_data->rc.rocker_l1;
  } else if (robot->robot_mode == ROBOT_CHASSIS_FOLLOW) {
    intent_shared.vx = 0.003f * (float)rc_data->rc.rocker_l_;
    intent_shared.vy = 0.003f * (float)rc_data->rc.rocker_l1;
  } else if (robot->robot_mode == ROBOT_CHASSIS_PROSTRATE_FOLLOW) {
    intent_shared.vx = (float)rc_data->rc.rocker_l_;
    intent_shared.vy = (float)rc_data->rc.rocker_l1;
  } else if (!is_stand && is_free) {
    intent_shared.vy = (float)rc_data->rc.rocker_l1;
    intent_shared.vx = (float)rc_data->rc.rocker_l_;
  }
  // RobotMotionSolve / free+rotate 后处理 / rc_data_last 更新均移至 CtrlSolve()
}

void MouseKeyCtrl(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  Vision_Receive_s* vision_recv_data = robot->vision_recv_data;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;

  static float mouse_trigger_time = 0;

  // 帧起始: MK 输出 flag 复位 (intent_shared 已由 JS 清零)
  mouse_fire = 0;
  mouse_burst = 0;
  mk_request_vision = 0;

  // 鼠标左键开火 -> 仅设 flag, 由 CtrlSolve OR 合并
  if (rc_data->mouse_key.mouse.press_l) {
    if (shoot_ctrl_cmd->friction_mode == FRICTION_ON) {
      mouse_fire = 1;
      mouse_burst = (DWT_GetTimeline_s() - mouse_trigger_time > 0.3f) ? 1 : 0;
    }
  } else {
    mouse_trigger_time = DWT_GetTimeline_s();
  }

  // 鼠标右键自瞄请求 -> 仅设 flag, CtrlSolve 中应用
  if (rc_data->mouse_key.mouse.press_r && has_non_zero_data(vision_recv_data)) {
    mk_request_vision = 1;
  }

  // 鼠标手动云台增量 (与 JS 摇杆 += 加性叠加; 自瞄激活时 CtrlSolve 会覆盖)
  gimbal_ctrl_cmd->yaw += -(float)rc_data->mouse_key.mouse.x * 0.002f;
  gimbal_ctrl_cmd->pitch += -(float)rc_data->mouse_key.mouse.y * 0.002f;

  // X 边沿: 云台 +180 度掉头
  if (rc_data->mouse_key.keyboard.x && !rc_data_last.mouse_key.keyboard.x) {
    gimbal_ctrl_cmd->yaw += 180.0f;
  }

  // C 边沿: 超级电容 BOOST/NORMAL 切换
  if (rc_data->mouse_key.keyboard.c && !rc_data_last.mouse_key.keyboard.c) {
    if (robot->chassis->super_cap->super_cap_ctrl_cmd == BOOST) {
      robot->chassis->super_cap->super_cap_ctrl_cmd = NORMAL;
    } else {
      robot->chassis->super_cap->super_cap_ctrl_cmd = BOOST;
    }
  }

  // Q/E 持续按住 -> roll 增量 (与 JS rocker_l_ 叠加)
  if (rc_data->mouse_key.keyboard.q) {
    intent_shared.roll_delta -= 0.4f;
  } else if (rc_data->mouse_key.keyboard.e) {
    intent_shared.roll_delta += 0.4f;
  }

  // R / F 三档腿长 (边沿触发)
  const uint8_t r_pressed = rc_data->mouse_key.keyboard.r && !rc_data_last.mouse_key.keyboard.r;
  const uint8_t f_pressed = rc_data->mouse_key.keyboard.f && !rc_data_last.mouse_key.keyboard.f;
  if (r_pressed) {
    if (leg_step < 2) leg_step++;
    mk_set_leg_length = 1;
    mk_leg_length_target = LEG_TABLE[leg_step];
  }
  if (f_pressed) {
    if (leg_step > 0) leg_step--;
    mk_set_leg_length = 1;
    mk_leg_length_target = LEG_TABLE[leg_step];
  }

  // Ctrl+G: hot switch between inverted pendulum and prostrate.
  const uint8_t ctrl_g = rc_data->mouse_key.keyboard.ctrl && rc_data->mouse_key.keyboard.g;
  const uint8_t ctrl_g_last = rc_data_last.mouse_key.keyboard.ctrl && rc_data_last.mouse_key.keyboard.g;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  if (ctrl_g && !ctrl_g_last && chassis_ctrl_cmd->chassis_mode != CHASSIS_RECOVERY && !mk_jump_started) {
    mk_jump_ready = 0;
    mk_jump_started_seen_active = 0;
    mk_stand_mode = !mk_stand_mode;
    ApplyOcdNormalMode(robot);
  }

  // Ctrl+V: jump ready toggle. V alone in ready state starts the jump.
  const uint8_t ctrl_v = rc_data->mouse_key.keyboard.ctrl && rc_data->mouse_key.keyboard.v;
  const uint8_t ctrl_v_last = rc_data_last.mouse_key.keyboard.ctrl && rc_data_last.mouse_key.keyboard.v;
  const uint8_t v_pressed = rc_data->mouse_key.keyboard.v && !rc_data_last.mouse_key.keyboard.v;
  if (ctrl_v && !ctrl_v_last && chassis_ctrl_cmd->chassis_mode != CHASSIS_RECOVERY && !mk_jump_started) {
    if (mk_jump_ready) {
      mk_jump_ready = 0;
      mk_jump_started_seen_active = 0;
      ApplyOcdNormalMode(robot);
    } else {
      mk_jump_ready = 1;
      mk_jump_started_seen_active = 0;
      chassis_ctrl_cmd->chassis_mode = CHASSIS_JUMP_READY;
      robot->robot_mode = ROBOT_CHASSIS_FOLLOW;
    }
  } else if (v_pressed && !rc_data->mouse_key.keyboard.ctrl && mk_jump_ready) {
    mk_jump_ready = 0;
    mk_jump_started = 1;
    mk_jump_started_seen_active = 0;
    mk_jump_started_time = DWT_GetTimeline_s();
    chassis_ctrl_cmd->chassis_mode = CHASSIS_JUMP_START;
    chassis_ctrl_cmd->jump_force = JUMP_FORCE;
    robot->robot_mode = ROBOT_CHASSIS_FREE;
  }

  // WASD -> intent_shared.vx/vy 加性叠加; 单位按当前 robot_mode 匹配 JS
  // FOLLOW / stand+FREE: m/s 量级 (JS 用 0.003*rocker, 全推≈2.0)
  // PROSTRATE_FOLLOW / !stand+FREE: 原始摇杆量级 (全推≈660)
  // ROTATE 系: RobotMotionSolve 不读 intent.vx/vy, 这里给 0 即可
  const uint8_t is_stand = OcdIsStand();
  float wasd_scale;
  switch (robot->robot_mode) {
    case ROBOT_CHASSIS_FOLLOW:
      wasd_scale = 2.0f;
      break;
    case ROBOT_CHASSIS_PROSTRATE_FOLLOW:
      wasd_scale = 660.0f;
      break;
    case ROBOT_CHASSIS_FREE:
      wasd_scale = is_stand ? 2.0f : 660.0f;
      break;
    default:
      wasd_scale = 0.0f;
      break;
  }
  if (rc_data->mouse_key.keyboard.w) intent_shared.vy += wasd_scale;
  if (rc_data->mouse_key.keyboard.s) intent_shared.vy -= wasd_scale;
  if (rc_data->mouse_key.keyboard.d) intent_shared.vx += wasd_scale;
  if (rc_data->mouse_key.keyboard.a) intent_shared.vx -= wasd_scale;

  // RobotMotionSolve / rc_data_last 更新均移至 CtrlSolve()
}

void CtrlSolve(RobotInstance* robot) {
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;

  // 1. 自瞄优先: MK 右键有效时覆盖 yaw/pitch
  if (mk_request_vision) {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_VISION;
    gimbal_ctrl_cmd->yaw = robot->vision_recv_data->gimbal_receive.yaw;
    gimbal_ctrl_cmd->pitch = robot->vision_recv_data->gimbal_receive.pitch;
  }

  // 2. 开火 OR 合并 (沿用 six_wheel pattern)
  if (shoot_ctrl_cmd->shoot_mode == SHOOT_ON && shoot_ctrl_cmd->friction_mode == FRICTION_ON) {
    if (joystick_burst || mouse_burst) {
      shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
    } else if (joystick_fire || mouse_fire) {
      shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
    } else {
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
    }
  }

  // 3. 三档腿长边沿: 一次性写绝对值; JS 的 leg_length_delta 由 RobotMotionSolve 内部叠加
  if (mk_set_leg_length) {
    chassis_ctrl_cmd->leg_length = mk_leg_length_target;
    mk_set_leg_length = 0;
  }

  // 4. 解算运动
  RobotMotionSolve(robot, &intent_shared);

  // 5. JS 原 free+rotate 后处理 (移自 JoyStickCtrl, 应用合并后的 intent)
  const uint8_t is_stand = OcdIsStand();
  const uint8_t is_free = (rc_data->button_status.pause_flag == 1);
  const uint8_t is_rotate = (abs(rc_data->rc.dial) > 20 || rc_data->mouse_key.keyboard.shift);
  if (is_free && is_rotate) {
    if (is_stand) {
      chassis_ctrl_cmd->vx =
          ramp_controller_update(&vx_ramp, intent_shared.vy, robot->chassis->state_var.x_b_d, robot->dt);
      chassis_ctrl_cmd->theta_ff = 0.0f;
      chassis_ctrl_cmd->roll += intent_shared.roll_delta;
      chassis_ctrl_cmd->leg_length += intent_shared.leg_length_delta;
      if (chassis_ctrl_cmd->leg_length > LEG_MAX_LENGTH)
        chassis_ctrl_cmd->leg_length = LEG_MAX_LENGTH;
      else if (chassis_ctrl_cmd->leg_length < LEG_MIN_LENGTH)
        chassis_ctrl_cmd->leg_length = LEG_MIN_LENGTH;
    } else {
      chassis_ctrl_cmd->vx = intent_shared.vy;
    }
  }

  if (mk_jump_started) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_JUMP_START;
    chassis_ctrl_cmd->jump_force = JUMP_FORCE;
    robot->robot_mode = ROBOT_CHASSIS_FREE;
    if (robot->chassis->jump_state != JUMP_STATE_IDLE) {
      mk_jump_started_seen_active = 1;
    }
    const uint8_t jump_state_done = (mk_jump_started_seen_active && robot->chassis->jump_state == JUMP_STATE_IDLE);
    const uint8_t jump_timeout = (DWT_GetTimeline_s() - mk_jump_started_time > 1.2f);
    if (jump_state_done || jump_timeout) {
      mk_jump_started = 0;
      mk_jump_started_seen_active = 0;
      ApplyOcdNormalMode(robot);
    }
  } else if (mk_jump_ready) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_JUMP_READY;
    robot->robot_mode = ROBOT_CHASSIS_FOLLOW;
  }

  // 6. 帧末更新 rc_data_last (供下一帧 MK 边沿检测)
  rc_data_last = *rc_data;
}

void EmergencyHandler(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;

  if (switch_left(rc_data->rc.mode_switch) || rc_data == NULL) {
    robot->robot_mode = ROBOT_POWER_OFF;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    LOGERROR("[CMD] emergency stop!");
  } else if ((robot_lost_control) && (chassis_ctrl_cmd->chassis_mode != CHASSIS_PROSTRATE)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_RECOVERY;
  } else {
    LOGINFO("[CMD] reinstate, robot ready");
  }
  if (switch_middle(rc_data->rc.mode_switch)) {
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }
}
#endif
