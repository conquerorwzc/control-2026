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
static VT13_RC_t* rc_data_last;
#endif

static uint8_t is_first_update = 1;

#define CTRL_SPEED_COFF 2.5f

Ramp_Controller_t chassis_ramp = {
    .planning_v = 0.0f,
    .max_v = 15.0f,
    .max_accel = 1.0f,
    .accel_base_speed = 0.3f,
    .max_decel = 3.7f,
    .min_decel = 1.0f,
    .decel_base_speed = 0.3f,
    .k_error_ff = 0.35f,
};

static float CalcFollowErrDeg(float move_angle_deg, float offset_angle_deg, float* direction_sign) {
  float front_err = wrap180(move_angle_deg - offset_angle_deg);
  float rear_err = wrap180(move_angle_deg - (offset_angle_deg + 180.0f));

  if (fabsf(front_err) <= fabsf(rear_err)) {
    *direction_sign = 1.0f;
    return front_err;
  }

  *direction_sign = -1.0f;
  return rear_err;
}

// ==========================================
// 统一的机器人运动解算层
// ==========================================
static void RobotMotionSolve(RobotInstance* robot, Ctrl_Intent_s* intent) {
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  float actual_vx = intent->vx;
  float actual_vy = intent->vy;
  float input_mag = sqrtf(actual_vx * actual_vx + actual_vy * actual_vy);
  static float phase_compensation = 0.03f;

  switch (robot->robot_mode) {
    case ROBOT_CHASSIS_ROTATE: {
      chassis_ctrl_cmd->target_yaw = robot->chassis->state_var.phi;
      chassis_ctrl_cmd->wz = 8.0f;

      float target_angle_to_gimbal = atan2f(actual_vy, actual_vx);
      float target_angle_to_chassis = target_angle_to_gimbal + robot->offset_angle * DEGREE_2_RAD;
      chassis_ctrl_cmd->vx = input_mag * sinf(target_angle_to_chassis + phase_compensation);
      break;
    }
    case ROBOT_CHASSIS_FOLLOW: {
#if (!defined(ONE_BOARD))
      float follow_direction_sign = 1.0f;
      float follow_err = 0.0f;
      if (input_mag > 0.0005f) {
        float move_angle_deg = (atan2f(actual_vy, actual_vx) - PI / 2.0f) * RAD_2_DEGREE;
        follow_err = CalcFollowErrDeg(move_angle_deg, robot->offset_angle, &follow_direction_sign);
        input_mag *= follow_direction_sign;
      } else {
        follow_err = CalcFollowErrDeg(0.0f, robot->offset_angle, &follow_direction_sign);
      }
      chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + follow_err * DEGREE_2_RAD;
      chassis_ctrl_cmd->wz = 0.0f;

      float align_attenuation = cosf(follow_err * DEGREE_2_RAD);
      if (align_attenuation < 0) align_attenuation = 0;
      input_mag *= align_attenuation * align_attenuation * align_attenuation;

      VAL_LIMIT(input_mag, -2.97f, 2.97f);
      chassis_ctrl_cmd->vx =
          ramp_controller_update(&chassis_ramp, input_mag, robot->chassis->state_var.x_b_d, robot->dt);
      chassis_ctrl_cmd->theta_ff = 0.0f;
      break;
#endif
    }
    case ROBOT_CHASSIS_FREE: {
#if defined(ONE_BOARD)
      chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD;
      chassis_ctrl_cmd->wz = 0.0f;
#else
      chassis_ctrl_cmd->target_yaw =
          robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD - robot->offset_angle * DEGREE_2_RAD;
      chassis_ctrl_cmd->wz = 0.0f;
#endif
      chassis_ctrl_cmd->vx =
          ramp_controller_update(&chassis_ramp, actual_vy, robot->chassis->state_var.x_b_d, robot->dt);
      chassis_ctrl_cmd->theta_ff = 0.0f;
      chassis_ctrl_cmd->roll = intent->roll_delta;
      chassis_ctrl_cmd->leg_length += intent->leg_length_delta;

      if (chassis_ctrl_cmd->leg_length > LEG_MAX_LENGTH)
        chassis_ctrl_cmd->leg_length = LEG_MAX_LENGTH;
      else if (chassis_ctrl_cmd->leg_length < LEG_MIN_LENGTH)
        chassis_ctrl_cmd->leg_length = LEG_MIN_LENGTH;
      break;
    }
    case ROBOT_CHASSIS_PROSTRATE_ROTATE: {
      chassis_ctrl_cmd->is_rotate = 1;
      chassis_ctrl_cmd->vx = 0.0f;
      chassis_ctrl_cmd->wz = 800.0f;

      if (input_mag > 5.0f) {
        float target_angle_to_gimbal_p = atan2f(actual_vy, actual_vx);
        float target_angle_to_chassis_p = target_angle_to_gimbal_p + robot->offset_angle * DEGREE_2_RAD;
        chassis_ctrl_cmd->vx = input_mag * sinf(target_angle_to_chassis_p - 1.57f);
      } else {
        chassis_ctrl_cmd->vx = 0.0f;
      }
      break;
    }
    case ROBOT_CHASSIS_PROSTRATE_FOLLOW: {
#if (!defined(ONE_BOARD))
      chassis_ctrl_cmd->is_rotate = 0;
      float follow_direction_sign = 1.0f;
      float follow_err = 0.0f;

      float max_speed = 800.0f;
      if (input_mag > max_speed) {
        actual_vx = actual_vx / input_mag * max_speed;
        actual_vy = actual_vy / input_mag * max_speed;
        input_mag = max_speed;
      }

      chassis_ctrl_cmd->wz = 0.0f;
      if (input_mag > 5.0f) {
        float move_angle_deg = (atan2f(actual_vy, actual_vx) - PI / 2.0f) * RAD_2_DEGREE;
        follow_err = CalcFollowErrDeg(move_angle_deg, robot->offset_angle, &follow_direction_sign);
        input_mag *= follow_direction_sign;
        chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + follow_err * DEGREE_2_RAD;
        chassis_ctrl_cmd->wz = follow_err;
      } else {
        follow_err = CalcFollowErrDeg(0.0f, robot->offset_angle, &follow_direction_sign);
        chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + follow_err * DEGREE_2_RAD;
      }

      float align_attenuation = cosf(follow_err * DEGREE_2_RAD);
      if (align_attenuation < 0) align_attenuation = 0;
      input_mag *= align_attenuation * align_attenuation * align_attenuation;
      chassis_ctrl_cmd->vx = input_mag;
      break;
#endif
    }
    case ROBOT_CHASSIS_PROSTRATE_FREE: {
      chassis_ctrl_cmd->vx = actual_vy;
      chassis_ctrl_cmd->wz = actual_vx;
      break;
    }
    default:
      break;
  }

  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_VISION) {
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kp = 2.0f;
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kd = 0.03f;
  } else {
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kp = 0.8f;
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kd = 0.02f;
  }

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

  static uint8_t is_rotate_mode = 0;
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
    is_rotate_mode = 1;
  } else {
    is_rotate_mode = 0;
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
  } else if (is_rotate_mode) {
    robot->robot_mode = ROBOT_CHASSIS_ROTATE;
  } else {
    robot->robot_mode = ROBOT_CHASSIS_FOLLOW;
  }

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
    chassis_ramp.planning_v = 0.0f;
    chassis_ramp.expected_a = 0.0f;
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

void JoyStickCtrl(RobotInstance* robot) {
  // USE_OCD_CTRL definition of rc_data is different, it is a pointer
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  static float trigger_time = 0;
  if (is_first_update) {
    rc_data_last = rc_data;
    is_first_update = 0;
  }

  Ctrl_Intent_s intent = {0};

  if (rc_data->button_status.fn_1_flag == 1) {
    if (chassis_ctrl_cmd->chassis_mode != CHASSIS_RECOVERY)
      chassis_ctrl_cmd->chassis_mode = CHASSIS_RECOVERY;
    else
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ON;
    if (abs(rc_data->rc.dial) > 20 || rc_data->mouse_key.keyboard.shift) {
      robot->robot_mode = ROBOT_CHASSIS_ROTATE;
    } else {
      robot->robot_mode = ROBOT_CHASSIS_FOLLOW;
    }
  } else {
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_RECOVERY) {
      if (abs(rc_data->rc.dial) > 20 || rc_data->mouse_key.keyboard.shift)
        robot->robot_mode = ROBOT_CHASSIS_ROTATE;
      else
        robot->robot_mode = ROBOT_CHASSIS_FOLLOW;
    } else if (rc_data->button_status.fn_1_flag == 0) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_PROSTRATE;
      if (abs(rc_data->rc.dial) > 20 || rc_data->mouse_key.keyboard.shift)
        robot->robot_mode = ROBOT_CHASSIS_PROSTRATE_ROTATE;
      else
        robot->robot_mode = ROBOT_CHASSIS_PROSTRATE_FOLLOW;
    }
    if (rc_data->button_status.pause_flag == 1) {
      robot->robot_mode = ROBOT_CHASSIS_FREE;
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
      if (DWT_GetTimeline_s() - trigger_time > 1.0f) {
        shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
      } else {
        shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
      }
    } else {
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
    }
  } else {
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }

  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
  gimbal_ctrl_cmd->yaw += -0.00035f * (float)rc_data->rc.rocker_r_;
  gimbal_ctrl_cmd->pitch += -0.00006f * (float)rc_data->rc.rocker_r1;

  if (robot->robot_mode == ROBOT_CHASSIS_ROTATE) {
    // intent.vx/vy remain 0
  } else if (robot->robot_mode == ROBOT_CHASSIS_FOLLOW) {
    intent.vx = 0.003f * (float)rc_data->rc.rocker_l_;
    intent.vy = 0.003f * (float)rc_data->rc.rocker_l1;
  } else if (robot->robot_mode == ROBOT_CHASSIS_FREE) {
    gimbal_ctrl_cmd->yaw += (-0.25f) * (float)rc_data->rc.rocker_r_ * robot->dt * DEGREE_2_RAD;
    intent.vy = (0.004f) * (float)rc_data->rc.rocker_r1;
    intent.roll_delta = 0.0004f * (float)rc_data->rc.rocker_l_ * (abs(rc_data->rc.rocker_l_) > 10);
    intent.leg_length_delta = 0.0000005f * (float)rc_data->rc.rocker_l1;
  } else if (robot->robot_mode == ROBOT_CHASSIS_PROSTRATE_FOLLOW) {
    intent.vx = (float)rc_data->rc.rocker_l_;
    intent.vy = (float)rc_data->rc.rocker_l1;
  } else if (robot->robot_mode == ROBOT_CHASSIS_PROSTRATE_ROTATE) {
    // Nothing here
  } else if (robot->robot_mode == ROBOT_CHASSIS_PROSTRATE_FREE) {
    intent.vy = (float)rc_data->rc.rocker_l1;
    intent.vx = (float)rc_data->rc.rocker_l_;
  }

  RobotMotionSolve(robot, &intent);
  rc_data_last = rc_data;
}

void MouseKeyCtrl(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  Vision_Receive_s* vision_recv_data = robot->vision_recv_data;

  static uint8_t is_rotate_mode = 0;
  static float trigger_time = 0;

  Ctrl_Intent_s intent = {0};

  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;

  if (rc_data->mouse_key.mouse.press_l) {
    if (shoot_ctrl_cmd->friction_mode == FRICTION_ON) {
      if (DWT_GetTimeline_s() - trigger_time > 0.3f) {
        shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
      } else {
        shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
      }
    }
  } else {
    if (!switch_is_up(rc_data->rc.mode_switch)) {
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
      trigger_time = DWT_GetTimeline_s();
    }
  }

  if (rc_data->mouse_key.mouse.press_r && has_non_zero_data(vision_recv_data)) {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_VISION;
    gimbal_ctrl_cmd->yaw = vision_recv_data->gimbal_receive.yaw;
    gimbal_ctrl_cmd->pitch = vision_recv_data->gimbal_receive.pitch;
  } else {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    gimbal_ctrl_cmd->yaw += -(float)rc_data->mouse_key.mouse.x * 0.002f;
    gimbal_ctrl_cmd->pitch += -(float)rc_data->mouse_key.mouse.y * 0.002f;

    if (rc_data->mouse_key.keyboard.x && !rc_data_last->mouse_key.keyboard.x) {
      gimbal_ctrl_cmd->yaw += 180.0f;
    }
  }

  if (abs(rc_data->rc.dial) > 20 || rc_data->mouse_key.keyboard.shift) {
    is_rotate_mode = 1;
  } else {
    is_rotate_mode = 0;
  }

  if (!rc_data_last->mouse_key.keyboard.ctrl && rc_data->mouse_key.keyboard.ctrl) {
    if (robot->chassis_fetch_data) {
      robot->chassis_fetch_data->force_refresh_ui = 1;
    }
  }

  if (rc_data->mouse_key.keyboard.c && !rc_data_last->mouse_key.keyboard.c) {
    if (robot->chassis->super_cap->super_cap_ctrl_cmd == BOOST) {
      robot->chassis->super_cap->super_cap_ctrl_cmd = NORMAL;
    } else {
      robot->chassis->super_cap->super_cap_ctrl_cmd = BOOST;
    }
  }

  if (is_rotate_mode) {
    robot->robot_mode = ROBOT_CHASSIS_ROTATE;
  } else {
    robot->robot_mode = ROBOT_CHASSIS_FOLLOW;
  }

  if (rc_data->mouse_key.keyboard.w)
    intent.vy = 0.5f * CTRL_SPEED_COFF;
  else if (rc_data->mouse_key.keyboard.s)
    intent.vy = -0.5f * CTRL_SPEED_COFF;
  else
    intent.vy = 0.0f;

  if (rc_data->mouse_key.keyboard.d)
    intent.vx = 0.5f * CTRL_SPEED_COFF;
  else if (rc_data->mouse_key.keyboard.a)
    intent.vx = -0.5f * CTRL_SPEED_COFF;
  else
    intent.vx = 0.0f;

  if (robot->robot_mode == ROBOT_CHASSIS_FREE) {
    if (rc_data->mouse_key.keyboard.w)
      intent.vy = (0.99f) * CTRL_SPEED_COFF;
    else if (rc_data->mouse_key.keyboard.s)
      intent.vy = (-0.99f) * CTRL_SPEED_COFF;
    else
      intent.vy = 0.0f;
  }

  RobotMotionSolve(robot, &intent);
  rc_data_last = rc_data;
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
  } else {
    LOGINFO("[CMD] reinstate, robot ready");
  }

  if (robot_lost_control) {
    if (chassis_ctrl_cmd->chassis_mode != CHASSIS_PROSTRATE) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_RECOVERY;
    }
  }

  if (switch_middle(rc_data->rc.mode_switch)) {
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }
}
#endif
