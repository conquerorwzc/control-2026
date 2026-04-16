#include "ctrl.h"

#include <math.h>

#include "general_def.h"
#include "robot_config.h"

// Static variables for control state
#ifdef USE_DUAL_RC
static RC_ctrl_t* rc_data;
static RC_ctrl_t rc_data_last;
#elifdef USE_DUAL_RC_NEW
static VT13_RC_t* rc_data;
static VT13_RC_t* rc_data_last;
#endif
static uint8_t is_first_update = 1;

// Intermediate variables
static float chassis_vx;
static float chassis_vy;
static float input_mag;
static float follow_err;
static float align_attenuation;

// 小陀螺相关参数 (moved from robot.c static)
static float rotate_frequency;  // 小陀螺旋转的频率
static float rotate_omega;      // 小陀螺旋转角速度

static uint8_t joystick_fire = 0;   // 遥控器要求开火
static uint8_t joystick_burst = 0;  // 遥控器要求连发
static uint8_t mouse_fire = 0;      // 鼠标要求开火
static uint8_t mouse_burst = 0;     // 鼠标要求连发

// Ramp controller (externed in header)
// 挺好用的一版
Ramp_Controller_t chassis_ramp = {
    .planning_v = 0.0f,
    .max_v = 15.0f,
    .max_accel = 1.0f,
    .accel_base_speed = 0.3f,
    .max_decel = 3.7f,
    .min_decel = 1.0f,
    .decel_base_speed = 0.3f,
    .k_error_ff = 0.35f,  // 速度误差前馈补偿系数，可根据实际测试调整
};

#ifdef USE_DUAL_RC
void JoyStickCtrl(RobotInstance* robot) {
  // Helper pointers
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  static float trigger_time = 0;

  if (is_first_update) {
    rc_data_last = rc_data[TEMP];
    is_first_update = 0;
  }

  // Need to handle case where modules might be NULL if not initialized (e.g. ONE_BOARD vs GIMBAL_BOARD)
  // Assuming robot is fully initialized as per RobotInit logic

  // 右[中]，底盘使能 ROBOT_CHASSIS_FOLLOW
  if (switch_is_mid(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    if (abs(rc_data[TEMP].rc.dial) > 20 || rc_data[TEMP].key[KEY_PRESS].shift) {
      robot->robot_mode = ROBOT_CHASSIS_ROTATE;
    } else {
      robot->robot_mode = ROBOT_CHASSIS_FOLLOW;
    }
  }
  // 右[上]，底盘使能，允许跳跃 ROBOT_CHASSIS_FREE
  else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
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
      chassis_ctrl_cmd->jump_force = 55 * JUMP_FORCE;
      // chassis_ctrl_cmd->jump_force = 0;
    }
  }
  if (!switch_is_up(rc_data[TEMP].rc.switch_right)) {
    // 左[中],云台启动，摩擦轮启动，准备射击
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
      shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
      shoot_ctrl_cmd->friction_mode = FRICTION_ON;
      // shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
      // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
    }
    // 左[上]，开火，发射，根据时间判断单发或者连发
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
      shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
      shoot_ctrl_cmd->friction_mode = FRICTION_ON;
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
      if (switch_is_mid(rc_data_last.rc.switch_left)) {
        trigger_time = DWT_GetTimeline_s();
      }
      if (DWT_GetTimeline_s() - trigger_time > 0.5f) {
        shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
      } else {
        shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
      }
    }
    // 云台使能,或视觉未识别到目标,纯遥控器拨杆控制
    if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {  // 按照摇杆的输出大小进行角度增量,增益系数需调整
      gimbal_ctrl_cmd->pitch -= 0.00006f * (float)rc_data[TEMP].rc.rocker_r1;
    }
  } else {
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {  // 按照摇杆的输出大小进行角度增量,增益系数需调整
    gimbal_ctrl_cmd->yaw += -0.00035f * (float)rc_data[TEMP].rc.rocker_r_;
    // gimbal_ctrl_cmd->yaw += -0.00034f * (float)rc_data[TEMP].rc.rocker_r_;
    // gimbal_ctrl_cmd->yaw += -0.00005f * (float)rc_data[TEMP].rc.rocker_r_;
  }
  // 云台PITCH轴软件限位 todo:没在云台有点不好
  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }

  switch (robot->robot_mode) {
    // 键鼠开了小陀螺，遥控器原地小陀螺不要开了，todo:之后可以把这个运动解算switch解耦出来，有点史
    case ROBOT_CHASSIS_ROTATE: {
      // // 小陀螺频率设置
      // rotate_frequency = 0.5f;
      //
      // // 小陀螺原地旋转
      // rotate_omega = rotate_frequency * 2.0f * PI;
      // chassis_ctrl_cmd->target_yaw += rotate_omega * robot->dt;

      // 设置目标速度矢量(vx,vy)
      chassis_vx = 0.0025f * (float)rc_data[TEMP].rc.rocker_l_;
      chassis_vy = 0.0025f * (float)rc_data[TEMP].rc.rocker_l1;
      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);  // 速度的模

      // 转换角度坐标系
      float target_angle_to_gimbal = atan2f(chassis_vy, chassis_vx);  // 目标方向矢量与云台正方向方向夹角
      float target_angle_to_chassis = target_angle_to_gimbal + robot->offset_angle * DEGREE_2_RAD;

      // 相位补偿，单位是rad todo:参数，要测试
      float phase_compensation = 0.03f;
      // 正弦速度调制
      chassis_ctrl_cmd->vx = input_mag * sinf(target_angle_to_chassis + phase_compensation);
      break;
    }
    case ROBOT_CHASSIS_FOLLOW: {
#if (!defined(ONE_BOARD))
      // 获取输入
      chassis_vx = 0.003f * (float)rc_data[TEMP].rc.rocker_l_;
      chassis_vy = 0.003f * (float)rc_data[TEMP].rc.rocker_l1;
      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
      if (input_mag > 0.0005f) {
        // 运动方向解算
        follow_err = (atan2f(chassis_vy, chassis_vx) - PI / 2.0f) * RAD_2_DEGREE - robot->offset_angle;
        while (follow_err > 180.0f) follow_err -= 360.0f;
        while (follow_err < -180.0f) follow_err += 360.0f;
        // 倒车优化
        if (abs(follow_err) > 90.0f) {
          if (follow_err > 0.0f)
            follow_err -= 180.0f;
          else
            follow_err += 180.0f;
          input_mag = -input_mag;
        }
        chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + follow_err * DEGREE_2_RAD;
      } else {
        chassis_ctrl_cmd->target_yaw =
            robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD - robot->offset_angle * DEGREE_2_RAD;
      }
      chassis_ctrl_cmd->wz = 0.0f;  // 无前馈角速度
      // 对齐衰减
      align_attenuation = cosf(follow_err * DEGREE_2_RAD);
      if (align_attenuation < 0) align_attenuation = 0;
      input_mag *= align_attenuation * align_attenuation * align_attenuation;
      VAL_LIMIT(input_mag, -2.50, 2.50);
      // chassis_ctrl_cmd->vx = ramp_controller_update(&chassis_ramp, input_mag, robot->chassis->state_var.x_b_d,
      // robot->dt); chassis_ctrl_cmd->vx = input_mag; chassis_ctrl_cmd->theta_ff = chassis_ramp.expected_a / 9.81f;
      chassis_ctrl_cmd->theta_ff = 0.0f;
      break;
#endif
    }
    case ROBOT_CHASSIS_FREE: {
#if defined(ONE_BOARD)
      // 摇杆积分目标角度
      chassis_ctrl_cmd->target_yaw += (-0.25f) * (float)rc_data[TEMP].rc.rocker_r_ * robot->dt * DEGREE_2_RAD;
      chassis_ctrl_cmd->wz = 0.0f;
#else
      // 双板：静止对齐云台
      chassis_ctrl_cmd->target_yaw =
          robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD - robot->offset_angle * DEGREE_2_RAD;
      chassis_ctrl_cmd->wz = 0.0f;
#endif
      chassis_ctrl_cmd->vx = ramp_controller_update(&chassis_ramp, (0.004f) * (float)rc_data[TEMP].rc.rocker_r1,
                                                    robot->chassis->state_var.x_b_d, robot->dt);
      // chassis_ctrl_cmd->vx = (0.0025f) * (float)rc_data[TEMP].rc.rocker_r1;
      // chassis_ctrl_cmd->theta_ff = chassis_ramp.expected_a / 9.81f;
      chassis_ctrl_cmd->theta_ff = 0.0f;
      chassis_ctrl_cmd->roll = 0.0004f * (float)rc_data[TEMP].rc.rocker_l_ * (abs(rc_data[TEMP].rc.rocker_l_) > 10);
      chassis_ctrl_cmd->leg_length += 0.0000005f * (float)rc_data[TEMP].rc.rocker_l1;
      if (chassis_ctrl_cmd->leg_length > LEG_MAX_LENGTH) {
        chassis_ctrl_cmd->leg_length = LEG_MAX_LENGTH;
      } else if (chassis_ctrl_cmd->leg_length < LEG_MIN_LENGTH) {
        chassis_ctrl_cmd->leg_length = LEG_MIN_LENGTH;
      }
      break;
    }
    default:
      break;
  }
  //  记录上一次数据
  // rc_data_last = rc_data[TEMP];
}
#elifdef USE_DUAL_RC_NEW
void JoyStickCtrl(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  static float trigger_time = 0;

  // 使用VT13遥控器的新控制逻辑
  // 中档：robot follow/rotate,pause键切换为robot free
  //      fn切换prostrate
  if (switch_middle(rc_data->rc.mode_switch)) {
    // 中档
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    if (rc_data->button_status.fn_1_flag == 0) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ON;
      if (rc_data->button_status.pause_flag == 0) {
        if (abs(rc_data[TEMP].rc.dial) > 20 || rc_data->mouse_key.keyboard.shift)
          robot->robot_mode = ROBOT_CHASSIS_ROTATE;
        else
          robot->robot_mode = ROBOT_CHASSIS_FOLLOW;
      } else if (rc_data->button_status.pause_flag == 1) {
        robot->robot_mode = ROBOT_CHASSIS_FREE;
      }
    } else if (rc_data->button_status.fn_1_flag == 1) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_PROSTRATE;
      if (abs(rc_data[TEMP].rc.dial) > 20 || rc_data->mouse_key.keyboard.shift)
        robot->robot_mode = ROBOT_CHASSIS_PROSTRATE_ROTATE;
      else
        robot->robot_mode = ROBOT_CHASSIS_PROSTRATE_FOLLOW;
    }
  }
  // 上档frc on/shoot，pause键切换为robot free
  if (switch_right(rc_data->rc.mode_switch)) {
    shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    shoot_ctrl_cmd->friction_mode = FRICTION_ON;
    static uint8_t trigger_last = 0;
    if (trigger_last == 0 && rc_data->rc.trigger == 1) {
      trigger_time = DWT_GetTimeline_s();
    }
    trigger_last = rc_data->rc.trigger;
    if (rc_data->rc.trigger == 1) {
      if (DWT_GetTimeline_s() - trigger_time > 1.0f) {
        joystick_fire = 1;
        joystick_burst = 1;
      } else {
        joystick_fire = 1;
        joystick_burst = 0;
      }
    } else {
      joystick_fire = 0;
      joystick_burst = 0;
    }
  }
  // 云台控制
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    gimbal_ctrl_cmd->yaw += -0.00035f * (float)rc_data->rc.rocker_r_;
    gimbal_ctrl_cmd->pitch -= 0.00006f * (float)rc_data->rc.rocker_r1;
  }

  switch (robot->robot_mode) {
    // 键鼠开了小陀螺，遥控器原地小陀螺不要开了
    case ROBOT_CHASSIS_ROTATE: {
      // 小陀螺频率设置
      rotate_frequency = 0.5f;

      // 小陀螺原地旋转
      rotate_omega = rotate_frequency * 2.0f * PI;
      chassis_ctrl_cmd->target_yaw += rotate_omega * robot->dt;

      // // 设置目标速度矢量(vx,vy)
      // chassis_vx = 0.0025f * (float)rc_data[TEMP].rc.rocker_l_;
      // chassis_vy = 0.0025f * (float)rc_data[TEMP].rc.rocker_l1;
      // input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);  // 速度的模
      //
      // // 转换角度坐标系
      // float target_angle_to_gimbal = atan2f(chassis_vy, chassis_vx);  // 目标方向矢量与云台正方向方向夹角
      // float target_angle_to_chassis = target_angle_to_gimbal + robot->offset_angle * DEGREE_2_RAD;
      //
      // // 相位补偿，单位是rad todo:参数，要测试
      // float phase_compensation = 0.03f;
      // // 正弦速度调制
      // chassis_ctrl_cmd->vx = input_mag * sinf(target_angle_to_chassis + phase_compensation);
      break;
    }
    case ROBOT_CHASSIS_FOLLOW: {
#if (!defined(ONE_BOARD))
      // 获取输入
      chassis_vx = 0.003f * (float)rc_data->rc.rocker_l_;
      chassis_vy = 0.003f * (float)rc_data->rc.rocker_l1;
      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
      if (input_mag > 0.0005f) {
        // 运动方向解算
        follow_err = (atan2f(chassis_vy, chassis_vx) - PI / 2.0f) * RAD_2_DEGREE - robot->offset_angle;
        while (follow_err > 180.0f) follow_err -= 360.0f;
        while (follow_err < -180.0f) follow_err += 360.0f;
        // 倒车优化
        if (abs(follow_err) > 90.0f) {
          if (follow_err > 0.0f)
            follow_err -= 180.0f;
          else
            follow_err += 180.0f;
          input_mag = -input_mag;
        }
        chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + follow_err * DEGREE_2_RAD;
      } else {
        chassis_ctrl_cmd->target_yaw =
            robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD - robot->offset_angle * DEGREE_2_RAD;
      }
      chassis_ctrl_cmd->wz = 0.0f;  // 无前馈角速度
      // 对齐衰减
      align_attenuation = cosf(follow_err * DEGREE_2_RAD);
      if (align_attenuation < 0) align_attenuation = 0;
      input_mag *= align_attenuation * align_attenuation * align_attenuation;
      VAL_LIMIT(input_mag, -2.50, 2.50);
      chassis_ctrl_cmd->vx =
          ramp_controller_update(&chassis_ramp, input_mag, robot->chassis->state_var.x_b_d, robot->dt);
      // chassis_ctrl_cmd->vx = input_mag;
      // chassis_ctrl_cmd->theta_ff = chassis_ramp.expected_a / 9.81f;
      chassis_ctrl_cmd->theta_ff = 0.0f;
      break;
#endif
    }
    case ROBOT_CHASSIS_FREE: {
#if defined(ONE_BOARD)
      // 摇杆积分目标角度
      chassis_ctrl_cmd->target_yaw += (-0.25f) * (float)rc_data->rc.rocker_r_ * robot->dt * DEGREE_2_RAD;
      chassis_ctrl_cmd->wz = 0.0f;
#else
      // 双板：静止对齐云台
      chassis_ctrl_cmd->target_yaw =
          robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD - robot->offset_angle * DEGREE_2_RAD;
      chassis_ctrl_cmd->wz = 0.0f;
#endif
      chassis_ctrl_cmd->vx = ramp_controller_update(&chassis_ramp, (0.004f) * (float)rc_data->rc.rocker_r1,
                                                    robot->chassis->state_var.x_b_d, robot->dt);
      chassis_ctrl_cmd->theta_ff = 0.0f;
      chassis_ctrl_cmd->roll = 0.0004f * (float)rc_data->rc.rocker_l_ * (abs(rc_data->rc.rocker_l_) > 10);
      chassis_ctrl_cmd->leg_length += 0.0000005f * (float)rc_data->rc.rocker_l1;
      if (chassis_ctrl_cmd->leg_length > LEG_MAX_LENGTH) {
        chassis_ctrl_cmd->leg_length = LEG_MAX_LENGTH;
      } else if (chassis_ctrl_cmd->leg_length < LEG_MIN_LENGTH) {
        chassis_ctrl_cmd->leg_length = LEG_MIN_LENGTH;
      }
      break;
    }
    case ROBOT_CHASSIS_PROSTRATE_FOLLOW: {
#if (!defined(ONE_BOARD))
      chassis_ctrl_cmd->is_rotate = 0;
      // 获取输入（左摇杆 → 云台坐标系下的 vx, vy）
      chassis_vx = (float)rc_data[TEMP].rc.rocker_l_;
      chassis_vy = (float)rc_data[TEMP].rc.rocker_l1;
      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
      if (input_mag > 5.0f) {
        // 运动方向解算
        follow_err = (atan2f(chassis_vy, chassis_vx) - PI / 2.0f) * RAD_2_DEGREE - robot->offset_angle;
        while (follow_err > 180.0f) follow_err -= 360.0f;
        while (follow_err < -180.0f) follow_err += 360.0f;
        // 倒车优化
        if (abs(follow_err) > 90.0f) {
          if (follow_err > 0.0f)
            follow_err -= 180.0f;
          else
            follow_err += 180.0f;
          input_mag = -input_mag;
        }
        chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + follow_err * DEGREE_2_RAD;
      } else {
        // 静止回正：底盘对齐云台
        chassis_ctrl_cmd->target_yaw =
            robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD - robot->offset_angle * DEGREE_2_RAD;
      }
      // 对齐衰减
      align_attenuation = cosf(follow_err * DEGREE_2_RAD);
      if (align_attenuation < 0) align_attenuation = 0;
      input_mag *= align_attenuation * align_attenuation * align_attenuation;
      // 直接传摇杆原始值，ChassisProstrateMode 里做映射
      chassis_ctrl_cmd->vx = input_mag;
#endif
      break;
    }
    case ROBOT_CHASSIS_PROSTRATE_ROTATE: {
      chassis_ctrl_cmd->is_rotate = 1;
      chassis_ctrl_cmd->vx = 0.0f;
      chassis_ctrl_cmd->wz = 800.0f;

      chassis_vx = 0.0f;
      chassis_vy = 0.0f;

      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
      if (input_mag > 5.0f) {  // 加死区，摇杆归中时不算
        float target_angle_to_gimbal_p = atan2f(chassis_vy, chassis_vx);
        float target_angle_to_chassis_p = target_angle_to_gimbal_p + robot->offset_angle * DEGREE_2_RAD;
        chassis_ctrl_cmd->vx = input_mag * sinf(target_angle_to_chassis_p - 1.57);
      } else
        chassis_ctrl_cmd->vx = 0.0f;
      break;
    }
    default:
      break;
  }
  rc_data_last = rc_data;
}
#endif

#ifdef USE_DUAL_RC
void MouseKeyCtrl(RobotInstance* robot) {
  // Helper pointers
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  Vision_Receive_s* vision_recv_data = robot->vision_recv_data;

  if (is_first_update) {
    rc_data_last = rc_data[TEMP];
    is_first_update = 0;
  }

  // 1. 基础初始化
  static float speed_coff = 1.0f;      // 速度系数
  static float rotate_coff = 1.0f;     // 小陀螺旋转频率系数
  static uint8_t x_key_last = 0;       // 记录上一次x键状态
  static uint8_t f_key_last = 0;       // 记录上一次F键状态
  static uint8_t ctrl_g_key_last = 0;  // 记录上一次Ctrl+R键状态
  static uint8_t c_key_last = 0;       // 记录上一次C键状态
  static uint8_t is_rotate_mode = 0;   // 小陀螺模式标志位
  static float trigger_time = 0;       // 开火时间

  // [Ctrl+Z]键设置速度，测试用
  switch (rc_data[TEMP].key_count[KEY_PRESS_WITH_CTRL][Key_Z] % 3) {
    case 0:
      speed_coff = 1.5f;
      rotate_coff = 1.5f;
      break;
    case 1:
      speed_coff = 2.0f;
      rotate_coff = 2.0f;
      break;
    case 2:
      speed_coff = 2.5f;
      rotate_coff = 2.5f;
      break;
    default:
      break;
  }
  // 2. 云台
  // 2.1 [右键]按住开启自瞄

  if (rc_data[TEMP].mouse.press_r) {
    if (has_non_zero_data(vision_recv_data)) {
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_VISION;  // 右键自瞄开启
      gimbal_ctrl_cmd->yaw = vision_recv_data->gimbal_receive.yaw;
      gimbal_ctrl_cmd->pitch = vision_recv_data->gimbal_receive.pitch;
    } else {
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;  // 没识别到目标，保持手动
    }
  } else {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;  // 松开右键，手动模式
  }

  // 2.2 射击控制逻辑,[左键]射击
  if (rc_data[TEMP].mouse.press_l) {
    // 校验：摩擦轮开启 + (自瞄开火标志有效 或 右键未处于自瞄状态)
    if (shoot_ctrl_cmd->friction_mode == FRICTION_ON &&
        (vision_recv_data->shoot_receive.fire_flag || rc_data[TEMP].mouse.press_r % 2 == 0)) {
      // 默认先设为单发
      shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
      // 检查按下的持续时间 (当前时间 - 上次松开的时间/按下起始时间),超过1秒，覆盖为连发
      if (DWT_GetTimeline_s() - trigger_time > 0.2f) {
        shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
      }
    }
  } else {
    if (!switch_is_up(rc_data[TEMP].rc.switch_left)) {
      //  鼠标左键松开时
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
      // 记录松开时间
      trigger_time = DWT_GetTimeline_s();
    }
  }

  // 2.3鼠标云台控制
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    gimbal_ctrl_cmd->yaw -= (float)rc_data[TEMP].mouse.x * 0.002f;    // X轴灵敏度
    gimbal_ctrl_cmd->pitch -= (float)rc_data[TEMP].mouse.y * 0.002f;  // Y轴灵敏度
    // 云台Pitch限位
    if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE)
      gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
    else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE)
      gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }

  // 3. 功能键
  // [Shift] 键按住小陀螺
  if (abs(rc_data[TEMP].rc.dial) > 20 || rc_data[TEMP].key[KEY_PRESS].shift) {
    is_rotate_mode = 1;
  } else {
    is_rotate_mode = 0;
  }

  // [X] 单次触发：转 180° 逃跑
  if (rc_data[TEMP].key[KEY_PRESS].x != x_key_last) {
    if (rc_data[TEMP].key[KEY_PRESS].x) {
      gimbal_ctrl_cmd->yaw += 180.0f;
    }
    x_key_last = rc_data[TEMP].key[KEY_PRESS].x;
  }
  // [Ctrl+G] 单次切换：自起
  if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].g != ctrl_g_key_last) {
    if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].g) {
      // 单次按下触发自起
      if (chassis_ctrl_cmd->chassis_mode != CHASSIS_RECOVERY) {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_RECOVERY;  // 进入自起模式
      } else {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_ON;
      }
    }
    ctrl_g_key_last = rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].g;
  }
  // [F] 单次切换：切换飞坡模式
  if (rc_data[TEMP].key[KEY_PRESS].f != f_key_last) {
    if (rc_data[TEMP].key[KEY_PRESS].f) {
      // 切换飞坡参数
      // 待添加...
    }
    f_key_last = rc_data[TEMP].key[KEY_PRESS].f;
  }
  // [C] 单次切换：超级电容开关
  if (rc_data[TEMP].key[KEY_PRESS].c != c_key_last) {
    if (rc_data[TEMP].key[KEY_PRESS].c) {
      // 超级电容开关
      // 待添加...
    }
    c_key_last = rc_data[TEMP].key[KEY_PRESS].c;
  }
  // [Q] 持续按住：左pike，[E] 持续按住：右pike
  if (rc_data[TEMP].key[KEY_PRESS].q) {
    chassis_ctrl_cmd->roll = -0.4f;
  } else if (rc_data[TEMP].key[KEY_PRESS].e) {
    chassis_ctrl_cmd->roll = 0.4f;
  } else {
    chassis_ctrl_cmd->roll = 0.0f;
  }

  // [Ctrl+R] 持续按住：升高腿长，[Ctrl+F] 持续按住：下降腿长
  if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].r) {
    chassis_ctrl_cmd->leg_length += 0.0000005f * 630.0f;
  } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].f) {
    chassis_ctrl_cmd->leg_length -= 0.0000005f * 630.0f;
  }

  // 腿长限位
  if (chassis_ctrl_cmd->leg_length > LEG_MAX_LENGTH) {
    chassis_ctrl_cmd->leg_length = LEG_MAX_LENGTH;
  } else if (chassis_ctrl_cmd->leg_length < LEG_MIN_LENGTH) {
    chassis_ctrl_cmd->leg_length = LEG_MIN_LENGTH;
  }

  // 4. 核心运动算法
  //  确定 Robot Mode
  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_RECOVERY || chassis_ctrl_cmd->chassis_mode == CHASSIS_JUMP_READY ||
      chassis_ctrl_cmd->chassis_mode == CHASSIS_JUMP_START) {
    robot->robot_mode = ROBOT_CHASSIS_FREE;  // 特殊动作保持FREE模式
  } else if (is_rotate_mode) {               // 小陀螺
    robot->robot_mode = ROBOT_CHASSIS_ROTATE;
  } else {
    robot->robot_mode = ROBOT_CHASSIS_FOLLOW;  // 默认是FOLLOW模式
  }
  // 处理对应模式
  switch (robot->robot_mode) {
    case ROBOT_CHASSIS_ROTATE:
      // 小陀螺频率设置
      rotate_frequency = 0.5f * rotate_coff;

      // 小陀螺原地旋转
      rotate_omega = rotate_frequency * 2.0f * PI;
      chassis_ctrl_cmd->target_yaw += rotate_omega * robot->dt;

      // // 设置目标速度矢量 (vx, vy),单位为m/s
      // if (rc_data[TEMP].key[KEY_PRESS].w)
      //   chassis_vy = 0.5f * speed_coff;
      // else if (rc_data[TEMP].key[KEY_PRESS].s)
      //   chassis_vy = -0.5f * speed_coff;
      // else
      //   chassis_vy = 0.0f;

      // if (rc_data[TEMP].key[KEY_PRESS].d)
      //   chassis_vx = 0.5f * speed_coff;
      // else if (rc_data[TEMP].key[KEY_PRESS].a)
      //   chassis_vx = -0.5f * speed_coff;
      // else
      //   chassis_vx = 0.0f;

      // input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);  // 速度的模

      // // 转换角度坐标系
      // float target_angle_to_gimbal = atan2f(chassis_vy, chassis_vx);  // 目标方向矢量与云台正方向方向夹角
      // float target_angle_to_chassis = target_angle_to_gimbal + robot->offset_angle * DEGREE_2_RAD;

      // // 相位补偿，单位是rad todo:参数，要测试
      // float phase_compensation = 0.03f;
      // // 正弦速度调制
      // chassis_ctrl_cmd->vx = input_mag * sinf(target_angle_to_chassis + phase_compensation);
      break;

    case ROBOT_CHASSIS_FOLLOW:
#if (!defined(ONE_BOARD))
      // 设置目标速度矢量 (vx, vy),单位为m/s
      if (rc_data[TEMP].key[KEY_PRESS].w)
        chassis_vy = 0.5f * speed_coff;
      else if (rc_data[TEMP].key[KEY_PRESS].s)
        chassis_vy = -0.5f * speed_coff;
      else
        chassis_vy = 0.0f;

      if (rc_data[TEMP].key[KEY_PRESS].d)
        chassis_vx = 0.5f * speed_coff;
      else if (rc_data[TEMP].key[KEY_PRESS].a)
        chassis_vx = -0.5f * speed_coff;
      else
        chassis_vx = 0.0f;

      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
      if (input_mag > 0.0005f) {
        // 运动方向解算
        follow_err = (atan2f(chassis_vy, chassis_vx) - PI / 2.0f) * RAD_2_DEGREE - robot->offset_angle;
        while (follow_err > 180.0f) follow_err -= 360.0f;
        while (follow_err < -180.0f) follow_err += 360.0f;
        // 倒车优化
        if (abs(follow_err) > 90.0f) {
          if (follow_err > 0.0f)
            follow_err -= 180.0f;
          else
            follow_err += 180.0f;
          input_mag = -input_mag;
        }
        // ====== 核心修改：直接计算目标yaw角度 ======
        // 目标 = 当前yaw - offset_angle + follow_err (让底盘朝向运动方向)
        // 等价于：让底盘转到 gimbal方向 再补偿 follow_err
        chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + follow_err * DEGREE_2_RAD;
      } else {
        // 静止回正：让底盘对齐云台 (offset → 0)
        chassis_ctrl_cmd->target_yaw =
            robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD - robot->offset_angle * DEGREE_2_RAD;
      }
      chassis_ctrl_cmd->wz = 0.0f;  // 无前馈角速度
      // 对齐衰减
      align_attenuation = cosf(follow_err * DEGREE_2_RAD);
      if (align_attenuation < 0) align_attenuation = 0;
      input_mag *= align_attenuation * align_attenuation * align_attenuation;
      VAL_LIMIT(input_mag, -2.97, 2.97);
      chassis_ctrl_cmd->vx =
          ramp_controller_update(&chassis_ramp, input_mag, robot->chassis->state_var.x_b_d, robot->dt);
      // chassis_ctrl_cmd->vx = input_mag;
      // chassis_ctrl_cmd->theta_ff = chassis_ramp.expected_a / 9.81f;
      chassis_ctrl_cmd->theta_ff = 0.0f;
      break;
#endif
    case ROBOT_CHASSIS_FREE:
#if (!defined(ONE_BOARD))
      // 双板：静止对齐云台
      chassis_ctrl_cmd->target_yaw =
          robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD - robot->offset_angle * DEGREE_2_RAD;
      chassis_ctrl_cmd->wz = 0.0f;
#endif
      if (rc_data[TEMP].key[KEY_PRESS].w)
        chassis_ctrl_cmd->vx =
            ramp_controller_update(&chassis_ramp, (0.99f) * speed_coff, robot->chassis->state_var.x_b_d, robot->dt);
      else if (rc_data[TEMP].key[KEY_PRESS].s)
        chassis_ctrl_cmd->vx =
            ramp_controller_update(&chassis_ramp, (-0.99f) * speed_coff, robot->chassis->state_var.x_b_d, robot->dt);
      else
        chassis_ctrl_cmd->vx = ramp_controller_update(&chassis_ramp, 0.0f, robot->chassis->state_var.x_b_d, robot->dt);

      chassis_ctrl_cmd->theta_ff = 0.0f;
      break;
    default:
      break;
  }
  // 6.更新历史数据
  rc_data_last = rc_data[TEMP];
}

#elifdef USE_DUAL_RC_NEW
void MouseKeyCtrl(RobotInstance* robot) {
  // Helper pointers
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  Vision_Receive_s* vision_recv_data = robot->vision_recv_data;

  if (is_first_update) {
    rc_data_last = rc_data;
    is_first_update = 0;
  }

  // 1. 基础初始化
  static float target_speed = 800.f;
  static float speed_coff = 1.0f;      // 速度系数
  static float rotate_coff = 1.0f;     // 小陀螺旋转频率系数
  static uint8_t x_key_last = 0;       // 记录上一次Space键状态
  static uint8_t f_key_last = 0;       // 记录上一次F键状态
  static uint8_t ctrl_g_key_last = 0;  // 记录上一次Ctrl+G键状态
  static uint8_t c_key_last = 0;       // 记录上一次C键状态
  static uint8_t is_rotate_mode = 0;   // 小陀螺模式标志位
  static float trigger_time = 0;       // 开火时间

  // 2. 云台
  // 2.1 [右键]按住开启自瞄

  if (rc_data->mouse_key.mouse.press_r) {
    if (has_non_zero_data(vision_recv_data)) {
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_VISION;  // 右键自瞄开启
      gimbal_ctrl_cmd->yaw = vision_recv_data->gimbal_receive.yaw;
      gimbal_ctrl_cmd->pitch = vision_recv_data->gimbal_receive.pitch;
    } else {
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;  // 没识别到目标，保持手动
    }
  } else {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;  // 松开右键，手动模式
  }

  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_VISION) {
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kp = 2.0f;
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kd = 0.03f;
  } else if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kp = 0.8f;
    robot->gimbal->yaw_motor->motor_controller.angle_PID.Kd = 0.02f;
  }

  // 2.2 射击控制逻辑,[左键]射击
  if (rc_data->mouse_key.mouse.press_l) {
    // 校验：摩擦轮开启
    if (shoot_ctrl_cmd->friction_mode == FRICTION_ON) {
      // 默认先设为单发
      shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
      // 检查按下的持续时间 (当前时间 - 上次松开的时间/按下起始时间),超过1秒，覆盖为连发
      if (DWT_GetTimeline_s() - trigger_time > 0.3f) {
        shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
      }
    }
  } else {
    if (!switch_is_up(rc_data->rc.mode_switch)) {
      //  鼠标左键松开时
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
      // 记录松开时间
      trigger_time = DWT_GetTimeline_s();
    }
  }

  // 2.3鼠标云台控制
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    gimbal_ctrl_cmd->yaw -= (float)rc_data->mouse_key.mouse.x * 0.002f;    // X轴灵敏度
    gimbal_ctrl_cmd->pitch -= (float)rc_data->mouse_key.mouse.y * 0.002f;  // Y轴灵敏度
    // 云台Pitch限位
    if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE)
      gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
    else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE)
      gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }

  // 3. 功能键
  // [Shift] 键按住小陀螺
  if (abs(rc_data[TEMP].rc.dial) > 20 || rc_data->mouse_key.keyboard.shift) {
    is_rotate_mode = 1;
  } else {
    is_rotate_mode = 0;
  }

  // [X] 单次触发：转 180° 逃跑
  if (rc_data->mouse_key.keyboard.x != x_key_last) {
    if (rc_data->mouse_key.keyboard.x) {
      gimbal_ctrl_cmd->yaw += 180.0f;
    }
    x_key_last = rc_data->mouse_key.keyboard.x;
  }

  if (!rc_data->mouse_key.keyboard.ctrl && rc_data->mouse_key.keyboard.ctrl) {
    if (robot->chassis_fetch_data) {
      robot->chassis_fetch_data->force_refresh_ui = 1;  // 告诉底盘板去刷新UI
    }
  }

  // [C] 单次切换：超级电容开关
  if (rc_data->mouse_key.keyboard.c != c_key_last) {
    if (rc_data->mouse_key.keyboard.c) {
      if (robot->chassis->super_cap->super_cap_ctrl_cmd == BOOST) {
        robot->chassis->super_cap->super_cap_ctrl_cmd = NORMAL;
      } else {
        robot->chassis->super_cap->super_cap_ctrl_cmd = BOOST;
      }
    }
    c_key_last = rc_data->mouse_key.keyboard.c;
  }

  // 4. 核心运动算法
  //  确定 Robot Mode
  if (is_rotate_mode) {  // 小陀螺
    robot->robot_mode = ROBOT_CHASSIS_ROTATE;
  } else {
    robot->robot_mode = ROBOT_CHASSIS_FOLLOW;  // 默认是FOLLOW模式
  }
  // 处理对应模式
  switch (robot->robot_mode) {
    case ROBOT_CHASSIS_ROTATE:
      // 小陀螺频率设置
      rotate_frequency = 0.5f * rotate_coff;

      // 小陀螺原地旋转
      rotate_omega = rotate_frequency * 2.0f * PI;
      chassis_ctrl_cmd->target_yaw += rotate_omega * robot->dt;

      // // 设置目标速度矢量 (vx, vy),单位为m/s
      // if (rc_data[TEMP].key[KEY_PRESS].w)
      //   chassis_vy = 0.5f * speed_coff;
      // else if (rc_data[TEMP].key[KEY_PRESS].s)
      //   chassis_vy = -0.5f * speed_coff;
      // else
      //   chassis_vy = 0.0f;

      // if (rc_data[TEMP].key[KEY_PRESS].d)
      //   chassis_vx = 0.5f * speed_coff;
      // else if (rc_data[TEMP].key[KEY_PRESS].a)
      //   chassis_vx = -0.5f * speed_coff;
      // else
      //   chassis_vx = 0.0f;

      // input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);  // 速度的模

      // // 转换角度坐标系
      // float target_angle_to_gimbal = atan2f(chassis_vy, chassis_vx);  // 目标方向矢量与云台正方向方向夹角
      // float target_angle_to_chassis = target_angle_to_gimbal + robot->offset_angle * DEGREE_2_RAD;

      // // 相位补偿，单位是rad todo:参数，要测试
      // float phase_compensation = 0.03f;
      // // 正弦速度调制
      // chassis_ctrl_cmd->vx = input_mag * sinf(target_angle_to_chassis + phase_compensation);
      break;

    case ROBOT_CHASSIS_FOLLOW:
#if (!defined(ONE_BOARD))
      // 设置目标速度矢量 (vx, vy),单位为m/s
      if (rc_data->mouse_key.keyboard.w)
        chassis_vy += 0.5f * speed_coff;
      else if (rc_data->mouse_key.keyboard.s)
        chassis_vy += -0.5f * speed_coff;
      else
        chassis_vy += 0.0f;

      if (rc_data->mouse_key.keyboard.d)
        chassis_vx += 0.5f * speed_coff;
      else if (rc_data->mouse_key.keyboard.a)
        chassis_vx += -0.5f * speed_coff;
      else
        chassis_vx += 0.0f;

      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
      if (input_mag > 0.0005f) {
        // 运动方向解算
        follow_err = (atan2f(chassis_vy, chassis_vx) - PI / 2.0f) * RAD_2_DEGREE - robot->offset_angle;
        while (follow_err > 180.0f) follow_err -= 360.0f;
        while (follow_err < -180.0f) follow_err += 360.0f;
        // 倒车优化
        if (abs(follow_err) > 90.0f) {
          if (follow_err > 0.0f)
            follow_err -= 180.0f;
          else
            follow_err += 180.0f;
          input_mag = -input_mag;
        }
        // 直接计算目标yaw角度
        // 目标 = 当前yaw - offset_angle + follow_err (让底盘朝向运动方向)
        // 等价于：让底盘转到 gimbal方向 再补偿 follow_err
        chassis_ctrl_cmd->target_yaw = robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD + follow_err * DEGREE_2_RAD;
      } else {
        // 静止回正：让底盘对齐云台 (offset → 0)
        chassis_ctrl_cmd->target_yaw =
            robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD - robot->offset_angle * DEGREE_2_RAD;
      }
      chassis_ctrl_cmd->wz = 0.0f;  // 无前馈角速度
      // 对齐衰减
      align_attenuation = cosf(follow_err * DEGREE_2_RAD);
      if (align_attenuation < 0) align_attenuation = 0;
      input_mag *= align_attenuation * align_attenuation * align_attenuation;
      VAL_LIMIT(input_mag, -2.97, 2.97);
      chassis_ctrl_cmd->vx =
          ramp_controller_update(&chassis_ramp, input_mag, robot->chassis->state_var.x_b_d, robot->dt);
      // chassis_ctrl_cmd->theta_ff = chassis_ramp.expected_a / 9.81f;
      chassis_ctrl_cmd->theta_ff = 0.0f;
      break;
#endif
    case ROBOT_CHASSIS_FREE:
#if (!defined(ONE_BOARD))
      // 双板：静止对齐云台
      chassis_ctrl_cmd->target_yaw =
          robot->chassis->imu->YawTotalAngle * DEGREE_2_RAD - robot->offset_angle * DEGREE_2_RAD;
      chassis_ctrl_cmd->wz = 0.0f;
#endif
      if (rc_data->mouse_key.keyboard.w)
        chassis_ctrl_cmd->vx =
            ramp_controller_update(&chassis_ramp, (0.99f) * speed_coff, robot->chassis->state_var.x_b_d, robot->dt);
      else if (rc_data->mouse_key.keyboard.s)
        chassis_ctrl_cmd->vx =
            ramp_controller_update(&chassis_ramp, (-0.99f) * speed_coff, robot->chassis->state_var.x_b_d, robot->dt);
      else
        chassis_ctrl_cmd->vx = ramp_controller_update(&chassis_ramp, 0.0f, robot->chassis->state_var.x_b_d, robot->dt);

      chassis_ctrl_cmd->theta_ff = 0.0f;
      break;
    default:
      break;
  }
  // 6.更新历史数据
  rc_data_last = rc_data;
}
#endif

#ifdef USE_DUAL_RC
void EmergencyHandler(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;

  if (robot_lost_control) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_RECOVERY;
  }
  // 两switch都在下或者遥控器断连，断电
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
  }  // 底盘失能
  if (switch_is_down(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
  }  // 发射失能
  if (switch_is_down(rc_data[TEMP].rc.switch_left)) {
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }
}
#elifdef USE_DUAL_RC_NEW
void EmergencyHandler(RobotInstance* robot) {
  rc_data = robot->rc_data;
  Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  // 新VT13遥控器紧急处理逻辑
  if (switch_left(rc_data->rc.mode_switch) || rc_data == NULL) {  // 拨杆在左或按下暂停键时断电
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
  // 失控处理
  if (robot_lost_control) {
    if (chassis_ctrl_cmd->chassis_mode != CHASSIS_PROSTRATE) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_RECOVERY;
    }
  }
  // shoot关闭
  if (switch_middle(rc_data->rc.mode_switch)) {  // 扳机按下时发射失能
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }
}
#endif
