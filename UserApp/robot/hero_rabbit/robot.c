//
// Created by zeg on 2025/12/3.
//

#include "robot.h"

#include "bsp_gpio.h"
#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"
#include "master_process.h"
#include "referee_task.h"

// ---------------- 条件编译头文件 ----------------
#if defined(USE_DUAL_RC_NEW)
#include "new_RC_VT13.h"
#elif defined(USE_DUAL_RC)
#include "remote_control.h"
#endif

static RobotInstance *robot;
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd;
static Shoot_Ctrl_Cmd_s *shoot_ctrl_cmd;
static Vision_Receive_s* vision_recv_data;

// ---------------- 条件编译遥控器结构体 ----------------
#if defined(USE_DUAL_RC_NEW)
static VT13_RC_t *rc_data;
static VT13_RC_t *rc_data_last;  // 遥控器数据,初始化时返回
// 手动为 VT13 实现键盘按键的上升沿计数（替代旧的 key_count）
static uint8_t key_b_count = 0;
static uint8_t key_g_count = 0;
static uint8_t key_q_count = 0;
static uint8_t key_e_count = 0;
static uint8_t key_v_count = 0;
static uint8_t key_r_count = 0;
static uint8_t key_f_count = 0;
#elif defined(USE_DUAL_RC)
static RC_ctrl_t *rc_data;
static RC_ctrl_t *rc_data_last;  // 遥控器数据,初始化时返回
#endif

/* Intermediate variables calculated by private functions */
static float trigger_time = 0;  // 触发时间
static float x_speed_time=0;    // x方向加速触发时间
static float y_speed_time=0;    // y方向加速触发时间
static float vx_initial;        // x轴输入控制量
static float vy_initial;        // y轴输入控制量
static float angle;
float new_max_pitch=0.0f;
float new_min_pitch=0.0f;
static uint8_t leg_up_latch = 0;
static uint8_t leg_down_latch = 0;
external_imu_t *external_imu_instance;

static GPIOInstance *gpio_5V_EN;
static GPIO_Init_Config_s gpio_init_config_5v = {
  .GPIO_Pin = POWER_5V_Pin,
  .GPIOx = POWER_5V_GPIO_Port,
  .pin_state = GPIO_PIN_SET,
};

/**
 * @brief 检查视觉数据非空
 */
uint8_t has_non_zero_data(const Vision_Receive_s* data) {
  if (data == NULL) return 0;
  return (data->gimbal_receive.pitch != 0) ||
         (data->gimbal_receive.yaw != 0) ||
         (data->shoot_receive.fire_flag != 0);
}

/**
 * @brief 计算和零位的误差
 */
static void CalcOffsetAngle() {
  angle = ((uint16_t)robot->gimbal->yaw_motor->measure.angle_single_round +
           (uint16_t)robot->gimbal->yaw_motor->measure.total_round % 2 * 360.0f) /
          2.0f;

  if (angle > YAW_ALIGN_ANGLE)
    chassis_ctrl_cmd->offset_angle = angle - YAW_ALIGN_ANGLE;
  else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
    chassis_ctrl_cmd->offset_angle = angle - YAW_ALIGN_ANGLE;
  else
    chassis_ctrl_cmd->offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 */
static void RemoteControlSet() {
#if defined(USE_DUAL_RC_NEW)
  // ===================== VT13 新遥控器逻辑 =====================
  // 左拨杆（mode_switch）控制基础模式
  if (switch_right(rc_data->rc.mode_switch)) {
    // 拨杆最下，此时系统也会在 EmergencyHandler 触发急停
  } else if (switch_middle(rc_data->rc.mode_switch)) {
    // 拨杆居中
    shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    shoot_ctrl_cmd->friction_mode = FRICTION_ON;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    chassis_ctrl_cmd->SuperCapBoost = 0;
  } else if (switch_left(rc_data->rc.mode_switch)) {
    // 拨杆最上，激活超容与射击就绪
    shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    shoot_ctrl_cmd->friction_mode = FRICTION_ON;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    chassis_ctrl_cmd->SuperCapBoost = 1;
  }

  // 云台控制（遥控器摇杆）
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    gimbal_ctrl_cmd->yaw += -0.003f * (float)rc_data->rc.rocker_r_;
    gimbal_ctrl_cmd->pitch += 0.0006f * (float)rc_data->rc.rocker_r1;
  }

  // PITCH 软限位
  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }

  // 底盘摇杆输入
  vx_initial = 60.0f * (float)rc_data->rc.rocker_l_;
  vy_initial = 60.0f * (float)rc_data->rc.rocker_l1;

  // 底盘特殊模式计算
  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW) {
    chassis_ctrl_cmd->wz = 40.0f * (float)rc_data->rc.rocker_r_;
  }



  // if (rc_data->rc.fn_1) {
  //   chassis_ctrl_cmd->leg_mode = LEG_MANUAL_UP;
  // }
  // else if (rc_data->rc.fn_2) {
  //   chassis_ctrl_cmd->leg_mode = LEG_MANUAL_DOWN;
  // }
  // else {
  //   chassis_ctrl_cmd->leg_mode = LEG_HOLD;
  // }

#elif defined(USE_DUAL_RC)
  // ===================== 旧 DJI 标准遥控器逻辑 =====================
  if (switch_is_down(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->leg_mode = LEG_MANUAL_DOWN;
  } else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->leg_mode = LEG_HOLD;
  } else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->leg_mode = LEG_MANUAL_UP;
  }

  if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
    shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    shoot_ctrl_cmd->friction_mode = FRICTION_ON;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    chassis_ctrl_cmd->SuperCapBoost=0;
  } else if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
    shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    shoot_ctrl_cmd->friction_mode = FRICTION_ON;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    chassis_ctrl_cmd->SuperCapBoost=1;
  }

  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    gimbal_ctrl_cmd->yaw += -0.003f * (float)rc_data[TEMP].rc.rocker_r_;
    gimbal_ctrl_cmd->pitch += 0.0006f * (float)rc_data[TEMP].rc.rocker_r1;
  }

  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }

  vx_initial= 60.0f * (float)rc_data[TEMP].rc.rocker_l_;
  vy_initial = 60.0f * (float)rc_data[TEMP].rc.rocker_l1;

  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_ROTATE) {
    chassis_ctrl_cmd->wz = 25.0f * (float)rc_data[TEMP].rc.dial;
  }
  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW) {
    chassis_ctrl_cmd->wz = 40.0f * (float)rc_data[TEMP].rc.rocker_r_;
  }
#endif
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 */
static void MouseKeySet() {
#if defined(USE_DUAL_RC_NEW)
  // ===================== VT13 键鼠控制逻辑 =====================
  // 1. 键盘提取
  if (chassis_ctrl_cmd->leg_mode == LEG_MANUAL_UP) {
    vy_initial+=1500.0f;
  }
  vy_initial += (float)((rc_data->mouse_key.keyboard.w) - rc_data->mouse_key.keyboard.s) * (float)chassis_ctrl_cmd->chassis_speed_buff;
  vx_initial += (float)(rc_data->mouse_key.keyboard.a - rc_data->mouse_key.keyboard.d) * (float)-chassis_ctrl_cmd->chassis_speed_buff;
  chassis_ctrl_cmd->vx = vx_initial * chassis_ctrl_cmd->chassis_direction;
  chassis_ctrl_cmd->vy = vy_initial * chassis_ctrl_cmd->chassis_direction;

  // 2. 鼠标控制云台
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    gimbal_ctrl_cmd->yaw -= (float)rc_data->mouse_key.mouse.x * 0.007f;
    gimbal_ctrl_cmd->pitch -= (float)rc_data->mouse_key.mouse.y * 0.003f;
    if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
      gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
    } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
      gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
    }
  }
  // 3. 按键边缘检测 (捕获上升沿)
  if (rc_data->mouse_key.keyboard.r && !rc_data_last->mouse_key.keyboard.r) key_r_count++;
  if (rc_data->mouse_key.keyboard.f && !rc_data_last->mouse_key.keyboard.f) key_f_count++;
  if (rc_data->mouse_key.keyboard.b && !rc_data_last->mouse_key.keyboard.b) key_b_count++;
  if (rc_data->mouse_key.keyboard.q && !rc_data_last->mouse_key.keyboard.q) key_q_count++;
  if (rc_data->mouse_key.keyboard.e && !rc_data_last->mouse_key.keyboard.e) key_e_count++;
  if (rc_data->mouse_key.keyboard.v && !rc_data_last->mouse_key.keyboard.v) key_v_count++;

  // ==================== 腿部控制与高度限位逻辑 ====================

  // 1. 提取当前与上一帧按键状态，简化代码判断
  uint8_t r_now = rc_data->mouse_key.keyboard.r;
  uint8_t r_last = rc_data_last->mouse_key.keyboard.r;
  uint8_t f_now = rc_data->mouse_key.keyboard.f;
  uint8_t f_last = rc_data_last->mouse_key.keyboard.f;
  uint8_t ctrl_now = rc_data->mouse_key.keyboard.ctrl;




  // 3. 组合键：捕获 Ctrl + F 上升沿 -> 循环切换高度限位
  if (f_now && !f_last && ctrl_now) {
    if (chassis_ctrl_cmd->leg_limit == FIRST_STEP) {
      chassis_ctrl_cmd->leg_limit = SECOND_STEP;
    } else {
      chassis_ctrl_cmd->leg_limit = FIRST_STEP;
    }
    // 注意：如果你希望按 Ctrl+F 切换限位的同时也打断上升锁存，
    // 可以解除下一行的注释：
    // leg_up_latch = 0;
  }
  if (rc_data->mouse_key.keyboard.r && !rc_data_last->mouse_key.keyboard.r) {
    // 如果当前已经是上升状态，就取消它（HOLD）；否则切换为上升
    if (chassis_ctrl_cmd->leg_mode == LEG_MANUAL_UP) {
      chassis_ctrl_cmd->leg_mode = LEG_HOLD;
    } else {
      chassis_ctrl_cmd->leg_mode = LEG_MANUAL_UP;
    }
  }

  // 捕获 F 键刚按下的瞬间（上升沿）
  else if (rc_data->mouse_key.keyboard.f && !rc_data_last->mouse_key.keyboard.f&&!ctrl_now) {
    // 如果当前已经是下降状态，就取消它（HOLD）；否则切换为下降
    if (chassis_ctrl_cmd->leg_mode == LEG_MANUAL_DOWN) {
      chassis_ctrl_cmd->leg_mode = LEG_HOLD;
    } else {
      chassis_ctrl_cmd->leg_mode = LEG_MANUAL_DOWN;
    }
  }


  // ==================== 处理 G 键及 Ctrl+G ====================
  //捕获 G 键的上升沿（刚刚按下的那一瞬间）
  if (rc_data->mouse_key.keyboard.g && !rc_data_last->mouse_key.keyboard.g) {

    // 按下 G 的时候，检查 Ctrl 是否已经被按住了
    if (rc_data->mouse_key.keyboard.ctrl) {
      if (shoot_ctrl_cmd->fire_mode == MANUAL_FIRE) shoot_ctrl_cmd->fire_mode = VISION_FIRE;
      else if (shoot_ctrl_cmd->fire_mode == VISION_FIRE) shoot_ctrl_cmd->fire_mode = MANUAL_FIRE;

    } else {
      // 触发单键：单纯按下 G
      // 走原来的单键逻辑：累加计数器，交给后面的 switch 处理单发/连发或底盘方向
      key_g_count++;
    }
  }
  // ==================== 【 C 键与 Ctrl+C 组合逻辑 】 ====================
  // 捕获 C 键的上升沿（刚刚按下的那一瞬间）
  if (rc_data->mouse_key.keyboard.c && !rc_data_last->mouse_key.keyboard.c) {

    // 判断此时 Ctrl 键是否已经被按住
    if (rc_data->mouse_key.keyboard.ctrl) {
      chassis_ctrl_cmd->leg_clear_error = LEG_CLEAR_ERROR;

    }
    else {

      // 触发：单按 C 键
      Referee_Interactive_info_t* ui_data = getUI();
      if (ui_data != NULL) ui_data->force_refresh_ui = 1;
      // 例如进行状态翻转计数：key_c_count++;

    }
  }
  
  switch (rc_data->mouse_key.mouse.press_r) {
    case 1:
      if (has_non_zero_data(vision_recv_data)==1) {
        gimbal_ctrl_cmd->gimbal_mode = GIMBAL_VISION;
        gimbal_ctrl_cmd->yaw = vision_recv_data->gimbal_receive.yaw;
        gimbal_ctrl_cmd->pitch = vision_recv_data->gimbal_receive.pitch;
      } else {
        gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
      }
      break;
    default:
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
      break;
  }

  // 6. 鼠标左键：射击逻辑
  switch (rc_data->mouse_key.mouse.press_l) {
    case 0:
      if (!switch_left(rc_data->rc.mode_switch)) {
        shoot_ctrl_cmd->load_mode = LOAD_STOP;
        trigger_time = DWT_GetTimeline_s();
      }
      break;
    default:
          switch (shoot_ctrl_cmd->fire_mode) {
      case MANUAL_FIRE:
            if (shoot_ctrl_cmd->friction_mode == FRICTION_ON) {
              shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
              if (DWT_GetTimeline_s() - trigger_time > 1.0f) {
                shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
              }
            }
              break;
      case VISION_FIRE:
              if (shoot_ctrl_cmd->friction_mode == FRICTION_ON&&vision_recv_data->shoot_receive.fire_flag==1) {
                shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
              }

               break;
          }
      break;
  }

  // 7. 小陀螺与底盘跟随切换
  switch (key_v_count % 2) {
    case 1:
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
      chassis_ctrl_cmd->wz = 5000.0f;
      if (rc_data->mouse_key.keyboard.b == 1 && rc_data_last->mouse_key.keyboard.b == 0) {
        gimbal_ctrl_cmd->yaw += 180.0f;
        if (key_b_count % 2 == 1) key_b_count++;
      }
      break;
    case 0:
      switch (key_b_count % 2) {
        case 0:
          if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW_REAR_END) gimbal_ctrl_cmd->yaw += 180.0f;
          chassis_ctrl_cmd->wz = 0.0f;
          chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
          break;
        case 1:
          if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW) gimbal_ctrl_cmd->yaw += 180.0f;
          chassis_ctrl_cmd->wz = 0.0f;
          chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW_REAR_END;
          break;
        default: break;
      }
      break;
    default: break;
  }



  switch (key_g_count % 2) {
    case 0: chassis_ctrl_cmd->chassis_direction = 1.0f; break;
    case 1: chassis_ctrl_cmd->chassis_direction = -1.0f; break;
    default: break;
  }
  switch (key_q_count % 3) {
    case 0: shoot_ctrl_cmd->heat_mode = REFEREE_CONTROL; break;
    case 1: shoot_ctrl_cmd->heat_mode = SIMULLATE_CONTROL; break;
    case 2: shoot_ctrl_cmd->heat_mode = NO_CONTROL; break;
    default: break;
  }

  switch (key_e_count % 3) {
    case 0: shoot_ctrl_cmd->bullet_speed_mode = DISABLE_BULLET_SPEED; break;
    case 1: shoot_ctrl_cmd->bullet_speed_mode = MANUAL_BULLET_SPEED;
      if (!rc_data_last->mouse_key.keyboard.z &&rc_data->mouse_key.keyboard.z ) {
        shoot_ctrl_cmd->friction_speed += 150.0f;
      }
      else if (!rc_data_last->mouse_key.keyboard.x &&rc_data->mouse_key.keyboard.x) {
        shoot_ctrl_cmd->friction_speed -= 150.0f;
      }
      break; // 待填
    case 2: shoot_ctrl_cmd->bullet_speed_mode = ENABLE_BULLET_SPEED; break;
    default: break;
  }
  if (rc_data->mouse_key.keyboard.shift == 1) {
    chassis_ctrl_cmd->SuperCapBoost=1;
  }
  else {
    chassis_ctrl_cmd->SuperCapBoost=0;
  }
  //调试用补丁
  if (rc_data->rc.trigger&&!rc_data_last->rc.trigger) {
    if (shoot_ctrl_cmd->friction_mode == FRICTION_ON) {
      shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
    }
  }
  // 保存这一帧状态
  *rc_data_last = *rc_data;

#elif defined(USE_DUAL_RC)
  // ===================== 旧 DJI 标准键鼠逻辑 =====================
  vy_initial += (float)((rc_data[TEMP].key[KEY_PRESS].w) - rc_data[TEMP].key[KEY_PRESS].s) * (float) chassis_ctrl_cmd->chassis_speed_buff;
  vx_initial += (float)(rc_data[TEMP].key[KEY_PRESS].a - rc_data[TEMP].key[KEY_PRESS].d) * (float) -chassis_ctrl_cmd->chassis_speed_buff;
  chassis_ctrl_cmd->vx = vx_initial * chassis_ctrl_cmd->chassis_direction;
  chassis_ctrl_cmd->vy = vy_initial * chassis_ctrl_cmd->chassis_direction;

  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    gimbal_ctrl_cmd->yaw -= (float)rc_data[TEMP].mouse.x * 0.007f;
    gimbal_ctrl_cmd->pitch -= (float)rc_data[TEMP].mouse.y * 0.003f;
    if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
      gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
    } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
      gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
    }
  }

  if (rc_data[TEMP].key[KEY_PRESS].r) chassis_ctrl_cmd->leg_mode = LEG_MANUAL_UP;
  else if (rc_data[TEMP].key[KEY_PRESS].f) chassis_ctrl_cmd->leg_mode = LEG_MANUAL_DOWN;

  if (rc_data_last[TEMP].key[KEY_PRESS].x==0 && rc_data[TEMP].key[KEY_PRESS].x==1) {
    if (chassis_ctrl_cmd->leg_limit == FIRST_STEP) chassis_ctrl_cmd->leg_limit = SECOND_STEP;
    else if (chassis_ctrl_cmd->leg_limit == SECOND_STEP) chassis_ctrl_cmd->leg_limit = FIRST_STEP;
  }

  if (!rc_data_last[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].ctrl) {
    Referee_Interactive_info_t* ui_data = getUI();
    if (ui_data != NULL) ui_data->force_refresh_ui = 1;
  }

  switch (rc_data[TEMP].mouse.press_r % 2) {
    case 1:
      if (has_non_zero_data(vision_recv_data)==1){
        gimbal_ctrl_cmd->gimbal_mode=GIMBAL_VISION;
        gimbal_ctrl_cmd->yaw=vision_recv_data->gimbal_receive.yaw;
        gimbal_ctrl_cmd->pitch=vision_recv_data->gimbal_receive.pitch;
      } else gimbal_ctrl_cmd->gimbal_mode=GIMBAL_ON;
      break;
    default: break;
  }

  switch (rc_data[TEMP].mouse.press_l % 2) {
    case 0:
      if (!switch_is_up(rc_data[TEMP].rc.switch_left)) {
        shoot_ctrl_cmd->load_mode=LOAD_STOP;
        trigger_time = DWT_GetTimeline_s();
      }
      break;
    default:
      switch (rc_data[TEMP].key_count[KEY_PRESS][Key_G] % 2) {
      case 0:
          if (shoot_ctrl_cmd->friction_mode==FRICTION_ON) {
            shoot_ctrl_cmd->load_mode=LOAD_1_BULLET;
            if (DWT_GetTimeline_s() - trigger_time > 1.0f) shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
          }
          break;
      default:
          if (shoot_ctrl_cmd->friction_mode==FRICTION_ON) shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
          break;
      }
      break;
  }

  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_V]%2) {
    case 1:
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
      chassis_ctrl_cmd->wz = 5000.0f;
      if (rc_data[TEMP].key[KEY_PRESS].b==1 && rc_data_last[TEMP].key[KEY_PRESS].b==0) {
            gimbal_ctrl_cmd->yaw += 180.0f;
        if (rc_data[TEMP].key_count[KEY_PRESS][Key_B]% 2==1) rc_data[TEMP].key_count[KEY_PRESS][Key_B]++;
      }
      break;
    case 0:
      if (abs(rc_data[TEMP].rc.dial) > 50) break;
      switch (rc_data[TEMP].key_count[KEY_PRESS][Key_B] % 2) {
      case 0:
          if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW_REAR_END) gimbal_ctrl_cmd->yaw += 180.0f;
          chassis_ctrl_cmd->wz = 0.0f;
          chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
          break;
      case 1:
          if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW) gimbal_ctrl_cmd->yaw += 180.0f;
          chassis_ctrl_cmd->wz = 0.0f;
          chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW_REAR_END;
          break;
      default: break;
      }
      break;
      default: break;
  }

  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Q] % 3) {
    case 0: shoot_ctrl_cmd->heat_mode=REFEREE_CONTROL; break;
    case 1: shoot_ctrl_cmd->heat_mode=SIMULLATE_CONTROL; break;
    case 2: shoot_ctrl_cmd->heat_mode=NO_CONTROL; default: break;
  }

  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_G] % 2) {
    case 0: chassis_ctrl_cmd->chassis_direction=1.0f; break;
    case 1: chassis_ctrl_cmd->chassis_direction=-1.0f; break;
    default: break;
  }

   switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 3) {
     case 0: shoot_ctrl_cmd->bullet_speed_mode=ENABLE_BULLET_SPEED; break;
     case 1: break;
     case 2: shoot_ctrl_cmd->bullet_speed_mode=DISABLE_BULLET_SPEED; default: break;
   }

  *rc_data_last = *rc_data;
#endif
}

/**
 * @brief  紧急停止 (重要模块离线/遥控器触发)
 */
static void EmergencyHandler() {
#if defined(USE_DUAL_RC_NEW)
  // VT13 失联 或 mode_switch 拨下 作为急停依据
  if (switch_right(rc_data->rc.mode_switch) || !VT13RemoteIsOnline()) {
    robot->robot_mode = ROBOT_POWER_OFF;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    chassis_ctrl_cmd->leg_mode = LEG_DISABLE;
    chassis_ctrl_cmd->SuperCapBoost = 0;
    LOGERROR("[CMD] emergency stop!");
  } else {
    LOGINFO("[CMD] reinstate, robot ready");
  }

#elif defined(USE_DUAL_RC)
  // 两switch都在下断电 (原代码逻辑)
  if (switch_is_down(rc_data[TEMP].rc.switch_left) || !RemoteControlIsOnline()) {
    robot->robot_mode = ROBOT_POWER_OFF;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    chassis_ctrl_cmd->leg_mode = LEG_DISABLE;
    chassis_ctrl_cmd->SuperCapBoost = 0;
    LOGERROR("[CMD] emergency stop!");
  } else {
    LOGINFO("[CMD] reinstate, robot ready");
  }
#endif
}

void RobotInit() {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

#if defined(USE_DUAL_RC_NEW)
  #ifdef STM32F4
  robot->rc_data = (void *)VT13RemoteInit(&huart3);
  #elif defined(STM32H7)
  robot->rc_data = (void *)VT13RemoteInit(&huart7);
  #endif
  rc_data_last = (VT13_RC_t *)zmalloc(sizeof(VT13_RC_t));
  rc_data = (VT13_RC_t *)robot->rc_data;
  *rc_data_last = *rc_data;  // 记录上一次遥控器的状态
#elif defined(USE_DUAL_RC)
  #ifdef STM32F4
  robot->rc_data = (void *)RemoteControlInit(&huart3);
  #elif defined(STM32H7)
  robot->rc_data = (void *)RemoteControlInit(&huart5);
  #endif
  rc_data_last = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
  rc_data = (RC_ctrl_t *)robot->rc_data;
  *rc_data_last = *rc_data;  // 记录上一次遥控器的状态
#endif

  robot->referee_data = RefereeInit(&huart1);  // 裁判系统初始化
  robot->super_cap = SuperCapInit(&super_cap_config);
  robot->chassis = ChassisInit(&chassis_init_config);

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
  robot->gimbal = GimbalInit(&gimbal_init_config);
  robot->shoot = ShootInit(&shoot_init_config);
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  robot->chassis->super_cap = robot->super_cap;
#endif

  // 初始化控制命令指针
  chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  chassis_ctrl_cmd->power_distribute = 1.0f;
  gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;

  shoot_ctrl_cmd->bullet_speed_mode = DISABLE_BULLET_SPEED;
  shoot_ctrl_cmd->heat_mode=REFEREE_CONTROL;
  shoot_ctrl_cmd->fire_mode=MANUAL_FIRE;
  vision_recv_data = VisionInit(&gimbal_init_config.imu_init_config);

  gpio_5V_EN = GPIORegister(&gpio_init_config_5v);
  GPIOSet(gpio_5V_EN);

  chassis_ctrl_cmd->chassis_speed_buff = 20000;
  chassis_ctrl_cmd->chassis_direction = 1.0f;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
  shoot_ctrl_cmd->initial_speed = robot->referee_data->ShootData.initial_speed;
  shoot_ctrl_cmd->shooter_barrel_heat = robot->referee_data->PowerHeatData.shooter_42mm_barrel_heat;
  shoot_ctrl_cmd->shooter_barrel_heat_limit=robot->referee_data->GameRobotState.shooter_barrel_heat_limit;
  chassis_ctrl_cmd->max_power = robot->referee_data->GameRobotState.chassis_power_limit;
  //chassis_ctrl_cmd->max_power = 180;

  CalcOffsetAngle();
  RemoteControlSet();
  MouseKeySet();
  EmergencyHandler();  // 处理模块离线和遥控器急停等紧急情况
}

void RobotTask() {
  VisionSend();
  RobotCMDTask();
  GimbalTask();
  ShootTask();
  ChassisTask();
  SuperCapSendMessage(robot->super_cap,
      (int16_t)robot->referee_data->GameRobotState.chassis_power_limit,
      robot->referee_data->PowerHeatData.buffer_energy,
      robot->referee_data->GameRobotState.power_management_chassis_output);
}

RobotInstance* getRobot() {
  return robot;
}