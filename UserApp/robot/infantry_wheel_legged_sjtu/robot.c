/**
******************************************************************************
* @file    robot.c
* @author  Enhao Zhang
* @date    2025/8/8
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief Infantry wheel-legged robot control module
******************************************************************************
* @attention
* None
*
******************************************************************************
*/

#include "robot.h"

#include "bsp_gpio.h"
#include "can_comm.h"
#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"

static RobotInstance* robot;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd;
static Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd;
static Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd;
static Vision_Receive_s* vision_recv_data;

#if !defined(ONE_BOARD)
static Chassis_Upload_Data_s* chassis_upload_data;
static Chassis_Fetch_Data_s* chassis_fetch_data;
#endif

static RC_ctrl_t* rc_data;
static RC_ctrl_t* rc_data_last;  // 遥控器数据,初始化时返回

/* Intermediate variables calculated by private functions */
static float trigger_time = 0;  // 触发时间
static float angle;

static float chassis_vx;         // x轴输入控制量
static float chassis_vy;         // y轴输入控制量
static float input_mag;          // 输入的模值
static float follow_err;         // follow最终计算出的角度误差
static float align_attenuation;  // 对齐时的衰减系数

// 小陀螺相关参数
static float rotate_frequency;  // 小陀螺旋转的频率
static float rotate_omega;      // 小陀螺旋转角速度

// vofa数据
float visualized_data[20];

void VOFATask() {
#if defined(GIMBAL_BOARD)
#elif defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  visualized_data[0] = robot->chassis->power_ctrl.P_total;
  visualized_data[2] = robot->chassis->power_ctrl.vel_max;
  visualized_data[4] = robot->chassis->power_ctrl.P_limit;
  visualized_data[5] = robot->chassis->limited_vx;           // 限制后速度
  visualized_data[6] = robot->chassis->chassis_ctrl_cmd.vx;  // 原始指令速度
  visualized_data[7] = robot->chassis->state_var.v_b_h;      // 实际速度
  // 新增：旋转占比（调试小陀螺功率分配）
  float w_L = robot->chassis->leg[1]->wheel_motor->measure.speed_aps * DEGREE_2_RAD;
  float w_R = robot->chassis->leg[0]->wheel_motor->measure.speed_aps * DEGREE_2_RAD;
  float w_sum = fabsf(w_L + w_R);
  float w_diff = fabsf(w_L - w_R);
  visualized_data[8] = (w_sum + w_diff > 0.1f) ? w_diff / (w_sum + w_diff) : 0.0f;  // rotate_ratio
  visualized_data[9] = robot->chassis->power_ctrl.P_wheel_L;
  visualized_data[10] = robot->chassis->power_ctrl.P_wheel_R;
#endif
  VOFAJustFloatSend(visualized_data, 20);
}

Ramp_Controller_t chassis_ramp = {
    .planning_v = 0.0f,
    .max_v = 2.5f,
    .max_accel = 1.0f,
    .accel_base_speed = 0.3f,
    .max_decel = 4.5f,
    .min_decel = 1.0f,
    .decel_base_speed = 0.8f,
};

#define robot_lost_control abs(robot->chassis->imu->Pitch) > 13.0f

/**
 * @brief  检查视觉接收数据是否有效（即是否识别到目标）
 * @note   该函数通过检测 Pitch、Yaw 或开火指令是否非零，来判断视觉是否处于正在跟踪状态。
 *         如果指针为空或所有关键数据均为0，则认为未识别到目标。
 *
 * @param  data 指向视觉接收数据结构体 (Vision_Receive_s) 的常量指针
 * @retval 1: 存在有效数据 (非零，表示识别到目标)
 * @retval 0: 指针为空 或 数据全为零 (未识别到目标)
 */
uint8_t has_non_zero_data(const Vision_Receive_s* data) {
  // 空指针检查
  if (data == NULL) {
    return 0;  // 或根据需求返回错误码
  }

  // 简化逻辑：只要任意字段非零，返回1；否则返回0
  return (data->gimbal_receive.pitch != 0) || (data->gimbal_receive.yaw != 0) || (data->shoot_receive.fire_flag != 0);
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle() {
  angle = robot->gimbal->yaw_motor->measure.angle_single_round;

#if YAW_CHASSIS_ALIGN_ECD > 4096  // 如果大于180度
  if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
    robot->offset_angle = angle - YAW_ALIGN_ANGLE;
  else if (angle > 180.0f + YAW_ALIGN_ANGLE)
    robot->offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
  else
    robot->offset_angle = angle - YAW_ALIGN_ANGLE;
#else  // 小于180度
  if (angle > YAW_ALIGN_ANGLE)
    robot->offset_angle = angle - YAW_ALIGN_ANGLE;
  else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
    robot->offset_angle = angle - YAW_ALIGN_ANGLE;
  else
    robot->offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet() {
  // 右[中]，底盘使能 ROBOT_CHASSIS_FOLLOW
  if (switch_is_mid(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    if (abs(rc_data[TEMP].rc.dial) > 20) {
      robot->robot_mode = ROBOT_CHASSIS_ROTATE;
    } else {
      robot->robot_mode = ROBOT_CHASSIS_FOLLOW;
    }
  }
  // 右[上]，底盘使能，允许跳跃 ROBOT_CHASSIS_FREE
  else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    if (abs(rc_data[TEMP].rc.dial) > 20) {
      robot->robot_mode = ROBOT_CHASSIS_ROTATE;
    } else {
      robot->robot_mode = ROBOT_CHASSIS_FREE;
    }

    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_JUMP_READY;
    }
    if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_JUMP_START;
      chassis_ctrl_cmd->jump_force = 15 * JUMP_FORCE;
      // chassis_ctrl_cmd->jump_force = 0;
    }
  }
  if (!switch_is_up(rc_data[TEMP].rc.switch_right)) {
    // 左[中],云台启动，摩擦轮启动，准备射击
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
      shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
      shoot_ctrl_cmd->friction_mode = FRICTION_ON;
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
      // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
    }
    // 左[上]，开火，发射，根据时间判断单发或者连发
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
      shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
      shoot_ctrl_cmd->friction_mode = FRICTION_ON;
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
      if (switch_is_mid(rc_data_last[TEMP].rc.switch_left)) {
        trigger_time = DWT_GetTimeline_s();
      }
      if (DWT_GetTimeline_s() - trigger_time > 1.0f) {
        shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
      } else {
        shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
      }
    }
    // 云台使能,或视觉未识别到目标,纯遥控器拨杆控制
    if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {  // 按照摇杆的输出大小进行角度增量,增益系数需调整
      gimbal_ctrl_cmd->pitch -= 0.0003f * (float)rc_data[TEMP].rc.rocker_r1;
    }
  } else {
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {  // 按照摇杆的输出大小进行角度增量,增益系数需调整
    gimbal_ctrl_cmd->yaw += -0.0005f * (float)rc_data[TEMP].rc.rocker_r_;
  }
  // 云台PITCH轴软件限位 todo:没在云台有点不好
  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }

  switch (robot->robot_mode) {
    case ROBOT_CHASSIS_ROTATE: {
      // 小陀螺频率设置
      rotate_frequency = 1.0f;

      // 小陀螺原地旋转
      rotate_omega = rotate_frequency * 2.0f * PI;
      // chassis_ctrl_cmd->wz = PIDCalculate(&robot->chassis_rotate_PID, robot->chassis->imu->Gyro[2], rotate_omega);
      chassis_ctrl_cmd->wz = rotate_omega;

      // 设置目标速度矢量(vx,vy)
      chassis_vx = 0.0025f * (float)rc_data[TEMP].rc.rocker_l_;
      chassis_vy = 0.0025f * (float)rc_data[TEMP].rc.rocker_l1;
      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);  // 速度的模

      // 转换角度坐标系
      float target_angle_to_gimbal = atan2f(chassis_vy, chassis_vx);  // 目标方向矢量与云台正方向方向夹角
      float target_angle_to_chassis = target_angle_to_gimbal + robot->offset_angle * DEGREE_2_RAD;

      // 相位补偿，单位是rad todo:参数，要测试
      float phase_compensation = 1.0f;
      // 速度补偿
      float speed_compensation = -0.05f * rotate_omega;
      // 正弦速度调制
      chassis_ctrl_cmd->vx = input_mag * sinf(target_angle_to_chassis + phase_compensation) + speed_compensation;
      break;
    }
    case ROBOT_CHASSIS_FOLLOW: {
#if (!defined(ONE_BOARD))
      // 获取输入
      chassis_vx = 0.0045f * (float)rc_data[TEMP].rc.rocker_l_;
      chassis_vy = 0.0045f * (float)rc_data[TEMP].rc.rocker_l1;
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
      chassis_ctrl_cmd->vx = ramp_controller_update(&chassis_ramp, input_mag, robot->dt);
      // chassis_ctrl_cmd->theta_ff = chassis_ramp.expected_a / 9.81f;
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
      chassis_ctrl_cmd->vx =
          ramp_controller_update(&chassis_ramp, (0.0045f) * (float)rc_data[TEMP].rc.rocker_r1, robot->dt);
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
  //  记录上一次数据，开键鼠的话注释掉
  // *rc_data_last = *rc_data;
}

#if 1
/**
 * @brief 键鼠控制逻辑 (适配轮腿机器人)
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                        鼠标控制 - 瞄准与射击                              │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ 鼠标[移动]         : 控制云台 Pitch/Yaw 轴运动                             │
 * │ 鼠标[左键]         : 射击                                                 │
 * │                      - 按下 < 0.2s : 单发                                │
 * │                      - 按下 > 0.2s : 自动转连发                           │
 * │ 鼠标[右键](按住)   : 开启视觉自瞄 (GIMBAL_VISION)，松开恢复手动              │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                        基础移动 - WASD                                   │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ [W] / [S]          : 底盘 前进 / 后退                                     │
 * │ [A] / [D]          : 底盘 左移 / 右移                                     │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                        功能模式切换                                       │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ [Shift](按住)      : 开启小陀螺模式，松开退出                               │
 * │ [Space]            : 跳跃控制                                             │
 * │                      - 按下 : 进入跳跃蓄力 (CHASSIS_JUMP_READY)           │
 * │                      - 松开 : 执行跳跃   (CHASSIS_JUMP_START)            │
 * │ [X]                : 单次触发，云台偏转 180° 逃跑                           │
 * │ [Ctrl+G]           : 单次触发，切换倒地自起模式 (CHASSIS_RECOVERY)          │
 * │                      自起完成后底盘自动恢复 CHASSIS_ON                     │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                        姿态与腿长控制                                     │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ [Q](按住)          : 机身向左侧倾 (Roll -)                                 │
 * │ [E](按住)          : 机身向右侧倾 (Roll +)                                 │
 * │ [Ctrl+R](按住)     : 升高腿长                                              │
 * │ [Ctrl+F](按住)     : 降低腿长                                              │
 * │                      腿长范围限制: [LEG_MIN_LENGTH, LEG_MAX_LENGTH]       │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                        参数动态调整                                       │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ [C]                : 单次触发，切换超级电容开关                              │
 * │ [F]                : 单次触发，切换飞坡模式 (待实现)                         │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                        速度系数 (随功率档位自动切换)                        │
 * ├──────────────────┬──────────────────┬────────────────────────────────────┤
 * │  功率档位         │  speed_coff      │  rotate_coff                       │
 * ├──────────────────┼──────────────────┼────────────────────────────────────┤
 * │  ≤ 45W           │  1.0             │  1.0                               │
 * │  ≤ 60W           │  1.25            │  1.25                              │
 * │  ≤ 80W           │  1.5             │  1.5                               │
 * │  > 80W           │  2.0             │  2.0                               │
 * └──────────────────┴──────────────────┴────────────────────────────────────┘
 *
 * @note 需在 RobotCMDTask 中周期调用
 */
static void MouseKeySet() {
  // 1. 基础初始化
  static float speed_coff = 1.0f;      // 速度系数
  static float rotate_coff = 1.0f;     // 小陀螺旋转频率系数
  static uint8_t space_key_last = 0;   // 记录上一次Space键状态
  static uint8_t x_key_last = 0;       // 记录上一次Space键状态
  static uint8_t f_key_last = 0;       // 记录上一次F键状态
  static uint8_t ctrl_g_key_last = 0;  // 记录上一次Ctrl+R键状态
  static uint8_t c_key_last = 0;       // 记录上一次C键状态
  static uint8_t is_rotate_mode = 0;   // 小陀螺模式标志位

  // 根据功率动态调整参数
  if (chassis_ctrl_cmd->max_power <= 45) {
    speed_coff = 1.0f;
    rotate_coff = 1.0f;
  } else if (chassis_ctrl_cmd->max_power <= 60) {
    speed_coff = 1.25f;
    rotate_coff = 1.25f;
  } else if (chassis_ctrl_cmd->max_power <= 80) {
    speed_coff = 1.5f;
    rotate_coff = 1.5f;
  } else {
    speed_coff = 2.0f;
    rotate_coff = 2.0f;
  }

  // 2. 云台
  // 2.1 [右键]按住开启自瞄
  if (rc_data[TEMP].mouse.press_r) {
    if (has_non_zero_data(vision_recv_data) == 1) {
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
    gimbal_ctrl_cmd->pitch += (float)rc_data[TEMP].mouse.y * 0.001f;  // Y轴灵敏度
    // 云台Pitch限位
    if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE)
      gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
    else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE)
      gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }

  // 3. 功能键
  // [Shift] 键按住小陀螺
  if (rc_data[TEMP].key[KEY_PRESS].shift) {
    is_rotate_mode = 1;
  } else {
    is_rotate_mode = 0;
  }
  // [Space] 键跳跃逻辑：按住蹲下，松开跳跃
  if (rc_data[TEMP].key[KEY_PRESS].space != space_key_last) {
    if (rc_data[TEMP].key[KEY_PRESS].space) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_JUMP_READY;
    } else {
      if (chassis_ctrl_cmd->chassis_mode == CHASSIS_JUMP_READY) {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_JUMP_START;
        chassis_ctrl_cmd->jump_force = 15 * JUMP_FORCE;
      }
    }
    space_key_last = rc_data[TEMP].key[KEY_PRESS].space;
  }
  // [X] 单次触发：转 180° 逃跑
  if (rc_data[TEMP].key[KEY_PRESS].x != x_key_last) {
    if (rc_data[TEMP].key[KEY_PRESS].x) {
      gimbal_ctrl_cmd->yaw += PI;
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
    chassis_ctrl_cmd->roll -= 0.1f;
  } else if (rc_data[TEMP].key[KEY_PRESS].e) {
    chassis_ctrl_cmd->roll += 0.1f;
  }
  // [Ctrl+R] 持续按住：升高腿长，[Ctrl+F] 持续按住：下降腿长
  if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].r) {
    chassis_ctrl_cmd->leg_length += 0.0000005f * 330.0f;
  } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].f) {
    chassis_ctrl_cmd->leg_length -= 0.0000005f * 330.0f;
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
      // 小陀螺旋转频率设置
      rotate_frequency = 0.5f * rotate_coff;

      // 小陀螺原地旋转
      rotate_omega = rotate_frequency * 2.0f * PI;
      // chassis_ctrl_cmd->wz = PIDCalculate(&robot->chassis_rotate_PID, robot->chassis->imu->Gyro[2], rotate_omega);
      chassis_ctrl_cmd->wz = rotate_omega;

      // 设置目标速度矢量 (vx, vy),单位为m/s
      if (rc_data[TEMP].key[KEY_PRESS].w)
        chassis_vx += 0.5f * speed_coff;
      else if (rc_data[TEMP].key[KEY_PRESS].s)
        chassis_vx += -0.5f * speed_coff;
      else
        chassis_vx += 0.0f;

      if (rc_data[TEMP].key[KEY_PRESS].a)
        chassis_vy += 0.5f * speed_coff;
      else if (rc_data[TEMP].key[KEY_PRESS].d)
        chassis_vy += -0.5f * speed_coff;
      else
        chassis_vy += 0.0f;

      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);  // 速度的模

      // 转换角度坐标系
      float target_angle_to_gimbal = atan2f(chassis_vy, chassis_vx);  // 目标方向矢量与云台正方向方向夹角
      float target_angle_to_chassis = target_angle_to_gimbal + robot->offset_angle * DEGREE_2_RAD;

      // 相位补偿，单位是rad todo:参数，要测试
      float phase_compensation = 1.0f;
      // 速度补偿，单位是m/s
      float speed_compensation = -0.05f * rotate_omega;
      // 正弦速度调制
      chassis_ctrl_cmd->vx += input_mag * sinf(target_angle_to_chassis + phase_compensation) + speed_compensation;
      break;

    case ROBOT_CHASSIS_FOLLOW:
#if (!defined(ONE_BOARD))
      // 设置目标速度矢量 (vx, vy),单位为m/s
      if (rc_data[TEMP].key[KEY_PRESS].w)
        chassis_vx += 0.5f * speed_coff;
      else if (rc_data[TEMP].key[KEY_PRESS].s)
        chassis_vx += -0.5f * speed_coff;
      else
        chassis_vx += 0.0f;

      if (rc_data[TEMP].key[KEY_PRESS].a)
        chassis_vy += 0.5f * speed_coff;
      else if (rc_data[TEMP].key[KEY_PRESS].d)
        chassis_vy += -0.5f * speed_coff;
      else
        chassis_vy += 0.0f;

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
      chassis_ctrl_cmd->vx = ramp_controller_update(&chassis_ramp, input_mag, robot->dt);
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
        chassis_ctrl_cmd->vx += ramp_controller_update(&chassis_ramp, (0.99f) * speed_coff, robot->dt);
      else if (rc_data[TEMP].key[KEY_PRESS].s)
        chassis_ctrl_cmd->vx += ramp_controller_update(&chassis_ramp, (-0.99f) * speed_coff, robot->dt);
      else
        chassis_ctrl_cmd->vx += 0.0f;

      chassis_ctrl_cmd->theta_ff = 0.0f;
      break;
    default:
      break;
  }
  // 6.更新历史数据(遥控器有的话这里就不用)
  *rc_data_last = *rc_data;
}
#endif

/**
 * @brief  紧急停止,包括遥控器右拨杆往下/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler() {
  if (robot_lost_control) {
    robot->chassis->chassis_ctrl_cmd.chassis_mode = CHASSIS_RECOVERY;  // todo:因该写成elif比较安全
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

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
  // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
  RemoteControlSet();
  MouseKeySet();
  EmergencyHandler();  // 处理模块离线和遥控器急停等紧急情况
#if defined(GIMBAL_BOARD)
  CalcOffsetAngle();
  chassis_fetch_data->chassis_ctrl_cmd = *chassis_ctrl_cmd;
  *chassis_upload_data = *(Chassis_Upload_Data_s*)CANCommGet(robot->can_comm);
  robot->chassis->imu->Roll = chassis_upload_data->Roll;
  robot->chassis->imu->Pitch = chassis_upload_data->Pitch;
  robot->chassis->imu->YawTotalAngle = chassis_upload_data->YawTotalAngle;
  robot->chassis->imu->Gyro[2] = chassis_upload_data->YawSpeed;
  robot->referee_data->ShootData.initial_speed = robot->referee_data->ShootData.initial_speed;

  CANCommSend(robot->can_comm, (void*)chassis_fetch_data);
#endif
#elif defined(CHASSIS_BOARD)
  chassis_upload_data->Pitch = robot->chassis->imu->Pitch;
  chassis_upload_data->Roll = robot->chassis->imu->Roll;
  chassis_upload_data->YawTotalAngle = robot->chassis->imu->YawTotalAngle;
  chassis_upload_data->YawSpeed = robot->chassis->imu->Gyro[2];
  chassis_upload_data->bullet_speed = robot->referee_data->ShootData.initial_speed;

  *chassis_fetch_data = *(Chassis_Fetch_Data_s*)CANCommGet(robot->can_comm);
  robot->chassis->chassis_ctrl_cmd = chassis_fetch_data->chassis_ctrl_cmd;
  CANCommSend(robot->can_comm, (void*)chassis_upload_data);
#endif
}

void RobotInit() {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)

  // 遥控器初始化
#if defined(STM32F4)
  robot->rc_data = RemoteControlInit(&huart3);
#elif defined(STM32H7)
  robot->rc_data = RemoteControlInit(&huart5);
#endif
  rc_data_last = (RC_ctrl_t*)zmalloc(sizeof(RC_ctrl_t));  // 分配独立内存空间，与robot->rc_data区分开
  *rc_data_last = *robot->rc_data;                        // 记录上一次遥控器的状态，传值确保内存空间独立

  PIDInit(&robot->chassis_rotate_PID, &chassis_rotate_PID_config);
  rc_data = robot->rc_data;
#if defined(GIMBAL_BOARD)
  robot->gimbal = GimbalInit(&gimbal_init_config);
  robot->shoot = ShootInit(&shoot_init_config);
  gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  vision_recv_data = VisionInit(&gimbal_init_config.imu_init_config);

  robot->chassis_upload_data = (Chassis_Upload_Data_s*)zmalloc(sizeof(Chassis_Upload_Data_s));
  robot->chassis_fetch_data = (Chassis_Fetch_Data_s*)zmalloc(sizeof(Chassis_Fetch_Data_s));
  chassis_upload_data = robot->chassis_upload_data;
  chassis_fetch_data = robot->chassis_fetch_data;

  robot->chassis = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));
  robot->chassis->imu = (INS_t*)zmalloc(sizeof(INS_t));
  robot->can_comm = CANCommInit(&gimbal_comm_conf);
#endif
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  robot->referee_data = RefereeInit(&huart7);  // 裁判系统初始化
  // robot->super_cap = SuperCapInit(&super_cap_config);
  robot->chassis = ChassisInit(&chassis_init_config);
#if defined(CHASSIS_BOARD)
  chassis_ctrl_cmd->max_power = robot->referee_data->GameRobotState.chassis_power_limit;

  robot->chassis_upload_data = (Chassis_Upload_Data_s*)zmalloc(sizeof(Chassis_Upload_Data_s));
  robot->chassis_fetch_data = (Chassis_Fetch_Data_s*)zmalloc(sizeof(Chassis_Fetch_Data_s));
  chassis_upload_data = robot->chassis_upload_data;
  chassis_fetch_data = robot->chassis_fetch_data;
  robot->can_comm = CANCommInit(&chassis_comm_conf);  // can comm初始化
  VOFAInit(&huart1);
#endif
#endif
  chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  chassis_ctrl_cmd->leg_length = chassis_init_config.param.initial_leg_length;  // 初始腿长
  DWT_GetDeltaT(&robot->DWT_CNT);
  chassis_ctrl_cmd->max_power = 60;  // 测试用
}

void RobotTask() {
  robot->dt = DWT_GetDeltaT(&robot->DWT_CNT);
  RobotCMDTask();
  VOFATask();
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
  GimbalTask();
  ShootTask();
  VisionSend();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  // ChassisTask();
#endif
}
