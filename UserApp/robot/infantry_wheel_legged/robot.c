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

static RobotInstance *robot;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd;
static Shoot_Ctrl_Cmd_s *shoot_ctrl_cmd;

#if !defined(ONE_BOARD)
static Chassis_Upload_Data_s *chassis_upload_data;
static Chassis_Fetch_Data_s *chassis_fetch_data;
#endif

static RC_ctrl_t *rc_data;
static RC_ctrl_t *rc_data_last;  // 遥控器数据,初始化时返回

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
static float rotate_time;       // 小陀螺旋转时长
static float rotate_omega;      // 小陀螺旋转角速度
static float rotate_T;          // 小陀螺旋转周期
static int rotate_T_flag;       // 小陀螺旋转周期标志，0表示在前二分之一个周期，1表示在后二分之一个周期

#define robot_lost_control abs(robot->chassis->chassis_IMU->Pitch) > 13.0f
/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle() {
  angle = robot->gimbal->yaw_motor->measure.angle_single_round;

#if YAW_CHASSIS_ALIGN_ECD > 4096  // 如果大于180度
  if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
    chassis_ctrl_cmd->offset_angle = angle - YAW_ALIGN_ANGLE;
  else if (angle > 180.0f + YAW_ALIGN_ANGLE)
    chassis_ctrl_cmd->offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
  else
    chassis_ctrl_cmd->offset_angle = angle - YAW_ALIGN_ANGLE;
#else  // 小于180度
  if (angle > YAW_ALIGN_ANGLE)
    chassis_ctrl_cmd->offset_angle = angle - YAW_ALIGN_ANGLE;
  else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
    chassis_ctrl_cmd->offset_angle = angle - YAW_ALIGN_ANGLE;
  else
    chassis_ctrl_cmd->offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
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
      if (DWT_GetTimeline_s() - trigger_time > 2.0f) {
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
    case ROBOT_CHASSIS_ROTATE:
      // // 小陀螺转速/频率设置
      // rotate_frequency = 0.1f;
      //
      // // 小陀螺原地旋转
      // rotate_omega = rotate_frequency * 2.0f * PI;
      // chassis_ctrl_cmd->wz = rotate_omega;
      //
      // // 设置目标速度(vx,vy)
      // chassis_vx = 0.001f * (float)rc_data[TEMP].rc.rocker_r_;
      // chassis_vy = 0.001f * (float)rc_data[TEMP].rc.rocker_r1;
      //
      // // 获取当前所在的二分之一个周期
      // rotate_T = 2.0f * PI / rotate_omega;
      // rotate_T_flag = 0;
      // rotate_time += robot->dt;
      // if (rotate_time >= rotate_T) rotate_T_flag = (rotate_T_flag + 1) % 2;
      //
      // // 每个二分之一周期做对应处理
      // switch (rotate_T_flag) {
      //   case 0:  // 在前半个周期时
      //     chassis_ctrl_cmd->wz += PIDCalculate(&robot->chassis_follow_PID, chassis_ctrl_cmd->offset_angle,
      //                                          atan2f(chassis_vy, chassis_vx) - PI / 2.0f);
      //     slope_following(sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy), &chassis_ctrl_cmd->vx,
      //                     1.0f * robot->dt);
      //     break;
      //   case 1:  // 在后半个周期时
      //     chassis_ctrl_cmd->wz += PIDCalculate(&robot->chassis_follow_PID, chassis_ctrl_cmd->offset_angle,
      //                                          PI / 2.0f - atan2f(chassis_vy, chassis_vx));
      //     slope_following(-1.0f * sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy), &chassis_ctrl_cmd->vx,
      //                     1.0f * robot->dt);
      //     break;
      //   default:
      //     break;
      // }
      // break;
      //
    case ROBOT_CHASSIS_FOLLOW:
#if (!defined(ONE_BOARD))
      // 获取输入
      chassis_vx = 0.0025f * (float)rc_data[TEMP].rc.rocker_l_;  // 水平分量
      chassis_vy = 0.0025f * (float)rc_data[TEMP].rc.rocker_l1;  // 垂直分量
      input_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);

      // 运动逻辑
      if (input_mag > 0.0005f) {
        // (Error = Target - Current)
        follow_err = (atan2f(chassis_vy, chassis_vx) - PI / 2.0f) * RAD_2_DEGREE - chassis_ctrl_cmd->offset_angle;

        // 角度归一化 (-180 ~ 180)，处理过零点问题
        while (follow_err > 180.0f) follow_err -= 360.0f;
        while (follow_err < -180.0f) follow_err += 360.0f;

        // 倒车优化 (如果误差 > 90度，则反向行驶)
        if (abs(follow_err) > 90.0f) {
          if (follow_err > 0.0f)
            follow_err -= 180.0f;
          else
            follow_err += 180.0f;
          input_mag = -input_mag;  // 速度反向
        }
        chassis_ctrl_cmd->wz =
            -0.0035f * (float)rc_data[TEMP].rc.rocker_r_ + PIDCalculate(&robot->chassis_follow_PID, -follow_err, 0);
        ;

      } else {
        // 静止回正逻辑：让底盘车头自动转回云台方向 (Offset -> 0)
        // 此时 PID(Measure=Offset, Target=0)
        chassis_ctrl_cmd->wz = -0.0035f * (float)rc_data[TEMP].rc.rocker_r_ +
                               PIDCalculate(&robot->chassis_follow_PID, chassis_ctrl_cmd->offset_angle, 0);
      }
      align_attenuation = cosf(follow_err * (PI / 180.0f));
      if (align_attenuation < 0) align_attenuation = 0;  // 防御性保护
      input_mag *= align_attenuation * align_attenuation * align_attenuation;

      // if (abs(follow_err) > 5) align_attenuation = 0;  // 防御性保护
      // input_mag *= align_attenuation;

      slope_following(input_mag, &chassis_ctrl_cmd->vx,
                      1.0f * robot->dt);  // 0.0045(最大3m/s)
      chassis_ctrl_cmd->vx = input_mag;
      break;
#endif
    case ROBOT_CHASSIS_FREE:
#if defined(ONE_BOARD)
      static float target_angle;
      target_angle += (-0.25f) * (float)rc_data[TEMP].rc.rocker_r_ * robot->dt;
      chassis_ctrl_cmd->wz =
          -0.0015f * (float)rc_data[TEMP].rc.rocker_r_ +
          PIDCalculate(&robot->chassis_follow_PID, robot->chassis->chassis_IMU->YawTotalAngle, target_angle);
      // chassis_ctrl_cmd->vx = (0.0025f) * (float)rc_data[TEMP].rc.rocker_r1;
#else
      chassis_ctrl_cmd->wz = -0.0035f * (float)rc_data[TEMP].rc.rocker_r_ +
                             PIDCalculate(&robot->chassis_follow_PID, chassis_ctrl_cmd->offset_angle, 0);
      chassis_ctrl_cmd->vx = (0.0025f) * (float)rc_data[TEMP].rc.rocker_r1;
#endif
      // slope_following((0.0045f) * (float)rc_data[TEMP].rc.rocker_r1, &chassis_ctrl_cmd->vx,
      // 1.5f * robot->dt);  // 0.0045(最大3m/s)
      chassis_ctrl_cmd->vx = (0.0025f) * (float)rc_data[TEMP].rc.rocker_r1;
      chassis_ctrl_cmd->roll = 0.0004f * (float)rc_data[TEMP].rc.rocker_l_ * (abs(rc_data[TEMP].rc.rocker_l_) > 10);
      chassis_ctrl_cmd->leg_length += 0.0000005f * (float)rc_data[TEMP].rc.rocker_l1;

      if (chassis_ctrl_cmd->leg_length > LEG_MAX_LENGTH) {
        chassis_ctrl_cmd->leg_length = LEG_MAX_LENGTH;
      } else if (chassis_ctrl_cmd->leg_length < LEG_MIN_LENGTH) {
        chassis_ctrl_cmd->leg_length = LEG_MIN_LENGTH;
      }
      break;
    default:
      break;
  }
  // 发射参数

  // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
  shoot_ctrl_cmd->shoot_rate = 8;

  *rc_data_last = *rc_data;
}

#if 0
/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet() {
  chassis_ctrl_cmd->vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300;  // 系数待测
  chassis_ctrl_cmd->vy = rc_data[TEMP].key[KEY_PRESS].s * 300 - rc_data[TEMP].key[KEY_PRESS].d * 300;

  gimbal_ctrl_cmd->yaw += (float)rc_data[TEMP].mouse.x / 660 * 10;  // 系数待测
  gimbal_ctrl_cmd->pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;

  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3)  // Z键设置弹速
  {
    case 0:
      shoot_ctrl_cmd->bullet_speed = 15;
      break;
    case 1:
      shoot_ctrl_cmd->bullet_speed = 18;
      break;
    default:
      shoot_ctrl_cmd->bullet_speed = 30;
      break;
  }
  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4)  // E键设置发射模式
  {
    case 0:
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
      break;
    case 1:
      shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
      break;
    case 2:
      shoot_ctrl_cmd->load_mode = LOAD_3_BULLET;
      break;
    default:
      shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
      break;
  }
  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2)  // F键开关摩擦轮
  {
    case 0:
      shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
      break;
    default:
      shoot_ctrl_cmd->friction_mode = FRICTION_ON;
      break;
  }
  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4)  // C键设置底盘速度
  {
    case 0:
      chassis_ctrl_cmd->chassis_speed_buff = 40;
      break;
    case 1:
      chassis_ctrl_cmd->chassis_speed_buff = 60;
      break;
    case 2:
      chassis_ctrl_cmd->chassis_speed_buff = 80;
      break;
    default:
      chassis_ctrl_cmd->chassis_speed_buff = 100;
      break;
  }
  switch (rc_data[TEMP].key[KEY_PRESS].shift)  // 待添加 按shift允许超功率 消耗缓冲能量
  {
    case 1:

      break;

    default:

      break;
  }
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
  // MouseKeySet();
  EmergencyHandler();  // 处理模块离线和遥控器急停等紧急情况
#if defined(GIMBAL_BOARD)
  CalcOffsetAngle();
  chassis_fetch_data->chassis_ctrl_cmd = *chassis_ctrl_cmd;
  *chassis_upload_data = *(Chassis_Upload_Data_s *)CANCommGet(robot->can_comm);
  robot->chassis->chassis_IMU->Roll = chassis_upload_data->Roll;
  robot->chassis->chassis_IMU->Pitch = chassis_upload_data->Pitch;
  robot->chassis->chassis_IMU->YawTotalAngle = chassis_upload_data->YawTotalAngle;
  CANCommSend(robot->can_comm, (void *)chassis_fetch_data);
#endif
#elif defined(CHASSIS_BOARD)
  chassis_upload_data->Pitch = robot->chassis->chassis_IMU->Pitch;
  chassis_upload_data->Roll = robot->chassis->chassis_IMU->Roll;
  chassis_upload_data->YawTotalAngle = robot->chassis->chassis_IMU->YawTotalAngle;

  *chassis_fetch_data = *(Chassis_Fetch_Data_s *)CANCommGet(robot->can_comm);
  robot->chassis->chassis_ctrl_cmd = chassis_fetch_data->chassis_ctrl_cmd;
  CANCommSend(robot->can_comm, (void *)chassis_upload_data);
#endif
}

void RobotInit() {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)

  // 遥控器初始化
#if defined(STM32F4)
  robot->rc_data = RemoteControlInit(&huart3);
#elif defined(STM32H7)
  robot->rc_data = RemoteControlInit(&huart5);
#endif
  rc_data_last = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));  // 分配独立内存空间，与robot->rc_data区分开
  *rc_data_last = *robot->rc_data;                         // 记录上一次遥控器的状态，传值确保内存空间独立

  PIDInit(&robot->chassis_follow_PID, &chassis_follow_PID_config);
  rc_data = robot->rc_data;
#if defined(GIMBAL_BOARD)
  robot->gimbal = GimbalInit(&gimbal_init_config);
  robot->shoot = ShootInit(&shoot_init_config);
  gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  robot->chassis_upload_data = (Chassis_Upload_Data_s *)zmalloc(sizeof(Chassis_Upload_Data_s));
  robot->chassis_fetch_data = (Chassis_Fetch_Data_s *)zmalloc(sizeof(Chassis_Fetch_Data_s));
  chassis_upload_data = robot->chassis_upload_data;
  chassis_fetch_data = robot->chassis_fetch_data;
  robot->chassis = (ChassisInstance *)zmalloc(sizeof(ChassisInstance));
  robot->chassis->chassis_IMU = (INS_t *)zmalloc(sizeof(INS_t));
  robot->can_comm = CANCommInit(&gimbal_comm_conf);
#endif
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  // robot->referee_data = RefereeInit(&huart6);  // 裁判系统初始化
  // robot->super_cap = SuperCapInit(&super_cap_config);
  robot->chassis = ChassisInit(&chassis_init_config);
#if defined(CHASSIS_BOARD)
  robot->chassis_upload_data = (Chassis_Upload_Data_s *)zmalloc(sizeof(Chassis_Upload_Data_s));
  robot->chassis_fetch_data = (Chassis_Fetch_Data_s *)zmalloc(sizeof(Chassis_Fetch_Data_s));
  chassis_upload_data = robot->chassis_upload_data;
  chassis_fetch_data = robot->chassis_fetch_data;
  robot->can_comm = CANCommInit(&chassis_comm_conf);  // can comm初始化
#endif
#endif
  chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  chassis_ctrl_cmd->leg_length = chassis_init_config.chassis_param.initial_leg_length;  // 初始腿长
  DWT_GetDeltaT(&robot->DWT_CNT);
}

void RobotTask() {
  robot->dt = DWT_GetDeltaT(&robot->DWT_CNT);
  RobotCMDTask();
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
  // GimbalTask();
  // ShootTask();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  ChassisTask();
#endif
}
