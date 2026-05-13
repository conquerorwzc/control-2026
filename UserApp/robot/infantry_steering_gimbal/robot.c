#include "robot.h"

#include "general_def.h"
#include "can_comm.h"
#include "master_process.h"
#include "robot_config.h"
#include "user_lib.h"
#include "rm_referee.h"
#include "arm_math.h"
#ifdef USE_UI
#include "referee_task.h"
#endif
upload_data* upload;
static RobotInstance *robot;
Int16ToBytes transmit_data;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
  Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd;
static Shoot_Ctrl_Cmd_s *shoot_ctrl_cmd;
Vision_Receive_s* vision_recv_data;
#ifdef USE_VT13
static VT13_RC_t *rc_data;
static VT13_RC_t *rc_data_last;  // VT13遥控器数据,初始化时返回
#else
static RC_ctrl_t *rc_data;
static RC_ctrl_t *rc_data_last;  // 遥控器数据,初始化时返回
#endif
//static Chassis_Ctrl_CanComm* chassis_ctrl_can_comm;
static CanComm_Pack* cancomm_pack;
CANCommInstance* can_comm_instance = NULL;

/* Intermediate variables calculated by private functions */
static float trigger_time = 0;  // 触发时间
static float x_speed_time=0;  //x方向加速触发时间
static float y_speed_time=0;  //y方向加速触发时间
static float vx_initial;   //x轴输入控制量
static float vy_initial;   //y轴输入控制量
static float angle;
// static  DJIMotorInstance* debug_motor;
// float noise_05(void)
// {
//     // rand() ∈ [0, RAND_MAX] → 映射到 [-0.5, 0.5]
//     return ((float)rand() / (float)RAND_MAX) - 0.5f;
// }

// 连续混乱曲线 + ±0.5 噪声
float chaos_func(float x)
{
    float t = 3.1415926f * x;

    float y =
        arm_cos_f32(t)
        + 0.5f   * arm_cos_f32(3.0f*t)
        + 0.25f  * arm_cos_f32(9.0f*t)
        + 0.125f * arm_cos_f32(27.0f*t)
        + 0.0625f* arm_cos_f32(81.0f*t)
        + 0.03125f*arm_cos_f32(243.0f*t);

    // 映射到 [1,24]
    float out = 5.75f * y + 12.5f;
   // float out = 5.75f * y + 12.5f;

    // 叠加 ±0.5 噪声
   // out += noise_05();

    return y;
}
/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
uint8_t has_non_zero_data(const Vision_Receive_s* data) {
  // 空指针检查
  if (data == NULL) {
    return 0;  // 或根据需求返回错误码
  }

  // 简化逻辑：只要任意字段非零，返回1；否则返回0
  return (data->gimbal_receive.pitch != 0) ||
         (data->gimbal_receive.yaw != 0) ||
         (data->shoot_receive.fire_flag != 0);
}
// static void CalcOffsetAngle() {
//   angle = (uint16_t)robot->gimbal->yaw_motor->measure.angle_single_round;
//   float delta =YAW_ALIGN_ANGLE - angle;
//   chassis_ctrl_cmd->offset_angle = delta;
//
//   if (chassis_ctrl_cmd->offset_angle > 180.0f) {
//     chassis_ctrl_cmd->offset_angle -= 360.0f;
//   } else if (chassis_ctrl_cmd->offset_angle <= -180.0f) {
//     chassis_ctrl_cmd->offset_angle += 360.0f;
//   }
// }
static void CalcOffsetAngle() {
  angle = (uint16_t)robot->gimbal->yaw_motor->measure.angle_single_round;
  float delta = angle-YAW_ALIGN_ANGLE;
  if (delta > 180.0f) {
    delta -= 360.0f;
  } else if (delta <= -180.0f) {
    delta += 360.0f;
  }
  if (abs(delta) < 2.0f) {
    delta =0.0f;
  }
   chassis_ctrl_cmd->offset_angle = delta;
}
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
#ifdef USE_VT13
/* ======================== VT13 遥控器版本 ======================== */
static void RemoteControlSet() {
  // mode_switch: L=0 云台+底盘跟随, M=1 云台+射击准备, R=2 开火
  if (switch_left(rc_data[TEMP].rc.mode_switch)) {
    // 左: 云台+底盘跟随
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    chassis_ctrl_cmd->SuperCapBoost = 0;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    if (abs(rc_data[TEMP].rc.dial) > 20) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    } else {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
    }
  } else if (switch_middle(rc_data[TEMP].rc.mode_switch)) {
    // 中: 云台+摩擦轮启动+射击准备
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    chassis_ctrl_cmd->SuperCapBoost = 0;
    shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
    shoot_ctrl_cmd->friction_mode = FRICTION_ON;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    if (abs(rc_data[TEMP].rc.dial) > 20) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    } else {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
    }
  } else if (switch_right(rc_data[TEMP].rc.mode_switch)) {
    // 右: 超电+开火(根据trigger判断单发/连发)
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    chassis_ctrl_cmd->SuperCapBoost = 1;
    shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
    shoot_ctrl_cmd->friction_mode = FRICTION_ON;
    if (abs(rc_data[TEMP].rc.dial) > 20) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    } else {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
    }
    // trigger按键判断单发/连发
    if (rc_data[TEMP].rc.trigger) {
      if (switch_middle(rc_data_last[TEMP].rc.mode_switch)) {
        trigger_time = DWT_GetTimeline_s();
      }
      if (DWT_GetTimeline_s() - trigger_time > 1.0f) {
        shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
      } else {
        shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
      }
    } else {
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
    }
  }

  // 云台使能,遥控器摇杆控制
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    gimbal_ctrl_cmd->yaw += -0.0013f * (float)rc_data[TEMP].rc.rocker_r_;
    gimbal_ctrl_cmd->pitch -= 0.0003f * (float)rc_data[TEMP].rc.rocker_r1;
  }

  // 云台PITCH轴软件限位
  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }

  // 底盘参数
  vx_initial = 60.0f * (float)rc_data[TEMP].rc.rocker_l_;
  vy_initial = 60.0f * (float)rc_data[TEMP].rc.rocker_l1;
  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_ROTATE) {
    chassis_ctrl_cmd->wz = 5000;
  }
  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW) {
    chassis_ctrl_cmd->wz = (25.0f) * (float)rc_data[TEMP].rc.rocker_r_;
  }

  *rc_data_last = *rc_data;
}

static void MouseKeySet() {
  vy_initial += (float)((rc_data[TEMP].key[KEY_PRESS].w) - rc_data[TEMP].key[KEY_PRESS].s) *
                (float)chassis_ctrl_cmd->chassis_speed_buff;
  vx_initial += (float)(rc_data[TEMP].key[KEY_PRESS].a - rc_data[TEMP].key[KEY_PRESS].d) *
                (float)-chassis_ctrl_cmd->chassis_speed_buff;
  if (rc_data[TEMP].key[KEY_PRESS].shift != 0 || abs(rc_data[TEMP].rc.dial) > 20) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    chassis_ctrl_cmd->wz = 25000;
  } else {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
    chassis_ctrl_cmd->wz = (50.0f) * (float)rc_data[TEMP].mouse_key.mouse.x +
                           (25.0f) * (float)rc_data[TEMP].rc.rocker_r_;
  }
  // 缓加速
  if (abs(vx_initial) <= 10000) {
    x_speed_time = DWT_GetTimeline_s();
    chassis_ctrl_cmd->vx = vx_initial;
  }
  if (vx_initial > 10000) {
    chassis_ctrl_cmd->vx = 10000 + (DWT_GetTimeline_s() - x_speed_time) * 10000;
  }
  if (vx_initial < -10000) {
    chassis_ctrl_cmd->vx = -10000 - (DWT_GetTimeline_s() - x_speed_time) * 10000;
  }
  if (abs(vy_initial) <= 10000) {
    y_speed_time = DWT_GetTimeline_s();
    chassis_ctrl_cmd->vy = vy_initial;
  }
  if (vy_initial > 10000) {
    chassis_ctrl_cmd->vy = 10000 + (DWT_GetTimeline_s() - y_speed_time) * 10000;
  }
  if (vy_initial < -10000) {
    chassis_ctrl_cmd->vy = -10000 - (DWT_GetTimeline_s() - y_speed_time) * 10000;
  }

  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    gimbal_ctrl_cmd->yaw -= (float)rc_data[TEMP].mouse_key.mouse.x * 0.003f;
    gimbal_ctrl_cmd->pitch -= (float)rc_data[TEMP].mouse_key.mouse.y * 0.003f;
  }

  // 右键进入自瞄模式
  switch (rc_data[TEMP].mouse_key.mouse.press_r % 2) {
    case 1:
      if (has_non_zero_data(vision_recv_data) == 1) {
        gimbal_ctrl_cmd->gimbal_mode = GIMBAL_VISION;
        gimbal_ctrl_cmd->yaw = vision_recv_data->gimbal_receive.yaw;
        gimbal_ctrl_cmd->pitch = vision_recv_data->gimbal_receive.pitch;
      } else {
        gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
      }
      break;
    default:
      break;
  }

  // 左键发射
  switch (rc_data[TEMP].mouse_key.mouse.press_l % 2) {
    case 0:
      if (!switch_right(rc_data[TEMP].rc.mode_switch)) {
        shoot_ctrl_cmd->load_mode = LOAD_STOP;
        trigger_time = DWT_GetTimeline_s();
      }
      break;
    default:
      switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2) {
        case 0:
          if (shoot_ctrl_cmd->friction_mode == FRICTION_ON) {
            shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
            if (DWT_GetTimeline_s() - trigger_time > 0.3f) {
              shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
            }
          }
          break;
        default:
          if (shoot_ctrl_cmd->friction_mode == FRICTION_ON)
            shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
          break;
      }
      break;
  }

  // C键设置底盘速度
  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) {
    case 0:
      chassis_ctrl_cmd->chassis_speed_buff = 10000;
      break;
    case 1:
      chassis_ctrl_cmd->chassis_speed_buff = 20000;
      break;
    case 2:
      chassis_ctrl_cmd->chassis_speed_buff = 40000;
      break;
    default:
      chassis_ctrl_cmd->chassis_speed_buff = 80000;
      break;
  }

  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }
}

static void EmergencyHandler() {
  // VT13: pause按键 或 拨杆断连 触发急停
  if (rc_data[TEMP].rc.pause) {
    robot->robot_mode = ROBOT_POWER_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    for (int i = 0; i < 16; i++)
      rc_data[TEMP].key_count[KEY_PRESS][i] = 0;
    LOGERROR("[CMD] emergency stop!");
    return;
  }
  LOGINFO("[CMD] reinstate, robot ready");

  // mode_switch左: 底盘+发射失能
  if (switch_left(rc_data[TEMP].rc.mode_switch)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }
  // mode_switch中: 云台+射击使能
  if (switch_middle(rc_data[TEMP].rc.mode_switch)) {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
    if (gimbal_ctrl_cmd->gimbal_mode != GIMBAL_VISION)
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
  }
  // mode_switch右: 全部使能+超电
  if (switch_right(rc_data[TEMP].rc.mode_switch)) {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
    if (gimbal_ctrl_cmd->gimbal_mode != GIMBAL_VISION)
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
  }
}

#else
/* ======================== 旧DR16遥控器版本 ======================== */
static void RemoteControlSet() {
  // 右[中]，云台
  if (switch_is_mid(rc_data[TEMP].rc.switch_right))
  {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    chassis_ctrl_cmd->SuperCapBoost =0;
    if (abs(rc_data[TEMP].rc.dial) > 20) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    } else
      chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
  }
  // 右[上]，超电，保持底盘跟随云台
  else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    chassis_ctrl_cmd->SuperCapBoost =1;
    if (abs(rc_data[TEMP].rc.dial) > 20) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    } else
      chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
  }
  // 左[中],云台启动，摩擦轮启动，拨弹盘启动，准备射击
  if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
    shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    shoot_ctrl_cmd->friction_mode = FRICTION_ON;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
    // ...
  }
  else if (switch_is_up(rc_data[TEMP].rc.switch_left))  // 开火，发射，根据时间判断单发或者连发
  {
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
    gimbal_ctrl_cmd->yaw += -0.0013f * (float)rc_data[TEMP].rc.rocker_r_;
    gimbal_ctrl_cmd->pitch -= 0.0003f * (float)rc_data[TEMP].rc.rocker_r1;
  }

  // 云台PITCH轴软件限位 todo:没在云台有点不好
  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }

  // 底盘参数,系数需要调整
  vx_initial = -60.0f * (float)rc_data[TEMP].rc.rocker_l1;  // l_水平方向
  vy_initial = 60.0f * (float)rc_data[TEMP].rc.rocker_l_;  // l1竖直方向
  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_ROTATE) {
    chassis_ctrl_cmd->wz =5000;
        //60.0f * (float)rc_data[TEMP].rc.dial;  // 小陀螺模式下的旋转分量，如果是跟随，则在底盘任务中计算旋转分量
  }
  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW) {
    chassis_ctrl_cmd->wz =(25.0f) *(float)rc_data[TEMP].rc.rocker_r_;  // 主动跟随量，todo：但是感觉一个变量拆成两段写好像有点抽象，这里有一段，chassis还有另一段
  }

  // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
  //shoot_ctrl_cmd->shoot_rate = 8;

  *rc_data_last = *rc_data;
}

static void MouseKeySet() {
  vy_initial += (float)((rc_data[TEMP].key[KEY_PRESS].w) - rc_data[TEMP].key[KEY_PRESS].s) *
                (float)chassis_ctrl_cmd->chassis_speed_buff;
  vx_initial += (float)(rc_data[TEMP].key[KEY_PRESS].a - rc_data[TEMP].key[KEY_PRESS].d) *
                (float)-chassis_ctrl_cmd->chassis_speed_buff;
    if (rc_data[TEMP].key[KEY_PRESS].shift!=0||abs(rc_data[TEMP].rc.dial) > 20)
    {
        chassis_ctrl_cmd->chassis_mode=CHASSIS_ROTATE;
        chassis_ctrl_cmd->wz =  25000;
        //chassis_ctrl_cmd->wz =  7000*sinf(DWT_GetTimeline_s()*8.f)+20000;
        //chassis_ctrl_cmd->wz =  3500*chaos_func(DWT_GetTimeline_s()*4.f)+20000;
        //chassis_ctrl_cmd->wz =  40000*powf(sinf(DWT_GetTimeline_s()*8.0f),3.f);
       // chassis_ctrl_cmd->SuperCapBoost=1;
       // (float)chassis_ctrl_cmd->chassis_speed_buff+60.0f * (float)rc_data[TEMP].rc.dial;  // 小陀螺模式下的旋转分量，如果是跟随，则在底盘任务中计算旋转分量
    }
    else
    {
        chassis_ctrl_cmd->chassis_mode=CHASSIS_FOLLOW;
        chassis_ctrl_cmd->wz =(50.0f) *(float)rc_data[TEMP].mouse.x+(25.0f) *(float)rc_data[TEMP].rc.rocker_r_;
    }
  // 缓加速
  if (abs(vx_initial)<=10000) {
    x_speed_time=DWT_GetTimeline_s();
    chassis_ctrl_cmd->vx=vx_initial;
  }//速度绝对值在10000以下输出控制量=输入控制量
  if (vx_initial > 10000 ) {
    chassis_ctrl_cmd->vx=10000+(DWT_GetTimeline_s()-x_speed_time)*10000;
  }
  if (vx_initial < -10000) {
    chassis_ctrl_cmd->vx=-10000-(DWT_GetTimeline_s()-x_speed_time)*10000;
  }//速度绝对值在10000以上输出控制量=10000+10000t(s)
  if (abs(vy_initial)<=10000) {
    y_speed_time=DWT_GetTimeline_s();
    chassis_ctrl_cmd->vy=vy_initial;
  }//速度绝对值在10000以下输出控制量=输入控制量
  if (vy_initial > 10000 ) {
    chassis_ctrl_cmd->vy=10000+(DWT_GetTimeline_s()-y_speed_time)*10000;
  }
  if (vy_initial < -10000) {
    chassis_ctrl_cmd->vy=-10000-(DWT_GetTimeline_s()-y_speed_time)*10000;
  }//速度绝对值在10000以上输出控制量=10000+10000t(s)

if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON)
  {
    gimbal_ctrl_cmd->yaw -= (float)rc_data[TEMP].mouse.x * 0.003f;  // 横向灵敏度调节
    gimbal_ctrl_cmd->pitch -= (float)rc_data[TEMP].mouse.y * 0.003f; // 纵向灵敏度调节 (负号反转Y轴)
  }
  // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3)  // Z键设置弹速
  // {
  //   case 0:
  //     shoot_ctrl_cmd->bullet_speed = 15;
  //     break;
  //   case 1:
  //     shoot_ctrl_cmd->bullet_speed = 18;
  //     break;
  //   default:
  //     shoot_ctrl_cmd->bullet_speed = 30;
  //     break;
  // }
  switch (rc_data[TEMP].mouse.press_r % 2) {  //右键进入自瞄预备模式
  case 1:
      if (has_non_zero_data(vision_recv_data)==1){
        gimbal_ctrl_cmd->gimbal_mode=GIMBAL_VISION;    // 右键自瞄开启
        gimbal_ctrl_cmd->yaw=vision_recv_data->gimbal_receive.yaw;
        gimbal_ctrl_cmd->pitch=vision_recv_data->gimbal_receive.pitch;
        //shoot_ctrl_cmd->load_mode=vision_recv_data->shoot_receive.fire_flag;
      }
      else
        gimbal_ctrl_cmd->gimbal_mode=GIMBAL_ON;      //人工操控模式
      break;
  default:
      break;
  }
  switch (rc_data[TEMP].mouse.press_l % 2)        // 左键发射
  {
  case 0:
      if (!switch_is_up(rc_data[TEMP].rc.switch_left))
      {
        shoot_ctrl_cmd->load_mode=LOAD_STOP;
        trigger_time = DWT_GetTimeline_s();
      }
      break;
  default:
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2)  // E键设置发射模式
    {
      case 0:                                              //单发+长按连发
        if (shoot_ctrl_cmd->friction_mode==FRICTION_ON)   //需预先开启摩擦轮，F键
        {
            shoot_ctrl_cmd->load_mode=LOAD_1_BULLET;
          if (DWT_GetTimeline_s() - trigger_time > 0.3f)  //长按检测，1秒
          {
            shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
          }
          break;
          default:                                         //连发
          if (shoot_ctrl_cmd->friction_mode==FRICTION_ON)
          shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
          break;
        }
    }
      break;
  }

  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4)  // C键设置底盘速度
  {
    case 0:
      chassis_ctrl_cmd->chassis_speed_buff = 10000;
      break;
    case 1:
      chassis_ctrl_cmd->chassis_speed_buff = 20000;
      break;
    case 2:
      chassis_ctrl_cmd->chassis_speed_buff = 40000;
      break;
    default:
      chassis_ctrl_cmd->chassis_speed_buff = 80000;
      break;
  }
  // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Q]%2) //新增Q自旋开启
  // {
  //   case 0:
  //     chassis_ctrl_cmd-> chassis_mode = CHASSIS_FOLLOW ;
  //     chassis_ctrl_cmd->wz+=(float)rc_data[TEMP].mouse.x * 30.0f; //主动跟随量
  //     break;
  //   default:
  //     chassis_ctrl_cmd-> chassis_mode = CHASSIS_ROTATE ;
  //     chassis_ctrl_cmd->wz+=5000;
  //     break;
  // }

  switch (rc_data[TEMP].key[KEY_PRESS].shift)  // 待添加 按shift允许超功率 消耗缓冲能量
  {
    case 1:

      break;

    default:

      break;
  }
 // shoot_ctrl_cmd->shoot_rate = 8;// 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler() {
  // 两switch都在下断电
  if ((switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left))||switch_is_off(rc_data[TEMP].rc.switch_left)||switch_is_off(rc_data[TEMP].rc.switch_right))  // 全部失能
  {
    robot->robot_mode = ROBOT_POWER_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    for (int i=0;i<16;i++)
      rc_data[TEMP].key_count[KEY_PRESS][i]=0;  //复位    注意：更改键位的时候要对这里以及下面的复位进行大改。
    LOGERROR("[CMD] emergency stop!");
  } else {
    LOGINFO("[CMD] reinstate, robot ready");
  }
  if (switch_is_down(rc_data[TEMP].rc.switch_right)||switch_is_off(rc_data[TEMP].rc.switch_right))  // 底盘失能
  {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
  }
  else
  {
    gimbal_ctrl_cmd->gimbal_mode=GIMBAL_ON;
  }
  if (switch_is_down(rc_data[TEMP].rc.switch_left)||switch_is_off(rc_data[TEMP].rc.switch_left))  // 发射失能
  {
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }
  else {
    shoot_ctrl_cmd->shoot_mode= SHOOT_ON;
    if (gimbal_ctrl_cmd->gimbal_mode!=GIMBAL_VISION)  //增加自瞄状态的优先级
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
  }
  // 遥控器右侧开关为[上],恢复正常运行
}
#endif

// void Steering_CANCommSend(CANCommInstance *can_comm_instance, RC_ctrl_t *rc_data)
// {
//   static Int16ToBytes transmit_data;
//
//   if (can_comm_instance == NULL || rc_data == NULL)
//   {
//     return;
//   }
//
//   transmit_data.value = rc_data->rc.rocker_l_;
//   can_comm_instance->can_ins->tx_buff[0] = transmit_data.bytes[0];
//   can_comm_instance->can_ins->tx_buff[1] = transmit_data.bytes[1];
//
//   transmit_data.value = rc_data->rc.rocker_l1;
//   can_comm_instance->can_ins->tx_buff[2] = transmit_data.bytes[0];
//   can_comm_instance->can_ins->tx_buff[3] = transmit_data.bytes[1];
//
//   transmit_data.value = rc_data->rc.rocker_r_;
//   can_comm_instance->can_ins->tx_buff[4] = transmit_data.bytes[0];
//   can_comm_instance->can_ins->tx_buff[5] = transmit_data.bytes[1];

  // transmit_data.value = rc_data->rc.rocker_r1;
  // can_comm_instance->can_ins->tx_buff[6] = transmit_data.bytes[0];
  // can_comm_instance->can_ins->tx_buff[7] = transmit_data.bytes[1];
  //
  // transmit_data.value = rc_data->rc.dial;
  // can_comm_instance->can_ins->tx_buff[8] = transmit_data.bytes[0];
  // can_comm_instance->can_ins->tx_buff[9] = transmit_data.bytes[1];

  // can_comm_instance->can_ins->tx_buff[10] = rc_data->rc.switch_left;
//   can_comm_instance->can_ins->tx_buff[6] = rc_data->rc.switch_right;
//
//   CANCommSend(can_comm_instance, can_comm_instance->can_ins->tx_buff);
// }

void RobotInit() {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

#ifdef USE_VT13
  #ifdef STM32F407xx
    robot->rc_data = VT13RemoteInit(&huart3);
  #elifdef STM32H723XX
    robot->rc_data = VT13RemoteInit(&huart5);
  #endif
  rc_data_last = (VT13_RC_t *)zmalloc(sizeof(VT13_RC_t));
#else
  #ifdef STM32F407xx
    robot->rc_data = RemoteControlInit(&huart3);
  #elifdef STM32H723XX
    robot->rc_data = RemoteControlInit(&huart5);
  #endif
  rc_data_last = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
#endif
  *rc_data_last = *robot->rc_data;  // 记录上一次遥控器的状态

  // robot->referee_data = RefereeInit(&huart6);  // 裁判系统初始化

  // robot->super_cap = SuperCapInit(&super_cap_config);

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
  robot->gimbal = GimbalInit(&gimbal_init_config);
  robot->shoot = ShootInit(&shoot_init_config);
  robot->referee_data = GetReferee();
#endif
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  robot->chassis = ChassisInit(&chassis_init_config);
#endif

  // 初始化控制命令指针
  //chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  chassis_ctrl_cmd=(Chassis_Ctrl_Cmd_s*)zmalloc(sizeof(Chassis_Ctrl_Cmd_s));
  cancomm_pack=(CanComm_Pack*)zmalloc(sizeof(CanComm_Pack));
  chassis_ctrl_cmd->max_power = 10;  // 随便给一个初始功率，后面应该要从裁判系统获取
  gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  rc_data = robot->rc_data;
  vision_recv_data=VisionInit(&gimbal_init_config.imu_init_config);
    shoot_ctrl_cmd->heat_mode=REFEREE_CONTROL;
    shoot_ctrl_cmd->bullet_speed_mode=ENABLE_BULLET_SPEED;
  can_comm_instance = CANCommInit(&comm_config);
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
  // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
  CalcOffsetAngle();
  RemoteControlSet();
  ///if (rc_data->rc.rocker_l1==0&&rc_data->rc.rocker_r1==0&&rc_data->rc.rocker_l_==0&&rc_data->rc.rocker_r_==0)
    MouseKeySet();
    shoot_ctrl_cmd->initial_speed = upload->bullet_speed;
    shoot_ctrl_cmd->shooter_barrel_heat=upload->Heat;
  EmergencyHandler();  // 处理模块离线和遥控器急停等紧急情况
}

void RobotTask() {
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
  upload=(upload_data*)CANCommGet(can_comm_instance);
  robot->referee_data->ShootData.initial_speed=upload->bullet_speed;
    robot->referee_data->GameRobotState.robot_id=upload->color;

  VisionSend();
  RobotCMDTask();
  GimbalTask();
  ShootTask();
  //Steering_CANCommSend(can_comm_instance, rc_data);
  // transmit_data.value = rc_data->rc.rocker_l_;
  // board_can_comm_data.tx_buff[0] = transmit_data.bytes[0];
  // board_can_comm_data.tx_buff[1] = transmit_data.bytes[1];
  //
  // transmit_data.value = rc_data->rc.rocker_l1;
  // board_can_comm_data.tx_buff[2] = transmit_data.bytes[0];
  // board_can_comm_data.tx_buff[3] = transmit_data.bytes[1];
  //
  // transmit_data.value = rc_data->rc.rocker_r_;
  // board_can_comm_data.tx_buff[4] = transmit_data.bytes[0];
  // board_can_comm_data.tx_buff[5] = transmit_data.bytes[1];
  //
  // transmit_data.value = rc_data->rc.dial;
  // board_can_comm_data.tx_buff[6] = transmit_data.bytes[0];
  // board_can_comm_data.tx_buff[7] = transmit_data.bytes[1];
  //
  // transmit_data.value = (int16_t)robot->gimbal->yaw_motor->measure.angle_single_round;
  // board_can_comm_data.tx_buff[8] = transmit_data.bytes[0];
  // board_can_comm_data.tx_buff[9] = transmit_data.bytes[1];
  //
  // board_can_comm_data.tx_buff[10] = rc_data->rc.switch_right;
  cancomm_pack->chassis_ctrl_can_comm.vx=chassis_ctrl_cmd->vx;
  cancomm_pack->chassis_ctrl_can_comm.vy=chassis_ctrl_cmd->vy;
  cancomm_pack->chassis_ctrl_can_comm.wz=chassis_ctrl_cmd->wz;
  cancomm_pack->chassis_ctrl_can_comm.chassis_mode=chassis_ctrl_cmd->chassis_mode;
  cancomm_pack->chassis_ctrl_can_comm.SuperCapBoost=chassis_ctrl_cmd->SuperCapBoost;
  cancomm_pack->chassis_ctrl_can_comm.offset_angle=chassis_ctrl_cmd->offset_angle;
  cancomm_pack->friction_mode=(uint8_t)robot->shoot->shoot_ctrl_cmd.friction_mode;
  cancomm_pack->pitch=(int16_t)gimbal_ctrl_cmd->pitch;
  cancomm_pack->friction_speed1=(uint16_t)robot->shoot->friction_motor[0]->measure.speed_aps;
  cancomm_pack->friction_speed2=(uint16_t)(-robot->shoot->friction_motor[1]->measure.speed_aps);
  cancomm_pack->load_mode=robot->shoot->shoot_ctrl_cmd.load_mode;
  cancomm_pack->shoot_mode=robot->shoot->shoot_ctrl_cmd.shoot_mode;
  cancomm_pack->gimbal_mode=gimbal_ctrl_cmd->gimbal_mode;
  //cancomm_pack->rest_heat=robot->shoot->shoot_ctrl_cmd.rest_heat;
  //cancomm_pack->shoot_rate=(uint8_t)robot->shoot->shoot_ctrl_cmd.shoot_rate;
  CANCommSend(can_comm_instance, (uint8_t*)cancomm_pack);

#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  ChassisTask();
#endif

  // 正确的赋值方式 - 直接赋值指针值
  // robot->shoot->friction_motor[1];
}
