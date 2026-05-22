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
Vision_Send_s *vision_send_data;
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
static uint8_t diagonal_mode = 0;
/* Intermediate variables calculated by private functions */
static float trigger_time = 0;  // 触发时间
static float trigger1_time = 0;  // 触发时间
static float x_speed_time=0;  //x方向加速触发时间
static float y_speed_time=0;  //y方向加速触发时间
static float vx_initial;   //x轴输入控制量
static float vy_initial;   //y轴输入控制量
static float angle;
static uint8_t rotate_mode_active = 0;
static uint8_t free_mode_active = 0;
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
  // if (abs(delta) < 2.0f) {
  //   delta =0.0f;
  // }
   chassis_ctrl_cmd->offset_angle = delta;
}
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
#ifdef USE_VT13
/* ======================== VT13 遥控器版本 ======================== */
static void RemoteControlSet() {
  // mode_switch: L=云台+底盘失能, M=底盘follow, R=底盘斜45度follow
  if (rc_data[TEMP].button_status.pause_flag) {
    // 左: 云台+底盘失能,强制关闭所有输出
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    chassis_ctrl_cmd->SuperCapBoost = 0;
  } else
  {
      // ---- FN1: 云台+shoot开关(toggle, 按一次开再按一次关) ----
      if (rc_data[TEMP].button_status.fn_1_flag) {
          gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
          //shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
      } else {
          gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
          //shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
          //shoot_ctrl_cmd->load_mode = LOAD_STOP;
      }

      // ---- FN2: 摩擦轮开关(toggle) ----
      shoot_ctrl_cmd->friction_mode = rc_data[TEMP].button_status.fn_2_flag ? FRICTION_ON : FRICTION_OFF;
          shoot_ctrl_cmd->shoot_mode = rc_data[TEMP].button_status.fn_2_flag ? SHOOT_ON : SHOOT_OFF;
      if (switch_left(rc_data[TEMP].rc.mode_switch))
      {
          chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
          chassis_ctrl_cmd->SuperCapBoost = 0;
      }
      // ---- ModeSwitch: 中/右位置 ----
      if (switch_middle(rc_data[TEMP].rc.mode_switch)) {
          // 中: 底盘follow, dial>20→rotate
          chassis_ctrl_cmd->SuperCapBoost = 0;
          gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
          shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
          if (abs(rc_data[TEMP].rc.dial) > 20) {
              rotate_mode_active=1;
              chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
          } else {
              chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
              diagonal_mode=0;
          }
      } else if (switch_right(rc_data[TEMP].rc.mode_switch)) {
          // 右: 底盘斜45度follow, dial>20→rotate, 超电开
          chassis_ctrl_cmd->SuperCapBoost = 1;
          gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
          shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
          if (abs(rc_data[TEMP].rc.dial) > 20) {
              rotate_mode_active=1;
              chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
          } else {
              chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
              diagonal_mode=1;
          }
      }

      // ---- Trigger: 发弹(长按连发,短按单发) ----
      if (shoot_ctrl_cmd->friction_mode==FRICTION_ON){
          if (rc_data[TEMP].rc.trigger) {
              if (!rc_data_last[TEMP].rc.trigger) {
                  trigger1_time = DWT_GetTimeline_s();
              }
              if (DWT_GetTimeline_s() - trigger1_time > 1.0f) {
                  shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
              } else {
                  shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
              }
          } else {
              shoot_ctrl_cmd->load_mode = LOAD_STOP;
          }
      }
      else shoot_ctrl_cmd->load_mode=LOAD_STOP;
  }
  // 云台使能,遥控器摇杆控制
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    gimbal_ctrl_cmd->yaw -= 0.0013f * (float)rc_data[TEMP].rc.rocker_r_;
    gimbal_ctrl_cmd->pitch -= 0.0003f * (float)rc_data[TEMP].rc.rocker_r1;
  }

  // 云台PITCH轴软件限位
  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }

  // 底盘参数
  vx_initial = -60.0f * (float)rc_data[TEMP].rc.rocker_l1;
  vy_initial = 60.0f * (float)rc_data[TEMP].rc.rocker_l_;
  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_ROTATE) {
    chassis_ctrl_cmd->wz = 15000;
  }
  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW || chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW_DIAGONAL) {
    chassis_ctrl_cmd->wz = 25.0f * (float)rc_data[TEMP].rc.rocker_r_;
  }


}

static void MouseKeySet()
{
    if (!switch_left(rc_data[TEMP].rc.mode_switch)) {
        vx_initial -= (float)((rc_data[TEMP].key[KEY_PRESS].w) - rc_data[TEMP].key[KEY_PRESS].s) *
                      (float)chassis_ctrl_cmd->chassis_speed_buff;
        vy_initial += (float)(rc_data[TEMP].key[KEY_PRESS].a - rc_data[TEMP].key[KEY_PRESS].d) *
                      (float)-chassis_ctrl_cmd->chassis_speed_buff;

        // wz基于当前底盘模式
        if (chassis_ctrl_cmd->chassis_mode == CHASSIS_ROTATE) {
            chassis_ctrl_cmd->wz = 25000;
        } else {
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
        if (chassis_ctrl_cmd->vx>vx_initial) chassis_ctrl_cmd->vx = vx_initial;
        if (chassis_ctrl_cmd->vy>vy_initial) chassis_ctrl_cmd->vy = vy_initial;





        // // C键设置底盘速度
        // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) {
        // case 0:
        //     chassis_ctrl_cmd->chassis_speed_buff = 10000;
        //     break;
        // case 1:
        //     chassis_ctrl_cmd->chassis_speed_buff = 20000;
        //     break;
        // case 2:
            chassis_ctrl_cmd->chassis_speed_buff = 40000;
        //     break;
        // default:
        //     chassis_ctrl_cmd->chassis_speed_buff = 80000;
        //     break;
        // }

        // ---- B 键：切换 FOLLOW / FOLLOW_DIAGONAL（ROTATE模式下B键无效） ----
        // static uint8_t prev_b_count = 0;
        if (chassis_ctrl_cmd->chassis_mode!=CHASSIS_ROTATE)
        {
            switch (rc_data[TEMP].key_count[KEY_PRESS][Key_B] %3)
            {
                case 0:
                chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
                break;
                case 1:
                chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW_DIAGONAL;
                break;
                case 2:
                chassis_ctrl_cmd->chassis_mode=CHASSIS_FREE;
                chassis_ctrl_cmd->wz=0;
                break ;
            }
        }
        // if (!(rc_data[TEMP].key_count[KEY_PRESS][Key_B] %2 == 1)) {
        //     if (chassis_ctrl_cmd->chassis_mode==CHASSIS_FOLLOW_DIAGONAL)
        //     chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
        //     } else {
        //         if (chassis_ctrl_cmd->chassis_mode==CHASSIS_FOLLOW )
        //         chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW_DIAGONAL;
        //         //chassis_ctrl_cmd->SuperCapBoost = 0;
        //     }
        }
    if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
        gimbal_ctrl_cmd->yaw -= (float)rc_data[TEMP].mouse_key.mouse.x * 0.006f;
        gimbal_ctrl_cmd->pitch -= (float)rc_data[TEMP].mouse_key.mouse.y * 0.003f;
    }
    switch ((rc_data[TEMP].key_count[KEY_PRESS][Key_V])%3)
    {
        case 0:
         robot->gimbal->gimbal_ctrl_cmd.vision_mode = AUTO_AIM;
        break;
        case 1:
        robot->gimbal->gimbal_ctrl_cmd.vision_mode = SMALL_BUFF;
        break;
        case 2:
        robot->gimbal->gimbal_ctrl_cmd.vision_mode = BIG_BUFF;
        break;
    }
    // 右键进入自瞄模式
    switch (rc_data[TEMP].mouse_key.mouse.press_r ) {
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
if (((rc_data[TEMP].mouse_key.mouse.press_l )&&(gimbal_ctrl_cmd->gimbal_mode!=GIMBAL_VISION))||
    ((rc_data[TEMP].mouse_key.mouse.press_l )&&(vision_recv_data->shoot_receive.fire_flag==1)&&(gimbal_ctrl_cmd->gimbal_mode == GIMBAL_VISION)))
{
    // 左键发射
    switch (rc_data[TEMP].mouse_key.mouse.press_l %2){
    case 0:
        trigger_time=DWT_GetTimeline_s();
        break;
    default:
        switch (rc_data[TEMP].key_count[KEY_PRESS][Key_G] % 2) {  // G键切换单发/连发
    case 0:
            if (shoot_ctrl_cmd->friction_mode == FRICTION_ON) {
                shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
                // if (DWT_GetTimeline_s() - trigger_time > 0.5f) {
                //     shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
                // }
            }
            break;
    default:
            if (shoot_ctrl_cmd->friction_mode == FRICTION_ON)
                shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
            break;
        }
        break;
    }
}
        // ---- Q 键：切换 ROTATE 模式（唯一能退出ROTATE的方式） ----
        static uint8_t prev_q_count = 0;

        static Chassis_Mode_e saved_chassis_mode = CHASSIS_FOLLOW;

        if (rc_data[TEMP].key_count[KEY_PRESS][Key_Q] %2 == 1) {
                //chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
                rotate_mode_active = 1;
            } else {
                //chassis_ctrl_cmd->chassis_mode = saved_chassis_mode;
                rotate_mode_active = 0;
            }
    // if (rc_data[TEMP].key_count[KEY_PRESS][Key_C] %2 == 1) {
    //     //chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    //     free_mode_active = 1;
    // } else {
    //     //chassis_ctrl_cmd->chassis_mode = saved_chassis_mode;
    //     free_mode_active = 0;
    // }
        // Shift: 按下开启超级电容，松开关闭（不改变底盘模式）
        chassis_ctrl_cmd->SuperCapBoost |= rc_data[TEMP].key[KEY_PRESS].shift ? 1 : 0;

        // 强制保持ROTATE模式（只有Q能退出）
        if (rotate_mode_active) {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
            chassis_ctrl_cmd->wz=25000;
        }
    if (free_mode_active)
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_FREE;
        chassis_ctrl_cmd->wz=0;
    }
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FREE)
        chassis_ctrl_cmd->wz=0;
    if (shoot_ctrl_cmd->bullet_speed_mode==MANUAL_BULLET_SPEED) {
        if (rc_data[TEMP].key[KEY_PRESS].z==1&&rc_data_last[TEMP].key[KEY_PRESS].z==0) {
            shoot_ctrl_cmd->friction_speed+=shoot_init_config.shoot_param.bullet_speed_adjustment;
        }
        else if (rc_data[TEMP].key[KEY_PRESS].x==1&&rc_data_last[TEMP].key[KEY_PRESS].x==0) {
            shoot_ctrl_cmd->friction_speed-=shoot_init_config.shoot_param.bullet_speed_adjustment;
        }
    }

        if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
            gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
        } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
            gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
        }


        // ---- R 键：将状态通过SuperCapBoost第二位传递到底盘板 (CAN) ----
        if (rc_data[TEMP].key[KEY_PRESS].r) {
            chassis_ctrl_cmd->SuperCapBoost |= 2;
        } else {
            chassis_ctrl_cmd->SuperCapBoost &= (uint8_t)~2;
        }

    *rc_data_last = *rc_data;
}
    static void EmergencyHandler() {
        // VT13: pause按键触发急停
        if (rc_data[TEMP].button_status.pause_flag||!VT13RemoteIsOnline()) {
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
    }

#else
/* ======================== 旧DR16遥控器版本 ======================== */
static void RemoteControlSet() {
  // 右[中]，云台
  if (switch_is_mid(rc_data[TEMP].rc.switch_right))
  {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    chassis_ctrl_cmd->SuperCapBoost =1;
    if (abs(rc_data[TEMP].rc.dial) > 20) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    } else
        chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW_DIAGONAL;
      diagonal_mode=0;
  }
  // 右[上]，超电，保持底盘跟随云台
  else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    chassis_ctrl_cmd->SuperCapBoost =1;
    if (abs(rc_data[TEMP].rc.dial) > 20) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    } else
        chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW_DIAGONAL;
      diagonal_mode=1;
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
    gimbal_ctrl_cmd->pitch += 0.0003f * (float)rc_data[TEMP].rc.rocker_r1;
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
  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW||chassis_ctrl_cmd->chassis_mode ==CHASSIS_FOLLOW_DIAGONAL) {
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
    // Shift: 按下开启超级电容，松开关闭（不改变底盘模式）
    chassis_ctrl_cmd->SuperCapBoost |= rc_data[TEMP].key[KEY_PRESS].shift ? 1 : 0;

    // wz基于当前底盘模式
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_ROTATE) {
        chassis_ctrl_cmd->wz = 15000;
    } else {
        chassis_ctrl_cmd->wz = (50.0f) * (float)rc_data[TEMP].mouse.x + (25.0f) * (float)rc_data[TEMP].rc.rocker_r_;
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
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_G] % 2)  // G键切换单发/连发
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
  // ---- B 键：切换 FOLLOW / FOLLOW_DIAGONAL（ROTATE模式下B键无效） ----
  static uint8_t prev_b_count = 0;
  if (rc_data[TEMP].key_count[KEY_PRESS][Key_B] != prev_b_count) {
    prev_b_count = rc_data[TEMP].key_count[KEY_PRESS][Key_B];
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW_DIAGONAL;
      chassis_ctrl_cmd->SuperCapBoost = 1;
    } else if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW_DIAGONAL) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
      chassis_ctrl_cmd->SuperCapBoost = 0;
    }
  }

  // ---- Q 键：切换 ROTATE 模式（唯一能退出ROTATE的方式） ----
  static uint8_t prev_q_count = 0;
  static uint8_t rotate_mode_active = 0;
  static Chassis_Mode_e saved_chassis_mode = CHASSIS_FOLLOW_DIAGONAL;

  if (rc_data[TEMP].key_count[KEY_PRESS][Key_Q] != prev_q_count) {
    prev_q_count = rc_data[TEMP].key_count[KEY_PRESS][Key_Q];
    if (!rotate_mode_active) {
      saved_chassis_mode = chassis_ctrl_cmd->chassis_mode;
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
      rotate_mode_active = 1;
    } else {
      chassis_ctrl_cmd->chassis_mode = saved_chassis_mode;
      rotate_mode_active = 0;
    }
  }

  // 强制保持ROTATE模式（只有Q能退出）
  if (rotate_mode_active) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
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
    robot->rc_data = VT13RemoteInit(&huart6);
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
    shoot_ctrl_cmd->bullet_speed_mode=MANUAL_BULLET_SPEED;
  can_comm_instance = CANCommInit(&comm_config);
    vision_send_data=GetVisionSend();
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
  // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
  CalcOffsetAngle();
  RemoteControlSet();
  ///if (rc_data->rc.rocker_l1==0&&rc_data->rc.rocker_r1==0&&rc_data->rc.rocker_l_==0&&rc_data->rc.rocker_r_==0)
    MouseKeySet();
    shoot_ctrl_cmd->initial_speed = upload->bullet_speed;
    shoot_ctrl_cmd->remain_heat=upload->Remain_Heat;
    shoot_ctrl_cmd->initial_speed=upload->bullet_speed;
    vision_send_data->gimbal_send.mode=gimbal_ctrl_cmd->vision_mode;
  //  shoot_ctrl_cmd->shooter_heat_limit=upload->heat_limt;
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
    cancomm_pack->vision_mode=gimbal_ctrl_cmd->vision_mode;
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
