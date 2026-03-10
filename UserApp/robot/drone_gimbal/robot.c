//
// Created by zeg on 2025/12/3.
//

#include "robot.h"

#include "bsp_gpio.h"
#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"
#include "master_process.h"
static RobotInstance *robot;
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd;
static Shoot_Ctrl_Cmd_s *shoot_ctrl_cmd;
static Vision_Receive_s* vision_recv_data;
static RC_ctrl_t *rc_data;
static RC_ctrl_t *rc_data_last;  // 遥控器数据,初始化时返回

/* Intermediate variables calculated by private functions */
static float trigger_time = 0;  // 触发时间
// static  DJIMotorInstance* debug_motor;

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
/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet() {

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
    gimbal_ctrl_cmd->yaw += -0.0016f * (float)rc_data[TEMP].rc.rocker_r_;
    gimbal_ctrl_cmd->pitch -= 0.0003f * (float)rc_data[TEMP].rc.rocker_r1;
  }

  // 云台PITCH轴软件限位 todo:没在云台有点不好
  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }
  // 云台PITCH轴软件限位 todo:没在云台有点不好
  if (gimbal_ctrl_cmd->yaw > YAW_MAX_ANGLE) {
    gimbal_ctrl_cmd->yaw = YAW_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->yaw < YAW_MIN_ANGLE) {
    gimbal_ctrl_cmd->yaw = YAW_MIN_ANGLE;
  }

  // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
  shoot_ctrl_cmd->shoot_rate = 8;

  *rc_data_last = *rc_data;
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet() {
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON)
  {
    gimbal_ctrl_cmd->yaw -= (float)rc_data[TEMP].mouse.x * 0.007f;  // 横向灵敏度调节
    gimbal_ctrl_cmd->pitch += (float)rc_data[TEMP].mouse.y * 0.003f; // 纵向灵敏度调节 (负号反转Y轴)
  }
  // // 检测ctrl键按下事件（从释放到按下），设置腿部为空中模式
  // if (!rc_data_last[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].ctrl) {
  //   chassis_ctrl_cmd->leg_mode = LEG_IN_AIR;
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

  // switch (rc_data[TEMP].mouse.press_l % 2)        // 左键发射
  // {
  //   case 0:
  //     if (!switch_is_up(rc_data[TEMP].rc.switch_left))
  //     {
  //       shoot_ctrl_cmd->load_mode=LOAD_STOP;
  //       trigger_time = DWT_GetTimeline_s();
  //     }
  //     break;
  //   default:
  //     switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2)  // E键设置发射模式
  //     {
  //     case 0:                                              //单发+长按连发
  //         if (shoot_ctrl_cmd->friction_mode==FRICTION_ON)   //需预先开启摩擦轮，F键
  //         {
  //           shoot_ctrl_cmd->load_mode=LOAD_1_BULLET;
  //           if (DWT_GetTimeline_s() - trigger_time > 1.0f)  //长按检测，1秒
  //           {
  //             shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
  //           }
  //           break;
  //           default:                                         //连发
  //           if (shoot_ctrl_cmd->friction_mode==FRICTION_ON)
  //             shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
  //           break;
  //         }
  //     }
  //     break;
  // }


  *rc_data_last = *rc_data;
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
  if ( switch_is_down(rc_data[TEMP].rc.switch_left))  // 全部失能
  {
    robot->robot_mode = ROBOT_POWER_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    LOGERROR("[CMD] emergency stop!");
  } else {
    LOGINFO("[CMD] reinstate, robot ready");
  }
}

void RobotInit() {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

#ifdef STM32F4
  robot->rc_data = RemoteControlInit(&huart3);  // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#elifdef STM32H7
  robot->rc_data = RemoteControlInit(&huart5);  // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#endif

  rc_data_last = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
  *rc_data_last = *robot->rc_data;  // 记录上一次遥控器的状态

  // robot->super_cap = SuperCapInit(&super_cap_config);

   robot->gimbal = GimbalInit(&gimbal_init_config);
   robot->shoot = ShootInit(&shoot_init_config);

  gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  rc_data = robot->rc_data;
  vision_recv_data=VisionInit(&gimbal_init_config.imu_init_config);
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
  // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
  shoot_ctrl_cmd->initial_speed=robot->referee_data->ShootData.initial_speed;
  RemoteControlSet();
  // MouseKeySet();
  EmergencyHandler();  // 处理模块离线和遥控器急停等紧急情况
}

void RobotTask() {
  VisionSend();
  RobotCMDTask();
  GimbalTask();
  ShootTask();


}