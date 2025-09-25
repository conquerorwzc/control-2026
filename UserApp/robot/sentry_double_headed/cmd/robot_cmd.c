// app
#include "robot_cmd.h"

#include "robot_def.h"
// module
#include "bmi088.h"
#include "dji_motor.h"
#include "general_def.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "remote_control.h"
#include "rm_referee.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "user_lib.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)              // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)              // pitch水平时电机的角度,0-360
#define LEFT_PITCH_HORIZON_ANGLE (LEFT_PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)    // 左侧pitch水平时电机的角度,0-360
#define RIGHT_PITCH_HORIZON_ANGLE (RIGHT_PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)  // 右侧pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD  // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm;  // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;    // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub;  // 底盘反馈信息订阅者
#endif                                  // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;       // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data;  // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;               // 遥控器数据,初始化时返回
static RC_ctrl_t *rc_data_last;          // 遥控器数据,初始化时返回
static referee_info_t *referee_data;     // 用于获取裁判系统的数据
static Vision_Recv_s *vision_recv_data;  // 视觉接收数据指针,初始化时返回
// static Vision_Send_s vision_send_data;  // 视觉发送数据

// static Publisher_t *gimbal_cmd_pub;             // 云台控制消息发布者
// static Subscriber_t *gimbal_feed_sub;           // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;       // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data;  // 从云台获取的反馈信息

static Publisher_t *gimbal_cmd_pub;                    // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;                  // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send_array;       // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data_array;  // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;            // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;          // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;       // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data;  // 从发射获取的反馈信息

static Robot_Status_e robot_state;  // 机器人整体工作状态

static float triger_time = 0;  // 开火触发时间，判断是单发态还是连发态

BMI088Instance *bmi088_test;  // 云台IMU
BMI088_Data_t bmi088_data;
void RobotCMDInit() {
  rc_data = RemoteControlInit(&huart5);  // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
  rc_data_last = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
  *rc_data_last = *rc_data;             // 记录上一次遥控器的状态
  referee_data = RefereeInit(&huart1);  // 裁判系统初始化
  // vision_recv_data = VisionInit(&huart1);  // 视觉通信串口
  //
  // gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
  // gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));

  gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
  gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));

  shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
  shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD  // 双板兼容
  chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
  chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif  // ONE_BOARD
#ifdef GIMBAL_BOARD
  CANComm_Init_Config_s comm_conf = {
      .can_config =
          {
              .can_handle = &hcan1,
              .tx_id = 0x312,
              .rx_id = 0x311,
          },
      .recv_data_len = sizeof(Chassis_Upload_Data_s),
      .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
  };
  cmd_can_comm = CANCommInit(&comm_conf);
#endif  // GIMBAL_BOARD
  gimbal_cmd_send.pitch = 0;
  chassis_cmd_send.max_power = 80;
  gimbal_cmd_send_array.left_pitch = LEFT_PITCH_HORIZON_ECD;
  gimbal_cmd_send_array.left_yaw = LEFT_YAW_HORIZON_ECD;
  gimbal_cmd_send_array.right_yaw = RIGHT_YAW_HORIZON_ECD;
  gimbal_cmd_send_array.right_pitch = RIGHT_PITCH_HORIZON_ECD;
  robot_state = ROBOT_READY;  // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle() {
  // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
  static float angle;
  angle = gimbal_fetch_data.yaw_motor_single_round_angle;  // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                              // 如果大于180度
  if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
  else if (angle > 180.0f + YAW_ALIGN_ANGLE)
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
  else
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else  // 小于180度
  if (angle > YAW_ALIGN_ANGLE)
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
  else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
  else
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet() {
  // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
  if (switch_is_mid(rc_data[TEMP].rc.switch_right))  // 右[中]，底盘跟随云台
  {
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    if (abs(rc_data[TEMP].rc.dial) > 20) {
      chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    } else
      chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
  } else if (switch_is_up(rc_data[TEMP].rc.switch_right))  // 右[上]，超电，保持底盘跟随云台
  {
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    if (abs(rc_data[TEMP].rc.dial) > 20) {
      chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    } else
      chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    // chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    // gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
  }

  // 云台参数,确定云台控制数据
  if (switch_is_mid(rc_data[TEMP].rc.switch_left))  // 左侧开关状态为[中],云台启动，摩擦轮启动，拨弹盘启动，准备射击
  {
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    shoot_cmd_send.friction_mode = FRICTION_ON;
    shoot_cmd_send.load_mode = LOAD_STOP;

    gimbal_cmd_send_array.gimbal_mode = GIMBAL_FREE_MODE;

    // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
    // ...

  } else if (switch_is_up(rc_data[TEMP].rc.switch_left))  // 开火，发射，根据时间判断单发或者连发
  {
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    shoot_cmd_send.friction_mode = FRICTION_ON;
    shoot_cmd_send.load_mode = LOAD_STOP;
    if (switch_is_mid(rc_data_last[TEMP].rc.switch_left)) {
      triger_time = DWT_GetTimeline_s();
    }
    if (DWT_GetTimeline_s() - triger_time > 2.0f) {
      shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    } else {
      shoot_cmd_send.load_mode = LOAD_1_BULLET;
    }
  }
  // 云台使能,或视觉未识别到目标,纯遥控器拨杆控制
  if ((gimbal_cmd_send.gimbal_mode == GIMBAL_GYRO_MODE) ||
      (vision_recv_data->target_state == NO_TARGET)) {  // 按照摇杆的输出大小进行角度增量,增益系数需调整
    gimbal_cmd_send.yaw -= 0.005f * (float)rc_data[TEMP].rc.rocker_r_;
    gimbal_cmd_send.pitch += 0.002f * (float)rc_data[TEMP].rc.rocker_r1;
    gimbal_cmd_send_array.left_yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_r_;
    gimbal_cmd_send_array.right_yaw -= 0.005f * (float)rc_data[TEMP].rc.rocker_r_;
    gimbal_cmd_send_array.left_pitch += 0.002f * (float)rc_data[TEMP].rc.rocker_r1;
    gimbal_cmd_send_array.right_pitch -= 0.002f * (float)rc_data[TEMP].rc.rocker_r1;
  }
  // 云台PITCH轴软件限位
  if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE) {
    gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE) {
    gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
  }
  // 左云台限位,pitch
  if (gimbal_cmd_send_array.left_pitch > LEFT_PITCH_MAX_ANGLE) {
    gimbal_cmd_send_array.left_pitch = LEFT_PITCH_MAX_ANGLE;
  } else if (gimbal_cmd_send_array.left_pitch < LEFT_PITCH_MIN_ANGLE) {
    gimbal_cmd_send_array.left_pitch = LEFT_PITCH_MIN_ANGLE;
  }
  // 左云台限位，yaw
  if (gimbal_cmd_send_array.left_yaw > LEFT_YAW_MAX_ANGLE) {
    gimbal_cmd_send_array.left_yaw = LEFT_YAW_MAX_ANGLE;
  } else if (gimbal_cmd_send_array.left_yaw < LEFT_YAW_MIN_ANGLE) {
    gimbal_cmd_send_array.left_yaw = LEFT_YAW_MIN_ANGLE;
  }

  // 右云台限位，pitch
  if (gimbal_cmd_send_array.right_pitch > RIGHT_PITCH_MAX_ANGLE) {
    gimbal_cmd_send_array.right_pitch = RIGHT_PITCH_MAX_ANGLE;
  } else if (gimbal_cmd_send_array.right_pitch < RIGHT_PITCH_MIN_ANGLE) {
    gimbal_cmd_send_array.right_pitch = RIGHT_PITCH_MIN_ANGLE;
  }

  // 右云台限位，yaw
  if (gimbal_cmd_send_array.right_yaw > RIGHT_YAW_MAX_ANGLE) {
    gimbal_cmd_send_array.right_yaw = RIGHT_YAW_MAX_ANGLE;
  } else if (gimbal_cmd_send_array.right_yaw < RIGHT_YAW_MIN_ANGLE) {
    gimbal_cmd_send_array.right_yaw = RIGHT_YAW_MIN_ANGLE;
  }
}
/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
// static void MouseKeySet()
// {
//     chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300; // 系数待测
//     chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].s * 300 - rc_data[TEMP].key[KEY_PRESS].d * 300;
//
//     gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
//     gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;
//
//     switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
//     {
//     case 0:
//         shoot_cmd_send.bullet_speed = 15;
//         break;
//     case 1:
//         shoot_cmd_send.bullet_speed = 18;
//         break;
//     default:
//         shoot_cmd_send.bullet_speed = 30;
//         break;
//     }
//     switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4) // E键设置发射模式
//     {
//     case 0:
//         shoot_cmd_send.load_mode = LOAD_STOP;
//         break;
//     case 1:
//         shoot_cmd_send.load_mode = LOAD_1_BULLET;
//         break;
//     case 2:
//         shoot_cmd_send.load_mode = LOAD_3_BULLET;
//         break;
//     default:
//         shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
//         break;
//     }
//     switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
//     {
//     case 0:
//         shoot_cmd_send.lid_mode = LID_OPEN;
//         break;
//     default:
//         shoot_cmd_send.lid_mode = LID_CLOSE;
//         break;
//     }
//     switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
//     {
//     case 0:
//         shoot_cmd_send.friction_mode = FRICTION_OFF;
//         break;
//     default:
//         shoot_cmd_send.friction_mode = FRICTION_ON;
//         break;
//     }
//     switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
//     {
//     case 0:
//         chassis_cmd_send.chassis_speed_buff = 40;
//         break;
//     case 1:
//         chassis_cmd_send.chassis_speed_buff = 60;
//         break;
//     case 2:
//         chassis_cmd_send.chassis_speed_buff = 80;
//         break;
//     default:
//         chassis_cmd_send.chassis_speed_buff = 100;
//         break;
//     }
//     switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
//     {
//     case 1:
//
//         break;
//
//     default:
//
//         break;
//     }
// }

// 裁判系统相关控制量设置
static void RefereeSet() {
  //     chassis_cmd_send.max_power=referee_data->GameRobotState.chassis_power_limit;
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
  if ((switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left)))  // 全部失能
  {
    robot_state = ROBOT_STOP;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    sentry_gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    LOGERROR("[CMD] emergency stop!");
  } else {
    robot_state = ROBOT_READY;
    LOGINFO("[CMD] reinstate, robot ready");
  }
  if (switch_is_down(rc_data[TEMP].rc.switch_right))  // 底盘失能
  {
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
  }
  if (switch_is_down(rc_data[TEMP].rc.switch_left))  // 发射失能
  {
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
  }
  // 遥控器右侧开关为[上],恢复正常运行
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
  // 保证数据是新的
  if (rc_data->updated) {
    rc_data->updated = 0;
    RemoteControlSet();
  }
  if (referee_data->custom_data.data_cmd_id) {
    referee_data->custom_data.data_cmd_id = 0;
    // RefereeUISet();
  }

  // 发送和回传数据
#ifdef ONE_BOARD
  // 从其他应用获取反馈信息
  SubGetMessage(chassis_feed_sub, &chassis_fetch_data);
  SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data_array);
  SubGetMessage(shoot_feed_sub, &shoot_fetch_data);

  // 计算云台和底盘的相对角度
  CalcOffsetAngle();

  // 将控制信息发送给其他应用
  PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
  PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send_array);
  PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
#endif  // ONE_BOARD

#ifdef GIMBAL_BOARD
  // gimbal board can only send chassis cmd to chassis board
  memcpy(cmd_can_comm->send_buff, &chassis_cmd_send, sizeof(Chassis_Ctrl_Cmd_s));
  CANCommSend(cmd_can_comm);
#endif  // GIMBAL_BOARD

  // 更新UI
  RefereeUISet();
}
