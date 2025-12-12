// //#include "bsp_init.h"
// #include "robot.h"
// #include "dmmotor.h"
// #include "remote_control.h"
// #include "os_task.h"
//
// // 编译warning,提醒开发者修改机器人参数
// #ifndef ROBOT_DEF_PARAM_WARNING
// #define ROBOT_DEF_PARAM_WARNING
// #endif // !ROBOT_DEF_PARAM_WARNING
//
// // 达妙4310电机实例
// static DMMotorInstance *J4310_motor;
// RC_ctrl_t *rc_data;
//
// // 达妙4310电机初始化配置
// Motor_Init_Config_s J4310_config = {
//     .can_init_config = {
//         .can_handle = &hcan1,        // 根据实际使用的CAN修改
//         .tx_id = 2,              // 发送ID，根据实际电机设置
//     },
//     .controller_param_init_config = {
//         .current_PID = {
//             .Kp = 0.0f,
//             .Ki = 0.0f,
//             .Kd = 0.0f,
//             .MaxOut = 0.0f,
//             .Improve = 0,
//             .DeadBand = 0,
//             .IntegralLimit = 0,
//         },
//         .speed_PID = {
//             .Kp = 1f,
//             .Ki = 0.01f,
//             .Kd = 0.0f,
//             .MaxOut = 5.0f,
//             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//             .DeadBand = 0.1f,
//             .IntegralLimit = 1000,
//         },
//         .angle_PID = {
//             .Kp = 0.0f,
//             .Ki = 0.0f,
//             .Kd = 0.0f,
//             .MaxOut = 10.0f,
//             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//             .DeadBand = 0.05f,
//             .IntegralLimit = 500,
//         },
//         .other_angle_feedback_ptr = NULL,
//         .other_speed_feedback_ptr = NULL,
//     },
//     .controller_setting_init_config = {
//         .angle_feedback_source = MOTOR_FEED,
//         .speed_feedback_source = MOTOR_FEED,
//         .outer_loop_type = SPEED_LOOP,                    // 默认外环为位置环
//         .close_loop_type = SPEED_LOOP,          // 启用位置环和速度环
//         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,     // 电机方向正常
//         .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL, // 反馈方向正常
//         .feedforward_flag = 0,                           // 不使用前馈
//     },
//     .motor_type = J4310,                         // 达妙电机类型
// };
//
// void RobotInit()
// {
//     // 关闭中断,防止在初始化过程中发生中断
//     // 请不要在初始化过程中使用中断和延时函数！
//     // 若必须,则只允许使用DWT_Delay()
//     __disable_irq();
//
//     // BSPInit();
//
//     // 初始化达妙4310电机
//     J4310_motor = DMMotorInit(&J4310_config);
//
//     // 初始化遥控器
//     rc_data = RemoteControlInit(&huart3);  // 根据实际使用的UART修改
//
//     // 设置电机外环控制模式为位置环
//     DMMotorOuterLoop(J4310_motor, SPEED_LOOP);
//
//     // 使能电机
//     //DMMotorEnable(J4310_motor);
//
//     // 初始化电机任务（会创建独立的控制线程）
//     DMMotorTaskInit();
//
//     // 初始化其他任务
//     OSTaskInit(); // 创建基础任务
//
//     // 初始化完成,开启中断
//     __enable_irq();
// }
//
// void RobotTask() {
//     // 达妙4310电机控制代码
//     if(RemoteControlIsOnline() && J4310_motor != NULL) {
//       // if(rc_data->rc.switch_left == RC_SW_UP) {
//       //   DMMotorPIDCal(J4310_motor, 20000);  // 转动
//       // } else {
//       //   DMMotorPIDCal(J4310_motor, 0);      // 停止
//       // }
//       DMMotorPIDCal(J4310_motor, rc_data->rc.rocker_l1/ 660.0f * 200.0f);
//     }
//     /*
//     if(J4310_motor != NULL && rc_data != NULL) {
//         static float target_angle = 0.0f;
//
//         // 根据遥控器左拨杆位置选择控制模式
//         if(rc_data->rc.switch_left == RC_SW_UP) {
//             // 上拨杆：位置控制模式
//             DMMotorOuterLoop(J4310_motor, ANGLE_LOOP);
//
//             // 使用右摇杆控制目标角度
//             target_angle = (float)rc_data->rc.rocker_r1 / 660.0f * 180.0f;
//
//             // 限制角度范围在电机安全范围内
//             if(target_angle > 360.0f) target_angle = 360.0f;
//             if(target_angle < 0.0f) target_angle = 0.0f;
//
//         } else if(rc_data->rc.switch_left == RC_SW_DOWN) {
//             // 下拨杆：速度控制模式
//             DMMotorOuterLoop(J4310_motor, SPEED_LOOP);
//
//             // 使用右摇杆控制目标速度
//             target_angle = (float)rc_data->rc.rocker_r1 / 660.0f * 20.0f;
//
//         } else {
//             // 中拨杆：停止电机，目标设为0
//             target_angle = 0.0f;
//         }
//         // 计算PID控制量
//         DMMotorPIDCal(J4310_motor, target_angle);
//     }
//     */
// }
#include "robot.h"

#include "general_def.h"
#include "master_process.h"
#include "robot_config.h"
#include "user_lib.h"

static RobotInstance *robot;
Int16ToBytes transmit_data;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd;
static Shoot_Ctrl_Cmd_s *shoot_ctrl_cmd;
Vision_Receive_s* vision_recv_data;
static RC_ctrl_t *rc_data;
static RC_ctrl_t *rc_data_last;  // 遥控器数据,初始化时返回

CANCommInstance* can_comm_instance = NULL;

/* Intermediate variables calculated by private functions */
static float trigger_time = 0;  // 触发时间
static float angle;
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
static void CalcOffsetAngle() {
  angle = (uint16_t)robot->gimbal->yaw_motor->measure.angle_single_round;
  float delta =YAW_ALIGN_ANGLE - angle;
  chassis_ctrl_cmd->offset_angle = delta;

  if (chassis_ctrl_cmd->offset_angle > 180.0f) {
    chassis_ctrl_cmd->offset_angle -= 360.0f;
  } else if (chassis_ctrl_cmd->offset_angle <= -180.0f) {
    chassis_ctrl_cmd->offset_angle += 360.0f;
  }
}
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet() {
  // 右[中]，云台
  if (switch_is_mid(rc_data[TEMP].rc.switch_right))
  {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    if (abs(rc_data[TEMP].rc.dial) > 20) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    } else
      chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
  }
  // 右[上]，超电，保持底盘跟随云台
  else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
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
    gimbal_ctrl_cmd->yaw += -0.0016f * (float)rc_data[TEMP].rc.rocker_r_;
    gimbal_ctrl_cmd->pitch -= 0.000006f * (float)rc_data[TEMP].rc.rocker_r1;
  }

  // 云台PITCH轴软件限位 todo:没在云台有点不好
  if (gimbal_ctrl_cmd->pitch > PITCH_ABS_MAX) {
    gimbal_ctrl_cmd->pitch = PITCH_ABS_MAX;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_ABS_MIN) {
    gimbal_ctrl_cmd->pitch = PITCH_ABS_MIN;
  }

  // 底盘参数,系数需要调整
  chassis_ctrl_cmd->vx = 60.0f * (float)rc_data[TEMP].rc.rocker_l_;  // l_水平方向
  chassis_ctrl_cmd->vy = 60.0f * (float)rc_data[TEMP].rc.rocker_l1;  // l1竖直方向
  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_ROTATE) {
    chassis_ctrl_cmd->wz =
        5.0f * (float)rc_data[TEMP].rc.dial;  // 小陀螺模式下的旋转分量，如果是跟随，则在底盘任务中计算旋转分量
  }
  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW) {
    chassis_ctrl_cmd->wz =(15.0f) *(float)rc_data[TEMP].rc.rocker_r_;  // 主动跟随量，todo：但是感觉一个变量拆成两段写好像有点抽象，这里有一段，chassis还有另一段
  }

  // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
  shoot_ctrl_cmd->shoot_rate = 8;

  *rc_data_last = *rc_data;
}

static void MouseKeySet() {
  chassis_ctrl_cmd->vy += (float)((rc_data[TEMP].key[KEY_PRESS].w) - rc_data[TEMP].key[KEY_PRESS].s) *
                         (float) chassis_ctrl_cmd->chassis_speed_buff;
  chassis_ctrl_cmd->vx += (float)(rc_data[TEMP].key[KEY_PRESS].d - rc_data[TEMP].key[KEY_PRESS].a) *
                         (float) -chassis_ctrl_cmd->chassis_speed_buff;
if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON)
  {
  gimbal_ctrl_cmd->yaw -= (float)rc_data[TEMP].mouse.x * 0.007f;  // 横向灵敏度调节
  gimbal_ctrl_cmd->pitch += (float)rc_data[TEMP].mouse.y * 0.003f; // 纵向灵敏度调节 (负号反转Y轴)
  }
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
  switch (rc_data[TEMP].mouse.press_r % 2) {  //右键进入自瞄预备模式
  case 1:
      if (has_non_zero_data(vision_recv_data)==1){
        gimbal_ctrl_cmd->gimbal_mode=GIMBAL_VISION;    // 右键自瞄开启
        gimbal_ctrl_cmd->yaw-=0.05*vision_recv_data->gimbal_receive.yaw;
        gimbal_ctrl_cmd->pitch+=0;
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
      shoot_ctrl_cmd->load_mode=LOAD_STOP;
      trigger_time = DWT_GetTimeline_s();
      break;
  default:
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2)  // E键设置发射模式
    {
      case 0:                                              //单发+长按连发
        if (shoot_ctrl_cmd->friction_mode==FRICTION_ON)   //需预先开启摩擦轮，F键
        {
            shoot_ctrl_cmd->load_mode=LOAD_1_BULLET;
          if (DWT_GetTimeline_s() - trigger_time > 1.0f)  //长按检测，1秒
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
  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Q]%2) //新增Q自旋开启
  {
    case 0:
      chassis_ctrl_cmd-> chassis_mode = CHASSIS_FOLLOW ;
      chassis_ctrl_cmd->wz+=(float)rc_data[TEMP].mouse.x * 30.0f; //主动跟随量
      break;
    default:
      chassis_ctrl_cmd-> chassis_mode = CHASSIS_ROTATE ;
      break;
  }

  switch (rc_data[TEMP].key[KEY_PRESS].shift)  // 待添加 按shift允许超功率 消耗缓冲能量
  {
    case 1:

      break;

    default:

      break;
  }
  if (gimbal_ctrl_cmd->pitch > PITCH_ABS_MAX) {
    gimbal_ctrl_cmd->pitch = PITCH_ABS_MAX;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_ABS_MIN) {
    gimbal_ctrl_cmd->pitch = PITCH_ABS_MIN;
  }
  shoot_ctrl_cmd->shoot_rate = 8;// 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
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
    if ((switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left))||!RemoteControlIsOnline)  // 全部失能
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
    if (switch_is_down(rc_data[TEMP].rc.switch_right)||!RemoteControlIsOnline)  // 底盘失能
    {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    }
    else
      {
    gimbal_ctrl_cmd->gimbal_mode=GIMBAL_ON;
      }
    if (switch_is_down(rc_data[TEMP].rc.switch_left)||!RemoteControlIsOnline)  // 发射失能
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

#ifdef STM32F407xx
  robot->rc_data = RemoteControlInit(&huart3);  // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#elifdef STM32H723XX
  robot->rc_data = RemoteControlInit(&huart5);  // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#endif

  rc_data_last = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
  *rc_data_last = *robot->rc_data;  // 记录上一次遥控器的状态

  // robot->referee_data = RefereeInit(&huart6);  // 裁判系统初始化

  // robot->super_cap = SuperCapInit(&super_cap_config);

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
  robot->gimbal = GimbalInit(&gimbal_init_config);
  robot->shoot = ShootInit(&shoot_init_config);
#endif
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  robot->chassis = ChassisInit(&chassis_init_config);
#endif

  // 初始化控制命令指针
  chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  chassis_ctrl_cmd->max_power = 80;  // 随便给一个初始功率，后面应该要从裁判系统获取
  gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  rc_data = robot->rc_data;
  vision_recv_data=VisionInit(&gimbal_init_config.imu_init_config);
  can_comm_instance = CANCommInit(&comm_config);
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
  // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
  CalcOffsetAngle();
  RemoteControlSet();
  MouseKeySet();
  EmergencyHandler();  // 处理模块离线和遥控器急停等紧急情况
}

void RobotTask() {
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
  VisionSend();
  RobotCMDTask();
  GimbalTask();
  ShootTask();
  //Steering_CANCommSend(can_comm_instance, rc_data);
  transmit_data.value = rc_data->rc.rocker_l_;
  board_can_comm_data.tx_buff[0] = transmit_data.bytes[0];
  board_can_comm_data.tx_buff[1] = transmit_data.bytes[1];

  transmit_data.value = rc_data->rc.rocker_l1;
  board_can_comm_data.tx_buff[2] = transmit_data.bytes[0];
  board_can_comm_data.tx_buff[3] = transmit_data.bytes[1];

  transmit_data.value = rc_data->rc.rocker_r_;
  board_can_comm_data.tx_buff[4] = transmit_data.bytes[0];
  board_can_comm_data.tx_buff[5] = transmit_data.bytes[1];

  transmit_data.value = rc_data->rc.dial;
  board_can_comm_data.tx_buff[6] = transmit_data.bytes[0];
  board_can_comm_data.tx_buff[7] = transmit_data.bytes[1];

  transmit_data.value = (int16_t)robot->gimbal->yaw_motor->measure.angle_single_round;
  board_can_comm_data.tx_buff[8] = transmit_data.bytes[0];
  board_can_comm_data.tx_buff[9] = transmit_data.bytes[1];

  board_can_comm_data.tx_buff[10] = rc_data->rc.switch_right;

  CANCommSend(can_comm_instance, board_can_comm_data.tx_buff);

#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  ChassisTask();
#endif

  // 正确的赋值方式 - 直接赋值指针值
  // robot->shoot->friction_motor[1];
}
