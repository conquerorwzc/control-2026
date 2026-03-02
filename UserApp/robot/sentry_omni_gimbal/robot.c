#include "robot.h"

#include "general_def.h"
#include "master_process.h"
#include "new_RC_VT13.h"
#include "robot_config.h"
#include "user_lib.h"
#include "can_comm.h"
static RobotInstance *robot;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd;
static Shoot_Ctrl_Cmd_s *shoot_ctrl_cmd;
static Vision_Receive_s* vision_recv_data;
static navigator_recv_t* navigator_data;
static RC_ctrl_t *rc_data;
static RC_ctrl_t *rc_data_last;  // 遥控器数据,初始化时返回
static VT13_RC_t *vt13_rc_data;
#ifdef USE_DUAL_RC
static Send_Data_RC *send_data;
#elifdef USE_DUAL_RC_NEW
static Send_Data_RC_NEW *send_data_new;
#endif
static CANCommInstance* can_comm_instance = NULL;
static Referee_Data *RefereeData;
/* Intermediate variables calculated by private functions */
static float trigger_time = 0;  // 触发时间
static float time=0;//判断按钮按下需要重复读取时间，这里简化成一次读取

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


/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
#ifdef USE_DUAL_RC
static void RemoteControlSet() {
  // 右[中]，云台
   if (switch_is_mid(rc_data[TEMP].rc.switch_right))
   {
     gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
   }
  // 右[上]，超电，保持底盘跟随云台
  else if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
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
      trigger_time = time;
    }
    if (time - trigger_time > 1.0f) {
      shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
    } else {
      shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
    }
  }
    if (switch_is_up(rc_data[TEMP].rc.switch_right))//除了右拨杆在上机器人使用导航数据，其余都正常人为控制
    {
      robot->control_mode=AUTO_MODE;
    }
    else {
      robot->control_mode=MANUAL_MODE;
    }
    // 云台使能,或视觉未识别到目标,纯遥控器拨杆控制
    if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {  // 按照摇杆的输出大小进行角度增量,增益系数需调整
      gimbal_ctrl_cmd->yaw += -0.0003f * (float)rc_data[TEMP].rc.rocker_r_;
      gimbal_ctrl_cmd->pitch -= 0.0001f * (float)rc_data[TEMP].rc.rocker_r1;
    }
    // 云台PITCH轴软件限位 todo:没在云台有点不好
    if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
      gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
    } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
      gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
    }

    // 底盘控制部分,系数需要调整
    if (robot->control_mode == MANUAL_MODE)//手动控制，遥控器控制量
    {
    }
    else if (robot->control_mode == AUTO_MODE) // 自动控制，直接接收上位机控制量
    {
      float NotFountTime=0;                                 //没有识别到装甲板的时间
      gimbal_ctrl_cmd->gimbal_mode=GIMBAL_VISION;           //自瞄开启
      if (has_non_zero_data(vision_recv_data)==1){
        gimbal_ctrl_cmd->yaw=vision_recv_data->gimbal_receive.yaw;
        gimbal_ctrl_cmd->pitch=vision_recv_data->gimbal_receive.pitch;
        // shoot_ctrl_cmd->load_mode=vision_recv_data->shoot_receive.fire_flag;
        NotFountTime=time;                   //识别到装甲板
      }
      else if (time-NotFountTime>0.5f){      //丢失目标超0.5秒，进入寻敌模式
        gimbal_ctrl_cmd->yaw+=0.15f;
        gimbal_ctrl_cmd->pitch+=0.045f*cosf(PI*1.5*time);
      }
    }
    *rc_data_last = *rc_data;
  }

static void MouseKeySet() {
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

        gimbal_ctrl_cmd->yaw=vision_recv_data->gimbal_receive.yaw;
        gimbal_ctrl_cmd->pitch=vision_recv_data->gimbal_receive.pitch;
        // shoot_ctrl_cmd->load_mode=vision_recv_data->shoot_receive.fire_flag;
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
        trigger_time = time;
      }
      break;
  default:
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2)  // E键设置发射模式
    {
      case 0:                                              //单发+长按连发
        if (shoot_ctrl_cmd->friction_mode==FRICTION_ON&&(vision_recv_data->shoot_receive.fire_flag||rc_data[TEMP].mouse.press_r % 2==0))   //需预先开启摩擦轮
        {
            shoot_ctrl_cmd->load_mode=LOAD_1_BULLET;
        if (time - trigger_time > 1.0f)  //长按检测，1秒
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

  // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4)  // C键设置底盘速度
  // {
  //   case 0:
  //     chassis_ctrl_cmd->chassis_speed_buff = 10000;
  //     break;
  //   case 1:
  //     chassis_ctrl_cmd->chassis_speed_buff = 20000;
  //     break;
  //   case 2:
  //     chassis_ctrl_cmd->chassis_speed_buff = 30000;
  //     break;
  //   default:
  //     chassis_ctrl_cmd->chassis_speed_buff = 40000;
  //     break;
  // }
  // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Q]%2) //Q自旋开启
  // {
  //   case 0:
  //     chassis_ctrl_cmd-> chassis_mode = CHASSIS_FOLLOW ;
  //     chassis_ctrl_cmd->wz+=(float)rc_data[TEMP].mouse.x * 30.0f; //主动跟随量
  //     break;
  //   default:
  //     chassis_ctrl_cmd-> chassis_mode = CHASSIS_ROTATE ;
  //     break;
  // }

  switch (rc_data[TEMP].key[KEY_PRESS].shift)  // 待添加 按shift允许超功率 消耗缓冲能量
  {
    case 1:

      break;

    default:

      break;
  }
  shoot_ctrl_cmd->shoot_rate = 8;// 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }
}
# elifdef USE_DUAL_RC_NEW
static void RemoteControlSet() {
  // 使用VT13遥控器的新控制逻辑
  // 控制云台&打弹
  if (switch_middle(vt13_rc_data->rc.mode_switch)) {  // 中档
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
  }
  if (switch_right(vt13_rc_data->rc.mode_switch)) {
    // 上档
    shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    shoot_ctrl_cmd->friction_mode = FRICTION_ON;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    if (vt13_rc_data->button_status.trigger_last==0) {
      trigger_time = time;
    }
    if (vt13_rc_data->rc.trigger == 1 && time - trigger_time > 1.0f) {
      shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
    } else {
      shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
    }
  }


  // 云台控制
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    gimbal_ctrl_cmd->yaw += -0.0003f * (float)vt13_rc_data->rc.rocker_r_;
    gimbal_ctrl_cmd->pitch -= 0.0001f * (float)vt13_rc_data->rc.rocker_r1;
  }

  // 云台PITCH轴软件限位
  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }

  // 控制模式切换
  if (vt13_rc_data->button_status.fn_2_flag == 1) {  // 按功能右键切换模式
    robot->control_mode = AUTO_MODE;
  } else {
    robot->control_mode = MANUAL_MODE;
  }

  if (robot->control_mode == AUTO_MODE) {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_VISION;
  }

  if (vt13_rc_data->button_status.fn_1_flag==1) {
    float NotFountTime=0;                                 //没有识别到装甲板的时间
    if (has_non_zero_data(vision_recv_data)==1){
      gimbal_ctrl_cmd->yaw=vision_recv_data->gimbal_receive.yaw;
      gimbal_ctrl_cmd->pitch=vision_recv_data->gimbal_receive.pitch;
      // shoot_ctrl_cmd->load_mode=vision_recv_data->shoot_receive.fire_flag;
      NotFountTime=time;                   //识别到装甲板
    }
    else if (time-NotFountTime>0.5f){      //丢失目标超0.5秒，进入寻敌模式
      gimbal_ctrl_cmd->yaw+=0.05f;
      gimbal_ctrl_cmd->pitch=20.0f*cosf(PI*3*time);//cos里数字越大，旋转速度越快
    }
  }
}

static void MouseKeySet() {
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {
    gimbal_ctrl_cmd->yaw -= (float)vt13_rc_data->mouse_key.mouse.x * 0.007f;
    gimbal_ctrl_cmd->pitch += (float)vt13_rc_data->mouse_key.mouse.y * 0.003f;
  }

  // 弹速设置 (Z键)
  switch (__builtin_popcount(vt13_rc_data->mouse_key.keyboard.z) % 3) {
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

  // 右键自瞄
  switch (vt13_rc_data->mouse_key.mouse.press_r % 2) {
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
  switch (vt13_rc_data->mouse_key.mouse.press_l % 2) {
    case 0:
      if (vt13_rc_data->rc.trigger != 1) {
        // 停止发射逻辑
        shoot_ctrl_cmd->load_mode = LOAD_STOP;
        trigger_time = time;
      }
      break;
    default:
      // 发射逻辑
      if (vt13_rc_data->mouse_key.keyboard.e) {  // E键设置发射模式
        if (shoot_ctrl_cmd->friction_mode == FRICTION_ON) {
          if (time - trigger_time > 1.0f) {
            shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;  // 连发
          } else {
            shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;   // 单发
          }
        }
      }
      break;
  }

  shoot_ctrl_cmd->shoot_rate = 8;

  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }
}
#else
// 如果没有定义任何遥控器宏，提供空实现
static void RemoteControlSet() {}
static void MouseKeySet() {}
#endif



/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler() {
#ifdef USE_DUAL_RC
  // 旧遥控器紧急处理逻辑
  // 两switch都在下断电
    if ((switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left))||switch_is_off(rc_data[TEMP].rc.switch_left)||switch_is_off(rc_data[TEMP].rc.switch_right))  // 全部失能
    {
      robot->robot_mode = ROBOT_POWER_ON;//?
      gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
      shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
      shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
      for (int i=0;i<16;i++)
        rc_data[TEMP].key_count[KEY_PRESS][i]=0;  //复位    注意：更改键位的时候要对这里以及下面的复位进行大改。
      LOGERROR("[CMD] emergency stop!");
    } else {
      LOGINFO("[CMD] reinstate, robot ready");
    }
    if (switch_is_down(rc_data[TEMP].rc.switch_left)||switch_is_off(rc_data[TEMP].rc.switch_left))  // 发射失能
    {
      shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
      shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
      shoot_ctrl_cmd->load_mode = LOAD_STOP;
    }
    else {
      if (gimbal_ctrl_cmd->gimbal_mode!=GIMBAL_VISION)  //增加自瞄状态的优先级
        gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    }

# elifdef USE_DUAL_RC_NEW
  // 新VT13遥控器紧急处理逻辑
  if (switch_left(vt13_rc_data->rc.mode_switch)||vt13_rc_data->button_status.pause_flag==1){  // 拨杆在左或按下暂停键时断电
    robot->robot_mode = ROBOT_POWER_ON;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    LOGERROR("[CMD] emergency stop!");
  } else {
    LOGINFO("[CMD] reinstate, robot ready");
  }

  // shoot关闭
  if (switch_middle(vt13_rc_data->rc.mode_switch)) {  // 扳机按下时发射失能
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
  }

#endif
}
void Gimbal_CANCommSend()
{
  #ifdef USE_DUAL_RC
  if (can_comm_instance == NULL || rc_data == NULL)
  {
    return;
  }

  send_data->Rc_vx = rc_data->rc.rocker_l_ + rc_data[TEMP].key[KEY_PRESS].d * 660 - rc_data[TEMP].key[KEY_PRESS].a * 660;

  send_data->Rc_vy = rc_data->rc.rocker_l1 + rc_data[TEMP].key[KEY_PRESS].w * 660 - rc_data[TEMP].key[KEY_PRESS].s * 660;

  send_data->Rc_yaw = rc_data->rc.rocker_r_ + rc_data[TEMP].mouse.x * 2.0f;

  send_data->Rc_vw = rc_data->rc.dial + rc_data[TEMP].key[KEY_PRESS].q*300;

  send_data->Yaw_single_round = (int16_t)robot->gimbal->yaw_motor->measure.angle_single_round;

  send_data->Switch_right = rc_data->rc.switch_right;
  CANCommSend(can_comm_instance,(void*)send_data);
  #elifdef USE_DUAL_RC_NEW
  if (can_comm_instance == NULL || vt13_rc_data == NULL)
  {
    return;
  }

  send_data_new->Rc_vx = vt13_rc_data->rc.rocker_l_ + vt13_rc_data->mouse_key.keyboard.d * 660 - vt13_rc_data->mouse_key.keyboard.a * 660;

  send_data_new->Rc_vy = vt13_rc_data->rc.rocker_l1 + vt13_rc_data->mouse_key.keyboard.w * 660 - vt13_rc_data->mouse_key.keyboard.s * 660;

  send_data_new->Rc_yaw = vt13_rc_data->rc.rocker_r_ + vt13_rc_data->mouse_key.mouse.x * 2;

  send_data_new->Rc_vw = vt13_rc_data->rc.dial + vt13_rc_data->mouse_key.keyboard.q*300;

  send_data_new->Yaw_single_round = (int16_t)robot->gimbal->yaw_motor->measure.angle_single_round;

  send_data_new->Mode_switch = vt13_rc_data->rc.mode_switch;

  send_data_new->Control_mode = robot->control_mode;

  send_data_new->Pause_flag = vt13_rc_data->button_status.pause_flag;
  CANCommSend(can_comm_instance,(void*)send_data_new);
  #endif


  }
static void DualBoardCtrlSet() {
  //chassis_ctrl_cmd->wz=0;
  if (CANCommIsOnline(can_comm_instance)) {
    // 检查是否有新数据更新
    *RefereeData = *(Referee_Data*)CANCommGet(can_comm_instance);
    // 如果收到数据，可以在这里处理
  }
}
void RobotInit() {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

#ifdef USE_DUAL_RC
  // 使用旧遥控器
  robot->rc_data = RemoteControlInit(&huart3);  // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
  rc_data_last = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
  *rc_data_last = *robot->rc_data;  // 记录上一次遥控器的状态
  rc_data = robot->rc_data;
#elif defined(USE_DUAL_RC_NEW)
  // 使用新VT13遥控器
  vt13_rc_data = (VT13_RC_t *) zmalloc(sizeof(VT13_RC_t));
  vt13_rc_data = VT13RemoteInit(&huart6);
#endif

  robot->vision_recv_data = VisionInit(&gimbal_init_config.imu_init_config);
  // robot->super_cap = SuperCapInit(&super_cap_config);

  robot->gimbal = GimbalInit(&gimbal_init_config);
  robot->shoot = ShootInit(&shoot_init_config);

  // 初始化控制命令指针
  // chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  // chassis_ctrl_cmd->max_power = 80;  // 随便给一个初始功率，后面应该要从裁判系统获取
  gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  // navigator_data  = robot->navigator_data;
  vision_recv_data=VisionInit(&gimbal_init_config.imu_init_config);
  can_comm_instance = CANCommInit(&comm_config);
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
  time = DWT_GetTimeline_s();
  RemoteControlSet();
  DualBoardCtrlSet();
  MouseKeySet();
  EmergencyHandler();  // 处理模块离线和遥控器急停等紧急情况
}

void RobotTask() {
  VisionSend();
  RobotCMDTask();
  Gimbal_CANCommSend();
  GimbalTask();
  ShootTask();

  // 正确的赋值方式 - 直接赋值指针值
  // robot->shoot->friction_motor[1];
}
