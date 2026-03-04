#include "robot.h"
#include "can_comm.h"
#include "general_def.h"
#include "master_process.h"
#include "new_RC_VT13.h"
#include "robot_config.h"
#include "user_lib.h"

static RobotInstance *robot;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd;
static Shoot_Ctrl_Cmd_s *shoot_ctrl_cmd;
static Vision_Receive_s* vision_recv_data;
// static navigator_recv_t* navigator_data;
#ifdef USE_DUAL_RC
static Send_Data_RC *rc_data_old;
static RC_ctrl_t *rc_data;
static RC_ctrl_t *rc_data_last;  // 遥控器数据,初始化时返回
#elifdef USE_DUAL_RC_NEW
static Send_Data_RC_NEW *rc_data_new;
static VT13_RC_t *vt13_rc_data;
#endif
static Sentry_Cmd_t sentry_cmd={0};
static SuperCapMode supercap_mode = SAFETY_MODE;
/* Intermediate variables calculated by private functions */
float trigger_time = 0;  // 触发时间
static float angle=0;
CANCommInstance* can_comm_instance = NULL;
static Referee_Data *referee_data;


static float x_speed_time=0;  //x方向加速触发时间
static float y_speed_time=0;  //y方向加速触发时间
static float vx_initial;   //x轴输入控制量
static float vy_initial;   //y轴输入控制量
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
  #ifdef USE_DUAL_RC
  angle = rc_data_old->Rotate_speed;
#elifdef USE_DUAL_RC_NEW
  angle = rc_data_new->Yaw_motor_angle;
#endif

  float delta = angle-YAW_ALIGN_ANGLE;
  if (delta > 180.0f) {
    delta -= 360.0f;
  } else if (delta <= -180.0f) {
    delta += 360.0f;
  }
  chassis_ctrl_cmd->offset_angle = delta;
}

static void SentryRefereeSend() {
  sentry_cmd.fields.confirm_respawn=1;
  sentry_cmd.fields.confirm_instant_respawn=0;
  sentry_cmd.fields.projectile_amount=1000;
  sentry_cmd.fields.projectile_req_cnt=1;
  sentry_cmd.fields.hp_req_cnt=0;
  sentry_cmd.fields.sentry_mode= robot->sentry_mode;
  sentry_cmd.fields.activate_power_rune=1;

  SentrySend(sentry_cmd.raw_data,sizeof(sentry_cmd.raw_data));
}

#if defined(USE_DUAL_RC)
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
  if (switch_is_up(rc_data[TEMP].rc.switch_right))//除了右拨杆在上机器人使用导航数据，其余都正常人为控制
  {
    robot->control_mode=AUTO_MODE;
  }
  else {
    robot->control_mode=MANUAL_MODE;
  }
  // 云台使能,或视觉未识别到目标,纯遥控器拨杆控制
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {  // 按照摇杆的输出大小进行角度增量,增益系数需调整
    gimbal_ctrl_cmd->yaw += -0.005f * (float)rc_data[TEMP].rc.rocker_r_;
    gimbal_ctrl_cmd->pitch -= 0.0003f * (float)rc_data[TEMP].rc.rocker_r1;
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
    vx_initial = 60.0f * (float)rc_data[TEMP].rc.rocker_l_;  // l_水平方向，最大660*60=39600
    vy_initial = 60.0f * (float)rc_data[TEMP].rc.rocker_l1;  // l1竖直方向，最大660*60
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_ROTATE) {
      chassis_ctrl_cmd->wz =
          20.0f * (float)rc_data[TEMP].rc.dial;  // 小陀螺模式下的旋转分量，如果是跟随，则在底盘任务中计算旋转分量
    }
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW) {
      chassis_ctrl_cmd->wz =(1.0f) *(float)rc_data[TEMP].rc.rocker_r_;  // 主动跟随量，todo：但是感觉一个变量拆成两段写好像有点抽象，这里有一段，chassis还有另一段
    }


  } else if (robot->control_mode == AUTO_MODE) // 自动控制，直接收上位机控制量
  {
    vx_initial = -robot->navigator_data->robot_cmd.speed_vector.vy*10000;
    //vx_initial = -robot->navigator_data->robot_cmd.speed_vector.vx*5000;
    vy_initial = robot->navigator_data->robot_cmd.speed_vector.vx*10000;
    chassis_ctrl_cmd->wz = robot->navigator_data->robot_cmd.speed_vector.wz*0;
    //gimbal_ctrl_cmd->yaw-=robot->navigator_data->robot_cmd.speed_vector.wz*0.01;
  }
  //缓加速
  if (abs(vx_initial)<=10000) {
    x_speed_time=DWT_GetTimeline_s();
    chassis_ctrl_cmd->vx=vx_initial;
  }//速度绝对值在10000以下输出控制量=输入控制量
  if (vx_initial > 10000 && chassis_ctrl_cmd->vx<= 60.0f * (float)rc_data[TEMP].rc.rocker_ ) {
    chassis_ctrl_cmd->vx=10000+(DWT_GetTimeline_s()-x_speed_time)*10000;
  }
  if (vx_initial < -10000 && chassis_ctrl_cmd->vx>= 60.0f * (float)rc_data[TEMP].rc.rocker_l_) {
    chassis_ctrl_cmd->vx=-10000-(DWT_GetTimeline_s()-x_speed_time)*10000;
  }//速度绝对值在10000以上输出控制量=10000+10000t(s)
  if (abs(vy_initial)<=10000) {
    y_speed_time=DWT_GetTimeline_s();
    chassis_ctrl_cmd->vy=vy_initial;
  }//速度绝对值在10000以下输出控制量=输入控制量
  if (vy_initial > 10000 && chassis_ctrl_cmd->vy<= 60.0f * (float)rc_data[TEMP].rc.rocker_l1 ) {
    chassis_ctrl_cmd->vy=10000+(DWT_GetTimeline_s()-y_speed_time)*10000;
  }
  if (vy_initial < -10000 && chassis_ctrl_cmd->vy>= 60.0f * (float)rc_data[TEMP].rc.rocker_l1) {
    chassis_ctrl_cmd->vy=-10000-(DWT_GetTimeline_s()-y_speed_time)*10000;
  }//速度绝对值在10000以上输出控制量=10000+10000t(s)
  *rc_data_last = *rc_data;
}

static void MouseKeySet() {
  vy_initial += (float)((rc_data[TEMP].key[KEY_PRESS].w) - rc_data[TEMP].key[KEY_PRESS].s) *
                (float)chassis_ctrl_cmd->chassis_speed_buff;
  vx_initial += (float)(rc_data[TEMP].key[KEY_PRESS].a - rc_data[TEMP].key[KEY_PRESS].d) *
                (float)-chassis_ctrl_cmd->chassis_speed_buff;

    //缓加速
  if (abs(vx_initial)<=10000) {
    x_speed_time=DWT_GetTimeline_s();
    chassis_ctrl_cmd->vx=vx_initial;
  }//速度绝对值在10000以下输出控制量=输入控制量
  if (vx_initial > 10000&&chassis_ctrl_cmd->vx<= 60.0f * (float)rc_data[TEMP].rc.rocker_l_ ) {
    chassis_ctrl_cmd->vx=10000+(DWT_GetTimeline_s()-x_speed_time)*10000;
  }
  if (vx_initial < -10000&&chassis_ctrl_cmd->vx>= 60.0f * (float)rc_data[TEMP].rc.rocker_l_) {
    chassis_ctrl_cmd->vx=-10000-(DWT_GetTimeline_s()-x_speed_time)*10000;
  }//速度绝对值在10000以上输出控制量=10000+10000t(s)
  if (abs(vy_initial)<=10000) {
    y_speed_time=DWT_GetTimeline_s();
    chassis_ctrl_cmd->vy=vy_initial;
  }//速度绝对值在10000以下输出控制量=输入控制量
  if (vy_initial > 10000&&chassis_ctrl_cmd->vy<= 60.0f * (float)rc_data[TEMP].rc.rocker_l1 ) {
    chassis_ctrl_cmd->vy=10000+(DWT_GetTimeline_s()-y_speed_time)*10000;
  }
  if (vy_initial < -10000&&chassis_ctrl_cmd->vy>= 60.0f * (float)rc_data[TEMP].rc.rocker_l1) {
    chassis_ctrl_cmd->vy=-10000-(DWT_GetTimeline_s()-y_speed_time)*10000;
  }//速度绝对值在10000以上输出控制量=10000+10000t(s)

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

        gimbal_ctrl_cmd->yaw=1.0f*vision_recv_data->gimbal_receive.yaw;
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
        if (shoot_ctrl_cmd->friction_mode==FRICTION_ON&&(vision_recv_data->shoot_receive.fire_flag||rc_data[TEMP].mouse.press_r % 2==0))   //需预先开启摩擦轮
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
      chassis_ctrl_cmd->chassis_speed_buff = 30000;
      break;
    default:
      chassis_ctrl_cmd->chassis_speed_buff = 40000;
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

#elifdef USE_DUAL_RC_NEW
static void RemoteControlSet() {
  if (switch_middle(vt13_rc_data->rc.mode_switch)||switch_right(vt13_rc_data->rc.mode_switch)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
    if (abs(vt13_rc_data->rc.dial) > 20)
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
  }

  // 底盘控制部分,系数需要调整
  if (robot->control_mode == MANUAL_MODE)//手动控制，遥控器控制量
  {
    vx_initial = 60.0f * (float)vt13_rc_data->rc.rocker_l_;  // l_水平方向，最大660*60=39600
    vy_initial = 60.0f * (float)vt13_rc_data->rc.rocker_l1;  // l1竖直方向，最大660*60
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_ROTATE)
      chassis_ctrl_cmd->wz = 20.0f * (float)vt13_rc_data->rc.dial;  // 小陀螺模式下的旋转分量，如果是跟随，则在底盘任务中计算旋转分量
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW) {
      chassis_ctrl_cmd->wz =(2.0f) *(float)vt13_rc_data->rc.rocker_r_;  // 主动跟随量，todo：但是感觉一个变量拆成两段写好像有点抽象，这里有一段，chassis还有另一段
    }
  } else if (robot->control_mode == AUTO_MODE) // 自动控制，直接收上位机控制量
  {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    vx_initial = -robot->navigator_data->robot_cmd.speed_vector.vy*10000;
    //vx_initial = -robot->navigator_data->robot_cmd.speed_vector.vx*5000;
    vy_initial = robot->navigator_data->robot_cmd.speed_vector.vx*10000;
    chassis_ctrl_cmd->wz = robot->navigator_data->robot_cmd.speed_vector.wz*100;
    //gimbal_ctrl_cmd->yaw-=robot->navigator_data->robot_cmd.speed_vector.wz*0.01;
  }
  //缓加速
  if (abs(vx_initial)<=10000) {
    x_speed_time=DWT_GetTimeline_s();
    chassis_ctrl_cmd->vx=vx_initial;
  }//速度绝对值在10000以下输出控制量=输入控制量
  if (vx_initial > 10000 && chassis_ctrl_cmd->vx<= 60.0f * (float)vt13_rc_data->rc.rocker_l_) {
    chassis_ctrl_cmd->vx=10000+(DWT_GetTimeline_s()-x_speed_time)*10000;
  }
  if (vx_initial < -10000 && chassis_ctrl_cmd->vx>= 60.0f * (float)vt13_rc_data->rc.rocker_l_) {
    chassis_ctrl_cmd->vx=-10000-(DWT_GetTimeline_s()-x_speed_time)*10000;
  }//速度绝对值在10000以上输出控制量=10000+10000t(s)
  if (abs(vy_initial)<=10000) {
    y_speed_time=DWT_GetTimeline_s();
    chassis_ctrl_cmd->vy=vy_initial;
  }//速度绝对值在10000以下输出控制量=输入控制量
  if (vy_initial > 10000 && chassis_ctrl_cmd->vy<= 60.0f * (float)vt13_rc_data->rc.rocker_l1) {
    chassis_ctrl_cmd->vy=10000+(DWT_GetTimeline_s()-y_speed_time)*10000;
  }
  if (vy_initial < -10000 && chassis_ctrl_cmd->vy>= 60.0f * (float)vt13_rc_data->rc.rocker_l1) {
    chassis_ctrl_cmd->vy=-10000-(DWT_GetTimeline_s()-y_speed_time)*10000;
  }//速度绝对值在10000以上输出控制量=10000+10000t(s)
}

static void MouseKeySet() {
}

#endif


/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler() {
  //底盘双板通信离线,好痛
  if (!CANCommIsOnline(can_comm_instance)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    LOGERROR("[CMD] Emergency Stop! DualBoardComm Lost");
  }
  else LOGINFO("[CMD]DualBoardComm is Online");

#ifdef USE_DUAL_RC

      if (switch_is_down(rc_data_old->Switch_right))  // 底盘失能
      {
        robot->robot_mode = ROBOT_POWER_ON;
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
      }
      // 遥控器右侧开关为[上],恢复正常运行

#elifdef  USE_DUAL_RC_NEW
  if (switch_left(vt13_rc_data->rc.mode_switch)||vt13_rc_data->button_status.pause_flag==1)  // 底盘失能
  {
    robot->robot_mode = ROBOT_POWER_ON;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
  }

#endif

}
static void SuperCapControl() {
  switch (supercap_mode) {
    case SAFETY_MODE:
      if (robot->super_cap->cap_msg.cap_v>18.0f)
        supercap_mode=PASSIVE_MODE;
      robot->chassis->chassis_ctrl_cmd.max_power=0;
      break;
    case FORCED_CHARGING_MODE:
      if (robot->super_cap->cap_msg.cap_v<8.0f)
        supercap_mode=SAFETY_MODE;
      if (robot->super_cap->cap_msg.cap_v>18.0f)
        supercap_mode=PASSIVE_MODE;
      robot->chassis->chassis_ctrl_cmd.max_power=(uint16_t)(0.4*robot->referee_data->GameRobotState.chassis_power_limit);
      break;
    case CHARGING_MODE:
      if (robot->super_cap->cap_msg.cap_v<10.0f)
        supercap_mode=FORCED_CHARGING_MODE;
      if (robot->super_cap->cap_msg.cap_v>18.0f)
        supercap_mode=PASSIVE_MODE;
      robot->chassis->chassis_ctrl_cmd.max_power=robot->referee_data->GameRobotState.chassis_power_limit-(uint16_t)powf((float)robot->referee_data->GameRobotState.chassis_power_limit*0.04f,2);
      break;
    case PASSIVE_MODE:
      if (chassis_ctrl_cmd->max_power==180)
        supercap_mode=ACTIVE_MODE;
      if (robot->super_cap->cap_msg.cap_v<12.0f)
        supercap_mode=CHARGING_MODE;
      robot->chassis->chassis_ctrl_cmd.max_power=robot->referee_data->GameRobotState.chassis_power_limit;
      break;
    case ACTIVE_MODE:
      if (robot->super_cap->cap_msg.cap_v<12.0f)
        supercap_mode=CHARGING_MODE;
      if (chassis_ctrl_cmd->max_power!=180)
        supercap_mode=PASSIVE_MODE;
      chassis_ctrl_cmd->max_power=180;
      break;
    default:
      supercap_mode=SAFETY_MODE;
  }
  SuperCapSendMessage(robot->super_cap,
        (int16_t)robot->referee_data->GameRobotState.chassis_power_limit,
        robot->referee_data->PowerHeatData.buffer_energy,
        robot->referee_data->GameRobotState.power_management_chassis_output);
}
void Chassis_CANCommSend()
{
  #ifdef USE_DUAL_RC
  if (can_comm_instance == NULL || rc_data == NULL)
  {
    return;
  }
  referee_data->projectile_allowance_17mm = robot->referee_data->ProjectileAllowance.projectile_allowance_17mm;

  referee_data->buffer_energy = robot->referee_data->PowerHeatData.buffer_energy;

  referee_data->shooter_17mm_barrel_heat = robot->referee_data->PowerHeatData.shooter_17mm_barrel_heat;

  CANCommSend(can_comm_instance, (void*)referee_data);
  #elifdef USE_DUAL_RC_NEW
  if (can_comm_instance == NULL || vt13_rc_data == NULL)
  {
    return;
  }

  referee_data->projectile_allowance_17mm = robot->referee_data->ProjectileAllowance.projectile_allowance_17mm;

  referee_data->buffer_energy = robot->referee_data->PowerHeatData.buffer_energy;

  referee_data->shooter_17mm_barrel_heat = robot->referee_data->PowerHeatData.shooter_17mm_barrel_heat;
  #endif

  CANCommSend(can_comm_instance, (void*)referee_data);
  }
//解析底盘板收到的遥控数据
static void DualBoardCtrlSet() {
  if (CANCommIsOnline(can_comm_instance)) {
#ifdef USE_DUAL_RC
    *rc_data_old = *(Send_Data_RC*)CANCommGet(can_comm_instance);
#elifdef USE_DUAL_RC_NEW
    *rc_data_new = *(Send_Data_RC_NEW*)CANCommGet(can_comm_instance);
#endif

#ifdef USE_DUAL_RC
      rc_data[TEMP].rc.rocker_l_=rc_data_old->Rc_vx;//todo:后面chassis改改把负号去掉
      rc_data[TEMP].rc.rocker_l1=rc_data_old->Rc_vy;
      rc_data[TEMP].rc.rocker_r_=rc_data_old->Rotate_speed;
      //if (rc_data_new->Rotate_speed>=0)
      // chassis_ctrl_cmd->wz=(45.0f-(45.0f-20.0f)*expf((float)-rc_data_new->Rotate_speed/50.0f))*rc_data_new->Rotate_speed;
      // else chassis_ctrl_cmd->wz=(45.0f-(45.0f-20.0f)*expf((float)rc_data_new->Rotate_speed/50.0f))*rc_data_new->Rotate_speed;
      rc_data[TEMP].rc.dial=rc_data_old->Spin_speed;
      rc_data[TEMP].rc.switch_right = rc_data_old->Switch_right;

      if (switch_is_mid(rc_data_old->Switch_right)){
        //gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
        if (abs(rc_data_old->Spin_speed) > 20) {
          chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
        } else
          chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
      }

#elifdef USE_DUAL_RC_NEW
      vt13_rc_data->rc.rocker_l_ = rc_data_new->Rc_vx;
      vt13_rc_data->rc.rocker_l1 = rc_data_new->Rc_vy;
      vt13_rc_data->rc.rocker_r_ = rc_data_new->Rotate_speed;
      vt13_rc_data->rc.dial = rc_data_new->Spin_speed;
      vt13_rc_data->rc.mode_switch = rc_data_new->Mode_switch;
      robot->control_mode = rc_data_new->Control_mode;
      vt13_rc_data->button_status.pause_flag = rc_data_new->Pause_flag;
#endif
    }
  }


void RobotInit() {
  //要在云台和底盘任务开始之前完成该任务的初始化
  vTaskDelay(CAN_COMM_TASK_INIT_TIME);
  // 初始化CAN接收
  can_comm_instance = CANCommInit(&comm_config);
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));
  referee_data = (Referee_Data* )zmalloc(sizeof(Referee_Data));
  #ifdef USE_DUAL_RC
    // 使用旧遥控器
    rc_data_old = (Send_Data_RC *)zmalloc(sizeof(Send_Data_RC));
    rc_data_last = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
    *rc_data_last = *robot->rc_data;  // 记录上一次遥控器的状态
    rc_data = robot->rc_data;
  #elif defined(USE_DUAL_RC_NEW)
    // 使用新VT13遥控器
    robot->vt13_rc_data = (VT13_RC_t *)zmalloc(sizeof(VT13_RC_t));
    vt13_rc_data = robot->vt13_rc_data;
    rc_data_new = (Send_Data_RC_NEW *)zmalloc(sizeof(Send_Data_RC_NEW));
  #endif

  //robot->vision_recv_data = VisionInit(&gimbal_init_config.imu_init_config);
  robot->navigator_data = navigator_init(&huart1);

  robot->referee_data = RefereeInit(&huart6);  // 裁判系统初始化
  robot->sentry_mode=1;

  // robot->super_cap = SuperCapInit(&super_cap_config);

  robot->chassis = ChassisInit(&chassis_init_config);
  // 初始化控制命令指针
  chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  // navigator_data  = robot->navigator_data;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发频率) */
void RobotCMDTask() {
  // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
  DualBoardCtrlSet();
  Chassis_CANCommSend();
  CalcOffsetAngle();
  RemoteControlSet();
  // MouseKeySet();
  SentryRefereeSend();
  EmergencyHandler();  // 处理模块离线和遥控器急停等紧急情况
}

void RobotTask() {
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
  GimbalTask();
#endif
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  navigator_send(&huart1,robot->referee_data);
  RobotCMDTask();
  // SuperCapControl();
  chassis_ctrl_cmd->max_power=robot->referee_data->GameRobotState.chassis_power_limit;
  ChassisTask();
#endif
}
