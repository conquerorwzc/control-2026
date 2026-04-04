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
static Vision_Receive_s *vision_recv_data;
// static navigator_recv_t* navigator_data;
#ifdef USE_DUAL_RC
static Send_Data_RC *rc_data_old;
static RC_ctrl_t *rc_data;
static RC_ctrl_t *rc_data_last;  // 遥控器数据,初始化时返回
#elifdef USE_DUAL_RC_NEW
static Send_Data_RC_NEW *rc_data_new;
static VT13_RC_t *vt13_rc_data;
#endif
static Sentry_Cmd_t sentry_cmd = {0};
static SuperCapMode supercap_mode = SAFETY_MODE;
float trigger_time = 0;  // 触发时间
static float angle = 0;
CANCommInstance *can_comm_instance = NULL;
static Referee_Data *referee_data;
static float time=0;  //判断按钮按下需要重复读取时间，这里简化成一次读取

static float x_speed_time = 0;  // x方向加速触发时间
static float y_speed_time = 0;  // y方向加速触发时间
static float vx_initial;        // x轴输入控制量
static float vy_initial;        // y轴输入控制量
// static  DJIMotorInstance* debug_motor;

static uint16_t EncodeBulletSpeedToU16(float speed_mps) {
  if (speed_mps <= 0.0f) return 0u;
  if (speed_mps >= 30.0f) return (uint16_t)(30.0f * 100.0f);
  return (uint16_t)(speed_mps * 100.0f);
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
uint8_t has_non_zero_data(const Vision_Receive_s *data) {
  // 空指针检查
  if (data == NULL) {
    return 0;  // 或根据需求返回错误码
  }

  // 简化逻辑：只要任意字段非零，返回1；否则返回0
  return (data->gimbal_receive.pitch != 0) || (data->gimbal_receive.yaw != 0) || (data->shoot_receive.fire_flag != 0);
}
static void CalcOffsetAngle() {
#ifdef USE_DUAL_RC
  angle = rc_data_old->Yaw_motor_angle;
#elifdef USE_DUAL_RC_NEW
  angle = rc_data_new->Yaw_motor_angle;
#endif

  float delta = angle - YAW_ALIGN_ANGLE;
  if (delta > 180.0f) {
    delta -= 360.0f;
  } else if (delta <= -180.0f) {
    delta += 360.0f;
  }
  chassis_ctrl_cmd->offset_angle = delta;
}

static void SentryRefereeSend() {
  sentry_cmd.fields.confirm_respawn = 1;
  sentry_cmd.fields.confirm_instant_respawn = 0;
  sentry_cmd.fields.projectile_amount = 1000;
  sentry_cmd.fields.projectile_req_cnt = 1;
  sentry_cmd.fields.hp_req_cnt = 0;
  sentry_cmd.fields.sentry_mode = robot->sentry_mode;
  sentry_cmd.fields.activate_power_rune = 1;

  SentrySend(sentry_cmd.raw_data, sizeof(sentry_cmd.raw_data));
}

#if defined(USE_DUAL_RC)
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet() {
  if (switch_is_mid(rc_data[TEMP].rc.switch_left)||switch_is_up(rc_data[TEMP].rc.switch_left)) {
    if (abs(rc_data[TEMP].rc.dial) > 20) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    } else
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
  }

  if ((switch_is_up(rc_data[TEMP].rc.switch_right)||switch_is_off(rc_data[TEMP].rc.switch_right))&&robot->referee_data->GameState.game_progress == 4)//除了右拨杆在上机器人使用导航数据，其余都正常人为控制
  {
    robot->control_mode=NAVIGATOR_MODE;
  }
  else {
    robot->control_mode=MANUAL_MODE;
  }

  // 底盘控制部分,系数需要调整
  if (robot->control_mode == MANUAL_MODE)  // 手动控制，遥控器控制量
  {
    vx_initial = 60.0f * (float)rc_data[TEMP].rc.rocker_l_;  // l_水平方向，最大660*60=39600
    vy_initial = 60.0f * (float)rc_data[TEMP].rc.rocker_l1;  // l1竖直方向，最大660*60
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_ROTATE) {
      chassis_ctrl_cmd->wz = 5.0f * (float)rc_data[TEMP].rc.dial;  // 小陀螺模式下的旋转分量，如果是跟随，则在底盘任务中计算旋转分量
    }
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW) {
      chassis_ctrl_cmd->wz = (2.0f) * (float)rc_data->rc.rocker_r_;  // 主动跟随量，todo：但是感觉一个变量拆成两段写好像有点抽象，这里有一段，chassis还有另一段
    }
  } else if (robot->control_mode == NAVIGATOR_MODE)  // 自动控制，直接收上位机控制量
  {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    vx_initial = -robot->navigator_data->robot_cmd.speed_vector.vy * 10000;
    vy_initial = robot->navigator_data->robot_cmd.speed_vector.vx * 10000;
    chassis_ctrl_cmd->wz = robot->navigator_data->robot_cmd.speed_vector.wz / WZ_CMD_TO_CAR_WZ_RAD_S;
  }
  // 缓加速
  if (abs(vx_initial) <= 10000) {
    x_speed_time = time;
    chassis_ctrl_cmd->vx = vx_initial;
  }  // 速度绝对值在10000以下输出控制量=输入控制量
  if (vx_initial > 10000 && chassis_ctrl_cmd->vx <= 60.0f * (float)rc_data[TEMP].rc.rocker_l_) {
    chassis_ctrl_cmd->vx = 10000 + (time - x_speed_time) * 10000;
  }
  if (vx_initial < -10000 && chassis_ctrl_cmd->vx >= 60.0f * (float)rc_data[TEMP].rc.rocker_l_) {
    chassis_ctrl_cmd->vx = -10000 - (time - x_speed_time) * 10000;
  }  // 速度绝对值在10000以上输出控制量=10000+10000t(s)
  if (abs(vy_initial) <= 10000) {
    y_speed_time = time;
    chassis_ctrl_cmd->vy = vy_initial;
  }  // 速度绝对值在10000以下输出控制量=输入控制量
  if (vy_initial > 10000 && chassis_ctrl_cmd->vy <= 60.0f * (float)rc_data[TEMP].rc.rocker_l1) {
    chassis_ctrl_cmd->vy = 10000 + (time - y_speed_time) * 10000;
  }
  if (vy_initial < -10000 && chassis_ctrl_cmd->vy >= 60.0f * (float)rc_data[TEMP].rc.rocker_l1) {
    chassis_ctrl_cmd->vy = -10000 - (time - y_speed_time) * 10000;
  }  // 速度绝对值在10000以上输出控制量=10000+10000t(s)
  *rc_data_last = *rc_data;
}

static void MouseKeySet() {}

#elifdef USE_DUAL_RC_NEW
static void RemoteControlSet() {
  if (switch_left(vt13_rc_data->rc.mode_switch)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    if (abs(vt13_rc_data->rc.dial) > 20) chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
  }

  // 底盘控制部分,系数需要调整
  if (robot->control_mode == MANUAL_MODE)  // 手动控制，遥控器控制量
  {
    vx_initial = 60.0f * (float)vt13_rc_data->rc.rocker_l_;  // l_水平方向，最大660*60=39600
    vy_initial = 60.0f * (float)vt13_rc_data->rc.rocker_l1;  // l1竖直方向，最大660*60
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_ROTATE)
      chassis_ctrl_cmd->wz = 5.0f * (float)vt13_rc_data->rc.dial;  // 小陀螺模式下的旋转分量，如果是跟随，则在底盘任务中计算旋转分量
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW) {
      chassis_ctrl_cmd->wz = (2.0f) * (float)vt13_rc_data->rc.rocker_r_;  // 主动跟随量，todo：但是感觉一个变量拆成两段写好像有点抽象，这里有一段，chassis还有另一段
    }
  } else if (robot->control_mode == NAVIGATOR_MODE)  // 自动控制，直接收上位机控制量
  {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
      vx_initial = -robot->navigator_data->robot_cmd.speed_vector.vy * 10000;
      vy_initial = robot->navigator_data->robot_cmd.speed_vector.vx * 10000;
      chassis_ctrl_cmd->wz = robot->navigator_data->robot_cmd.speed_vector.wz / WZ_CMD_TO_CAR_WZ_RAD_S;
  }
  // 缓加速
  if (abs(vx_initial) <= 10000) {
    x_speed_time = time;
    chassis_ctrl_cmd->vx = vx_initial;
  }  // 速度绝对值在10000以下输出控制量=输入控制量
  if (vx_initial > 10000 && chassis_ctrl_cmd->vx <= 60.0f * (float)vt13_rc_data->rc.rocker_l_) {
    chassis_ctrl_cmd->vx = 10000 + (time - x_speed_time) * 10000;
  }
  if (vx_initial < -10000 && chassis_ctrl_cmd->vx >= 60.0f * (float)vt13_rc_data->rc.rocker_l_) {
    chassis_ctrl_cmd->vx = -10000 - (time - x_speed_time) * 10000;
  }  // 速度绝对值在10000以上输出控制量=10000+10000t(s)
  if (abs(vy_initial) <= 10000) {
    y_speed_time = time;
    chassis_ctrl_cmd->vy = vy_initial;
  }  // 速度绝对值在10000以下输出控制量=输入控制量
  if (vy_initial > 10000 && chassis_ctrl_cmd->vy <= 60.0f * (float)vt13_rc_data->rc.rocker_l1) {
    chassis_ctrl_cmd->vy = 10000 + (time - y_speed_time) * 10000;
  }
  if (vy_initial < -10000 && chassis_ctrl_cmd->vy >= 60.0f * (float)vt13_rc_data->rc.rocker_l1) {
    chassis_ctrl_cmd->vy = -10000 - (time - y_speed_time) * 10000;
  }  // 速度绝对值在10000以上输出控制量=10000+10000t(s)
}

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
  // 底盘双板通信离线,好痛
  if (!CANCommIsOnline(can_comm_instance)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    LOGERROR("[CMD] Emergency Stop! DualBoardComm Lost");
  } else
    LOGINFO("[CMD]DualBoardComm is Online");

  if (robot->navigator_data->robot_cmd.is_recovering==1 && robot->referee_data->GameRobotState.current_HP<robot->referee_data->GameRobotState.maximum_HP*0.8) {
    chassis_ctrl_cmd->chassis_mode=CHASSIS_POWER_OFF;
  }

#ifdef USE_DUAL_RC

  if (switch_is_down(rc_data[TEMP].rc.switch_left))  // 底盘失能
  {
    robot->robot_mode = ROBOT_POWER_ON;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
  }

#elifdef USE_DUAL_RC_NEW
  if (switch_right(vt13_rc_data->rc.mode_switch) || vt13_rc_data->button_status.pause_flag == 1)  // 底盘失能
  {
    robot->robot_mode = ROBOT_POWER_ON;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
  }
#endif
}

static void ModeControl() {
  if (robot->control_mode==NAVIGATOR_MODE){
    // if (robot->referee_data->ProjectileAllowance.projectile_allowance_17mm==0) {
    //   robot->sentry_mode=DEFENSE_POSE;    //无可用弹丸进入防御姿态
    //   robot->chassis->chassis_ctrl_cmd.wz=-1500;
    // }
    // else
    if (robot->chassis->chassis_ctrl_cmd.vx==0&&robot->chassis->chassis_ctrl_cmd.vy==0) {
      robot->sentry_mode=OFFENSE_POSE;    //高于50%血或占据堡垒进入进攻姿态
      robot->chassis->chassis_ctrl_cmd.wz=-3500;
    }
    else {
      robot->sentry_mode=MOBILITY_POSE;
      // robot->chassis->chassis_ctrl_cmd.wz=-1500;
    }
  }
}
static void SuperCapControl() {
  switch (supercap_mode) {
    case SAFETY_MODE:
      if (robot->super_cap->cap_msg.cap_v > 18.0f) supercap_mode = PASSIVE_MODE;
      robot->chassis->chassis_ctrl_cmd.max_power = 0;
      break;
    case FORCED_CHARGING_MODE:
      if (robot->super_cap->cap_msg.cap_v < 8.0f) supercap_mode = SAFETY_MODE;
      if (robot->super_cap->cap_msg.cap_v > 18.0f) supercap_mode = PASSIVE_MODE;
      robot->chassis->chassis_ctrl_cmd.max_power =
          (uint16_t)(0.4 * robot->referee_data->GameRobotState.chassis_power_limit);
      break;
    case CHARGING_MODE:
      if (robot->super_cap->cap_msg.cap_v < 10.0f) supercap_mode = FORCED_CHARGING_MODE;
      if (robot->super_cap->cap_msg.cap_v > 18.0f) supercap_mode = PASSIVE_MODE;
      robot->chassis->chassis_ctrl_cmd.max_power =
          robot->referee_data->GameRobotState.chassis_power_limit -
          (uint16_t)powf((float)robot->referee_data->GameRobotState.chassis_power_limit * 0.04f, 2);
      break;
    case PASSIVE_MODE:
      if (chassis_ctrl_cmd->max_power == 180) supercap_mode = ACTIVE_MODE;
      if (robot->super_cap->cap_msg.cap_v < 12.0f) supercap_mode = CHARGING_MODE;
      robot->chassis->chassis_ctrl_cmd.max_power = robot->referee_data->GameRobotState.chassis_power_limit;
      break;
    case ACTIVE_MODE:
      if (robot->super_cap->cap_msg.cap_v < 12.0f) supercap_mode = CHARGING_MODE;
      if (chassis_ctrl_cmd->max_power != 180) supercap_mode = PASSIVE_MODE;
      chassis_ctrl_cmd->max_power = 180;
      break;
    default:
      supercap_mode = SAFETY_MODE;
  }
  SuperCapSendMessage(robot->super_cap, (int16_t)robot->referee_data->GameRobotState.chassis_power_limit,
                      robot->referee_data->PowerHeatData.buffer_energy,
                      robot->referee_data->GameRobotState.power_management_chassis_output);
}
void Chassis_CANCommSend() {
#ifdef USE_DUAL_RC
  if (can_comm_instance == NULL || rc_data == NULL) {
    return;
  }
#elifdef USE_DUAL_RC_NEW
  if (can_comm_instance == NULL || vt13_rc_data == NULL) {
    return;
  }
#endif
  // referee_data->projectile_allowance_17mm = robot->referee_data->ProjectileAllowance.projectile_allowance_17mm;
  referee_data->initial_speed = EncodeBulletSpeedToU16(robot->referee_data->ShootData.initial_speed);
  referee_data->shooter_17mm_barrel_heat = robot->referee_data->PowerHeatData.shooter_17mm_barrel_heat;
  referee_data->robot_id = robot->referee_data->GameRobotState.robot_id;
  CANCommSend(can_comm_instance, (void *)referee_data);
}
// 解析底盘板收到的遥控数据
static void DualBoardCtrlSet() {
  if (CANCommIsOnline(can_comm_instance)) {
#ifdef USE_DUAL_RC
    *rc_data_old = *(Send_Data_RC *)CANCommGet(can_comm_instance);
#elifdef USE_DUAL_RC_NEW
    *rc_data_new = *(Send_Data_RC_NEW *)CANCommGet(can_comm_instance);
#endif

#ifdef USE_DUAL_RC
    rc_data[TEMP].rc.rocker_l_ = rc_data_old->Rc_vx;  // todo:后面chassis改改把负号去掉
    rc_data[TEMP].rc.rocker_l1 = rc_data_old->Rc_vy;
    rc_data[TEMP].rc.rocker_r_ = rc_data_old->Rotate_speed;
    rc_data[TEMP].rc.dial = rc_data_old->Spin_speed;
    rc_data[TEMP].rc.switch_left = rc_data_old->rc_switch_left;
    rc_data[TEMP].rc.switch_right = rc_data_old->rc_switch_right;
    // robot->control_mode=rc_data_old->Control_mode;
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
  // 要在云台和底盘任务开始之前完成该任务的初始化
  vTaskDelay(CAN_COMM_TASK_INIT_TIME);
  // 初始化CAN接收
  can_comm_instance = CANCommInit(&comm_config);
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));
  referee_data = (Referee_Data *)zmalloc(sizeof(Referee_Data));
#ifdef USE_DUAL_RC
  // 使用旧遥控器
  rc_data_old = (Send_Data_RC *)zmalloc(sizeof(Send_Data_RC));
  robot->rc_data = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
  rc_data_last = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
  *rc_data_last = *robot->rc_data;  // 记录上一次遥控器的状态
  rc_data = robot->rc_data;
#elif defined(USE_DUAL_RC_NEW)
  // 使用新VT13遥控器
  robot->vt13_rc_data = (VT13_RC_t *)zmalloc(sizeof(VT13_RC_t));
  vt13_rc_data = robot->vt13_rc_data;
  rc_data_new = (Send_Data_RC_NEW *)zmalloc(sizeof(Send_Data_RC_NEW));
#endif

  // robot->vision_recv_data = VisionInit(&gimbal_init_config.imu_init_config);
  robot->navigator_data = navigator_init(&huart1);

  robot->referee_data = RefereeInit(&huart6);  // 裁判系统初始化
  robot->sentry_mode = 1;

  // robot->super_cap = SuperCapInit(&super_cap_config);

  robot->chassis = ChassisInit(&chassis_init_config);
  // 初始化控制命令指针
  chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  // navigator_data  = robot->navigator_data;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发频率) */
void RobotCMDTask() {
  static float last_rc_dualboard_time = 0.0f;
  static uint8_t rc_dualboard_first_run = 1;
  time = DWT_GetTimeline_s();
  // 双板数据按100Hz更新，其他安全逻辑维持高频
  if (rc_dualboard_first_run || (time - last_rc_dualboard_time) >= 0.012f) {
    rc_dualboard_first_run = 0;
    last_rc_dualboard_time = time;
    Chassis_CANCommSend();
    // SentryRefereeSend();
  }
  DualBoardCtrlSet();
  CalcOffsetAngle();
  RemoteControlSet();
  // MouseKeySet();
  EmergencyHandler();  // 处理模块离线和遥控器急停等紧急情况
}

void RobotTask() {
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
  GimbalTask();
#endif
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  navigator_send(&huart1, robot->referee_data);
  RobotCMDTask();
  // SuperCapControl();
  chassis_ctrl_cmd->max_power = robot->referee_data->GameRobotState.chassis_power_limit;
  ModeControl();
  ChassisTask();
#endif
}
