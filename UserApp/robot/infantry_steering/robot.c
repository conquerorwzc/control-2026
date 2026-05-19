#include "robot.h"
#include "can_comm.h"
#include "general_def.h"
#include "referee_task.h"
#include "robot_config.h"
//#include "super_cap.h"
#include "user_lib.h"
#include "stdlib.h"
#define USECANREMOTE 1 //是否使用云台板的遥控数据
#define U16TOI16(u)((u)>32767?32767:(int16_t)(u))
#pragma pack(1)
typedef struct {
  float bullet_speed;
  uint16_t HP;
  // uint16_t Heat;
  //   uint16_t heat_limt;
    int16_t Remain_Heat;
    //float power;
    float cap_v;
    //uint8_t error_code;
    uint8_t color;
} upload_data;
#pragma pack()

static RobotInstance *robot;
static upload_data upload;
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd;
static Shoot_Ctrl_Cmd_s *shoot_ctrl_cmd;
//static RC_ctrl_t *rc_data;
//static RC_ctrl_t *rc_data_last;  // 遥控器数据,初始化时返回
float temp=24.0f;
CanComm_Pack* cancomm_pack;
Referee_Interactive_info_t *interactive_data;
/* Intermediate variables calculated by private functions */
static float trigger_time = 0;  // 触发时间
static float angle=0;
uint8_t* received_data = NULL;
CANCommInstance* can_comm_instance = NULL;
float noise_05(void)
{
    // rand() ∈ [0, RAND_MAX] → 映射到 [-0.5, 0.5]
    return ((float)rand() / (float)RAND_MAX) - 0.5f;
}

//连续混乱曲线 + ±0.5 噪声
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

    // 叠加 ±0.5 噪声
    out += noise_05();

    return out;
}
/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */

//解析底盘板收到的遥控数据
//value16数组0表示左遥感横向，1表示纵向，2表示右摇杆横，3表示左侧滚轮，byte10表示右侧拨杆
static void DualBoardCtrlSet() {
  //chassis_ctrl_cmd->wz=0;
  if (CANCommIsOnline(can_comm_instance)) {
    // 检查是否有新数据更新
    cancomm_pack = (CanComm_Pack*)CANCommGet(can_comm_instance);
    //robot->chassis->chassis_ctrl_cmd=*chassis_ctrl_cmd;

    robot->chassis->chassis_ctrl_cmd.vx = cancomm_pack->chassis_ctrl_can_comm.vx;
    robot->chassis->chassis_ctrl_cmd.vy = cancomm_pack->chassis_ctrl_can_comm.vy;
    robot->chassis->chassis_ctrl_cmd.wz = cancomm_pack->chassis_ctrl_can_comm.wz;
    robot->chassis->chassis_ctrl_cmd.chassis_mode = cancomm_pack->chassis_ctrl_can_comm.chassis_mode;
    //robot->chassis->chassis_ctrl_cmd.chassis_speed_buff=cchassis_ctrl_can_comm->chassis_speed_buff;
    robot->chassis->chassis_ctrl_cmd.offset_angle = cancomm_pack->chassis_ctrl_can_comm.offset_angle;
    robot->chassis->chassis_ctrl_cmd.SuperCapBoost = cancomm_pack->chassis_ctrl_can_comm.SuperCapBoost;
    // R键上升沿检测 (SuperCapBoost bit1: 0→1 触发UI重初始化)
    static uint8_t prev_r_bit = 0;
    uint8_t cur_r_bit = (robot->chassis->chassis_ctrl_cmd.SuperCapBoost >> 1) & 1;
    if (cur_r_bit && !prev_r_bit) {
        interactive_data->ui_init = 1;
    }
    prev_r_bit = cur_r_bit;
    // robot->gimbal->gimbal_ctrl_cmd.gimbal_mode = cancomm_pack->gimbal_mode;
    // robot->shoot->shoot_ctrl_cmd.shoot_mode = cancomm_pack->shoot_mode;
    // robot->shoot->shoot_ctrl_cmd.friction_mode = cancomm_pack->friction_mode;
    // robot->shoot->shoot_ctrl_cmd.load_mode = cancomm_pack->load_mode;
    // robot->gimbal->gimbal_ctrl_cmd.pitch = -(float)cancomm_pack->pitch;
    // robot->shoot->shoot_ctrl_cmd.rest_heat=cancomm_pack->rest_heat;
    // robot->shoot->shoot_ctrl_cmd.shoot_rate = cancomm_pack->shoot_rate;
    // robot->shoot->friction_motor[0]->measure.speed_aps = (float)cancomm_pack->friction_speed1;
    // robot->shoot->friction_motor[1]->measure.speed_aps = (float)cancomm_pack->friction_speed2;
   //
       interactive_data->chassis_mode = robot->chassis->chassis_ctrl_cmd.chassis_mode;
   //
     //  interactive_data->gimbal_mode = robot->gimbal->gimbal_ctrl_cmd.gimbal_mode;
      interactive_data->gimbal_mode = cancomm_pack->gimbal_mode;
   //
   //interactive_data-shoot_mode = robot->shoot->shoot_ctrl_cmd.shoot_mode;
     interactive_data->shoot_mode = cancomm_pack->shoot_mode;
   //
   //  //interactive_data.friction_mode = robotdata->shoot->shoot_ctrl_cmd.friction_mode;
      interactive_data->friction_mode = cancomm_pack->friction_mode;
   //
   //  //interactive_data.lid_mode = robotdata->shoot->shoot_ctrl_cmd.load_mode;
      interactive_data->load_mode = cancomm_pack->load_mode;
   //
    interactive_data->Chassis_Power_Data.chassis_power_mx = robot->chassis->chassis_ctrl_cmd.max_power; // 示例功率值
   //
   //  //interactive_data.pitch_angle = robotdata->gimbal->gimbal_ctrl_cmd.pitch;
      interactive_data->pitch_angle = (float)cancomm_pack->pitch;
    //
    // // interactive_data.Shoot_heat = robotdata->shoot->shoot_ctrl_cmd.rest_heat;
      interactive_data->Shoot_heat = cancomm_pack->rest_heat;
    // // interactive_data.Shoot_rate = robotdata->shoot->shoot_ctrl_cmd.shoot_rate;
      interactive_data->Shoot_rate = cancomm_pack->shoot_rate;
    //
    // // interactive_data.autoaim_mode = robotdata->gimbal->vision_mode;          // 自瞄模式
    // //interactive_data.autoaim_mode = robotdata->gimbal->gimbal_ctrl_cmd.gimbal_mode == GIMBAL_VISION ? 1 : 0;  // 自瞄模式(1为开启，0为关闭)
      interactive_data->autoaim_mode = cancomm_pack->gimbal_mode == GIMBAL_VISION ? 1 : 0;  // 自瞄模式(1为开启，0为关闭)

    // interactive_data.cap_voltage = robotdata->super_cap->cap_msg.vol / 1000.0f;
    // 检查使用的是哪种超级电容模块
    // #ifdef QQ_SUPER_CAP
    //   interactive_data.cap_voltage = robotdata->super_cap->cap_msg.vol;  // 齐奇模块，已是伏特单位
    // #else
    //interactive_data.cap_voltage = robotdata->super_cap->cap_msg.cap_v;  // 标准模块，需要转换为伏特
    // #endif

    // interactive_data.cap_mode = robotdata->super_cap->cap_msg.status;

    //interactive_data.bullet_left_real = referee_recv_info->ProjectileAllowance.projectile_allowance_17mm; // 实体弹丸剩余
    interactive_data->cap_voltage=robot->chassis->super_cap->cap_msg.cap_v;
      if (robot->chassis->super_cap->cap_msg.error_detect==0)
      {
          if (robot->chassis->chassis_ctrl_cmd.SuperCapBoost & 1)
              interactive_data->cap_mode=2;
          else
              interactive_data->cap_mode=1;
      }
      else interactive_data->cap_mode=0;
    //interactive_data.fric_speed_left = (uint16_t)robotdata->shoot->friction_motor[0]->measure.speed_aps; // 左摩擦轮转速（取反使向上为正）
     interactive_data->fric_speed_left = cancomm_pack->friction_speed1; // 左摩擦轮转速（取反使向上为正）
    //interactive_data.fric_speed_right = (uint16_t)robotdata->shoot->friction_motor[1]->measure.speed_aps; // 右摩擦轮转速
     interactive_data->fric_speed_right = cancomm_pack->friction_speed2; // 右摩擦轮转速

    interactive_data->chassis_relative_angle = robot->chassis->chassis_ctrl_cmd.offset_angle; // 底盘相对于云台的角度


    // robot->chassis->chassis_ctrl_cmd.max_power=chassis_ctrl_cmd->max_power;
    // 如果收到数据，可以在这里处理
    // if (received_data != NULL) {
    //   // 解析接收到的数据到全局变量
    //   //memcpy(board_can_comm_data.rx_buff, received_data, 16);
    //
    //   for (int i = 0; i < 24; i++)
    //     CanData.bytes[i] = received_data[i];
    //   chassis_ctrl_cmd->vx=60.0f*CanData.value16[0];//todo:后面chassis改改把负号去掉
    //   chassis_ctrl_cmd->vy=60.0f*CanData.value16[1];
    //   //if (CanData.value16[2]>=0)
    //   // chassis_ctrl_cmd->wz=(45.0f-(45.0f-20.0f)*expf((float)-CanData.value16[2]/50.0f))*CanData.value16[2];
    //   // else chassis_ctrl_cmd->wz=(45.0f-(45.0f-20.0f)*expf((float)CanData.value16[2]/50.0f))*CanData.value16[2];
    //   //chassis_ctrl_cmd->wz=0;
    //   chassis_ctrl_cmd->wz=35.0f*CanData.value16[2];//前馈
    //   // temp+=secondOrderDiffFF(CanData.value16[2]);
    //   // chassis_ctrl_cmd->wz+=temp;
    //   if (switch_is_mid(CanData.bytes[10])) {
    //     //gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    //     chassis_ctrl_cmd->max_power=50;
    //     if (abs((CanData.value16[3])) > 20) {
    //       chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    //       chassis_ctrl_cmd->wz = 40.0f*abs(CanData.value16[3]);
    //     } else
    //       chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
    //   }
    //   else if (switch_is_up(CanData.bytes[10])) {
    //     chassis_ctrl_cmd->max_power=350;
    //     chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
    //   }
    // }
  }
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
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler() {
  //底盘侧双板通信离线
  if (!CANCommIsOnline(can_comm_instance)) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    LOGERROR("[CMD] Emergency Stop! DualBoardComm Lost");
  }
  else LOGINFO("[CMD]DualBoardComm is Online");
}

RobotInstance * RobotInit() {
  if (robot!=NULL)
    return robot;
  //要在云台和底盘任务开始之前完成该任务的初始化
  // vTaskDelay(CAN_COMM_TASK_INIT_TIME);
  // 初始化CAN接收
  can_comm_instance = CANCommInit(&comm_config);
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));
  //supercap_mode=SAFETY_MODE;
#ifdef STM32F407xx
 // robot->rc_data = RemoteControlInit(&huart3);  // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#elifdef STM32H723XX
  robot->rc_data = RemoteControlInit(&huart5);  // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#endif

  //rc_data_last = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
 // *rc_data_last = *robot->rc_data;  // 记录上一次遥控器的状态

  robot->referee_data = RefereeInit(&huart6);  // 裁判系统初始化

  robot->chassis = ChassisInit(&chassis_init_config);
  //robot->shoot=(ShootInstance*)zmalloc(sizeof(ShootInstance));
   interactive_data=getUI();
  // 初始化控制命令指针
  chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  chassis_ctrl_cmd->max_power = 80;  // 随便给一个初始功率，后面应该要从裁判系统获取
  gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
    //srand(1);
  //rc_data = robot->rc_data;
 // MyUIInit();
  return robot;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
  // 根据gimbal的反馈值计算云台和底盘正方向的夹角             ,不需要传参,通过static私有变量完成
  //CalcOffsetAngle();
  //RemoteControlSet();
  DualBoardCtrlSet();
  // MouseKeySet();
  EmergencyHandler();  // 处理模块离线和遥控器急停等紧急情况
  upload.bullet_speed=robot->referee_data->ShootData.initial_speed;
  upload.HP=robot->referee_data->GameRobotState.current_HP;
    upload.Remain_Heat=U16TOI16(robot->referee_data->GameRobotState.shooter_barrel_heat_limit-robot->referee_data->PowerHeatData.shooter_17mm_barrel_heat);
  //upload.Heat=robot->referee_data->PowerHeatData.shooter_17mm_barrel_heat;
    //upload.heat_limt=robot->referee_data->GameRobotState.shooter_barrel_heat_limit;
    //upload.power=robot->chassis->super_cap->cap_msg.out_p;
    upload.cap_v=robot->chassis->super_cap->cap_msg.cap_v;
    //upload.error_code=robot->chassis->super_cap->cap_msg.error_detect;
    upload.color=robot->referee_data->GameRobotState.robot_id;
  CANCommSend(can_comm_instance,(uint8_t *)&upload);
}

void RobotTask() {
  if (DEVICE_ROLE_TX) {
    // 云台发送板控制任务
    // 测试数据,实际应用中这些数据应该来自其他模块
    board_can_comm_data.tx_buff[0] = 1;  // ui_flag
    board_can_comm_data.tx_buff[1] = 0;  // fric_flag
    board_can_comm_data.tx_buff[2] = 100; // chassis_vx
    board_can_comm_data.tx_buff[3] = 50;  // chassis_vy
    board_can_comm_data.tx_buff[4] = (3000 >> 8) & 0xFF;  // pitch_abs 高字节
    board_can_comm_data.tx_buff[5] = 3000 & 0xFF;         // pitch_abs 低字节
    board_can_comm_data.tx_buff[6] = 1;  // chassis_behaviour
    board_can_comm_data.tx_buff[7] = 0;  // cap_flag

    CANCommSend(can_comm_instance, board_can_comm_data.tx_buff);
    //can数据数据发送
    vTaskDelay(CAN_COMM_TASK_TIME);
  }
  else {

    //底盘接收板控制任务
    RobotCMDTask();

    //超级电容自动控制
    // switch (supercap_mode) {
    //   case SAFETY_MODE:
    //     if (temp>18.0f)
    //       supercap_mode=PASSIVE_MODE;
    //     robot->chassis->chassis_ctrl_cmd.max_power=30;
    //     break;
    //   case FORCED_CHARGING_MODE:
    //     if (temp<8.0f)
    //       supercap_mode=SAFETY_MODE;
    //     if (temp>18.0f)
    //       supercap_mode=PASSIVE_MODE;
    //     robot->chassis->chassis_ctrl_cmd.max_power=(uint16_t)(0.4*robot->referee_data->GameRobotState.chassis_power_limit);
    //     break;
    //   case CHARGING_MODE:
    //     if (temp<10.0f)
    //       supercap_mode=FORCED_CHARGING_MODE;
    //     if (temp>18.0f)
    //       supercap_mode=PASSIVE_MODE;
    //     robot->chassis->chassis_ctrl_cmd.max_power=robot->referee_data->GameRobotState.chassis_power_limit-(uint16_t)powf((float)robot->referee_data->GameRobotState.chassis_power_limit*0.04f,2);
    //     break;
    //   case PASSIVE_MODE:
    //     if (chassis_ctrl_cmd->max_power==180)
    //       supercap_mode=ACTIVE_MODE;
    //     if (temp<12.0f)
    //       supercap_mode=CHARGING_MODE;
    //     robot->chassis->chassis_ctrl_cmd.max_power=robot->referee_data->GameRobotState.chassis_power_limit;
    //     break;
    //   case ACTIVE_MODE:
    //     if (temp<12.0f)
    //       supercap_mode=CHARGING_MODE;
    //     if (chassis_ctrl_cmd->max_power!=180)
    //       supercap_mode=PASSIVE_MODE;
    //     robot->chassis->chassis_ctrl_cmd.max_power=130;
    //     break;
    //   default:
    //     supercap_mode=SAFETY_MODE;
    // }


     // robot->chassis->super_cap->cap_msg.cap_v=chaos_func(DWT_GetTimeline_s()/10.0f);
    //GimbalTask();
    //ShootTask();
    ChassisTask();

    // SuperCapSendMessage(robot->super_cap,
    //   (int16_t)robot->referee_data->GameRobotState.chassis_power_limit,
    //   robot->referee_data->PowerHeatData.buffer_energy,
    //   robot->referee_data->GameRobotState.power_management_chassis_output);
    //将原本motortask的can发送改到这里，和pid计算同频，减少无用发送
    //DJIMotorCANTransmit();
  }


  // 正确的赋值方式 - 直接赋值指针值
  // robot->shoot->friction_motor[1];
}
RobotInstance* RobotGet() {
  if (robot!=NULL)
    return robot;
  return NULL;
}