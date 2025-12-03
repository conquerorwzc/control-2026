#include "robot.h"

#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"

static RobotInstance *robot;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
static Grab_Ctrl_Cmd_s *grab_ctrl_cmd;

static RC_ctrl_t *rc_data;
static RC_ctrl_t *rc_data_last;  // 遥控器数据,初始化时返回
static float temp;
/* Intermediate variables calculated by private functions */
// static float trigger_time = 0;  // 触发时间
// static float angle;
int b=0;
// static  DJIMotorInstance* debug_motor;

/* Private function declarations */
static void RemoteControlSet(void);
static void EmergencyHandler(void);

/* function declarations */
void RobotInit();
void RobotCMDTask();
void RobotTask() ;


void RobotInit() {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

#ifdef STM32F4
  robot->rc_data = RemoteControlInit(&huart3);  // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#elifdef STM32H7
  robot->rc_data = RemoteControlInit(&huart5);  // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#endif

  rc_data_last = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
  *rc_data_last = *robot->rc_data;  // 记录上一次遥控器的状态

  // robot->referee_data = RefereeInit(&huart6);  // 裁判系统初始化

  // robot->super_cap = SuperCapInit(&super_cap_config);

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
  // robot->gimbal = GimbalInit(&gimbal_init_config);
  // robot->shoot = ShootInit(&shoot_init_config);
  robot->grab = GrabInit(&grab_init_config_s);
#endif
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  robot->chassis = ChassisInit(&chassis_init_config);
#endif

  // 初始化控制命令指针
  chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  chassis_ctrl_cmd->max_power = 80;  // 随便给一个初始功率，后面应该要从裁判系统获取
  // gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  // shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  grab_ctrl_cmd = &robot->grab->grab_ctrl_cmd;
  rc_data = robot->rc_data;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
  // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
  RemoteControlSet();
  // MouseKeySet();
  EmergencyHandler();  // 处理模块离线和遥控器急停等紧急情况
}

void RobotTask() {
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
  RobotCMDTask();
  GrabTask();
  grab_ctrl_cmd->grab_mode=b;
  // GimbalTask();
  // ShootTask();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  // ChassisTask();
#endif

  // 正确的赋值方式 - 直接赋值指针值
  // robot->shoot->friction_motor[1];
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet() {
  // 右[中]，云台
  if (switch_is_mid(rc_data[TEMP].rc.switch_right))
  {
    // temp=(float)0.01*rc_data[TEMP].rc.rocker_r1;
    // grab_ctrl_cmd->r1=LIMIT_MIN_MAX(temp,-12.5,12.5);
    // temp=(float)0.01*rc_data[TEMP].rc.rocker_r_;
    // grab_ctrl_cmd->r2=LIMIT_MIN_MAX(temp,-12.5,12.5);
    temp=(float)0.01*rc_data[TEMP].rc.dial;
    grab_ctrl_cmd->r3=LIMIT_MIN_MAX(temp,-12.5,12.5);
  }

  // // 右[上]，超电，保持底盘跟随云台
  // else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
  //   gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
  //   if (abs(rc_data[TEMP].rc.dial) > 20) {
  //     chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
  //   } else
  //     chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
  // }
  // // 左[中],云台启动，摩擦轮启动，拨弹盘启动，准备射击
  // if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
  //   shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
  //   gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
  //   shoot_ctrl_cmd->friction_mode = FRICTION_ON;
  //   shoot_ctrl_cmd->load_mode = LOAD_STOP;
  //   // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
  //   // ...
  // } else if (switch_is_up(rc_data[TEMP].rc.switch_left))  // 开火，发射，根据时间判断单发或者连发
  // {
  //   shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
  //   gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
  //   shoot_ctrl_cmd->friction_mode = FRICTION_ON;
  //   shoot_ctrl_cmd->load_mode = LOAD_STOP;
  //   if (switch_is_mid(rc_data_last[TEMP].rc.switch_left)) {
  //     trigger_time = DWT_GetTimeline_s();
  //   }
  //   if (DWT_GetTimeline_s() - trigger_time > 1.0f) {
  //     shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
  //   } else {
  //     shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
  //   }
  // }
  // // 云台使能,或视觉未识别到目标,纯遥控器拨杆控制
  // if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON) {  // 按照摇杆的输出大小进行角度增量,增益系数需调整
  //   gimbal_ctrl_cmd->yaw -= -0.003f * (float)rc_data[TEMP].rc.rocker_r_;
  //   gimbal_ctrl_cmd->pitch += 0.0006f * (float)rc_data[TEMP].rc.rocker_r1;
  // }
  //
  // // 云台PITCH轴软件限位 todo:没在云台有点不好
  // if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
  //   gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  // } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
  //   gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  // }

  // 底盘参数,系数需要调整
  chassis_ctrl_cmd->vx = 60.0f * (float)rc_data[TEMP].rc.rocker_l1;  // 水平方向（工程的xy与其他的不同）
  chassis_ctrl_cmd->vy = 60.0f * (float)rc_data[TEMP].rc.rocker_l_;  // 竖直方向
  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_ROTATE) {
    chassis_ctrl_cmd->wz =
        -25.0f * (float)rc_data[TEMP].rc.dial;  // 小陀螺模式下的旋转分量，如果是跟随，则在底盘任务中计算旋转分量
  }
  // if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW) {
  //   chassis_ctrl_cmd->wz =(25.0f) *(float)rc_data[TEMP].rc.rocker_r_;  //
  //   主动跟随量，todo：但是感觉一个变量拆成两段写好像有点抽象，这里有一段，chassis还有另一段
  // }
  // 发射参数

  // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
  // shoot_ctrl_cmd->shoot_rate = 8;

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
    robot->robot_mode = ROBOT_POWER_ON;
    // gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    // shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    // shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    // shoot_ctrl_cmd->load_mode = LOAD_STOP;
    LOGERROR("[CMD] emergency stop!");
  } else {
    LOGINFO("[CMD] reinstate, robot ready");
  }
  if (switch_is_down(rc_data[TEMP].rc.switch_right))  // 底盘失能
  {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
  }
  // if (switch_is_down(rc_data[TEMP].rc.switch_left))  // 发射失能
  // {
  //   shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
  //   shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
  //   shoot_ctrl_cmd->load_mode = LOAD_STOP;
  // }
  // 遥控器右侧开关为[上],恢复正常运行
}