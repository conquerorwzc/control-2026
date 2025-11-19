#include "robot.h"

#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "string.h"
static RobotInstance *robot;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
static Gantry_Ctrl_Cmd_s *gantry_ctrl_cmd; // 【新增】龙门架控制命令指针

static RC_ctrl_t *rc_data;
static RC_ctrl_t *rc_data_last;  // 遥控器数据,初始化时返回

/* Intermediate variables calculated by private functions */
static float angle;

extern Gantry_Init_Config_s gantry_init_config;

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet() {
  // 右[中]
  if (switch_is_mid(rc_data[TEMP].rc.switch_right)) {
    if (abs(rc_data[TEMP].rc.dial) > 20) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    } else
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
  }
  // 右[上]，超电，保持底盘跟随云台
  else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    if (abs(rc_data[TEMP].rc.dial) > 20) {
      chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    } else{
      chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
    }
  }
  // 右[下] 在 EmergencyHandler 中处理断电

  // 龙门架模式设置 (利用左侧开关)
  if (gantry_ctrl_cmd != NULL) {
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
      // 左[中]：遥控器控制龙门架
      gantry_ctrl_cmd->Gantry_mode = GANTRY_MODE_CONTROL_REMOTE;
    } else if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
      // 左[上]：龙门架锁死 (方便操作底盘时保持机械臂位置)
      gantry_ctrl_cmd->Gantry_mode = GANTRY_MODE_LOCK;
    }
    // 左[下] 在 EmergencyHandler 中处理断电
  }

  // 底盘参数,系数需要调整
  chassis_ctrl_cmd->vx = 60.0f * (float)rc_data[TEMP].rc.rocker_l1;  // 水平方向
  chassis_ctrl_cmd->vy = 60.0f * (float)rc_data[TEMP].rc.rocker_l_;  // 竖直方向
  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_ROTATE) {
    chassis_ctrl_cmd->wz =
        -25.0f * (float)rc_data[TEMP].rc.dial;  // 小陀螺模式下的旋转分量，如果是跟随，则在底盘任务中计算旋转分量
  }
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
    robot->robot_mode = ROBOT_POWER_OFF;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    if (gantry_ctrl_cmd != NULL) {
      gantry_ctrl_cmd->Gantry_mode = GANTRY_MODE_POWER_OFF; // 【新增】龙门架断电
    }
    LOGERROR("[CMD] emergency stop!");
  } else {
    // 恢复正常运行，如果龙门架模式是断电，则恢复到遥控模式
    if (gantry_ctrl_cmd != NULL && gantry_ctrl_cmd->Gantry_mode == GANTRY_MODE_POWER_OFF) {
      gantry_ctrl_cmd->Gantry_mode = GANTRY_MODE_CONTROL_REMOTE;
    }
    LOGINFO("[CMD] reinstate, robot ready");
  }
  if (switch_is_down(rc_data[TEMP].rc.switch_right))  // 右下，底盘失能
  {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
  }
  if (switch_is_down(rc_data[TEMP].rc.switch_left))  // 左下，龙门架锁死
  {
    if (gantry_ctrl_cmd != NULL) {
      gantry_ctrl_cmd->Gantry_mode = GANTRY_MODE_LOCK;
    }
  }
  // 遥控器右侧开关为[上],恢复正常运行
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
  robot->gantry = GantryInit(&gantry_init_config);

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  robot->chassis = ChassisInit(&chassis_init_config);
#endif

  // 初始化控制命令指针
  chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  chassis_ctrl_cmd->max_power = 80;  // 随便给一个初始功率，后面应该要从裁判系统获取

  // 【新增】龙门架控制命令指针
  if (robot->gantry != NULL) {
    gantry_ctrl_cmd = &robot->gantry->Gantry_ctrl_cmd;
  }
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
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  ChassisTask();
#endif
  // 正确的赋值方式 - 直接赋值指针值
  // robot->shoot->friction_motor[1];
}
