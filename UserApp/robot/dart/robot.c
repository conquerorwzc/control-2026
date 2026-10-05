//
// Created by PC on 2025/11/18.
// 飞镖机器人: 纯遥控器遥操作
//   右拨杆(switch_right): 下档 = 4 个电机 + 舵机全部失能; 中档 = 同步带 + yaw; 上档 = 扳机位置调整
//   左拨杆(switch_left):  下档 = 舵机失能; 中档 = 转到角度1; 上档 = 转到角度2
//
#include "robot.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "robot_config.h"
#include "user_lib.h"

/* Private variables ---------------------------------------------------------*/
RobotInstance* robot = NULL;
static DartShootInstance* dart = NULL;
static RC_ctrl_t* rc_data = NULL;
static float status_log_time = 0.0f;  // 状态日志时间戳

/* Private function prototypes -----------------------------------------------*/
static void DartUpdateCommand(void);
static void DartStatusLog(void);

/* Private user code ---------------------------------------------------------*/
bool IsInDeadzone(int16_t value) {
  return (value > -RC_DEADZONE) && (value < RC_DEADZONE);
}  // 死区检测

float MapStickToSpeed(int16_t stick_value, float max_speed) {
  if (IsInDeadzone(stick_value)) {
    return 0.0f;
  }
  return float_constrain(((float)stick_value / DART_RC_STICK_MAX) * max_speed, -max_speed, max_speed);
}  // 映射摇杆值到速度参考(死区内返回 0)

int8_t StickToDirection(int16_t stick_value, int16_t threshold) {
  if (stick_value > threshold) {
    return 1;
  }
  if (stick_value < -threshold) {
    return -1;
  }
  return 0;
}  // 摇杆方向判定, 与摇杆具体数值无关

/**
 * @brief 机器人初始化
 */
void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));

  // 遥控器: 老遥控器(DT7)在C板上使用USART3, 自研板需选用带反相器的串口
#ifdef STM32F407xx
  rc_data = RemoteControlInit(&huart3);
#elifdef STM32H723xx
  rc_data = RemoteControlInit(&huart5);
#endif
  robot->rc_data = rc_data;

  // 发射机构组件(同步带 3508 x2 + yaw 2006 + 扳机位置 3508 + 拉扳机舵机)
  dart = DartShootInit(&dart_shoot_init_config);
  robot->dart_shoot = dart;

  // 上电默认全部失能, 等遥控器给出指令
  robot->robot_mode = ROBOT_MODE_DISABLED;
  robot->rc_online = 0;
  DartShootSetMode(dart, DART_SHOOT_MODE_DISABLED);
  DartShootSetBeltDir(dart, 0);
  DartShootSetYawDir(dart, 0);
  DartShootSetTriggerSpeed(dart, 0.0f);
  DartShootSetServo(dart, DART_SERVO_CMD_DISABLED);
  LOGINFO("[dart] RobotInit done: right switch = mode, left switch = servo");
}

/**
 * @brief 机器人主任务
 */
void RobotTask(void) {
  // 1. 遥控器 -> 档位与各执行器指令
  DartUpdateCommand();

  // 2. 组件任务 (DJI电机的CAN报文由MOTOR任务统一发送, 此处不重复调用DJIMotorTask)
  DartShootTask(dart);

  // 3. 周期性状态日志, 便于现场确认方向/位置/在线状态
  DartStatusLog();
}

/**
 * @brief 遥控器 -> 档位与各执行器指令
 *
 * @note  全部按"档位"电平判断, 不用边沿, 因此拨杆停在哪个位置就是哪个状态;
 *        遥控器离线时一律失能。
 */
static void DartUpdateCommand(void) {
  if (dart == NULL) {
    return;
  }

  robot->rc_online = RemoteControlIsOnline();
  if (rc_data == NULL || !robot->rc_online) {
    // 遥控器离线或尚未初始化: 全部失能(唯一的安全兜底)
    robot->robot_mode = ROBOT_MODE_DISABLED;
  } else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) {
    robot->robot_mode = ROBOT_MODE_AIM;
  } else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    robot->robot_mode = ROBOT_MODE_TRIGGER_ADJ;
  } else {
    // 下档(RC_SW_DOWN)与异常值都当作失能档
    robot->robot_mode = ROBOT_MODE_DISABLED;
  }

  switch (robot->robot_mode) {
    case ROBOT_MODE_AIM:
      // 左摇杆竖直: 同步带位置累加方向; 右摇杆水平: yaw 位置累加方向(只看方向, 与摇杆大小无关)
      DartShootSetMode(dart, DART_SHOOT_MODE_AIM);
      DartShootSetBeltDir(dart, StickToDirection(rc_data[TEMP].rc.rocker_l1, DART_RC_DIR_THRESHOLD));
      DartShootSetYawDir(dart, StickToDirection(rc_data[TEMP].rc.rocker_r_, DART_RC_DIR_THRESHOLD));
      DartShootSetTriggerSpeed(dart, 0.0f);
      break;

    case ROBOT_MODE_TRIGGER_ADJ:
      // 只有扳机位置电机可用: 右摇杆竖直 -> 速度环参考, 摇杆回死区后组件直接让电机失能
      DartShootSetMode(dart, DART_SHOOT_MODE_TRIGGER);
      DartShootSetBeltDir(dart, 0);
      DartShootSetYawDir(dart, 0);
      DartShootSetTriggerSpeed(dart, MapStickToSpeed(rc_data[TEMP].rc.rocker_r1, dart->param.trigger_speed_limit));
      break;

    case ROBOT_MODE_DISABLED:
    default:
      DartShootSetMode(dart, DART_SHOOT_MODE_DISABLED);
      DartShootSetBeltDir(dart, 0);
      DartShootSetYawDir(dart, 0);
      DartShootSetTriggerSpeed(dart, 0.0f);
      break;
  }

  // 左拨杆: 舵机(下档失能 / 中档角度1 / 上档角度2)
  // 右拨杆下档时组件内部会强制舵机失能, 这里的指令优先级更低
  if (robot->rc_online && switch_is_mid(rc_data[TEMP].rc.switch_left)) {
    DartShootSetServo(dart, DART_SERVO_CMD_ANGLE_MID);
  } else if (robot->rc_online && switch_is_up(rc_data[TEMP].rc.switch_left)) {
    DartShootSetServo(dart, DART_SERVO_CMD_ANGLE_UP);
  } else {
    DartShootSetServo(dart, DART_SERVO_CMD_DISABLED);
  }
}

/**
 * @brief 周期性状态日志: 档位/在线状态/位置/使能状态
 */
static void DartStatusLog(void) {
  float now = DWT_GetTimeline_ms();
  if (dart == NULL || now - status_log_time < DART_STATUS_LOG_PERIOD_MS) {
    return;
  }
  status_log_time = now;

  char belt_pos[16], belt_tgt[16], yaw_pos[16], yaw_tgt[16], trig_spd[16], servo_ang[16];
  Float2Str(belt_pos, dart->feed.belt_position);
  Float2Str(belt_tgt, dart->feed.belt_target);
  Float2Str(yaw_pos, dart->feed.yaw_position);
  Float2Str(yaw_tgt, dart->feed.yaw_target);
  Float2Str(trig_spd, dart->feed.trigger_speed);
  Float2Str(servo_ang, dart->feed.servo_angle);

  LOGINFO(
      "[dart] mode %d rc %d online %d%d%d | belt %s/%s%s en %d | yaw %s/%s%s en %d | trigger %s en %d | servo %s en %d",
      robot->robot_mode, robot->rc_online, dart->feed.belt_online, dart->feed.yaw_online, dart->feed.trigger_online,
      belt_pos, belt_tgt, dart->feed.belt_limited ? "!" : "", dart->feed.belt_enabled, yaw_pos, yaw_tgt,
      dart->feed.yaw_limited ? "!" : "", dart->feed.yaw_enabled, trig_spd, dart->feed.trigger_enabled, servo_ang,
      dart->feed.servo_enabled);
}
