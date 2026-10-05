//
// Created by PC on 2025/11/18.
// 飞镖机器人: 纯遥控器遥操作
//   右拨杆(switch_right): 下档 = 4 个电机 + 舵机全部失能
//                         中档 = 左摇杆竖直控同步带 + 右摇杆水平控 yaw
//                         上档 = 左摇杆水平控 yaw + 右摇杆竖直控扳机位置电机(同步带锁定在上次位置)
//   左拨杆(switch_left):  下档 = 舵机失能; 中档 = 转到角度1; 上档 = 转到角度2
//
#include "robot.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "remote_control.h"
#include "robot_config.h"
#include "user_lib.h"

/* Private variables ---------------------------------------------------------*/
RobotInstance* robot = NULL;
static DartShootInstance* dart = NULL;
static float status_log_time = 0.0f;  // 状态日志时间戳

/*
 * 本文件不写函数前置声明: 私有函数按"被调用者在前"的顺序定义, 对外接口声明在 robot.h 中。
 */

/*=======私有函数: 依赖在前, 不需要前置声明=======*/
/**
 * @brief 摇杆方向判定: 摇杆只看正负, 与具体数值无关
 */
static int8_t StickToDirection(int16_t stick_value, int16_t threshold) {
  if (stick_value > threshold) {
    return 1;
  }
  if (stick_value < -threshold) {
    return -1;
  }
  return 0;
}

/**
 * @brief 遥控器 -> 档位与各执行器指令
 *
 * @note  档位按电平判断, 不用边沿: 拨杆停在哪个位置就是哪个状态;
 *        遥控器离线时一律失能。
 */
static void DartUpdateCommand(void) {
  robot->rc_online = RemoteControlIsOnline();

  // ---- 右拨杆: 选档位 ----
  if (robot->rc_data == NULL || !robot->rc_online) {
    robot->dart_mode = DART_SHOOT_MODE_DISABLED;  // 遥控器离线或尚未初始化: 全部失能
  } else if (switch_is_mid(robot->rc_data[TEMP].rc.switch_right)) {
    robot->dart_mode = DART_SHOOT_MODE_AIM;
  } else if (switch_is_up(robot->rc_data[TEMP].rc.switch_right)) {
    robot->dart_mode = DART_SHOOT_MODE_TRIGGER;
  } else {
    robot->dart_mode = DART_SHOOT_MODE_DISABLED;  // 下档与异常值都当作失能档
  }
  DartShootSetMode(dart, robot->dart_mode);

  // ---- 摇杆: 各执行器指令 ----
  switch (robot->dart_mode) {
    case DART_SHOOT_MODE_AIM:
      // 左摇杆竖直: 同步带累加方向; 右摇杆水平: yaw 累加方向(只看方向, 与摇杆大小无关)
      DartShootSetBeltDir(dart, StickToDirection(robot->rc_data[TEMP].rc.rocker_l1, DART_RC_DIR_THRESHOLD));
      DartShootSetYawDir(dart, StickToDirection(robot->rc_data[TEMP].rc.rocker_r_, DART_RC_DIR_THRESHOLD));
      DartShootSetTriggerDir(dart, 0);
      break;

    case DART_SHOOT_MODE_TRIGGER:
      // 上档: 左摇杆水平 -> yaw 累加方向, 右摇杆竖直 -> 扳机位置电机
      // 同步带 belt_dir = 0: 组件保持位置环给定, 锁死在中档拉到的位置上
      DartShootSetBeltDir(dart, 0);
      DartShootSetYawDir(dart, StickToDirection(robot->rc_data[TEMP].rc.rocker_l_, DART_RC_DIR_THRESHOLD));
      DartShootSetTriggerDir(dart, StickToDirection(robot->rc_data[TEMP].rc.rocker_r1, DART_RC_DIR_THRESHOLD));
      break;

    case DART_SHOOT_MODE_DISABLED:
    default:
      DartShootSetBeltDir(dart, 0);
      DartShootSetYawDir(dart, 0);
      DartShootSetTriggerDir(dart, 0);
      break;
  }

  // ---- 左拨杆: 舵机(下档失能 / 中档角度1 / 上档角度2) ----
  // 右拨杆下档时组件内部会强制舵机失能, 这里的指令优先级更低
  if (robot->rc_online && switch_is_mid(robot->rc_data[TEMP].rc.switch_left)) {
    DartShootSetServo(dart, DART_SERVO_CMD_ANGLE_MID);
  } else if (robot->rc_online && switch_is_up(robot->rc_data[TEMP].rc.switch_left)) {
    DartShootSetServo(dart, DART_SERVO_CMD_ANGLE_UP);
  } else {
    DartShootSetServo(dart, DART_SERVO_CMD_DISABLED);
  }
}

/**
 * @brief 周期性状态日志: 输入(拨杆/摇杆/档位)与输出(目标位置/下发给电机的 pid_ref/使能状态)
 *
 * @note  pid_ref 就是位置环的给定值: 摇杆推动时它不变 -> 断在组件之前(档位或摇杆方向);
 *        它变了但电机不动 -> 断在组件之后(使能状态或 CAN)。
 */
static void DartStatusLog(void) {
  float now = DWT_GetTimeline_ms();
  if (now - status_log_time < DART_STATUS_LOG_PERIOD_MS) {
    return;
  }
  status_log_time = now;

  char belt_pos[16], belt_tgt[16], belt_pid[16], yaw_pos[16], yaw_tgt[16], yaw_pid[16], servo_ang[16];
  Float2Str(belt_pos, dart->feed.belt_position);
  Float2Str(belt_tgt, dart->feed.belt_target);
  Float2Str(belt_pid, dart->belt_motor[0]->motor_controller.pid_ref);
  Float2Str(yaw_pos, dart->feed.yaw_position);
  Float2Str(yaw_tgt, dart->feed.yaw_target);
  Float2Str(yaw_pid, dart->yaw_motor->motor_controller.pid_ref);
  Float2Str(servo_ang, dart->feed.servo_angle);

  if (robot->rc_data == NULL) {
    LOGERROR("[dart] rc_data is NULL: RemoteControlInit() did not run");
  } else {
    LOGINFO("[dart] in : sw %d/%d stick %d %d %d %d | rc %d mode %d", robot->rc_data[TEMP].rc.switch_left,
            robot->rc_data[TEMP].rc.switch_right, robot->rc_data[TEMP].rc.rocker_l1, robot->rc_data[TEMP].rc.rocker_l_,
            robot->rc_data[TEMP].rc.rocker_r1, robot->rc_data[TEMP].rc.rocker_r_, robot->rc_online, robot->dart_mode);
  }

  LOGINFO("[dart] out: belt pos/tgt/pid %s/%s/%s en %d | yaw pos/tgt/pid %s/%s/%s en %d | trig dir %d en %d | "
          "servo %s en %d | motors %d",
          belt_pos, belt_tgt, belt_pid, dart->feed.belt_enabled, yaw_pos, yaw_tgt, yaw_pid, dart->feed.yaw_enabled,
          dart->ctrl_cmd.trigger_dir, dart->feed.trigger_enabled, servo_ang, dart->feed.servo_enabled,
          dart->feed.online);
}

/*=======对外接口: 原型见 robot.h=======*/
/**
 * @brief 机器人初始化
 */
void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));

  // 遥控器: 老遥控器(DT7/DBUS)在C板上使用USART3, 自研板需选用带反相器的串口
#ifdef STM32F407xx
  robot->rc_data = RemoteControlInit(&huart3);
#elifdef STM32H723xx
  robot->rc_data = RemoteControlInit(&huart5);
#endif

  // 发射机构组件(同步带 3508 x2 + yaw 2006 + 扳机位置 3508 + 拉扳机舵机)
  dart = DartShootInit(&dart_shoot_init_config);
  robot->dart_shoot = dart;

  // 上电默认全部失能, 等遥控器给出指令
  robot->dart_mode = DART_SHOOT_MODE_DISABLED;
  robot->rc_online = 0;
  DartShootSetMode(dart, DART_SHOOT_MODE_DISABLED);
  DartShootSetBeltDir(dart, 0);
  DartShootSetYawDir(dart, 0);
  DartShootSetTriggerDir(dart, 0);
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

  // 3. 周期性状态日志, 便于现场确认方向/位置/使能状态
  DartStatusLog();
}
