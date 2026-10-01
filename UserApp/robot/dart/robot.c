/**
 * @file robot.c
 * @brief dart 发射架机器人入口: DR16 遥控器解析、使能/急停安全逻辑与命令分发
 *
 * 操作映射见 robot_config.h 底部说明。
 */
#include "robot.h"

#include "bsp_log.h"
#include "main.h"
#include "robot_config.h"
#include "user_lib.h"

static RobotInstance* robot = NULL;
static DartLauncherInstance* launcher = NULL;
static RC_ctrl_t* rc_data = NULL;

/* 摇杆/拨轮归一化 [-1, 1], 带死区 */
static float StickToNorm(int16_t stick) {
  if (fabsf((float)stick) <= (float)DART_RC_DEADZONE) return 0.0f;
  return (float)stick / (float)DART_STICK_FULL;
}

/* 使能/失能与急停: 右开关下档或遥控器离线即全部停机 */
static void SafetyUpdate(void) {
  bool online = RemoteControlIsOnline() == 1;
  bool enable = online && !switch_is_down(rc_data[TEMP].rc.switch_right);
  DartLauncherSetEnable(launcher, enable);
  robot->robot_mode = enable ? ROBOT_POWER_ON : ROBOT_EMERGENCY_STOP;
}

/* 遥控器输入解析与命令分发 */
static void RCCommandUpdate(void) {
  RC_ctrl_t* rc = &rc_data[TEMP];

  // 右摇杆水平 -> yaw 角速度
  DartLauncherSetYawRate(launcher, StickToNorm(rc->rc.rocker_r_) * DART_YAW_SENSITIVITY_DPS);

  // 侧边拨轮 -> 射力(扳机卡位)微调, 仅 IDLE/READY 生效(launcher 内部约束)
  float dial = StickToNorm(rc->rc.dial);
  if (dial != 0.0f) {
    DartLauncherAdjustTrigger(launcher, dial * DART_TRIGGER_ADJ_DPS * DART_TASK_DT_S);
  }

  // 右开关上档 = 调试点动
  bool debug = switch_is_up(rc->rc.switch_right);
  if (debug) {
    DartLauncherSetDebugJog(launcher, true, StickToNorm(rc->rc.rocker_l1) * DART_DEBUG_BELT_MAX_SPEED_DPS,
                            StickToNorm(rc->rc.rocker_l_) * DART_DEBUG_TRIGGER_MAX_SPEED_DPS);
  } else {
    DartLauncherSetDebugJog(launcher, false, 0.0f, 0.0f);
  }

  // 左开关上升沿命令: 中档 = 储能(故障/未校准时为重新校准); 上档 = 发射
  uint8_t sw = rc->rc.switch_left;
  uint8_t sw_last = rc_data[LAST].rc.switch_left;
  if (sw != sw_last) {
    if (switch_is_mid(sw)) {
      if (launcher->state == DART_STATE_FAULT || !launcher->is_calibrated) {
        DartLauncherSetCommand(launcher, DART_CMD_CALIBRATE);
      } else {
        DartLauncherSetCommand(launcher, DART_CMD_CHARGE);
      }
    } else if (switch_is_up(sw)) {
      DartLauncherSetCommand(launcher, DART_CMD_FIRE);
    }
  }
}

void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));

  rc_data = RemoteControlInit(DART_RC_UART);
  robot->rc_data = rc_data;

  launcher = DartLauncherInit();
  robot->launcher = launcher;

  robot->robot_mode = ROBOT_POWER_ON;
  LOGINFO("[dart] robot init done");
}

void RobotTask(void) {
  if (robot == NULL || launcher == NULL || rc_data == NULL) return;

  SafetyUpdate();  // 失能/急停优先于一切控制
  if (launcher->enabled) RCCommandUpdate();
  DartLauncherTask(launcher);
}
