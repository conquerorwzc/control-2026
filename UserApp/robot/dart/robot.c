/**
 * @file robot.c
 * @brief dart 发射架机器人入口: DR16 遥控器解析、使能/急停安全逻辑与命令分发
 *
 * 操作映射见 robot_config.h 底部说明。
 */
#include "robot.h"

#include "bsp_log.h"
#include "dart_buzzer.h"
#include "main.h"
#include "robot_config.h"
#include "user_lib.h"

static RobotInstance* robot = NULL;
static DartLauncherInstance* launcher = NULL;
static RC_ctrl_t* rc_data = NULL;
static bool rc_was_online = true;   // 用于失联/恢复边沿提示
static bool was_debug_mode = false; // 用于进入调试档边沿提示

/* 摇杆/拨轮归一化 [-1, 1], 带死区 */
static float StickToNorm(int16_t stick) {
  if (fabsf((float)stick) <= (float)DART_RC_DEADZONE) return 0.0f;
  return (float)stick / (float)DART_STICK_FULL;
}

/* 使能/失能与急停: 右开关下档或遥控器离线即全部停机
 * 调试档(右开关上)使能时不自动校准: 上电即可在 IDLE 点动验方向,
 * 之后拨回中档并动一下左开关(下->中)即开始校准 */
static void SafetyUpdate(void) {
  bool online = RemoteControlIsOnline() == 1;
  bool debug = switch_is_up(rc_data[TEMP].rc.switch_right);
  bool enable = online && !switch_is_down(rc_data[TEMP].rc.switch_right);
  DartLauncherSetEnable(launcher, enable, !debug);
  robot->robot_mode = enable ? ROBOT_POWER_ON : ROBOT_EMERGENCY_STOP;

  // 失联/恢复边沿提示(持续指示由 dart_buzzer 维护)
  if (online != rc_was_online) {
    if (online) {
      DartBuzzerRcOk();
    } else {
      DartBuzzerRcLost();
    }
    rc_was_online = online;
  }
  // 进入调试档边沿提示
  if (debug && !was_debug_mode) DartBuzzerDebugMode();
  was_debug_mode = debug;
}

/* 遥控器输入解析与命令分发 */
static void RCCommandUpdate(void) {
  RC_ctrl_t* rc = &rc_data[TEMP];

  // 右摇杆水平 -> yaw 角速度
  DartLauncherSetYawRate(launcher, StickToNorm(rc->rc.rocker_r_) * DART_YAW_SENSITIVITY_DPS);

  // 右摇杆竖直 -> 正常档: 射力(丝杆位置)微调; 调试档: 舵机角度
  // (侧边拨轮已弃用: 实车拨轮损坏)
  float power_stick = StickToNorm(rc->rc.rocker_r1);

  // 右开关上档 = 调试点动
  bool debug = switch_is_up(rc->rc.switch_right);
  if (debug) {
    // 调试档: 右摇杆竖直 = 舵机角度增量(松手即停, 防松手跳变误触发)
    if (power_stick != 0.0f) {
      DartLauncherAdjustServoAngle(launcher, power_stick * DART_SERVO_ADJ_DPS * DART_TASK_DT_S);
    }
    // 左摇杆竖直 = 同步带点动(上推=储能方向); 左摇杆水平 = 丝杆点动(右推=正方向)
    DartLauncherSetDebugJog(launcher, true, StickToNorm(rc->rc.rocker_l1) * DART_DEBUG_BELT_MAX_SPEED_DPS,
                            StickToNorm(rc->rc.rocker_l_) * DART_DEBUG_SCREW_MAX_SPEED_DPS);
  } else {
    DartLauncherSetDebugJog(launcher, false, 0.0f, 0.0f);
    // 右摇杆竖直 -> 射力(丝杆位置)微调, 仅 IDLE/READY 生效(launcher 内部约束)
    if (power_stick != 0.0f) {
      DartLauncherAdjustScrewPos(launcher, power_stick * DART_SCREW_ADJ_DPS * DART_TASK_DT_S);
    }
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
