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
static uint8_t sw_left_last = RC_SW_OFF;  // 左开关上一帧(任务侧边沿检测)
/* @note 边沿必须在任务侧自己记: rc 模块的 TEMP/LAST 在串口回调里解析完立即互相拷贝,
 * 任务侧读到的 TEMP==LAST, 用它做开关边沿检测永远不触发(踩过: 一键储能无声无息) */

/* 摇杆/拨轮归一化 [-1, 1], 带死区 */
static float StickToNorm(int16_t stick) {
  if (fabsf((float)stick) <= (float)DART_RC_DEADZONE) return 0.0f;
  return (float)stick / (float)DART_STICK_FULL;
}

/* 使能/失能与急停: 右开关下档或遥控器离线即全部停机
 * 调试档(右开关上)使能时不自动校准: 上电即可在 IDLE 点动验方向,
 * 之后拨回中档并动一下左开关(下->中)即开始校准
 *
 * @note 时序竞争修复: 上电后"第一帧遥控数据到达之前"开关字节是 0(RC_SW_OFF), 而
 *       DaemonIsOnline() 此时已判在线 -> 旧逻辑会把它当成"使能+正常档", 于是自动校准在
 *       遥控器还没真正接上时就跑起来了(随后收到真帧发现是调试档又被中止)。
 *       现在要求开关值必须是有效档位(上/中/下 三者之一)才允许使能, 从根上避免这个竞争 */
static void SafetyUpdate(void) {
  bool online = RemoteControlIsOnline() == 1;
  uint8_t sw_right = rc_data[TEMP].rc.switch_right;
  bool sw_valid = (sw_right == RC_SW_UP) || (sw_right == RC_SW_MID) || (sw_right == RC_SW_DOWN);
  bool debug = switch_is_up(sw_right);
  bool enable = online && sw_valid && !switch_is_down(sw_right);
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

/* 遥控器输入解析与命令分发
 * 摇杆分组(防误动): 左摇杆 = 同步带 + yaw, 右摇杆 = 扳机(丝杆 + 舵机),
 * 拉同步带时手不会碰到扳机轴; 同一执行器在正常/调试档保持同一摇杆轴, 切换档位不换手 */
static void RCCommandUpdate(void) {
  RC_ctrl_t* rc = &rc_data[TEMP];

  // 左摇杆水平 -> yaw 角速度(两档一致)
  DartLauncherSetYawRate(launcher, StickToNorm(rc->rc.rocker_l_) * DART_YAW_SENSITIVITY_DPS);

  // 右摇杆竖直 -> 扳机丝杆(两档同轴): 正常档 = 射力微调; 调试档 = 点动
  float screw_stick = StickToNorm(rc->rc.rocker_r1);

  // 右开关上档 = 调试点动
  bool debug = switch_is_up(rc->rc.switch_right);
  if (debug) {
    // 右摇杆水平 = 舵机角度增量(松手即停, 防松手跳变误触发)
    float servo_stick = StickToNorm(rc->rc.rocker_r_);
    if (servo_stick != 0.0f) {
      DartLauncherAdjustServoAngle(launcher, servo_stick * DART_SERVO_ADJ_DPS * DART_TASK_DT_S);
    }
    // 左摇杆竖直 = 同步带点动(上推=储能方向); 右摇杆竖直 = 丝杆点动(上推=射力增大)
    DartLauncherSetDebugJog(launcher, true, StickToNorm(rc->rc.rocker_l1) * DART_DEBUG_BELT_MAX_SPEED_DPS,
                            screw_stick * DART_DEBUG_SCREW_MAX_SPEED_DPS);
  } else {
    DartLauncherSetDebugJog(launcher, false, 0.0f, 0.0f);
    // 右摇杆竖直 -> 射力(丝杆位置)微调, 仅 IDLE 生效(launcher 内部约束: READY 时扳机带载锁着平台)
    if (screw_stick != 0.0f) {
      DartLauncherAdjustScrewPos(launcher, screw_stick * DART_SCREW_ADJ_DPS * DART_TASK_DT_S);
    }
  }

  // 左开关命令(任务侧边沿检测):
  //   进中档(下->中/上->中) = 储能(故障/未校准时为重新校准);
  //   下->上快速扫过中档   = 同样按储能处理(防快速拨动漏触发);
  //   中->上               = 发射(两段式: 必须经过中档才允许发射, 防误发)
  uint8_t sw = rc->rc.switch_left;
  uint8_t sw_last = sw_left_last;
  sw_left_last = sw;
  if (sw != sw_last && !switch_is_off(sw) && !switch_is_off(sw_last)) {
    if (switch_is_mid(sw) || (switch_is_up(sw) && switch_is_down(sw_last))) {
      if (launcher->state == DART_STATE_FAULT || !launcher->is_calibrated) {
        DartLauncherSetCommand(launcher, DART_CMD_CALIBRATE);
      } else {
        DartLauncherSetCommand(launcher, DART_CMD_CHARGE);
      }
    } else if (switch_is_up(sw) && switch_is_mid(sw_last)) {
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

  DartBuzzerService();  // 校准 BGM 分段补队(组曲长, 须周期调用)
  SafetyUpdate();  // 失能/急停优先于一切控制
  if (launcher->enabled) RCCommandUpdate();
  DartLauncherTask(launcher);
}
