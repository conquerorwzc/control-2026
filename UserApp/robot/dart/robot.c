//
// Created by PC on 2025/11/18.
//
#include "robot.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "general_def.h"
#include "robot_config.h"
#include "string.h"
#include "user_lib.h"

/* Private variables ---------------------------------------------------------*/
RobotInstance* robot = NULL;
static DartShootInstance* dart = NULL;
static RC_ctrl_t* rc_data = NULL;
static Robot_Mode_e last_robot_mode = ROBOT_POWER_ON;
static DartMode_e last_dart_mode = DART_MODE_STOPPED;

/* 手动测试 */
static float servo_angle_set = 0.0f;  // 手动模式下由拨轮/右摇杆决定的舵机角度
static float test_log_time = 0.0f;    // 手动模式状态日志时间戳

/* 摇杆边沿检测 */
static int16_t last_stick_l1 = 0;
static int16_t last_stick_r1 = 0;
static int16_t last_stick_r_ = 0;
static uint8_t edge_l1 = DART_TEST_EDGE_NONE;
static uint8_t edge_r1 = DART_TEST_EDGE_NONE;
static uint8_t edge_r_ = DART_TEST_EDGE_NONE;

/* 自动循环测试 */
static uint8_t auto_cycle_enabled = 0;
static DartTestStep_e auto_step = TEST_STEP_IDLE;
static uint32_t auto_attempt = 0;
static uint32_t auto_fail_streak = 0;
static uint8_t arrive_by_encoder = 0;
static float cycle_start_time = 0.0f;

/* Private function prototypes -----------------------------------------------*/
static uint8_t DartTestEdge(int16_t stick, int16_t threshold, int16_t* last);
static const char* DartStateName(DartShoot_State_e state);
static void DartAutoCycleStart(void);
static void DartAutoCycleStop(void);
static void DartAutoCycleHandler(void);
static void DartAutoCyclePass(float cycle_time_ms);
static void DartAutoCycleFail(DartShoot_Error_e error);
static void DartTestReport(void);

/* Private user code ---------------------------------------------------------*/
bool IsInDeadzone(int16_t value) {
  return (value > -RC_DEADZONE) && (value < RC_DEADZONE);
}  // 死区检测

float MapStickToSpeed(int16_t stick_value, float max_speed) {
  if (IsInDeadzone(stick_value)) {
    return 0.0f;
  }
  return float_constrain(((float)stick_value / 660.0f) * max_speed, -max_speed, max_speed);
}  // 映射摇杆值到电机速度

/**
 * @brief 飞镖发射机构初始化
 */
void DartInit(void) {
  dart = DartShootInit(&dart_shoot_init_config);
  servo_angle_set = dart->param.servo_lock_angle;
  auto_cycle_enabled = 0;
  auto_step = TEST_STEP_IDLE;
}

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

  // 发射机构组件
  DartInit();
  robot->dart_shoot = dart;

  robot->robot_mode = ROBOT_POWER_ON;
  robot->dart_mode = DART_MODE_STOPPED;
  last_robot_mode = robot->robot_mode;
  last_dart_mode = robot->dart_mode;
  LOGINFO("[dart] RobotInit done");
}

/**
 * @brief 机器人主任务
 */
void RobotTask(void) {
  // 1. 更新机器人状态与飞镖模式, 并检测摇杆边沿
  DartStateMachineUpdate();

  // 2. 状态/模式切换处理
  if (last_dart_mode != robot->dart_mode || last_robot_mode != robot->robot_mode) {
    last_dart_mode = robot->dart_mode;
    last_robot_mode = robot->robot_mode;
    if (robot->robot_mode == ROBOT_EMERGENCY_STOP) {
      DartEmergencyHandler();
    } else {
      DartModeChangeHandler();
    }
  }

  // 3. 急停: 组件停机(电机不输出, 舵机回锁止位), 不执行任何测试逻辑
  if (robot->robot_mode == ROBOT_EMERGENCY_STOP) {
    DartShootSetMode(dart, DART_SHOOT_MODE_STOPPED);
    DartShootTask(dart);
    return;
  }

  // 4. 根据飞镖模式执行测试逻辑
  switch (robot->dart_mode) {
    case DART_MODE_MANUAL:
      DartManualModeHandler();
      break;
    case DART_MODE_AUTO:
      DartAutoTestHandler();
      break;
    case DART_MODE_STOPPED:
    default:
      DartStoppedHandler();
      break;
  }

  // 5. 组件任务 (DJI电机的CAN报文由MOTOR任务统一发送, 此处不重复调用DJIMotorTask)
  DartShootTask(dart);
}

/**
 * @brief 机器人状态与飞镖模式更新
 */
void DartStateMachineUpdate(void) {
  if (rc_data == NULL) {
    return;
  }

  // 遥控器离线或左侧开关未解锁 -> 急停
  if (!RemoteControlIsOnline() || !switch_is_up(rc_data[TEMP].rc.switch_left)) {
    robot->robot_mode = ROBOT_EMERGENCY_STOP;
  } else {
    robot->robot_mode = ROBOT_POWER_ON;
  }

  // 右侧开关选择模式
  if (switch_is_mid(rc_data[TEMP].rc.switch_right)) {
    robot->dart_mode = DART_MODE_MANUAL;
  } else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    robot->dart_mode = DART_MODE_AUTO;
  } else {
    robot->dart_mode = DART_MODE_STOPPED;
  }

  // 摇杆边沿检测, 每个控制周期只更新一次
  edge_l1 = DartTestEdge(rc_data[TEMP].rc.rocker_l1, DART_TEST_CMD_THRESHOLD, &last_stick_l1);
  edge_r1 = DartTestEdge(rc_data[TEMP].rc.rocker_r1, DART_TEST_CMD_THRESHOLD, &last_stick_r1);
  edge_r_ = DartTestEdge(rc_data[TEMP].rc.rocker_r_, DART_TEST_MODE_THRESHOLD, &last_stick_r_);
}

/**
 * @brief 飞镖模式切换处理
 */
void DartModeChangeHandler(void) {
  switch (robot->dart_mode) {
    case DART_MODE_MANUAL:
      auto_cycle_enabled = 0;
      auto_step = TEST_STEP_IDLE;
      servo_angle_set = dart->param.servo_lock_angle;
      DartShootSetMode(dart, DART_SHOOT_MODE_MANUAL);
      LOGINFO("[dart] manual test: L1 belt, L_ trigger motor, dial servo, R_ servo trim");
      break;

    case DART_MODE_AUTO:
      auto_step = TEST_STEP_IDLE;
      DartShootSetMode(dart, DART_SHOOT_MODE_AUTO);
      LOGINFO("[dart] auto test: L1 pull/reload, R1 fire/reset, R_ start/stop auto cycle");
      if (!dart->feed.is_homed) {
        DartShootSendCmd(dart, DART_SHOOT_CMD_HOME);  // 未回零时先自动回零
      }
      break;

    case DART_MODE_STOPPED:
    default:
      auto_cycle_enabled = 0;
      auto_step = TEST_STEP_IDLE;
      DartShootSetMode(dart, DART_SHOOT_MODE_STOPPED);
      LOGWARNING("[dart] test off: motors disabled");
      break;
  }
}

/**
 * @brief 急停处理
 */
void DartEmergencyHandler(void) {
  auto_cycle_enabled = 0;
  auto_step = TEST_STEP_IDLE;
  DartShootSetMode(dart, DART_SHOOT_MODE_STOPPED);
  LOGERROR("[dart] emergency stop: motors off, servo back to lock");
}

/**
 * @brief 关闭模式
 */
void DartStoppedHandler(void) {
  DartShootSetMode(dart, DART_SHOOT_MODE_STOPPED);
}

/**
 * @brief 手动测试模式: 逐个执行器单独给定, 用于确认接线/方向/舵机角度
 */
void DartManualModeHandler(void) {
  float now = DWT_GetTimeline_ms();

  // 左摇杆竖直: 同步带电机, 前推 = 拉滑块方向
  float pull_speed = MapStickToSpeed(rc_data[TEMP].rc.rocker_l1, dart->param.pull_speed_limit) *
                     (float)dart->param.pull_direction;
  // 左摇杆水平: 扳机位置调整电机
  float trigger_speed = MapStickToSpeed(rc_data[TEMP].rc.rocker_l_, dart->param.trigger_speed_limit);
  // 拨轮: 舵机角度(锁止位 ~ 释放位线性映射)
  float servo_range = dart->param.servo_release_angle - dart->param.servo_lock_angle;
  servo_angle_set =
      dart->param.servo_lock_angle + ((float)rc_data[TEMP].rc.dial / DART_TEST_STICK_MAX) * servo_range;
  // 右摇杆水平: 舵机角度微调(增量), 用于找准锁止位/释放位
  if (!IsInDeadzone(rc_data[TEMP].rc.rocker_r_)) {
    servo_angle_set += ((float)rc_data[TEMP].rc.rocker_r_ / DART_TEST_STICK_MAX) * DART_TEST_SERVO_TRIM_STEP;
  }
  servo_angle_set =
      float_constrain(servo_angle_set, dart->param.servo_angle_min, dart->param.servo_angle_max);

  DartShootSetManualOutput(dart, pull_speed, trigger_speed, servo_angle_set);

  // 周期性输出状态, 便于记录电机方向、行程与舵机角度
  if (now - test_log_time >= DART_TEST_LOG_PERIOD_MS) {
    char pos_str[16], speed_str[16], angle_str[16];
    test_log_time = now;
    Float2Str(pos_str, dart->feed.pull_position);
    Float2Str(speed_str, dart->feed.pull_speed);
    Float2Str(angle_str, servo_angle_set);
    LOGINFO("[dart] manual: pos %s, belt %s deg/s, servo %s deg", pos_str, speed_str, angle_str);
  }
}

/**
 * @brief 自动测试模式
 *
 * @note  右摇杆水平推到底启动/停止自动循环测试;
 *        自动循环测试自动执行 回零 -> 拉滑块 -> 击发 -> 复位, 并统计耗时与成败
 */
void DartAutoTestHandler(void) {
  DartShoot_Feed_s* feed = &dart->feed;

  // 状态变化输出
  static DartShoot_State_e last_state = DART_SHOOT_STATE_STOPPED;
  if (last_state != feed->state) {
    last_state = feed->state;
    LOGINFO("[dart] dart_shoot state -> %s", DartStateName(feed->state));
  }

  // 右摇杆水平推到底: 启动/停止自动循环测试
  if (edge_r_ == DART_TEST_EDGE_UP) {
    DartAutoCycleStart();
  } else if (edge_r_ == DART_TEST_EDGE_DOWN) {
    DartAutoCycleStop();
  }

  if (auto_cycle_enabled) {
    DartAutoCycleHandler();
    return;
  }

  // 单步测试: 左摇杆竖直 拉滑块/复位, 右摇杆竖直 击发/清异常
  if (edge_l1 == DART_TEST_EDGE_UP) {
    DartShootSendCmd(dart, DART_SHOOT_CMD_PULL);
  } else if (edge_l1 == DART_TEST_EDGE_DOWN) {
    DartShootSendCmd(dart, DART_SHOOT_CMD_RELOAD);
  }
  if (edge_r1 == DART_TEST_EDGE_UP) {
    DartShootSendCmd(dart, DART_SHOOT_CMD_FIRE);
  } else if (edge_r1 == DART_TEST_EDGE_DOWN) {
    DartShootSendCmd(dart, DART_SHOOT_CMD_RESET);
  }
}

/*=======自动循环测试=======*/
/**
 * @brief 启动自动循环测试
 */
static void DartAutoCycleStart(void) {
  if (auto_cycle_enabled) {
    return;
  }
  if (!dart->feed.is_online) {
    LOGERROR("[dart] auto cycle refused: 3508 offline, check CAN wiring");
    return;
  }

  memset(&robot->test_stat, 0, sizeof(DartTest_Stat_s));
  auto_cycle_enabled = 1;
  auto_step = TEST_STEP_IDLE;
  auto_attempt = 0;
  auto_fail_streak = 0;
  LOGINFO("[dart] ===== auto cycle test start, target %d cycles =====", DART_TEST_AUTO_CYCLE_NUM);
}

/**
 * @brief 停止自动循环测试
 */
static void DartAutoCycleStop(void) {
  if (!auto_cycle_enabled) {
    return;
  }
  auto_cycle_enabled = 0;
  auto_step = TEST_STEP_IDLE;
  DartShootSendCmd(dart, DART_SHOOT_CMD_ABORT);
  LOGWARNING("[dart] auto cycle test stopped by user");
  DartTestReport();
}

/**
 * @brief 自动循环测试状态机
 */
static void DartAutoCycleHandler(void) {
  DartShoot_Feed_s* feed = &dart->feed;
  float now = DWT_GetTimeline_ms();

  if (!feed->is_online) {
    LOGERROR("[dart] motor offline during auto cycle, test stopped");
    DartAutoCycleFail(DART_SHOOT_ERROR_MOTOR_OFFLINE);
    auto_cycle_enabled = 0;
    DartTestReport();
    return;
  }

  switch (auto_step) {
    case TEST_STEP_IDLE:
      if (auto_attempt >= DART_TEST_AUTO_CYCLE_NUM) {
        auto_cycle_enabled = 0;
        DartTestReport();
        return;
      }
      if (feed->is_homed) {
        auto_attempt++;
        cycle_start_time = now;
        DartShootSendCmd(dart, DART_SHOOT_CMD_PULL);
        auto_step = TEST_STEP_PULLING;
      } else {
        DartShootSendCmd(dart, DART_SHOOT_CMD_HOME);
        auto_step = TEST_STEP_HOMING;
      }
      break;

    case TEST_STEP_HOMING:
      if (feed->is_homed) {
        auto_attempt++;
        cycle_start_time = now;
        DartShootSendCmd(dart, DART_SHOOT_CMD_PULL);
        auto_step = TEST_STEP_PULLING;
      } else if (feed->error != DART_SHOOT_ERROR_NONE) {
        DartAutoCycleFail(feed->error);
      }
      break;

    case TEST_STEP_PULLING:
      if (feed->state == DART_SHOOT_STATE_READY) {
        // 记录到位判定方式, 用于判断 pull_travel 是否标定正确
        arrive_by_encoder = (feed->pull_traveled + 2.0f * dart->param.pull_position_tolerance >=
                             dart->param.pull_travel);
        DartShootSendCmd(dart, DART_SHOOT_CMD_FIRE);
        auto_step = TEST_STEP_FIRING;
      } else if (feed->error != DART_SHOOT_ERROR_NONE) {
        DartAutoCycleFail(feed->error);
      }
      break;

    case TEST_STEP_FIRING:
      if (feed->state == DART_SHOOT_STATE_IDLE) {
        DartAutoCyclePass(now - cycle_start_time);
        auto_step = TEST_STEP_IDLE;
      } else if (feed->error != DART_SHOOT_ERROR_NONE) {
        DartAutoCycleFail(feed->error);
      }
      break;

    default:
      auto_step = TEST_STEP_IDLE;
      break;
  }
}

/**
 * @brief 自动循环测试: 单个周期通过
 */
static void DartAutoCyclePass(float cycle_time_ms) {
  DartShoot_Feed_s* feed = &dart->feed;
  DartTest_Stat_s* stat = &robot->test_stat;
  char pull_str[16], fire_str[16], cycle_str[16];

  stat->pass++;
  auto_fail_streak = 0;
  stat->pull_time_sum_ms += feed->pull_time_ms;
  stat->fire_time_sum_ms += feed->fire_time_ms;
  stat->cycle_time_sum_ms += cycle_time_ms;
  if (stat->pull_time_min_ms <= 0.0f || feed->pull_time_ms < stat->pull_time_min_ms) {
    stat->pull_time_min_ms = feed->pull_time_ms;
  }
  if (feed->pull_time_ms > stat->pull_time_max_ms) {
    stat->pull_time_max_ms = feed->pull_time_ms;
  }
  if (arrive_by_encoder) {
    stat->arrive_by_encoder++;
  } else {
    stat->arrive_by_stall++;
  }

  Float2Str(pull_str, feed->pull_time_ms);
  Float2Str(fire_str, feed->fire_time_ms);
  Float2Str(cycle_str, cycle_time_ms);
  LOGINFO("[dart] cycle %u/%d PASS: pull %s ms, fire %s ms, cycle %s ms, arrive by %s", auto_attempt,
          DART_TEST_AUTO_CYCLE_NUM, pull_str, fire_str, cycle_str, arrive_by_encoder ? "encoder" : "stall");
}

/**
 * @brief 自动循环测试: 单个周期失败
 */
static void DartAutoCycleFail(DartShoot_Error_e error) {
  DartTest_Stat_s* stat = &robot->test_stat;

  stat->fail++;
  auto_fail_streak++;
  switch (error) {
    case DART_SHOOT_ERROR_PULL_TIMEOUT:
      stat->fail_pull_timeout++;
      break;
    case DART_SHOOT_ERROR_HOME_TIMEOUT:
      stat->fail_home_timeout++;
      break;
    case DART_SHOOT_ERROR_RELOAD_TIMEOUT:
      stat->fail_reload_timeout++;
      break;
    case DART_SHOOT_ERROR_MOTOR_OFFLINE:
      stat->fail_offline++;
      break;
    default:
      break;
  }

  LOGERROR("[dart] cycle %u FAILED, error code: %d, fail streak: %u", auto_attempt, error, auto_fail_streak);
  DartShootSendCmd(dart, DART_SHOOT_CMD_RESET);
  auto_step = TEST_STEP_IDLE;

  if (auto_fail_streak >= DART_TEST_AUTO_MAX_FAIL) {
    LOGERROR("[dart] too many failures, auto cycle test stopped");
    auto_cycle_enabled = 0;
    DartTestReport();
  }
}

/**
 * @brief 输出自动循环测试统计结果
 */
static void DartTestReport(void) {
  DartTest_Stat_s* stat = &robot->test_stat;
  char time1[16], time2[16], time3[16];

  LOGINFO("[dart] ===== auto cycle test result =====");
  LOGINFO("[dart] attempts %u, pass %u, fail %u", auto_attempt, stat->pass, stat->fail);
  LOGINFO("[dart] arrive by encoder %u, by stall %u", stat->arrive_by_encoder, stat->arrive_by_stall);
  if (stat->pass > 0) {
    Float2Str(time1, stat->pull_time_sum_ms / (float)stat->pass);
    Float2Str(time2, stat->pull_time_min_ms);
    Float2Str(time3, stat->pull_time_max_ms);
    LOGINFO("[dart] pull time avg/min/max: %s / %s / %s ms", time1, time2, time3);
    Float2Str(time1, stat->fire_time_sum_ms / (float)stat->pass);
    Float2Str(time2, stat->cycle_time_sum_ms / (float)stat->pass);
    LOGINFO("[dart] fire time avg: %s ms, cycle time avg: %s ms", time1, time2);
  }
  LOGINFO("[dart] fail detail: pull timeout %u, home timeout %u, reload timeout %u, offline %u",
          stat->fail_pull_timeout, stat->fail_home_timeout, stat->fail_reload_timeout, stat->fail_offline);
}

/**
 * @brief 摇杆边沿检测: 返回上跳/下跳沿, 并保存本次摇杆值
 */
static uint8_t DartTestEdge(int16_t stick, int16_t threshold, int16_t* last) {
  uint8_t edge = DART_TEST_EDGE_NONE;
  if (stick > threshold && *last <= threshold) {
    edge = DART_TEST_EDGE_UP;
  } else if (stick < -threshold && *last >= -threshold) {
    edge = DART_TEST_EDGE_DOWN;
  }
  *last = stick;
  return edge;
}

/**
 * @brief 组件状态名, 用于日志输出
 */
static const char* DartStateName(DartShoot_State_e state) {
  switch (state) {
    case DART_SHOOT_STATE_STOPPED:
      return "STOPPED";
    case DART_SHOOT_STATE_MANUAL:
      return "MANUAL";
    case DART_SHOOT_STATE_IDLE:
      return "IDLE";
    case DART_SHOOT_STATE_HOMING:
      return "HOMING";
    case DART_SHOOT_STATE_PULLING:
      return "PULLING";
    case DART_SHOOT_STATE_READY:
      return "READY";
    case DART_SHOOT_STATE_RELEASING:
      return "RELEASING";
    case DART_SHOOT_STATE_RELOADING:
      return "RELOADING";
    case DART_SHOOT_STATE_ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}
