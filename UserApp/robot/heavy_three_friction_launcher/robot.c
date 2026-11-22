/**
******************************************************************************
* @file    robot.c
* @brief   heavy_three_friction_launcher —— 云台 + 三摩擦轮发射机构台架
*          只初始化云台(yaw/pitch)与发射机构(3 摩擦轮 + 1 拨盘)
*          不初始化底盘、超级电容、也不依赖裁判系统数据
*
*          遥控: DBUS (C 板走 USART3, H7 板走 UART5, 均 100kbps)
*          无裁判系统: 不读裁判报文; 热量上限与当前热量直接写进
*          Shoot_Ctrl_Cmd_s, 由 shoot.c 的 SIMULLATE_CONTROL 模式本地模拟
*
*          左拨杆三档: 下=停火 / 中=单发 / 上=连发
*          发弹触发: 左摇杆向下拨动(rocker_l1, 单发档一次一发, 连发档按住连续发)
*
*          安全/调试补丁(见文末说明):
*            1) 上电先全停, 首次收到遥控数据且在线后才放开
*            2) 拨盘"到位门限": 上一发没走完不放行下一发, 卡弹时目标不再叠加
*            3) 拨盘堵转标志被置位时自动倒转一发并清标志
*            4) 状态日志只在变化时打印(原来 200Hz 每帧都打)
*            5) yaw 增加 ±150° 软限位
*            6) bench_dbg 调试快照, 供 Ozone / watch 观察
******************************************************************************
*/

#include "robot.h"
#include "robot_config.h"

#include "bsp_log.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "user_lib.h"
#include "remote_control.h"
#include "controller.h"      // ErrorType_e / PID_MOTOR_BLOCKED_ERROR

#include <math.h>

/* 摩擦轮基础转速与拨轮微调步长.
 * 注意量纲: 速度环反馈是 measure.speed_aps(度/秒),
 * 所以 25000 表示 25000 °/s ≈ 4167 rpm, 不是 25000 rpm */
#define FRICTION_SPEED_BASE 25000.0f
#define FRICTION_DIAL_TRIM 2.0f

/* 摩擦轮半径(米)与传动比, 只为把 speed_aps 换算成轮面线速度 v = ω * R / i.
 * 直连转子时 i = 1; 若摩擦轮装在 19:1 齿箱输出上, i 改成 19 */
#define FRICTION_WHEEL_RADIUS_M 0.030f
#define FRICTION_GEAR_RATIO     1.0f
#define BENCH_DEG2RAD           0.0174533f

/* 台架热量上限: 2027 重装装好 42mm 发射机构后为 300, 与 robot_config.h 里
 * shoot_param.shooter_barrel_heat_limit 保持一致, 两处要一起改 */
#define BENCH_HEAT_LIMIT 300

/* 左摇杆(y 通道, rocker_l1)发弹判定: 低于 DOWN 算"拨下", 回到 RELEASE 以上才允许下一次触发 */
#define STICK_DOWN_THRESHOLD    (-300)
#define STICK_RELEASE_THRESHOLD (-150)

/* yaw 软限位, 按台架实际走线/机械范围调整 */
#define YAW_SOFT_LIMIT 150.0f

/* 拨盘到位门限: 一发对应电机角 60*100*(-1) = -6000°,
 * 走过 5000° 视为这一发已经完成; 没走完之前不放行下一发 */
#define LOADER_STEP_ANGLE_DEG   6000.0f
#define LOADER_STEP_DONE_DEG    5000.0f

static RobotInstance *robot;
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd;
static Shoot_Ctrl_Cmd_s *shoot_ctrl_cmd;
static ShootInstance *shoot_inst;

static RC_ctrl_t *rc_data;  // 长度为 2 的数组, TEMP 为当前帧

/* 左摇杆回中锁: 1=允许触发, 0=已触发过一次, 回中后解锁 */
static uint8_t stick_armed = 1;

/* 上电保护: 0=还没收到过在线遥控, 保持全停 */
static uint8_t bench_rc_ready = 0;

/* 拨盘到位门限状态 */
static uint8_t loader_busy = 0;          // 1 = 上一发还没走完
static float loader_start_angle = 0.0f;  // 上一发触发瞬间的电机总角度

/* 调试快照 */
volatile BenchDebug_s bench_dbg;

/**
 * @brief 标记"已经放行一发", 记录起始角度, 用于到位门限
 */
static void BenchMarkShot(void) {
  loader_start_angle = shoot_inst->loader_motor->measure.total_angle;
  loader_busy = 1;
}

/**
 * @brief 遥控器输入 -> 云台/发射控制量
 *        左拨杆: 上=停火, 中=单发, 下=连发
 *        发弹:   左摇杆向下拨动(rocker_l1)
 */
static void RemoteControlSet(void) {
  /* 上电后还没拿到过在线遥控: 一切保持停机, 不累加云台角、不动电机 */
  if (!bench_rc_ready) return;

  int16_t stick = rc_data[TEMP].rc.rocker_l1;  // 左摇杆竖直轴(向下为负)
  uint8_t stick_down = (stick <= STICK_DOWN_THRESHOLD) ? 1 : 0;

  /* ---------------- 云台: 只要没离线就一直可动 ---------------- */
  gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
  gimbal_ctrl_cmd->yaw += -0.003f * (float)rc_data[TEMP].rc.rocker_r_;
  gimbal_ctrl_cmd->pitch += 0.0006f * (float)rc_data[TEMP].rc.rocker_r1;

  if (gimbal_ctrl_cmd->yaw > YAW_SOFT_LIMIT) {
    gimbal_ctrl_cmd->yaw = YAW_SOFT_LIMIT;
  } else if (gimbal_ctrl_cmd->yaw < -YAW_SOFT_LIMIT) {
    gimbal_ctrl_cmd->yaw = -YAW_SOFT_LIMIT;
  }

  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }

  /* ---------------- 摩擦轮 ----------------
   * 上档 = 停火: 三只摩擦轮连同拨盘一起停(shoot.c 里 SHOOT_OFF 会直接 DJIMotorStop)
   * 中档(单发) / 下档(连发): 摩擦轮一直转, 只受拨轮微调 */
  if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->friction_speed = 0.0f;
  } else {
    shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
    shoot_ctrl_cmd->friction_mode = FRICTION_ON;

    /* 拨轮: 在基础转速上做微调, ±660 对应 ±1320°/s 左右 */
    shoot_ctrl_cmd->friction_speed =
        FRICTION_SPEED_BASE + FRICTION_DIAL_TRIM * (float)rc_data[TEMP].rc.dial;
  }

  /* ---------------- 拨盘到位门限 ----------------
   * 上一发只要走过 LOADER_STEP_DONE_DEG 就算完成; 没走完之前不放行新触发,
   * 这样卡弹时目标角不会一发一发越叠越远, 脱卡瞬间就不会猛冲 */
  if (loader_busy) {
    float delta = fabsf(shoot_inst->loader_motor->measure.total_angle - loader_start_angle);
    if (delta >= LOADER_STEP_DONE_DEG) loader_busy = 0;
  }

  /* ---------------- 拨盘 ----------------
   * 每帧先置 LOAD_STOP, 再按档位放行:
   * 否则上一帧的单发指令会一直留在命令结构体里, 变成每 1s 自动一发 */
  shoot_ctrl_cmd->load_mode = LOAD_STOP;

  if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
    /* 上档 = 停火: 保持 LOAD_STOP 不发弹; 同时清掉到位阻塞, 作为卡弹时的人工复位 */
    loader_busy = 0;
  } else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
    /* 单发档: 左摇杆拨下的一瞬间 -> 一发(6000° 电机角), 回中前不重复触发 */
    if (stick_down && stick_armed && !loader_busy) {
      shoot_ctrl_cmd->load_mode = LOAD_1_BULLET;
      BenchMarkShot();
      stick_armed = 0;
    }
  } else {
    /* 下档 = 连发: 左摇杆按住向下 -> 持续给连发, 500ms 节拍由 shoot.c 的 dead_time 控制 */
    if (stick_down && !loader_busy) {
      shoot_ctrl_cmd->load_mode = LOAD_BURSTFIRE;
      BenchMarkShot();
    }
  }

  if (stick > STICK_RELEASE_THRESHOLD) {
    stick_armed = 1;  // 摇杆回到中位, 解锁下一次单发
  }
}

/**
 * @brief 拨盘堵转处理
 *        shoot.c 只对拨盘速度环启用了 PID_ErrorHandle, 但整个工程里没人读这个标志,
 *        这里读一次: 被置位就倒转一发(LOAD_REVERSE)并清标志, 让它自己退出来
 */
static void LoaderBlockedHandler(void) {
  PIDInstance *speed_pid = &shoot_inst->loader_motor->motor_controller.speed_PID;

  if (speed_pid->ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR) {
    shoot_ctrl_cmd->load_mode = LOAD_REVERSE;
    speed_pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
    speed_pid->ERRORHandler.ERRORCount = 0;

    /* 倒转后重新计一发, 避免接着被判"没走完" */
    BenchMarkShot();
    LOGERROR("[bench] loader blocked, reverse one step");
  }
}

/**
 * @brief 急停: 只在遥控器离线时触发
 *        (左拨杆下档已用于"停火", 不再兼作急停)
 *        日志只在状态切换时打印, 避免 200Hz 刷屏
 */
static void EmergencyHandler(void) {
  static uint8_t last_state = 0xFF;  // 0xFF=未知, 0=离线, 1=在线

  if (!RemoteControlIsOnline()) {
    robot->robot_mode = ROBOT_POWER_OFF;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->load_mode = LOAD_STOP;
    bench_rc_ready = 0;  // 掉线后又回到上电保护状态
    loader_busy = 0;
    if (last_state != 0) {
      LOGERROR("[CMD] bench emergency stop: RC offline!");
      last_state = 0;
    }
  } else {
    robot->robot_mode = ROBOT_POWER_ON;
    bench_rc_ready = 1;
    if (last_state != 1) {
      LOGINFO("[CMD] bench ready");
      last_state = 1;
    }
  }
}

/**
 * @brief 调试快照: 每帧刷新 bench_dbg, Ozone / watch 里看这一个结构就够
 */
static void BenchUpdateDebug(void) {
  DJIMotorInstance *yaw = robot->gimbal->yaw_motor;
  DJIMotorInstance *pitch = robot->gimbal->pitch_motor;
  DJIMotorInstance *loader = shoot_inst->loader_motor;

  bench_dbg.rc_online = RemoteControlIsOnline();
  bench_dbg.sw_left = rc_data[TEMP].rc.switch_left;
  bench_dbg.rocker_l1 = rc_data[TEMP].rc.rocker_l1;
  bench_dbg.rocker_r_ = rc_data[TEMP].rc.rocker_r_;
  bench_dbg.rocker_r1 = rc_data[TEMP].rc.rocker_r1;
  bench_dbg.dial = rc_data[TEMP].rc.dial;
  bench_dbg.stick_armed = stick_armed;
  bench_dbg.load_mode_now = (uint8_t)shoot_ctrl_cmd->load_mode;
  bench_dbg.loader_busy = loader_busy;

  bench_dbg.yaw_angle = yaw->measure.total_angle;
  bench_dbg.yaw_speed = yaw->measure.speed_aps;
  bench_dbg.pitch_angle = pitch->measure.total_angle;
  bench_dbg.pitch_speed = pitch->measure.speed_aps;

  bench_dbg.loader_angle = loader->measure.total_angle;
  bench_dbg.loader_delta = loader->measure.total_angle - loader_start_angle;
  bench_dbg.loader_speed = loader->measure.speed_aps;
  bench_dbg.loader_current = loader->measure.real_current;
  bench_dbg.loader_temp = loader->measure.temperature;
  bench_dbg.loader_err_type = (uint32_t)loader->motor_controller.speed_PID.ERRORHandler.ERRORType;
  bench_dbg.loader_err_cnt = (uint32_t)loader->motor_controller.speed_PID.ERRORHandler.ERRORCount;

  for (int i = 0; i < FRICTION_NUM; i++) {
    bench_dbg.fric_ref[i] = shoot_inst->friction_motor[i]->motor_controller.pid_ref;
    bench_dbg.fric_out[i] = shoot_inst->friction_motor[i]->motor_controller.final_output;
    bench_dbg.fric_stop[i] = (shoot_inst->friction_motor[i]->stop_flag == MOTOR_STOP) ? 1 : 0;
    bench_dbg.fric_speed[i] = shoot_inst->friction_motor[i]->measure.speed_aps;
    bench_dbg.fric_current[i] = shoot_inst->friction_motor[i]->measure.real_current;
    bench_dbg.fric_temp[i] = shoot_inst->friction_motor[i]->measure.temperature;

    /* 轮面线速度 = |°/s| * (π/180) * R / i, 目标侧用同样换算方便对比 */
    bench_dbg.fric_v[i] =
        fabsf(bench_dbg.fric_speed[i]) / FRICTION_GEAR_RATIO * BENCH_DEG2RAD * FRICTION_WHEEL_RADIUS_M;
    bench_dbg.fric_v_ref[i] =
        fabsf(bench_dbg.fric_ref[i]) / FRICTION_GEAR_RATIO * BENCH_DEG2RAD * FRICTION_WHEEL_RADIUS_M;
    /* 跟踪比: |实际| / |目标|, 目标为 0(停火)时置 0 避免除零 */
    bench_dbg.fric_track[i] =
        (fabsf(bench_dbg.fric_ref[i]) > 1.0f) ? fabsf(bench_dbg.fric_speed[i]) / fabsf(bench_dbg.fric_ref[i]) : 0.0f;
  }
}

void RobotInit() {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

  /* ---------------- 遥控器: DBUS ----------------
   * C 板(F407): USART3 = DBUS 口, 100kbps
   * H7 板:       UART5  = DBUS 口, 100kbps */
#ifdef STM32F407xx
  robot->rc_data = (void *)RemoteControlInit(&huart3);
#elif defined(STM32H7)
  robot->rc_data = (void *)RemoteControlInit(&huart5);
#endif
  rc_data = (RC_ctrl_t *)robot->rc_data;

  /* ---------------- 云台 + 发射 ---------------- */
  robot->gimbal = GimbalInit(&gimbal_init_config);
  robot->shoot = ShootInit(&shoot_init_config);

  gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;
  shoot_inst = robot->shoot;

  /* 台架: 不做弹速闭环, 热量用本地模拟控制.
   * 热量上限/当前热量由 cmd 直接给, 不经过裁判系统解析 */
  shoot_ctrl_cmd->bullet_speed_mode = DISABLE_BULLET_SPEED;
  shoot_ctrl_cmd->heat_mode = SIMULLATE_CONTROL;
  shoot_ctrl_cmd->shooter_barrel_heat_limit = BENCH_HEAT_LIMIT;
  shoot_ctrl_cmd->shooter_barrel_heat = 0;
  shoot_ctrl_cmd->initial_speed = 0.0f;

  /* ---------------- 上电先全停 ----------------
   * 等第一次收到在线遥控(由 EmergencyHandler 置 bench_rc_ready)再放开,
   * 避免刚上电、DBUS 还没数据时就先给电机下指令 */
  shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
  shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
  shoot_ctrl_cmd->load_mode = LOAD_STOP;
  gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;

  loader_start_angle = shoot_inst->loader_motor->measure.total_angle;
}

/* 台架控制指令: 只处理遥控映射、堵转处理与急停, 不涉及底盘 */
static void RobotCMDTask(void) {
  RemoteControlSet();
  LoaderBlockedHandler();
  EmergencyHandler();
}

void RobotTask() {
  RobotCMDTask();
  GimbalTask();
  ShootTask();
  BenchUpdateDebug();
}

RobotInstance *getRobot() {
  return robot;
}
