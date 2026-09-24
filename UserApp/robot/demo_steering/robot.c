//
// 四舵轮 demo: robot 层初始化、遥控与周期性自检
//
// 底盘组件使用 dev 分支的 chassis_steering 版本(2026/1/1 重写版)。
// 该组件的 ChassisInit() 会无条件注册 4 个 3508 + 4 个 6020, 没有使能掩码;
// 超电与裁判系统都是可选的: 超电未配置时不注册 CAN 实例、不做状态机,
// 没有裁判系统时功率上限退回 DEMO_CHASSIS_POWER_LIMIT(见 robot_config.h)。
// 因此本文件只负责:
//   1. 初始化底盘与遥控器;
//   2. 把摇杆映射成底盘指令;
//   3. 周期性打印观测信息(供调试)。
//
#include "robot.h"

#include <math.h>

#include "arm_math.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "ins_task.h"
#include "robot_config.h"
#include "user_lib.h"

static RobotInstance *robot;
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;

/* 自检打印的计时锚点, 使用 DWT 时间轴(ms) */
static uint32_t last_report_ms;
#if DEMO_SPIN_TEST
static uint32_t spin_start_ms;
#endif

#if DEMO_VIRTUAL_GIMBAL
/**
 * @brief 虚拟云台状态。
 *
 * IMU 的 Yaw 给出"底盘相对上电朝向转过了多少度", 写进 offset_angle 后组件就会
 * 把平移指令始终解算到【上电时的车头朝向】上, 于是能一边自旋一边直行。
 */
static INS_t *ins;              /* INS_Init() 返回的单例(见 ins_task.c: 重复调用直接返回) */
static float imu_boot_yaw;      /* 上电朝向对应的 YawTotalAngle, 即"虚拟云台 0 点" */
static uint8_t boot_yaw_valid;  /* 0 = 仍在采样窗口内, 还没定出 0 点 */
static uint32_t boot_yaw_start_ms;
static float boot_yaw_sum;      /* 采样窗口内的累加与计数, 用来求平均 */
static uint16_t boot_yaw_cnt;
static uint8_t virtual_gimbal_on; /* 左开关下位 = 开 */
#endif

#if DEMO_USE_REMOTE
/* 遥控器数据, 由 RemoteControlInit() 返回; 未初始化时为 NULL */
static RC_ctrl_t *rc_data;

/**
 * @brief 摇杆原始值 → 底盘指令, 带死区与归一化.
 */
static float StickToCmd(int16_t stick, float max_cmd) {
  float value = (float)stick;
  if (fabsf(value) < (float)DEMO_RC_DEADBAND) return 0.0f;
  return max_cmd * value / DEMO_RC_STICK_RANGE;
}
#endif

/**
 * @brief 周期性观测打印: 四个轮电机的反馈 + 遥控器链路状态.
 *
 * 全部使用整数格式: 本工程的 SEGGER_RTT_printf 未开启浮点格式化,
 * 日志接口也不支持 %f(见 bsp_log.h 的说明), 因此速度等浮点量按整数打印.
 */
static void ConnectivityReport(void) {
  if ((uint32_t)(DWT_GetTimeline_ms() - last_report_ms) < DEMO_REPORT_PERIOD_MS) return;
  last_report_ms = DWT_GetTimeline_ms();

  static const char *const kName[4] = {"LF", "LB", "RB", "RF"};

  for (uint8_t i = 0; i < 4; ++i) {
    DJIMotorInstance *m = robot->chassis->wheel_motor[i];
    if (m == NULL) continue;

    // online=1 表示 C620 正在回传 0x200+ID 报文(即收发至少一帧);
    // 手动转动轮子时 ecd 应连续变化, 停止后 speed 回到 0.
    LOGINFO("[demo] wheel %s online=%d ecd=%d speed=%d temp=%d out=%d", kName[i], DaemonIsOnline(m->daemon),
            (int)m->measure.ecd, (int)m->measure.speed_aps, (int)m->measure.temperature,
            (int)m->motor_controller.final_output);
  }

#if DEMO_USE_REMOTE
  /**
   * 遥控器链路 + 【原始摇杆值】+ 【换算后的指令】。
   *
   * raw 用的是同一个函数的原始值, 用来判定极性: 把某个摇杆往某个方向拨到底,
   * 看 raw 的符号即可确定映射对不对 —— 不用再靠猜。
   *     rawL1 = 左竖直(前进轴)   rawL_ = 左水平(横移轴)   rawR_ = 右水平(自转轴)
   *     rawSL = 左侧开关
   */
  if (rc_data != NULL) {
    LOGINFO("[demo] rc online=%d rawL1=%d rawL_=%d rawR_=%d rawSL=%d | cmd vx=%d vy=%d wz=%d mode=%d",
            (int)RemoteControlIsOnline(), (int)rc_data[TEMP].rc.rocker_l1, (int)rc_data[TEMP].rc.rocker_l_,
            (int)rc_data[TEMP].rc.rocker_r_, (int)rc_data[TEMP].rc.switch_left, (int)chassis_ctrl_cmd->vx,
            (int)chassis_ctrl_cmd->vy, (int)chassis_ctrl_cmd->wz, (int)chassis_ctrl_cmd->chassis_mode);
  } else {
    LOGINFO("[demo] rc NOT initialized");
  }
#endif
}

/**
 * @brief 可选的通电转动自检, 默认关闭(DEMO_SPIN_TEST = 0).
 *
 * 速度环 Kp=1 时 speed_PID 的输出约等于目标速度, 所以这里给的速度参考近似于电流指令,
 * 梯形缓升缓降避免电流突变. 调试时请务必把轮子架空.
 */
static void SpinTest(void) {
#if DEMO_SPIN_TEST
  float elapsed = (float)(uint32_t)(DWT_GetTimeline_ms() - spin_start_ms);
  float phase = fmodf(elapsed, (float)DEMO_SPIN_PERIOD_MS) / (float)DEMO_SPIN_PERIOD_MS;  // 0~1

  // 梯形(缓升-保持-缓降)包络, 峰值出现在 0.5, 端点处为 0
  float ramp = phase < 0.25f   ? phase * 4.0f
               : phase < 0.75f ? 1.0f
                               : (1.0f - phase) * 4.0f;

  // master 版无条件注册全部电机, 这里对四个轮电机同时施加, 保持整车姿态一致
  for (uint8_t i = 0; i < 4; ++i) {
    if (robot->chassis->wheel_motor[i] != NULL) {
      DJIMotorSetPIDRef(robot->chassis->wheel_motor[i], DEMO_SPIN_SPEED * ramp);
    }
  }
#endif
}

/**
 * @brief 舵机零偏标定检查.
 *
 * 组件会无条件注册四个 GM6020; 若某个零偏仍是 0, 组件自身会拒绝使能舵电机
 * (轮电机不受影响, 台架单轮组调试照常)。这里只是把结论显式打印出来,
 * 免得"舵机不动"被误判成硬件故障。
 */
static void VerifyRudderCalibration(void) {
  static const char *const kName[4] = {"LF", "LB", "RB", "RF"};

  for (uint8_t i = 0; i < 4; ++i) {
    if (chassis_init_config.chassis_param.rudder_motor_offset[i] == 0) {
      LOGERROR("[demo] rudder %s offset is 0: calibrate it before powering the servos", kName[i]);
    }
  }

  if (!ChassisIsRudderReady()) {
    LOGWARNING("[demo] rudder motors will stay DISABLED until all four offsets are non-zero");
  }
}

/**
 * @brief 周期性状态报告 —— 一行、带结论、只在"变了"或"有错"时才打印。
 *
 * 【为什么要这样设计】之前的版本每 100ms 打 5 行原始数据, 加上电机离线警告
 * (见 dji_motor.c 的限流说明), RTT 会刷到看不清。这里做了三件事:
 *   1. 四个轮子压缩成【一行】, 只列关键列;
 *   2. 每隔 DEMO_REPORT_PERIOD_MS 才评估一次(默认 500ms, 2Hz, 看得清);
 *   3. 只有内容变化或存在故障时才输出 —— 静止正常时终端是安静的。
 *
 * 输出示例:
 *   [dbg] cmd(0,0,0) wheel=OK | LF st=0 cur=0 e=0 o=0
 *                               LB st=0 cur=0 e=0 o=0
 *                               RB st=0 cur=0 e=0 o=0
 *                               RF st=0 cur=0 e=0 o=0
 *   [dbg] cmd(0,10000,0) wheel=OK | LF st=0 cur=1 e=0 o=0 ...
 *
 * 列含义(角度单位: 度):
 *   cmd   = (vx, vy, wz) 坐标变换后的底盘系指令
 *   wheel = 四个 3508 的在线状态, OK / BAD(有轮子离线)
 *   st    = 舵角目标(0 = 指向正前方, +90 = 指向车头右侧)
 *   cur   = 原始ecd - 零偏 => 该轮【现在物理上指向哪】(与 reverse flag 无关)
 *   e     = cur - st => 跟随误差。正常应在个位数; 持续很大说明舵机跟不上
 *   o     = 舵机角度环输出(上限 ±1920)。静止时有明显非零值 => 在较劲
 *
 * 期望值速查:
 *   推前进(cmd vy>0) -> 四轮 st 全 0
 *   推右移(cmd vx>0) -> 四轮 st 全 +90
 *   顺时针自转       -> LF +45  LB -45  RB -135  RF +135
 */
static void DebugWheelDump(void) {
#if DEMO_DEBUG_WHEELS
  static uint32_t last_debug_ms;
  static Chassis_Debug_State_s last;
  static int8_t last_online[4] = {-1, -1, -1, -1};
  static uint8_t has_last;
  static float last_offset_angle;  /* 用于判断 offset_angle 是否变化 */
#if DEMO_VIRTUAL_GIMBAL
  static float last_imu_yaw;
  static uint8_t last_vgimbal;
#endif

  if ((uint32_t)(DWT_GetTimeline_ms() - last_debug_ms) < DEMO_REPORT_PERIOD_MS) return;
  last_debug_ms = DWT_GetTimeline_ms();

  static const char *const kName[4] = {"LF", "LB", "RB", "RF"};

  Chassis_Debug_State_s dbg;
  ChassisGetDebugState(&dbg);

#if DEMO_VIRTUAL_GIMBAL
  int imu_yaw = (ins == NULL) ? 0 : (int)ins->YawTotalAngle;
  int vgimbal = (int)virtual_gimbal_on;
#else
  int imu_yaw = 0;
  int vgimbal = 0;
#endif
  int offset_angle = (int)chassis_ctrl_cmd->offset_angle;

  // 每个轮子的在线状态与跟随误差
  int8_t online[4];
  int err[4], out[4];
  uint8_t wheels_ok = 1;

  for (uint8_t i = 0; i < 4; ++i) {
    DJIMotorInstance *w = robot->chassis->wheel_motor[i];
    DJIMotorInstance *r = robot->chassis->rudder_motor[i];

    online[i] = (r != NULL && DaemonIsOnline(r->daemon)) ? 1 : 0;
    out[i] = (r == NULL) ? 0 : (int)r->motor_controller.angle_PID.Output;

    float e = dbg.cur[i] - dbg.st[i];
    while (e > 180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    err[i] = (int)e;

    if (w == NULL || !DaemonIsOnline(w->daemon)) wheels_ok = 0;
  }

  /**
   * 判断"有没有变化": 指令变了 / 某个 st 或 cur 变了超过 2 度 / 在线状态变了 /
   * offset_angle 变了超过 1 度 / 虚拟云台开关状态变了。
   *
   * @note offset_angle 必须纳入判据: 自旋时它每周期都在变, 不判它就会退化成刷屏;
   *       但也正因为它在变, 自旋时本来就该打印 —— 这正是我们要观察的量。
   */
  uint8_t changed = 0;
  if (!has_last) {
    changed = 1;
  } else {
    if (fabsf(dbg.chassis_vx - last.chassis_vx) > 1.0f || fabsf(dbg.chassis_vy - last.chassis_vy) > 1.0f) changed = 1;
    if (abs(offset_angle - (int)last_offset_angle) > 1) changed = 1;
#if DEMO_VIRTUAL_GIMBAL
    if (abs(imu_yaw - (int)last_imu_yaw) > 2) changed = 1;
    if (vgimbal != (int)last_vgimbal) changed = 1;
#endif
    for (uint8_t i = 0; i < 4; ++i) {
      if (fabsf(dbg.st[i] - last.st[i]) > 2.0f) changed = 1;
      if (fabsf(dbg.cur[i] - last.cur[i]) > 2.0f) changed = 1;
      if (online[i] != last_online[i]) changed = 1;
    }
  }

  last = dbg;
  for (uint8_t i = 0; i < 4; ++i) last_online[i] = online[i];
  last_offset_angle = (float)offset_angle;
#if DEMO_VIRTUAL_GIMBAL
  last_imu_yaw = (float)imu_yaw;
  last_vgimbal = (uint8_t)vgimbal;
#endif
  has_last = 1;

  if (!changed) return;  // 一切照旧 -> 不刷屏

  for (uint8_t i = 0; i < 4; ++i) {
    if (i == 0) {
      /**
       * imu = IMU 的累计 yaw(度, 逆时针为正, 未取模)
       * off = offset_angle(度) —— 虚拟云台算出"底盘相对上电朝向转过了多少度"
       * vg  = 虚拟云台开关(1=开)
       */
      LOGINFO("[dbg] cmd(%d,%d,%d) vg=%d imu=%d off=%d wheel=%s | %s st=%d cur=%d e=%d o=%d", (int)dbg.chassis_vx,
              (int)dbg.chassis_vy, (int)chassis_ctrl_cmd->wz, vgimbal, imu_yaw, offset_angle,
              wheels_ok ? "OK" : "BAD(offline)", kName[i], (int)dbg.st[i], (int)dbg.cur[i], err[i], out[i]);
    } else {
      LOGINFO("                             | %s st=%d cur=%d e=%d o=%d", kName[i], (int)dbg.st[i], (int)dbg.cur[i],
              err[i], out[i]);
    }
  }
#endif
}

/**
 * @brief 遥控器初始化. 未接遥控器或不需要拨杆驱动时把 DEMO_USE_REMOTE 置 0.
 */
static void RemoteControlInitIfNeeded(void) {
#if DEMO_USE_REMOTE
  robot->rc_data = RemoteControlInit(DEMO_RC_UART);
  rc_data = robot->rc_data;
  if (rc_data == NULL) {
    LOGERROR("[demo] RemoteControlInit failed, chassis will stay powered off");
  }
#endif
}

/**
 * @brief 虚拟云台: 用 IMU 的 Yaw 算出底盘相对"上电朝向"转过了多少度, 写进 offset_angle。
 *
 * 【它解决什么问题】
 *   组件在 TransformCommandToChassisFrame() 里用 offset_angle 把【世界系】平移指令
 *   旋回【底盘系】。把它设成"底盘相对上电朝向的转角", 平移方向就被锁死在上电时的
 *   车头朝向上: 车体自转多少度都不影响前进方向, 于是能一边自旋一边直行。
 *
 * 【为什么用相对差而不是绝对值】
 *   参考步兵的做法(infantry_wheel_legged_heu/robot.c:281 把 YawTotalAngle 当 PID 测量值,
 *   目标角里含同一个 yaw 项, 相减时基准抵消): 这里同样只取
 *   (YawTotalAngle - imu_boot_yaw)。yaw 的常值偏差会在差分里自己消掉, 因此不需要
 *   任何"绝对朝向"的标定仪式。
 *
 * 【为什么不能在这里归一化到 ±180】
 *   YawTotalAngle 是【未取模】的累计角, 自旋时会一路累加。一旦取模, 角度会在 ±180
 *   处跳变, offset_angle 跟着跳, 车会突然反向。所以必须原样使用。
 *
 * @note 上电 0 点用【采样窗口内多点平均】定出, 而不是延时后单点采样:
 *       INS_Init() 返回时 EKF 刚初始化、yaw 仍在收敛, 单点会被收敛残差污染。
 *       窗口长度见 DEMO_IMU_SETTLE_MS。
 */
static void CalcOffsetAngle(void) {
#if DEMO_VIRTUAL_GIMBAL
  if (ins == NULL || rc_data == NULL) return;

  // ---- 阶段 1: 上电后先在采样窗口内累加求平均, 定出虚拟云台 0 点 ----
  if (!boot_yaw_valid) {
    boot_yaw_sum += ins->YawTotalAngle;
    boot_yaw_cnt++;
    if ((uint32_t)(DWT_GetTimeline_ms() - boot_yaw_start_ms) >= (uint32_t)DEMO_IMU_SETTLE_MS) {
      imu_boot_yaw = boot_yaw_sum / (float)boot_yaw_cnt;
      boot_yaw_valid = 1;
      LOGINFO("[demo] virtual gimbal: boot yaw=%d (x10 deg), averaged over %d samples", (int)(imu_boot_yaw * 10.0f),
              (int)boot_yaw_cnt);
    }
    // 0 点还没定出来之前, 按"没有虚拟云台"处理, 避免拿未收敛的 yaw 去做变换
    chassis_ctrl_cmd->offset_angle = 0.0f;
    return;
  }

  // ---- 阶段 2: 左开关下位 = 虚拟云台开; 其余档位 = 关(严格恢复原手感) ----
  virtual_gimbal_on = switch_is_down(rc_data[TEMP].rc.switch_left) ? 1 : 0;

  if (virtual_gimbal_on) {
    // 符号见 robot_config.h 的 DEMO_IMU_YAW_SIGN:
    // 组件约定 +wz = 顺时针, IMU 的 Yaw 是逆时针为正, 故默认取负。
    chassis_ctrl_cmd->offset_angle = DEMO_IMU_YAW_SIGN * (ins->YawTotalAngle - imu_boot_yaw);
  } else {
    chassis_ctrl_cmd->offset_angle = 0.0f;
  }
#else
  // 功能被编译掉时, 保持"前进 = 车头方向"的原始行为
  chassis_ctrl_cmd->offset_angle = 0.0f;
#endif
}

/**
 * @brief 摇杆 → 底盘指令.
 *
 * 【坐标系】(与 chassis.c 的 SteeringCalculate 约定一致)
 *    +y  = 前进          +x  = 向右横移          +wz = 顺时针自转
 *
 * 【摇杆映射】(DBUS: rocker_l1 左竖直"正=上", rocker_l_ 左水平"正=右", rocker_r_ 右水平"正=右")
 *    左摇杆竖直 -> vy : 推前 = 前进(+y)
 *    左摇杆水平 -> vx : 推右 = 向右横移(+x)
 *    右摇杆水平 -> wz : 推右 = 顺时针(+wz)
 *    左侧开关   : 上 = 平移+自转(虚拟云台关), 中 = 只平移(虚拟云台关), 下 = 虚拟云台开
 *    右侧开关   : 下 = 底盘断电(急停)
 *
 * @attention 左侧开关三位已被"关/关/开"占满, 因此【下位不再断电】。
 *            断电入口移到【右开关下位】, 这也符合仓库惯例(infantry_steering 用右开关下位做底盘失能)。
 *
 * @attention 上板第一步先验三个方向, 不对只改对应的一个符号宏, 不要动解算公式:
 *    1. 推左摇杆向上, 车应【向前】     -> 不对: 把 DEMO_SIGN_FWD 取负;
 *    2. 推左摇杆向右, 车应【向右横移】 -> 不对: 把 DEMO_SIGN_LAT 取负;
 *    3. 推右摇杆向右, 车应【顺时针转】 -> 不对: 把 DEMO_SIGN_YAW 取负。
 *
 * @note 为什么三个宏默认都是 +1
 *       DBUS 摇杆原始值本来就是"竖直向上为正、水平向右为正", 与底盘坐标系的
 *       "+y 前进 / +x 右移 / +wz 顺时针" 同向, 所以这里是【直通】映射。
 *       若实测某轴反向, 用对应的 DEMO_SIGN_* 翻一下即可, 不要去改解算。
 *
 * @note 为什么用 CHASSIS_FREE 而不是自己造一个枚举外的值
 *       CHASSIS_FREE 是底盘组件为"纯手动"预留的模式: ApplyChassisMode() 对它不做任何
 *       跟随/累加处理, 平移与自转完全由本文件给定的 vx/vy/wz 决定, offset_angle 也
 *       交给本文件(CalcOffsetAngle)维护。组件只保证在枚举取值范围内工作, 传枚举外的
 *       值属于未定义用法, 因此不要再用 "DEMO_CHASSIS_MODE_MANUAL = 3" 那种写法。
 *
 * @note 为什么不用 CHASSIS_FOLLOW
 *       组件在该模式下会执行 `wz += PIDCalculate(follow_pid, offset_angle, 0)` 并依赖
 *       一个真实存在的云台 yaw 反馈做闭环。本车没有云台, 开了就是一个"只涨不回"的
 *       积分器 —— 车会自己越转越快。
 * @note 为什么不用 CHASSIS_ROTATE
 *       组件在该模式下会【自己】持续累加 offset_angle(且那行符号与
 *       TransformCommandToChassisFrame 相反), 会和我们从 IMU 算出来的值打架,
 *       所以虚拟云台必须走 CHASSIS_FREE, 由本文件独占 offset_angle。
 */
static void RemoteControlSet(void) {
#if DEMO_USE_REMOTE
  if (rc_data == NULL || !RemoteControlIsOnline()) return;  // 离线时保持上一状态, 由急停兜底

  // 三个轴的极性由 robot_config.h 的 DEMO_SIGN_* 统一仲裁, 排查方向问题时只改那三个宏。
  // 推左摇杆: 竖直 -> 前进(+y), 水平 -> 右移(+x); 右摇杆水平 -> 顺时针(+wz)
  chassis_ctrl_cmd->vy = DEMO_SIGN_FWD * StickToCmd(rc_data[TEMP].rc.rocker_l1, DEMO_RC_MAX_VY);
  chassis_ctrl_cmd->vx = DEMO_SIGN_LAT * StickToCmd(rc_data[TEMP].rc.rocker_l_, DEMO_RC_MAX_VX);
  float wz = DEMO_SIGN_YAW * StickToCmd(rc_data[TEMP].rc.rocker_r_, DEMO_RC_MAX_WZ);

  // offset_angle 由 CalcOffsetAngle() 统一维护(虚拟云台), 这里不再清零。
  // 见 RobotTask() 的调用顺序: 必须在 ChassisTask() 之前算好。

  if (switch_is_down(rc_data[TEMP].rc.switch_right)) {
    // 右开关下位: 底盘断电(急停)。左开关三位已让给虚拟云台, 断电入口移到这里。
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    chassis_ctrl_cmd->vx = 0.0f;
    chassis_ctrl_cmd->vy = 0.0f;
    chassis_ctrl_cmd->wz = 0.0f;
  } else if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
    // 左上: 平移 + 自转 (虚拟云台【关】, 平移方向 = 车头方向)
    chassis_ctrl_cmd->chassis_mode = CHASSIS_FREE;
    chassis_ctrl_cmd->wz = wz;
  } else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
    // 左中: 只平移 (虚拟云台【关】)
    chassis_ctrl_cmd->chassis_mode = CHASSIS_FREE;
    chassis_ctrl_cmd->wz = 0.0f;
  } else {
    // 左下: 虚拟云台【开】—— 平移方向锁定为上电时的车头朝向, 自转不影响前进方向。
    //       此时推前进 + 右摇杆满舵 = 一边自旋一边直行。
    chassis_ctrl_cmd->chassis_mode = CHASSIS_FREE;
    chassis_ctrl_cmd->wz = wz;
  }
#endif
}

/**
 * @brief 急停与失控保护.
 *
 * 遥控器离线(关遥控器/掉线)时立即停止底盘输出。
 * RemoteControlIsOnline() 在未初始化时返回 0, 因此漏掉初始化也会落到断电分支.
 *
 * @note 断电时把 offset_angle 也清零: 否则下次使能瞬间会拿着一个陈旧的朝向基准
 *       去解算, 车会朝一个意料之外的方向窜一下。
 */
static void EmergencyHandler(void) {
#if DEMO_USE_REMOTE
  if (rc_data == NULL || !RemoteControlIsOnline()) {
    chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    chassis_ctrl_cmd->vx = 0.0f;
    chassis_ctrl_cmd->vy = 0.0f;
    chassis_ctrl_cmd->wz = 0.0f;
    chassis_ctrl_cmd->offset_angle = 0.0f;
  }
#endif
}

void RobotInit() {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

#if DEMO_CALIB_MODE
  // 零偏标定模式: 强制把所有零偏清零, 组件据此拒绝使能舵电机。
  // 这样舵机不会和你的手较劲, 掰到正前方松手后会停在原地。
  for (uint8_t i = 0; i < 4; ++i) {
    chassis_init_config.chassis_param.rudder_motor_offset[i] = 0;
  }
  LOGWARNING("[demo] CALIB MODE: all rudder offsets forced to 0, rudders stay DISABLED.");
  LOGWARNING("[demo] CALIB MODE: turn each wheel to point FORWARD, read ecd= below, then fill them in.");
#endif

  robot->chassis = ChassisInit(&chassis_init_config);
  if (robot->chassis == NULL) {
    // 初始化失败时不能继续调用 ChassisTask(), 否则会解引用空指针
    while (1) {
      LOGERROR("[demo] ChassisInit failed, halting");
    }
  }

  // 无裁判系统时组件的功率上限退回这个设定值(默认 80W)。
  // @attention 若这里传 0, 组件的功率环会把四个轮电流削到 0, 轮子完全不动。
  ChassisSetPowerLimit(DEMO_CHASSIS_POWER_LIMIT);

  VerifyRudderCalibration();

  chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
  // 上电默认断电: 遥控器在线后由开关决定是否使能, 避免一上电就窜出去.
  chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;

  RemoteControlInitIfNeeded();

#if DEMO_VIRTUAL_GIMBAL
  /**
   * 虚拟云台: 初始化 IMU, 并起算"上电 0 点"的采样窗口。
   *
   * @attention 首次调用 INS_Init() 会阻塞一会儿(见 robot_config.h 的说明):
   *   它要把 IMU 加热到 39~41℃ 再采样 5000 次标定陀螺零偏, 因此
   *   【此刻车必须静止且水平】, 否则标出来的零偏是错的。
   *   冷机开机时这个阻塞可能十几秒, 属正常, 不要断电。
   *
   * @note 这里【不】取 0 点, 只起算计时窗口: INS_Init() 返回时 EKF 刚初始化、
   *       yaw 还在收敛, 单点采样会被收敛残差污染。真正的 0 点由 CalcOffsetAngle()
   *       在窗口内多点平均得出。
   */
  ins = INS_Init(&imu_init_config);
  boot_yaw_start_ms = DWT_GetTimeline_ms();
  boot_yaw_sum = 0.0f;
  boot_yaw_cnt = 0;
  boot_yaw_valid = 0;
  LOGINFO("[demo] virtual gimbal ON: settling %dms to fix boot heading, KEEP THE CAR STILL", (int)DEMO_IMU_SETTLE_MS);
#endif

#if DEMO_SPIN_TEST
  spin_start_ms = DWT_GetTimeline_ms();
  LOGWARNING("[demo] SPIN TEST ENABLED: ramping to %d, LIFT THE WHEELS OFF THE GROUND", (int)DEMO_SPIN_SPEED);
#endif

  LOGINFO("[demo] boot: wheel id (LF,LB,RB,RF)=(%d,%d,%d,%d), ctrl 0x200, feedback 0x200+id", DEMO_WHEEL_ID_LF,
          DEMO_WHEEL_ID_LB, DEMO_WHEEL_ID_RB, DEMO_WHEEL_ID_RF);
  LOGINFO("[demo] remote use=%d vx_max=%d vy_max=%d wz_max=%d", (int)DEMO_USE_REMOTE, (int)DEMO_RC_MAX_VX,
          (int)DEMO_RC_MAX_VY, (int)DEMO_RC_MAX_WZ);
  LOGINFO("[demo] chassis: super_cap=%d rudder_ready=%d power_limit=%dW", robot->chassis->super_cap == NULL ? 0 : 1,
          (int)ChassisIsRudderReady(), (int)ChassisGetPowerLimit());

  /**
   * 上电自检汇总 —— 这一行是用来"一眼确认配置对不对"的, 不用翻源码。
   *
   *   rudder_ready 必须为 1, 否则舵机电机会被组件拒绝使能(零偏里有 0)。
   *   rudder_only=1 表示轮子被锁死不会滚, 只验舵向; 要开车必须是 0。
   *   offsets 应等于 DEMO_RUDDER_OFFSET_* 那四个值。
   */
  LOGINFO("[demo] SELFCHECK rudder_ready=%d rudder_only=%d calib_mode=%d offsets=(%d,%d,%d,%d)", 
          (int)ChassisIsRudderReady(), (int)DEMO_RUDDER_ONLY_TEST, (int)DEMO_CALIB_MODE,
          (int)chassis_init_config.chassis_param.rudder_motor_offset[0],
          (int)chassis_init_config.chassis_param.rudder_motor_offset[1],
          (int)chassis_init_config.chassis_param.rudder_motor_offset[2],
          (int)chassis_init_config.chassis_param.rudder_motor_offset[3]);

  if (!ChassisIsRudderReady()) {
    LOGERROR("[demo] rudder NOT ready -> servo will stay disabled. Check DEMO_RUDDER_OFFSET_* (a 0 means uncalibrated).");
  }
#if DEMO_RUDDER_ONLY_TEST
  LOGWARNING("[demo] RUDDER-ONLY TEST: wheels are locked at zero speed, the car will NOT drive.");
#endif
}

void RobotTask() {
  RemoteControlSet();   // 1. 摇杆 -> vx/vy/wz 与 chassis_mode
  EmergencyHandler();   // 2. 失联/急停兜底(会清掉 offset_angle)
  CalcOffsetAngle();    // 3. 定 offset_angle —— 必须在 ChassisTask() 之前, 组件要用它做坐标变换
  ChassisTask();        // 4. 组件: 世界系 -> 底盘系变换 + 逆解算 + 功率限制
  SpinTest();
  ConnectivityReport();
  DebugWheelDump();
}
