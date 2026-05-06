/**
 * @file ui.c
 * @brief 步兵轮腿机器人(SJTU)的自定义裁判系统 UI。
 *
 * UI 元素分为以下几个模块：
 *  - Relative   : 底盘相对云台角度指示（圆环 + 圆弧）
 *  - Leg        : 五连杆腿部位姿（左右腿）
 *  - Status     : 机器人各子系统状态文字
 *  - Cap        : 超级电容电量弧 + 电压数字 + 控制命令
 *  - Speed      : 底盘速度弧 + 数字
 *  - Aim        : 中心瞄准框 + 命中十字
 *
 * 每个模块都遵循统一的三段式结构：
 *  - DrawXxxStatic    : 添加（ADD）静态图形（仅初始化用）
 *  - DrawXxxDynamic   : 添加/更新动态图形（数值会变的）
 *  - 模块持有本地图形描述符（static），跨调用保持有效
 *
 * 调度由 UITask 周期性触发：
 *  - 每帧采样数据，调用 UIChangeCheck() 设置脏标志
 *  - 调用 MyUIRefresh() 只刷新发生变化的元素
 *  - 周期性强制全量刷新，防止客户端图形丢失
 */

#include "ui.h"

#include <math.h>

#include "cmsis_os.h"
#include "referee.h"
#include "referee_ui.h"
#include "robot_config.h"

/* ===========================================================================
 * 常量定义
 * =========================================================================*/

/* ---- 通用任务周期 ---- */
#define UI_TASK_PERIOD_MS 30
#define UI_AUTO_REFRESH_PERIOD_MS 10000  // 全量刷新周期
#define UI_LEG_REFRESH_PERIOD_MS 100     // 腿部强制重绘周期
#define UI_STATUS_REFRESH_PERIOD_MS 300  // 状态强制重绘周期

#define UI_AUTO_REFRESH_INTERVAL_TICKS ((UI_AUTO_REFRESH_PERIOD_MS + UI_TASK_PERIOD_MS - 1) / UI_TASK_PERIOD_MS)
#define UI_LEG_REFRESH_INTERVAL_TICKS ((UI_LEG_REFRESH_PERIOD_MS + UI_TASK_PERIOD_MS - 1) / UI_TASK_PERIOD_MS)
#define UI_STATUS_REFRESH_INTERVAL_TICKS ((UI_STATUS_REFRESH_PERIOD_MS + UI_TASK_PERIOD_MS - 1) / UI_TASK_PERIOD_MS)

/* ---- 相对角度指示器 ---- */
#define UI_GRAPH_LAYER 7
#define UI_RELATIVE_CENTER_X 960
#define UI_RELATIVE_CENTER_Y 540
#define UI_RELATIVE_RADIUS 88
#define UI_RELATIVE_ARC_HALF_ANGLE 16.0f
#define UI_RELATIVE_RING_WIDTH 3
#define UI_RELATIVE_ARC_WIDTH 8

/* ---- 中心瞄准 ---- */
#define UI_AIM_LAYER 8
#define UI_AIM_CROSS_HALF_LEN 18
#define UI_AIM_CROSS_WIDTH 3
#define UI_AIM_RECT_HALF_W 300
#define UI_AIM_RECT_HALF_H 200
#define UI_AIM_RECT_WIDTH 2

/* ---- 五连杆腿部 ---- */
#define UI_LEG_LAYER 7
#define UI_LEG_BASE_X 1750
#define UI_LEG_BASE_Y 480
#define UI_LEG_TOP_BASE_Y 700
#define UI_LEG_LABEL_X 1600
#define UI_LEG_LABEL_OFFSET_Y 20
#define UI_LEG_LABEL_FONT_SIZE 15
#define UI_LEG_LABEL_WIDTH 2
#define UI_LEG_SCALE 400.0f
#define UI_LEG_ROD_WIDTH 5

/* ---- 状态文字 ---- */
#define UI_STATUS_LAYER 8
#define UI_STATUS_LABEL_X 90
#define UI_STATUS_VALUE_X 245
#define UI_STATUS_BASE_Y 850
#define UI_STATUS_ROW_GAP 38
#define UI_STATUS_FONT_SIZE 14
#define UI_STATUS_WIDTH 2
#define UI_STATUS_ROW_COUNT 6
#define UI_STATUS_VALUE_CHARS 12

/* ---- 超级电容 ---- */
#define UI_CAP_LAYER 6
#define UI_CAP_CENTER_X 960
#define UI_CAP_CENTER_Y 540
#define UI_CAP_RADIUS_X 370
#define UI_CAP_RADIUS_Y 370
#define UI_CAP_START_ANGLE 270
#define UI_CAP_MAX_SWEEP 40.0f
#define UI_CAP_WIDTH 22
#define UI_CAP_EMPTY_VOLTAGE 14.0f
#define UI_CAP_FULL_VOLTAGE 23.0f
#define UI_CAP_TEXT_SIZE 17
#define UI_CAP_TEXT_WIDTH 2
#define UI_CAP_E_X 610
#define UI_CAP_E_Y 545
#define UI_CAP_F_X 702
#define UI_CAP_F_Y 775
#define UI_CAP_VOLTAGE_X 480
#define UI_CAP_VOLTAGE_Y 660
#define UI_CAP_CTRL_X UI_CAP_VOLTAGE_X
#define UI_CAP_CTRL_Y 700
#define UI_CAP_CTRL_TEXT_SIZE UI_CAP_TEXT_SIZE
#define UI_CAP_CTRL_TEXT_WIDTH UI_CAP_TEXT_WIDTH

/* ---- 速度 ---- */
#define UI_SPEED_LAYER 6
#define UI_SPEED_CENTER_X UI_CAP_CENTER_X
#define UI_SPEED_CENTER_Y UI_CAP_CENTER_Y
#define UI_SPEED_RADIUS_X UI_CAP_RADIUS_X
#define UI_SPEED_RADIUS_Y UI_CAP_RADIUS_Y
#define UI_SPEED_CENTER_ANGLE 270
#define UI_SPEED_MAX_SWEEP UI_CAP_MAX_SWEEP
#define UI_SPEED_WIDTH UI_CAP_WIDTH
#define UI_SPEED_LEG_MAX 2.5f
#define UI_SPEED_PROSTRATE_MAX 3.0f
#define UI_SPEED_WARN_RATIO 0.85f
#define UI_SPEED_TEXT_X 480
#define UI_SPEED_TEXT_Y 420
#define UI_SPEED_TEXT_SIZE 15
#define UI_SPEED_TEXT_WIDTH UI_CAP_TEXT_WIDTH

/* ===========================================================================
 * 文件级状态
 * =========================================================================*/

/* 调试观察用变量（保留与原文件一致）。 */
int32_t watch_data1[5], watch_data2[5];

/* 通信序号占位（与原文件一致）。 */
uint8_t UI_Seq;

/* 缓存 RobotInstance 中的裁判系统数据指针。 */
static referee_info_t *referee_recv_info;

/* 本文件维护的 UI 交互状态（含脏标志、上次值等）。 */
static Referee_Interactive_info_t interactive_data;

/* ===========================================================================
 * 各 UI 模块持有的图形描述符
 * 跨调用必须保持，因此使用 static 全局存储。
 * =========================================================================*/

/* 相对角度模块 */
static Graph_Data_t UI_RelativeRing;
static Graph_Data_t UI_RelativeArc[2];

/* 腿部模块 */
static Graph_Data_t UI_LegRods[2][5];  // [0]=left, [1]=right
static String_Data_t UI_LegLabel[2];

/* 瞄准模块 */
static Graph_Data_t UI_AimCross[2];
static Graph_Data_t UI_AimRect;
static uint8_t UI_AimCrossVisible;

/* 电容模块 */
static Graph_Data_t UI_CapArc;
static String_Data_t UI_CapTextE;
static String_Data_t UI_CapTextF;
static String_Data_t UI_CapVoltage;
static String_Data_t UI_CapCtrlCmd;

/* 速度模块 */
static Graph_Data_t UI_SpeedArc;
static String_Data_t UI_SpeedValue;

/* 状态模块 */
static String_Data_t UI_StatusLabel[UI_STATUS_ROW_COUNT];
static String_Data_t UI_StatusValue[UI_STATUS_ROW_COUNT];

/* ===========================================================================
 * 通用工具函数
 * =========================================================================*/

/**
 * @brief 生成 3 字节图形名（首字母 + 两位十进制索引），方便批量命名。
 */
static void MakeUiName(char name[4], char prefix, uint8_t index) {
  name[0] = prefix;
  name[1] = (char)('0' + index / 10u);
  name[2] = (char)('0' + index % 10u);
  name[3] = '\0';
}

/** @brief 浮点绝对值。 */
static float AbsFloat(float value) { return value >= 0.0f ? value : -value; }

/** @brief 角度归一化到 [0, 360)。 */
static float NormalizeAngle(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

/** @brief 转换为四舍五入后的 [0, 359] 整数角度。 */
static uint32_t AngleToUiDegree(float angle) {
  uint32_t degree = (uint32_t)(NormalizeAngle(angle) + 0.5f);
  return degree >= 360 ? degree - 360 : degree;
}

/* ===========================================================================
 * 模式 → 字符串映射
 * =========================================================================*/

static const char *RobotModeStr(Robot_Mode_e mode) {
  switch (mode) {
    case ROBOT_POWER_OFF:
      return "P_OFF";
    case ROBOT_CHASSIS_ROTATE:
      return "LEG_ROTATE";
    case ROBOT_CHASSIS_FOLLOW:
      return "LEG_FOLLOW";
    case ROBOT_CHASSIS_FREE:
      return "LEG_FREE";
    case ROBOT_CHASSIS_PROSTRATE_ROTATE:
      return "PRO_ROTATE";
    case ROBOT_CHASSIS_PROSTRATE_FOLLOW:
      return "PRO_FOLLOW";
    case ROBOT_CHASSIS_PROSTRATE_FREE:
      return "PRO_FREE";
    default:
      return "UNK";
  }
}

static const char *ChassisModeStr(Chassis_Mode_e mode) {
  switch (mode) {
    case CHASSIS_POWER_OFF:
      return "P_OFF";
    case CHASSIS_RECOVERY:
      return "RECOVERY";
    case CHASSIS_ON:
      return "ON";
    case CHASSIS_JUMP_READY:
      return "JUMP_READY";
    case CHASSIS_JUMP_START:
      return "JUMP_START";
    case CHASSIS_PROSTRATE:
      return "PROSTRATE";
    default:
      return "UNK";
  }
}

static const char *GimbalModeStr(Gimbal_Mode_e mode) {
  switch (mode) {
    case GIMBAL_POWER_OFF:
      return "P_OFF";
    case GIMBAL_ON:
      return "ON";
    case GIMBAL_VISION:
      return "VISION";
    default:
      return "UNK";
  }
}

static const char *FrictionModeStr(Friction_Mode_e mode) {
  switch (mode) {
    case FRICTION_OFF:
      return "OFF";
    case FRICTION_ON:
      return "ON";
    default:
      return "UNK";
  }
}

static const char *LoaderModeStr(Loader_Mode_e mode) {
  switch (mode) {
    case LOAD_STOP:
      return "STOP";
    case LOAD_REVERSE:
      return "REVERSE";
    case LOAD_1_BULLET:
      return "1";
    case LOAD_3_BULLET:
      return "3";
    case LOAD_BURSTFIRE:
      return "BURST";
    default:
      return "UNK";
  }
}

static const char *SuperCapModeStr(SuperCap_Mode_e mode) {
  switch (mode) {
    case SAFETY_MODE:
      return "SAFETY";
    case PASSIVE_MODE:
      return "PASSIVE";
    case ACTIVE_MODE:
      return "ACTIVE";
    case CHARGING_MODE:
      return "CHARGING";
    default:
      return "UNK";
  }
}

static const char *SuperCapCtrlCmdStr(SuperCap_Ctrl_Cmd_e cmd) { return cmd == BOOST ? "BOOST" : "NORMAL"; }

static uint32_t SuperCapCtrlCmdColor(SuperCap_Ctrl_Cmd_e cmd) {
  return cmd == BOOST ? UI_Color_Purplish_red : UI_Color_Cyan;
}

/** @brief 判断当前是否处于"匍匐"模式（影响速度上限/姿势 UI 等）。 */
static uint8_t IsProstrateMode(Robot_Mode_e robot_mode, Chassis_Mode_e chassis_mode) {
  return chassis_mode == CHASSIS_PROSTRATE || robot_mode == ROBOT_CHASSIS_PROSTRATE_ROTATE ||
         robot_mode == ROBOT_CHASSIS_PROSTRATE_FOLLOW || robot_mode == ROBOT_CHASSIS_PROSTRATE_FREE;
}

static float SpeedMax(uint8_t is_prostrate) { return is_prostrate ? UI_SPEED_PROSTRATE_MAX : UI_SPEED_LEG_MAX; }

static uint32_t SpeedArcColor(const Referee_Interactive_info_t *data) {
  const float max_speed = SpeedMax(data->speed_is_prostrate);
  return AbsFloat(data->speed) >= max_speed * UI_SPEED_WARN_RATIO ? UI_Color_Purplish_red : UI_Color_Cyan;
}

/* ===========================================================================
 * 五连杆运动学求解
 * =========================================================================*/

/**
 * @brief 用关节角解算五连杆的 5 个铰接点和 4 段杆角度。
 * @return 1 成功；0 失败（输入无效或无实数解）
 */
static uint8_t CalculateLegPosture(const LegInstance *leg, float model_x[5], float model_y[5], float leg_phi[4]) {
  if (leg == NULL || leg->joint_motor[0] == NULL || leg->joint_motor[1] == NULL) {
    return 0;
  }

  const Leg_Param_t *param = &leg->param;
  const float phi1 = param->joint_motor_zero_offset[0] + leg->joint_motor[0]->measure.position;
  const float phi4 = param->joint_motor_zero_offset[1] + leg->joint_motor[1]->measure.position;

  const float xb = param->rod_length[0] * cosf(phi1);
  const float yb = param->rod_length[0] * sinf(phi1);
  const float xd = param->rod_length[4] + param->rod_length[3] * cosf(phi4);
  const float yd = param->rod_length[3] * sinf(phi4);

  const float a0 = 2.0f * param->rod_length[1] * (xd - xb);
  const float b0 = 2.0f * param->rod_length[1] * (yd - yb);
  const float c0 = param->rod_length[1] * param->rod_length[1] + (xb - xd) * (xb - xd) + (yb - yd) * (yb - yd) -
                   param->rod_length[2] * param->rod_length[2];
  const float sqrt_arg = a0 * a0 + b0 * b0 - c0 * c0;
  if (sqrt_arg < 0.0f) {
    return 0;
  }

  const float phi2 = 2.0f * atan2f(b0 + sqrtf(sqrt_arg), a0 + c0);
  const float phi3 = atan2f(yb - yd + param->rod_length[1] * sinf(phi2), xb - xd + param->rod_length[1] * cosf(phi2));
  const float xc = xb + param->rod_length[1] * cosf(phi2);
  const float yc = yb + param->rod_length[1] * sinf(phi2);

  const float l5 = param->rod_length[4];
  model_x[0] = 0.0f;
  model_y[0] = 0.0f;
  model_x[1] = l5;
  model_y[1] = 0.0f;
  model_x[2] = l5 - xb;
  model_y[2] = yb;
  model_x[3] = l5 - xc;
  model_y[3] = yc;
  model_x[4] = l5 - xd;
  model_y[4] = yd;

  leg_phi[0] = phi1;
  leg_phi[1] = phi2;
  leg_phi[2] = phi3;
  leg_phi[3] = phi4;
  return 1;
}

/* ===========================================================================
 * 裁判系统 ID 配置
 * =========================================================================*/

/**
 * @brief 根据裁判系统下发的机器人 ID 配置接收方/客户端 ID。
 */
static void DeterminRobotID(void) {
  referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
  referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
  referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID;
  referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

/* ===========================================================================
 * 数据采样：从 RobotInstance 中提取本帧 UI 所需信息
 * =========================================================================*/

/**
 * @brief 获取底盘相对云台角度。
 *
 * - 单板（ONE_BOARD）下使用 robot->offset_angle；
 * - 双板下若底盘板数据可用，则用上报的 ui_chassis_relative_angle_deg_x10。
 */
static float GetRelativeAngle(RobotInstance *robot) {
  float angle = robot->offset_angle;
#if !defined(ONE_BOARD)
  if (robot->chassis_fetch_data) {
    angle = (float)robot->chassis_fetch_data->ui_status.ui_chassis_relative_angle_deg_x10 * 0.1f;
  }
#endif
  return angle;
}

/** @brief 采样腿部位姿（任一腿可用即视为有效）。 */
static void SampleLegPosture(RobotInstance *robot, Referee_Interactive_info_t *data) {
  float model_x[5];
  float model_y[5];
  float leg_phi[4];

  if (robot->chassis && (CalculateLegPosture(robot->chassis->leg[0], model_x, model_y, leg_phi) ||
                         CalculateLegPosture(robot->chassis->leg[1], model_x, model_y, leg_phi))) {
    data->leg_valid = 1;
    data->leg_phi1 = leg_phi[0];
    data->leg_phi2 = leg_phi[1];
    data->leg_phi3 = leg_phi[2];
    data->leg_phi4 = leg_phi[3];
  } else {
    data->leg_valid = 0;
  }
}

/** @brief 采样机器人各子系统状态、电容、速度、瞄准等数据。 */
static void SampleStatusData(RobotInstance *robot, Referee_Interactive_info_t *data) {
  /* 默认值 */
  data->robot_mode = ROBOT_POWER_OFF;
  data->chassis_mode = CHASSIS_POWER_OFF;
  data->gimbal_mode = GIMBAL_POWER_OFF;
  data->friction_mode = FRICTION_OFF;
  data->loader_mode = LOAD_STOP;
  data->super_cap_mode = SAFETY_MODE;
  data->super_cap_ctrl_cmd = NORMAL;
  data->cap_voltage = 0.0f;
  data->cap_error = 1;
  data->speed = 0.0f;
  data->speed_is_prostrate = 0;
  data->aim_target_flag = 0;

  if (robot == NULL) return;

  data->robot_mode = robot->robot_mode;

  if (robot->chassis) {
    data->chassis_mode = robot->chassis->chassis_ctrl_cmd.chassis_mode;
    data->speed = robot->chassis->state_var.x_b_d;
  }
  if (robot->gimbal) {
    data->gimbal_mode = robot->gimbal->gimbal_ctrl_cmd.gimbal_mode;
  }
  if (robot->shoot) {
    data->friction_mode = robot->shoot->shoot_ctrl_cmd.friction_mode;
    data->loader_mode = robot->shoot->shoot_ctrl_cmd.load_mode;
  }
  if (robot->chassis && robot->chassis->super_cap) {
    data->super_cap_mode = robot->chassis->super_cap->super_cap_mode;
    data->super_cap_ctrl_cmd = robot->chassis->super_cap->super_cap_ctrl_cmd;
    data->cap_voltage = robot->chassis->super_cap->cap_msg.cap_v;
    data->cap_error = robot->chassis->super_cap->cap_msg.error_detect;
  }
  if (robot->vision_recv_data) {
    data->aim_target_flag = robot->vision_recv_data->shoot_receive.fire_flag != 0;
  }

#if !defined(ONE_BOARD)
  if (robot->chassis_fetch_data) {
    const UI_Remote_Status_s *status = &robot->chassis_fetch_data->ui_status;
    data->robot_mode = (Robot_Mode_e)status->robot_mode;
    data->gimbal_mode = (Gimbal_Mode_e)status->gimbal_mode;
    data->friction_mode = (Friction_Mode_e)status->friction_mode;
    data->loader_mode = (Loader_Mode_e)status->loader_mode;
    data->aim_target_flag = status->fire_flag != 0;
  }
#endif

  data->speed_is_prostrate = IsProstrateMode(data->robot_mode, data->chassis_mode);
}

/* ===========================================================================
 * 模块：Relative —— 底盘相对云台角度指示
 * =========================================================================*/

static void DrawRelativePosition(float offset_angle, uint32_t operate) {
  const int32_t cx = UI_RELATIVE_CENTER_X;
  const int32_t cy = UI_RELATIVE_CENTER_Y;

  /*
   * 测得的角度方向与裁判系统 UI 水平方向相反，取负后才能保持
   * 前后不变、左右不再镜像。
   */
  const float ui_angle = NormalizeAngle(-offset_angle);
  const float start = NormalizeAngle(ui_angle - UI_RELATIVE_ARC_HALF_ANGLE);
  const float end = NormalizeAngle(ui_angle + UI_RELATIVE_ARC_HALF_ANGLE);

  /* 背景圆环 */
  UICircleDraw(&UI_RelativeRing, "rg0", operate, UI_GRAPH_LAYER, UI_Color_White, UI_RELATIVE_RING_WIDTH, cx, cy,
               UI_RELATIVE_RADIUS);

  if (start <= end) {
    /* 不跨 0°：单段绿色圆弧；第二段画极小黑色圆弧"清除"上一帧分裂状态 */
    UIArcDraw(&UI_RelativeArc[0], "sa0", operate, UI_GRAPH_LAYER, UI_Color_Green, AngleToUiDegree(start),
              AngleToUiDegree(end), UI_RELATIVE_ARC_WIDTH, cx, cy, UI_RELATIVE_RADIUS, UI_RELATIVE_RADIUS);
    UIArcDraw(&UI_RelativeArc[1], "sa1", operate, UI_GRAPH_LAYER, UI_Color_Black, 0, 1, UI_RELATIVE_ARC_WIDTH, cx, cy,
              UI_RELATIVE_RADIUS, UI_RELATIVE_RADIUS);
  } else {
    /* 跨 0°：拆成 [start..360] 和 [0..end] 两段绘制 */
    UIArcDraw(&UI_RelativeArc[0], "sa0", operate, UI_GRAPH_LAYER, UI_Color_Green, AngleToUiDegree(start), 360,
              UI_RELATIVE_ARC_WIDTH, cx, cy, UI_RELATIVE_RADIUS, UI_RELATIVE_RADIUS);
    UIArcDraw(&UI_RelativeArc[1], "sa1", operate, UI_GRAPH_LAYER, UI_Color_Green, 0, AngleToUiDegree(end),
              UI_RELATIVE_ARC_WIDTH, cx, cy, UI_RELATIVE_RADIUS, UI_RELATIVE_RADIUS);
  }

  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_RelativeRing);
  UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_RelativeArc[0], UI_RelativeArc[1]);
}

/* ===========================================================================
 * 模块：Aim —— 中心瞄准框 + 命中十字
 * =========================================================================*/

static void DrawAimIndicator(uint8_t target_locked, uint32_t operate) {
  const uint32_t rect_color = target_locked ? UI_Color_Purplish_red : UI_Color_Yellow;
  const int32_t cx = UI_CENTER_X;
  const int32_t cy = UI_CENTER_Y;

  /* 命中十字：仅在锁定目标时显示，状态切换时按需 ADD/CHANGE/Del */
  if (target_locked || UI_AimCrossVisible) {
    const uint32_t cross_op = target_locked ? (UI_AimCrossVisible ? UI_Graph_Change : UI_Graph_ADD) : UI_Graph_Del;

    UILineDraw(&UI_AimCross[0], "ax0", cross_op, UI_AIM_LAYER, UI_Color_Purplish_red, UI_AIM_CROSS_WIDTH,
               cx - UI_AIM_CROSS_HALF_LEN, cy - UI_AIM_CROSS_HALF_LEN, cx + UI_AIM_CROSS_HALF_LEN,
               cy + UI_AIM_CROSS_HALF_LEN);
    UILineDraw(&UI_AimCross[1], "ax1", cross_op, UI_AIM_LAYER, UI_Color_Purplish_red, UI_AIM_CROSS_WIDTH,
               cx - UI_AIM_CROSS_HALF_LEN, cy + UI_AIM_CROSS_HALF_LEN, cx + UI_AIM_CROSS_HALF_LEN,
               cy - UI_AIM_CROSS_HALF_LEN);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_AimCross[0], UI_AimCross[1]);
    UI_AimCrossVisible = target_locked;
  }

  /* 外框矩形：常显，颜色根据是否锁定切换 */
  UIRectangleDraw(&UI_AimRect, "ar0", operate, UI_AIM_LAYER, rect_color, UI_AIM_RECT_WIDTH, cx - UI_AIM_RECT_HALF_W,
                  cy - UI_AIM_RECT_HALF_H, cx + UI_AIM_RECT_HALF_W, cy + UI_AIM_RECT_HALF_H);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_AimRect);
}

/* ===========================================================================
 * 模块：Leg —— 五连杆腿部位姿
 * =========================================================================*/

static void DrawSingleLegPosture(const LegInstance *leg, uint8_t ui_index, int32_t base_y, uint32_t operate) {
  int32_t px[5] = {UI_LEG_BASE_X, UI_LEG_BASE_X, UI_LEG_BASE_X, UI_LEG_BASE_X, UI_LEG_BASE_X};
  int32_t py[5] = {base_y, base_y, base_y, base_y, base_y};
  uint32_t color = UI_Color_Black;

  if (leg) {
    float mx[5], my[5], phi[4];
    if (CalculateLegPosture(leg, mx, my, phi)) {
      for (uint8_t i = 0; i < 5; i++) {
        px[i] = UI_LEG_BASE_X + (int32_t)(mx[i] * UI_LEG_SCALE);
        py[i] = base_y - (int32_t)(my[i] * UI_LEG_SCALE);
      }
      color = UI_Color_Yellow;
    }
  }

  /* 5 段杆的命名：左腿用 "ll0..ll4"，右腿用 "rl0..rl4" */
  const char *name_table[2][5] = {
      {"ll0", "ll1", "ll2", "ll3", "ll4"},
      {"rl0", "rl1", "rl2", "rl3", "rl4"},
  };
  /* 端点连线：0-1, 1-2, 2-3, 3-4, 4-0 */
  const uint8_t edges[5][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}};

  for (uint8_t i = 0; i < 5; i++) {
    UILineDraw(&UI_LegRods[ui_index][i], (char *)name_table[ui_index][i], operate, UI_LEG_LAYER, color,
               UI_LEG_ROD_WIDTH, px[edges[i][0]], py[edges[i][0]], px[edges[i][1]], py[edges[i][1]]);
  }
  UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_LegRods[ui_index][0], UI_LegRods[ui_index][1],
                 UI_LegRods[ui_index][2], UI_LegRods[ui_index][3], UI_LegRods[ui_index][4]);
}

static void DrawLegLabels(uint32_t operate) {
  UICharDraw(&UI_LegLabel[0], "llb", operate, UI_LEG_LAYER, UI_Color_White, UI_LEG_LABEL_FONT_SIZE, UI_LEG_LABEL_WIDTH,
             UI_LEG_LABEL_X, UI_LEG_TOP_BASE_Y + UI_LEG_LABEL_OFFSET_Y, "LEFT");
  UICharRefresh(&referee_recv_info->referee_id, UI_LegLabel[0]);

  UICharDraw(&UI_LegLabel[1], "rlb", operate, UI_LEG_LAYER, UI_Color_White, UI_LEG_LABEL_FONT_SIZE, UI_LEG_LABEL_WIDTH,
             UI_LEG_LABEL_X, UI_LEG_BASE_Y + UI_LEG_LABEL_OFFSET_Y, "RIGHT");
  UICharRefresh(&referee_recv_info->referee_id, UI_LegLabel[1]);
}

static void DrawLegPosture(RobotInstance *robot, uint32_t operate) {
  const LegInstance *left_leg = NULL;
  const LegInstance *right_leg = NULL;

  if (robot->chassis) {
    left_leg = robot->chassis->leg[1];
    right_leg = robot->chassis->leg[0];
  }
  DrawSingleLegPosture(left_leg, 0, UI_LEG_TOP_BASE_Y, operate);
  DrawSingleLegPosture(right_leg, 1, UI_LEG_BASE_Y, operate);
}

/* ===========================================================================
 * 模块：Status —— 状态文字（标签 + 值）
 * =========================================================================*/

/** @brief 添加 6 行固定标签（仅 Init 时调用）。 */
static void DrawStatusStatic(uint32_t operate) {
  static const char *labels[UI_STATUS_ROW_COUNT] = {
      "ROBOT:", "CHASSIS:", "GIMBAL:", "FRICTION:", "LOADER:", "SUPERCAP:"};

  for (uint8_t i = 0; i < UI_STATUS_ROW_COUNT; i++) {
    char name[4];
    MakeUiName(name, 'u', i);
    UICharDraw(&UI_StatusLabel[i], name, operate, UI_STATUS_LAYER, UI_Color_Purplish_red, UI_STATUS_FONT_SIZE,
               UI_STATUS_WIDTH, UI_STATUS_LABEL_X, UI_STATUS_BASE_Y - i * UI_STATUS_ROW_GAP, labels[i]);
    UICharRefresh(&referee_recv_info->referee_id, UI_StatusLabel[i]);
  }
}

/** @brief 把第 i 行状态值写到 UI_StatusValue[i] 并发送。 */
static void DrawStatusValue(uint8_t row, uint32_t operate, const char *value_str) {
  char name[4];
  MakeUiName(name, 'v', row);
  UICharDraw(&UI_StatusValue[row], name, operate, UI_STATUS_LAYER, UI_Color_Green, UI_STATUS_FONT_SIZE, UI_STATUS_WIDTH,
             UI_STATUS_VALUE_X, UI_STATUS_BASE_Y - row * UI_STATUS_ROW_GAP, "%-*s", UI_STATUS_VALUE_CHARS, value_str);
  UICharRefresh(&referee_recv_info->referee_id, UI_StatusValue[row]);
}

/** @brief 刷新 6 行动态状态值。 */
static void DrawStatusDynamic(Referee_Interactive_info_t *data, uint32_t operate) {
  DrawStatusValue(0, operate, RobotModeStr(data->robot_mode));
  DrawStatusValue(1, operate, ChassisModeStr(data->chassis_mode));
  DrawStatusValue(2, operate, GimbalModeStr(data->gimbal_mode));
  DrawStatusValue(3, operate, FrictionModeStr(data->friction_mode));
  DrawStatusValue(4, operate, LoaderModeStr(data->loader_mode));
  DrawStatusValue(5, operate, SuperCapModeStr(data->super_cap_mode));
}

/* ===========================================================================
 * 模块：Cap —— 超级电容
 * =========================================================================*/

/** @brief 根据电压/错误状态决定电容弧的颜色。 */
static uint32_t CapArcColor(const Referee_Interactive_info_t *data) {
  if (data->cap_error != 0 || data->cap_voltage <= 0.0f) {
    return UI_Color_Black;
  }
  if (data->cap_voltage > 18.0f) return UI_Color_Green;
  if (data->cap_voltage > 15.0f) return UI_Color_Yellow;
  return UI_Color_Orange;
}

static void DrawCapStatic(uint32_t operate) {
  UICharDraw(&UI_CapTextE, "ce0", operate, UI_CAP_LAYER, UI_Color_White, UI_CAP_TEXT_SIZE, UI_CAP_TEXT_WIDTH,
             UI_CAP_E_X, UI_CAP_E_Y, "E");
  UICharRefresh(&referee_recv_info->referee_id, UI_CapTextE);

  UICharDraw(&UI_CapTextF, "cf0", operate, UI_CAP_LAYER, UI_Color_White, UI_CAP_TEXT_SIZE, UI_CAP_TEXT_WIDTH,
             UI_CAP_F_X, UI_CAP_F_Y, "F");
  UICharRefresh(&referee_recv_info->referee_id, UI_CapTextF);
}

static void DrawCapDynamic(const Referee_Interactive_info_t *data, uint32_t operate) {
  /* 电压映射成弧的扫角 */
  float sweep =
      (data->cap_voltage - UI_CAP_EMPTY_VOLTAGE) / (UI_CAP_FULL_VOLTAGE - UI_CAP_EMPTY_VOLTAGE) * UI_CAP_MAX_SWEEP;
  if (sweep < 1.0f) sweep = 1.0f;
  if (sweep > UI_CAP_MAX_SWEEP) sweep = UI_CAP_MAX_SWEEP;

  int32_t voltage_x10 = (int32_t)(data->cap_voltage * 10.0f + 0.5f);
  if (voltage_x10 < 0) voltage_x10 = 0;

  /* 电量弧 */
  UIArcDraw(&UI_CapArc, "cp0", operate, UI_CAP_LAYER, CapArcColor(data), UI_CAP_START_ANGLE,
            UI_CAP_START_ANGLE + (uint32_t)sweep, UI_CAP_WIDTH, UI_CAP_CENTER_X, UI_CAP_CENTER_Y, UI_CAP_RADIUS_X,
            UI_CAP_RADIUS_Y);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_CapArc);

  /* 电压数字 */
  UICharDraw(&UI_CapVoltage, "cv0", operate, UI_CAP_LAYER, UI_Color_Cyan, UI_CAP_TEXT_SIZE, UI_CAP_TEXT_WIDTH,
             UI_CAP_VOLTAGE_X, UI_CAP_VOLTAGE_Y, "%2d.%dV   ", (int)(voltage_x10 / 10), (int)(voltage_x10 % 10));
  UICharRefresh(&referee_recv_info->referee_id, UI_CapVoltage);

  /* 控制命令文字（NORMAL / BOOST） */
  UICharDraw(&UI_CapCtrlCmd, "cc0", operate, UI_CAP_LAYER, SuperCapCtrlCmdColor(data->super_cap_ctrl_cmd),
             UI_CAP_CTRL_TEXT_SIZE, UI_CAP_CTRL_TEXT_WIDTH, UI_CAP_CTRL_X, UI_CAP_CTRL_Y, "%-6s",
             SuperCapCtrlCmdStr(data->super_cap_ctrl_cmd));
  UICharRefresh(&referee_recv_info->referee_id, UI_CapCtrlCmd);
}

/* ===========================================================================
 * 模块：Speed —— 底盘速度
 * =========================================================================*/

static void DrawSpeedDynamic(const Referee_Interactive_info_t *data, uint32_t operate) {
  const float max_speed = SpeedMax(data->speed_is_prostrate);
  const uint32_t color = SpeedArcColor(data);

  float sweep = AbsFloat(data->speed) / max_speed * UI_SPEED_MAX_SWEEP;
  if (sweep < 1.0f) sweep = 1.0f;
  if (sweep > UI_SPEED_MAX_SWEEP) sweep = UI_SPEED_MAX_SWEEP;

  int32_t speed_x10 = (int32_t)(data->speed * 10.0f + (data->speed >= 0.0f ? 0.5f : -0.5f));
  int32_t abs_speed_x10 = speed_x10 >= 0 ? speed_x10 : -speed_x10;
  const char sign = speed_x10 < 0 ? '-' : ' ';

  /* 速度弧 */
  UIArcDraw(&UI_SpeedArc, "sp0", operate, UI_SPEED_LAYER, color, UI_SPEED_CENTER_ANGLE - (uint32_t)sweep,
            UI_SPEED_CENTER_ANGLE, UI_SPEED_WIDTH, UI_SPEED_CENTER_X, UI_SPEED_CENTER_Y, UI_SPEED_RADIUS_X,
            UI_SPEED_RADIUS_Y);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_SpeedArc);

  /* 速度数字 */
  UICharDraw(&UI_SpeedValue, "sv0", operate, UI_SPEED_LAYER, color, UI_SPEED_TEXT_SIZE, UI_SPEED_TEXT_WIDTH,
             UI_SPEED_TEXT_X, UI_SPEED_TEXT_Y, "%c%d.%dm/s  ", sign, (int)(abs_speed_x10 / 10),
             (int)(abs_speed_x10 % 10));
  UICharRefresh(&referee_recv_info->referee_id, UI_SpeedValue);
}

/* ===========================================================================
 * 脏标志检测：根据当前数据与上一次数据的差异，决定本帧需要刷新哪些 UI
 * =========================================================================*/

static void UIChangeCheck(Referee_Interactive_info_t *data) {
  /* 相对角度：原文件以"任何不等"作为变化条件，这里保持一致 */
  if (data->chassis_relative_angle != data->last_chassis_relative_angle) {
    data->UI_Interactive_Flag.relative_flag = 1;
    data->last_chassis_relative_angle = data->chassis_relative_angle;
  }

  /* 腿部：只要有效就持续刷新（数据连续变化） */
  if (data->leg_valid != data->last_leg_valid || data->leg_valid) {
    data->UI_Interactive_Flag.leg_flag = 1;
    data->last_leg_valid = data->leg_valid;
    data->last_leg_phi1 = data->leg_phi1;
    data->last_leg_phi2 = data->leg_phi2;
    data->last_leg_phi3 = data->leg_phi3;
    data->last_leg_phi4 = data->leg_phi4;
  }

  /* 各模式状态：任一变化即重绘整组状态文字 */
  uint8_t status_changed =
      data->robot_mode != data->last_robot_mode || data->chassis_mode != data->last_chassis_mode ||
      data->gimbal_mode != data->last_gimbal_mode || data->friction_mode != data->last_friction_mode ||
      data->loader_mode != data->last_loader_mode || data->super_cap_mode != data->last_super_cap_mode;
  if (status_changed) {
    data->UI_Interactive_Flag.status_flag = 1;
    data->last_robot_mode = data->robot_mode;
    data->last_chassis_mode = data->chassis_mode;
    data->last_gimbal_mode = data->gimbal_mode;
    data->last_friction_mode = data->friction_mode;
    data->last_loader_mode = data->loader_mode;
    data->last_super_cap_mode = data->super_cap_mode;
  }

  /* 电容：电压抖动 0.1V 以上、错误位变化、控制命令变化 */
  if (fabsf(data->cap_voltage - data->last_cap_voltage) > 0.1f || data->cap_error != data->last_cap_error ||
      data->super_cap_ctrl_cmd != data->last_super_cap_ctrl_cmd) {
    data->UI_Interactive_Flag.cap_flag = 1;
    data->last_cap_voltage = data->cap_voltage;
    data->last_cap_error = data->cap_error;
    data->last_super_cap_ctrl_cmd = data->super_cap_ctrl_cmd;
  }

  /* 速度：变化大于 0.05 m/s，或匍匐/腿模式切换 */
  if (fabsf(data->speed - data->last_speed) > 0.05f || data->speed_is_prostrate != data->last_speed_is_prostrate) {
    data->UI_Interactive_Flag.speed_flag = 1;
    data->last_speed = data->speed;
    data->last_speed_is_prostrate = data->speed_is_prostrate;
  }

  /* 瞄准：是否锁定目标 */
  if (data->aim_target_flag != data->last_aim_target_flag) {
    data->UI_Interactive_Flag.aim_flag = 1;
    data->last_aim_target_flag = data->aim_target_flag;
  }
}

/* ===========================================================================
 * 主刷新：按脏标志重绘各模块
 * =========================================================================*/

static void MyUIRefresh(RobotInstance *robot, Referee_Interactive_info_t *data) {
  if (data->UI_Interactive_Flag.relative_flag) {
    DrawRelativePosition(data->chassis_relative_angle, UI_Graph_Change);
    data->UI_Interactive_Flag.relative_flag = 0;
  }
  if (data->UI_Interactive_Flag.leg_flag) {
    DrawLegPosture(robot, UI_Graph_Change);
    data->UI_Interactive_Flag.leg_flag = 0;
  }
  if (data->UI_Interactive_Flag.status_flag) {
    DrawStatusDynamic(data, UI_Graph_Change);
    data->UI_Interactive_Flag.status_flag = 0;
  }
  if (data->UI_Interactive_Flag.cap_flag) {
    DrawCapDynamic(data, UI_Graph_Change);
    data->UI_Interactive_Flag.cap_flag = 0;
  }
  if (data->UI_Interactive_Flag.speed_flag) {
    DrawSpeedDynamic(data, UI_Graph_Change);
    data->UI_Interactive_Flag.speed_flag = 0;
  }
  if (data->UI_Interactive_Flag.aim_flag) {
    DrawAimIndicator(data->aim_target_flag, UI_Graph_Change);
    data->UI_Interactive_Flag.aim_flag = 0;
  }
}

/* ===========================================================================
 * 缓存初始化辅助：把 last_* 与当前值同步，避免初始化后立即触发不必要的 CHANGE
 * =========================================================================*/

static void SyncLastValues(Referee_Interactive_info_t *d) {
  d->last_chassis_relative_angle = d->chassis_relative_angle;

  d->last_leg_valid = d->leg_valid;
  d->last_leg_phi1 = d->leg_phi1;
  d->last_leg_phi2 = d->leg_phi2;
  d->last_leg_phi3 = d->leg_phi3;
  d->last_leg_phi4 = d->leg_phi4;

  d->last_robot_mode = d->robot_mode;
  d->last_chassis_mode = d->chassis_mode;
  d->last_gimbal_mode = d->gimbal_mode;
  d->last_friction_mode = d->friction_mode;
  d->last_loader_mode = d->loader_mode;
  d->last_super_cap_mode = d->super_cap_mode;
  d->last_super_cap_ctrl_cmd = d->super_cap_ctrl_cmd;

  d->last_cap_voltage = d->cap_voltage;
  d->last_cap_error = d->cap_error;
  d->last_speed = d->speed;
  d->last_speed_is_prostrate = d->speed_is_prostrate;
  d->last_aim_target_flag = d->aim_target_flag;
}

/* ===========================================================================
 * 公有 API
 * =========================================================================*/

/**
 * @brief 全量初始化（清空 + ADD 所有图形）。
 *
 * 启动后、客户端重连后、强制刷新时调用。
 */
void MyUIInit(RobotInstance *robot) {
  /* 重新缓存裁判系统数据指针 */
  referee_recv_info = robot->referee_data;

  /* 等待裁判系统下发有效机器人 ID */
  while (referee_recv_info->GameRobotState.robot_id == 0) {
    osDelay(100);
  }

  /* 配置发送方/客户端 ID，并清空客户端旧 UI */
  DeterminRobotID();
  UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0);
  UI_AimCrossVisible = 0;

  /* 采样并同步 last_* 值，再 ADD 全部图形 */
  interactive_data.chassis_relative_angle = GetRelativeAngle(robot);
  SampleLegPosture(robot, &interactive_data);
  SampleStatusData(robot, &interactive_data);
  SyncLastValues(&interactive_data);

  DrawRelativePosition(interactive_data.chassis_relative_angle, UI_Graph_ADD);
  DrawAimIndicator(interactive_data.aim_target_flag, UI_Graph_ADD);
  DrawLegLabels(UI_Graph_ADD);
  DrawLegPosture(robot, UI_Graph_ADD);
  DrawStatusStatic(UI_Graph_ADD);
  DrawStatusDynamic(&interactive_data, UI_Graph_ADD);
  DrawCapStatic(UI_Graph_ADD);
  DrawCapDynamic(&interactive_data, UI_Graph_ADD);
  DrawSpeedDynamic(&interactive_data, UI_Graph_ADD);
}

/**
 * @brief 暴露内部交互状态，便于其他模块请求强制刷新或读取当前 UI 数据。
 */
Referee_Interactive_info_t *getUI(void) { return &interactive_data; }

/**
 * @brief 周期性 UI 任务（建议每 UI_TASK_PERIOD_MS 调用一次）。
 *
 * 工作流程：
 *  1. 必要时执行强制全量刷新（手动或周期触发）。
 *  2. 采样最新数据。
 *  3. 周期触发部分模块的强制重绘（防止客户端图形丢失）。
 *  4. 比较脏标志，仅刷新发生变化的模块。
 */
void UITask(RobotInstance *robot) {
  referee_recv_info = robot->referee_data;

  /* ---------- 1. 全量刷新判断 ---------- */
  static uint16_t auto_refresh_counter = 0;
  uint8_t need_full_refresh = (interactive_data.force_refresh_ui == 1);

  if (++auto_refresh_counter >= UI_AUTO_REFRESH_INTERVAL_TICKS) {
    auto_refresh_counter = 0;
    need_full_refresh = 1;
  }
  if (need_full_refresh) {
    MyUIInit(robot);
    interactive_data.force_refresh_ui = 0;
    auto_refresh_counter = 0;
    /* 全量刷新已经画过最新状态，此处直接返回也可，
       但为了保留原行为（继续往下采样并按脏标志刷新），不直接 return。 */
  }

  /* ---------- 2. 采样最新数据 ---------- */
  interactive_data.chassis_relative_angle = GetRelativeAngle(robot);
  SampleLegPosture(robot, &interactive_data);
  SampleStatusData(robot, &interactive_data);

  /* ---------- 3. 周期性强制脏标志 ---------- */
  static uint16_t leg_refresh_counter = 0;
  if (++leg_refresh_counter >= UI_LEG_REFRESH_INTERVAL_TICKS) {
    leg_refresh_counter = 0;
    interactive_data.UI_Interactive_Flag.leg_flag = 1;
  }

  static uint16_t status_refresh_counter = 0;
  if (++status_refresh_counter >= UI_STATUS_REFRESH_INTERVAL_TICKS) {
    status_refresh_counter = 0;
    interactive_data.UI_Interactive_Flag.status_flag = 1;
    interactive_data.UI_Interactive_Flag.cap_flag = 1;
    interactive_data.UI_Interactive_Flag.speed_flag = 1;
    interactive_data.UI_Interactive_Flag.aim_flag = 1;
  }

  /* ---------- 4. 比较 + 刷新 ---------- */
  UIChangeCheck(&interactive_data);
  MyUIRefresh(robot, &interactive_data);
}