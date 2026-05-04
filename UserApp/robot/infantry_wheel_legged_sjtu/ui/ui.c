#include "ui.h"

#include <math.h>

#include "cmsis_os.h"
#include "referee.h"
#include "referee_ui.h"
#include "robot_config.h"

/*
 * 本文件负责 infantry_wheel_legged_sjtu 机器人的自定义裁判系统 UI。
 * 当前绘制的主要内容是一个底盘相对角度指示器：
 * 白色圆环作为参考背景，绿色圆弧表示底盘相对云台/车体参考方向的角度。
 *
 * 裁判系统 UI 的坐标和角度约定：
 * - 屏幕中心点在 ui.h 中定义。
 * - UI 圆弧接口使用 [0, 360] 范围内的角度值。
 * - 机器人侧的角度会先转换到屏幕坐标系，使 0 度显示在圆环正上方。
 */
#define UI_GRAPH_LAYER 7                  // Layer for chassis relative-angle indicator.
#define UI_RELATIVE_CENTER_X 960          // Relative-angle circle center x.
#define UI_RELATIVE_CENTER_Y 540          // Relative-angle circle center y.
#define UI_RELATIVE_RADIUS 88             // Relative-angle circle radius.
#define UI_RELATIVE_ARC_HALF_ANGLE 16.0f  // Half width of the relative-angle arc, in degrees.
#define UI_RELATIVE_RING_WIDTH 3          // Background circle line width.
#define UI_RELATIVE_ARC_WIDTH 8           // Relative-angle arc line width.
#define UI_TASK_PERIOD_MS 30              // Expected UI task period.
#define UI_AIM_LAYER 8                    // Layer for center aim indicator.
#define UI_AIM_CROSS_HALF_LEN 18          // Half length of center X cross.
#define UI_AIM_CROSS_WIDTH 3              // Center X cross line width.
#define UI_AIM_RECT_HALF_W 300            // Half width of center target rectangle.
#define UI_AIM_RECT_HALF_H 200            // Half height of center target rectangle.
#define UI_AIM_RECT_WIDTH 2               // Center target rectangle line width.
#define UI_AUTO_REFRESH_PERIOD_MS 10000   // 10秒刷新一次UI
// Auto refresh period converted from milliseconds to UITask ticks.
#define UI_AUTO_REFRESH_INTERVAL_TICKS ((UI_AUTO_REFRESH_PERIOD_MS + UI_TASK_PERIOD_MS - 1) / UI_TASK_PERIOD_MS)
#define UI_LEG_LAYER 7                 // Layer for five-link leg posture.
#define UI_LEG_BASE_X 1750             // Leg drawing origin x.
#define UI_LEG_BASE_Y 480              // Leg drawing origin y.
#define UI_LEG_TOP_BASE_Y 700         // Upper leg drawing origin y.
#define UI_LEG_LABEL_X 1600            // Leg label x.
#define UI_LEG_LABEL_OFFSET_Y 20       // Leg label y offset from drawing origin.
#define UI_LEG_LABEL_FONT_SIZE 15      // Leg label text size.
#define UI_LEG_LABEL_WIDTH 2           // Leg label stroke width.
#define UI_LEG_SCALE 400.0f            // Meter-to-pixel scale for leg drawing.
#define UI_LEG_ROD_WIDTH 5             // Leg rod line width.
#define UI_LEG_CHANGE_THRESHOLD 0.02f  // Reserved posture change threshold.
#define UI_LEG_REFRESH_PERIOD_MS 100   // Forced leg redraw period.
// Leg refresh period converted from milliseconds to UITask ticks.
#define UI_LEG_REFRESH_INTERVAL_TICKS ((UI_LEG_REFRESH_PERIOD_MS + UI_TASK_PERIOD_MS - 1) / UI_TASK_PERIOD_MS)
#define UI_STATUS_LAYER 8                // Layer for status text.
#define UI_STATUS_LABEL_X 90             // Status label x.
#define UI_STATUS_VALUE_X 245            // Status value x.
#define UI_STATUS_BASE_Y 850             // First status row y.
#define UI_STATUS_ROW_GAP 38             // Vertical gap between status rows.
#define UI_STATUS_FONT_SIZE 14           // Status text size.
#define UI_STATUS_WIDTH 2                // Status text stroke width.
#define UI_STATUS_ROW_COUNT 6            // ROBOT, CHASSIS, GIMBAL, FRICTION, LOADER, SUPERCAP.
#define UI_STATUS_VALUE_CHARS 12         // Fixed value width; clears old longer strings.
#define UI_STATUS_REFRESH_PERIOD_MS 300  // Forced status redraw period.
// Status refresh period converted from milliseconds to UITask ticks.
#define UI_STATUS_REFRESH_INTERVAL_TICKS ((UI_STATUS_REFRESH_PERIOD_MS + UI_TASK_PERIOD_MS - 1) / UI_TASK_PERIOD_MS)
#define UI_CAP_LAYER 6                         // Layer for super capacitor arc.
#define UI_CAP_CENTER_X 960                    // Super capacitor arc center x.
#define UI_CAP_CENTER_Y 540                    // Super capacitor arc center y.
#define UI_CAP_RADIUS_X 370                    // Super capacitor arc x radius.
#define UI_CAP_RADIUS_Y 370                    // Super capacitor arc y radius.
#define UI_CAP_START_ANGLE 270                 // Empty capacitor arc angle.
#define UI_CAP_MAX_SWEEP 40.0f                 // Full capacitor arc sweep, in degrees.
#define UI_CAP_WIDTH 22                        // Super capacitor arc line width.
#define UI_CAP_EMPTY_VOLTAGE 14.0f             // Voltage displayed as empty.
#define UI_CAP_FULL_VOLTAGE 23.0f              // Voltage displayed as full.
#define UI_CAP_TEXT_SIZE 17                    // E/F text size.
#define UI_CAP_TEXT_WIDTH 2                    // E/F text stroke width.
#define UI_CAP_E_X 610                         // E label x.
#define UI_CAP_E_Y 545                         // E label y.
#define UI_CAP_F_X 702                         // F label x.
#define UI_CAP_F_Y 775                         // F label y.
#define UI_CAP_VOLTAGE_X 480                   // Voltage text x, left of capacitor arc.
#define UI_CAP_VOLTAGE_Y 660                   // Voltage text y, left of capacitor arc.
#define UI_CAP_CTRL_X UI_CAP_VOLTAGE_X         // Super capacitor command text x.
#define UI_CAP_CTRL_Y 700                      // Super capacitor command text y.
#define UI_CAP_CTRL_TEXT_SIZE UI_CAP_TEXT_SIZE // Super capacitor command text size.
#define UI_CAP_CTRL_TEXT_WIDTH UI_CAP_TEXT_WIDTH // Super capacitor command text stroke width.
#define UI_SPEED_LAYER 6                       // Layer for speed arc.
#define UI_SPEED_CENTER_X UI_CAP_CENTER_X      // Speed arc center x.
#define UI_SPEED_CENTER_Y UI_CAP_CENTER_Y      // Speed arc center y.
#define UI_SPEED_RADIUS_X UI_CAP_RADIUS_X      // Speed arc x radius.
#define UI_SPEED_RADIUS_Y UI_CAP_RADIUS_Y      // Speed arc y radius.
#define UI_SPEED_CENTER_ANGLE 270              // Shared bottom angle of speed arc.
#define UI_SPEED_MAX_SWEEP UI_CAP_MAX_SWEEP    // Full speed arc sweep, in degrees.
#define UI_SPEED_WIDTH UI_CAP_WIDTH            // Speed arc line width.
#define UI_SPEED_LEG_MAX 2.5f                  // Leg-mode full speed, m/s.
#define UI_SPEED_PROSTRATE_MAX 3.0f            // Prostrate-mode full observed speed, m/s.
#define UI_SPEED_WARN_RATIO 0.85f              // Red when speed exceeds this ratio.
#define UI_SPEED_TEXT_X 480                   // Speed text x, right of speed arc.
#define UI_SPEED_TEXT_Y 420                    // Speed text y, right of speed arc.
#define UI_SPEED_TEXT_SIZE 15    // Speed text size.
#define UI_SPEED_TEXT_WIDTH UI_CAP_TEXT_WIDTH  // Speed text stroke width.
int32_t watch_data1[5], watch_data2[5];

/* 缓存 RobotInstance 中的裁判系统数据指针，便于本文件各函数直接访问。 */
static referee_info_t *referee_recv_info;

/*
 * 本文件维护的 UI 交互状态。
 * 其中保存了当前需要显示的数据，以及用于判断本轮 UITask 是否需要刷新图形的脏标志。
 */
static Referee_Interactive_info_t interactive_data;

/*
 * UI 通信序号的预留变量。
 * 当前文件暂时没有使用它，但项目内其他 UI 模块通常会保留一个全局 UI 序号计数。
 */
uint8_t UI_Seq;

/*
 * 图形描述符使用 static 保存，保证多次函数调用之间内存一直有效。
 * UICircleDraw() / UIArcDraw() 会先填充这些结构体，
 * 然后再通过 UIGraphRefresh() 发送给客户端。
 */
static Graph_Data_t UI_RelativeRing;
static Graph_Data_t UI_RelativeArc[2];
static Graph_Data_t UI_LegRods[2][5];
static String_Data_t UI_LegLabel[2];
static Graph_Data_t UI_AimCross[2];
static Graph_Data_t UI_AimRect;
static uint8_t UI_AimCrossVisible;
static Graph_Data_t UI_CapArc;
static Graph_Data_t UI_SpeedArc;
static String_Data_t UI_CapTextE;
static String_Data_t UI_CapTextF;
static String_Data_t UI_CapVoltage;
static String_Data_t UI_CapCtrlCmd;
static String_Data_t UI_SpeedValue;
static String_Data_t UI_StatusLabel[UI_STATUS_ROW_COUNT];
static String_Data_t UI_StatusValue[UI_STATUS_ROW_COUNT];

static void MakeUiName(char name[4], char prefix, uint8_t index) {
  name[0] = prefix;
  name[1] = (char)('0' + index / 10u);
  name[2] = (char)('0' + index % 10u);
  name[3] = '\0';
}

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

static const char *SuperCapCtrlCmdStr(SuperCap_Ctrl_Cmd_e cmd) {
  return cmd == BOOST ? "BOOST" : "NORMAL";
}

static uint32_t SuperCapCtrlCmdColor(SuperCap_Ctrl_Cmd_e cmd) {
  return cmd == BOOST ? UI_Color_Purplish_red : UI_Color_Cyan;
}

static uint8_t IsProstrateMode(Robot_Mode_e robot_mode, Chassis_Mode_e chassis_mode) {
  return chassis_mode == CHASSIS_PROSTRATE || robot_mode == ROBOT_CHASSIS_PROSTRATE_ROTATE ||
         robot_mode == ROBOT_CHASSIS_PROSTRATE_FOLLOW || robot_mode == ROBOT_CHASSIS_PROSTRATE_FREE;
}

static float AbsFloat(float value) { return value >= 0.0f ? value : -value; }

static float SpeedMax(uint8_t is_prostrate) { return is_prostrate ? UI_SPEED_PROSTRATE_MAX : UI_SPEED_LEG_MAX; }

static uint32_t SpeedArcColor(const Referee_Interactive_info_t *data) {
  const float max_speed = SpeedMax(data->speed_is_prostrate);
  return AbsFloat(data->speed) >= max_speed * UI_SPEED_WARN_RATIO ? UI_Color_Purplish_red : UI_Color_Cyan;
}

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

static void DeterminRobotID(void) {
  /*
   * 客户端 ID 和发送方 ID 依赖裁判系统下发的机器人 ID。
   * 红方机器人 ID 通常为 1..7，蓝方机器人 ID 大于 7。
   * 客户端 ID 按标准规则计算：0x0100 + robot_id。
   */
  referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
  referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
  referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID;
  referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static float NormalizeAngle(float angle) {
  /*
   * 将所有 UI 角度归一化到 [0, 360) 半开区间。
   * 这样 -10 度、370 度等越界角度在传入裁判系统绘图接口前都会被修正。
   */
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

static uint32_t AngleToUiDegree(float angle) {
  /*
   * 绘图接口使用整数角度。
   * 强制类型转换前加 0.5f，用于四舍五入到最近的整数角度，而不是简单截断小数。
   */
  uint32_t degree = (uint32_t)(NormalizeAngle(angle) + 0.5f);

  /*
   * 359.6 度四舍五入后可能变成 360。
   * 这里再把 360 映射回 0，保证返回值始终位于 [0, 359]。
   */
  return degree >= 360 ? degree - 360 : degree;
}

static float GetRelativeAngle(RobotInstance *robot) {
  /*
   * 默认使用 robot->offset_angle。
   * 在 ONE_BOARD 单板构建下，没有独立底盘板上传的数据包，因此只能使用该来源。
   */
  float angle = robot->offset_angle;
#if !defined(ONE_BOARD)
  /*
   * 在双板构建下，如果底盘板数据可用，则优先使用底盘板上报的 UI 角度。
   * 该字段放大了 10 倍保存，例如 123 表示 12.3 度。
   */
  if (robot->chassis_fetch_data) {
    angle = (float)robot->chassis_fetch_data->ui_status.ui_chassis_relative_angle_deg_x10 * 0.1f;
  }
#endif
  return angle;
}

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

static void SampleStatusData(RobotInstance *robot, Referee_Interactive_info_t *data) {
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

  if (robot == NULL) {
    return;
  }

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

static void DrawStatusStatic(uint32_t operate) {
  static const char *labels[UI_STATUS_ROW_COUNT] = {
      "ROBOT:", "CHASSIS:", "GIMBAL:", "FRICTION:", "LOADER:", "SUPERCAP:"};

  for (uint8_t i = 0; i < UI_STATUS_ROW_COUNT; i++) {
    char name[4];
    MakeUiName(name, 'u', i);
    UICharDraw(&UI_StatusLabel[i], name, operate, UI_STATUS_LAYER, UI_Color_Purplish_red, UI_STATUS_FONT_SIZE, UI_STATUS_WIDTH,
               UI_STATUS_LABEL_X, UI_STATUS_BASE_Y - i * UI_STATUS_ROW_GAP, labels[i]);
    UICharRefresh(&referee_recv_info->referee_id, UI_StatusLabel[i]);
  }
}

static void DrawStatusDynamic(Referee_Interactive_info_t *data, uint32_t operate) {
  char name[4];

  MakeUiName(name, 'v', 0);
  uint32_t row_y = UI_STATUS_BASE_Y;
  UICharDraw(&UI_StatusValue[0], name, operate, UI_STATUS_LAYER, UI_Color_Green, UI_STATUS_FONT_SIZE, UI_STATUS_WIDTH,
             UI_STATUS_VALUE_X, row_y, "%-*s", UI_STATUS_VALUE_CHARS, RobotModeStr(data->robot_mode));
  UICharRefresh(&referee_recv_info->referee_id, UI_StatusValue[0]);

  MakeUiName(name, 'v', 1);
  row_y = UI_STATUS_BASE_Y - UI_STATUS_ROW_GAP;
  UICharDraw(&UI_StatusValue[1], name, operate, UI_STATUS_LAYER, UI_Color_Green, UI_STATUS_FONT_SIZE, UI_STATUS_WIDTH,
             UI_STATUS_VALUE_X, row_y, "%-*s", UI_STATUS_VALUE_CHARS, ChassisModeStr(data->chassis_mode));
  UICharRefresh(&referee_recv_info->referee_id, UI_StatusValue[1]);

  MakeUiName(name, 'v', 2);
  row_y = UI_STATUS_BASE_Y - 2 * UI_STATUS_ROW_GAP;
  UICharDraw(&UI_StatusValue[2], name, operate, UI_STATUS_LAYER, UI_Color_Green, UI_STATUS_FONT_SIZE, UI_STATUS_WIDTH,
             UI_STATUS_VALUE_X, row_y, "%-*s", UI_STATUS_VALUE_CHARS, GimbalModeStr(data->gimbal_mode));
  UICharRefresh(&referee_recv_info->referee_id, UI_StatusValue[2]);

  MakeUiName(name, 'v', 3);
  row_y = UI_STATUS_BASE_Y - 3 * UI_STATUS_ROW_GAP;
  UICharDraw(&UI_StatusValue[3], name, operate, UI_STATUS_LAYER, UI_Color_Green, UI_STATUS_FONT_SIZE, UI_STATUS_WIDTH,
             UI_STATUS_VALUE_X, row_y, "%-*s", UI_STATUS_VALUE_CHARS, FrictionModeStr(data->friction_mode));
  UICharRefresh(&referee_recv_info->referee_id, UI_StatusValue[3]);

  MakeUiName(name, 'v', 4);
  row_y = UI_STATUS_BASE_Y - 4 * UI_STATUS_ROW_GAP;
  UICharDraw(&UI_StatusValue[4], name, operate, UI_STATUS_LAYER, UI_Color_Green, UI_STATUS_FONT_SIZE, UI_STATUS_WIDTH,
             UI_STATUS_VALUE_X, row_y, "%-*s", UI_STATUS_VALUE_CHARS, LoaderModeStr(data->loader_mode));
  UICharRefresh(&referee_recv_info->referee_id, UI_StatusValue[4]);

  MakeUiName(name, 'v', 5);
  row_y = UI_STATUS_BASE_Y - 5 * UI_STATUS_ROW_GAP;
  UICharDraw(&UI_StatusValue[5], name, operate, UI_STATUS_LAYER, UI_Color_Green, UI_STATUS_FONT_SIZE, UI_STATUS_WIDTH,
             UI_STATUS_VALUE_X, row_y, "%-*s", UI_STATUS_VALUE_CHARS, SuperCapModeStr(data->super_cap_mode));
  UICharRefresh(&referee_recv_info->referee_id, UI_StatusValue[5]);
}

static uint32_t CapArcColor(const Referee_Interactive_info_t *data) {
  if (data->cap_error != 0 || data->cap_voltage <= 0.0f) {
    return UI_Color_Black;
  }

  if (data->cap_voltage > 18.0f) {
    return UI_Color_Green;
  }
  if (data->cap_voltage > 15.0f) {
    return UI_Color_Yellow;
  }
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
  float sweep =
      (data->cap_voltage - UI_CAP_EMPTY_VOLTAGE) / (UI_CAP_FULL_VOLTAGE - UI_CAP_EMPTY_VOLTAGE) * UI_CAP_MAX_SWEEP;
  int32_t voltage_x10 = (int32_t)(data->cap_voltage * 10.0f + 0.5f);
  if (sweep < 1.0f) {
    sweep = 1.0f;
  }
  if (sweep > UI_CAP_MAX_SWEEP) {
    sweep = UI_CAP_MAX_SWEEP;
  }
  if (voltage_x10 < 0) {
    voltage_x10 = 0;
  }

  UIArcDraw(&UI_CapArc, "cp0", operate, UI_CAP_LAYER, CapArcColor(data), UI_CAP_START_ANGLE,
            UI_CAP_START_ANGLE + (uint32_t)sweep, UI_CAP_WIDTH, UI_CAP_CENTER_X, UI_CAP_CENTER_Y, UI_CAP_RADIUS_X,
            UI_CAP_RADIUS_Y);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_CapArc);

  UICharDraw(&UI_CapVoltage, "cv0", operate, UI_CAP_LAYER, UI_Color_Cyan, UI_CAP_TEXT_SIZE, UI_CAP_TEXT_WIDTH,
             UI_CAP_VOLTAGE_X, UI_CAP_VOLTAGE_Y, "%2d.%dV   ", (int)(voltage_x10 / 10), (int)(voltage_x10 % 10));
  UICharRefresh(&referee_recv_info->referee_id, UI_CapVoltage);

  UICharDraw(&UI_CapCtrlCmd, "cc0", operate, UI_CAP_LAYER, SuperCapCtrlCmdColor(data->super_cap_ctrl_cmd),
             UI_CAP_CTRL_TEXT_SIZE, UI_CAP_CTRL_TEXT_WIDTH, UI_CAP_CTRL_X, UI_CAP_CTRL_Y, "%-6s",
             SuperCapCtrlCmdStr(data->super_cap_ctrl_cmd));
  UICharRefresh(&referee_recv_info->referee_id, UI_CapCtrlCmd);
}

static void DrawSpeedDynamic(const Referee_Interactive_info_t *data, uint32_t operate) {
  const float max_speed = SpeedMax(data->speed_is_prostrate);
  const uint32_t color = SpeedArcColor(data);
  float sweep = AbsFloat(data->speed) / max_speed * UI_SPEED_MAX_SWEEP;
  int32_t speed_x10 = (int32_t)(data->speed * 10.0f + (data->speed >= 0.0f ? 0.5f : -0.5f));
  int32_t abs_speed_x10 = speed_x10 >= 0 ? speed_x10 : -speed_x10;
  const char sign = speed_x10 < 0 ? '-' : ' ';

  if (sweep < 1.0f) {
    sweep = 1.0f;
  }
  if (sweep > UI_SPEED_MAX_SWEEP) {
    sweep = UI_SPEED_MAX_SWEEP;
  }

  UIArcDraw(&UI_SpeedArc, "sp0", operate, UI_SPEED_LAYER, color, UI_SPEED_CENTER_ANGLE - (uint32_t)sweep,
            UI_SPEED_CENTER_ANGLE, UI_SPEED_WIDTH, UI_SPEED_CENTER_X, UI_SPEED_CENTER_Y, UI_SPEED_RADIUS_X,
            UI_SPEED_RADIUS_Y);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_SpeedArc);

  UICharDraw(&UI_SpeedValue, "sv0", operate, UI_SPEED_LAYER, color, UI_SPEED_TEXT_SIZE, UI_SPEED_TEXT_WIDTH,
             UI_SPEED_TEXT_X, UI_SPEED_TEXT_Y, "%c%d.%dm/s  ", sign, (int)(abs_speed_x10 / 10),
             (int)(abs_speed_x10 % 10));
  UICharRefresh(&referee_recv_info->referee_id, UI_SpeedValue);
}

static void DrawAimIndicator(uint8_t target_locked, uint32_t operate) {
  const uint32_t rect_color = target_locked ? UI_Color_Purplish_red : UI_Color_Yellow;
  const int32_t center_x = UI_CENTER_X;
  const int32_t center_y = UI_CENTER_Y;

  if (target_locked || UI_AimCrossVisible) {
    const uint32_t cross_operate = target_locked ? (UI_AimCrossVisible ? UI_Graph_Change : UI_Graph_ADD) : UI_Graph_Del;
    UILineDraw(&UI_AimCross[0], "ax0", cross_operate, UI_AIM_LAYER, UI_Color_Purplish_red, UI_AIM_CROSS_WIDTH,
               center_x - UI_AIM_CROSS_HALF_LEN, center_y - UI_AIM_CROSS_HALF_LEN,
               center_x + UI_AIM_CROSS_HALF_LEN, center_y + UI_AIM_CROSS_HALF_LEN);
    UILineDraw(&UI_AimCross[1], "ax1", cross_operate, UI_AIM_LAYER, UI_Color_Purplish_red, UI_AIM_CROSS_WIDTH,
               center_x - UI_AIM_CROSS_HALF_LEN, center_y + UI_AIM_CROSS_HALF_LEN,
               center_x + UI_AIM_CROSS_HALF_LEN, center_y - UI_AIM_CROSS_HALF_LEN);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_AimCross[0], UI_AimCross[1]);
    UI_AimCrossVisible = target_locked;
  }

  UIRectangleDraw(&UI_AimRect, "ar0", operate, UI_AIM_LAYER, rect_color, UI_AIM_RECT_WIDTH, center_x - UI_AIM_RECT_HALF_W,
                  center_y - UI_AIM_RECT_HALF_H, center_x + UI_AIM_RECT_HALF_W, center_y + UI_AIM_RECT_HALF_H);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_AimRect);
}

static void DrawSingleLegPosture(const LegInstance *leg, uint8_t ui_index, int32_t base_y, uint32_t operate) {
  int32_t point_x[5] = {UI_LEG_BASE_X, UI_LEG_BASE_X, UI_LEG_BASE_X, UI_LEG_BASE_X, UI_LEG_BASE_X};
  int32_t point_y[5] = {base_y, base_y, base_y, base_y, base_y};
  uint32_t color = UI_Color_Black;

  if (leg) {
    float model_x[5];
    float model_y[5];
    float leg_phi[4];

    if (CalculateLegPosture(leg, model_x, model_y, leg_phi)) {
      for (uint8_t i = 0; i < 5; i++) {
        point_x[i] = UI_LEG_BASE_X + (int32_t)(model_x[i] * UI_LEG_SCALE);
        point_y[i] = base_y - (int32_t)(model_y[i] * UI_LEG_SCALE);
      }
      color = UI_Color_Yellow;
    }
  }

  UILineDraw(&UI_LegRods[ui_index][0], ui_index == 0 ? "ll0" : "rl0", operate, UI_LEG_LAYER, color, UI_LEG_ROD_WIDTH,
             point_x[0], point_y[0], point_x[1], point_y[1]);
  UILineDraw(&UI_LegRods[ui_index][1], ui_index == 0 ? "ll1" : "rl1", operate, UI_LEG_LAYER, color, UI_LEG_ROD_WIDTH,
             point_x[1], point_y[1], point_x[2], point_y[2]);
  UILineDraw(&UI_LegRods[ui_index][2], ui_index == 0 ? "ll2" : "rl2", operate, UI_LEG_LAYER, color, UI_LEG_ROD_WIDTH,
             point_x[2], point_y[2], point_x[3], point_y[3]);
  UILineDraw(&UI_LegRods[ui_index][3], ui_index == 0 ? "ll3" : "rl3", operate, UI_LEG_LAYER, color, UI_LEG_ROD_WIDTH,
             point_x[3], point_y[3], point_x[4], point_y[4]);
  UILineDraw(&UI_LegRods[ui_index][4], ui_index == 0 ? "ll4" : "rl4", operate, UI_LEG_LAYER, color, UI_LEG_ROD_WIDTH,
             point_x[4], point_y[4], point_x[0], point_y[0]);
  UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_LegRods[ui_index][0], UI_LegRods[ui_index][1],
                 UI_LegRods[ui_index][2], UI_LegRods[ui_index][3], UI_LegRods[ui_index][4]);
}

static void DrawLegLabels(uint32_t operate) {
  UICharDraw(&UI_LegLabel[0], "llb", operate, UI_LEG_LAYER, UI_Color_White, UI_LEG_LABEL_FONT_SIZE,
             UI_LEG_LABEL_WIDTH, UI_LEG_LABEL_X, UI_LEG_TOP_BASE_Y + UI_LEG_LABEL_OFFSET_Y, "LEFT");
  UICharRefresh(&referee_recv_info->referee_id, UI_LegLabel[0]);

  UICharDraw(&UI_LegLabel[1], "rlb", operate, UI_LEG_LAYER, UI_Color_White, UI_LEG_LABEL_FONT_SIZE,
             UI_LEG_LABEL_WIDTH, UI_LEG_LABEL_X, UI_LEG_BASE_Y + UI_LEG_LABEL_OFFSET_Y, "RIGHT");
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

static void DrawRelativePosition(float offset_angle, uint32_t operate) {
  /*
   * 绘制一个白色圆环和一个绿色方向圆弧。
   * operate 会直接传给裁判系统 UI 辅助函数，
   * 调用者通过它决定本次是初始化时的 ADD 操作，还是运行中的 CHANGE 操作。
   */
  const int32_t center_x = UI_RELATIVE_CENTER_X;
  const int32_t center_y = UI_RELATIVE_CENTER_Y;

  /*
   * 将机器人侧的相对角度转换成 UI 坐标系角度。
   * 使用 90 度减去 offset_angle，是为了把机器人坐标系映射到屏幕显示习惯：
   * 圆环正上方作为视觉上的前方。
   */
  /*
   * The measured relative angle has the opposite sign from the referee UI's horizontal direction.
   * Negating it keeps front/back unchanged while fixing left/right mirroring.
   */
  const float ui_angle = NormalizeAngle(-offset_angle);

  /*
   * 绿色指示区域是一个以 ui_angle 为中心的对称圆弧。
   * UI_RELATIVE_ARC_HALF_ANGLE 控制这个方向扇区的一半宽度。
   */
  const float start_angle = NormalizeAngle(ui_angle - UI_RELATIVE_ARC_HALF_ANGLE);
  const float end_angle = NormalizeAngle(ui_angle + UI_RELATIVE_ARC_HALF_ANGLE);

  /* 绘制背景参考圆环。 */
  UICircleDraw(&UI_RelativeRing, "rg0", operate, UI_GRAPH_LAYER, UI_Color_White, UI_RELATIVE_RING_WIDTH, center_x,
               center_y, UI_RELATIVE_RADIUS);

  if (start_angle <= end_angle) {
    /*
     * 普通情况：圆弧没有跨过 0 度，可以用一段绿色圆弧直接表示。
     * 第二个图形发送一段很小的黑色圆弧，用于清除或隐藏上一帧可能存在的分裂圆弧。
     */
    UIArcDraw(&UI_RelativeArc[0], "sa0", operate, UI_GRAPH_LAYER, UI_Color_Green, AngleToUiDegree(start_angle),
              AngleToUiDegree(end_angle), UI_RELATIVE_ARC_WIDTH, center_x, center_y, UI_RELATIVE_RADIUS,
              UI_RELATIVE_RADIUS);
    UIArcDraw(&UI_RelativeArc[1], "sa1", operate, UI_GRAPH_LAYER, UI_Color_Black, 0, 1, UI_RELATIVE_ARC_WIDTH, center_x,
              center_y, UI_RELATIVE_RADIUS, UI_RELATIVE_RADIUS);
  } else {
    /*
     * 跨 0 度情况：目标圆弧跨过 0 度，例如 350..10。
     * 裁判系统圆弧接口无法用一个连续区间表达这种情况，
     * 因此拆成两段绘制：start..360 和 0..end。
     */
    UIArcDraw(&UI_RelativeArc[0], "sa0", operate, UI_GRAPH_LAYER, UI_Color_Green, AngleToUiDegree(start_angle), 360,
              UI_RELATIVE_ARC_WIDTH, center_x, center_y, UI_RELATIVE_RADIUS, UI_RELATIVE_RADIUS);
    UIArcDraw(&UI_RelativeArc[1], "sa1", operate, UI_GRAPH_LAYER, UI_Color_Green, 0, AngleToUiDegree(end_angle),
              UI_RELATIVE_ARC_WIDTH, center_x, center_y, UI_RELATIVE_RADIUS, UI_RELATIVE_RADIUS);
  }

  /*
   * 将圆环和圆弧发送到客户端。
   * 圆环和圆弧分开发送，是因为 UIGraphRefresh() 的第二个参数表示后面跟随的图形数量。
   */
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_RelativeRing);
  UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_RelativeArc[0], UI_RelativeArc[1]);
}

static void UIChangeCheck(Referee_Interactive_info_t *data) {
  /*
   * 避免因为角度微小抖动而频繁刷新裁判系统 UI。
   * 只有当前显示的底盘相对角度变化超过 1 度时，才请求重绘。
   */
  if (data->chassis_relative_angle != data->last_chassis_relative_angle) {
    data->UI_Interactive_Flag.relative_flag = 1;
    data->last_chassis_relative_angle = data->chassis_relative_angle;
  }

  if (data->leg_valid != data->last_leg_valid || data->leg_valid) {
    data->UI_Interactive_Flag.leg_flag = 1;
    data->last_leg_valid = data->leg_valid;
    data->last_leg_phi1 = data->leg_phi1;
    data->last_leg_phi2 = data->leg_phi2;
    data->last_leg_phi3 = data->leg_phi3;
    data->last_leg_phi4 = data->leg_phi4;
  }

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

  if (fabsf(data->cap_voltage - data->last_cap_voltage) > 0.1f || data->cap_error != data->last_cap_error ||
      data->super_cap_ctrl_cmd != data->last_super_cap_ctrl_cmd) {
    data->UI_Interactive_Flag.cap_flag = 1;
    data->last_cap_voltage = data->cap_voltage;
    data->last_cap_error = data->cap_error;
    data->last_super_cap_ctrl_cmd = data->super_cap_ctrl_cmd;
  }

  if (fabsf(data->speed - data->last_speed) > 0.05f || data->speed_is_prostrate != data->last_speed_is_prostrate) {
    data->UI_Interactive_Flag.speed_flag = 1;
    data->last_speed = data->speed;
    data->last_speed_is_prostrate = data->speed_is_prostrate;
  }

  if (data->aim_target_flag != data->last_aim_target_flag) {
    data->UI_Interactive_Flag.aim_flag = 1;
    data->last_aim_target_flag = data->aim_target_flag;
  }
}

static void MyUIRefresh(RobotInstance *robot, Referee_Interactive_info_t *data) {
  /*
   * 基于脏标志刷新 UI：只绘制发生变化的 UI 元素。
   * 这样可以减少裁判系统交互数据链路上的带宽占用。
   */
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

void MyUIInit(RobotInstance *robot) {
  /* 保存最新的裁判系统数据指针，防止 RobotInstance 重建后仍使用旧指针。 */
  referee_recv_info = robot->referee_data;

  /*
   * 启动后机器人 ID 不一定立即有效。
   * 必须等裁判系统上报非 0 ID 后，才能计算客户端 ID 并发送 UI 数据包。
   */
  while (referee_recv_info->GameRobotState.robot_id == 0) {
    osDelay(100);
  }

  /*
   * 配置发送方/客户端 ID，清空客户端上的旧图形，
   * 然后添加初始的相对位置指示器。
   */
  DeterminRobotID();
  UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0);
  UI_AimCrossVisible = 0;

  /*
   * 将当前角度和上一次角度初始化为同一个值。
   * 这样下面执行 ADD 操作后，UIChangeCheck() 不会立刻再触发一次多余的 CHANGE。
   */
  interactive_data.chassis_relative_angle = GetRelativeAngle(robot);
  interactive_data.last_chassis_relative_angle = interactive_data.chassis_relative_angle;
  SampleLegPosture(robot, &interactive_data);
  interactive_data.last_leg_valid = interactive_data.leg_valid;
  interactive_data.last_leg_phi1 = interactive_data.leg_phi1;
  interactive_data.last_leg_phi2 = interactive_data.leg_phi2;
  interactive_data.last_leg_phi3 = interactive_data.leg_phi3;
  interactive_data.last_leg_phi4 = interactive_data.leg_phi4;
  SampleStatusData(robot, &interactive_data);
  interactive_data.last_robot_mode = interactive_data.robot_mode;
  interactive_data.last_chassis_mode = interactive_data.chassis_mode;
  interactive_data.last_gimbal_mode = interactive_data.gimbal_mode;
  interactive_data.last_friction_mode = interactive_data.friction_mode;
  interactive_data.last_loader_mode = interactive_data.loader_mode;
  interactive_data.last_super_cap_mode = interactive_data.super_cap_mode;
  interactive_data.last_super_cap_ctrl_cmd = interactive_data.super_cap_ctrl_cmd;
  interactive_data.last_cap_voltage = interactive_data.cap_voltage;
  interactive_data.last_cap_error = interactive_data.cap_error;
  interactive_data.last_speed = interactive_data.speed;
  interactive_data.last_speed_is_prostrate = interactive_data.speed_is_prostrate;
  interactive_data.last_aim_target_flag = interactive_data.aim_target_flag;
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

/*
 * 对外暴露 UI 状态，其他模块可以通过该指针请求强制刷新，
 * 或查看当前的交互数据。
 */
Referee_Interactive_info_t *getUI(void) { return &interactive_data; }

void UITask(RobotInstance *robot) {
  /*
   * UITask 预期由 RTOS 任务周期性调用。
   * 每次调用都会采样最新机器人数据，检查 UI 是否需要更新，
   * 并只发送必要的图形变更。
   */
  referee_recv_info = robot->referee_data;

  static uint16_t auto_refresh_counter = 0;
  uint8_t need_full_refresh = interactive_data.force_refresh_ui == 1;

  if (++auto_refresh_counter >= UI_AUTO_REFRESH_INTERVAL_TICKS) {
    auto_refresh_counter = 0;
    need_full_refresh = 1;
  }

  if (need_full_refresh) {
    /*
     * 强制刷新会重新构建全部 UI 图形。
     * 适用于客户端重连、手动删除 UI、Ctrl 手动刷新、周期自动刷新，
     * 或者客户端图形状态可能和本地状态不一致的情况。
     */
    MyUIInit(robot);
    interactive_data.force_refresh_ui = 0;
    auto_refresh_counter = 0;
    need_full_refresh = 0;
  }

  /* 处理完可能的重新初始化后，再采样最新角度。 */
  interactive_data.chassis_relative_angle = GetRelativeAngle(robot);
  SampleLegPosture(robot, &interactive_data);
  SampleStatusData(robot, &interactive_data);

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

  /*
   * 先根据数据变化设置脏标志，再绘制所有脏 UI 元素。
   * 底盘相对角度和五连杆位姿都使用这套机制。
   */
  UIChangeCheck(&interactive_data);
  MyUIRefresh(robot, &interactive_data);
}
