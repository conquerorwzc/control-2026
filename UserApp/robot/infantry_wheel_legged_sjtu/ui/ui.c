/**
 * @file ui.c
 * @brief 轮腿步兵的裁判系统 UI 绘制与刷新。
 *
 * 本文件只做三件事：
 *  - 采样机器人状态，整理成 UI 所需的稳定快照；
 *  - 根据快照变化设置脏标志，避免无意义重绘；
 *  - 通过 referee_ui 接口增量刷新裁判客户端图形。
 *
 * 图形名必须稳定，图形描述符必须跨帧存活。因此所有图形描述符都
 * 保存在文件级 static 变量中，初始化时 ADD，运行时 CHANGE。
 *
 * 裁判客户端偶尔会丢图，本模块保留周期性全量重绘作为兜底。
 */

#include "ui.h"

#include <math.h>

#include "cmsis_os.h"
#include "referee.h"
#include "referee_ui.h"
#include "robot_config.h"

/* ===========================================================================
 * 布局与刷新节奏
 * =========================================================================*/

/* UI 任务按固定周期运行，所有刷新间隔都折算成 tick。 */
#define UI_TASK_PERIOD_MS 30
#define UI_AUTO_REFRESH_PERIOD_MS 10000  // 客户端丢图兜底
#define UI_LEG_REFRESH_PERIOD_MS 100     // 腿部姿态变化快，定期重发
#define UI_STATUS_REFRESH_PERIOD_MS 300  // 状态类图元定期保活

#define UI_AUTO_REFRESH_INTERVAL_TICKS ((UI_AUTO_REFRESH_PERIOD_MS + UI_TASK_PERIOD_MS - 1) / UI_TASK_PERIOD_MS)
#define UI_LEG_REFRESH_INTERVAL_TICKS ((UI_LEG_REFRESH_PERIOD_MS + UI_TASK_PERIOD_MS - 1) / UI_TASK_PERIOD_MS)
#define UI_STATUS_REFRESH_INTERVAL_TICKS ((UI_STATUS_REFRESH_PERIOD_MS + UI_TASK_PERIOD_MS - 1) / UI_TASK_PERIOD_MS)

/* 底盘相对云台角度：中心小圆环 + 方向弧。 */
#define UI_GRAPH_LAYER 7
#define UI_RELATIVE_CENTER_X 960
#define UI_RELATIVE_CENTER_Y 540
#define UI_RELATIVE_RADIUS 88
#define UI_RELATIVE_ARC_HALF_ANGLE 16.0f
#define UI_RELATIVE_RING_WIDTH 3
#define UI_RELATIVE_ARC_WIDTH 8

/* 中心瞄准框。锁定目标时额外显示十字。 */
#define UI_AIM_LAYER 8
#define UI_AIM_CROSS_HALF_LEN 18
#define UI_AIM_CROSS_WIDTH 3
#define UI_AIM_RECT_HALF_W 300
#define UI_AIM_RECT_HALF_H 200
#define UI_AIM_RECT_WIDTH 2

/* 五连杆腿部姿态，显示在画面右侧。 */
#define UI_LEG_LAYER 7
#define UI_LEG_BASE_X 1750
#define UI_LEG_BASE_Y 480
#define UI_LEG_TOP_BASE_Y 700
#define UI_LEG_LABEL_X 1600
#define UI_LEG_FRONT_LABEL_X 1810
#define UI_LEG_LABEL_OFFSET_Y 40
#define UI_LEG_LABEL_FONT_SIZE 15
#define UI_LEG_LABEL_WIDTH 2
#define UI_LEG_LENGTH_X 1640
#define UI_LEG_LENGTH_OFFSET_Y -105
#define UI_LEG_LENGTH_FONT_SIZE 14
#define UI_LEG_LENGTH_WIDTH 2
#define UI_LEG_SCALE 400.0f
#define UI_LEG_ROD_WIDTH 5

/* 左侧状态面板。 */
#define UI_STATUS_LAYER 8
#define UI_STATUS_LABEL_X 90
#define UI_STATUS_VALUE_X 245
#define UI_STATUS_BASE_Y 850
#define UI_STATUS_ROW_GAP 38
#define UI_STATUS_FONT_SIZE 14
#define UI_STATUS_WIDTH 2
#define UI_STATUS_ROW_COUNT 8
#define UI_STATUS_VALUE_CHARS 12

/* 超级电容：左侧能量弧、数字电压和控制命令。 */
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
#define UI_CAP_ERROR_X 450
#define UI_CAP_ERROR_Y 620
#define UI_CAP_CTRL_X UI_CAP_VOLTAGE_X
#define UI_CAP_CTRL_Y 700
#define UI_CAP_CTRL_TEXT_SIZE UI_CAP_TEXT_SIZE
#define UI_CAP_CTRL_TEXT_WIDTH UI_CAP_TEXT_WIDTH

/* 底盘速度：与电容弧共享圆心和半径，方向相反。 */
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

/* 下方地面透视引导线。 */
#define UI_GUIDE_LAYER 8
#define UI_GUIDE_COLOR UI_Color_Green
#define UI_GUIDE_LINE_WIDTH 2

/* 裁判系统坐标的 y 轴向上，数值越小越靠近屏幕底部。 */
#define UI_GUIDE_BOTTOM_Y 50        // 近端
#define UI_GUIDE_BOTTOM_HALF_W 900  // 近端半宽
#define UI_GUIDE_TOP_Y 430          // 远端
#define UI_GUIDE_TOP_HALF_W 300     // 远端半宽
#define UI_GUIDE_CENTER_X 960

/* ===========================================================================
 * 文件级状态
 * =========================================================================*/

/* 当前用于发送 UI 数据的裁判系统实例。 */
static referee_info_t *referee_recv_info;

/* UI 快照：当前值、上一帧值、脏标志和外部强制刷新请求。 */
static Referee_Interactive_info_t interactive_data;

/* ===========================================================================
 * 图形描述符
 *
 * 裁判 UI 的 CHANGE/DEL 依赖稳定图形名；底层发送接口又按值拷贝
 * Graph_Data_t/String_Data_t。因此描述符集中保存在文件级 static 区。
 * =========================================================================*/

/* 相对角度 */
static Graph_Data_t UI_RelativeRing;
static Graph_Data_t UI_RelativeArc[2];

/* 腿部，[0] 为左腿 UI，[1] 为右腿 UI。 */
static Graph_Data_t UI_LegRods[2][5];
static String_Data_t UI_LegLabel[4];
static String_Data_t UI_LegLength[2];

/* 瞄准框 */
static Graph_Data_t UI_AimCross[2];
static Graph_Data_t UI_AimRect;
static uint8_t UI_AimCrossVisible;

/* 超级电容 */
static Graph_Data_t UI_CapArc;
static String_Data_t UI_CapTextE;
static String_Data_t UI_CapTextF;
static String_Data_t UI_CapVoltage;
static String_Data_t UI_CapErrDetect;
static String_Data_t UI_CapCtrlCmd;

/* 速度 */
static Graph_Data_t UI_SpeedArc;
static String_Data_t UI_SpeedValue;

/* 状态文字 */
static String_Data_t UI_StatusLabel[UI_STATUS_ROW_COUNT];
static String_Data_t UI_StatusValue[UI_STATUS_ROW_COUNT];

/* 地面引导线 */
static Graph_Data_t UI_GuideSide[2];

/* 腿部图元名称和连杆拓扑是常量，不参与运行时决策。 */
static char UI_LegRodNameTable[2][5][4] = {
    {"ll0", "ll1", "ll2", "ll3", "ll4"},
    {"rl0", "rl1", "rl2", "rl3", "rl4"},
};
static const uint8_t UI_LegRodEdges[5][2] = {
    {0, 1},
    {1, 2},
    {2, 3},
    {3, 4},
    {4, 0},
};
static const char *const UI_StatusLabelText[UI_STATUS_ROW_COUNT] = {
    "ROBOT:", "CHASSIS:", "GIMBAL:", "FRICTION:", "LOADER:", "SUPERCAP:", "PITCH:", "ROLL:"};

/* ===========================================================================
 * 小工具
 * =========================================================================*/

/**
 * @brief 生成裁判 UI 的 3 字节图形名。
 *
 * 名称格式为 prefix + 两位十进制编号，例如 g02。调用方必须保证
 * 同一图元跨帧使用同一个名称。
 */
static void MakeUiName(char name[4], char prefix, uint8_t index) {
  name[0] = prefix;
  name[1] = (char)('0' + index / 10u);
  name[2] = (char)('0' + index % 10u);
  name[3] = '\0';
}

/** @brief 不引入额外库依赖的 float 绝对值。 */
static float AbsFloat(float value) { return value >= 0.0f ? value : -value; }

/** @brief 将任意角度折回 [0, 360)。 */
static float NormalizeAngle(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

/** @brief 转换成裁判 UI 使用的整数角度。 */
static uint32_t AngleToUiDegree(float angle) {
  uint32_t degree = (uint32_t)(NormalizeAngle(angle) + 0.5f);
  return degree >= 360 ? degree - 360 : degree;
}

/** @brief 限幅，保留表达式的意图。 */
static float ClampFloat(float value, float min_value, float max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

/** @brief 对正负数都对称的四舍五入。 */
static int32_t RoundToInt(float value) { return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f); }

/* ===========================================================================
 * 状态文案
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
    case CHASSIS_STAIR:
      return "STAIR";
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

/** @brief 速度量程随底盘姿态变化，匍匐模式单独判定。 */
static uint8_t IsProstrateMode(Robot_Mode_e robot_mode, Chassis_Mode_e chassis_mode) {
  return chassis_mode == CHASSIS_PROSTRATE || robot_mode == ROBOT_CHASSIS_PROSTRATE_ROTATE ||
         robot_mode == ROBOT_CHASSIS_PROSTRATE_FOLLOW;
}

static float SpeedMax(uint8_t is_prostrate) { return is_prostrate ? UI_SPEED_PROSTRATE_MAX : UI_SPEED_LEG_MAX; }

static uint32_t SpeedArcColor(const Referee_Interactive_info_t *data) {
  const float max_speed = SpeedMax(data->speed_is_prostrate);
  return AbsFloat(data->speed) >= max_speed * UI_SPEED_WARN_RATIO ? UI_Color_Purplish_red : UI_Color_Cyan;
}

/* ===========================================================================
 * 腿部几何
 * =========================================================================*/

/**
 * @brief 从两个主动关节角解出五连杆在局部坐标系下的姿态。
 *
 * 返回 0 表示输入缺失或几何无实数解。调用方据此把腿部 UI
 * 退化成黑色静止线框，而不是发送无意义坐标。
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
 * 裁判通信身份
 * =========================================================================*/

/**
 * @brief 由机器人 ID 推导客户端 ID。
 *
 * UI 发送给操作手客户端，receiver_ID 固定为当前机器人对应的
 * 0x0100 + robot_id。
 */
static void DeterminRobotID(void) {
  referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
  referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
  referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID;
  referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

/* ===========================================================================
 * 快照采样
 * =========================================================================*/

/**
 * @brief 读取底盘相对云台角度，优先使用双板通信中的 UI 快照。
 *
 * 双板模式下，云台板的本地 offset_angle 未必代表底盘侧最终状态；
 * chassis_fetch_data 存在时，以底盘板上报值为准。
 */
static float GetRelativeAngle(RobotInstance *robot) {
  if (robot == NULL) {
    return 0.0f;
  }

  float angle = robot->offset_angle;
#if !defined(ONE_BOARD)
  if (robot->chassis_fetch_data) {
    angle = (float)robot->chassis_fetch_data->gamestate.ui_status.ui_chassis_relative_angle_deg_x10 * 0.1f;
  }
#endif
  return angle;
}

/** @brief 采样腿部角度缓存；任一腿可解算即认为腿部数据有效。 */
static void SampleLegPosture(RobotInstance *robot, Referee_Interactive_info_t *data) {
  if (data == NULL) {
    return;
  }

  float model_x[5];
  float model_y[5];
  float leg_phi[4];

  data->leg_valid = 0;
  if (robot == NULL || robot->chassis == NULL) {
    return;
  }

  if (CalculateLegPosture(robot->chassis->leg[0], model_x, model_y, leg_phi) ||
      CalculateLegPosture(robot->chassis->leg[1], model_x, model_y, leg_phi)) {
    data->leg_valid = 1;
    data->leg_phi1 = leg_phi[0];
    data->leg_phi2 = leg_phi[1];
    data->leg_phi3 = leg_phi[2];
    data->leg_phi4 = leg_phi[3];
  }
}

/** @brief 采样状态面板、能量弧、速度弧和瞄准框所需的全部字段。 */
static void SampleStatusData(RobotInstance *robot, Referee_Interactive_info_t *data) {
  if (data == NULL) {
    return;
  }

  /* 先写安全默认值，缺失子系统不会留下上一帧的脏数据。 */
  data->robot_mode = ROBOT_POWER_OFF;
  data->chassis_mode = CHASSIS_POWER_OFF;
  data->gimbal_mode = GIMBAL_POWER_OFF;
  data->friction_mode = FRICTION_OFF;
  data->loader_mode = LOAD_STOP;
  data->chassis_pitch = 0.0f;
  data->chassis_roll = 0.0f;
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
    if (robot->chassis->imu) {
      data->chassis_pitch = robot->chassis->imu->Pitch;
      data->chassis_roll = robot->chassis->imu->Roll;
    }
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
    const UI_Remote_Status_s *status = &robot->chassis_fetch_data->gamestate.ui_status;
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
 * Relative：底盘相对云台角度
 * =========================================================================*/

static void DrawRelativePosition(float offset_angle, uint32_t operate) {
  const int32_t cx = UI_RELATIVE_CENTER_X;
  const int32_t cy = UI_RELATIVE_CENTER_Y;

  /*
   * 传感器角度与裁判 UI 坐标系左右相反。这里统一取负，让弧线在
   * 画面中表达真实车体朝向，而不是底层坐标的符号习惯。
   */
  const float ui_angle = NormalizeAngle(-offset_angle);
  const float start = NormalizeAngle(ui_angle - UI_RELATIVE_ARC_HALF_ANGLE);
  const float end = NormalizeAngle(ui_angle + UI_RELATIVE_ARC_HALF_ANGLE);

  /* 白色圆环是固定参照，绿色圆弧是当前方向。 */
  UICircleDraw(&UI_RelativeRing, "rg0", operate, UI_GRAPH_LAYER, UI_Color_White, UI_RELATIVE_RING_WIDTH, cx, cy,
               UI_RELATIVE_RADIUS);

  if (start <= end) {
    /* 不跨 0 度时只需要一段弧；第二段用黑色短弧清掉上一帧残留。 */
    UIArcDraw(&UI_RelativeArc[0], "sa0", operate, UI_GRAPH_LAYER, UI_Color_Green, AngleToUiDegree(start),
              AngleToUiDegree(end), UI_RELATIVE_ARC_WIDTH, cx, cy, UI_RELATIVE_RADIUS, UI_RELATIVE_RADIUS);
    UIArcDraw(&UI_RelativeArc[1], "sa1", operate, UI_GRAPH_LAYER, UI_Color_Black, 0, 1, UI_RELATIVE_ARC_WIDTH, cx, cy,
              UI_RELATIVE_RADIUS, UI_RELATIVE_RADIUS);
  } else {
    /* 跨 0 度时裁判 UI 无法表达环绕区间，必须拆成两段。 */
    UIArcDraw(&UI_RelativeArc[0], "sa0", operate, UI_GRAPH_LAYER, UI_Color_Green, AngleToUiDegree(start), 360,
              UI_RELATIVE_ARC_WIDTH, cx, cy, UI_RELATIVE_RADIUS, UI_RELATIVE_RADIUS);
    UIArcDraw(&UI_RelativeArc[1], "sa1", operate, UI_GRAPH_LAYER, UI_Color_Green, 0, AngleToUiDegree(end),
              UI_RELATIVE_ARC_WIDTH, cx, cy, UI_RELATIVE_RADIUS, UI_RELATIVE_RADIUS);
  }

  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_RelativeRing);
  UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_RelativeArc[0], UI_RelativeArc[1]);
}

/* ===========================================================================
 * Aim：中心瞄准框
 * =========================================================================*/

static void DrawAimIndicator(uint8_t target_locked, uint32_t operate) {
  const uint32_t rect_color = target_locked ? UI_Color_Purplish_red : UI_Color_Yellow;
  const int32_t cx = UI_CENTER_X;
  const int32_t cy = UI_CENTER_Y;

  /* 十字只在视觉锁定时存在；状态切换时要显式 ADD 或 DEL。 */
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

  /* 外框常驻，颜色传达锁定状态。 */
  UIRectangleDraw(&UI_AimRect, "ar0", operate, UI_AIM_LAYER, rect_color, UI_AIM_RECT_WIDTH, cx - UI_AIM_RECT_HALF_W,
                  cy - UI_AIM_RECT_HALF_H, cx + UI_AIM_RECT_HALF_W, cy + UI_AIM_RECT_HALF_H);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_AimRect);
}

/* ===========================================================================
 * Leg：五连杆位姿
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

  for (uint8_t i = 0; i < 5; i++) {
    UILineDraw(&UI_LegRods[ui_index][i], UI_LegRodNameTable[ui_index][i], operate, UI_LEG_LAYER, color,
               UI_LEG_ROD_WIDTH, px[UI_LegRodEdges[i][0]], py[UI_LegRodEdges[i][0]], px[UI_LegRodEdges[i][1]],
               py[UI_LegRodEdges[i][1]]);
  }
  UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_LegRods[ui_index][0], UI_LegRods[ui_index][1],
                 UI_LegRods[ui_index][2], UI_LegRods[ui_index][3], UI_LegRods[ui_index][4]);
}

static void DrawLegLabels(uint32_t operate) {
  UICharDraw(&UI_LegLabel[0], "lb0", operate, UI_LEG_LAYER, UI_Color_Cyan, UI_LEG_LABEL_FONT_SIZE, UI_LEG_LABEL_WIDTH,
             UI_LEG_LABEL_X, UI_LEG_TOP_BASE_Y + UI_LEG_LABEL_OFFSET_Y, "BACK");
  UICharRefresh(&referee_recv_info->referee_id, UI_LegLabel[0]);

  UICharDraw(&UI_LegLabel[1], "lf0", operate, UI_LEG_LAYER, UI_Color_Cyan, UI_LEG_LABEL_FONT_SIZE, UI_LEG_LABEL_WIDTH,
             UI_LEG_FRONT_LABEL_X, UI_LEG_TOP_BASE_Y + UI_LEG_LABEL_OFFSET_Y, "FRONT");
  UICharRefresh(&referee_recv_info->referee_id, UI_LegLabel[1]);

  UICharDraw(&UI_LegLabel[2], "lb1", operate, UI_LEG_LAYER, UI_Color_Cyan, UI_LEG_LABEL_FONT_SIZE, UI_LEG_LABEL_WIDTH,
             UI_LEG_LABEL_X, UI_LEG_BASE_Y + UI_LEG_LABEL_OFFSET_Y, "BACK");
  UICharRefresh(&referee_recv_info->referee_id, UI_LegLabel[2]);

  UICharDraw(&UI_LegLabel[3], "lf1", operate, UI_LEG_LAYER, UI_Color_Cyan, UI_LEG_LABEL_FONT_SIZE, UI_LEG_LABEL_WIDTH,
             UI_LEG_FRONT_LABEL_X, UI_LEG_BASE_Y + UI_LEG_LABEL_OFFSET_Y, "FRONT");
  UICharRefresh(&referee_recv_info->referee_id, UI_LegLabel[3]);
}

static void DrawLegLengthValue(const LegInstance *leg, uint8_t ui_index, int32_t base_y, uint32_t operate) {
  char name[4];
  MakeUiName(name, 'l', ui_index);

  if (leg == NULL) {
    UICharDraw(&UI_LegLength[ui_index], name, operate, UI_LEG_LAYER, UI_Color_Black, UI_LEG_LENGTH_FONT_SIZE,
               UI_LEG_LENGTH_WIDTH, UI_LEG_LENGTH_X, base_y + UI_LEG_LENGTH_OFFSET_Y, "L:---mm ");
  } else {
    int32_t length_mm = RoundToInt(leg->virtual_model.length * 1000.0f);
    if (length_mm < 0) length_mm = 0;
    UICharDraw(&UI_LegLength[ui_index], name, operate, UI_LEG_LAYER, UI_Color_Green, UI_LEG_LENGTH_FONT_SIZE,
               UI_LEG_LENGTH_WIDTH, UI_LEG_LENGTH_X, base_y + UI_LEG_LENGTH_OFFSET_Y, "L:%3dmm ", (int)length_mm);
  }

  UICharRefresh(&referee_recv_info->referee_id, UI_LegLength[ui_index]);
}

static void DrawLegPosture(RobotInstance *robot, uint32_t operate) {
  const LegInstance *top_leg = NULL;
  const LegInstance *bottom_leg = NULL;

  if (robot->chassis) {
    top_leg = robot->chassis->leg[1];
    bottom_leg = robot->chassis->leg[0];
  }
  DrawSingleLegPosture(top_leg, 0, UI_LEG_TOP_BASE_Y, operate);
  DrawLegLengthValue(top_leg, 0, UI_LEG_TOP_BASE_Y, operate);
  DrawSingleLegPosture(bottom_leg, 1, UI_LEG_BASE_Y, operate);
  DrawLegLengthValue(bottom_leg, 1, UI_LEG_BASE_Y, operate);
}

/* ===========================================================================
 * Status：状态文字
 * =========================================================================*/

/** @brief 固定标签只在 ADD 阶段发送。 */
static void DrawStatusStatic(uint32_t operate) {
  for (uint8_t i = 0; i < UI_STATUS_ROW_COUNT; i++) {
    char name[4];
    MakeUiName(name, 'u', i);
    UICharDraw(&UI_StatusLabel[i], name, operate, UI_STATUS_LAYER, UI_Color_Purplish_red, UI_STATUS_FONT_SIZE,
               UI_STATUS_WIDTH, UI_STATUS_LABEL_X, UI_STATUS_BASE_Y - i * UI_STATUS_ROW_GAP,
               UI_StatusLabelText[i]);
    UICharRefresh(&referee_recv_info->referee_id, UI_StatusLabel[i]);
  }
}

/** @brief 写入单行状态值，并用定宽格式覆盖旧字符。 */
static void DrawStatusValue(uint8_t row, uint32_t operate, const char *value_str) {
  char name[4];
  MakeUiName(name, 'v', row);
  UICharDraw(&UI_StatusValue[row], name, operate, UI_STATUS_LAYER, UI_Color_Green, UI_STATUS_FONT_SIZE, UI_STATUS_WIDTH,
             UI_STATUS_VALUE_X, UI_STATUS_BASE_Y - row * UI_STATUS_ROW_GAP, "%-*s", UI_STATUS_VALUE_CHARS, value_str);
  UICharRefresh(&referee_recv_info->referee_id, UI_StatusValue[row]);
}

/** @brief 状态值作为一组刷新，保持面板一致。 */
static void DrawStatusAngleValue(uint8_t row, uint32_t operate, float angle_deg) {
  char name[4];
  MakeUiName(name, 'v', row);

  int32_t angle_x10 = RoundToInt(angle_deg * 10.0f);
  int32_t abs_angle_x10 = angle_x10 >= 0 ? angle_x10 : -angle_x10;
  const char sign = angle_x10 < 0 ? '-' : '+';

  UICharDraw(&UI_StatusValue[row], name, operate, UI_STATUS_LAYER, UI_Color_Cyan, UI_STATUS_FONT_SIZE, UI_STATUS_WIDTH,
             UI_STATUS_VALUE_X, UI_STATUS_BASE_Y - row * UI_STATUS_ROW_GAP, "%c%d.%ddeg  ", sign,
             (int)(abs_angle_x10 / 10), (int)(abs_angle_x10 % 10));
  UICharRefresh(&referee_recv_info->referee_id, UI_StatusValue[row]);
}

static void DrawStatusDynamic(Referee_Interactive_info_t *data, uint32_t operate) {
  DrawStatusValue(0, operate, RobotModeStr(data->robot_mode));
  DrawStatusValue(1, operate, ChassisModeStr(data->chassis_mode));
  DrawStatusValue(2, operate, GimbalModeStr(data->gimbal_mode));
  DrawStatusValue(3, operate, FrictionModeStr(data->friction_mode));
  DrawStatusValue(4, operate, LoaderModeStr(data->loader_mode));
  DrawStatusValue(5, operate, SuperCapModeStr(data->super_cap_mode));
  DrawStatusAngleValue(6, operate, data->chassis_pitch);
  DrawStatusAngleValue(7, operate, data->chassis_roll);
}

/* ===========================================================================
 * Cap：超级电容
 * =========================================================================*/

/** @brief 电容异常直接熄灭能量弧，低电压逐级警示。 */
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
  /* 电压线性映射到固定扫角，并保留最小可见宽度。 */
  const float sweep =
      ClampFloat((data->cap_voltage - UI_CAP_EMPTY_VOLTAGE) / (UI_CAP_FULL_VOLTAGE - UI_CAP_EMPTY_VOLTAGE) *
                     UI_CAP_MAX_SWEEP,
                 1.0f, UI_CAP_MAX_SWEEP);

  int32_t voltage_x10 = RoundToInt(data->cap_voltage * 10.0f);
  if (voltage_x10 < 0) voltage_x10 = 0;

  /* 电量弧。 */
  UIArcDraw(&UI_CapArc, "cp0", operate, UI_CAP_LAYER, CapArcColor(data), UI_CAP_START_ANGLE,
            UI_CAP_START_ANGLE + (uint32_t)sweep, UI_CAP_WIDTH, UI_CAP_CENTER_X, UI_CAP_CENTER_Y, UI_CAP_RADIUS_X,
            UI_CAP_RADIUS_Y);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_CapArc);

  /* 电压数字。 */
  UICharDraw(&UI_CapVoltage, "cv0", operate, UI_CAP_LAYER, UI_Color_Cyan, UI_CAP_TEXT_SIZE, UI_CAP_TEXT_WIDTH,
             UI_CAP_VOLTAGE_X, UI_CAP_VOLTAGE_Y, "%2d.%dV   ", (int)(voltage_x10 / 10), (int)(voltage_x10 % 10));
  UICharRefresh(&referee_recv_info->referee_id, UI_CapVoltage);

  UICharDraw(&UI_CapErrDetect, "cd0", operate, UI_CAP_LAYER,
             data->cap_error == 0 ? UI_Color_Green : UI_Color_Purplish_red, UI_CAP_TEXT_SIZE, UI_CAP_TEXT_WIDTH,
             UI_CAP_ERROR_X, UI_CAP_ERROR_Y, "ERR:%1u ", (unsigned)data->cap_error);
  UICharRefresh(&referee_recv_info->referee_id, UI_CapErrDetect);

  /* 控制命令。 */
  UICharDraw(&UI_CapCtrlCmd, "cc0", operate, UI_CAP_LAYER, SuperCapCtrlCmdColor(data->super_cap_ctrl_cmd),
             UI_CAP_CTRL_TEXT_SIZE, UI_CAP_CTRL_TEXT_WIDTH, UI_CAP_CTRL_X, UI_CAP_CTRL_Y, "%-6s",
             SuperCapCtrlCmdStr(data->super_cap_ctrl_cmd));
  UICharRefresh(&referee_recv_info->referee_id, UI_CapCtrlCmd);
}

/* ===========================================================================
 * Speed：底盘速度
 * =========================================================================*/

static void DrawSpeedDynamic(const Referee_Interactive_info_t *data, uint32_t operate) {
  const float max_speed = SpeedMax(data->speed_is_prostrate);
  const uint32_t color = SpeedArcColor(data);

  const float sweep = ClampFloat(AbsFloat(data->speed) / max_speed * UI_SPEED_MAX_SWEEP, 1.0f, UI_SPEED_MAX_SWEEP);

  int32_t speed_x10 = RoundToInt(data->speed * 10.0f);
  int32_t abs_speed_x10 = speed_x10 >= 0 ? speed_x10 : -speed_x10;
  const char sign = speed_x10 < 0 ? '-' : ' ';

  /* 速度弧与电容弧共用视觉语言，超阈值变为警示色。 */
  UIArcDraw(&UI_SpeedArc, "sp0", operate, UI_SPEED_LAYER, color, UI_SPEED_CENTER_ANGLE - (uint32_t)sweep,
            UI_SPEED_CENTER_ANGLE, UI_SPEED_WIDTH, UI_SPEED_CENTER_X, UI_SPEED_CENTER_Y, UI_SPEED_RADIUS_X,
            UI_SPEED_RADIUS_Y);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_SpeedArc);

  /* 数字保留符号，避免倒车时只看弧长产生歧义。 */
  UICharDraw(&UI_SpeedValue, "sv0", operate, UI_SPEED_LAYER, color, UI_SPEED_TEXT_SIZE, UI_SPEED_TEXT_WIDTH,
             UI_SPEED_TEXT_X, UI_SPEED_TEXT_Y, "%c%d.%dm/s  ", sign, (int)(abs_speed_x10 / 10),
             (int)(abs_speed_x10 % 10));
  UICharRefresh(&referee_recv_info->referee_id, UI_SpeedValue);
}

/* ===========================================================================
 * Guide：地面透视引导线
 * =========================================================================*/

/**
 * @brief 绘制地面透视线。
 *
 * 两条侧边负责收敛，横向刻度用非线性插值压向远端，给驾驶员
 * 一个稳定的车体宽度和前方空间参考。
 */
static void DrawGuideLine(uint32_t operate) {
  const int32_t cx = UI_GUIDE_CENTER_X;
  const int32_t y_bot = UI_GUIDE_BOTTOM_Y;
  const int32_t y_top = UI_GUIDE_TOP_Y;
  const int32_t half_bot = UI_GUIDE_BOTTOM_HALF_W;
  const int32_t half_top = UI_GUIDE_TOP_HALF_W;

  /* 两条侧边定义透视梯形。 */
  UILineDraw(&UI_GuideSide[0], "gd0", operate, UI_GUIDE_LAYER, UI_GUIDE_COLOR, UI_GUIDE_LINE_WIDTH, cx - half_bot,
             y_bot, cx - half_top, y_top);
  UILineDraw(&UI_GuideSide[1], "gd1", operate, UI_GUIDE_LAYER, UI_GUIDE_COLOR, UI_GUIDE_LINE_WIDTH, cx + half_bot,
             y_bot, cx + half_top, y_top);
  UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_GuideSide[0], UI_GuideSide[1]);
}

/* ===========================================================================
 * 变化检测
 * =========================================================================*/

static void UIChangeCheck(Referee_Interactive_info_t *data) {
  /* 相对角度需要即时响应，不做死区。 */
  if (data->chassis_relative_angle != data->last_chassis_relative_angle) {
    data->UI_Interactive_Flag.relative_flag = 1;
    data->last_chassis_relative_angle = data->chassis_relative_angle;
  }

  /* 腿部是连续运动图元，只要数据有效就持续推送。 */
  if (data->leg_valid != data->last_leg_valid || data->leg_valid) {
    data->UI_Interactive_Flag.leg_flag = 1;
    data->last_leg_valid = data->leg_valid;
    data->last_leg_phi1 = data->leg_phi1;
    data->last_leg_phi2 = data->leg_phi2;
    data->last_leg_phi3 = data->leg_phi3;
    data->last_leg_phi4 = data->leg_phi4;
  }

  /* 状态面板按组刷新，避免不同子系统文字分帧跳变。 */
  uint8_t status_changed =
      data->robot_mode != data->last_robot_mode || data->chassis_mode != data->last_chassis_mode ||
      data->gimbal_mode != data->last_gimbal_mode || data->friction_mode != data->last_friction_mode ||
      data->loader_mode != data->last_loader_mode || data->super_cap_mode != data->last_super_cap_mode ||
      fabsf(data->chassis_pitch - data->last_chassis_pitch) > 0.1f ||
      fabsf(data->chassis_roll - data->last_chassis_roll) > 0.1f;
  if (status_changed) {
    data->UI_Interactive_Flag.status_flag = 1;
    data->last_robot_mode = data->robot_mode;
    data->last_chassis_mode = data->chassis_mode;
    data->last_gimbal_mode = data->gimbal_mode;
    data->last_friction_mode = data->friction_mode;
    data->last_loader_mode = data->loader_mode;
    data->last_super_cap_mode = data->super_cap_mode;
    data->last_chassis_pitch = data->chassis_pitch;
    data->last_chassis_roll = data->chassis_roll;
  }

  /* 电压小幅抖动不刷新；错误和控制命令必须立即刷新。 */
  if (fabsf(data->cap_voltage - data->last_cap_voltage) > 0.1f || data->cap_error != data->last_cap_error ||
      data->super_cap_ctrl_cmd != data->last_super_cap_ctrl_cmd) {
    data->UI_Interactive_Flag.cap_flag = 1;
    data->last_cap_voltage = data->cap_voltage;
    data->last_cap_error = data->cap_error;
    data->last_super_cap_ctrl_cmd = data->super_cap_ctrl_cmd;
  }

  /* 速度用小死区抑制抖动，量程切换必须刷新。 */
  if (fabsf(data->speed - data->last_speed) > 0.05f || data->speed_is_prostrate != data->last_speed_is_prostrate) {
    data->UI_Interactive_Flag.speed_flag = 1;
    data->last_speed = data->speed;
    data->last_speed_is_prostrate = data->speed_is_prostrate;
  }

  /* 锁定状态决定十字的 ADD/DEL。 */
  if (data->aim_target_flag != data->last_aim_target_flag) {
    data->UI_Interactive_Flag.aim_flag = 1;
    data->last_aim_target_flag = data->aim_target_flag;
  }
}

/* ===========================================================================
 * 增量刷新
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
 * 快照同步
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
  d->last_chassis_pitch = d->chassis_pitch;
  d->last_chassis_roll = d->chassis_roll;
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
 * @brief 清空客户端并重新 ADD 全部图形。
 *
 * 启动、裁判客户端重连、手动刷新和周期兜底都会走同一条路径。
 */
void MyUIInit(RobotInstance *robot) {
  /* UITask 可能长期运行，入口处重新绑定裁判数据指针。 */
  referee_recv_info = robot->referee_data;

  /* 没有 robot_id 时无法推导客户端 ID，只能等待。 */
  while (referee_recv_info->GameRobotState.robot_id == 0) {
    osDelay(100);
  }

  /* 先删除客户端旧图元，再以 ADD 重建稳定图元集合。 */
  DeterminRobotID();
  UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0);
  UI_AimCrossVisible = 0;

  /* 初始化后的第一帧不应因为 last_* 未同步而重复 CHANGE。 */
  interactive_data.chassis_relative_angle = GetRelativeAngle(robot);
  SampleLegPosture(robot, &interactive_data);
  SampleStatusData(robot, &interactive_data);
  SyncLastValues(&interactive_data);

  DrawRelativePosition(interactive_data.chassis_relative_angle, UI_Graph_ADD);
  DrawAimIndicator(interactive_data.aim_target_flag, UI_Graph_ADD);
  DrawGuideLine(UI_Graph_ADD);
  DrawLegLabels(UI_Graph_ADD);
  DrawLegPosture(robot, UI_Graph_ADD);
  DrawStatusStatic(UI_Graph_ADD);
  DrawStatusDynamic(&interactive_data, UI_Graph_ADD);
  DrawCapStatic(UI_Graph_ADD);
  DrawCapDynamic(&interactive_data, UI_Graph_ADD);
  DrawSpeedDynamic(&interactive_data, UI_Graph_ADD);
}

/**
 * @brief 返回 UI 快照，外部主要用于置位 force_refresh_ui。
 */
Referee_Interactive_info_t *getUI(void) { return &interactive_data; }

/**
 * @brief 周期性 UI 任务。
 *
 * 调用频率应与 UI_TASK_PERIOD_MS 一致。任务先处理全量重绘请求，
 * 再采样最新状态，最后按脏标志增量刷新。
 */
void UITask(RobotInstance *robot) {
  referee_recv_info = robot->referee_data;

  /* 全量刷新：外部请求或周期兜底。 */
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
    /* 保持原调度语义：全量刷新后仍继续执行一次增量流程。 */
  }

  /* 采样最新状态。 */
  interactive_data.chassis_relative_angle = GetRelativeAngle(robot);
  SampleLegPosture(robot, &interactive_data);
  SampleStatusData(robot, &interactive_data);

  /* 周期性保活图元，降低客户端丢图后的残缺时间。 */
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

  /* 变化检测与增量刷新。 */
  UIChangeCheck(&interactive_data);
  MyUIRefresh(robot, &interactive_data);
}
