#include "ui.h"

#include "cmsis_os.h"
#include "referee.h"
#include "referee_ui.h"

#include <math.h>
#include <stdio.h>

#define UI_LABEL_LAYER 8
#define UI_GRAPH_LAYER 7
#define UI_TEXT_SIZE 15
#define UI_TEXT_WIDTH 2

static referee_info_t *referee_recv_info;
static Referee_Interactive_info_t interactive_data;

uint8_t UI_Seq;

static String_Data_t UI_StaticText[11];
static String_Data_t UI_ModeText;
static String_Data_t UI_PowerText;
static String_Data_t UI_PointText;
static String_Data_t UI_JumpText;
static String_Data_t UI_SpeedText;
static String_Data_t UI_DistanceText;
static String_Data_t UI_VisionText;

static Graph_Data_t UI_RelativeCircle;
static Graph_Data_t UI_RelativeArrow;
static Graph_Data_t UI_LegBody;
static Graph_Data_t UI_LegLeft;
static Graph_Data_t UI_LegRight;
static Graph_Data_t UI_VisionBox;
static Graph_Data_t UI_HitCross[2];

static void DeterminRobotID(void) {
  referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
  referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
  referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID;
  referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static const char *RobotModeString(Robot_Mode_e mode) {
  switch (mode) {
    case ROBOT_POWER_OFF:
      return "off";
    case ROBOT_CHASSIS_ROTATE:
      return "rot";
    case ROBOT_CHASSIS_FOLLOW:
      return "fol";
    case ROBOT_CHASSIS_FREE:
      return "fre";
    case ROBOT_CHASSIS_PROSTRATE_ROTATE:
      return "prt";
    case ROBOT_CHASSIS_PROSTRATE_FOLLOW:
      return "prf";
    case ROBOT_CHASSIS_PROSTRATE_FREE:
      return "prx";
    default:
      return "unk";
  }
}

static const char *ChassisModeString(Chassis_Mode_e mode) {
  switch (mode) {
    case CHASSIS_POWER_OFF:
      return "off";
    case CHASSIS_RECOVERY:
      return "rec";
    case CHASSIS_ON:
      return "on";
    case CHASSIS_JUMP_READY:
      return "jrd";
    case CHASSIS_JUMP_START:
      return "jmp";
    case CHASSIS_PROSTRATE:
      return "pro";
    default:
      return "unk";
  }
}

static const char *GimbalModeString(Gimbal_Mode_e mode) {
  switch (mode) {
    case GIMBAL_POWER_OFF:
      return "off";
    case GIMBAL_ON:
      return "on";
    case GIMBAL_VISION:
      return "vis";
    default:
      return "unk";
  }
}

static const char *ShootModeString(Shoot_Mode_e mode, Friction_Mode_e fric_mode) {
  if (mode == SHOOT_OFF) {
    return "off";
  }
  return fric_mode == FRICTION_ON ? "frc" : "on";
}

static const char *JumpStateString(Jump_State_e state) {
  switch (state) {
    case JUMP_STATE_IDLE:
      return "idle";
    case JUMP_STATE_COMPRESS:
      return "cmp";
    case JUMP_STATE_EXTEND:
      return "ext";
    case JUMP_STATE_RETRACT:
      return "ret";
    default:
      return "unk";
  }
}

static uint8_t VisionHasTarget(const Vision_Receive_s *vision) {
  return vision != NULL &&
         (vision->gimbal_receive.yaw != 0.0f || vision->gimbal_receive.pitch != 0.0f ||
          vision->shoot_receive.fire_flag != 0);
}

static void SetAllRefreshFlags(void) {
  interactive_data.UI_Interactive_Flag.mode_flag = 1;
  interactive_data.UI_Interactive_Flag.power_flag = 1;
  interactive_data.UI_Interactive_Flag.yaw_flag = 1;
  interactive_data.UI_Interactive_Flag.leg_flag = 1;
  interactive_data.UI_Interactive_Flag.vision_flag = 1;
  interactive_data.UI_Interactive_Flag.hit_flag = 1;
  interactive_data.UI_Interactive_Flag.point_flag = 1;
  interactive_data.UI_Interactive_Flag.jump_flag = 1;
  interactive_data.UI_Interactive_Flag.speed_flag = 1;
  interactive_data.UI_Interactive_Flag.distance_flag = 1;
}

static void UIChangeCheck(Referee_Interactive_info_t *data) {
  if (data->robot_mode != data->last_robot_mode || data->chassis_mode != data->last_chassis_mode ||
      data->gimbal_mode != data->last_gimbal_mode || data->shoot_mode != data->last_shoot_mode ||
      data->friction_mode != data->last_friction_mode) {
    data->UI_Interactive_Flag.mode_flag = 1;
    data->last_robot_mode = data->robot_mode;
    data->last_chassis_mode = data->chassis_mode;
    data->last_gimbal_mode = data->gimbal_mode;
    data->last_shoot_mode = data->shoot_mode;
    data->last_friction_mode = data->friction_mode;
  }

  if (fabsf(data->cap_msg.cap_v - data->last_cap_voltage) > 0.1f ||
      fabsf(data->cap_msg.in_p - data->last_cap_in_power) > 0.1f ||
      fabsf(data->cap_msg.out_p - data->last_cap_out_power) > 0.1f ||
      data->cap_msg.error_detect != data->last_cap_error || data->buffer_energy != data->last_buffer_energy ||
      data->cap_ctrl_cmd != data->last_cap_ctrl_cmd) {
    data->UI_Interactive_Flag.power_flag = 1;
    data->last_cap_voltage = data->cap_msg.cap_v;
    data->last_cap_in_power = data->cap_msg.in_p;
    data->last_cap_out_power = data->cap_msg.out_p;
    data->last_cap_error = data->cap_msg.error_detect;
    data->last_buffer_energy = data->buffer_energy;
    data->last_cap_ctrl_cmd = data->cap_ctrl_cmd;
  }

  if (fabsf(data->chassis_relative_angle - data->last_chassis_relative_angle) > 1.0f) {
    data->UI_Interactive_Flag.yaw_flag = 1;
    data->last_chassis_relative_angle = data->chassis_relative_angle;
  }

  if (fabsf(data->leg_left_angle - data->last_leg_left_angle) > 0.02f ||
      fabsf(data->leg_right_angle - data->last_leg_right_angle) > 0.02f) {
    data->UI_Interactive_Flag.leg_flag = 1;
    data->last_leg_left_angle = data->leg_left_angle;
    data->last_leg_right_angle = data->leg_right_angle;
  }

  if (data->vision_tracking != data->last_vision_tracking) {
    data->UI_Interactive_Flag.vision_flag = 1;
    data->last_vision_tracking = data->vision_tracking;
  }

  if (data->autoaim_hit != data->last_autoaim_hit) {
    data->UI_Interactive_Flag.hit_flag = 1;
    data->last_autoaim_hit = data->autoaim_hit;
  }

  if (data->event_type != data->last_event_type) {
    data->UI_Interactive_Flag.point_flag = 1;
    data->last_event_type = data->event_type;
  }

  if (data->auto_jump_req != data->last_auto_jump_req || data->jump_state != data->last_jump_state) {
    data->UI_Interactive_Flag.jump_flag = 1;
    data->UI_Interactive_Flag.distance_flag = 1;
    data->last_auto_jump_req = data->auto_jump_req;
    data->last_jump_state = data->jump_state;
  }

  if (fabsf(data->current_speed - data->last_current_speed) > 0.05f) {
    data->UI_Interactive_Flag.speed_flag = 1;
    data->last_current_speed = data->current_speed;
  }

  if (fabsf(data->jump_distance - data->last_jump_distance) > 0.02f ||
      data->auto_jump_req != data->last_auto_jump_req) {
    data->UI_Interactive_Flag.distance_flag = 1;
    data->last_jump_distance = data->jump_distance;
  }
}

static void RefreshRelativePosition(Referee_Interactive_info_t *data, uint32_t operate) {
  const int32_t center_x = 1590;
  const int32_t center_y = 285;
  const uint32_t radius = 72;
  float angle_rad = data->chassis_relative_angle * 3.1415926f / 180.0f;
  int32_t end_x = center_x + (int32_t)(cosf(angle_rad) * (float)radius);
  int32_t end_y = center_y - (int32_t)(sinf(angle_rad) * (float)radius);

  UICircleDraw(&UI_RelativeCircle, "rp0", operate, UI_GRAPH_LAYER, UI_Color_White, 3, (uint32_t)center_x,
               (uint32_t)center_y, radius);
  UILineDraw(&UI_RelativeArrow, "rp1", operate, UI_GRAPH_LAYER, UI_Color_Green, 5, (uint32_t)center_x,
             (uint32_t)center_y, (uint32_t)end_x, (uint32_t)end_y);
  UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_RelativeCircle, UI_RelativeArrow);
}

static void RefreshLegPosture(Referee_Interactive_info_t *data, uint32_t operate) {
  const int32_t body_y = 410;
  const int32_t left_hip_x = 1490;
  const int32_t right_hip_x = 1630;
  const uint32_t leg_len = 90;
  float left_angle = 1.5708f + data->leg_left_angle;
  float right_angle = 1.5708f + data->leg_right_angle;
  int32_t left_foot_x = left_hip_x + (int32_t)(cosf(left_angle) * (float)leg_len);
  int32_t left_foot_y = body_y + (int32_t)(sinf(left_angle) * (float)leg_len);
  int32_t right_foot_x = right_hip_x + (int32_t)(cosf(right_angle) * (float)leg_len);
  int32_t right_foot_y = body_y + (int32_t)(sinf(right_angle) * (float)leg_len);

  UILineDraw(&UI_LegBody, "lg0", operate, UI_GRAPH_LAYER, UI_Color_White, 5, (uint32_t)(left_hip_x - 35),
             (uint32_t)body_y, (uint32_t)(right_hip_x + 35), (uint32_t)body_y);
  UILineDraw(&UI_LegLeft, "lg1", operate, UI_GRAPH_LAYER, UI_Color_Yellow, 6, (uint32_t)left_hip_x,
             (uint32_t)body_y, (uint32_t)left_foot_x, (uint32_t)left_foot_y);
  UILineDraw(&UI_LegRight, "lg2", operate, UI_GRAPH_LAYER, UI_Color_Yellow, 6, (uint32_t)right_hip_x,
             (uint32_t)body_y, (uint32_t)right_foot_x, (uint32_t)right_foot_y);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_LegBody);
  UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_LegLeft, UI_LegRight);
}

static void RefreshVisionBox(Referee_Interactive_info_t *data, uint32_t operate) {
  uint32_t color = data->vision_tracking ? UI_Color_Green : UI_Color_Black;
  UIRectangleDraw(&UI_VisionBox, "vb0", operate, UI_GRAPH_LAYER, color, 3, UI_CENTER_X - 90, UI_CENTER_Y - 55,
                  UI_CENTER_X + 90, UI_CENTER_Y + 55);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_VisionBox);

  UICharDraw(&UI_VisionText, "vt0", operate, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH, 120,
             500, "vision:%s", data->vision_tracking ? "box" : "none");
  UICharRefresh(&referee_recv_info->referee_id, UI_VisionText);
}

static void RefreshHitCross(Referee_Interactive_info_t *data, uint32_t operate) {
  uint32_t color = data->autoaim_hit ? UI_Color_Pink : UI_Color_Black;
  UILineDraw(&UI_HitCross[0], "hx0", operate, UI_GRAPH_LAYER, color, 6, UI_CENTER_X - 24, UI_CENTER_Y - 24,
             UI_CENTER_X + 24, UI_CENTER_Y + 24);
  UILineDraw(&UI_HitCross[1], "hx1", operate, UI_GRAPH_LAYER, color, 6, UI_CENTER_X - 24, UI_CENTER_Y + 24,
             UI_CENTER_X + 24, UI_CENTER_Y - 24);
  UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_HitCross[0], UI_HitCross[1]);
}

static void RefreshModeText(Referee_Interactive_info_t *data, uint32_t operate) {
  UICharDraw(&UI_ModeText, "md0", operate, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH, 120, 650,
             "R:%s C:%s G:%s S:%s", RobotModeString(data->robot_mode), ChassisModeString(data->chassis_mode),
             GimbalModeString(data->gimbal_mode), ShootModeString(data->shoot_mode, data->friction_mode));
  UICharRefresh(&referee_recv_info->referee_id, UI_ModeText);
}

static void RefreshPowerText(Referee_Interactive_info_t *data, uint32_t operate) {
  int32_t cap_v10 = (int32_t)(data->cap_msg.cap_v * 10.0f);
  if (cap_v10 < 0) cap_v10 = 0;
  UICharDraw(&UI_PowerText, "pw0", operate, UI_LABEL_LAYER, UI_Color_Yellow, UI_TEXT_SIZE, UI_TEXT_WIDTH, 120, 600,
             "E:%u C:%ld.%ldV %s", data->buffer_energy, cap_v10 / 10, cap_v10 % 10,
             data->cap_msg.error_detect ? "cap_bad" : (data->cap_ctrl_cmd == BOOST ? "boost" : "normal"));
  UICharRefresh(&referee_recv_info->referee_id, UI_PowerText);
}

static void RefreshPointText(Referee_Interactive_info_t *data, uint32_t operate) {
  uint8_t center = (uint8_t)((data->event_type >> 23) & 0x03u);
  uint8_t fortress = (uint8_t)((data->event_type >> 25) & 0x03u);
  uint8_t outpost = (uint8_t)((data->event_type >> 27) & 0x03u);
  uint8_t base = (uint8_t)((data->event_type >> 29) & 0x01u);
  UICharDraw(&UI_PointText, "pt0", operate, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH, 120, 550,
             "P C:%u F:%u O:%u B:%u", center, fortress, outpost, base);
  UICharRefresh(&referee_recv_info->referee_id, UI_PointText);
}

static void RefreshJumpText(Referee_Interactive_info_t *data, uint32_t operate) {
  UICharDraw(&UI_JumpText, "jp0", operate, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH, 120, 450,
             "J:%s %s", data->auto_jump_req ? "req" : "off", JumpStateString(data->jump_state));
  UICharRefresh(&referee_recv_info->referee_id, UI_JumpText);
}

static void RefreshSpeedText(Referee_Interactive_info_t *data, uint32_t operate) {
  int32_t speed100 = (int32_t)(data->current_speed * 100.0f);
  const char *sign = speed100 < 0 ? "-" : "";
  if (speed100 < 0) speed100 = -speed100;
  UICharDraw(&UI_SpeedText, "sp0", operate, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH, 120, 400,
             "V:%s%ld.%02ldm/s", sign, speed100 / 100, speed100 % 100);
  UICharRefresh(&referee_recv_info->referee_id, UI_SpeedText);
}

static void RefreshDistanceText(Referee_Interactive_info_t *data, uint32_t operate) {
  if (data->auto_jump_req) {
    int32_t dist100 = (int32_t)(data->jump_distance * 100.0f);
    if (dist100 < 0) dist100 = 0;
    UICharDraw(&UI_DistanceText, "ds0", operate, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH, 120,
               350, "D:%ld.%02ldm", dist100 / 100, dist100 % 100);
  } else {
    UICharDraw(&UI_DistanceText, "ds0", operate, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH, 120,
               350, "D:--");
  }
  UICharRefresh(&referee_recv_info->referee_id, UI_DistanceText);
}

static void MyUIRefresh(Referee_Interactive_info_t *data) {
  if (data->UI_Interactive_Flag.mode_flag) {
    RefreshModeText(data, UI_Graph_Change);
    data->UI_Interactive_Flag.mode_flag = 0;
  }
  if (data->UI_Interactive_Flag.power_flag) {
    RefreshPowerText(data, UI_Graph_Change);
    data->UI_Interactive_Flag.power_flag = 0;
  }
  if (data->UI_Interactive_Flag.point_flag) {
    RefreshPointText(data, UI_Graph_Change);
    data->UI_Interactive_Flag.point_flag = 0;
  }
  if (data->UI_Interactive_Flag.jump_flag) {
    RefreshJumpText(data, UI_Graph_Change);
    data->UI_Interactive_Flag.jump_flag = 0;
  }
  if (data->UI_Interactive_Flag.speed_flag) {
    RefreshSpeedText(data, UI_Graph_Change);
    data->UI_Interactive_Flag.speed_flag = 0;
  }
  if (data->UI_Interactive_Flag.distance_flag) {
    RefreshDistanceText(data, UI_Graph_Change);
    data->UI_Interactive_Flag.distance_flag = 0;
  }
  if (data->UI_Interactive_Flag.yaw_flag) {
    RefreshRelativePosition(data, UI_Graph_Change);
    data->UI_Interactive_Flag.yaw_flag = 0;
  }
  if (data->UI_Interactive_Flag.leg_flag) {
    RefreshLegPosture(data, UI_Graph_Change);
    data->UI_Interactive_Flag.leg_flag = 0;
  }
  if (data->UI_Interactive_Flag.vision_flag) {
    RefreshVisionBox(data, UI_Graph_Change);
    data->UI_Interactive_Flag.vision_flag = 0;
  }
  if (data->UI_Interactive_Flag.hit_flag) {
    RefreshHitCross(data, UI_Graph_Change);
    data->UI_Interactive_Flag.hit_flag = 0;
  }
}

void MyUIInit(RobotInstance *robot) {
  referee_recv_info = robot->referee_data;

  while (referee_recv_info->GameRobotState.robot_id == 0) {
    osDelay(100);
  }

  DeterminRobotID();
  UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0);

  UICharDraw(&UI_StaticText[0], "lb0", UI_Graph_ADD, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH, 120,
             700, "robot/chassis/gimbal/shoot");
  UICharRefresh(&referee_recv_info->referee_id, UI_StaticText[0]);
  UICharDraw(&UI_StaticText[1], "lb1", UI_Graph_ADD, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH, 120,
             625, "battery + supercap");
  UICharRefresh(&referee_recv_info->referee_id, UI_StaticText[1]);
  UICharDraw(&UI_StaticText[2], "lb2", UI_Graph_ADD, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH, 120,
             575, "point info");
  UICharRefresh(&referee_recv_info->referee_id, UI_StaticText[2]);
  UICharDraw(&UI_StaticText[3], "lb3", UI_Graph_ADD, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH, 120,
             475, "auto jump request");
  UICharRefresh(&referee_recv_info->referee_id, UI_StaticText[3]);
  UICharDraw(&UI_StaticText[4], "lb4", UI_Graph_ADD, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH, 120,
             425, "current speed");
  UICharRefresh(&referee_recv_info->referee_id, UI_StaticText[4]);
  UICharDraw(&UI_StaticText[5], "lb5", UI_Graph_ADD, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH, 120,
             375, "distance");
  UICharRefresh(&referee_recv_info->referee_id, UI_StaticText[5]);
  UICharDraw(&UI_StaticText[6], "lb6", UI_Graph_ADD, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH, 120,
             525, "autoaim box");
  UICharRefresh(&referee_recv_info->referee_id, UI_StaticText[6]);
  UICharDraw(&UI_StaticText[7], "lb7", UI_Graph_ADD, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH,
             1460, 190, "chassis-gimbal relative");
  UICharRefresh(&referee_recv_info->referee_id, UI_StaticText[7]);
  UICharDraw(&UI_StaticText[8], "lb8", UI_Graph_ADD, UI_LABEL_LAYER, UI_Color_White, UI_TEXT_SIZE, UI_TEXT_WIDTH,
             1510, 535, "leg posture");
  UICharRefresh(&referee_recv_info->referee_id, UI_StaticText[8]);

  SetAllRefreshFlags();
  RefreshModeText(&interactive_data, UI_Graph_ADD);
  RefreshPowerText(&interactive_data, UI_Graph_ADD);
  RefreshPointText(&interactive_data, UI_Graph_ADD);
  RefreshJumpText(&interactive_data, UI_Graph_ADD);
  RefreshSpeedText(&interactive_data, UI_Graph_ADD);
  RefreshDistanceText(&interactive_data, UI_Graph_ADD);
  RefreshRelativePosition(&interactive_data, UI_Graph_ADD);
  RefreshLegPosture(&interactive_data, UI_Graph_ADD);
  RefreshVisionBox(&interactive_data, UI_Graph_ADD);
  RefreshHitCross(&interactive_data, UI_Graph_ADD);
}

Referee_Interactive_info_t *getUI(void) { return &interactive_data; }

void UITask(RobotInstance *robot) {
  referee_recv_info = robot->referee_data;

  if (interactive_data.force_refresh_ui == 1) {
    MyUIInit(robot);
    interactive_data.force_refresh_ui = 0;
  }

  static uint16_t slow_refresh_counter = 0;
  if (++slow_refresh_counter >= 150) {
    slow_refresh_counter = 0;
    SetAllRefreshFlags();
  }

  interactive_data.robot_mode = robot->robot_mode;
  interactive_data.chassis_relative_angle = robot->offset_angle;

  if (robot->referee_data) {
    interactive_data.buffer_energy = robot->referee_data->PowerHeatData.buffer_energy;
    interactive_data.event_type = robot->referee_data->EventData.event_type;
  }

  if (robot->chassis) {
    interactive_data.chassis_mode = robot->chassis->chassis_ctrl_cmd.chassis_mode;
    interactive_data.current_speed = robot->chassis->state_var.x_b_d;
    interactive_data.leg_left_angle = robot->chassis->state_var.theta_l;
    interactive_data.leg_right_angle = robot->chassis->state_var.theta_r;
    interactive_data.jump_state = robot->chassis->jump_state;
    interactive_data.auto_jump_req =
        robot->chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_JUMP_READY ||
        robot->chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_JUMP_START ||
        robot->chassis->jump_state != JUMP_STATE_IDLE;
    if (robot->chassis->super_cap) {
      interactive_data.cap_msg = robot->chassis->super_cap->cap_msg;
      interactive_data.cap_ctrl_cmd = robot->chassis->super_cap->super_cap_ctrl_cmd;
    }
  }

  if (robot->gimbal) {
    interactive_data.gimbal_mode = robot->gimbal->gimbal_ctrl_cmd.gimbal_mode;
  }

  if (robot->shoot) {
    interactive_data.shoot_mode = robot->shoot->shoot_ctrl_cmd.shoot_mode;
    interactive_data.friction_mode = robot->shoot->shoot_ctrl_cmd.friction_mode;
  }

#if !defined(ONE_BOARD)
  if (robot->chassis_fetch_data) {
    if (!robot->gimbal) {
      interactive_data.gimbal_mode = (Gimbal_Mode_e)robot->chassis_fetch_data->ui_gimbal_mode;
    }
    if (!robot->shoot) {
      interactive_data.shoot_mode =
          robot->chassis_fetch_data->ui_friction_mode == FRICTION_ON ? SHOOT_ON : SHOOT_OFF;
      interactive_data.friction_mode = (Friction_Mode_e)robot->chassis_fetch_data->ui_friction_mode;
    }
    interactive_data.chassis_relative_angle =
        (float)robot->chassis_fetch_data->ui_chassis_relative_angle_deg_x10 * 0.1f;
  }
#endif

  interactive_data.vision_tracking = VisionHasTarget(robot->vision_recv_data);
  interactive_data.autoaim_hit =
      robot->vision_recv_data != NULL && robot->vision_recv_data->shoot_receive.fire_flag != 0;

  UIChangeCheck(&interactive_data);
  MyUIRefresh(&interactive_data);
}
