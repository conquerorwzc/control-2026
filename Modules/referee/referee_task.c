/**
 * @file referee_task.c
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"

#include "cmsis_os.h"
#include "math.h"
#include "referee_UI.h"
#include "rm_referee.h"
#include "robot.h"
#include "robot_config.h"
#include "stdio.h"
#include "string.h"

static referee_info_t *referee_recv_info;  // 接收到的裁判系统数据
RobotInstance *robotdata;
uint8_t UI_Seq;  // 包序号，供整个referee文件使用

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID() {
  // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
  referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
  referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
  referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID;  // 计算客户端ID
  referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}


static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_drone_width_line[2]; //车辆示宽线
static Graph_Data_t UI_shoot_dir_circle[3]; //射击中心圆
static Graph_Data_t UI_Energy[3];      // 电容能量条
static String_Data_t UI_State_sta[6];  // 机器人状态,静态只需画一次
static String_Data_t UI_Shoot_sta[3];  //射频和枪口热度,静态只需画一次
static String_Data_t UI_State_dyn[6];  // 机器人状态,动态先add才能change
// static String_Data_t UI_Shoot_dyn[3];  //射频和枪口热度,动态先add才能change
static Graph_Data_t UI_Shoot_dyn[3];  //射频和枪口热度,动态先add才能change
static uint32_t shoot_line_location[10] = {540, 960, 490, 515, 565};

// 定义用于UI更新的数据结构

// 云台俯仰角表盘
static Graph_Data_t UI_pitch_ticks[20];    // 刻度线
static String_Data_t UI_pitch_labels[20];  // 刻度标签
static Graph_Data_t UI_pitch_needle;       // 指针
static String_Data_t UI_pitch_value;       // 当前角度值显示

// 自瞄模式选择器
static Graph_Data_t UI_autoaim_bg;         // 背景圆弧
static Graph_Data_t UI_autoaim_indicator;  // 选择指示圆弧
static String_Data_t UI_autoaim_text;      // 模式文字 "A  B  C"

// 左侧功率/弹量圆弧
static Graph_Data_t UI_cap_arc;          // 电容能量圆弧
static Graph_Data_t UI_ammo_arc;         // 弹量圆弧
static String_Data_t UI_cap_text_E;      // 文字 "E"
static String_Data_t UI_cap_text_F;      // 文字 "F"
static String_Data_t UI_ammo_text_full;  // 文字 "500"
static String_Data_t UI_ammo_text_mid;   // 文字 "250"

// 摩擦轮转速指示器
static Graph_Data_t UI_fric_bg_left;        // 左竖条背景矩形
static Graph_Data_t UI_fric_bg_mid;         // 中竖条背景矩形
static Graph_Data_t UI_fric_bg_right;       // 右竖条背景矩形
static Graph_Data_t UI_fric_pointer_left;   // 左指针线
static Graph_Data_t UI_fric_pointer_mid;    // 中指针线
static Graph_Data_t UI_fric_pointer_right;  // 右指针线
static String_Data_t UI_fric_text_down;     // 文字 "3"
static String_Data_t UI_fric_text_mid;      // 文字 "4"
static String_Data_t UI_fric_text_up;       // 文字 "5"
// 车头方向动态圆弧
static Graph_Data_t UI_yaw_arc;  // 方向指示弧
// --- 自定义右侧扇形仪表 UI 变量 ---
static Graph_Data_t UI_custom_sector_lines[2]; // 扇形的上下两条直边
static Graph_Data_t UI_custom_sector_arc;      // 扇形的外弧线
static Graph_Data_t UI_custom_color_arc[2];    // 扇形外侧边缘的两个彩色区域
static Graph_Data_t UI_custom_pointer[2];      // 两根动态指针 (例如:蓝、绿)
static String_Data_t UI_leg_labels;      // 第一行：LEGL    LEGR
static String_Data_t UI_leg_state_L;     // 第二行左：1/0
static String_Data_t UI_leg_state_R;     // 第二行右：1/0
static Graph_Data_t  UI_leg_value_L;     // 第三行左：数字
static Graph_Data_t  UI_leg_value_R;     // 第三行右：数字
// Vehicle side-view pitch indicator
static Graph_Data_t UI_side_ground;
static Graph_Data_t UI_side_ref_line[5];
static Graph_Data_t UI_side_body;
static Graph_Data_t UI_side_front_link;
static Graph_Data_t UI_side_leg[3];
static Graph_Data_t UI_side_joint[3];
static Graph_Data_t UI_side_front_wheel;
static Graph_Data_t UI_side_rear_wheel;
static String_Data_t UI_side_pitch_text;

static Graph_Data_t Line_DecoMid;
static Graph_Data_t Arc_DecoUp;
static Graph_Data_t Arc_DecoDown;

static Referee_Interactive_info_t interactive_data;

// 检测UI变化的函数
static float UIClampFloat(float value, float min_value, float max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static void DrawRabbitLegSideView(uint32_t x, uint32_t y, uint32_t width, uint32_t height, float pitch_deg,
                                  float left_leg_position, float right_leg_position, uint32_t graph_operate) {
  const float deg_to_rad = 0.0174532925f;
  const float left_leg_horizontal = 1.8636f;
  const float right_leg_horizontal = 1.008f;
  const float left_leg_max = 2.5396f;
  const float right_leg_max = 0.332f;
  const float rear_leg_extension_display_scale = 0.6f;
  uint32_t ground_y = y + 35;
  uint32_t wheel_radius = 15;
  int32_t front_body_x = x + width / 4;
  int32_t rear_wheel_x = x + width * 3 / 4;
  int32_t body_length = rear_wheel_x - front_body_x;
  int32_t rear_wheel_y = ground_y + wheel_radius;
  float left_leg_ratio = (left_leg_position - left_leg_horizontal) / (left_leg_max - left_leg_horizontal);
  float right_leg_ratio = (right_leg_horizontal - right_leg_position) / (right_leg_horizontal - right_leg_max);
  float rear_leg_ratio = UIClampFloat((left_leg_ratio + right_leg_ratio) * 0.5f, 0.0f, 1.0f);
  int32_t front_body_gap = body_length * 15 / 40;
  int32_t rear_body_x = rear_wheel_x;
  int32_t rear_body_y =
      rear_wheel_y + front_body_gap + (int32_t)(rear_leg_ratio * height * rear_leg_extension_display_scale);
  float pitch_rad = pitch_deg * deg_to_rad;
  front_body_x = rear_body_x - (int32_t)(body_length * cosf(pitch_rad));
  int32_t front_body_y = rear_body_y - (int32_t)(body_length * sinf(pitch_rad));
  int32_t front_wheel_x = front_body_x;
  int32_t front_wheel_y = front_body_y - front_body_gap;
  float leg_angle = (44.0f + rear_leg_ratio * 28.0f) * deg_to_rad;
  float leg_len = (float)height * 0.28f;
  int32_t knee_x = rear_wheel_x - (int32_t)(leg_len * cosf(leg_angle));
  int32_t knee_y = (rear_body_y + rear_wheel_y) / 2 + (int32_t)(leg_len * sinf(leg_angle) * 0.25f);
  uint32_t ref_y = rear_wheel_y + front_body_gap;

  if (graph_operate == UI_Graph_ADD) {
    uint32_t dash_len = width / 9;
    UILineDraw(&UI_side_ground, "vg0", graph_operate, 4, UI_Color_Green, 2, x + 12, ground_y, x + width - 12, ground_y);
    for (uint8_t i = 0; i < 5; i++) {
      uint32_t dash_x = x + 22 + i * (dash_len + 18);
      char name[3] = {'v', 'r', (char)('0' + i)};
      UILineDraw(&UI_side_ref_line[i], name, graph_operate, 4, UI_Color_White, 1, dash_x, ref_y, dash_x + dash_len,
                 ref_y);
    }
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_side_ground);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_side_ref_line[0], UI_side_ref_line[1], UI_side_ref_line[2],
                   UI_side_ref_line[3], UI_side_ref_line[4]);
  }

  UILineDraw(&UI_side_body, "vb0", graph_operate, 5, UI_Color_Cyan, 5, front_body_x, front_body_y, rear_body_x,
             rear_body_y);
  UILineDraw(&UI_side_front_link, "vf0", graph_operate, 5, UI_Color_White, 1, front_body_x, front_body_y,
             front_wheel_x, front_wheel_y);
  UILineDraw(&UI_side_leg[0], "vl0", graph_operate, 5, UI_Color_Orange, 3, rear_body_x, rear_body_y, knee_x, knee_y);
  UILineDraw(&UI_side_leg[1], "vl1", graph_operate, 5, UI_Color_Orange, 3, knee_x, knee_y, rear_wheel_x,
             rear_wheel_y);
  UILineDraw(&UI_side_leg[2], "vl2", graph_operate, 5, UI_Color_Orange, 1, rear_body_x, rear_body_y, rear_wheel_x,
             rear_wheel_y);
  UICircleDraw(&UI_side_front_wheel, "vw0", graph_operate, 5, UI_Color_White, 2, front_wheel_x, front_wheel_y,
               wheel_radius);
  UICircleDraw(&UI_side_rear_wheel, "vw1", graph_operate, 5, UI_Color_White, 2, rear_wheel_x, rear_wheel_y,
               wheel_radius);
  UICircleDraw(&UI_side_joint[0], "vj0", graph_operate, 5, UI_Color_Yellow, 2, rear_body_x, rear_body_y, 5);
  UICircleDraw(&UI_side_joint[1], "vj1", graph_operate, 5, UI_Color_Yellow, 2, knee_x, knee_y, 5);
  UICircleDraw(&UI_side_joint[2], "vj2", graph_operate, 5, UI_Color_Yellow, 2, rear_wheel_x, rear_wheel_y, 5);

  int32_t pitch_tenth = (int32_t)(pitch_deg * 10.0f);
  char pitch_sign = '+';
  if (pitch_tenth < 0) {
    pitch_sign = '-';
    pitch_tenth = -pitch_tenth;
  }
  UICharDraw(&UI_side_pitch_text, "vp0", graph_operate, 5, UI_Color_White, 13, 2, x + 12, y + 10,
             "Pitch:%c%d.%ddeg", pitch_sign, pitch_tenth / 10, pitch_tenth % 10);

  UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_side_body, UI_side_front_link, UI_side_leg[0], UI_side_leg[1],
                 UI_side_leg[2]);
  UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_side_front_wheel, UI_side_rear_wheel, UI_side_joint[0],
                 UI_side_joint[1], UI_side_joint[2]);
  UICharRefresh(&referee_recv_info->referee_id, UI_side_pitch_text);
}

static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data) {
  if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode) {
    _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
    _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
  }

  if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode) {
    _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
    _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
  }

  if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode) {
    _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
    _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
  }

  if (_Interactive_data->lid_mode != _Interactive_data->lid_last_mode) {
    _Interactive_data->Referee_Interactive_Flag.lid_flag = 1;
    _Interactive_data->lid_last_mode = _Interactive_data->lid_mode;
  }

  if (_Interactive_data->Chassis_Power_Data.chassis_power_mx !=
      _Interactive_data->Chassis_last_Power_Data.chassis_power_mx) {
    _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
    _Interactive_data->Chassis_last_Power_Data.chassis_power_mx =
        _Interactive_data->Chassis_Power_Data.chassis_power_mx;
  }

  // 俯仰角变化
  if (fabsf(_Interactive_data->pitch_angle - _Interactive_data->last_pitch_angle) > 0.1f) {
    _Interactive_data->Referee_Interactive_Flag.pitch_flag = 1;
    _Interactive_data->last_pitch_angle = _Interactive_data->pitch_angle;
  }

  if (fabsf(_Interactive_data->side_pitch_angle - _Interactive_data->last_side_pitch_angle) > 0.1f) {
    _Interactive_data->Referee_Interactive_Flag.pitch_flag = 1;
    _Interactive_data->last_side_pitch_angle = _Interactive_data->side_pitch_angle;
  }

  if (fabsf(_Interactive_data->left_leg_position - _Interactive_data->last_left_leg_position) > 0.01f ||
      fabsf(_Interactive_data->right_leg_position - _Interactive_data->last_right_leg_position) > 0.01f) {
    _Interactive_data->Referee_Interactive_Flag.pitch_flag = 1;
    _Interactive_data->last_left_leg_position = _Interactive_data->left_leg_position;
    _Interactive_data->last_right_leg_position = _Interactive_data->right_leg_position;
  }

  if (_Interactive_data->autoaim_mode != _Interactive_data->last_autoaim_mode) {
    _Interactive_data->Referee_Interactive_Flag.autoaim_flag = 1;
    _Interactive_data->last_autoaim_mode = _Interactive_data->autoaim_mode;
  }

  if (fabsf(_Interactive_data->cap_voltage - _Interactive_data->last_cap_voltage) > 0.1f
      ) {
    _Interactive_data->Referee_Interactive_Flag.cap_flag = 1;
    _Interactive_data->last_cap_voltage = _Interactive_data->cap_voltage;

  }
  if (_Interactive_data->bullet_left_real != _Interactive_data->last_bullet_left_real) {
    _Interactive_data->Referee_Interactive_Flag.ammo_flag = 1;
    _Interactive_data->last_bullet_left_real = _Interactive_data->bullet_left_real;
  }

  if (_Interactive_data->fric_speed_left != _Interactive_data->last_fric_speed_left ||
    _Interactive_data->fric_speed_mid != _Interactive_data->last_fric_speed_mid ||
    _Interactive_data->fric_speed_right != _Interactive_data->last_fric_speed_right) {
    _Interactive_data->Referee_Interactive_Flag.fric_flag = 1;
    _Interactive_data->last_fric_speed_left = _Interactive_data->fric_speed_left;
    _Interactive_data->last_fric_speed_mid = _Interactive_data->fric_speed_mid;
    _Interactive_data->last_fric_speed_right = _Interactive_data->fric_speed_right;
    }

  if (fabsf(_Interactive_data->chassis_relative_angle - _Interactive_data->last_chassis_relative_angle) > 0.01f) {
    _Interactive_data->Referee_Interactive_Flag.yaw_flag = 1;
    _Interactive_data->last_chassis_relative_angle = _Interactive_data->chassis_relative_angle;
  }
  if (_Interactive_data->legl_flag != _Interactive_data->last_legl_flag ||
        _Interactive_data->legr_flag != _Interactive_data->last_legr_flag ||
        fabsf(_Interactive_data->legl_val - _Interactive_data->last_legl_val) > 0.01f ||
        fabsf(_Interactive_data->legr_val - _Interactive_data->last_legr_val) > 0.01f)
  {
    _Interactive_data->Referee_Interactive_Flag.leg_status_flag = 1; // 需在头文件定义此flag
    _Interactive_data->last_legl_flag = _Interactive_data->legl_flag;
    _Interactive_data->last_legr_flag = _Interactive_data->legr_flag;
    _Interactive_data->last_legl_val = _Interactive_data->legl_val;
    _Interactive_data->last_legr_val = _Interactive_data->legr_val;
  }
  // 检测扇形仪表指针是否变化
  if (fabsf(_Interactive_data->custom_needle1_angle - _Interactive_data->last_custom_needle1_angle) > 0.1f ||
      fabsf(_Interactive_data->custom_needle2_angle - _Interactive_data->last_custom_needle2_angle) > 0.1f) {
    _Interactive_data->Referee_Interactive_Flag.custom_gauge_flag = 1;
    _Interactive_data->last_custom_needle1_angle = _Interactive_data->custom_needle1_angle;
    _Interactive_data->last_custom_needle2_angle = _Interactive_data->custom_needle2_angle;
      }
}

// UI更新函数
static void MyUIRefresh(Referee_Interactive_info_t *interactive_data) {
  // 更新云台状态

    char *gimbal_str;
    switch (interactive_data->heat_mode_e) {
      case NO_CONTROL:
        gimbal_str = "no";
        break;
      case REFEREE_CONTROL:
        gimbal_str = "referee      ";
        break;
      case SIMULLATE_CONTROL:
        gimbal_str = "simulate  ";
        break;
      default:

        break;
    }
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, gimbal_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;


  // 更新射击状态

    char *bullet_str;
    switch (interactive_data->bullet_speed_mode_e) {
      case NO_CONTROL:
        bullet_str = "no";
        break;
      case MANUAL_BULLET_SPEED:
        bullet_str = "man      ";
        break;
      case ENABLE_BULLET_SPEED:
        bullet_str = "en  ";
        break;
      default:

        break;
    }
    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 650, bullet_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    interactive_data->Referee_Interactive_Flag.shoot_flag = 0;


  // 更新摩擦轮状态
  if (interactive_data->Referee_Interactive_Flag.friction_flag == 1) {
    char *friction_str = interactive_data->friction_mode == FRICTION_ON ? "on " : "off";
    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, friction_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    interactive_data->Referee_Interactive_Flag.friction_flag = 0;
  }


    char *cap_str = interactive_data->cap_msg.error_detect == 0? "good " : "bad";
    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, cap_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
    interactive_data->Referee_Interactive_Flag.lid_flag = 0;


  // 自瞄模式指示器
  if (interactive_data->Referee_Interactive_Flag.autoaim_flag == 1) {
    uint32_t start_angle, end_angle;
    switch (interactive_data->autoaim_mode) {
      case 0:
        start_angle = 168;
        end_angle = 180;
        break;  // 左段
      case 1:
        start_angle = 180;
        end_angle = 192;
        break;  // 右段
      default:
        start_angle = 168;
        end_angle = 192;
        break;  // 全段
    }
    UIArcDraw(&UI_autoaim_indicator, "ac0", UI_Graph_Change, 6, UI_Color_Pink, start_angle, end_angle, 14, 960, 540,
              310, 240);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_autoaim_indicator);
    interactive_data->Referee_Interactive_Flag.autoaim_flag = 0;
  }

  // 电容能量圆弧
  if (interactive_data->Referee_Interactive_Flag.cap_flag == 1) {
    float volt = interactive_data->cap_voltage;
    float bar = (volt - 14.0f) / (23.0f - 14.0f) * 40.0f;
    if (bar < 1) bar = 1;
    if (bar > 40) bar = 40;
    uint32_t end_angle = 270 + (uint32_t)bar;
    uint32_t color;
    UIArcDraw(&UI_cap_arc, "ap0", UI_Graph_Change, 6, color, 270, end_angle, 22, 960, 540, 370, 370);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_cap_arc);
    interactive_data->Referee_Interactive_Flag.cap_flag = 0;
  }

  // 弹量圆弧
  if (interactive_data->Referee_Interactive_Flag.ammo_flag == 1) {
    uint16_t bullet = interactive_data->bullet_left_real;
    if (bullet > 500) bullet = 500;
    float bar = (float)bullet / 500.0f * 40.0f;
    uint32_t start_angle = 270 - (uint32_t)bar;
    uint32_t color;
    if (bullet > 80)
      color = UI_Color_Green;
    else if (bullet > 30)
      color = UI_Color_Yellow;
    else if (bullet > 0)
      color = UI_Color_Orange;
    else
      color = UI_Color_Black;
    UIArcDraw(&UI_ammo_arc, "aa0", UI_Graph_Change, 6, color, start_angle, 270, 22, 960, 535, 370, 370);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_ammo_arc);
    interactive_data->Referee_Interactive_Flag.ammo_flag = 0;
  }

  // 显示左侧摩擦轮转速（精确到 100）
  if (interactive_data->Referee_Interactive_Flag.fric_flag == 1) {
    uint16_t left_speed = interactive_data->fric_speed_left;
    uint16_t display_speed = left_speed / 100;  // 26000 -> 260
    char fric_speed_str[16];
    sprintf(fric_speed_str, "frispeed:%d", display_speed);
    UICharDraw(&UI_fric_text_down, "fs0", UI_Graph_Change, 6, UI_Color_White, 18, 2, 1556, 850, fric_speed_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_fric_text_down);
    interactive_data->Referee_Interactive_Flag.fric_flag = 0;
  }

  // 车头方向圆弧
  if (interactive_data->Referee_Interactive_Flag.yaw_flag == 1) {
    int yaw_deg = (int)interactive_data->chassis_relative_angle;
    if (yaw_deg < 0) yaw_deg += 360;
    uint32_t show1, show2;
    if (yaw_deg < 30) {
      show1 = yaw_deg + 30;
      show2 = 360 - (30 - yaw_deg);
    } else if (yaw_deg > 345) {
      show1 = yaw_deg - 330;
      show2 = yaw_deg - 30;
    } else {
      show1 = yaw_deg + 30;
      show2 = yaw_deg - 30;
    }
    uint32_t color;
    switch (interactive_data->chassis_mode) {
      case CHASSIS_FOLLOW:
        color = UI_Color_Pink;
        break;
      case CHASSIS_ROTATE:
        color = UI_Color_Green;
        break;
      default:
        color = UI_Color_Orange;
        break;
    }
    UIArcDraw(&UI_yaw_arc, "yd0", UI_Graph_Change, 6, color, show1, show2, 23, 1556, 721, 88, 88);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_yaw_arc);
    interactive_data->Referee_Interactive_Flag.yaw_flag = 0;
  }

  // 俯仰角仪表盘指针和数值
  if (interactive_data->Referee_Interactive_Flag.pitch_flag == 1) {
    float pitch_deg = interactive_data->pitch_angle;
    float ui_angle = 90.0f + pitch_deg;  // 转换为UI坐标系角度

    while (ui_angle < 0) ui_angle += 360.0f;
    while (ui_angle >= 360) ui_angle -= 360.0f;

    uint32_t start_angle = (uint32_t)(ui_angle - 1);
    uint32_t end_angle = (uint32_t)(ui_angle + 1);

    if (start_angle >= 360) start_angle -= 360;
    if (end_angle >= 360) end_angle -= 360;

    UIArcDraw(&UI_pitch_needle, "pn0", UI_Graph_Change, 6, UI_Color_Pink, start_angle, end_angle, 38, 960, 540, 365,
              365);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pitch_needle);
    DrawRabbitLegSideView(120, 730, 360, 145, interactive_data->side_pitch_angle, interactive_data->left_leg_position,
                          interactive_data->right_leg_position, UI_Graph_Change);
    interactive_data->Referee_Interactive_Flag.pitch_flag = 0;
  }
  if (interactive_data->Referee_Interactive_Flag.leg_status_flag == 1) {
    uint32_t text_x_base = 1556;
    uint32_t text_y_base = 440;
    uint32_t offset_x = 65;

    // 更新左腿 1/0 颜色和字符
    uint32_t color_l = (interactive_data->legl_flag == 1) ? UI_Color_Green : UI_Color_Purplish_red;
    UICharDraw(&UI_leg_state_L, "ls0", UI_Graph_Change, 8, color_l, 20, 3,
               text_x_base - offset_x - 5, text_y_base - 35, "%d", interactive_data->legl_flag);

    // 更新右腿 1/0 颜色和字符
    uint32_t color_r = (interactive_data->legr_flag == 1) ? UI_Color_Green : UI_Color_Purplish_red;
    UICharDraw(&UI_leg_state_R, "ls1", UI_Graph_Change, 8, color_r, 20, 3,
               text_x_base + offset_x - 5, text_y_base - 35, "%d", interactive_data->legr_flag);

    // 更新第三行数值 (乘以1000是因为 UIFloatDraw 内部通常会除以1000处理)
    UIFloatDraw(&UI_leg_value_L, "lv0", UI_Graph_Change, 8, UI_Color_White, 18, 2, 2,
                text_x_base - offset_x - 20, text_y_base - 70, (int32_t)(interactive_data->legl_val * 1000));
    UIFloatDraw(&UI_leg_value_R, "lv1", UI_Graph_Change, 8, UI_Color_White, 18, 2, 2,
                text_x_base + offset_x - 20, text_y_base - 70, (int32_t)(interactive_data->legr_val * 1000));

    // 推送更新
    UICharRefresh(&referee_recv_info->referee_id, UI_leg_state_L);
    UICharRefresh(&referee_recv_info->referee_id, UI_leg_state_R);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_leg_value_L, UI_leg_value_R);

    interactive_data->Referee_Interactive_Flag.leg_status_flag = 0;
  }
  // 刷新扇形仪表动态指针
  if (interactive_data->Referee_Interactive_Flag.custom_gauge_flag == 1) {
    uint32_t cx = 1556;
    uint32_t cy = 600;
    uint32_t needle_len = 115;
    float to_rad = 0.0174532925f;

    // 此时传入的角度应为 RM 角度，90是平，170是最底
    float rm_ang1 = interactive_data->custom_needle1_angle;
    float rm_ang2 = interactive_data->custom_needle2_angle;

    // 限幅防指针越出边框 (90度~170度之间)
    rm_ang1 = UIClampFloat(rm_ang1, 90.0f, 170.0f);
    rm_ang2 = UIClampFloat(rm_ang2, 90.0f, 170.0f);

    // 转换为 math.h 标准弧度
    float rad1 = ((int)(450.0f - rm_ang1) % 360) * to_rad;
    float rad2 = ((int)(450.0f - rm_ang2) % 360) * to_rad;

    UILineDraw(&UI_custom_pointer[0], "cp0", UI_Graph_Change, 7, UI_Color_Cyan, 3,
               cx, cy, cx + (int32_t)(needle_len * cosf(rad1)), cy + (int32_t)(needle_len * sinf(rad1)));

    UILineDraw(&UI_custom_pointer[1], "cp1", UI_Graph_Change, 7, UI_Color_Green, 3,
               cx, cy, cx + (int32_t)(needle_len * cosf(rad2)), cy + (int32_t)(needle_len * sinf(rad2)));

    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_custom_pointer[0], UI_custom_pointer[1]);
    interactive_data->Referee_Interactive_Flag.custom_gauge_flag = 0;
  }
}

/**
 * @brief 初始化俯仰角仪表盘UI组件
 * @param[in] center_x 仪表盘圆心X坐标
 * @param[in] center_y 仪表盘圆心Y坐标
 * @param[in] radius 仪表盘半径
 * @return void
 */
void UIPitchGaugeInit(uint32_t center_x, uint32_t center_y, uint32_t radius) {
  const int main_ticks[] = {50, 70, 90, 110, 130};
  const char *main_labels[] = {"50", "70", "90", "110", "130"};
  const int sub_ticks[] = {60, 80, 100, 120};

  for (int i = 0; i < 5; i++) {
    float ui_angle = 90.0f - (float)main_ticks[i];
    float angle_rad = ui_angle * 3.1415926535f / 180.0f;
    uint32_t tick_length = 27;

    int32_t start_x = center_x + (int32_t)(radius * cosf(angle_rad));
    int32_t start_y = center_y - (int32_t)(radius * sinf(angle_rad));
    int32_t end_x = center_x + (int32_t)((radius - tick_length) * cosf(angle_rad));
    int32_t end_y = center_y - (int32_t)((radius - tick_length) * sinf(angle_rad));

    char graph_name[3] = {0};
    graph_name[0] = 'p';
    graph_name[1] = 't';
    graph_name[2] = '0' + i;

    UILineDraw(&UI_pitch_ticks[i], graph_name, UI_Graph_ADD, 8, UI_Color_White, 7, start_x, start_y, end_x, end_y);

    int32_t label_x, label_y;
    int32_t label_distance = radius - tick_length - 25;
    int32_t base_x = center_x + (int32_t)(label_distance * cosf(angle_rad));
    int32_t base_y = center_y - (int32_t)(label_distance * sinf(angle_rad));
    int32_t text_width = 18;
    int32_t text_height = 12;
    float cos_val = cosf(angle_rad);
    float sin_val = sinf(angle_rad);

    if (cos_val > 0.7f) {
      label_x = base_x - text_width;
    } else if (cos_val < -0.7f) {
      label_x = base_x;
    } else if (cos_val > 0.3f) {
      label_x = base_x - text_width / 2;
    } else if (cos_val < -0.3f) {
      label_x = base_x - text_width / 2;
    } else {
      label_x = base_x - text_width / 2;
    }

    if (sin_val > 0.7f) {
      label_y = base_y;
    } else if (sin_val < -0.7f) {
      label_y = base_y - text_height;
    } else if (sin_val > 0.3f) {
      label_y = base_y - text_height / 2;
    } else if (sin_val < -0.3f) {
      label_y = base_y - text_height / 2;
    } else {
      label_y = base_y - text_height / 2;
    }

    switch (main_ticks[i]) {
      case 50:
        label_x -= 2;
        label_y += 4;
        break;
      case 70:
        label_x -= 1;
        label_y += 2;
        break;
      case 90:
        break;
      case 110:
        label_x -= 1;
        label_y -= 2;
        break;
      case 130:
        label_x -= 2;
        label_y -= 4;
        break;
    }

    char label_name[3] = {0};
    label_name[0] = 'p';
    label_name[1] = 'l';
    label_name[2] = '0' + i;

    UICharDraw(&UI_pitch_labels[i], label_name, UI_Graph_ADD, 8, UI_Color_White, 12, 2, label_x, label_y,
               main_labels[i]);
  }

  for (int i = 0; i < 4; i++) {
    float ui_angle = 90.0f - (float)sub_ticks[i];
    float angle_rad = ui_angle * 3.1415926535f / 180.0f;
    uint32_t tick_length = 13;

    int32_t start_x = center_x + (int32_t)(radius * cosf(angle_rad));
    int32_t start_y = center_y - (int32_t)(radius * sinf(angle_rad));
    int32_t end_x = center_x + (int32_t)((radius - tick_length) * cosf(angle_rad));
    int32_t end_y = center_y - (int32_t)((radius - tick_length) * sinf(angle_rad));

    char graph_name[3] = {0};
    graph_name[0] = 'p';
    graph_name[1] = 's';
    graph_name[2] = '0' + i;

    static Graph_Data_t UI_sub_ticks[4];
    UILineDraw(&UI_sub_ticks[i], graph_name, UI_Graph_ADD, 8, UI_Color_White, 7, start_x, start_y, end_x, end_y);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_sub_ticks[i]);
  }

  for (int i = 0; i < 5; i++) {
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pitch_ticks[i]);
    UICharRefresh(&referee_recv_info->referee_id, UI_pitch_labels[i]);
  }
}

void MyUIInit() {
  RobotInstance *robot = getRobot();
  referee_recv_info = robot->referee_data;

  while (referee_recv_info->GameRobotState.robot_id == 0) osDelay(100);
  DeterminRobotID();
  UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0);
   // 绘制发射基准线
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 2, CENTER_X - 300, CENTER_Y, CENTER_X + 300, CENTER_Y);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 1, CENTER_X - 50, CENTER_Y + Aim_Line_1, CENTER_X + 50, CENTER_Y + Aim_Line_1);
    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_White, 1, CENTER_X - 80, CENTER_Y + Aim_Line_2, CENTER_X + 80, CENTER_Y + Aim_Line_2);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_White, 1, CENTER_X - 120, CENTER_Y + Aim_Line_3, CENTER_X + 120, CENTER_Y + Aim_Line_3);
    UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_White, 1, CENTER_X - 200, CENTER_Y + Aim_Line_4, CENTER_X + 200, CENTER_Y + Aim_Line_4);
    UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_White, 1, CENTER_X, 300, CENTER_X, 650);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4]);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_shoot_line[5]);

    // 绘制车辆示宽线
    UILineDraw(&UI_drone_width_line[0], "sl6", UI_Graph_ADD, 7, UI_Color_Green, 2, 960 - WIDTHLINE_UP, 320, 960 - WIDTHLINE_DOWN, 0);
    UILineDraw(&UI_drone_width_line[1], "sl7", UI_Graph_ADD, 7, UI_Color_Green, 2, 960 + WIDTHLINE_DOWN, 0, 960 + WIDTHLINE_UP, 320);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_drone_width_line[0], UI_drone_width_line[1]);

    // 绘制发射中心圆
    UICircleDraw(&UI_shoot_dir_circle[0], "sc0", UI_Graph_ADD, 7, UI_Color_White, 3, 960, 540, 15);
    //绘制车头方向指示的中心圆
    UICircleDraw(&UI_shoot_dir_circle[1], "sc1", UI_Graph_ADD, 7, UI_Color_White, 3, 1556, 721, 76);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_dir_circle[0],  UI_shoot_dir_circle[1]);

    // 绘制车辆状态标志指示
    // UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 750, "chassis:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 700, "heat:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 650, "bullet:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 600, "frict:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 550, "supercap:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);


    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    // UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 750, "PowerOff");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 700, "PowerOff");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 650, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 600, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 550, "close ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);

    //绘制枪口热度，射频指示
    // UICharDraw(&UI_Shoot_sta[0], "ss5", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 1500, 750, "SHOOT_HEAT:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_Shoot_sta[0]);
    // UICharDraw(&UI_Shoot_sta[1], "ss6", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 1500, 700, "SHOOT_RATE:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_Shoot_sta[1]);

    //绘制枪口热度，射频指示，动态
    // UICharDraw(&UI_Shoot_dyn[0], "sd7", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 1700, 750, "0");
    // UIFloatDraw(&UI_Shoot_dyn[0], "sd7", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 2, 1700, 750, 0);
    // UICharRefresh(&referee_recv_info->referee_id, UI_Shoot_dyn[0]);
    // UICharDraw(&UI_Shoot_dyn[1], "sd8", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 1700, 700, "0");
    // UIFloatDraw(&UI_Shoot_dyn[1], "sd8", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 2, 1700, 700, 0);
    // UICharRefresh(&referee_recv_info->referee_id, UI_Shoot_dyn[1]);

    // 底盘功率显示，静态
    // UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 620, 230, "Power:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);
    // 能量条框
    // UIRectangleDraw(&UI_Energy[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1220, 180);
    // UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);

    // 底盘功率显示,动态
    // UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 750, 230, 24000);
    // 能量条初始状态
    // UILineDraw(&UI_Energy[2], "sd6", UI_Graph_ADD, 8, UI_Color_Pink, 30, 720, 160, 1020, 160);
    // UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);

    UIPitchGaugeInit(960, 540, 390);

    // 自瞄模式选择器 (假设有3个模式，显示 "A  B  C")
    UIArcDraw(&UI_autoaim_bg, "ab0", UI_Graph_ADD, 5, UI_Color_White, 168, 192, 14, 960, 540, 310, 240);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_autoaim_bg);
    UIArcDraw(&UI_autoaim_indicator, "ac0", UI_Graph_ADD, 6, UI_Color_Pink, 168, 180, 14, 960, 540, 310, 240);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_autoaim_indicator);
    UICharDraw(&UI_autoaim_text, "at0", UI_Graph_ADD, 5, UI_Color_White, 18, 3, 900, 280, "A  B  C");
    UICharRefresh(&referee_recv_info->referee_id, UI_autoaim_text);

    // 左侧功率/弹量圆弧
    UIArcDraw(&UI_cap_arc, "ap0", UI_Graph_ADD, 6, UI_Color_Pink, 270, 310, 22, 960, 540, 370, 370);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_cap_arc);
    // UIArcDraw(&UI_ammo_arc, "aa0", UI_Graph_ADD, 6, UI_Color_Main, 230, 270, 22, 960, 535, 370, 370);
    // UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_ammo_arc);
    UICharDraw(&UI_cap_text_E, "ce0", UI_Graph_ADD, 6, UI_Color_White, 17, 2, 610, 545, "E");
    UICharRefresh(&referee_recv_info->referee_id, UI_cap_text_E);
    UICharDraw(&UI_cap_text_F, "cf0", UI_Graph_ADD, 6, UI_Color_White, 17, 2, 702, 775, "F");
    UICharRefresh(&referee_recv_info->referee_id, UI_cap_text_F);
    UICharDraw(&UI_ammo_text_full, "af0", UI_Graph_ADD, 6, UI_Color_White, 12, 2, 697, 300, "500");
    UICharRefresh(&referee_recv_info->referee_id, UI_ammo_text_full);
    UICharDraw(&UI_ammo_text_mid, "am0", UI_Graph_ADD, 6, UI_Color_White, 12, 2, 630, 420, "250");
    UICharRefresh(&referee_recv_info->referee_id, UI_ammo_text_mid);
  // --- 腿部状态面板初始化 ---
  uint32_t text_x_base = 1556;
  uint32_t text_y_base = 440; // 位于扇形下方
  uint32_t offset_x = 65;     // 左右间距偏移

  // 第一行：静态标签
  UICharDraw(&UI_leg_labels, "ll0", UI_Graph_ADD, 8, UI_Color_White, 15, 2,
             text_x_base - 75, text_y_base, "LEGL      LEGR");
  UICharRefresh(&referee_recv_info->referee_id, UI_leg_labels);

  // 第二行：标志位初始状态（默认红色0）
  UICharDraw(&UI_leg_state_L, "ls0", UI_Graph_ADD, 8, UI_Color_Purplish_red, 20, 3,
             text_x_base - offset_x - 5, text_y_base - 35, "0");
  UICharDraw(&UI_leg_state_R, "ls1", UI_Graph_ADD, 8, UI_Color_Purplish_red, 20, 3,
             text_x_base + offset_x - 5, text_y_base - 35, "0");
  UICharRefresh(&referee_recv_info->referee_id, UI_leg_state_L);
  UICharRefresh(&referee_recv_info->referee_id, UI_leg_state_R);

  // 第三行：数值初始状态（使用 Float 类型减少带宽消耗）
  UIFloatDraw(&UI_leg_value_L, "lv0", UI_Graph_ADD, 8, UI_Color_White, 18, 2, 2,
              text_x_base - offset_x - 20, text_y_base - 70, 0);
  UIFloatDraw(&UI_leg_value_R, "lv1", UI_Graph_ADD, 8, UI_Color_White, 18, 2, 2,
              text_x_base + offset_x - 20, text_y_base - 70, 0);
  UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_leg_value_L, UI_leg_value_R);
    // 摩擦轮转速指示器
  // 初始化显示左侧摩擦轮转速
  UICharDraw(&UI_fric_text_down, "fs0", UI_Graph_ADD, 6, UI_Color_White, 18, 2, 1556, 850, "frispeed:0");
  UICharRefresh(&referee_recv_info->referee_id, UI_fric_text_down);

    // 车头方向动态圆弧
    UIArcDraw(&UI_yaw_arc, "yd0", UI_Graph_ADD, 6, UI_Color_Main, 15, 345, 23, 1556, 721, 88, 88);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_yaw_arc);

    // 俯仰角仪表盘指针
    // 绘制指针初始位置（指向90°）
    UIArcDraw(&UI_pitch_needle, "pn0", UI_Graph_ADD, 6, UI_Color_Pink, 90 - 1, 90 + 1, 45, 960, 540, 365, 365);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pitch_needle);
    DrawRabbitLegSideView(120, 730, 360, 145, 0.0f, 2.922f, 1.008f, UI_Graph_ADD);

    UILineDraw(&Line_DecoMid, "ll0", UI_Graph_ADD, 6, UI_Color_White, 7, 577, 538, 606, 538);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, Line_DecoMid);
    UIArcDraw(&Arc_DecoUp, "ll1", UI_Graph_ADD, 6, UI_Color_White, 310, 311, 22, 960, 540, 370, 370);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, Arc_DecoUp);
    UIArcDraw(&Arc_DecoDown, "ll2", UI_Graph_ADD, 6, UI_Color_White, 229, 230, 22, 960, 535, 370, 370);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, Arc_DecoDown);

  // ==========================================
    // 绘制右侧自定义扇形仪表 (已修正坐标系映射)
    // ==========================================
    uint32_t cx = 1556;
    uint32_t cy = 600;
    uint32_t r = 130;
    float to_rad = 0.0174532925f;

    // 【统一定义角度】：使用 RM UI 角度系统
    // 0度为正上方，顺时针增加。90度为正右，180度为正下
    uint32_t arc_start = 90;  // 对应水平线 (up)
    uint32_t arc_end = 170;   // 对应斜向下线 (down)

    // 1. 绘制扇形基础外圈圆弧 (完美匹配 RM 坐标系)
    UIArcDraw(&UI_custom_sector_arc, "ca0", UI_Graph_ADD, 5, UI_Color_White, arc_start, arc_end, 2, cx, cy, r, r);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_custom_sector_arc);

    // 2. 绘制外部彩色警告区域 (覆盖在基础圆弧上)
    // 颜色段1：最下方 (155°~170°) - 对应你草图黄色的位置
    UIArcDraw(&UI_custom_color_arc[0], "cc0", UI_Graph_ADD, 6, UI_Color_Yellow, 155, 170, 6, cx, cy, r, r);
    // 颜色段2：靠下方 (140°~155°) - 对应你草图绿色的位置
    UIArcDraw(&UI_custom_color_arc[1], "cc1", UI_Graph_ADD, 6, UI_Color_Green, 140, 155, 6, cx, cy, r, r);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_custom_color_arc[0], UI_custom_color_arc[1]);

    // 3. 绘制扇形的上下两条静态直边 (通过转换公式适配 math.h)
    // 转换公式: C语言数学弧度 = (450 - RM角度) % 360 * 弧度转换系数
    float line_start_rad = ((450 - arc_start) % 360) * to_rad;
    float line_end_rad   = ((450 - arc_end) % 360) * to_rad;

    UILineDraw(&UI_custom_sector_lines[0], "cg0", UI_Graph_ADD, 5, UI_Color_White, 2,
               cx, cy, cx + (int32_t)(r * cosf(line_start_rad)), cy + (int32_t)(r * sinf(line_start_rad)));
    UILineDraw(&UI_custom_sector_lines[1], "cg1", UI_Graph_ADD, 5, UI_Color_White, 2,
               cx, cy, cx + (int32_t)(r * cosf(line_end_rad)), cy + (int32_t)(r * sinf(line_end_rad)));
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_custom_sector_lines[0], UI_custom_sector_lines[1]);

    // 4. 初始化两根动态指针 (假设初始停在 110度 和 130度)
    uint32_t needle_len = 115;
    float p1_rad = ((450 - 110) % 360) * to_rad;
    float p2_rad = ((450 - 130) % 360) * to_rad;

    UILineDraw(&UI_custom_pointer[0], "cp0", UI_Graph_ADD, 7, UI_Color_Cyan, 3,
               cx, cy, cx + (int32_t)(needle_len * cosf(p1_rad)), cy + (int32_t)(needle_len * sinf(p1_rad)));
    UILineDraw(&UI_custom_pointer[1], "cp1", UI_Graph_ADD, 7, UI_Color_Green, 3,
               cx, cy, cx + (int32_t)(needle_len * cosf(p2_rad)), cy + (int32_t)(needle_len * sinf(p2_rad)));
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_custom_pointer[0], UI_custom_pointer[1]);
}

void UITask() {
  RobotInstance *robot = getRobot();
  referee_recv_info = robot->referee_data;

  // 【1】按键强制清屏重绘（仅当按下 Ctrl 时触发）
  if (interactive_data.force_refresh_ui == 1) {
    MyUIInit();  // 只有手动按下 Ctrl，才允许执行全屏 Delete 和静态线 ADD
    interactive_data.force_refresh_ui = 0;
  }

  // 【2】防丢包心跳保底（不删图层，只重发 CHANGE）
  static uint16_t slow_refresh_counter = 0;
  if (++slow_refresh_counter >= 150) {
    slow_refresh_counter = 0;
    // 仅仅把所有动态标志位置 1，逼迫 MyUIRefresh 重发一遍最新参数
    interactive_data.Referee_Interactive_Flag.gimbal_flag = 1;
    interactive_data.Referee_Interactive_Flag.shoot_flag = 1;
    interactive_data.Referee_Interactive_Flag.friction_flag = 1;
    interactive_data.Referee_Interactive_Flag.lid_flag = 1;
    interactive_data.Referee_Interactive_Flag.cap_flag = 1;
    interactive_data.Referee_Interactive_Flag.ammo_flag = 1;
    interactive_data.Referee_Interactive_Flag.fric_flag = 1;
    interactive_data.Referee_Interactive_Flag.yaw_flag = 1;
    interactive_data.Referee_Interactive_Flag.pitch_flag = 1;
    interactive_data.Referee_Interactive_Flag.autoaim_flag = 1;
  }

  // 【3】核心修复：必须把真实数据喂给 interactive_data，UIChangeCheck 才能正常工作！
  // 基础状态
  interactive_data.chassis_mode = robot->chassis->chassis_ctrl_cmd.chassis_mode;
  interactive_data.gimbal_mode = robot->gimbal->gimbal_ctrl_cmd.gimbal_mode;
  interactive_data.shoot_mode = robot->shoot->shoot_ctrl_cmd.shoot_mode;
  interactive_data.friction_mode = robot->shoot->shoot_ctrl_cmd.friction_mode;
  interactive_data.bullet_speed_mode_e=robot->shoot->shoot_ctrl_cmd.bullet_speed_mode;
  interactive_data.heat_mode_e=robot->shoot->shoot_ctrl_cmd.heat_mode;
  interactive_data.cap_voltage = robot->super_cap->cap_msg.cap_v;

  // 动态数值（如果这里的变量名和你的底层解算名字不一致，请手动微调一下）
  interactive_data.pitch_angle = robot->gimbal->gimbal_ctrl_cmd.pitch;                      // 俯仰角
  interactive_data.side_pitch_angle =
      robot->chassis->chassis_external_imu != NULL ? robot->chassis->chassis_external_imu->roll : 0.0f;
  interactive_data.chassis_relative_angle = robot->chassis->chassis_ctrl_cmd.offset_angle;  // 底盘朝向
  interactive_data.left_leg_position =
      robot->chassis->leg_motor[0] != NULL ? robot->chassis->leg_motor[0]->measure.position : 1.8636f;
  interactive_data.right_leg_position =
      robot->chassis->leg_motor[1] != NULL ? robot->chassis->leg_motor[1]->measure.position : 1.008f;
  interactive_data.bullet_left_real =
      referee_recv_info->ProjectileAllowance.projectile_allowance_17mm;  // 剩余金币或弹量

  // 自瞄模式简易判断 (根据 gimbal_mode)
  interactive_data.autoaim_mode = (interactive_data.gimbal_mode == GIMBAL_VISION) ? 1 : 0;

  // TODO: 摩擦轮真实转速需要你从电机 measure 里拿，这里暂时用给定的指令值代替演示，你可以自己换成真实反馈值
  interactive_data.fric_speed_left = robot->shoot->friction_motor[0]->measure.speed_aps;
  interactive_data.fric_speed_mid = robot->shoot->friction_motor[1]->measure.speed_aps;
  interactive_data.fric_speed_right = robot->shoot->friction_motor[2]->measure.speed_aps;
  // (接在你现有的赋值逻辑下面)
  interactive_data.legl_flag = robot->chassis->leg_motor[0]->measure.state==1;
  interactive_data.legr_flag = robot->chassis->leg_motor[1]->measure.state==1;
  interactive_data.legl_val = robot->chassis->leg_motor[0]->measure.torque;
  interactive_data.legr_val = robot->chassis->leg_motor[1]->measure.torque;
  // 这里接入你的实际数据，并将数值映射到 90~170 度之间
  if (robot->chassis->chassis_ctrl_cmd.leg_limit == FIRST_STEP) {
    interactive_data.custom_needle1_angle = 170.0f ; // 测试数据：停在中间
  }
  if (robot->chassis->chassis_ctrl_cmd.leg_limit == SECOND_STEP) {
    interactive_data.custom_needle1_angle = 140.0f ; // 测试数据：停在中间
  }
  float needle2_value = robot->chassis->chassis_ctrl_cmd.leg_theta;
  if (needle2_value < 0.35f) needle2_value = 0.35f;
  if (needle2_value > 1.1f) needle2_value = 1.1f;
  interactive_data.custom_needle2_angle =  (needle2_value - 0.35f) * (80.0f / 0.75f) + 90.0f; // 测试数据：靠近下方彩色区


  // 检查是否有变化
  UIChangeCheck(&interactive_data);

  // 执行UI刷新
  MyUIRefresh(&interactive_data);
}

Referee_Interactive_info_t *getUI() { return &interactive_data; }
