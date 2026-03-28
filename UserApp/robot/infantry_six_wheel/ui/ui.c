#include "ui.h"
#include "cmsis_os.h"
#include "referee_ui.h"
#include "referee.h"
#include "robot.h"
#include "bsp_log.h"

// 接收到的裁判系统数据
static referee_info_t *referee_recv_info;
static RobotInstance *robotdata;

// 提前声明
static void DeterminRobotID(void);

// UI 变量定义
static Graph_Data_t UI_shoot_line[10];       // 射击准线
static Graph_Data_t UI_drone_width_line[2];  // 车辆示宽线
static Graph_Data_t UI_shoot_dir_circle[3];  // 射击中心圆
static Graph_Data_t UI_Energy[3];            // 电容能量条
static String_Data_t UI_State_sta[6];        // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[6];        // 机器人状态,动态先add才能change
static Graph_Data_t UI_Shoot_dyn[3];  // 射频和枪口热度,动态先add才能change

// 云台俯仰角表盘
static Graph_Data_t UI_pitch_ticks[20];    // 刻度线
static String_Data_t UI_pitch_labels[20];  // 刻度标签
static Graph_Data_t UI_pitch_needle;       // 指针

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
static Graph_Data_t UI_fric_bg_right;       // 右竖条背景矩形
static Graph_Data_t UI_fric_pointer_left;   // 左指针线
static Graph_Data_t UI_fric_pointer_right;  // 右指针线
static String_Data_t UI_fric_text_down;     // 文字 "5"
static String_Data_t UI_fric_text_mid;      // 文字 "6"
static String_Data_t UI_fric_text_up;       // 文字 "7"

// 车头方向动态圆弧
static Graph_Data_t UI_yaw_arc;  // 方向指示弧

static Graph_Data_t Line_DecoMid;
static Graph_Data_t Arc_DecoUp;
static Graph_Data_t Arc_DecoDown;

// 交互数据
static Referee_Interactive_info_t interactive_data;

// 常量定义
#define CENTER_X 960
#define CENTER_Y 540
#define Aim_Line_1 40
#define Aim_Line_2 80
#define Aim_Line_3 130
#define Aim_Line_4 220
#define WIDTHLINE_UP 100
#define WIDTHLINE_DOWN 140
#define FRIC_UPPER 5000 // 假设值，需根据实际情况调整
#define FRIC_LOWER 3000 // 假设值

// 检测UI变化的函数
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data) {
  if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode) {
    _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
    _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
  }

  if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode) {
    _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
    _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
  }

  if (_Interactive_data->fric_mode != _Interactive_data->fric_last_mode) {
    _Interactive_data->Referee_Interactive_Flag.fric_flag = 1;
    _Interactive_data->fric_last_mode = _Interactive_data->fric_mode;
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

  if (_Interactive_data->autoaim_mode != _Interactive_data->last_autoaim_mode) {
    _Interactive_data->Referee_Interactive_Flag.autoaim_flag = 1;
    _Interactive_data->last_autoaim_mode = _Interactive_data->autoaim_mode;
  }

  if (fabsf(_Interactive_data->cap_voltage - _Interactive_data->last_cap_voltage) > 0.1f ||
      _Interactive_data->cap_mode != _Interactive_data->last_cap_mode) {
    _Interactive_data->Referee_Interactive_Flag.cap_flag = 1;
    _Interactive_data->last_cap_voltage = _Interactive_data->cap_voltage;
    _Interactive_data->last_cap_mode = _Interactive_data->cap_mode;
  }
  if (_Interactive_data->bullet_left_real != _Interactive_data->last_bullet_left_real) {
    _Interactive_data->Referee_Interactive_Flag.ammo_flag = 1;
    _Interactive_data->last_bullet_left_real = _Interactive_data->bullet_left_real;
  }

  if (_Interactive_data->fric_speed_left != _Interactive_data->last_fric_speed_left ||
      _Interactive_data->fric_speed_right != _Interactive_data->last_fric_speed_right) {
    _Interactive_data->Referee_Interactive_Flag.fric_flag = 1;
    _Interactive_data->last_fric_speed_left = _Interactive_data->fric_speed_left;
    _Interactive_data->last_fric_speed_right = _Interactive_data->fric_speed_right;
  }
  if (fabsf(_Interactive_data->chassis_relative_angle - _Interactive_data->last_chassis_relative_angle) > 0.01f) {
    _Interactive_data->Referee_Interactive_Flag.yaw_flag = 1;
    _Interactive_data->last_chassis_relative_angle = _Interactive_data->chassis_relative_angle;
  }
}

static void DeterminRobotID(void) {
  // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
  referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
  referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
  referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID;  // 计算客户端ID
  referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

// UI更新函数
static void MyUIRefresh(Referee_Interactive_info_t *interactive_data) {
  // 更新云台状态
  if (interactive_data->Referee_Interactive_Flag.gimbal_flag == 1) {
    char *gimbal_str;
    switch (interactive_data->gimbal_mode) {
      case GIMBAL_POWER_OFF:
        gimbal_str = "PowerOff";
        break;
      case GIMBAL_ON:
        gimbal_str = "On      ";
        break;
      case GIMBAL_VISION:
        gimbal_str = "Vision  ";
        break;
      default:
        gimbal_str = "unknown ";
        break;
    }
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, gimbal_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
  }

  // 更新射击状态
  if (interactive_data->Referee_Interactive_Flag.shoot_flag == 1) {
    char *shoot_str = interactive_data->shoot_mode == SHOOT_ON ? "on " : "off";
    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 650, shoot_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
  }

  // 更新摩擦轮状态
  if (interactive_data->Referee_Interactive_Flag.fric_flag == 1) {
    char *fric_str = interactive_data->fric_mode == FRICTION_ON ? "on " : "off";
    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, fric_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    interactive_data->Referee_Interactive_Flag.fric_flag = 0;
  }

  // 更新弹舱盖状态
  if (interactive_data->Referee_Interactive_Flag.lid_flag == 1) {
    char *lid_str = interactive_data->lid_mode == LID_OPEN ? "open " : "close";
    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, lid_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
    interactive_data->Referee_Interactive_Flag.lid_flag = 0;
  }

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
    uint8_t mode = interactive_data->cap_mode;
    // 计算弧长（假设电压范围 16V ~ 23V 对应 0~40度）
    float bar = (volt - 16.0f) / (23.0f - 16.0f) * 40.0f;
    if (bar < 1) bar = 1;
    if (bar > 40) bar = 40;
    uint32_t end_angle = 270 + (uint32_t)bar;
    uint32_t color;
    switch (mode) {
      case 0:
        color = UI_Color_Pink;
        break;  // 关闭
      case 1:
        color = UI_Color_Green;
        break;  // 开启
      case 2:
        color = UI_Color_Cyan;
        break;  // 超级电容
      default:
        color = UI_Color_Orange;
        break;
    }
    UIArcDraw(&UI_cap_arc, "ap0", UI_Graph_Change, 6, color, 270, end_angle, 22, 960, 540, 370, 370);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_cap_arc);
    interactive_data->Referee_Interactive_Flag.cap_flag = 0;
  }

  // 弹量圆弧
  if (interactive_data->Referee_Interactive_Flag.ammo_flag == 1) {
    uint16_t bullet = interactive_data->bullet_left_real;
    if (bullet > 500) bullet = 500;  // 上限500发
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

  // 摩擦轮指针
  if (interactive_data->Referee_Interactive_Flag.fric_flag == 1) {
    uint16_t left_speed = interactive_data->fric_speed_left;    // 左摩擦轮转速
    uint16_t right_speed = interactive_data->fric_speed_right;  // 右摩擦轮转速

    // 左指针绘制
    uint32_t left_y1, left_y2;
    uint32_t left_color = UI_Color_Green;

    if (left_speed < FRIC_LOWER) {
      left_y1 = 665;
      left_y2 = 695;
      left_color = UI_Color_Yellow;
    } else if (left_speed > FRIC_UPPER) {
      left_y1 = 755;
      left_y2 = 785;
      left_color = UI_Color_Yellow;
    } else {
      float ratio = (float)(left_speed - FRIC_LOWER) / (float)(FRIC_UPPER - FRIC_LOWER);
      uint32_t base = 665 + (uint32_t)(ratio * 120.0f);
      left_y1 = base - 3;
      left_y2 = base + 3;
    }
    UILineDraw(&UI_fric_pointer_left, "fl1", UI_Graph_Change, 7, left_color, 22, 1526, left_y1, 1526, left_y2);

    // 右指针绘制
    uint32_t right_y1, right_y2;
    uint32_t right_color = UI_Color_Green;

    if (right_speed < FRIC_LOWER) {
      right_y1 = 665;
      right_y2 = 695;
      right_color = UI_Color_Yellow;
    } else if (right_speed > FRIC_UPPER) {
      right_y1 = 755;
      right_y2 = 785;
      right_color = UI_Color_Yellow;
    } else {
      float ratio = (float)(right_speed - FRIC_LOWER) / (float)(FRIC_UPPER - FRIC_LOWER);
      uint32_t base = 665 + (uint32_t)(ratio * 120.0f);
      right_y1 = base - 3;
      right_y2 = base + 3;
    }
    UILineDraw(&UI_fric_pointer_right, "fr1", UI_Graph_Change, 7, right_color, 22, 1590, right_y1, 1590, right_y2);

    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_fric_pointer_left, UI_fric_pointer_right);
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
      case ROBOT_CHASSIS_PROSTRATE_FOLLOW:
        color = UI_Color_Pink;
        break;
      case ROBOT_CHASSIS_PROSTRATE_ROTATE:
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

  // 俯仰角仪表盘指针
  if (interactive_data->Referee_Interactive_Flag.pitch_flag == 1) {
    float pitch_deg = interactive_data->pitch_angle;
    float ui_angle = 90.0f - pitch_deg;

    while (ui_angle < 0) ui_angle += 360.0f;
    while (ui_angle >= 360) ui_angle -= 360.0f;

    uint32_t start_angle = (uint32_t)(ui_angle - 1);
    uint32_t end_angle = (uint32_t)(ui_angle + 1);
    if (start_angle >= 360) start_angle -= 360;
    if (end_angle >= 360) end_angle -= 360;

    UIArcDraw(&UI_pitch_needle, "pn0", UI_Graph_Change, 6, UI_Color_Pink, start_angle, end_angle, 38, 960, 540, 365,
              365);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pitch_needle);
    interactive_data->Referee_Interactive_Flag.pitch_flag = 0;
  }
}

// 初始化俯仰角仪表盘UI组件
static void UIPitchGaugeInit(uint32_t center_x, uint32_t center_y, uint32_t radius) {
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

    if (cos_val > 0.7f) label_x = base_x - text_width;
    else if (cos_val < -0.7f) label_x = base_x;
    else label_x = base_x - text_width / 2;

    if (sin_val > 0.7f) label_y = base_y;
    else if (sin_val < -0.7f) label_y = base_y - text_height;
    else label_y = base_y - text_height / 2;

    switch (main_ticks[i]) {
      case 50: label_x -= 2; label_y += 4; break;
      case 70: label_x -= 1; label_y += 2; break;
      case 110: label_x -= 1; label_y -= 2; break;
      case 130: label_x -= 2; label_y -= 4; break;
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

// 机器人特定的UI初始化
void MyUIInit(void* robot) {
  if (robot == NULL) return;
  robotdata = (RobotInstance*)robot;
  referee_recv_info = robotdata->referee_data;
  
  osThreadDef(uitask, UITask, osPriorityNormal, 0, 512);
  osThreadCreate(osThread(uitask), robot);
}

// 机器人特定的UI任务
void UITask(void const *argument) {
  if (robotdata == NULL) {
    robotdata = (RobotInstance*)argument;
  }
  if (robotdata != NULL) {
    referee_recv_info = robotdata->referee_data;
  }
  
  if (referee_recv_info == NULL) return;

  if (!referee_recv_info->init_flag) {
     while (!referee_recv_info->init_flag)
      osDelay(100);
  }

  while (referee_recv_info->GameRobotState.robot_id == 0)
      osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查
  DeterminRobotID();                                            // 确定ui要发送到的目标客户端
  UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

  // 绘制发射基准线
  UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 2, CENTER_X - 300, CENTER_Y, CENTER_X + 300,
             CENTER_Y);
  UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 1, CENTER_X - 50, CENTER_Y + Aim_Line_1,
             CENTER_X + 50, CENTER_Y + Aim_Line_1);
  UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_White, 1, CENTER_X - 80, CENTER_Y + Aim_Line_2,
             CENTER_X + 80, CENTER_Y + Aim_Line_2);
  UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_White, 1, CENTER_X - 120, CENTER_Y + Aim_Line_3,
             CENTER_X + 120, CENTER_Y + Aim_Line_3);
  UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_White, 1, CENTER_X - 200, CENTER_Y + Aim_Line_4,
             CENTER_X + 200, CENTER_Y + Aim_Line_4);
  UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_White, 1, CENTER_X, 300, CENTER_X, 650);
  UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2],
                 UI_shoot_line[3], UI_shoot_line[4]);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_shoot_line[5]);

  // 绘制车辆示宽线
  UILineDraw(&UI_drone_width_line[0], "sl5", UI_Graph_ADD, 7, UI_Color_Green, 2, 960 - WIDTHLINE_UP, 320,
             960 - WIDTHLINE_DOWN, 0);
  UILineDraw(&UI_drone_width_line[1], "sl6", UI_Graph_ADD, 7, UI_Color_Green, 2, 960 + WIDTHLINE_DOWN, 0,
             960 + WIDTHLINE_UP, 320);
  UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_drone_width_line[0], UI_drone_width_line[1]);

  // 绘制发射中心圆
  UICircleDraw(&UI_shoot_dir_circle[0], "sc0", UI_Graph_ADD, 7, UI_Color_White, 3, 960, 540, 15);
  // 绘制车头方向指示的中心圆
  UICircleDraw(&UI_shoot_dir_circle[1], "sc1", UI_Graph_ADD, 7, UI_Color_White, 3, 1556, 721, 76);
  UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_dir_circle[0], UI_shoot_dir_circle[1]);

  // 绘制车辆状态标志指示
  UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 700, "gimbal:");
  UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
  UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 650, "shoot:");
  UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
  UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 600, "frict:");
  UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
  UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 550, "lid:");
  UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);

  // 绘制车辆状态标志，动态
  UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 700, "PowerOff");
  UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
  UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 650, "off");
  UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
  UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 600, "off");
  UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
  UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 550, "close ");
  UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);

  UIPitchGaugeInit(960, 540, 390);

  // 自瞄模式选择器
  UIArcDraw(&UI_autoaim_bg, "ab0", UI_Graph_ADD, 5, UI_Color_White, 168, 192, 14, 960, 540, 310, 240);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_autoaim_bg);
  UIArcDraw(&UI_autoaim_indicator, "ac0", UI_Graph_ADD, 6, UI_Color_Pink, 168, 180, 14, 960, 540, 310, 240);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_autoaim_indicator);
  UICharDraw(&UI_autoaim_text, "at0", UI_Graph_ADD, 5, UI_Color_White, 18, 3, 900, 280, "A  B  C");
  UICharRefresh(&referee_recv_info->referee_id, UI_autoaim_text);

  // 左侧功率/弹量圆弧
  UIArcDraw(&UI_cap_arc, "ap0", UI_Graph_ADD, 6, UI_Color_Pink, 270, 310, 22, 960, 540, 370, 370);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_cap_arc);

  UICharDraw(&UI_cap_text_E, "ce0", UI_Graph_ADD, 6, UI_Color_White, 17, 2, 610, 545, "E");
  UICharRefresh(&referee_recv_info->referee_id, UI_cap_text_E);
  UICharDraw(&UI_cap_text_F, "cf0", UI_Graph_ADD, 6, UI_Color_White, 17, 2, 702, 775, "F");
  UICharRefresh(&referee_recv_info->referee_id, UI_cap_text_F);
  UICharDraw(&UI_ammo_text_full, "af0", UI_Graph_ADD, 6, UI_Color_White, 12, 2, 697, 300, "500");
  UICharRefresh(&referee_recv_info->referee_id, UI_ammo_text_full);
  UICharDraw(&UI_ammo_text_mid, "am0", UI_Graph_ADD, 6, UI_Color_White, 12, 2, 630, 420, "250");
  UICharRefresh(&referee_recv_info->referee_id, UI_ammo_text_mid);

  // 摩擦轮转速指示器
  UIRectangleDraw(&UI_fric_bg_left, "fl0", UI_Graph_ADD, 7, UI_Color_White, 2, 1513, 663, 1541, 783);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_fric_bg_left);
  UIRectangleDraw(&UI_fric_bg_right, "fr0", UI_Graph_ADD, 7, UI_Color_White, 2, 1576, 663, 1604, 783);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_fric_bg_right);

  UILineDraw(&UI_fric_pointer_left, "fl1", UI_Graph_ADD, 7, UI_Color_Main, 22, 1526, 720, 1526, 726);
  UILineDraw(&UI_fric_pointer_right, "fr1", UI_Graph_ADD, 7, UI_Color_Main, 22, 1590, 720, 1590, 726);
  UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_fric_pointer_left, UI_fric_pointer_right);
  UICharDraw(&UI_fric_text_down, "fd0", UI_Graph_ADD, 6, UI_Color_White, 20, 1, 1552, 685, "3");
  UICharRefresh(&referee_recv_info->referee_id, UI_fric_text_down);
  UICharDraw(&UI_fric_text_mid, "fm0", UI_Graph_ADD, 6, UI_Color_White, 20, 1, 1552, 735, "4");
  UICharRefresh(&referee_recv_info->referee_id, UI_fric_text_mid);
  UICharDraw(&UI_fric_text_up, "fu0", UI_Graph_ADD, 6, UI_Color_White, 20, 1, 1552, 782, "5");
  UICharRefresh(&referee_recv_info->referee_id, UI_fric_text_up);

  // 车头方向动态圆弧
  UIArcDraw(&UI_yaw_arc, "yd0", UI_Graph_ADD, 6, UI_Color_Main, 15, 345, 23, 1556, 721, 88, 88);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_yaw_arc);

  // 俯仰角仪表盘指针
  UIArcDraw(&UI_pitch_needle, "pn0", UI_Graph_ADD, 6, UI_Color_Pink, 90 - 1, 90 + 1, 45, 960, 540, 365, 365);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pitch_needle);

  UILineDraw(&Line_DecoMid, "ll0", UI_Graph_ADD, 6, UI_Color_White, 7, 577, 538, 606, 538);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, Line_DecoMid);
  UIArcDraw(&Arc_DecoUp, "ll1", UI_Graph_ADD, 6, UI_Color_White, 310, 311, 22, 960, 540, 370, 370);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, Arc_DecoUp);
  UIArcDraw(&Arc_DecoDown, "ll2", UI_Graph_ADD, 6, UI_Color_White, 229, 230, 22, 960, 535, 370, 370);
  UIGraphRefresh(&referee_recv_info->referee_id, 1, Arc_DecoDown);
  
  LOGINFO("[freeRTOS] UI Init Done, communication with ref has established");

  for (;;) {
    // 每给裁判系统发送一包数据会挂起一次,详见UITask函数的refereeSend()
    UIChangeCheck(&interactive_data);
    MyUIRefresh(&interactive_data);
    osDelay(1);  // 即使没有任何UI需要刷新,也挂起一次,防止卡在UITask中无法切换
  }
}

Referee_Interactive_info_t *getUI() { return &interactive_data; }
