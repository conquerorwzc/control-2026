/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "robot_config.h"
#ifdef USE_UI
#include "referee_task.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "math.h"
#include "cmsis_os.h"
#include "robot.h"
// #include "robot.c"
static uint8_t GETRCDATA;
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
static Referee_Interactive_info_t interactive_data;
// @todo 不应该使用全局变量

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
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
//new
// 定义用于UI更新的数据结构

// 云台俯仰角表盘
static Graph_Data_t UI_pitch_ticks[20];     // 刻度线
static String_Data_t UI_pitch_labels[20];   // 刻度标签
static Graph_Data_t UI_pitch_needle;       // 指针
static String_Data_t UI_pitch_value;       // 当前角度值显示

// 自瞄模式选择器
static Graph_Data_t UI_autoaim_bg;        // 背景圆弧
static Graph_Data_t UI_autoaim_indicator; // 选择指示圆弧
static String_Data_t UI_autoaim_text;     // 模式文字 "A  B  C"

// 左侧功率/弹量圆弧
static Graph_Data_t UI_cap_arc;            // 电容能量圆弧
static Graph_Data_t UI_ammo_arc;           // 弹量圆弧
static String_Data_t UI_cap_text_E;        // 文字 "E"
static String_Data_t UI_cap_text_F;        // 文字 "F"
static String_Data_t UI_ammo_text_full;    // 文字 "500"
static String_Data_t UI_ammo_text_mid;     // 文字 "250"

// 摩擦轮转速指示器
static Graph_Data_t UI_fric_bg_left;       // 左竖条背景矩形
static Graph_Data_t UI_fric_bg_right;      // 右竖条背景矩形
static Graph_Data_t UI_fric_pointer_left;  // 左指针线
static Graph_Data_t UI_fric_pointer_right; // 右指针线
static String_Data_t UI_fric_text_down;    // 文字 "5"
static String_Data_t UI_fric_text_mid;     // 文字 "6"
static String_Data_t UI_fric_text_up;      // 文字 "7"

// 车头方向动态圆弧
static Graph_Data_t UI_yaw_arc;             // 方向指示弧

static Graph_Data_t Line_DecoMid;
static Graph_Data_t Arc_DecoUp;
static Graph_Data_t Arc_DecoDown;

// 自瞄画面框
static Graph_Data_t UI_autoaim_box[4];  // 4条边构成矩形框



// typedef struct
// {
//     uint32_t chassis_flag : 1;
//     uint32_t gimbal_flag : 1;
//     uint32_t shoot_flag : 1;
//     uint32_t lid_flag : 1;
//     uint32_t friction_flag : 1;
//     uint32_t Power_flag : 1;
// } Referee_Interactive_Flag_t;





// 检测UI变化的函数
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    // if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
    //     _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    // }

    if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
        _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    }

    if (_Interactive_data->load_mode != _Interactive_data->load_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.load_flag = 1;
        _Interactive_data->load_last_mode = _Interactive_data->load_mode;
    }

    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    }
    if (_Interactive_data->vision_mode != _Interactive_data->vision_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.vision_flag = 1;
        _Interactive_data->vision_last_mode = _Interactive_data->vision_mode;
    }

    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx)
    {
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
        _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.chassis_power_mx;
    }

    // 俯仰角变化
    if (fabsf(_Interactive_data->pitch_angle - _Interactive_data->last_pitch_angle) > 0.1f)
    {
      _Interactive_data->Referee_Interactive_Flag.pitch_flag = 1;
      _Interactive_data->last_pitch_angle = _Interactive_data->pitch_angle;
    }

    if (_Interactive_data->autoaim_mode != _Interactive_data->last_autoaim_mode)
    {
      _Interactive_data->Referee_Interactive_Flag.autoaim_flag = 1;
      _Interactive_data->last_autoaim_mode = _Interactive_data->autoaim_mode;
    }

    if (fabsf(_Interactive_data->cap_voltage - _Interactive_data->last_cap_voltage) > 0.1f ||
        _Interactive_data->cap_mode != _Interactive_data->last_cap_mode)
    {
      _Interactive_data->Referee_Interactive_Flag.cap_flag = 1;
      _Interactive_data->last_cap_voltage = _Interactive_data->cap_voltage;
      _Interactive_data->last_cap_mode = _Interactive_data->cap_mode;
    }
    if (_Interactive_data->bullet_left_real != _Interactive_data->last_bullet_left_real)
    {
      _Interactive_data->Referee_Interactive_Flag.ammo_flag = 1;
      _Interactive_data->last_bullet_left_real = _Interactive_data->bullet_left_real;
    }

    if (_Interactive_data->fric_speed_left != _Interactive_data->last_fric_speed_left ||
        _Interactive_data->fric_speed_right != _Interactive_data->last_fric_speed_right)
    {
      _Interactive_data->Referee_Interactive_Flag.fric_flag = 1;
      _Interactive_data->last_fric_speed_left = _Interactive_data->fric_speed_left;
      _Interactive_data->last_fric_speed_right = _Interactive_data->fric_speed_right;
    }
    if (fabsf(_Interactive_data->chassis_relative_angle - _Interactive_data->last_chassis_relative_angle) > 0.01f)
    {
      _Interactive_data->Referee_Interactive_Flag.yaw_flag = 1;
      _Interactive_data->last_chassis_relative_angle = _Interactive_data->chassis_relative_angle;
    }
}

// UI更新函数
static void MyUIRefresh(Referee_Interactive_info_t *interactive_data)
{
    // 更新底盘状态
    if (interactive_data->Referee_Interactive_Flag.load_flag == 1)
    {
        char *load_str;
        switch(interactive_data->load_mode)
        {
            case LOAD_STOP:
                load_str = "LOAD_STOP";
                break;
            case LOAD_1_BULLET:
                load_str = "LOAD_1_BULLET";
                break;
            case LOAD_BURSTFIRE:
                load_str = "LOAD_BURSTFIRE";
                break;
              case LOAD_REVERSE:
            load_str = "LOAD_REVERSE";
            break;
            default:
                load_str= "error";
                break;
        }
        UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 650, load_str);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
        interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    }

    // 更新云台状态
    if (interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
    {
        char *gimbal_str;
        switch(interactive_data->gimbal_mode)
        {
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
    // if (interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
    // {
    //     char *shoot_str = interactive_data->shoot_mode == SHOOT_ON ? "on " : "off";
    //     UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 650, shoot_str);
    //     UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    //     interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    // }

    // 更新摩擦轮状态
    if (interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    {
        char *friction_str = interactive_data->friction_mode == FRICTION_ON ? "on " : "off";
        UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, friction_str);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    }

    // 更新弹舱盖状态
    if (interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        char *chassis_str;
        //char *lid_str = interactive_data->lid_mode == LOPEN ? "open " : "close";
        switch(interactive_data->chassis_mode)
        {

        case CHASSIS_POWER_OFF:
            chassis_str = "PowerOff";
            break;
        case CHASSIS_ROTATE:
            chassis_str = "rotate  ";
            break;
        case CHASSIS_FOLLOW:
            chassis_str = "follow  ";
            break;
        case CHASSIS_FOLLOW_DIAGONAL:
            chassis_str = "follow45 ";
            break;
        case CHASSIS_FREE:
            chassis_str = "free     ";
            break;
        default:
            chassis_str = "error";
            break;
        }
        UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, chassis_str);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
        interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }

    if (interactive_data->Referee_Interactive_Flag.vision_flag == 1)
    {
        char *vision_str;
        switch(interactive_data->vision_mode)
        {
            case 0:
            vision_str = "IDLE  ";
            break;
            case 1:
            vision_str = "AUTO_AIM  ";
            break;
            case 2:
            vision_str = "small_buff  ";
            break;
            case 3:
            vision_str = "big_buff     ";
            break;
        }
        UICharDraw(&UI_State_dyn[5], "sd5", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 750, vision_str);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[5]);
        interactive_data->Referee_Interactive_Flag.vision_flag = 0;
    }
    // 更新功率显示
    // if (interactive_data->Referee_Interactive_Flag.Power_flag == 1)
    // {
    //     int32_t power_value = (int32_t)(interactive_data->Chassis_Power_Data.chassis_power_mx * 1000);
    //     UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 750, 230, power_value);
    //
    //     // 更新能量条长度
    //     uint32_t energy_bar_length = 720 + (uint32_t)(interactive_data->Chassis_Power_Data.chassis_power_mx * 5);
    //     UILineDraw(&UI_Energy[2], "sd6", UI_Graph_Change, 8, UI_Color_Pink, 30, 720, 160, energy_bar_length, 160);
    //     UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
    //     interactive_data->Referee_Interactive_Flag.Power_flag = 0;
    // }

    // 自瞄模式指示器
    if (interactive_data->Referee_Interactive_Flag.autoaim_flag == 1)
    {
      uint32_t start_angle, end_angle;
      // 根据 autoaim_mode 选择圆弧覆盖区域（这里假设3个模式分别对应左中右三段）
      switch (interactive_data->autoaim_mode) {
        case 0: start_angle = 168; end_angle = 180; break; // 左段
        case 1: start_angle = 180; end_angle = 192; break; // 右段
        default: start_angle = 168; end_angle = 192; break; // 全段
      }
      UIArcDraw(&UI_autoaim_indicator, "ac0", UI_Graph_Change, 6, UI_Color_Pink,
                start_angle, end_angle, 14, 960, 540, 310, 240);
      UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_autoaim_indicator);
      interactive_data->Referee_Interactive_Flag.autoaim_flag = 0;
    }

    // 电容能量圆弧
      if (interactive_data->Referee_Interactive_Flag.cap_flag == 1)
      {
          float volt = interactive_data->cap_voltage;
          uint8_t mode = interactive_data->cap_mode;
          // 计算弧长（假设电压范围 12V ~ 23V 对应 0~40度）
          float bar = (volt - 14.0f) / (23.0f - 14.0f) * 40.0f;
          if (bar < 1) bar = 1;
          if (bar > 40) bar = 40;
          uint32_t end_angle = 270 + (uint32_t)bar;
          uint32_t color;
          switch (mode) {
              case 0: color = UI_Color_Green; break;   // 关闭
              case 1: color = UI_Color_Pink; break;  // 开启
              case 2: color = UI_Color_Cyan; break;   // 超级电容
              default: color = UI_Color_Orange; break;
          }
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

      // 摩擦轮指针
      if (interactive_data->Referee_Interactive_Flag.fric_flag == 1)
      {
        uint16_t left_speed = interactive_data->fric_speed_left;   // 左摩擦轮转速（已处理为正方向）
        uint16_t right_speed = interactive_data->fric_speed_right; // 右摩擦轮转速

        // 左指针绘制
        uint32_t left_y1, left_y2;
        uint32_t left_color = UI_Color_Green; // 默认正常颜色

        if (left_speed < FRIC_LOWER) {
          left_y1 = 665;
          left_y2 = 695;
          left_color = UI_Color_Yellow; // 过低变黄
        } else if (left_speed > FRIC_UPPER) {
          left_y1 = 755;
          left_y2 = 785;
          left_color = UI_Color_Yellow; // 过高变黄
        } else {
          // 正常范围内，按比例计算指针位置（120像素高度范围）
          float ratio = (float)(left_speed - FRIC_LOWER) / (float)(FRIC_UPPER - FRIC_LOWER);
          uint32_t base = 665 + (uint32_t)(ratio * 120.0f);
          left_y1 = base - 3;
          left_y2 = base + 3;
          // 颜色保持绿色
        }
        UILineDraw(&UI_fric_pointer_left, "fl1", UI_Graph_Change, 7, left_color, 22,
                   1526, left_y1, 1526, left_y2);

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
        UILineDraw(&UI_fric_pointer_right, "fr1", UI_Graph_Change, 7, right_color, 22,
                   1590, right_y1, 1590, right_y2);

        // 立即发送两个指针更新
        UIGraphRefresh(&referee_recv_info->referee_id, 2,
                       UI_fric_pointer_left, UI_fric_pointer_right);

        interactive_data->Referee_Interactive_Flag.fric_flag = 0;
      }

      // 车头方向圆弧
      if (interactive_data->Referee_Interactive_Flag.yaw_flag == 1)
      {
          int yaw_deg = (int)interactive_data->chassis_relative_angle;
          if (yaw_deg < 0) yaw_deg += 360;
          uint32_t show1, show2;
          // 计算60°扇形的起止角度，处理跨越0°的情况
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
              case CHASSIS_FOLLOW: color = UI_Color_Pink; break;
              case CHASSIS_ROTATE: color = UI_Color_Green; break;
              default: color = UI_Color_Orange; break;
          }
          UIArcDraw(&UI_yaw_arc, "yd0", UI_Graph_Change, 6, color, show1, show2, 23, 1556, 721, 88, 88);
          UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_yaw_arc);
          interactive_data->Referee_Interactive_Flag.yaw_flag = 0;
      }

      // 俯仰角仪表盘指针和数值
      if (interactive_data->Referee_Interactive_Flag.pitch_flag == 1)
      {
        float pitch_deg = interactive_data->pitch_angle;
        float ui_angle = 90.0f - pitch_deg;          // 转换为UI坐标系角度

        // 归一化到 0~360 度
        while (ui_angle < 0) ui_angle += 360.0f;
        while (ui_angle >= 360) ui_angle -= 360.0f;

        uint32_t start_angle = (uint32_t)(ui_angle - 1);
        uint32_t end_angle   = (uint32_t)(ui_angle + 1);
        // 边界处理（防止超出 0~359）
        if (start_angle >= 360) start_angle -= 360;
        if (end_angle   >= 360) end_angle   -= 360;

        // 绘制圆弧指针（线宽38，半径365）
        UIArcDraw(&UI_pitch_needle, "pn0", UI_Graph_Change, 6, UI_Color_Pink,
                  start_angle, end_angle, 38, 960, 540, 365, 365);
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pitch_needle);
        interactive_data->Referee_Interactive_Flag.pitch_flag = 0;
  }
}

/**
 * @brief 初始化俯仰角仪表盘UI组件
 * @param[in] center_x 仪表盘圆心X坐标
 * @param[in] center_y 仪表盘圆心Y坐标
 * @param[in] radius 仪表盘半径
 * @return void
 */
void UIPitchGaugeInit(uint32_t center_x,uint32_t center_y,uint32_t radius) {
  // 主刻度：50、70、90、110、130度
  const int main_ticks[] = {50, 70, 90, 110, 130};
  const char* main_labels[] = {"50", "70", "90", "110", "130"};
  // 中间小刻度：60、80、100、120度
  const int sub_ticks[] = {60, 80, 100, 120};

  // 绘制主刻度线和标签
  for (int i = 0; i < 5; i++) {
    // 角度转换：UI角度 = 实际角度 - 90
    float ui_angle = 90.0f - (float)main_ticks[i];
    float angle_rad = ui_angle * 3.1415926535f / 180.0f;
    uint32_t tick_length = 27; // 主刻度线长度

    // 计算刻度线位置
    int32_t start_x = center_x + (int32_t)(radius * cosf(angle_rad));
    int32_t start_y = center_y - (int32_t)(radius * sinf(angle_rad));
    int32_t end_x = center_x + (int32_t)((radius - tick_length) * cosf(angle_rad));
    int32_t end_y = center_y - (int32_t)((radius - tick_length) * sinf(angle_rad));

    // 绘制主刻度线
    char graph_name[3] = {0};
    graph_name[0] = 'p';
    graph_name[1] = 't';
    graph_name[2] = '0' + i;

    UILineDraw(&UI_pitch_ticks[i], graph_name, UI_Graph_ADD, 8, UI_Color_White,
               7, start_x, start_y, end_x, end_y);

    // 绘制主刻度标签
    // int32_t label_x = center_x + (int32_t)((radius - tick_length - 25) * cosf(angle_rad)) - 8;
    // int32_t label_y = center_y - (int32_t)((radius - tick_length - 25) * sinf(angle_rad)) - 6;
    // 绘制主刻度标签
    // 方案3：根据角度动态计算标签位置，使标签贴合刻度线
    int32_t label_x, label_y;

    // 1. 计算标签基准位置（在刻度线末端）
    int32_t label_distance = radius - tick_length - 25;  // 标签离圆心的距离
    int32_t base_x = center_x + (int32_t)(label_distance * cosf(angle_rad));
    int32_t base_y = center_y - (int32_t)(label_distance * sinf(angle_rad));

    // 2. 估算文本大小（根据你的12号字体）
    int32_t text_width = 18;  // "130"这样的3个字符，每个大约6像素
    int32_t text_height = 12; // 12号字体高度大约12像素

    // 3. 根据角度动态调整标签位置
    // 使用cos和sin值判断刻度线的方向
    float cos_val = cosf(angle_rad);
    float sin_val = sinf(angle_rad);

    // 水平方向调整
    if (cos_val > 0.7f) { // 右侧（cos接近1）
      // 右侧刻度：文本应该左对齐，让文本在刻度线左边
      label_x = base_x - text_width;  // 文本宽度向左偏移
    } else if (cos_val < -0.7f) { // 左侧（cos接近-1）
      // 左侧刻度：文本应该右对齐
      label_x = base_x;  // 从基准点开始
    } else if (cos_val > 0.3f) { // 右偏中
      // 稍微向右偏：文本中心对齐
      label_x = base_x - text_width / 2;
    } else if (cos_val < -0.3f) { // 左偏中
      // 稍微向左偏：文本中心对齐
      label_x = base_x - text_width / 2;
    } else { // 中间区域（cos接近0）
      // 正上方或正下方：文本居中对齐
      label_x = base_x - text_width / 2;
    }

    // 垂直方向调整
    if (sin_val > 0.7f) { // 上方（sin接近1）
      // 上方刻度：文本应该在刻度线下方
      label_y = base_y;  // 从基准点开始（文本顶部在基准点）
    } else if (sin_val < -0.7f) { // 下方（sin接近-1）
      // 下方刻度：文本应该在刻度线上方
      label_y = base_y - text_height;  // 向上偏移文本高度
    } else if (sin_val > 0.3f) { // 上偏中
      // 稍微向上偏：文本垂直居中
      label_y = base_y - text_height / 2;
    } else if (sin_val < -0.3f) { // 下偏中
      // 稍微向下偏：文本垂直居中
      label_y = base_y - text_height / 2;
    } else { // 中间区域（sin接近0）
      // 正左或正右：文本垂直居中
      label_y = base_y - text_height / 2;
    }

    // 4. 微调：根据具体刻度进一步优化
    switch (main_ticks[i]) {
      case 50:  // 左上区域
        label_x -= 2;  // 稍微向左调整
        label_y += 4;  // 稍微向下调整
        break;
      case 70:  // 左中上
        label_x -= 1;
        label_y += 2;
        break;
      case 90:  // 正右
        // 已经调整得很好，保持
        break;
      case 110: // 右中下
        label_x -= 1;
        label_y -= 2;
        break;
      case 130: // 右下
        label_x -= 2;
        label_y -= 4;
        break;
    }

    char label_name[3] = {0};
    label_name[0] = 'p';
    label_name[1] = 'l';
    label_name[2] = '0' + i;

    UICharDraw(&UI_pitch_labels[i], label_name, UI_Graph_ADD, 8, UI_Color_White,
               12, 2, label_x, label_y, main_labels[i]);
  }

  // 绘制中间小刻度线（60、80、100、120度）
  for (int i = 0; i < 4; i++) {
    float ui_angle = 90.0f - (float)sub_ticks[i];
    float angle_rad = ui_angle * 3.1415926535f / 180.0f;
    uint32_t tick_length = 13; // 小刻度线长度

    int32_t start_x = center_x + (int32_t)(radius * cosf(angle_rad));
    int32_t start_y = center_y - (int32_t)(radius * sinf(angle_rad));
    int32_t end_x = center_x + (int32_t)((radius - tick_length) * cosf(angle_rad));
    int32_t end_y = center_y - (int32_t)((radius - tick_length) * sinf(angle_rad));

    // 绘制小刻度线
    char graph_name[3] = {0};
    graph_name[0] = 'p';
    graph_name[1] = 's';
    graph_name[2] = '0' + i;

    static Graph_Data_t UI_sub_ticks[4];
    UILineDraw(&UI_sub_ticks[i], graph_name, UI_Graph_ADD, 8, UI_Color_White,
               7, start_x, start_y, end_x, end_y);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_sub_ticks[i]);

    // 绘制短刻度标签
    // int32_t label_x = center_x + (int32_t)((radius - tick_length - 25) * cosf(angle_rad)) - 6;
    // int32_t label_y = center_y - (int32_t)((radius - tick_length - 25) * sinf(angle_rad)) - 5;
    //
    // char label_name[3] = {0};
    // label_name[0] = 'p';
    // label_name[1] = 'b';  // 'b' for sub label
    // label_name[2] = '0' + i;
    //
    // // 创建标签文本
    // char sub_label_text[4];
    // snprintf(sub_label_text, sizeof(sub_label_text), "%d", sub_ticks[i]);
    //
    // // 使用UI_pitch_labels数组的后面4个元素（假设有9个元素）
    // // 或者如果你不想用UI_pitch_labels，可以这样：
    // UICharDraw(&UI_pitch_labels[5 + i], label_name, UI_Graph_ADD, 8, UI_Color_White,
    //            12, 2, label_x, label_y, sub_label_text);
    // UICharRefresh(&referee_recv_info->referee_id, UI_pitch_labels[5 + i]);
  }

  // 发送主刻度线和标签
  for (int i = 0; i < 5; i++) {
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pitch_ticks[i]);
    UICharRefresh(&referee_recv_info->referee_id, UI_pitch_labels[i]);
  }
}

//newend

void MyUIInit()
{
  referee_recv_info=RefereeInit(&huart6);
     // if (!referee_recv_info->init_flag)
     //     vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务
    // while (referee_recv_info->GameState.game_progress == 0)
    //     osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查
    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 显示队标SRM
    static String_Data_t UI_srm_logo;
    static String_Data_t UI_srm_init;
    static String_Data_t UI_srm_type;
    static Graph_Data_t UI_srm_bg;
    // 绘制背景矩形
    // UIRectangleDraw(&UI_srm_bg, "bg0", UI_Graph_ADD, 0, UI_Color_Pink, 5,
    //                 CENTER_X - 250, CENTER_Y - 100, CENTER_X + 250, CENTER_Y + 100);
    // UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_srm_bg);
    // 绘制SRM文字 (超大字体，size=100)
    UICharDraw(&UI_srm_logo, "sr0", UI_Graph_ADD, 0, UI_Color_Black, 200, 25,
               CENTER_X -260, CENTER_Y+80, "SRM");
    UICharRefresh(&referee_recv_info->referee_id, UI_srm_logo);
    // 绘制Initializing...文字 (小字，size=25)
    UICharDraw(&UI_srm_init, "si0", UI_Graph_ADD, 0, UI_Color_White, 25, 2,
               CENTER_X - 150, CENTER_Y - 170, "Initializing...");
    UICharRefresh(&referee_recv_info->referee_id, UI_srm_init);
    osDelay(200); // 短暂显示

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
    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 700, "gimbal:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 650, "loader:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 600, "frict:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 550, "chassis:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);
    UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 750, "vision:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);

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
    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 550, "PowerOff");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
    UICharDraw(&UI_State_dyn[5], "sd5", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 750, "AUTO_AIM");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[5]);

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

    // 摩擦轮转速指示器
    UIRectangleDraw(&UI_fric_bg_left, "fl0", UI_Graph_ADD, 7, UI_Color_White, 2, 1513, 663, 1541, 783);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_fric_bg_left);
    UIRectangleDraw(&UI_fric_bg_right, "fr0", UI_Graph_ADD, 7, UI_Color_White, 2, 1576, 663, 1604, 783);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_fric_bg_right);

    // 指针初始位置（中间）
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
    // 绘制指针初始位置（指向90°）
    UIArcDraw(&UI_pitch_needle, "pn0", UI_Graph_ADD, 6, UI_Color_Pink, 90 - 1, 90 + 1, 45, 960, 540, 365, 365);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pitch_needle);

    UILineDraw(&Line_DecoMid, "ll0", UI_Graph_ADD, 6, UI_Color_White, 7, 577, 538, 606, 538);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, Line_DecoMid);
    UIArcDraw(&Arc_DecoUp, "ll1", UI_Graph_ADD, 6, UI_Color_White, 310, 311, 22, 960, 540, 370, 370);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, Arc_DecoUp);
    UIArcDraw(&Arc_DecoDown, "ll2", UI_Graph_ADD, 6, UI_Color_White, 229, 230, 22, 960, 535, 370, 370);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, Arc_DecoDown);

    // 自瞄画面框初始化
    {
      // 计算自瞄相机FOV (基于焦距比例)
      // FOV_auto = 2 * atan(tan(FOV_tx/2) * f_tx / f_auto)
      float fov_tx_rad = TX_CAM_FOV_H * 3.1415926535f / 180.0f;
      float fov_auto_rad = 2.0f * atanf(tanf(fov_tx_rad / 2.0f) * TX_CAM_FOCAL_LEN / AUTOAIM_CAM_FOCAL_LEN);

      // 计算垂直FOV (假设16:9比例)
      float fov_auto_v_rad = 2.0f * atanf(tanf(fov_auto_rad / 2.0f) * SENSOR_RATIO_H / SENSOR_RATIO_W);
      float fov_auto_v_deg = fov_auto_v_rad * 180.0f / 3.1415926535f;

      // 计算框在屏幕上的尺寸
      // 屏幕分辨率 1920x1080，图传FOV 139° 对应屏幕宽度
      uint32_t box_width = (uint32_t)(1920.0f * tanf(fov_auto_rad / 2.0f) / tanf(fov_tx_rad / 2.0f));
      uint32_t box_height = (uint32_t)(box_width * SENSOR_RATIO_H / SENSOR_RATIO_W);

      // 计算固定pitch安装角偏移 (自瞄相机相对图传的安装角度差)
      float pitch_offset_y = 1080.0f / fov_auto_v_deg * AUTOAIM_CAM_PITCH_DEG;

      // 计算框的四个顶点 (以屏幕中心为基准，加上固定pitch偏移)
      int32_t box_left = CENTER_X - (int32_t)(box_width / 2);
      int32_t box_right = CENTER_X + (int32_t)(box_width / 2);
      int32_t box_top = CENTER_Y - (int32_t)(box_height / 2) + (int32_t)pitch_offset_y;
      int32_t box_bottom = CENTER_Y + (int32_t)(box_height / 2) + (int32_t)pitch_offset_y;

      // 边界保护
      if (box_left < 0) box_left = 0;
      if (box_right > 1919) box_right = 1919;
      if (box_top < 0) box_top = 0;
      if (box_bottom > 1079) box_bottom = 1079;

      // 绘制4条边 (矩形框)
      UILineDraw(&UI_autoaim_box[0], "ab1", UI_Graph_ADD, 5, UI_Color_Cyan, 2,
                 box_left, box_top, box_right, box_top);       // 上边
      UILineDraw(&UI_autoaim_box[1], "ab2", UI_Graph_ADD, 5, UI_Color_Cyan, 2,
                 box_left, box_bottom, box_right, box_bottom); // 下边
      UILineDraw(&UI_autoaim_box[2], "ab3", UI_Graph_ADD, 5, UI_Color_Cyan, 2,
                 box_left, box_top, box_left, box_bottom);     // 左边
      UILineDraw(&UI_autoaim_box[3], "ab4", UI_Graph_ADD, 5, UI_Color_Cyan, 2,
                 box_right, box_top, box_right, box_bottom);   // 右边

      // 批量发送4条线
      UIGraphRefresh(&referee_recv_info->referee_id, 4,
                     UI_autoaim_box[0], UI_autoaim_box[1],
                     UI_autoaim_box[2], UI_autoaim_box[3]);
    }

    // 删除队标SRM
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_Layer, 0); // 删除图层0
    UICharDraw(&UI_srm_type, "sr0", UI_Graph_ADD, 0, UI_Color_Main, 15, 2,
               CENTER_X -200, CENTER_Y-500, "SRM:ICU RUDDER WHEEL INFANTRY");
    UICharRefresh(&referee_recv_info->referee_id, UI_srm_type);
}

// 实现缺失的UITask函数
void UITask()
{
    //if (interactive_data.chassis_mode!=CHASSIS_POWER_OFF)
    // 首次运行时初始化指针
    // if (referee_recv_info == NULL) {
    //     referee_recv_info = RefereeInit(&huart6); // 假设使用默认串口
    // }
    if (interactive_data.ui_init==1)
    {
        MyUIInit();
        interactive_data.ui_init=0;
    }
    else
    {
        UIChangeCheck(&interactive_data);
    }
    // 更新交互数据（模拟从系统其他部分获取数据）
    // 这些值应该从实际的机器人系统中获取

    // 检查是否有变化


    // 执行UI刷新
    MyUIRefresh(&interactive_data);
}

Referee_Interactive_info_t* getUI()
{
    return &interactive_data;
}
#endif