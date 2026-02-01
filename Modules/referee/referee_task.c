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
#include "referee_task.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"
#include "robot.h"

static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
RobotInstance* robotdata;
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
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
static Graph_Data_t UI_Energy[3];      // 电容能量条
static String_Data_t UI_State_sta[6];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[6];  // 机器人状态,动态先add才能change
static uint32_t shoot_line_location[10] = {540, 960, 490, 515, 565};
//new
// 定义用于UI更新的数据结构

// 云台俯仰角表盘
static Graph_Data_t UI_pitch_ticks[20];     // 刻度线
static String_Data_t UI_pitch_labels[20];   // 刻度标签
static Graph_Data_t UI_pitch_needle;       // 指针
static String_Data_t UI_pitch_value;       // 当前角度值显示


typedef enum
{
    LID_CLOSE = 0,
    LID_OPEN
} lid_mode_e;

typedef struct
{
    float chassis_power_mx; // 最大功率限制
    float chassis_power;    // 当前功率
} Chassis_Power_Data_s;

// typedef struct
// {
//     uint32_t chassis_flag : 1;
//     uint32_t gimbal_flag : 1;
//     uint32_t shoot_flag : 1;
//     uint32_t lid_flag : 1;
//     uint32_t friction_flag : 1;
//     uint32_t Power_flag : 1;
// } Referee_Interactive_Flag_t;

typedef struct
{
    Referee_Interactive_Flag_t Referee_Interactive_Flag;
    // 为UI绘制以及交互数据所用
    Chassis_Mode_e chassis_mode;    // 底盘模式
    Gimbal_Mode_e gimbal_mode;      // 云台模式
    Shoot_Mode_e shoot_mode;        // 发射模式设置
    Friction_Mode_e friction_mode;  // 摩擦轮关闭
    lid_mode_e lid_mode;            // 弹舱盖打开
    Chassis_Power_Data_s Chassis_Power_Data; // 功率控制
    float pitch_angle; // 云台俯仰角

    // 上一次的模式，用于flag判断
    Chassis_Mode_e chassis_last_mode;
    Gimbal_Mode_e gimbal_last_mode;
    Shoot_Mode_e shoot_last_mode;
    Friction_Mode_e friction_last_mode;
    lid_mode_e lid_last_mode;   
    Chassis_Power_Data_s Chassis_last_Power_Data;
    float last_pitch_angle; // 上一次的俯仰角
} Referee_Interactive_info_t;

static Referee_Interactive_info_t interactive_data;

// 检测UI变化的函数
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }
    
    if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
        _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    }
    
    if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
        _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
    }
    
    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    }
    
    if (_Interactive_data->lid_mode != _Interactive_data->lid_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 1;
        _Interactive_data->lid_last_mode = _Interactive_data->lid_mode;
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
}

// UI更新函数
static void MyUIRefresh(Referee_Interactive_info_t *interactive_data)
{
    // 更新底盘状态
    if (interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        char *chassis_str;
        switch(interactive_data->chassis_mode)
        {
            case CHASSIS_POWER_OFF:
                chassis_str = "PowerOff";
                break;
            case CHASSIS_ROTATE:
                chassis_str = "Rotate";
                break;
            case CHASSIS_FOLLOW:
                chassis_str = "Follow";
                break;
            default:
                chassis_str = "unknown";
                break;
        }
        UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, chassis_str);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
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
                gimbal_str = "On";
                break;
            case GIMBAL_VISION:
                gimbal_str = "Vision";
                break;
            default:
                gimbal_str = "unknown";
                break;
        }
        UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, gimbal_str);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
    }
    
    // 更新射击状态
    if (interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
    {
        char *shoot_str = interactive_data->shoot_mode == SHOOT_ON ? "on" : "off";
        UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 650, shoot_str);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
        interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    }
    
    // 更新摩擦轮状态
    if (interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    {
        char *friction_str = interactive_data->friction_mode == FRICTION_ON ? "on" : "off";
        UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, friction_str);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    }
    
    // 更新弹舱盖状态
    if (interactive_data->Referee_Interactive_Flag.lid_flag == 1)
    {
        char *lid_str = interactive_data->lid_mode == LID_OPEN ? "open" : "close";
        UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, lid_str);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
        interactive_data->Referee_Interactive_Flag.lid_flag = 0;
    }
    
    // 更新功率显示
    if (interactive_data->Referee_Interactive_Flag.Power_flag == 1)
    {
        int32_t power_value = (int32_t)(interactive_data->Chassis_Power_Data.chassis_power_mx * 1000);
        UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 750, 230, power_value);
        
        // 更新能量条长度
        uint32_t energy_bar_length = 720 + (uint32_t)(interactive_data->Chassis_Power_Data.chassis_power_mx * 5);
        UILineDraw(&UI_Energy[2], "sd6", UI_Graph_Change, 8, UI_Color_Pink, 30, 720, 160, energy_bar_length, 160);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
        interactive_data->Referee_Interactive_Flag.Power_flag = 0;
    }
}
//newend
void MyUIInit()
{
  robotdata=RobotGet();
  referee_recv_info=robotdata->referee_data;
     // if (!referee_recv_info->init_flag)
     //     vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查
    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 绘制发射基准线
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);
    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[2], 1110, shoot_line_location[2]);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[3], 1110, shoot_line_location[3]);
    UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[4], 1110, shoot_line_location[4]);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4]);

    // 绘制车辆状态标志指示
    UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 150, 750, "chassis:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 150, 700, "gimbal:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 150, 650, "shoot:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 600, "frict:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 550, "lid:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);

    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 750, "PowerOff");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 270, 700, "PowerOff");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 270, 650, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 600, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 550, "open ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);

    // 底盘功率显示，静态
    UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 620, 230, "Power:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);
    // 能量条框
    UIRectangleDraw(&UI_Energy[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1220, 180);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);

    // 底盘功率显示,动态
    UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 750, 230, 24000);
    // 能量条初始状态
    UILineDraw(&UI_Energy[2], "sd6", UI_Graph_ADD, 8, UI_Color_Pink, 30, 720, 160, 1020, 160);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);

    // 俯仰角仪表盘刻度线和指针初始化
    uint32_t center_x = 960;  // 圆心X坐标
    uint32_t center_y = 540;  // 圆心Y坐标
    uint32_t radius = 394;    // 半径

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

      // 计算刻度线位置 //u
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

    // 初始化指针（指向90度，即正右方）
    // float ui_angle_90 = 90.0f - 90.0f;
    // float angle_rad_90 = ui_angle_90 * 3.1415926535f / 180.0f;
    // uint32_t needle_end_x = center_x + (uint32_t)(radius * cosf(angle_rad_90));
    // uint32_t needle_end_y = center_y - (uint32_t)(radius * sinf(angle_rad_90));
    //
    // UILineDraw(&UI_pitch_needle, "pn0", UI_Graph_ADD, 9, UI_Color_White,
    //            7, center_x, center_y, needle_end_x, needle_end_y);
    // UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pitch_needle);

    // 初始化角度值显示（放在圆心下方）
    UICharDraw(&UI_pitch_value, "pv0", UI_Graph_ADD, 9, UI_Color_White,
               18, 3, center_x - 30, center_y + 50, "90.0°");
    UICharRefresh(&referee_recv_info->referee_id, UI_pitch_value);
}

// 实现缺失的UITask函数
void UITask()
{
    // 首次运行时初始化指针
    if (referee_recv_info == NULL) {
        referee_recv_info = RefereeInit(&huart6); // 假设使用默认串口
    }
    
    // 更新交互数据（模拟从系统其他部分获取数据）
    // 这些值应该从实际的机器人系统中获取
    //interactive_data.chassis_mode = CHASSIS_NORMAL;
    interactive_data.chassis_mode = robotdata->chassis->chassis_ctrl_cmd.chassis_mode;
    //interactive_data.gimbal_mode = GIMBAL_NORMAL;
    interactive_data.gimbal_mode = robotdata->gimbal->gimbal_ctrl_cmd.gimbal_mode;
   // interactive_data.shoot_mode = SHOOT_OFF;
  interactive_data.shoot_mode = robotdata->shoot->shoot_ctrl_cmd.shoot_mode;
   // interactive_data.friction_mode = FRICTION_OFF;
  interactive_data.friction_mode = robotdata->shoot->shoot_ctrl_cmd.friction_mode;
    interactive_data.lid_mode = LID_OPEN;
    interactive_data.Chassis_Power_Data.chassis_power_mx = robotdata->chassis->chassis_ctrl_cmd.max_power; // 示例功率值
    
    // 检查是否有变化
    UIChangeCheck(&interactive_data);
    
    // 执行UI刷新
    MyUIRefresh(&interactive_data);
}