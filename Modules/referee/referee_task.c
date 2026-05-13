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
#include "referee_protocol.h"

static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
Referee_Interactive_info_t interactive_data;
// @todo 不应该使用全局变量

//前置声明
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data);
static void MyUIRefresh(Referee_Interactive_info_t *interactive_data);

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
static Graph_Data_t UI_Laser_Bar[2]; // 0号是外框，1号是里面的填充条
static Graph_Data_t UI_Energy[3];      // 电容能量条
static String_Data_t UI_State_sta[8];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[8];  // 机器人状态,动态先add才能change
static uint32_t shoot_line_location[10] = {540, 960, 490, 515, 565};

void MyUIInit()
{
    if (!referee_recv_info->init_flag)
        vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查

    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 1. 绘制发射基准线 (十字准星)
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);
    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[2], 1110, shoot_line_location[2]);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[3], 1110, shoot_line_location[3]);
    UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[4], 1110, shoot_line_location[4]);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4]);

    // 2. 绘制无人机专属状态标签 (静态标签, 数组索引 0, 1, 2, 4, 5，避开了3不用管)
    UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 150, 700, "gimbal:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);

    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 150, 650, "shoot:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);

    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 600, "frict:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);

    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 150, 550, "Heat:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);

    UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 8, UI_Color_Cyan, 15, 2, 150, 500, "Speed:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);

    // 3. 绘制动态数值的初始状态 (初值均为 0 或 off)
    UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);

    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 270, 650, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);

    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 600, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);

    UIIntDraw(&UI_State_dyn[4].Graph_Control, "sd4", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 270, 550, 0);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_State_dyn[4].Graph_Control);

    UIFloatDraw(&UI_State_dyn[5].Graph_Control, "sd5", UI_Graph_ADD, 8, UI_Color_Cyan, 15, 3, 2, 270, 500, 0);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_State_dyn[5].Graph_Control);

    // 4. 无人机激光进度条 (画外框和初始0长度的填充条)
    UIRectangleDraw(&UI_Laser_Bar[0], "la0", UI_Graph_ADD, 7, UI_Color_White, 2, 710, 300, 1210, 330);
    UILineDraw(&UI_Laser_Bar[1], "la1", UI_Graph_ADD, 8, UI_Color_Cyan, 25, 712, 315, 712, 315);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Laser_Bar[0], UI_Laser_Bar[1]);
}

/**
 * @brief 检查无人机状态是否发生变化
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data) {
  // 1. 检查云台状态
  if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode) {
    _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
    _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
  }

  // 2. 检查发射状态
  if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode) {
    _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
    _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
  }

  // 3. 检查摩擦轮状态
  if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode) {
    _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
    _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
  }

  // 4. 检查激光照射时间 (防抖：变化超过一定数值才刷新，或者直接比对)
  if (_Interactive_data->laser_time != _Interactive_data->last_laser_time) {
    _Interactive_data->Referee_Interactive_Flag.laser_flag = 1;
    _Interactive_data->last_laser_time = _Interactive_data->laser_time;
  }

  // 5. 检查热量变化 (热量只要变了就刷新)
  if (_Interactive_data->heat != _Interactive_data->last_heat) {
    _Interactive_data->Referee_Interactive_Flag.heat_flag = 1;
    _Interactive_data->last_heat = _Interactive_data->heat;
  }

  // 6. 检查初速变化 (浮点数，设置0.5m/s的阈值防抖，避免串口堵塞)
  if (fabsf(_Interactive_data->initial_speed - _Interactive_data->last_initial_speed) > 0.5f) {
    _Interactive_data->Referee_Interactive_Flag.speed_flag = 1;
    _Interactive_data->last_initial_speed = _Interactive_data->initial_speed;
  }

  // 7. 检查自瞄状态变化
  if (_Interactive_data->vision_lock != _Interactive_data->last_vision_lock) {
    _Interactive_data->Referee_Interactive_Flag.vision_flag = 1;
    _Interactive_data->last_vision_lock = _Interactive_data->vision_lock;
  }
}

/**
 * @brief 局部刷新动态 UI
 */
static void MyUIRefresh(Referee_Interactive_info_t *interactive_data) {
  // 0.1 刷新云台状态
  if (interactive_data->Referee_Interactive_Flag.gimbal_flag == 1) {
    char *gimbal_str;
    if (interactive_data->gimbal_mode == GIMBAL_ON) gimbal_str = "normal   ";
    else if (interactive_data->gimbal_mode == GIMBAL_VISION) gimbal_str = "vision   ";
    else gimbal_str = "zeroforce";

    UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, gimbal_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
  }

  // 0.2 刷新发射状态
  if (interactive_data->Referee_Interactive_Flag.shoot_flag == 1) {
    char *shoot_str = (interactive_data->shoot_mode == SHOOT_ON) ? "on   " : "off  ";
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 650, shoot_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
  }

  // 1. 刷新摩擦轮状态 (假设动态字符画在 sd2)
  if (interactive_data->Referee_Interactive_Flag.friction_flag == 1) {
    char *frict_str;
    if (interactive_data->friction_mode == FRICTION_ON) {
        frict_str = "on   ";
    } else {
        frict_str = "off  ";
    }
    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, frict_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    interactive_data->Referee_Interactive_Flag.friction_flag = 0;
  }

  // 2. 刷新激光蓄力条 (假设外框是 la0，填充条是 la1)
  // 如果你在 MyUIInit 里定义了 UI_Laser_Bar 数组，这里就可以直接更新它的长度
  if (interactive_data->Referee_Interactive_Flag.laser_flag == 1) {
    // 假设满蓄力是 3000ms，进度条最大长度是 496 像素
    uint32_t current_width = (uint32_t)(interactive_data->laser_time * (496.0f / 3000.0f));
    if (current_width > 496) current_width = 496; // 限制最大长度

    // 注意这里要用到你之前定义的 UI_Laser_Bar，或者你原来用的图层变量
    // 这里使用 change 模式改变直线的终点 X 坐标
    UILineDraw(&UI_Laser_Bar[1], "la1", UI_Graph_Change, 8, UI_Color_Cyan, 25, 712, 315, 712 + current_width, 315);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Laser_Bar[1]);

    interactive_data->Referee_Interactive_Flag.laser_flag = 0;
  }

  // 3. 刷新枪口热量 (Int类型)
  if (interactive_data->Referee_Interactive_Flag.heat_flag == 1) {
    // 接近热量上限(假设上限是200)时，可以做颜色判断，这里先统一用橙色
    UIIntDraw(&UI_State_dyn[4].Graph_Control, "sd4", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 550, interactive_data->heat);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_State_dyn[4].Graph_Control);
    interactive_data->Referee_Interactive_Flag.heat_flag = 0;
  }

  // 4. 刷新射击初速度 (Float类型)
  if (interactive_data->Referee_Interactive_Flag.speed_flag == 1) {
    // UIFloatDraw 要求传入 (实际浮点数 * 1000) 的整形
    int32_t speed_display = (int32_t)(interactive_data->initial_speed * 1000);
    UIFloatDraw(&UI_State_dyn[5].Graph_Control, "sd5", UI_Graph_Change, 8, UI_Color_Cyan, 15, 3, 2, 270, 500, speed_display);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_State_dyn[5].Graph_Control);
    interactive_data->Referee_Interactive_Flag.speed_flag = 0;
  }

  // 5. 刷新自瞄变色准星 (改变 UI_shoot_line[0] 和 [1] 的颜色)
  if (interactive_data->Referee_Interactive_Flag.vision_flag == 1) {
    // 如果开启自瞄，准星变粉色；如果关闭，恢复白色
    uint32_t crosshair_color = (interactive_data->vision_lock == 1) ? UI_Color_Pink : UI_Color_White;

    // 重绘中心十字的两条线 (坐标使用你初始化的宏定义)
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_Change, 7, crosshair_color, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_Change, 7, crosshair_color, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);

    // 批量推送 2 个图形
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[0], UI_shoot_line[1]);

    interactive_data->Referee_Interactive_Flag.vision_flag = 0;
  }
}

void UITask() {
  // 1. 获取全局机器人实例
  RobotInstance *robot = GetRobotInstance();
  if (robot == NULL || robot->referee_data == NULL) return; // 防护：没连上时不刷新

  referee_recv_info = robot->referee_data;

  // 【1】按键强制清屏重绘
  if (interactive_data.force_refresh_ui == 1) {
    MyUIInit();
    interactive_data.force_refresh_ui = 0;
  }

  // 【2】防丢包心跳保底
  static uint16_t slow_refresh_counter = 0;
  if (++slow_refresh_counter >= 150) {
    slow_refresh_counter = 0;
    // 仅仅把无人机拥有的4个状态标志位置1
    interactive_data.Referee_Interactive_Flag.gimbal_flag = 1;
    interactive_data.Referee_Interactive_Flag.shoot_flag = 1;
    interactive_data.Referee_Interactive_Flag.friction_flag = 1;
    interactive_data.Referee_Interactive_Flag.laser_flag = 1;
    interactive_data.Referee_Interactive_Flag.heat_flag = 1;
    interactive_data.Referee_Interactive_Flag.speed_flag = 1;
    interactive_data.Referee_Interactive_Flag.vision_flag = 1;
  }

  // 【3】核心：把真实数据喂给 interactive_data
  // 基础模式
  interactive_data.gimbal_mode   = robot->gimbal->gimbal_ctrl_cmd.gimbal_mode;
  interactive_data.shoot_mode    = robot->shoot->shoot_ctrl_cmd.shoot_mode;
  interactive_data.friction_mode = robot->shoot->shoot_ctrl_cmd.friction_mode;

  // 获取裁判系统的热量和初速
  interactive_data.heat = referee_recv_info->PowerHeatData.shooter_17mm_barrel_heat;
  interactive_data.initial_speed = referee_recv_info->ShootData.initial_speed;

  // 自瞄状态：目前用云台是否处于视觉模式来粗略替代 (如果你有类似 vision_recv_data->is_tracking 的变量，可以替换这里)
  interactive_data.vision_lock = (robot->gimbal->gimbal_ctrl_cmd.gimbal_mode == GIMBAL_VISION) ? 1 : 0;

  // 裁判系统下发的激光照射时间
  interactive_data.laser_time = referee_recv_info->AerialRobotEnergy.attack_time;

  // 【4】检查数据是否有变化
  UIChangeCheck(&interactive_data);

  // 【5】执行最终的图层刷新
  MyUIRefresh(&interactive_data);
}
// 【6】对外提供 UI 数据指针
Referee_Interactive_info_t *getUI(void) {
  return &interactive_data;
}