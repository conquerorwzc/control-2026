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

    // 上一次的模式，用于flag判断
    Chassis_Mode_e chassis_last_mode;
    Gimbal_Mode_e gimbal_last_mode;
    Shoot_Mode_e shoot_last_mode;
    Friction_Mode_e friction_last_mode;
    lid_mode_e lid_last_mode;   
    Chassis_Power_Data_s Chassis_last_Power_Data;
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
}

// UI更新函数
static void MyUIRefresh(Referee_Interactive_info_t *_Interactive_data)
{
    // 更新底盘状态
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        char *chassis_str;
        switch(_Interactive_data->chassis_mode)
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
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }
    
    // 更新云台状态
    if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
    {
        char *gimbal_str;
        switch(_Interactive_data->gimbal_mode)
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
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
    }
    
    // 更新射击状态
    if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
    {
        char *shoot_str = _Interactive_data->shoot_mode == SHOOT_ON ? "on" : "off";
        UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 650, shoot_str);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    }
    
    // 更新摩擦轮状态
    if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    {
        char *friction_str = _Interactive_data->friction_mode == FRICTION_ON ? "on" : "off";
        UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, friction_str);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    }
    
    // 更新弹舱盖状态
    if (_Interactive_data->Referee_Interactive_Flag.lid_flag == 1)
    {
        char *lid_str = _Interactive_data->lid_mode == LID_OPEN ? "open" : "close";
        UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, lid_str);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 0;
    }
    
    // 更新功率显示
    if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1)
    {
        int32_t power_value = (int32_t)(_Interactive_data->Chassis_Power_Data.chassis_power_mx * 1000);
        UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 750, 230, power_value);
        
        // 更新能量条长度
        uint32_t energy_bar_length = 720 + (uint32_t)(_Interactive_data->Chassis_Power_Data.chassis_power_mx * 300);
        UILineDraw(&UI_Energy[2], "sd6", UI_Graph_Change, 8, UI_Color_Pink, 30, 720, 160, energy_bar_length, 160);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 0;
    }
}
//newend
void MyUIInit()
{
    robotdata=RobotInit();
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
    UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 750, "zeroforce");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
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
}

// 实现缺失的UITask函数
void UITask()
{
    // 首次运行时初始化指针
    if (referee_recv_info == NULL) {
        referee_recv_info = RefereeInit(NULL); // 假设使用默认串口
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