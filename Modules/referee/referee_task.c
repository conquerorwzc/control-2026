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
static String_Data_t UI_State_sta[6];  // 机器人状态，静态只需画一次
static String_Data_t UI_State_dyn[6];  // 机器人状态，动态先 add 才能 change
static uint32_t shoot_line_location[10] = {540, 960, 490, 515, 565};

static Graph_Data_t Line_DecoMid;
static Graph_Data_t Arc_DecoUp;
static Graph_Data_t Arc_DecoDown;

// === 工程机器人 UI 变量 ===
static String_Data_t UI_engineer_mode_text[2];   // 机器人模式显示 (G 键控制): 标签 + 值
static String_Data_t UI_grab_control_text[2];    // 机械臂控制模式显示 (F 键控制): 标签 + 值
static String_Data_t UI_gripper_status_text[2];  // 夹爪状态显示：标签 + 值
static String_Data_t UI_cali_state_text[4];   // 标定状态显示：Arm 标签、Arm 值、Lift 标签、Lift 值
static String_Data_t UI_motor_angle_name[5];     // 5 个控制器电机名称标签
static String_Data_t UI_motor_angle_value[5];    // 5 个控制器电机角度数值
static String_Data_t UI_arm_angle_value[5];      // 5 个机械臂关节角度数值

// === 新增抬升显示 UI 变量 ===
static Graph_Data_t UI_lift_bar_chassis_bg;    // 底盘抬升进度条背景
static Graph_Data_t UI_lift_bar_chassis_fg;    // 底盘抬升进度条前景（动态值）
static Graph_Data_t UI_lift_bar_arm_bg;        // 机械臂抬升进度条背景
static Graph_Data_t UI_lift_bar_arm_fg;        // 机械臂抬升进度条前景（动态值）
static String_Data_t UI_lift_bar_chassis_text; // 底盘抬升标签 "CHS"
static String_Data_t UI_lift_bar_arm_text;     // 机械臂抬升标签 "ARM"

// === 车辆示宽线 UI 变量 ===
static Graph_Data_t UI_vehicle_width_line_left;   // 左侧示宽线
static Graph_Data_t UI_vehicle_width_line_right;  // 右侧示宽线

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

// Referee_Interactive_Flag_t 已在 rm_referee.h 中定义，此处不再重复定义

typedef struct
{
    Referee_Interactive_Flag_t Referee_Interactive_Flag;
    // 为 UI 绘制以及交互数据所用
    uint8_t arm_cali_state;         // 0-Not Calib, 1-Calibrating, 2-Calib OK
    uint8_t lift_cali_state;        // 0-Not Calib, 1-Calibrating, 2-Calib OK
    
    uint8_t last_arm_cali_state;    // 上一次 Arm 标定状态
    uint8_t last_lift_cali_state;   // 上一次 Lift 标定状态
    
    // === 工程机器人数据 ===
    Robot_Mode_e robot_mode;         // 机器人模式 (G 键控制): 0-断电，1-行车，2-兑换，3-上台阶，4-急停

    uint8_t gripper_opened;          // 夹爪状态：0-关闭，1-打开
    float motor_angles[5];           // 5 个控制器的角度
    float arm_angles[5];             // 5 个机械臂关节角度
    
    // === 新增抬升数据 ===
    float lift_ratio;                // 底盘抬升比例 (0.0~1.0)
    float arm_lift;                  // 机械臂抬升高度

    // 上一次的模式，用于 flag 判断
    
    // === 工程机器人上一次状态 ===
    Robot_Mode_e last_robot_mode;           // 上一次机器人模式

    uint8_t last_gripper_opened;            // 上一次夹爪状态
    float last_motor_angles[5];             // 上一次控制器电机角度
    float last_arm_angles[5];               // 上一次机械臂关节角度
    
    // === 上一次抬升数据 ===
    float last_lift_ratio;                  // 上一次底盘抬升比例
    float last_arm_lift;                    // 上一次机械臂抬升高度
} Referee_Interactive_info_t;

static Referee_Interactive_info_t interactive_data;

// 检测 UI 变化的函数
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    // Arm 标定状态变化检测
    if (_Interactive_data->arm_cali_state != _Interactive_data->last_arm_cali_state)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->last_arm_cali_state = _Interactive_data->arm_cali_state;
    }
    
    // Lift 标定状态变化检测
    if (_Interactive_data->lift_cali_state != _Interactive_data->last_lift_cali_state)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->last_lift_cali_state = _Interactive_data->lift_cali_state;
    }
    
    // === 工程机器人变化检测 ===
    
    // 机器人模式变化检测 (G 键控制)
    if (_Interactive_data->robot_mode != _Interactive_data->last_robot_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.robot_mode_flag = 1;
        _Interactive_data->last_robot_mode = _Interactive_data->robot_mode;
    }

    
    // 夹爪状态变化检测
    if (_Interactive_data->gripper_opened != _Interactive_data->last_gripper_opened)
    {
        _Interactive_data->Referee_Interactive_Flag.gripper_flag = 1;
        _Interactive_data->last_gripper_opened = _Interactive_data->gripper_opened;
    }
}

// UI 更新函数
static void MyUIRefresh(Referee_Interactive_info_t *interactive_data)
{
    // === 标定状态刷新（持续刷新） ===
    char arm_buf[20];
    char lift_buf[20];
    uint32_t arm_color;
    uint32_t lift_color;

    // Arm 状态
    switch (interactive_data->arm_cali_state)
    {
        case 0:
            snprintf(arm_buf, sizeof(arm_buf), "%-14s", "Not Calib");
            arm_color = UI_Color_Purplish_red;
            break;
        case 1:
            snprintf(arm_buf, sizeof(arm_buf), "%-14s", "Calibrating");
            arm_color = UI_Color_Yellow;
            break;
        case 2:
            snprintf(arm_buf, sizeof(arm_buf), "%-14s", "Calib OK");
            arm_color = UI_Color_Green;
            break;
        default:
            snprintf(arm_buf, sizeof(arm_buf), "%-14s", "Unknown");
            arm_color = UI_Color_White;
            break;
    }

    // Lift 状态
    switch (interactive_data->lift_cali_state)
    {
        case 0:
            snprintf(lift_buf, sizeof(lift_buf), "%-14s", "Not Calib");
            lift_color = UI_Color_Purplish_red;
            break;
        case 1:
            snprintf(lift_buf, sizeof(lift_buf), "%-14s", "Calibrating");
            lift_color = UI_Color_Yellow;
            break;
        case 2:
            snprintf(lift_buf, sizeof(lift_buf), "%-14s", "Calib OK");
            lift_color = UI_Color_Green;
            break;
        default:
            snprintf(lift_buf, sizeof(lift_buf), "%-14s", "Unknown");
            lift_color = UI_Color_White;
            break;
    }

    // 第一行：Arm
    UICharDraw(&UI_cali_state_text[1], "ac1", UI_Graph_Change, 7, arm_color, 16, 2, 350, 740, arm_buf);
    UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[1]);

    // 第二行：Lift
    UICharDraw(&UI_cali_state_text[3], "lc1", UI_Graph_Change, 7, lift_color, 16, 2, 350, 700, lift_buf);
    UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[3]);

    // === 工程机器人 UI 刷新 ===

    // 1. 机器人模式显示 (G 键控制)
    if (interactive_data->Referee_Interactive_Flag.robot_mode_flag == 1)
    {
        char mode_buf[20];
        uint32_t mode_color;
        switch (interactive_data->robot_mode)
        {
            case ROBOT_POWER_OFF:
                snprintf(mode_buf, sizeof(mode_buf), "%-10s", "POWER OFF");
                mode_color = UI_Color_Purplish_red;
                break;
            case ROBOT_POWER_ON:
                snprintf(mode_buf, sizeof(mode_buf), "%-10s", "NORMAL");
                mode_color = UI_Color_Green;
                break;
            default:
                snprintf(mode_buf, sizeof(mode_buf), "%-10s", "UNKNOWN");
                mode_color = UI_Color_White;
                break;
        }
        UICharDraw(&UI_engineer_mode_text[1], "rs1", UI_Graph_Change, 7, mode_color, 16, 2, 350, 860, mode_buf);
        UICharRefresh(&referee_recv_info->referee_id, UI_engineer_mode_text[1]);
        interactive_data->Referee_Interactive_Flag.robot_mode_flag = 0;
    }


    // 3. 夹爪状态显示
    if (interactive_data->Referee_Interactive_Flag.gripper_flag == 1)
    {
        char gripper_buf[20];
        uint32_t gripper_color;
        if (interactive_data->gripper_opened)
        {
            snprintf(gripper_buf, sizeof(gripper_buf), "%-10s", "OPEN");
            gripper_color = UI_Color_Green;
        }
        else
        {
            snprintf(gripper_buf, sizeof(gripper_buf), "%-10s", "CLOSED");
            gripper_color = UI_Color_Pink;
        }
        UICharDraw(&UI_gripper_status_text[1], "gs1", UI_Graph_Change, 7, gripper_color, 16, 2, 350, 780, gripper_buf);
        UICharRefresh(&referee_recv_info->referee_id, UI_gripper_status_text[1]);
        interactive_data->Referee_Interactive_Flag.gripper_flag = 0;
    }
      

    // === 5. 底盘抬升显示（进度条 + 文字，每次刷新） ===
    // 计算进度条高度（总高度 140px，底部 y=610，顶部 y=470）
    int chs_bar_height = (int)(interactive_data->lift_ratio * 140.0f);
    if (chs_bar_height < 0) chs_bar_height = 0;
    if (chs_bar_height > 140) chs_bar_height = 140;
    
    // 计算顶部 Y 坐标并确保不小于 470
    int chs_top_y = 610 - chs_bar_height;
    if (chs_top_y < 470) chs_top_y = 470;
    
    // 绘制底盘抬升进度条前景（竖直矩形填充）
    UIRectangleDraw(&UI_lift_bar_chassis_fg, "lbf", UI_Graph_Change, 7, UI_Color_Cyan,
                   6, 1705, chs_top_y, 1730, 610);  // x1=1705, y1=chs_top_y, x2=1730, y2=610
    
    // 发送前景图形更新
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_chassis_fg);
    
    // === 6. 机械臂抬升显示（进度条 + 文字，每次刷新） ===
    // 使用标准归一化公式：(当前值 - 最小值) / (最大值 - 最小值)
    // 结果与 lift_ratio 保持一致，为 0~1 的比例值
    float arm_ratio = 0.0f;  // 0~1 的比例值
    
    // 限制比例范围 0~1
    if (arm_ratio < 0.0f) arm_ratio = 0.0f;
    if (arm_ratio > 1.0f) arm_ratio = 1.0f;
    
    // 转换为百分比用于显示和绘制
    int arm_bar_height = (int)(arm_ratio * 140.0f);
    
    // 计算顶部 Y 坐标并确保不小于 470
    int arm_top_y = 610 - arm_bar_height;
    if (arm_top_y < 470) arm_top_y = 470;
    
    // 绘制机械臂抬升进度条前景（竖直矩形填充）
    UIRectangleDraw(&UI_lift_bar_arm_fg, "laf", UI_Graph_Change, 7, UI_Color_Yellow,
                   6, 1645, arm_top_y, 1670, 610);  // x1=1645, y1=arm_top_y, x2=1670, y2=610
    
    // 发送前景图形更新
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_arm_fg);

    // 4. 5 个电机角度显示（仅文字，每次刷新）
    for (int i = 0; i < 5; i++)
    {
        // 绘制角度数值
        char angle_buf[16];
        char graph_name[4];
        snprintf(graph_name, sizeof(graph_name), "mv%d", i);  // 生成唯一图形名：mv0~mv4 (motor value)
        // 使用固定宽度格式化，右对齐，不足补空格，确保每次长度一致
        snprintf(angle_buf, sizeof(angle_buf), "%6.1f", interactive_data->motor_angles[i]);
        UICharDraw(&UI_motor_angle_value[i], graph_name, UI_Graph_Change, 7, UI_Color_Green, 16, 2,
                  1200 + i * 120, 830, angle_buf);
    }

    // 发送文字更新（角度值）
    for (int i = 0; i < 5; i++)
    {
        UICharRefresh(&referee_recv_info->referee_id, UI_motor_angle_value[i]);
    }

    // 5. 5 个机械臂关节角度显示（仅文字，每次刷新）
    for (int i = 0; i < 5; i++)
    {
        // 绘制角度数值
        char angle_buf[16];
        char graph_name[4];
        snprintf(graph_name, sizeof(graph_name), "av%d", i);  // 生成唯一图形名：av0~av4 (arm value)
        // 使用固定宽度格式化，右对齐，不足补空格，确保每次长度一致
        snprintf(angle_buf, sizeof(angle_buf), "%6.1f", interactive_data->arm_angles[i]);
        UICharDraw(&UI_arm_angle_value[i], graph_name, UI_Graph_Change, 7, UI_Color_Cyan, 16, 2,
                  1200 + i * 120, 800, angle_buf);
    }

    // 发送文字更新（角度值）
    for (int i = 0; i < 5; i++)
    {
        UICharRefresh(&referee_recv_info->referee_id, UI_arm_angle_value[i]);
    }
}

// UIPitchGaugeInit 函数已删除，因为 UI_pitch_ticks 和 UI_pitch_labels 未定义
// void UIPitchGaugeInit(uint32_t center_x,uint32_t center_y,uint32_t radius) {...}

void MyUIInit()
{
    robotdata = RobotGet();
    referee_recv_info = GetRefereeInfo();  // 从裁判系统模块获取数据
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据，等待一段时间后再检查
    DeterminRobotID();                                            // 确定 ui 要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空 UI

    // === 工程机器人 UI 初始化 ===

    // 1. 机器人模式显示 (G 键控制) - 静态标签
    UICharDraw(&UI_engineer_mode_text[0], "rs0", UI_Graph_ADD, 7, UI_Color_White, 16, 2, 80, 860, "Robot State:");
    UICharRefresh(&referee_recv_info->referee_id, UI_engineer_mode_text[0]);
    // 动态值初始化为 POWER OFF
    UICharDraw(&UI_engineer_mode_text[1], "rs1", UI_Graph_ADD, 7, UI_Color_Purplish_red, 16, 2, 350, 860, "POWER OFF");
    UICharRefresh(&referee_recv_info->referee_id, UI_engineer_mode_text[1]);

    // 2. 机械臂控制模式显示 (F 键控制) - 静态标签
    UICharDraw(&UI_grab_control_text[0], "cs0", UI_Graph_ADD, 7, UI_Color_White, 16, 2, 80, 820, "Control Source:");
    UICharRefresh(&referee_recv_info->referee_id, UI_grab_control_text[0]);
    // 动态值初始化为 KEYBOARD
    UICharDraw(&UI_grab_control_text[1], "cs1", UI_Graph_ADD, 7, UI_Color_Green, 16, 2, 350, 820, "KEYBOARD");
    UICharRefresh(&referee_recv_info->referee_id, UI_grab_control_text[1]);

    // 3. 夹爪状态显示 - 静态标签
    UICharDraw(&UI_gripper_status_text[0], "gs0", UI_Graph_ADD, 7, UI_Color_White, 16, 2, 80, 780, "Gripper State:");
    UICharRefresh(&referee_recv_info->referee_id, UI_gripper_status_text[0]);
    // 动态值初始化为 CLOSED
    UICharDraw(&UI_gripper_status_text[1], "gs1", UI_Graph_ADD, 7, UI_Color_Pink, 16, 2, 350, 780, "CLOSED");
    UICharRefresh(&referee_recv_info->referee_id, UI_gripper_status_text[1]);


    // 初始化 5 个电机角度数值（仅文字，无进度条）
    const char* motor_names[] = {"Base", "E_Roll", "E_Pitch", "W_Pitch", "W_Roll"};
    for (int i = 0; i < 5; i++)
    {
        // 绘制电机名称标签
        char graph_name[4];
        snprintf(graph_name, sizeof(graph_name), "mn%d", i);  // 生成唯一图形名：mn0~mn4 (motor name)
        UICharDraw(&UI_motor_angle_name[i], graph_name, UI_Graph_ADD, 7, UI_Color_Yellow, 14, 2,
                 1200 + i * 120, 860, motor_names[i]);  // Y: 390（状态区下方）

        // 初始化电机角度数值为 0°
        char angle_graph_name[4];
        snprintf(angle_graph_name, sizeof(angle_graph_name), "mv%d", i);  // 生成唯一图形名：mv0~mv4 (motor value)
        UICharDraw(&UI_motor_angle_value[i], angle_graph_name, UI_Graph_ADD, 7, UI_Color_Green, 16, 2,
                 1200 + i * 120, 830, "0.0");  // Y: 420（名称下方 30px）
    }

    // 发送文字更新（电机名称）
    for (int i = 0; i < 5; i++)
    {
        UICharRefresh(&referee_recv_info->referee_id, UI_motor_angle_name[i]);
    }

    // 发送文字更新（电机角度初始值）
    for (int i = 0; i < 5; i++)
    {
        UICharRefresh(&referee_recv_info->referee_id, UI_motor_angle_value[i]);
    }

    for (int i = 0; i < 5; i++)
    {
        char angle_graph_name[4];
        snprintf(angle_graph_name, sizeof(angle_graph_name), "av%d", i);
        UICharDraw(&UI_arm_angle_value[i], angle_graph_name, UI_Graph_ADD, 7, UI_Color_Cyan, 16, 2,
                 1200 + i * 120, 800, "0.0");
    }

    // 发送文字更新（机械臂角度初始值）
    for (int i = 0; i < 5; i++)
    {
        UICharRefresh(&referee_recv_info->referee_id, UI_arm_angle_value[i]);
    }
   
    // === 初始化抬升显示 UI ===

    // ARM 背景条（左，竖条）
    UIRectangleDraw(&UI_lift_bar_arm_bg, "lab", UI_Graph_ADD, 7, UI_Color_Yellow,
                   2, 1645, 470, 1670, 610);  // x1=1645, y1=470, x2=1670, y2=610（竖直长条，宽 25px）
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_arm_bg);

    // CHS 背景条（右，竖条）
    UIRectangleDraw(&UI_lift_bar_chassis_bg, "lbb", UI_Graph_ADD, 7, UI_Color_Cyan,
                   2, 1705, 470, 1730, 610);  // x1=1705, y1=470, x2=1730, y2=610（竖直长条，宽 25px）
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_chassis_bg);

    // ARM 文字（放在条下面）
    UICharDraw(&UI_lift_bar_arm_text, "lat", UI_Graph_ADD, 7, UI_Color_Yellow, 14, 2,
              1638, 625, "ARM");  // X=1638 居中，Y=625（条下方）
    UICharRefresh(&referee_recv_info->referee_id, UI_lift_bar_arm_text);

    // CHS 文字（放在条下面）
    UICharDraw(&UI_lift_bar_chassis_text, "lct", UI_Graph_ADD, 7, UI_Color_Cyan, 14, 2,
              1698, 625, "CHS");  // X=1698 居中，Y=625（条下方）
    UICharRefresh(&referee_recv_info->referee_id, UI_lift_bar_chassis_text);

    // === 初始化车辆示宽线 ===
    // 左侧示宽线（更往左张开、上端下移）
    UILineDraw(&UI_vehicle_width_line_left, "vwl", UI_Graph_ADD, 7, UI_Color_Yellow,
               3, 820, 500, 560, 180);  // 从 (820,500) 到 (560,180) 的斜线

    // 右侧示宽线（镜像对称）
    UILineDraw(&UI_vehicle_width_line_right, "vwr", UI_Graph_ADD, 7, UI_Color_Yellow,
               3, 1100, 500, 1360, 180);  // 从 (1100,500) 到 (1360,180) 的斜线

    // 发送示宽线图形
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_vehicle_width_line_left, UI_vehicle_width_line_right);
}

void UITask()
{
    // 首次运行时初始化指针
    if (robotdata == NULL)
    {
        robotdata = RobotGet();
        if (robotdata == NULL)
        {
            // Robot 实例还未初始化，等待一下
            osDelay(100);
            return;
        }
    }

    // 安全检查：确保 chassis 不为 NULL
    if (robotdata->chassis == NULL)
    {
        // chassis 还未初始化，等待一下
        osDelay(100);
        return;
    }

    // === 获取标定状态 (简化版本，固定为未标定) ===
    interactive_data.arm_cali_state = 0;  // Not Calib
    interactive_data.lift_cali_state = 0; // Not Calib
        
    // 获取夹爪状态 (简化版本，固定为关闭)
    interactive_data.gripper_opened = 0;   // CLOSED

    // 获取 5 个电机角度 (简化版本，全部设为 0)
    for (int i = 0; i < 5; i++)
    {
        interactive_data.motor_angles[i] = 0.0f;
    }
    
    // 获取 5 个机械臂关节角度 (简化版本，全部设为 0)
    for (int i = 0; i < 5; i++)
    {
        interactive_data.arm_angles[i] = 0.0f;
    }
    
    // === 获取抬升数据 ===

    // 获取底盘抬升比例 (从底盘控制命令)
    interactive_data.lift_ratio = 0.0f;

    // 获取机械臂抬升高度 (简化版本，设为 0)
    interactive_data.arm_lift = 0.0f;

    // 检查是否有变化
    UIChangeCheck(&interactive_data);

    // 执行 UI 刷新
    MyUIRefresh(&interactive_data);
}
