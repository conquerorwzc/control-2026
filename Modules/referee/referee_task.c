/**
 * @file referee_task.c
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 */
#include "referee_task.h"
#include "cmsis_os.h"
#include "referee_UI.h"
#include "rm_referee.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "user_lib.h"

#ifdef ROBOT_ENGINEERING
#include "robot.h"
#endif

static referee_info_t *referee_recv_info; // 接收到的裁判系统数据
uint8_t UI_Seq;                           // 包序号，供整个referee文件使用

/**
 * @brief  判断各种ID，选择客户端ID
 */
static void DeterminRobotID()
{
    if (referee_recv_info == NULL)
        return;
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

// =================================================================================
// 👇 以下是工程机器人专属 UI 变量与函数区
// =================================================================================
#ifdef ROBOT_ENGINEERING

// 🌟 核心修复：找回定点一位小数格式化函数，彻底解决嵌入式不支持 %f 的空输出问题！
static void fmt_1dp(char *buf, size_t n, float x)
{
    int32_t x10 = (int32_t)(x * 10.0f + (x >= 0 ? 0.5f : -0.5f));
    int32_t ip = x10 / 10;
    int32_t fp = x10 % 10;
    if (fp < 0) fp = -fp;
    snprintf(buf, n, "%ld.%01ld", (long)ip, (long)fp);
}

RobotInstance *robotdata;

// === 工程机器人 UI 变量 ===
static String_Data_t UI_engineer_mode_text[2];
static String_Data_t UI_grab_control_text[2];
static String_Data_t UI_gripper_status_text[2];
static String_Data_t UI_cali_state_text[8];
static String_Data_t UI_motor_angle_name[5];
static String_Data_t UI_motor_angle_value[5];
static String_Data_t UI_arm_angle_value[5];

// === 新增抬升显示 UI 变量 ===
static Graph_Data_t UI_lift_bar_chassis_bg;
static Graph_Data_t UI_lift_bar_chassis_fg;
static Graph_Data_t UI_lift_bar_arm_bg;
static Graph_Data_t UI_lift_bar_arm_fg;
static String_Data_t UI_lift_bar_chassis_text;
static String_Data_t UI_lift_bar_arm_text;

// === 车辆示宽线 UI 变量 ===
static Graph_Data_t UI_vehicle_width_line_left;
static Graph_Data_t UI_vehicle_width_line_right;

typedef struct
{
    Referee_Interactive_Flag_t Referee_Interactive_Flag;
    uint8_t wrist_cali_state;
    uint8_t extend_cali_state;
    uint8_t lift_zero_state;
    uint8_t lift_max_state;

    uint8_t last_wrist_cali_state;
    uint8_t last_extend_cali_state;
    uint8_t last_lift_zero_state;
    uint8_t last_lift_max_state;
    Robot_Mode_e robot_mode;
    GrabControlMode_e grab_control_mode;
    uint8_t gripper_opened;

    float lift_ratio;
    float arm_lift;
    Robot_Mode_e last_robot_mode;
    GrabControlMode_e last_grab_control_mode;
    uint8_t last_gripper_opened;

    float arm_err_angles[5];
    float joint_loads[5];
} Referee_Interactive_info_t;

static Referee_Interactive_info_t interactive_data;

// 检测 UI 变化的函数
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->wrist_cali_state != _Interactive_data->last_wrist_cali_state) {
        _Interactive_data->Referee_Interactive_Flag.wrist_cali_flag = 1;
        _Interactive_data->last_wrist_cali_state = _Interactive_data->wrist_cali_state;
    }
    if (_Interactive_data->extend_cali_state != _Interactive_data->last_extend_cali_state) {
        _Interactive_data->Referee_Interactive_Flag.extend_cali_flag = 1;
        _Interactive_data->last_extend_cali_state = _Interactive_data->extend_cali_state;
    }
    if (_Interactive_data->lift_zero_state != _Interactive_data->last_lift_zero_state) {
        _Interactive_data->Referee_Interactive_Flag.lift_zero_flag = 1;
        _Interactive_data->last_lift_zero_state = _Interactive_data->lift_zero_state;
    }
    if (_Interactive_data->lift_max_state != _Interactive_data->last_lift_max_state) {
        _Interactive_data->Referee_Interactive_Flag.lift_max_flag = 1;
        _Interactive_data->last_lift_max_state = _Interactive_data->lift_max_state;
    }
    if (_Interactive_data->robot_mode != _Interactive_data->last_robot_mode) {
        _Interactive_data->Referee_Interactive_Flag.robot_mode_flag = 1;
        _Interactive_data->last_robot_mode = _Interactive_data->robot_mode;
    }
    if (_Interactive_data->grab_control_mode != _Interactive_data->last_grab_control_mode) {
        _Interactive_data->Referee_Interactive_Flag.grab_control_flag = 1;
        _Interactive_data->last_grab_control_mode = _Interactive_data->grab_control_mode;
    }
    if (_Interactive_data->gripper_opened != _Interactive_data->last_gripper_opened) {
        _Interactive_data->Referee_Interactive_Flag.gripper_flag = 1;
        _Interactive_data->last_gripper_opened = _Interactive_data->gripper_opened;
    }
}

static void MyUIRefresh(Referee_Interactive_info_t *data_ptr)
{
    // 1. 标定文字刷新部分
    if (data_ptr->Referee_Interactive_Flag.wrist_cali_flag) {
        char buf[20]; uint32_t color;
        if (data_ptr->wrist_cali_state == CALI_DONE) { snprintf(buf, sizeof(buf), "%-10s", "OK"); color = UI_Color_Green; }
        else if (data_ptr->wrist_cali_state == CALI_RUNNING) { snprintf(buf, sizeof(buf), "%-10s", "RUNNING"); color = UI_Color_Yellow; }
        else { snprintf(buf, sizeof(buf), "%-10s", "ERROR"); color = UI_Color_Purplish_red; }
        UICharDraw(&UI_cali_state_text[1], "wc1", UI_Graph_Change, 7, color, 16, 2, 350, 740, buf);
        UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[1]);
        data_ptr->Referee_Interactive_Flag.wrist_cali_flag = 0;
    }

    if (data_ptr->Referee_Interactive_Flag.extend_cali_flag) {
        char buf[20]; uint32_t color;
        if (data_ptr->extend_cali_state == CALI_DONE) { snprintf(buf, sizeof(buf), "%-10s", "OK"); color = UI_Color_Green; }
        else if (data_ptr->extend_cali_state == CALI_RUNNING) { snprintf(buf, sizeof(buf), "%-10s", "RUNNING"); color = UI_Color_Yellow; }
        else { snprintf(buf, sizeof(buf), "%-10s", "ERROR"); color = UI_Color_Purplish_red; }
        UICharDraw(&UI_cali_state_text[3], "ec1", UI_Graph_Change, 7, color, 16, 2, 350, 700, buf);
        UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[3]);
        data_ptr->Referee_Interactive_Flag.extend_cali_flag = 0;
    }

    if (data_ptr->Referee_Interactive_Flag.lift_zero_flag) {
        char buf[20]; uint32_t color;
        if (data_ptr->lift_zero_state == CALI_DONE) { snprintf(buf, sizeof(buf), "%-10s", "OK"); color = UI_Color_Green; }
        else if (data_ptr->lift_zero_state == CALI_RUNNING) { snprintf(buf, sizeof(buf), "%-10s", "RUNNING"); color = UI_Color_Yellow; }
        else { snprintf(buf, sizeof(buf), "%-10s", "ERROR"); color = UI_Color_Purplish_red; }
        UICharDraw(&UI_cali_state_text[5], "lz1", UI_Graph_Change, 7, color, 16, 2, 350, 660, buf);
        UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[5]);
        data_ptr->Referee_Interactive_Flag.lift_zero_flag = 0;
    }

    if (data_ptr->Referee_Interactive_Flag.lift_max_flag) {
        char buf[20]; uint32_t color;
        if (data_ptr->lift_max_state == CALI_DONE) { snprintf(buf, sizeof(buf), "%-10s", "OK"); color = UI_Color_Green; }
        else if (data_ptr->lift_max_state == CALI_RUNNING) { snprintf(buf, sizeof(buf), "%-10s", "RUNNING"); color = UI_Color_Yellow; }
        else { snprintf(buf, sizeof(buf), "%-10s", "ERROR"); color = UI_Color_Purplish_red; }
        UICharDraw(&UI_cali_state_text[7], "lm1", UI_Graph_Change, 7, color, 16, 2, 350, 620, buf);
        UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[7]);
        data_ptr->Referee_Interactive_Flag.lift_max_flag = 0;
    }

    // 2. 大模式与控制源刷新
    if (data_ptr->Referee_Interactive_Flag.robot_mode_flag == 1) {
        char buf[20]; uint32_t color;
        if (data_ptr->robot_mode == ROBOT_EXCHANGE_MODE) { snprintf(buf, sizeof(buf), "%-15s", "EXCHANGE"); color = UI_Color_Pink; }
        else if (data_ptr->robot_mode == ROBOT_CLIMB_MODE) { snprintf(buf, sizeof(buf), "%-15s", "CLIMB_UP"); color = UI_Color_Orange; }
        else if (data_ptr->robot_mode == ROBOT_DOWN_STAIRS_MODE) { snprintf(buf, sizeof(buf), "%-15s", "CLIMB_DOWN"); color = UI_Color_Orange; }
        else if (data_ptr->robot_mode == ROBOT_POWER_ON) { snprintf(buf, sizeof(buf), "%-15s", "NORMAL"); color = UI_Color_Green; }
        else { snprintf(buf, sizeof(buf), "%-15s", "POWER OFF"); color = UI_Color_Purplish_red; }
        UICharDraw(&UI_engineer_mode_text[1], "rs1", UI_Graph_Change, 7, color, 16, 2, 350, 860, buf);
        UICharRefresh(&referee_recv_info->referee_id, UI_engineer_mode_text[1]);
        data_ptr->Referee_Interactive_Flag.robot_mode_flag = 0;
    }

    if (data_ptr->Referee_Interactive_Flag.grab_control_flag == 1) {
        char buf[20]; uint32_t color;
        if (data_ptr->grab_control_mode == GRAB_CONTROL_KEYBOARD) { snprintf(buf, sizeof(buf), "%-15s", "KEYBOARD"); color = UI_Color_Green; }
        else if (data_ptr->grab_control_mode == GRAB_CONTROL_HALF_AUTO) { snprintf(buf, sizeof(buf), "%-15s", "HALF_AUTO"); color = UI_Color_Cyan; }
        else { snprintf(buf, sizeof(buf), "%-15s", "CUSTOM_CTRL"); color = UI_Color_Orange; }
        UICharDraw(&UI_grab_control_text[1], "cs1", UI_Graph_Change, 7, color, 16, 2, 350, 820, buf);
        UICharRefresh(&referee_recv_info->referee_id, UI_grab_control_text[1]);
        data_ptr->Referee_Interactive_Flag.grab_control_flag = 0;
    }

    if (data_ptr->Referee_Interactive_Flag.gripper_flag == 1) {
        if (data_ptr->gripper_opened) {
            UICharDraw(&UI_gripper_status_text[1], "gs1", UI_Graph_Change, 7, UI_Color_Yellow, 16, 2, 350, 780, "OPENED");
        } else {
            UICharDraw(&UI_gripper_status_text[1], "gs1", UI_Graph_Change, 7, UI_Color_Pink, 16, 2, 350, 780, "CLOSED");
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_gripper_status_text[1]);
        data_ptr->Referee_Interactive_Flag.gripper_flag = 0;
    }

    // 3. 实时能量条刷新
    int chs_bar_height = (int)(data_ptr->lift_ratio * 140.0f);
    if (chs_bar_height < 0) chs_bar_height = 0;
    if (chs_bar_height > 140) chs_bar_height = 140;

    UIRectangleDraw(&UI_lift_bar_chassis_fg, "lbf", UI_Graph_Change, 7, UI_Color_Cyan, 6, 1705, 610, 1730, 610 - chs_bar_height);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_chassis_fg);

    float arm_ratio = 0.0f;
    if (robotdata != NULL && robotdata->grab != NULL && robotdata->grab->arm != NULL) {
        float max_lift = robotdata->grab->arm->arm_lift_max;
        if (max_lift > 10.0f) arm_ratio = data_ptr->arm_lift / max_lift;
    }
    if (arm_ratio < 0.0f) arm_ratio = 0.0f;
    if (arm_ratio > 1.0f) arm_ratio = 1.0f;

    uint16_t arm_top_y = 610 - (uint16_t)(arm_ratio * 140.0f);
    UIRectangleDraw(&UI_lift_bar_arm_fg, "laf", UI_Graph_Change, 7, UI_Color_Yellow, 6, 1645, 610, 1670, arm_top_y);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_arm_fg);

    // ==============================================================
    // 🌟 4. 第一排看板：显示负载率 (%)
    // ==============================================================
    for (int i = 0; i < 5; i++) {
        char graph_name[4];
        snprintf(graph_name, sizeof(graph_name), "mv%d", i);

        uint32_t load_color = UI_Color_Green;
        if (data_ptr->joint_loads[i] > 85.0f) load_color = UI_Color_Purplish_red;
        else if (data_ptr->joint_loads[i] > 60.0f) load_color = UI_Color_Yellow;

        // 使用 %d 打印整数，使用 %% 打印百分号符号，安全防崩！
        UICharDraw(&UI_motor_angle_value[i], graph_name, UI_Graph_Change, 7, load_color, 16, 2, 1200 + i * 120, 830, "L:%d%%", (int)data_ptr->joint_loads[i]);
        UICharRefresh(&referee_recv_info->referee_id, UI_motor_angle_value[i]);
    }

    // ==============================================================
    // 🌟 5. 第二排看板：显示误差角度 (E: xxx.x)
    // ==============================================================
    for (int i = 0; i < 5; i++) {
        char graph_name[4];
        snprintf(graph_name, sizeof(graph_name), "av%d", i);

        float err = data_ptr->arm_err_angles[i];
        uint32_t err_color = UI_Color_Cyan;
        if (fabsf(err) > 5.0f) err_color = UI_Color_Orange;

        // 核心修复：使用 fmt_1dp 手动转浮点数，再用 %s 安全打印
        char float_str[10];
        fmt_1dp(float_str, sizeof(float_str), err);
        UICharDraw(&UI_arm_angle_value[i], graph_name, UI_Graph_Change, 7, err_color, 16, 2, 1200 + i * 120, 800, "E:%s", float_str);
        UICharRefresh(&referee_recv_info->referee_id, UI_arm_angle_value[i]);
    }
}

#endif // ROBOT_ENGINEERING

// =================================================================================
// 👇 以下是通用任务对外接口 (暴露给多机器人系统)
// =================================================================================

void MyUIInit()
{
    referee_recv_info = GetRefereeInfo();
    if (referee_recv_info == NULL || referee_recv_info->GameRobotState.robot_id == 0)
    {
        return;
    }

    DeterminRobotID();
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0);

#ifdef ROBOT_ENGINEERING
    robotdata = RobotGet();

    UICharDraw(&UI_engineer_mode_text[0], "rs0", UI_Graph_ADD, 7, UI_Color_White, 16, 2, 80, 860, "Robot State:");
    UICharRefresh(&referee_recv_info->referee_id, UI_engineer_mode_text[0]);
    UICharDraw(&UI_engineer_mode_text[1], "rs1", UI_Graph_ADD, 7, UI_Color_Purplish_red, 16, 2, 350, 860, "POWER OFF");
    UICharRefresh(&referee_recv_info->referee_id, UI_engineer_mode_text[1]);

    UICharDraw(&UI_grab_control_text[0], "cs0", UI_Graph_ADD, 7, UI_Color_White, 16, 2, 80, 820, "Control Source:");
    UICharRefresh(&referee_recv_info->referee_id, UI_grab_control_text[0]);
    UICharDraw(&UI_grab_control_text[1], "cs1", UI_Graph_ADD, 7, UI_Color_Green, 16, 2, 350, 820, "KEYBOARD");
    UICharRefresh(&referee_recv_info->referee_id, UI_grab_control_text[1]);

    UICharDraw(&UI_gripper_status_text[0], "gs0", UI_Graph_ADD, 7, UI_Color_White, 16, 2, 80, 780, "Gripper State:");
    UICharRefresh(&referee_recv_info->referee_id, UI_gripper_status_text[0]);
    UICharDraw(&UI_gripper_status_text[1], "gs1", UI_Graph_ADD, 7, UI_Color_Pink, 16, 2, 350, 780, "CLOSED");
    UICharRefresh(&referee_recv_info->referee_id, UI_gripper_status_text[1]);

    UICharDraw(&UI_cali_state_text[0], "wc0", UI_Graph_ADD, 7, UI_Color_White, 16, 2, 80, 740, "Wrist Cali:");
    UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[0]);
    UICharDraw(&UI_cali_state_text[1], "wc1", UI_Graph_ADD, 7, UI_Color_Purplish_red, 16, 2, 350, 740, "ERROR");
    UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[1]);

    UICharDraw(&UI_cali_state_text[2], "ec0", UI_Graph_ADD, 7, UI_Color_White, 16, 2, 80, 700, "Extend Cali:");
    UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[2]);
    UICharDraw(&UI_cali_state_text[3], "ec1", UI_Graph_ADD, 7, UI_Color_Purplish_red, 16, 2, 350, 700, "ERROR");
    UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[3]);

    UICharDraw(&UI_cali_state_text[4], "lz0", UI_Graph_ADD, 7, UI_Color_White, 16, 2, 80, 660, "Lift Zero:");
    UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[4]);
    UICharDraw(&UI_cali_state_text[5], "lz1", UI_Graph_ADD, 7, UI_Color_Purplish_red, 16, 2, 350, 660, "ERROR");
    UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[5]);

    UICharDraw(&UI_cali_state_text[6], "lm0", UI_Graph_ADD, 7, UI_Color_White, 16, 2, 80, 620, "Lift Max:");
    UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[6]);
    UICharDraw(&UI_cali_state_text[7], "lm1", UI_Graph_ADD, 7, UI_Color_Purplish_red, 16, 2, 350, 620, "ERROR");
    UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[7]);

    const char *motor_names[] = {"Base", "E_Roll", "E_Pitch", "W_Pitch", "W_Roll"};
    for (int i = 0; i < 5; i++)
    {
        char graph_name[4];
        snprintf(graph_name, sizeof(graph_name), "mn%d", i);
        UICharDraw(&UI_motor_angle_name[i], graph_name, UI_Graph_ADD, 7, UI_Color_Yellow, 14, 2, 1200 + i * 120, 860,
                   (char *)motor_names[i]);
        UICharRefresh(&referee_recv_info->referee_id, UI_motor_angle_name[i]);

        char angle_graph_name[4];
        snprintf(angle_graph_name, sizeof(angle_graph_name), "mv%d", i);
        UICharDraw(&UI_motor_angle_value[i], angle_graph_name, UI_Graph_ADD, 7, UI_Color_Green, 16, 2, 1200 + i * 120,
                   830, "L:0%%");
        UICharRefresh(&referee_recv_info->referee_id, UI_motor_angle_value[i]);
    }

    for (int i = 0; i < 5; i++)
    {
        char angle_graph_name[4];
        snprintf(angle_graph_name, sizeof(angle_graph_name), "av%d", i);
        UICharDraw(&UI_arm_angle_value[i], angle_graph_name, UI_Graph_ADD, 7, UI_Color_Cyan, 16, 2, 1200 + i * 120, 800,
                   "E:0.0");
        UICharRefresh(&referee_recv_info->referee_id, UI_arm_angle_value[i]);
    }

    UIRectangleDraw(&UI_lift_bar_arm_bg, "lab", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 1645, 470, 1670, 610);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_arm_bg);

    UIRectangleDraw(&UI_lift_bar_arm_fg, "laf", UI_Graph_ADD, 7, UI_Color_Yellow, 6, 1645, 610, 1670, 610);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_arm_fg);

    UIRectangleDraw(&UI_lift_bar_chassis_bg, "lbb", UI_Graph_ADD, 7, UI_Color_Cyan, 2, 1705, 470, 1730, 610);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_chassis_bg);

    UIRectangleDraw(&UI_lift_bar_chassis_fg, "lbf", UI_Graph_ADD, 7, UI_Color_Cyan, 6, 1705, 610, 1730, 610);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_chassis_fg);

    UICharDraw(&UI_lift_bar_arm_text, "lat", UI_Graph_ADD, 7, UI_Color_Yellow, 14, 2, 1638, 625, "ARM");
    UICharRefresh(&referee_recv_info->referee_id, UI_lift_bar_arm_text);

    UICharDraw(&UI_lift_bar_chassis_text, "lct", UI_Graph_ADD, 7, UI_Color_Cyan, 14, 2, 1698, 625, "CHS");
    UICharRefresh(&referee_recv_info->referee_id, UI_lift_bar_chassis_text);

    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_vehicle_width_line_left, UI_vehicle_width_line_right);
#endif // ROBOT_ENGINEERING
}

void UITask()
{
#ifdef ROBOT_ENGINEERING
    if (robotdata == NULL) {
        robotdata = RobotGet();
        if (robotdata == NULL) { osDelay(100); return; }
    }
    if (robotdata->chassis == NULL) { osDelay(100); return; }

    // 1. UI 重置检查
    if (robotdata->ui_reset_flag == 1) {
        if (referee_recv_info == NULL) referee_recv_info = GetRefereeInfo();
        if (referee_recv_info != NULL && referee_recv_info->GameRobotState.robot_id != 0) {
            MyUIInit();
            interactive_data.Referee_Interactive_Flag.wrist_cali_flag = 1;
            interactive_data.Referee_Interactive_Flag.extend_cali_flag = 1;
            interactive_data.Referee_Interactive_Flag.lift_zero_flag = 1;
            interactive_data.Referee_Interactive_Flag.lift_max_flag = 1;
            interactive_data.Referee_Interactive_Flag.robot_mode_flag = 1;
            interactive_data.Referee_Interactive_Flag.grab_control_flag = 1;
            interactive_data.Referee_Interactive_Flag.gripper_flag = 1;
            robotdata->ui_reset_flag = 0;
        }
    }

    // 2. 状态机提取 (精准对接 OOP 标定类)
    if (robotdata->grab != NULL && robotdata->grab->actuator != NULL)
        interactive_data.wrist_cali_state = robotdata->grab->actuator->wrist_cali_obj.state;
    if (robotdata->grab != NULL && robotdata->grab->arm != NULL)
        interactive_data.extend_cali_state = robotdata->grab->arm->extend_cali_obj.state;

    if (robotdata->chassis != NULL) {
        if (robotdata->chassis->cali_state.all_cali_done)
            interactive_data.lift_zero_state = CALI_DONE;
        else if (robotdata->chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_CALIBRATING)
            interactive_data.lift_zero_state = CALI_RUNNING;
        else
            interactive_data.lift_zero_state = CALI_ERROR;

        if (robotdata->chassis->cali_state.is_max_calibrated)
            interactive_data.lift_max_state = CALI_DONE;
        else if (robotdata->chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_CLIMB_BOTH_EXTEND &&
                 robotdata->chassis->cali_state.all_cali_done)
            interactive_data.lift_max_state = CALI_RUNNING;
        else
            interactive_data.lift_max_state = CALI_ERROR;
    }

    // 3. 底盘物理抬升反馈
    if (robotdata->chassis->cali_state.is_max_calibrated) {
        float stroke = fabsf(robotdata->chassis->cali_state.max_angle[2] - robotdata->chassis->cali_state.init_angle[2]);
        if (stroke > 100.0f) {
            float cur = fabsf(robotdata->chassis->front_legs[0].motor->measure.total_angle - robotdata->chassis->cali_state.init_angle[2]);
            interactive_data.lift_ratio = cur / stroke;
        }
    } else {
        interactive_data.lift_ratio = 0.0f;
    }

    // 4. 机械臂物理抬升位移
    if (robotdata->grab != NULL)
        interactive_data.arm_lift = robotdata->grab->grab_measure.arm_lift;
    else
        interactive_data.arm_lift = 0.0f;

    // 🌟 5. 5大关节：计算底层命令的负载率(%) 与 误差角度(E)
    if (robotdata->grab != NULL && robotdata->grab->arm != NULL && robotdata->grab->actuator != NULL) {

        // 计算达妙(前三轴)负载率
        for (int i = 0; i < 3; i++) {
            float tq_cmd = robotdata->grab->arm->grab_dmmotor[i]->motor_controller.final_output;
            interactive_data.joint_loads[i] = fabsf(tq_cmd) / 28.0f * 100.0f;
        }

        // 计算大疆(后两轴)负载率
        float c3_cmd = robotdata->grab->actuator->grab_djimotor[0]->motor_controller.final_output;
        interactive_data.joint_loads[3] = fabsf(c3_cmd) / 10000.0f * 100.0f;
        float c4_cmd = robotdata->grab->actuator->grab_djimotor[2]->motor_controller.final_output;
        interactive_data.joint_loads[4] = fabsf(c4_cmd) / 10000.0f * 100.0f;

        // 🌟 核心修复：获取真实的物理角度！
        // 必须用 grab_measure，不能用 arm 或 actuator 里的缓存数据（那里面存的是指令）
        float real_angles[5];
        real_angles[0] = robotdata->grab->grab_measure.base_joint;
        real_angles[1] = robotdata->grab->grab_measure.elbow_roll;
        real_angles[2] = robotdata->grab->grab_measure.elbow_pitch;
        real_angles[3] = robotdata->grab->grab_measure.wrist_pitch;
        real_angles[4] = robotdata->grab->grab_measure.wrist_roll;

        // 获取控制器下发的指令角度
        float cmd_angles[5];
        cmd_angles[0] = robotdata->grab->grab_ctrl_cmd.base_joint;
        cmd_angles[1] = robotdata->grab->grab_ctrl_cmd.elbow_roll;
        cmd_angles[2] = robotdata->grab->grab_ctrl_cmd.elbow_pitch;
        cmd_angles[3] = robotdata->grab->grab_ctrl_cmd.wrist_pitch;
        cmd_angles[4] = robotdata->grab->grab_ctrl_cmd.wrist_roll;

        // 误差 = 真实反馈角度 - 目标指令角度
        for (int i = 0; i < 5; i++) {
            interactive_data.arm_err_angles[i] = real_angles[i] - cmd_angles[i];
        }

    } else {
        for (int i=0; i<5; i++) {
            interactive_data.joint_loads[i] = 0.0f;
            interactive_data.arm_err_angles[i] = 0.0f;
        }
    }

    interactive_data.robot_mode = robotdata->robot_mode;
    interactive_data.grab_control_mode = GetGrabControlMode();
    if (robotdata->grab != NULL)
        interactive_data.gripper_opened = (robotdata->grab->grab_ctrl_cmd.gripper_state == GRIPPER_OPEN) ? 1 : 0;
    else
        interactive_data.gripper_opened = 0;

    UIChangeCheck(&interactive_data);
    MyUIRefresh(&interactive_data);
#endif
}