/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 */
#include "referee_task.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "stdint.h"

// 👇 只有当前配置为工程机器人时，才引用工程的 robot 库
#ifdef ROBOT_ENGINEERING
#include "robot.h"
#endif

static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用

/**
 * @brief  判断各种ID，选择客户端ID
 */
static void DeterminRobotID()
{
    if (referee_recv_info == NULL) return;
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

// 定点一位小数格式化函数：将 float 转为 x.y 格式的字符串
static void fmt_1dp(char *buf, size_t n, float x)
{
    int32_t x10 = (int32_t)(x * 10.0f + (x >= 0 ? 0.5f : -0.5f));
    int32_t ip = x10 / 10;
    int32_t fp = x10 % 10;
    if (fp < 0) fp = -fp;
    snprintf(buf, n, "%ld.%01ld", (long)ip, (long)fp);
}

RobotInstance* robotdata;

// === 工程机器人 UI 变量 ===
static String_Data_t UI_engineer_mode_text[2];
static String_Data_t UI_grab_control_text[2];
static String_Data_t UI_gripper_status_text[2];
static String_Data_t UI_cali_state_text[4];
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
    uint8_t arm_cali_state;
    uint8_t lift_cali_state;
    uint8_t last_arm_cali_state;
    uint8_t last_lift_cali_state;
    Robot_Mode_e robot_mode;
    GrabControlMode_e grab_control_mode;
    uint8_t gripper_opened;
    float motor_angles[5];
    float arm_angles[5];
    float lift_ratio;
    float arm_lift;
    Robot_Mode_e last_robot_mode;
    GrabControlMode_e last_grab_control_mode;
    uint8_t last_gripper_opened;
    float last_motor_angles[5];
    float last_arm_angles[5];
    float last_lift_ratio;
    float last_arm_lift;
} Referee_Interactive_info_t;

static Referee_Interactive_info_t interactive_data;

// 检测 UI 变化的函数
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->arm_cali_state != _Interactive_data->last_arm_cali_state) {
        _Interactive_data->Referee_Interactive_Flag.arm_cali_flag = 1;
        _Interactive_data->last_arm_cali_state = _Interactive_data->arm_cali_state;
    }
    if (_Interactive_data->lift_cali_state != _Interactive_data->last_lift_cali_state) {
        _Interactive_data->Referee_Interactive_Flag.lift_cali_flag = 1;
        _Interactive_data->last_lift_cali_state = _Interactive_data->lift_cali_state;
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

// UI 更新函数
static void MyUIRefresh(Referee_Interactive_info_t *interactive_data)
{
    if (interactive_data->Referee_Interactive_Flag.arm_cali_flag == 1) {
        char arm_buf[20];
        uint32_t arm_color;
        switch (interactive_data->arm_cali_state) {
            case 0: snprintf(arm_buf, sizeof(arm_buf), "%-14s", "Not Calib"); arm_color = UI_Color_Purplish_red; break;
            case 1: snprintf(arm_buf, sizeof(arm_buf), "%-14s", "Calibrating"); arm_color = UI_Color_Yellow; break;
            case 2: snprintf(arm_buf, sizeof(arm_buf), "%-14s", "Calib OK"); arm_color = UI_Color_Green; break;
            default: snprintf(arm_buf, sizeof(arm_buf), "%-14s", "Unknown"); arm_color = UI_Color_White; break;
        }
        UICharDraw(&UI_cali_state_text[1], "ac1", UI_Graph_Change, 7, arm_color, 16, 2, 350, 740, arm_buf);
        UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[1]);
        interactive_data->Referee_Interactive_Flag.arm_cali_flag = 0;
    }

    if (interactive_data->Referee_Interactive_Flag.lift_cali_flag == 1) {
        char lift_buf[20];
        uint32_t lift_color;
        switch (interactive_data->lift_cali_state) {
            case 0: snprintf(lift_buf, sizeof(lift_buf), "%-14s", "Not Calib"); lift_color = UI_Color_Purplish_red; break;
            case 1: snprintf(lift_buf, sizeof(lift_buf), "%-14s", "Calibrating"); lift_color = UI_Color_Yellow; break;
            case 2: snprintf(lift_buf, sizeof(lift_buf), "%-14s", "Calib OK"); lift_color = UI_Color_Green; break;
            default: snprintf(lift_buf, sizeof(lift_buf), "%-14s", "Unknown"); lift_color = UI_Color_White; break;
        }
        UICharDraw(&UI_cali_state_text[3], "lc1", UI_Graph_Change, 7, lift_color, 16, 2, 350, 700, lift_buf);
        UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[3]);
        interactive_data->Referee_Interactive_Flag.lift_cali_flag = 0;
    }

    if (interactive_data->Referee_Interactive_Flag.robot_mode_flag == 1) {
        char mode_buf[20];
        uint32_t mode_color;
        switch (interactive_data->robot_mode) {
            case ROBOT_POWER_OFF: snprintf(mode_buf, sizeof(mode_buf), "%-10s", "POWER OFF"); mode_color = UI_Color_Purplish_red; break;
            case ROBOT_POWER_ON: snprintf(mode_buf, sizeof(mode_buf), "%-10s", "NORMAL"); mode_color = UI_Color_Green; break;
            case ROBOT_EXCHANGE_MODE: snprintf(mode_buf, sizeof(mode_buf), "%-10s", "EXCHANGE"); mode_color = UI_Color_Cyan; break;
            case ROBOT_CLIMB_MODE: snprintf(mode_buf, sizeof(mode_buf), "%-10s", "CLIMB"); mode_color = UI_Color_Yellow; break;
            case ROBOT_EMERGENCY_STOP: snprintf(mode_buf, sizeof(mode_buf), "%-10s", "E-STOP"); mode_color = UI_Color_Purplish_red; break;
            default: snprintf(mode_buf, sizeof(mode_buf), "%-10s", "UNKNOWN"); mode_color = UI_Color_White; break;
        }
        UICharDraw(&UI_engineer_mode_text[1], "rs1", UI_Graph_Change, 7, mode_color, 16, 2, 350, 860, mode_buf);
        UICharRefresh(&referee_recv_info->referee_id, UI_engineer_mode_text[1]);
        interactive_data->Referee_Interactive_Flag.robot_mode_flag = 0;
    }

    if (interactive_data->Referee_Interactive_Flag.grab_control_flag == 1) {
        char ctrl_buf[20];
        uint32_t ctrl_color;
        switch (interactive_data->grab_control_mode) {
            case GRAB_CONTROL_KEYBOARD: snprintf(ctrl_buf, sizeof(ctrl_buf), "%-10s", "KEYBOARD"); ctrl_color = UI_Color_Green; break;
            case GRAB_CONTROL_CUSTOM: snprintf(ctrl_buf, sizeof(ctrl_buf), "%-10s", "CUSTOM"); ctrl_color = UI_Color_Cyan; break;
            case GRAB_CONTROL_HALF_AUTO: snprintf(ctrl_buf, sizeof(ctrl_buf), "%-10s", "HALF-AUTO"); ctrl_color = UI_Color_Yellow; break;
            default: snprintf(ctrl_buf, sizeof(ctrl_buf), "%-10s", "UNKNOWN"); ctrl_color = UI_Color_White; break;
        }
        UICharDraw(&UI_grab_control_text[1], "cs1", UI_Graph_Change, 7, ctrl_color, 16, 2, 350, 820, ctrl_buf);
        UICharRefresh(&referee_recv_info->referee_id, UI_grab_control_text[1]);
        interactive_data->Referee_Interactive_Flag.grab_control_flag = 0;
    }

    if (interactive_data->Referee_Interactive_Flag.gripper_flag == 1) {
        char gripper_buf[20];
        uint32_t gripper_color;
        if (interactive_data->gripper_opened) {
            snprintf(gripper_buf, sizeof(gripper_buf), "%-10s", "OPEN");
            gripper_color = UI_Color_Green;
        } else {
            snprintf(gripper_buf, sizeof(gripper_buf), "%-10s", "CLOSED");
            gripper_color = UI_Color_Pink;
        }
        UICharDraw(&UI_gripper_status_text[1], "gs1", UI_Graph_Change, 7, gripper_color, 16, 2, 350, 780, gripper_buf);
        UICharRefresh(&referee_recv_info->referee_id, UI_gripper_status_text[1]);
        interactive_data->Referee_Interactive_Flag.gripper_flag = 0;
    }

    int chs_bar_height = (int)(interactive_data->lift_ratio * 140.0f);
    if (chs_bar_height < 0) chs_bar_height = 0;
    if (chs_bar_height > 140) chs_bar_height = 140;

    int chs_top_y = 610 - chs_bar_height;
    if (chs_top_y < 470) chs_top_y = 470;

    UIRectangleDraw(&UI_lift_bar_chassis_fg, "lbf", UI_Graph_Change, 7, UI_Color_Cyan,
                   6, 1705, 610, 1730, chs_top_y);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_chassis_fg);

    float arm_ratio = 0.0f;
    if (robotdata != NULL && robotdata->grab != NULL) {
        arm_ratio = interactive_data->arm_lift / 280.0f;
    }
    if (arm_ratio < 0.0f) arm_ratio = 0.0f;
    if (arm_ratio > 1.0f) arm_ratio = 1.0f;

    uint16_t arm_bar_height = (uint16_t)(arm_ratio * 140.0f);
    uint16_t arm_top_y = 610 - arm_bar_height;

    UIRectangleDraw(&UI_lift_bar_arm_fg, "laf", UI_Graph_Change, 7, UI_Color_Yellow,
                    6, 1645, 610, 1670, arm_top_y);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_arm_fg);

    for (int i = 0; i < 5; i++) {
        char angle_buf[16];
        char graph_name[4];
        snprintf(graph_name, sizeof(graph_name), "mv%d", i);
        fmt_1dp(angle_buf, sizeof(angle_buf), interactive_data->motor_angles[i]);
        UICharDraw(&UI_motor_angle_value[i], graph_name, UI_Graph_Change, 7, UI_Color_Green, 16, 2,
                  1200 + i * 120, 830, angle_buf);
        UICharRefresh(&referee_recv_info->referee_id, UI_motor_angle_value[i]);
    }

    for (int i = 0; i < 5; i++) {
        char angle_buf[16];
        char graph_name[4];
        snprintf(graph_name, sizeof(graph_name), "av%d", i);
        fmt_1dp(angle_buf, sizeof(angle_buf), interactive_data->arm_angles[i]);
        UICharDraw(&UI_arm_angle_value[i], graph_name, UI_Graph_Change, 7, UI_Color_Cyan, 16, 2,
                  1200 + i * 120, 800, angle_buf);
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
    if (referee_recv_info == NULL || referee_recv_info->GameRobotState.robot_id == 0) {
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

    UICharDraw(&UI_cali_state_text[0], "as0", UI_Graph_ADD, 7, UI_Color_White, 16, 2, 80, 740, "Arm State:");
    UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[0]);
    UICharDraw(&UI_cali_state_text[1], "ac1", UI_Graph_ADD, 7, UI_Color_Purplish_red, 16, 2, 350, 740, "Not Calib");
    UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[1]);

    UICharDraw(&UI_cali_state_text[2], "ls0", UI_Graph_ADD, 7, UI_Color_White, 16, 2, 80, 700, "Lift State:");
    UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[2]);
    UICharDraw(&UI_cali_state_text[3], "lc1", UI_Graph_ADD, 7, UI_Color_Purplish_red, 16, 2, 350, 700, "Not Calib");
    UICharRefresh(&referee_recv_info->referee_id, UI_cali_state_text[3]);

    const char* motor_names[] = {"Base", "E_Roll", "E_Pitch", "W_Pitch", "W_Roll"};
    for (int i = 0; i < 5; i++) {
        char graph_name[4];
        snprintf(graph_name, sizeof(graph_name), "mn%d", i);
        UICharDraw(&UI_motor_angle_name[i], graph_name, UI_Graph_ADD, 7, UI_Color_Yellow, 14, 2,
                 1200 + i * 120, 860, (char *)motor_names[i]);
        UICharRefresh(&referee_recv_info->referee_id, UI_motor_angle_name[i]);

        char angle_graph_name[4];
        snprintf(angle_graph_name, sizeof(angle_graph_name), "mv%d", i);
        UICharDraw(&UI_motor_angle_value[i], angle_graph_name, UI_Graph_ADD, 7, UI_Color_Green, 16, 2,
                 1200 + i * 120, 830, "0.0");
        UICharRefresh(&referee_recv_info->referee_id, UI_motor_angle_value[i]);
    }

    for (int i = 0; i < 5; i++) {
        char angle_graph_name[4];
        snprintf(angle_graph_name, sizeof(angle_graph_name), "av%d", i);
        UICharDraw(&UI_arm_angle_value[i], angle_graph_name, UI_Graph_ADD, 7, UI_Color_Cyan, 16, 2,
                 1200 + i * 120, 800, "0.0");
        UICharRefresh(&referee_recv_info->referee_id, UI_arm_angle_value[i]);
    }

    UIRectangleDraw(&UI_lift_bar_arm_bg, "lab", UI_Graph_ADD, 7, UI_Color_Yellow,
                   2, 1645, 470, 1670, 610);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_arm_bg);

    UIRectangleDraw(&UI_lift_bar_arm_fg, "laf", UI_Graph_ADD, 7, UI_Color_Yellow,
                    6, 1645, 610, 1670, 610);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_arm_fg);

    UIRectangleDraw(&UI_lift_bar_chassis_bg, "lbb", UI_Graph_ADD, 7, UI_Color_Cyan,
                   2, 1705, 470, 1730, 610);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_chassis_bg);

    UIRectangleDraw(&UI_lift_bar_chassis_fg, "lbf", UI_Graph_ADD, 7, UI_Color_Cyan,
                    6, 1705, 610, 1730, 610);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_lift_bar_chassis_fg);

    UICharDraw(&UI_lift_bar_arm_text, "lat", UI_Graph_ADD, 7, UI_Color_Yellow, 14, 2,
              1638, 625, "ARM");
    UICharRefresh(&referee_recv_info->referee_id, UI_lift_bar_arm_text);

    UICharDraw(&UI_lift_bar_chassis_text, "lct", UI_Graph_ADD, 7, UI_Color_Cyan, 14, 2,
              1698, 625, "CHS");
    UICharRefresh(&referee_recv_info->referee_id, UI_lift_bar_chassis_text);

    // UILineDraw(&UI_vehicle_width_line_left, "vwl", UI_Graph_ADD, 7, UI_Color_Yellow,
    //            3, 820, 500, 560, 180);
    // UILineDraw(&UI_vehicle_width_line_right, "vwr", UI_Graph_ADD, 7, UI_Color_Yellow,
    //            3, 1100, 500, 1360, 180);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_vehicle_width_line_left, UI_vehicle_width_line_right);
#endif // ROBOT_ENGINEERING
}

void UITask()
{
#ifdef ROBOT_ENGINEERING
    if (robotdata == NULL) {
        robotdata = RobotGet();
        if (robotdata == NULL) {
            osDelay(100); return;
        }
    }
    if (robotdata->chassis == NULL) {
        osDelay(100); return;
    }

    if (robotdata->ui_reset_flag == 1) {
        if (referee_recv_info == NULL) referee_recv_info = GetRefereeInfo();
        if (referee_recv_info != NULL && referee_recv_info->GameRobotState.robot_id != 0) {
            MyUIInit();
            interactive_data.Referee_Interactive_Flag.arm_cali_flag = 1;
            interactive_data.Referee_Interactive_Flag.lift_cali_flag = 1;
            interactive_data.Referee_Interactive_Flag.robot_mode_flag = 1;
            interactive_data.Referee_Interactive_Flag.grab_control_flag = 1;
            interactive_data.Referee_Interactive_Flag.gripper_flag = 1;
            robotdata->ui_reset_flag = 0;
        }
    }

    if (robotdata->grab != NULL && robotdata->grab->actuator != NULL) {
        if (robotdata->grab->actuator->wrist_cali_obj.state == CALI_DONE)
            interactive_data.arm_cali_state = 2;
        else if (robotdata->grab->actuator->wrist_cali_obj.state == CALI_ERROR)
            interactive_data.arm_cali_state = 0;
        else
            interactive_data.arm_cali_state = 1;
    } else {
        interactive_data.arm_cali_state = 0;
    }

    if (robotdata->chassis != NULL) {
        uint8_t all_done = robotdata->chassis->cali_state.all_cali_done;
        uint8_t max_cali = robotdata->chassis->cali_state.is_max_calibrated;
        if (all_done && max_cali) interactive_data.lift_cali_state = 2;
        else if (!all_done && !max_cali) interactive_data.lift_cali_state = 0;
        else interactive_data.lift_cali_state = 1;
    } else {
        interactive_data.lift_cali_state = 0;
    }

    if (robotdata->grab != NULL) {
        if (robotdata->grab->grab_measure.torque > 0.5f) interactive_data.gripper_opened = 0;
        else interactive_data.gripper_opened = 1;
    } else {
        interactive_data.gripper_opened = 0;
    }

    if (robotdata->self_control != NULL) {
        interactive_data.motor_angles[0] = robotdata->self_control->unpacked_data.motors[3].angle;
        interactive_data.motor_angles[1] = robotdata->self_control->unpacked_data.motors[0].angle;
        interactive_data.motor_angles[2] = robotdata->self_control->unpacked_data.motors[1].angle;
        interactive_data.motor_angles[3] = robotdata->self_control->unpacked_data.motors[2].angle;
        interactive_data.motor_angles[4] = robotdata->self_control->unpacked_data.motors[4].angle;
    } else {
        for (int i = 0; i < 5; i++) interactive_data.motor_angles[i] = 0.0f;
    }

    if (robotdata->grab != NULL && robotdata->grab->arm != NULL && robotdata->grab->actuator != NULL) {
        interactive_data.arm_angles[0] = robotdata->grab->arm->base_joint;
        interactive_data.arm_angles[1] = robotdata->grab->arm->elbow_roll;
        interactive_data.arm_angles[2] = robotdata->grab->arm->elbow_pitch;
        interactive_data.arm_angles[3] = robotdata->grab->actuator->wrist_pitch;
        interactive_data.arm_angles[4] = robotdata->grab->actuator->wrist_roll;
    } else {
        for (int i = 0; i < 5; i++) interactive_data.arm_angles[i] = 0.0f;
    }

    if (robotdata->chassis != NULL) interactive_data.lift_ratio = robotdata->chassis->chassis_ctrl_cmd.lift_ratio;
    else interactive_data.lift_ratio = 0.0f;

    if (robotdata->grab != NULL) interactive_data.arm_lift = robotdata->grab->grab_measure.arm_lift;
    else interactive_data.arm_lift = 0.0f;

    interactive_data.robot_mode = robotdata->robot_mode;
    interactive_data.grab_control_mode = GetGrabControlMode();

    UIChangeCheck(&interactive_data);
    MyUIRefresh(&interactive_data);

#else
    // 如果当前编译的不是工程机器人，这里执行占位逻辑，防止报错
    if (referee_recv_info == NULL) {
        referee_recv_info = GetRefereeInfo();
    }
    osDelay(100); // 维持任务的心跳
#endif
}