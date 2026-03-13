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
static String_Data_t UI_engineer_mode_text[3];   // 机器人模式显示 (G 键控制)
static String_Data_t UI_grab_control_text[3];    // 机械臂控制模式显示 (F 键控制)
static String_Data_t UI_gripper_status_text[2];  // 夹爪状态显示
static String_Data_t UI_motor_angle_name[5];     // 5 个控制器电机名称标签
static String_Data_t UI_motor_angle_value[6];    // 5 个控制器电机角度数值 + 1 个标题
static String_Data_t UI_arm_angle_name[5];       // 5 个机械臂关节名称标签
static String_Data_t UI_arm_angle_value[6];      // 5 个机械臂关节角度数值 + 1 个标题

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
    // 为 UI 绘制以及交互数据所用
    Chassis_Mode_e chassis_mode;    // 底盘模式
    
    // === 工程机器人数据 ===
    Robot_Mode_e robot_mode;         // 机器人模式 (G 键控制): 0-断电，1-行车，2-兑换，3-上台阶，4-急停
    GrabControlMode_e grab_control_mode;  // 机械臂控制模式 (F 键控制): 0-键鼠，1-自定义，2-半自动
    uint8_t gripper_opened;          // 夹爪状态：0-关闭，1-打开
    float motor_angles[5];           // 5 个控制器的角度
    float arm_angles[5];             // 5 个机械臂关节角度

    // 上一次的模式，用于 flag 判断
    Chassis_Mode_e chassis_last_mode;
    
    // === 工程机器人上一次状态 ===
    Robot_Mode_e last_robot_mode;           // 上一次机器人模式
    GrabControlMode_e last_grab_control_mode;  // 上一次机械臂控制模式
    uint8_t last_gripper_opened;            // 上一次夹爪状态
    float last_motor_angles[5];             // 上一次控制器电机角度
    float last_arm_angles[5];               // 上一次机械臂关节角度
} Referee_Interactive_info_t;

static Referee_Interactive_info_t interactive_data;

// 检测 UI 变化的函数
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    // 底盘模式变化检测
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }
    
    // === 工程机器人变化检测 ===
    
    // 机器人模式变化检测 (G 键控制)
    if (_Interactive_data->robot_mode != _Interactive_data->last_robot_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.robot_mode_flag = 1;
        _Interactive_data->last_robot_mode = _Interactive_data->robot_mode;
    }
    
    // 机械臂控制模式变化检测 (F 键控制)
    if (_Interactive_data->grab_control_mode != _Interactive_data->last_grab_control_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.grab_control_flag = 1;
        _Interactive_data->last_grab_control_mode = _Interactive_data->grab_control_mode;
    }
    
    // 夹爪状态变化检测
    if (_Interactive_data->gripper_opened != _Interactive_data->last_gripper_opened)
    {
        _Interactive_data->Referee_Interactive_Flag.gripper_flag = 1;
        _Interactive_data->last_gripper_opened = _Interactive_data->gripper_opened;
    }
    
    // 电机角度变化检测 (任意一个电机角度变化超过 1 度)
    for (int i = 0; i < 5; i++)
    {
        if (fabsf(_Interactive_data->motor_angles[i] - _Interactive_data->last_motor_angles[i]) > 1.0f)
        {
            _Interactive_data->Referee_Interactive_Flag.motor_angle_flag = 1;
            for (int j = 0; j < 5; j++)
            {
                _Interactive_data->last_motor_angles[j] = _Interactive_data->motor_angles[j];
            }
            break;
        }
    }
    
    // 机械臂关节角度变化检测 (任意一个关节角度变化超过 1 度)
    for (int i = 0; i < 5; i++)
    {
        if (fabsf(_Interactive_data->arm_angles[i] - _Interactive_data->last_arm_angles[i]) > 1.0f)
        {
            _Interactive_data->Referee_Interactive_Flag.arm_angle_flag = 1;
            for (int j = 0; j < 5; j++)
            {
                _Interactive_data->last_arm_angles[j] = _Interactive_data->arm_angles[j];
            }
            break;
        }
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
            case CHASSIS_CALIBRATING:
         chassis_str = "Calibrating";
                break;
            case CHASSIS_CLIMB_IDLE:
            case CHASSIS_CLIMB_BOTH_EXTEND:
            case CHASSIS_CLIMB_FRONT_RETRACT:
            case CHASSIS_CLIMB_ALL_RETRACT:
         chassis_str = "Climb";
                break;
            default:
         chassis_str = "unknown";
                break;
        }
        UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, chassis_str);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }
      
      // === 工程机器人 UI 刷新 ===
      
      // 1. 机器人模式显示 (G 键控制)
      if (interactive_data->Referee_Interactive_Flag.robot_mode_flag == 1)
      {
          char *mode_str;
          uint32_t mode_color;
          switch (interactive_data->robot_mode)
          {
              case ROBOT_POWER_OFF:
                  mode_str = "POWER OFF";
                  mode_color = UI_Color_Purplish_red;
                  break;
              case ROBOT_POWER_ON:
                  mode_str = "NORMAL";
                  mode_color = UI_Color_Green;
                  break;
              case ROBOT_EXCHANGE_MODE:
                  mode_str = "EXCHANGE";
                  mode_color = UI_Color_Cyan;
                  break;
              case ROBOT_CLIMB_MODE:
                  mode_str = "CLIMB";
                  mode_color = UI_Color_Yellow;
                  break;
              case ROBOT_EMERGENCY_STOP:
                  mode_str = "E-STOP";
                  mode_color = UI_Color_Purplish_red;
                  break;
              default:
                  mode_str = "UNKNOWN";
                  mode_color = UI_Color_White;
                  break;
          }
          UICharDraw(&UI_engineer_mode_text[0], "robotMode", UI_Graph_Change, 7, mode_color, 20, 3, 100, 100, mode_str);
          UICharRefresh(&referee_recv_info->referee_id, UI_engineer_mode_text[0]);
          interactive_data->Referee_Interactive_Flag.robot_mode_flag = 0;
      }
      
      // 2. 机械臂控制模式显示 (F 键控制)
      if (interactive_data->Referee_Interactive_Flag.grab_control_flag == 1)
      {
          char *ctrl_str;
          uint32_t ctrl_color;
          switch (interactive_data->grab_control_mode)
          {
              case GRAB_CONTROL_KEYBOARD:
                  ctrl_str = "KEYBOARD";
                  ctrl_color = UI_Color_Green;
                  break;
              case GRAB_CONTROL_CUSTOM:
                  ctrl_str = "CUSTOM";
                  ctrl_color = UI_Color_Cyan;
                  break;
              case GRAB_CONTROL_HALF_AUTO:
                  ctrl_str = "HALF-AUTO";
                  ctrl_color = UI_Color_Yellow;
                  break;
              default:
                  ctrl_str = "UNKNOWN";
                  ctrl_color = UI_Color_White;
                  break;
          }
          UICharDraw(&UI_grab_control_text[0], "grabCtl", UI_Graph_Change, 7, ctrl_color, 18, 2, 100, 140, ctrl_str);
          UICharRefresh(&referee_recv_info->referee_id, UI_grab_control_text[0]);
          interactive_data->Referee_Interactive_Flag.grab_control_flag = 0;
      }
      
      // 3. 夹爪状态显示
      if (interactive_data->Referee_Interactive_Flag.gripper_flag == 1)
      {
          char *gripper_str;
          uint32_t gripper_color;
          if (interactive_data->gripper_opened)
          {
              gripper_str = "OPEN";
              gripper_color = UI_Color_Green;
          }
          else
          {
              gripper_str = "CLOSED";
              gripper_color = UI_Color_Pink;
          }
          UICharDraw(&UI_gripper_status_text[0], "gripSt", UI_Graph_Change, 7, gripper_color, 18, 2, 100, 180, gripper_str);
          UICharRefresh(&referee_recv_info->referee_id, UI_gripper_status_text[0]);
          interactive_data->Referee_Interactive_Flag.gripper_flag = 0;
      }

      // 4. 5 个电机角度显示（仅文字）
      if (interactive_data->Referee_Interactive_Flag.motor_angle_flag == 1)
      {
      for (int i = 0; i < 5; i++)
        {
            // 绘制角度数值
   char angle_buf[10];
   char graph_name[8];
     snprintf(graph_name, sizeof(graph_name), "motAngle%d", i);  // 生成唯一图形名：motAngle0~motAngle4
            snprintf(angle_buf, sizeof(angle_buf), "%.1f°", interactive_data->motor_angles[i]);
            UICharDraw(&UI_motor_angle_value[i], graph_name, UI_Graph_Change, 7, UI_Color_Green, 16, 2,
                    100 + i * 120, 230, angle_buf);
        }
             
       // 发送文字更新（角度值）
       for (int i = 0; i < 5; i++)
        {
           UICharRefresh(&referee_recv_info->referee_id, UI_motor_angle_value[i]);
        }
              
  interactive_data->Referee_Interactive_Flag.motor_angle_flag = 0;
      }
      
      // 5. 5 个机械臂关节角度显示（仅文字）
      if (interactive_data->Referee_Interactive_Flag.arm_angle_flag == 1)
      {
      for (int i = 0; i < 5; i++)
        {
            // 绘制角度数值
   char angle_buf[10];
   char graph_name[8];
     snprintf(graph_name, sizeof(graph_name), "armVal%d", i);  // 生成唯一图形名：armVal0~armVal4
            snprintf(angle_buf, sizeof(angle_buf), "%.1f°", interactive_data->arm_angles[i]);
            UICharDraw(&UI_arm_angle_value[i], graph_name, UI_Graph_Change, 7, UI_Color_Green, 16, 2,
                    100 + i * 120, 300, angle_buf);
        }
             
       // 发送文字更新（角度值）
       for (int i = 0; i < 5; i++)
        {
           UICharRefresh(&referee_recv_info->referee_id, UI_arm_angle_value[i]);
        }
              
  interactive_data->Referee_Interactive_Flag.arm_angle_flag = 0;
      }
}

// UIPitchGaugeInit 函数已删除，因为 UI_pitch_ticks 和 UI_pitch_labels 未定义
// void UIPitchGaugeInit(uint32_t center_x,uint32_t center_y,uint32_t radius) {...}

void MyUIInit()
{
  robotdata=RobotGet();
  referee_recv_info = GetRefereeInfo();  // 从裁判系统模块获取数据
     // if (!referee_recv_info->init_flag)
     //     vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查
    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // === 工程机器人 UI 初始化 ===
    
    // 1. 机器人模式显示 (G 键控制) - 静态标签
    UICharDraw(&UI_engineer_mode_text[1], "robotStLbl", UI_Graph_ADD, 7, UI_Color_White, 16, 2, 100, 70, "Robot State:");
    UICharRefresh(&referee_recv_info->referee_id, UI_engineer_mode_text[1]);
    // 动态值初始化为 POWER OFF
    UICharDraw(&UI_engineer_mode_text[0], "robotState", UI_Graph_ADD, 7, UI_Color_Purplish_red, 20, 3, 100, 100, "POWER OFF");
    UICharRefresh(&referee_recv_info->referee_id, UI_engineer_mode_text[0]);
    
    // 2. 机械臂控制模式显示 (F 键控制) - 静态标签
    UICharDraw(&UI_grab_control_text[1], "ctrlSrcLbl", UI_Graph_ADD, 7, UI_Color_White, 16, 2, 100, 110, "Control Source:");
    UICharRefresh(&referee_recv_info->referee_id, UI_grab_control_text[1]);
    // 动态值初始化为 KEYBOARD
    UICharDraw(&UI_grab_control_text[0], "ctrlSource", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 100, 140, "KEYBOARD");
    UICharRefresh(&referee_recv_info->referee_id, UI_grab_control_text[0]);
    
    // 3. 夹爪状态显示 - 静态标签
    UICharDraw(&UI_gripper_status_text[1], "gripStLbl", UI_Graph_ADD, 7, UI_Color_White, 16, 2, 100, 150, "Gripper State:");
    UICharRefresh(&referee_recv_info->referee_id, UI_gripper_status_text[1]);
    // 动态值初始化为 CLOSED
    UICharDraw(&UI_gripper_status_text[0], "gripperSt", UI_Graph_ADD, 7, UI_Color_Pink, 18, 2, 100, 180, "CLOSED");
    UICharRefresh(&referee_recv_info->referee_id, UI_gripper_status_text[0]);
    
    // 4. 电机角度显示标题
    UICharDraw(&UI_motor_angle_value[5], "ctrlAnglesTitl", UI_Graph_ADD, 7, UI_Color_Cyan, 18, 3, 100, 190, "Controller Angles:");
    UICharRefresh(&referee_recv_info->referee_id, UI_motor_angle_value[5]);
    
    // 初始化 5 个电机角度数值（仅文字，无进度条）
   const char* motor_names[] = {"Base", "Elbow Roll", "Elbow Pitch", "Wrist Pitch", "Wrist Roll"};
   for (int i = 0; i < 5; i++)
   {
       // 绘制电机名称标签
    char graph_name[12];
     snprintf(graph_name, sizeof(graph_name), "motorName%d", i);  // 生成唯一图形名：motorName0~motorName4
       UICharDraw(&UI_motor_angle_name[i], graph_name, UI_Graph_ADD, 7, UI_Color_Yellow, 14, 2, 
                100 + i * 120, 200, motor_names[i]);
       
       // 初始化电机角度数值为 0°
    char angle_graph_name[12];
     snprintf(angle_graph_name, sizeof(angle_graph_name), "motorAngle%d", i);  // 生成唯一图形名：motorAngle0~motorAngle4
       UICharDraw(&UI_motor_angle_value[i], angle_graph_name, UI_Graph_ADD, 7, UI_Color_Green, 16, 2,
                100 + i * 120, 230, "0.0°");
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
   
   // === 添加第二组：机械臂关节角度显示 ===
   UICharDraw(&UI_arm_angle_value[5], "armJointTitl", UI_Graph_ADD, 7, UI_Color_Cyan, 18, 3, 100, 260, "Arm Joint Angles:");
   UICharRefresh(&referee_recv_info->referee_id, UI_arm_angle_value[5]);
   
   const char* arm_joint_names[] = {"Base", "Elbow Roll", "Elbow Pitch", "Wrist Pitch", "Wrist Roll"};
   for (int i = 0; i < 5; i++)
   {
       // 绘制关节名称标签
    char graph_name[12];
     snprintf(graph_name, sizeof(graph_name), "armJointName%d", i);  // 生成唯一图形名：armJointName0~armJointName4
       UICharDraw(&UI_arm_angle_name[i], graph_name, UI_Graph_ADD, 7, UI_Color_Yellow, 14, 2,
                 100 + i * 120, 270, arm_joint_names[i]);
       
       // 初始化机械臂角度数值为 0°
    char angle_graph_name[12];
     snprintf(angle_graph_name, sizeof(angle_graph_name), "armJointAngle%d", i);  // 生成唯一图形名：armJointAngle0~armJointAngle4
       UICharDraw(&UI_arm_angle_value[i], angle_graph_name, UI_Graph_ADD, 7, UI_Color_Green, 16, 2,
                100 + i * 120, 300, "0.0°");
   }
   
   // 发送文字更新（机械臂关节名称）
   for (int i = 0; i < 5; i++)
   {
       UICharRefresh(&referee_recv_info->referee_id, UI_arm_angle_name[i]);
   }
   
   // 发送文字更新（机械臂角度初始值）
   for (int i = 0; i < 5; i++)
   {
       UICharRefresh(&referee_recv_info->referee_id, UI_arm_angle_value[i]);
   }
}

void UITask()
{
    // 首次运行时初始化指针
    // if (referee_recv_info == NULL) {
    //     referee_recv_info = RefereeInit(&huart6); // 假设使用默认串口
    // }

    // 更新交互数据（模拟从系统其他部分获取数据）
    interactive_data.chassis_mode = robotdata->chassis->chassis_ctrl_cmd.chassis_mode;
    interactive_data.robot_mode = robotdata->robot_mode;  // 机器人模式 (G 键控制)
    interactive_data.grab_control_mode = GetGrabControlMode();  // 机械臂控制模式 (F 键控制)
        
    // 获取夹爪状态 (从自定义控制器)
    if (robotdata->self_control != NULL && robotdata->self_control->unpacked_data.gripper_opened != 0)
    {
        interactive_data.gripper_opened = 1;
    }
    else
    {
        interactive_data.gripper_opened = 0;
    }
        
    // 获取 5 个电机角度 (从自定义控制器)
    if (robotdata->self_control != NULL)
    {
  interactive_data.motor_angles[0] = robotdata->self_control->unpacked_data.motors[3].angle; // Base
  interactive_data.motor_angles[1] = robotdata->self_control->unpacked_data.motors[0].angle; // Elbow Roll
  interactive_data.motor_angles[2] = robotdata->self_control->unpacked_data.motors[1].angle; // Elbow Pitch
  interactive_data.motor_angles[3] = robotdata->self_control->unpacked_data.motors[2].angle; // Wrist Pitch
  interactive_data.motor_angles[4] = robotdata->self_control->unpacked_data.motors[4].angle; // Wrist Roll
    }
    else
    {
       for (int i = 0; i < 5; i++)
        {
     interactive_data.motor_angles[i] = 0.0f;
        }
    }
    
    // 获取 5 个机械臂关节角度 (从机械臂模块)
    if (robotdata->grab != NULL && robotdata->grab->arm != NULL)
    {
     interactive_data.arm_angles[0] = robotdata->grab->arm->base_joint;     // 基座旋转
     interactive_data.arm_angles[1] = robotdata->grab->arm->elbow_roll;     // 肘部旋转
     interactive_data.arm_angles[2] = robotdata->grab->arm->elbow_pitch;    // 肘部俯仰
     interactive_data.arm_angles[3] = robotdata->grab->actuator->wrist_pitch;  // 腕部俯仰 (需要根据实际硬件调整)
     interactive_data.arm_angles[4] = robotdata->grab->actuator->wrist_roll;  // 腕部旋转 (需要根据实际硬件调整)
    }
    else
    {
       for (int i = 0; i < 5; i++)
       {
        interactive_data.arm_angles[i] = 0.0f;
       }
    }

    // 检查是否有变化
    UIChangeCheck(&interactive_data);

    // 执行 UI 刷新
    MyUIRefresh(&interactive_data);
}
