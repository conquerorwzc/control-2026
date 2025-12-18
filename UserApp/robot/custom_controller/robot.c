#include "robot.h"

#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "string.h"
#include "custom_controller.h"
static RobotInstance *robot;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
// static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
// static Gantry_Ctrl_Cmd_s *gantry_ctrl_cmd; // 【新增】龙门架控制命令指针

static RC_ctrl_t *rc_data;
static RC_ctrl_t *rc_data_last;  // 遥控器数据,初始化时返回

/* Intermediate variables calculated by private functions */
static float angle;

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet() {
  // 在这个专用于自定义控制器的机器人类型中，我们可以简化遥控器控制逻辑
  // 或者根据需要添加特定的控制逻辑
  // 更新自定义控制器状态
  if (robot->custom_controller != NULL) {
    CustomControllerUpdate(robot->custom_controller);
  }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler() {
  // 简化的紧急停止逻辑
}

void RobotInit() {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

#ifdef STM32F4
  robot->rc_data = RemoteControlInit(&huart3);  // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#elifdef STM32H7
  robot->rc_data = RemoteControlInit(&huart5);  // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#endif

  rc_data_last = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
  *rc_data_last = *robot->rc_data;  // 记录上一次遥控器的状态

  // 重置舵机索引计数器，确保初始化正确
  SerialServoResetIndex();
  
  // 初始化自定义控制器（包含舵机）
  robot->custom_controller = CustomControllerInit();

  rc_data = robot->rc_data;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
  // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
  RemoteControlSet();
  // MouseKeySet();
  EmergencyHandler();  // 处理模块离线和遥控器急停等紧急情况

}

/**
 * @brief  向总线舵机发送指令 (以字符串形式)
 * @param  cmd: 指向要发送的指令字符串的指针
 * @retval None
 */

void RobotTask() {
#if defined(ONE_BOARD)
  RobotCMDTask();
#endif
}