/**
 ******************************************************************************
 * @file    robot.c
 * @brief   双闭环轮腿机器人顶层对象初始化与任务编排
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "robot.h"

#include "robot_config.h"
#include "user_lib.h"

/* Private define ------------------------------------------------------------*/

/* Intermediate variables calculated by private functions -------------------*/
/* 唯一的顶层机器人对象入口；J-Link 中从 robot 向下展开全部运行状态。 */
static RobotInstance *robot;

/* Private function prototypes -----------------------------------------------*/
void RobotInit(void);
void RobotTask(void);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 初始化顶层机器人对象、遥控器和底盘 component。
 *
 * 具体的关节 CAN、链传动、机械零位、几何参数全部在 robot_config.h 中定义；
 * 本函数只把该配置传给通用底盘 component，不处理底盘内部细节。
 */
void RobotInit(void)
{
    robot = (RobotInstance *)zmalloc(sizeof(*robot));
    if (robot == NULL)
    {
        return;
    }

    robot->robot_mode = ROBOT_MODE_POWER_OFF;
    robot->rc_data = RemoteControlInit(&huart5); /* H7 板上遥控器使用带反相器的 USART5。 */
    robot->chassis = (WheelLeggedChassisInstance_t *)zmalloc(sizeof(*robot->chassis));
    if (robot->chassis == NULL)
    {
        return;
    }

    WheelLeggedChassisInit(robot->chassis, &g_chassis_init_config);
}

/**
 * @brief 执行一次机器人顶层周期任务。
 *
 * 调度顺序固定为：先由上层命令写入底盘目标，再由底盘完成反馈、FK 和输出仲裁。
 */
void RobotTask(void)
{
    if (robot == NULL || robot->chassis == NULL)
    {
        return;
    }

    RobotCMDTask(robot);
    ChassisTask(robot->chassis);
}
