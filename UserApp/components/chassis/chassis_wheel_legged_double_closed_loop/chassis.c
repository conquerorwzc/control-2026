/**
 ******************************************************************************
 * @file    chassis.c
 * @brief   双闭环轮腿底盘的初始化与周期任务调度
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "chassis.h"

#include <string.h>

#include "chassis_private.h"

/* Private define ------------------------------------------------------------*/

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 初始化底盘对象、左右腿机构学对象和轮毂接线记录。
 *
 * 初始化配置由 robot 层传入，底盘 component 不依赖任何具体机器人目录的配置文件。
 *
 * @param chassis 待初始化的底盘对象。
 * @param init_config 本车的左右腿和轮毂初始化配置。
 */
void WheelLeggedChassisInit(WheelLeggedChassisInstance_t *chassis,
                            WheelLeggedChassisInitConfig_t *init_config)
{
    if (chassis == NULL || init_config == NULL)
    {
        return;
    }

    memset(chassis, 0, sizeof(*chassis));
    chassis->chassis_ctrl_cmd.chassis_mode = CHASSIS_POWER_OFF;
    WheelLeggedLegInit(&chassis->left_leg, init_config->left_leg_init_config);
    WheelLeggedLegInit(&chassis->right_leg, init_config->right_leg_init_config);
    chassis->left_wheel.can_config = init_config->left_wheel_can_config;
    chassis->right_wheel.can_config = init_config->right_wheel_can_config;
}

/**
 * @brief 执行一次底盘周期任务。
 *
 * 固定顺序为：更新关节反馈和正运动学，再根据上层命令进行唯一的关节输出仲裁。
 * 当前轮毂型号尚未确认，本任务不向轮毂发送任何 CAN 控制命令。
 *
 * @param chassis 底盘对象。
 */
void ChassisTask(WheelLeggedChassisInstance_t *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

    WheelLeggedLegUpdate(&chassis->left_leg);
    WheelLeggedLegUpdate(&chassis->right_leg);
    WheelLeggedChassisApplyJointMotorState(chassis);
}
