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
static void WheelLeggedWheelInit(WheelLeggedWheelInstance_t *wheel, WheelLeggedWheelInitConfig_t *config);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 初始化底盘对象、左右腿、左右轮毂和底盘 IMU。
 *
 * 初始化配置由 robot 层传入，底盘 component 不依赖任何具体机器人目录的配置文件。
 *
 * @param chassis 待初始化的底盘对象。
 * @param init_config 本车完整的底盘初始化配置。
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
    if (init_config->state_config != NULL)
    {
        chassis->state_config = *init_config->state_config;
    }
    WheelLeggedLegInit(&chassis->left_leg, init_config->left_leg_init_config);
    WheelLeggedLegInit(&chassis->right_leg, init_config->right_leg_init_config);
    WheelLeggedWheelInit(&chassis->left_wheel, init_config->left_wheel_init_config);
    WheelLeggedWheelInit(&chassis->right_wheel, init_config->right_wheel_init_config);
    if (init_config->imu_init_config != NULL)
    {
        chassis->imu = INS_Init(init_config->imu_init_config);
    }
}

/**
 * @brief 初始化一台轮毂电机和轮端运动学参数，并立即保持零力矩状态。
 *
 * 达妙初始化过程会注册 CAN 回调并发送使能模式命令；完成注册后立即 Stop，
 * 后续状态读取仅访问反馈，不在此模块向轮毂写入控制参考。
 *
 * @param wheel 待初始化的轮毂运行对象。
 * @param config 本轮的电机和轮端参数配置。
 */
static void WheelLeggedWheelInit(WheelLeggedWheelInstance_t *wheel, WheelLeggedWheelInitConfig_t *config)
{
    if (wheel == NULL || config == NULL)
    {
        return;
    }

    wheel->wheel_radius = config->wheel_radius;
    wheel->reduction_ratio = config->reduction_ratio;
    wheel->direction = config->direction;
    wheel->configured = config->configured;
    wheel->motor = DMMotorInit(&config->motor_config);
    if (wheel->motor != NULL)
    {
        DMMotorStop(wheel->motor);
    }
}

/**
 * @brief 执行一次底盘周期任务。
 *
 * 固定顺序为：更新关节反馈和正运动学、组装整车十维状态、计算两条腿的 VMC 映射，
 * 最后根据上层命令进行唯一的关节输出仲裁。轮毂仅用于读取反馈，本任务不向轮毂写入控制参考。
 * TODO：轮毂闭环控制接入时，必须新增唯一轮毂输出仲裁入口；不得在状态读取模块或
 * 上层命令模块直接调用 DMMotorSetRef。
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
    WheelLeggedChassisStateUpdate(chassis);
    WheelLeggedLegVmcUpdate(&chassis->left_leg);
    WheelLeggedLegVmcUpdate(&chassis->right_leg);
    WheelLeggedChassisApplyJointMotorState(chassis);
}
