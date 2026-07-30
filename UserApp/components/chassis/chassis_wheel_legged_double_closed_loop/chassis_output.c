/**
 ******************************************************************************
 * @file    chassis_output.c
 * @brief   双闭环轮腿关节电机的唯一输出仲裁
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "chassis_private.h"

/* Private define ------------------------------------------------------------*/

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/
static void WheelLeggedSetLegJointMotorState(WheelLeggedLegInstance_t *leg, uint8_t request_enable);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 根据底盘模式在状态变化时切换关节电机的 Stop 或 Enable。
 *
 * 同一状态下不重复下发 Stop 或 Enable，避免 1 ms 周期任务反复抢写电机状态。
 * 后续位置、力矩、电流等控制参考也只能由本输出层下发。
 *
 * @param chassis 底盘对象。
 */
void WheelLeggedChassisApplyJointMotorState(WheelLeggedChassisInstance_t *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

    const uint8_t request_enable = chassis->chassis_ctrl_cmd.chassis_mode != CHASSIS_POWER_OFF;
    if (request_enable == chassis->joint_motor_enabled)
    {
        return;
    }

    WheelLeggedSetLegJointMotorState(&chassis->left_leg, request_enable);
    WheelLeggedSetLegJointMotorState(&chassis->right_leg, request_enable);
    chassis->joint_motor_enabled = request_enable;
}

/**
 * @brief 对一条腿的两个主动关节统一下发 Stop 或 Enable。
 *
 * @param leg 待操作的腿对象。
 * @param request_enable 非零时使能，零时置为零力矩。
 */
static void WheelLeggedSetLegJointMotorState(WheelLeggedLegInstance_t *leg, uint8_t request_enable)
{
    if (leg == NULL)
    {
        return;
    }

    if (request_enable != 0u)
    {
        if (leg->front_joint.motor != NULL)
        {
            DMMotorEnable(leg->front_joint.motor);
        }
        if (leg->rear_joint.motor != NULL)
        {
            DMMotorEnable(leg->rear_joint.motor);
        }
    }
    else
    {
        if (leg->front_joint.motor != NULL)
        {
            DMMotorStop(leg->front_joint.motor);
        }
        if (leg->rear_joint.motor != NULL)
        {
            DMMotorStop(leg->rear_joint.motor);
        }
    }
}
