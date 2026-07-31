/**
 ******************************************************************************
 * @file    chassis_output.c
 * @brief   双闭环轮腿关节电机的唯一输出仲裁
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "chassis_private.h"

#include <math.h>

/* Private define ------------------------------------------------------------*/
/* VMC 悬空测试时单台 J4310 电机轴允许的最大绝对力矩，单位 N·m。 */
#define WHEEL_LEGGED_VMC_TEST_MOTOR_TORQUE_LIMIT 2.0f

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/
static void WheelLeggedSetLegJointMotorState(WheelLeggedLegInstance_t *leg, uint8_t request_enable);
static void WheelLeggedSetLegVmcTorqueReference(WheelLeggedLegInstance_t *leg, uint8_t request_enable);
static float WheelLeggedLimitVmcTestMotorTorque(float torque);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 根据底盘模式切换关节电机 Stop 或 Enable，并从唯一出口写入 VMC 测试力矩。
 *
 * 同一状态下不重复下发 Stop 或 Enable，避免 1 ms 周期任务反复抢写电机状态。
 * 正常使能时默认持续写零力矩；只有本腿 torque_test_enable=1 且 VMC 有效时，
 * 才允许写入经过 ±2 N·m 电机轴限幅后的 VMC 测试参考。
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
    if (request_enable != chassis->joint_motor_enabled)
    {
        WheelLeggedSetLegJointMotorState(&chassis->left_leg, request_enable);
        WheelLeggedSetLegJointMotorState(&chassis->right_leg, request_enable);
        chassis->joint_motor_enabled = request_enable;
    }

    WheelLeggedSetLegVmcTorqueReference(&chassis->left_leg, request_enable);
    WheelLeggedSetLegVmcTorqueReference(&chassis->right_leg, request_enable);
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

/**
 * @brief 将一条腿的 VMC 力矩测试结果写入两个达妙电机，未授权测试时始终写零力矩。
 *
 * 本函数是 VMC 唯一允许调用 DMMotorSetRef 的位置。即使关节已经处于 Enable 状态，
 * 只要 torque_test_enable 为 0、VMC 无效或底盘请求失能，两个电机参考均保持为 0。
 *
 * @param leg 待写入力矩参考的一条腿。
 * @param request_enable 底盘是否请求使能；为 0 时强制清零。
 */
static void WheelLeggedSetLegVmcTorqueReference(WheelLeggedLegInstance_t *leg, uint8_t request_enable)
{
    float front_torque = 0.0f;
    float rear_torque = 0.0f;
    if (leg == NULL)
    {
        return;
    }

    if (request_enable != 0u && leg->vmc.torque_test_enable != 0u && leg->vmc.valid != 0u)
    {
        front_torque = WheelLeggedLimitVmcTestMotorTorque(leg->vmc.front_motor_torque);
        rear_torque = WheelLeggedLimitVmcTestMotorTorque(leg->vmc.rear_motor_torque);
    }

    leg->vmc.front_motor_torque_output = front_torque;
    leg->vmc.rear_motor_torque_output = rear_torque;
    if (leg->front_joint.motor != NULL)
    {
        DMMotorSetRef(leg->front_joint.motor, front_torque);
    }
    if (leg->rear_joint.motor != NULL)
    {
        DMMotorSetRef(leg->rear_joint.motor, rear_torque);
    }
}

/**
 * @brief 将单台电机轴 VMC 测试力矩限制在安全上限内。
 *
 * @param torque 待限幅的电机轴力矩，单位 N·m。
 * @return 有限且位于 ±2 N·m 范围内的电机轴力矩；异常值返回 0。
 */
static float WheelLeggedLimitVmcTestMotorTorque(float torque)
{
    if (!isfinite(torque))
    {
        return 0.0f;
    }
    if (torque > WHEEL_LEGGED_VMC_TEST_MOTOR_TORQUE_LIMIT)
    {
        return WHEEL_LEGGED_VMC_TEST_MOTOR_TORQUE_LIMIT;
    }
    if (torque < -WHEEL_LEGGED_VMC_TEST_MOTOR_TORQUE_LIMIT)
    {
        return -WHEEL_LEGGED_VMC_TEST_MOTOR_TORQUE_LIMIT;
    }
    return torque;
}
