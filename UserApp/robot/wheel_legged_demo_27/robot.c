/**
 ******************************************************************************
 * @file    robot.c
 * @brief   双闭环轮腿机器人板级对象初始化与零力矩观测任务
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "robot.h"

#include <string.h>

#include "robot_config.h"
#include "user_lib.h"

/* Private define ------------------------------------------------------------*/
#define WHEEL_LEGGED_IK_TEST_CONTROL_PERIOD_S (0.001f) /* 机器人任务周期，单位 s。 */

/* Intermediate variables calculated by private functions -------------------*/
/* 唯一的顶层机器人对象入口；J-Link 中从 robot 向下展开全部运行状态。 */
static RobotInstance *robot;

/* Private function prototypes -----------------------------------------------*/
static uint8_t IsValidLegKinematicsInput(LegKinematicsInput_e input);
static JointTransmissionConfig_t WheelLeggedBuildJointTransmissionConfig(
    const WheelLeggedChainTransmissionConfig_t *chain_config);
static void WheelLeggedLegInit(WheelLeggedLegInstance_t *leg, WheelLeggedLegInitConfig_t *config);
static float WheelLeggedJointStopAndGetAngle(WheelLeggedJointInstance_t *joint);
static void WheelLeggedLegUpdate(WheelLeggedLegInstance_t *leg);
static void WheelLeggedLegUpdateInverseKinematicsTest(WheelLeggedLegInstance_t *leg);
static float WheelLeggedLimitReferenceStep(float current, float target, float maximum_step);
static void WheelLeggedJointSetPIDReference(WheelLeggedJointInstance_t *joint, float reference,
                                            float maximum_speed, float maximum_torque);
static void WheelLeggedChassisInit(WheelLeggedChassisInstance_t *chassis);
void RobotInit(void);
void RobotTask(void);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 初始化顶层机器人对象及其底盘子对象。
 *
 * 当前仅完成左右腿达妙电机的停止状态观测与机构学观测初始化；
 * 云台、遥控器、IMU 和轮毂电机驱动等待对应硬件配置确认后再初始化。
 */
void RobotInit(void)
{
    robot = (RobotInstance *)zmalloc(sizeof(*robot));
    if (robot == NULL)
    {
        return;
    }

    robot->robot_mode = ROBOT_MODE_POWER_OFF;
    robot->chassis = (WheelLeggedChassisInstance_t *)zmalloc(sizeof(*robot->chassis));
    if (robot->chassis == NULL)
    {
        return;
    }

    /* 当前工程只做底盘 FK 观测，不初始化云台、遥控器和 IMU 对象。 */
    WheelLeggedChassisInit(robot->chassis);
}

/**
 * @brief 初始化底盘内的左腿、右腿、左轮毂和右轮毂对象。
 *
 * @param chassis 待初始化的底盘对象。
 */
static void WheelLeggedChassisInit(WheelLeggedChassisInstance_t *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

    memset(chassis, 0, sizeof(*chassis));
    WheelLeggedLegInit(&chassis->left_leg, &g_left_leg_init_config);
    WheelLeggedLegInit(&chassis->right_leg, &g_right_leg_init_config);
    chassis->left_wheel.can_config = g_left_wheel_can_config;
    chassis->right_wheel.can_config = g_right_wheel_can_config;
}

/**
 * @brief 初始化一条腿的前后主动关节和纯机构学实例。
 *
 * @param leg 待初始化的腿对象。
 * @param config 该腿的关节电机与机构学配置。
 */
static void WheelLeggedLegInit(WheelLeggedLegInstance_t *leg, WheelLeggedLegInitConfig_t *config)
{
    if (leg == NULL || config == NULL)
    {
        return;
    }

    memset(leg, 0, sizeof(*leg));
    if (config->geometry_config != NULL)
    {
        leg->kinematics_runtime_config.geometry = *config->geometry_config;
    }
    leg->front_joint_kinematics_input = config->front_joint_kinematics_input;
    leg->rear_joint_kinematics_input = config->rear_joint_kinematics_input;
    if (IsValidLegKinematicsInput(leg->front_joint_kinematics_input) &&
        IsValidLegKinematicsInput(leg->rear_joint_kinematics_input) &&
        leg->front_joint_kinematics_input != leg->rear_joint_kinematics_input)
    {
        leg->kinematics_runtime_config.transmission[leg->front_joint_kinematics_input] =
            WheelLeggedBuildJointTransmissionConfig(config->front_joint_chain_config);
        leg->kinematics_runtime_config.transmission[leg->rear_joint_kinematics_input] =
            WheelLeggedBuildJointTransmissionConfig(config->rear_joint_chain_config);
    }
    leg->front_joint.motor = DMMotorInit(&config->front_joint_motor_config);
    leg->rear_joint.motor = DMMotorInit(&config->rear_joint_motor_config);
    DoubleClosedLoopLegInit(&leg->kinematics, &leg->kinematics_runtime_config);
    leg->inverse_kinematics_test.status = DOUBLE_CLOSED_LOOP_LEG_INVALID_ARGUMENT;
    leg->inverse_kinematics_test.maximum_motor_reference_error = 0.25f;
    leg->inverse_kinematics_test.maximum_motor_reference_speed = 0.40f;
    leg->inverse_kinematics_test.maximum_motor_torque = 0.40f;
    leg->inverse_kinematics_test.transmission_status[LEG_KINEMATICS_INPUT_PHI1] = JOINT_TRANSMISSION_INVALID_ARGUMENT;
    leg->inverse_kinematics_test.transmission_status[LEG_KINEMATICS_INPUT_PHI2] = JOINT_TRANSMISSION_INVALID_ARGUMENT;
}

/**
 * @brief 判断主动关节反馈对应的机构学输入槽位是否有效。
 *
 * @param input 待判断的 phi1 或 phi2 槽位。
 * @return 有效返回 1，无效返回 0。
 */
static uint8_t IsValidLegKinematicsInput(LegKinematicsInput_e input)
{
    return input < LEG_KINEMATICS_INPUT_COUNT;
}

/**
 * @brief 根据链轮齿数生成底层传动换算所需的角度比例配置。
 *
 * 用户只需填写“机械主动轴 phi=0 时的电机累计角”。函数内部据此计算传动模块
 * 所需的零位偏置，使主动轴角满足 phi=direction*gain*(motor_angle-motor_zero_angle)。
 * 链轮齿数为零时保持未配置状态，避免除零或误用未标定参数。
 *
 * @param chain_config 人工填写的链轮传动标定参数。
 * @return 供纯传动模块使用的电机角到主动轴角配置。
 */
static JointTransmissionConfig_t WheelLeggedBuildJointTransmissionConfig(
    const WheelLeggedChainTransmissionConfig_t *chain_config)
{
    JointTransmissionConfig_t transmission_config = {0};

    if (chain_config == NULL || chain_config->driving_sprocket_teeth == 0u || chain_config->driven_sprocket_teeth == 0u)
    {
        return transmission_config;
    }

    transmission_config.configured = chain_config->configured != 0u;
    transmission_config.gain = (float)chain_config->driving_sprocket_teeth / (float)chain_config->driven_sprocket_teeth;
    transmission_config.direction = chain_config->direction;
    transmission_config.zero_offset =
        -transmission_config.direction * transmission_config.gain * chain_config->motor_zero_angle;
    return transmission_config;
}

/**
 * @brief 执行一次机器人周期任务。
 *
 * 当前直接按左腿、右腿的具名对象更新，保持主动关节停止，并刷新机构学观测状态。
 */
void RobotTask(void)
{
    if (robot == NULL || robot->chassis == NULL)
    {
        return;
    }

    WheelLeggedLegUpdate(&robot->chassis->left_leg);
    WheelLeggedLegUpdate(&robot->chassis->right_leg);
}

/**
 * @brief 更新一条腿的电机反馈与正运动学观测状态。
 *
 * 该函数先停止两个主动关节，再把两个累计电机角交给传动与双闭环机构学模块。
 *
 * @param leg 待更新的腿对象。
 */
static void WheelLeggedLegUpdate(WheelLeggedLegInstance_t *leg)
{
    if (leg == NULL)
    {
        return;
    }

    leg->sequence++;
    const float front_actuator_angle = WheelLeggedJointStopAndGetAngle(&leg->front_joint);
    const float rear_actuator_angle = WheelLeggedJointStopAndGetAngle(&leg->rear_joint);
    float actuator_angle_by_kinematics_input[LEG_KINEMATICS_INPUT_COUNT] = {0.0f};
    if (IsValidLegKinematicsInput(leg->front_joint_kinematics_input) &&
        IsValidLegKinematicsInput(leg->rear_joint_kinematics_input) &&
        leg->front_joint_kinematics_input != leg->rear_joint_kinematics_input)
    {
        actuator_angle_by_kinematics_input[leg->front_joint_kinematics_input] = front_actuator_angle;
        actuator_angle_by_kinematics_input[leg->rear_joint_kinematics_input] = rear_actuator_angle;
    }
    DoubleClosedLoopLegUpdate(&leg->kinematics, actuator_angle_by_kinematics_input[LEG_KINEMATICS_INPUT_PHI1],
                              actuator_angle_by_kinematics_input[LEG_KINEMATICS_INPUT_PHI2]);
    WheelLeggedLegUpdateInverseKinematicsTest(leg);
    leg->update_count++;
    leg->sequence++;
}

/**
 * @brief 刷新一条腿的逆运动学测试结果，并在显式使能后执行受限位置控制。
 *
 * calculation_enabled 为 0 时，将当前 FK 足端位置拷为调试目标，便于在 J-Link
 * 中先观察，再将其置 1 后只修改 target_p.x、target_p.y 验证 IK 输出。
 *
 * @param leg 待更新的腿对象。
 */
static void WheelLeggedLegUpdateInverseKinematicsTest(WheelLeggedLegInstance_t *leg)
{
    WheelLeggedLegInverseKinematicsTest_t *test;
    DoubleClosedLoopLegInput_t previous_input;
    float maximum_reference_step;

    if (leg == NULL)
    {
        return;
    }
    test = &leg->inverse_kinematics_test;
    maximum_reference_step = WHEEL_LEGGED_IK_TEST_CONTROL_PERIOD_S * test->maximum_motor_reference_speed;
    if (leg->kinematics.forward_kinematics_status != DOUBLE_CLOSED_LOOP_LEG_OK)
    {
        test->status = leg->kinematics.forward_kinematics_status;
        test->command_initialized = 0u;
        test->output_active = 0u;
        return;
    }
    if (test->calculation_enabled == 0u)
    {
        test->target_p = leg->kinematics.state.p;
        test->target_ready = 1u;
        test->status = DOUBLE_CLOSED_LOOP_LEG_INVALID_ARGUMENT;
        test->command_initialized = 0u;
        test->output_active = 0u;
        return;
    }

    previous_input.phi1 = leg->kinematics.phi[LEG_KINEMATICS_INPUT_PHI1];
    previous_input.phi2 = leg->kinematics.phi[LEG_KINEMATICS_INPUT_PHI2];
    test->status = DoubleClosedLoopLegInverseKinematics(&leg->kinematics_runtime_config.geometry, &test->target_p,
                                                        &previous_input, &test->solution);
    if (test->status != DOUBLE_CLOSED_LOOP_LEG_OK)
    {
        test->command_initialized = 0u;
        test->output_active = 0u;
        return;
    }
    test->transmission_status[LEG_KINEMATICS_INPUT_PHI1] =
        JointTransmissionJointToMotor(&leg->kinematics_runtime_config.transmission[LEG_KINEMATICS_INPUT_PHI1],
                                      test->solution.phi1, &test->actuator_angle_reference[LEG_KINEMATICS_INPUT_PHI1]);
    test->transmission_status[LEG_KINEMATICS_INPUT_PHI2] =
        JointTransmissionJointToMotor(&leg->kinematics_runtime_config.transmission[LEG_KINEMATICS_INPUT_PHI2],
                                      test->solution.phi2, &test->actuator_angle_reference[LEG_KINEMATICS_INPUT_PHI2]);
    if (test->motor_output_enabled == 0u || leg->front_joint.feedback_ready == 0u || leg->rear_joint.feedback_ready == 0u ||
        test->transmission_status[LEG_KINEMATICS_INPUT_PHI1] != JOINT_TRANSMISSION_OK ||
        test->transmission_status[LEG_KINEMATICS_INPUT_PHI2] != JOINT_TRANSMISSION_OK ||
        test->maximum_motor_reference_error <= 0.0f || test->maximum_motor_reference_speed <= 0.0f ||
        test->maximum_motor_torque <= 0.0f ||
        fabsf(test->actuator_angle_reference[LEG_KINEMATICS_INPUT_PHI1] -
              leg->kinematics.actuator_angle[LEG_KINEMATICS_INPUT_PHI1]) > test->maximum_motor_reference_error ||
        fabsf(test->actuator_angle_reference[LEG_KINEMATICS_INPUT_PHI2] -
              leg->kinematics.actuator_angle[LEG_KINEMATICS_INPUT_PHI2]) > test->maximum_motor_reference_error)
    {
        test->command_initialized = 0u;
        test->output_active = 0u;
        return;
    }
    if (test->command_initialized == 0u)
    {
        test->actuator_angle_command[LEG_KINEMATICS_INPUT_PHI1] =
            leg->kinematics.actuator_angle[LEG_KINEMATICS_INPUT_PHI1];
        test->actuator_angle_command[LEG_KINEMATICS_INPUT_PHI2] =
            leg->kinematics.actuator_angle[LEG_KINEMATICS_INPUT_PHI2];
        test->command_initialized = 1u;
    }
    test->actuator_angle_command[LEG_KINEMATICS_INPUT_PHI1] = WheelLeggedLimitReferenceStep(
        test->actuator_angle_command[LEG_KINEMATICS_INPUT_PHI1],
        test->actuator_angle_reference[LEG_KINEMATICS_INPUT_PHI1], maximum_reference_step);
    test->actuator_angle_command[LEG_KINEMATICS_INPUT_PHI2] = WheelLeggedLimitReferenceStep(
        test->actuator_angle_command[LEG_KINEMATICS_INPUT_PHI2],
        test->actuator_angle_reference[LEG_KINEMATICS_INPUT_PHI2], maximum_reference_step);
    WheelLeggedJointSetPIDReference(&leg->rear_joint, test->actuator_angle_command[LEG_KINEMATICS_INPUT_PHI1],
                                    test->maximum_motor_reference_speed, test->maximum_motor_torque);
    WheelLeggedJointSetPIDReference(&leg->front_joint, test->actuator_angle_command[LEG_KINEMATICS_INPUT_PHI2],
                                    test->maximum_motor_reference_speed, test->maximum_motor_torque);
    test->output_active = 1u;
}

/**
 * @brief 以最大单周期变化量将当前参考平滑靠近目标参考。
 *
 * @param current 当前已经下发的电机角参考，单位 rad。
 * @param target 新的电机角目标，单位 rad。
 * @param maximum_step 单周期最大变化量，单位 rad。
 * @return 经过速度限制后的电机角参考，单位 rad。
 */
static float WheelLeggedLimitReferenceStep(float current, float target, float maximum_step)
{
    if (target > current + maximum_step)
    {
        return current + maximum_step;
    }
    if (target < current - maximum_step)
    {
        return current - maximum_step;
    }
    return target;
}

/**
 * @brief 以测试专用的速度与力矩限制向一个主动关节下发电机侧角度 PID 参考。
 *
 * @param joint 主动关节对象。
 * @param reference 电机累计角参考，单位 rad。
 * @param maximum_speed 角度环输出的最大速度，单位 rad/s。
 * @param maximum_torque 速度环输出的最大力矩，单位 N·m。
 */
static void WheelLeggedJointSetPIDReference(WheelLeggedJointInstance_t *joint, float reference,
                                            float maximum_speed, float maximum_torque)
{
    if (joint == NULL || joint->motor == NULL)
    {
        return;
    }
    joint->motor->motor_controller.angle_PID.MaxOut = maximum_speed;
    joint->motor->motor_controller.speed_PID.MaxOut = maximum_torque;
    DMMotorEnable(joint->motor);
    DMMotorSetPIDRef(joint->motor, reference);
}

/**
 * @brief 停止主动关节输出，并读取累计转子角。
 *
 * DMMotorStop() 会令达妙驱动任务固定发送零力矩命令，防止后续代码误写的
 * final_output 被实际下发；电机反馈帧仍可用于正运动学观测。
 *
 * @param joint 主动关节对象。
 * @return 电机累计转子角；关节或电机对象无效时返回 0。
 */
static float WheelLeggedJointStopAndGetAngle(WheelLeggedJointInstance_t *joint)
{
    if (joint == NULL || joint->motor == NULL)
    {
        return 0.0f;
    }

    DMMotorStop(joint->motor);
    joint->feedback_ready = joint->motor->measure.state != 0u;
    return joint->motor->measure.total_angle;
}
