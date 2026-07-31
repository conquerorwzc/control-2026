/**
 ******************************************************************************
 * @file    chassis_state.c
 * @brief   双闭环轮腿底盘十维状态读取与组装
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "chassis_private.h"

#include <math.h>

#include "general_def.h"
#include "hip_odometry.h"

/* Private define ------------------------------------------------------------*/
/* 髋点纵向里程计所需的全部原始来源有效时应具有的有效位掩码。 */
#define WHEEL_LEGGED_STATE_VALID_RAW_SOURCES \
    (WHEEL_LEGGED_STATE_VALID_LEFT_WHEEL | WHEEL_LEGGED_STATE_VALID_RIGHT_WHEEL | \
     WHEEL_LEGGED_STATE_VALID_IMU | WHEEL_LEGGED_STATE_VALID_LEFT_LEG | WHEEL_LEGGED_STATE_VALID_RIGHT_LEG)

/* 十维状态包含髋点纵向里程计后的完整有效位掩码。 */
#define WHEEL_LEGGED_STATE_VALID_ALL \
    (WHEEL_LEGGED_STATE_VALID_RAW_SOURCES | WHEEL_LEGGED_STATE_VALID_HIP_ODOMETRY)

/* Intermediate variables calculated by private functions -------------------*/
typedef struct
{
    float length;             /* 虚拟腿长，单位 m。 */
    float length_dot;         /* 虚拟腿长度变化率，单位 m/s。 */
    float relative_theta;     /* 相对机身的虚拟腿角，单位 rad。 */
    float relative_theta_dot; /* 相对机身的虚拟腿角速度，单位 rad/s。 */
} WheelLeggedLegStateSnapshot_t;

/* Private function prototypes -----------------------------------------------*/
static uint8_t WheelLeggedWheelReadState(WheelLeggedWheelInstance_t *wheel, float *wheel_angle,
                                         float *wheel_speed);
static uint8_t WheelLeggedLegReadState(const WheelLeggedLegInstance_t *leg, float relative_direction,
                                       WheelLeggedLegStateSnapshot_t *state);
static uint8_t WheelLeggedImuReadState(const INS_t *imu, float *yaw, float *yaw_rate, float *pitch,
                                       float *pitch_rate);
static uint8_t WheelLeggedIsDirectionValid(float direction);
static void WheelLeggedChassisStateFillVector(WheelLeggedChassisState_t *state);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 读取底盘传感器和双闭环机构学结果，组装固定顺序的十维状态。
 *
 * 本函数只读取轮毂、IMU 和关节反馈，不调用任何电机输出接口。每周期先在局部变量中
 * 组装完整快照，最后一次性写回 chassis_state，避免无效来源沿用上一帧的数值。
 *
 * @param chassis 待更新状态的底盘对象。
 */
void WheelLeggedChassisStateUpdate(WheelLeggedChassisInstance_t *chassis)
{
    WheelLeggedChassisState_t next_state = {0};
    uint16_t valid_mask = 0u;
    float yaw = 0.0f;
    float yaw_rate = 0.0f;
    float pitch = 0.0f;
    float pitch_rate = 0.0f;
    WheelLeggedLegStateSnapshot_t left_leg_state = {0};
    WheelLeggedLegStateSnapshot_t right_leg_state = {0};
    WheelLeggedHipOdometryLegInput_t left_hip_input = {0};
    WheelLeggedHipOdometryLegInput_t right_hip_input = {0};
    WheelLeggedHipOdometryLegOutput_t left_hip_output = {0};
    WheelLeggedHipOdometryLegOutput_t right_hip_output = {0};

    if (chassis == NULL)
    {
        return;
    }

    next_state.s_origin = chassis->chassis_state.s_origin;
    next_state.yaw_origin = chassis->chassis_state.yaw_origin;
    next_state.pitch_origin = chassis->chassis_state.pitch_origin;
    next_state.origin_captured = chassis->chassis_state.origin_captured;
    next_state.update_count = chassis->chassis_state.update_count + 1u;

    if (WheelLeggedWheelReadState(&chassis->left_wheel, &next_state.left_wheel_angle,
                                  &next_state.left_wheel_speed) != 0u)
    {
        valid_mask |= WHEEL_LEGGED_STATE_VALID_LEFT_WHEEL;
    }
    if (WheelLeggedWheelReadState(&chassis->right_wheel, &next_state.right_wheel_angle,
                                  &next_state.right_wheel_speed) != 0u)
    {
        valid_mask |= WHEEL_LEGGED_STATE_VALID_RIGHT_WHEEL;
    }
    if (WheelLeggedImuReadState(chassis->imu, &yaw, &yaw_rate, &pitch, &pitch_rate) != 0u)
    {
        valid_mask |= WHEEL_LEGGED_STATE_VALID_IMU;
    }
    if (WheelLeggedLegReadState(&chassis->left_leg, chassis->state_config.left_leg_relative_direction,
                                &left_leg_state) != 0u)
    {
        next_state.left_leg_relative_theta = left_leg_state.relative_theta;
        next_state.left_leg_relative_theta_dot = left_leg_state.relative_theta_dot;
        next_state.left_leg_length = left_leg_state.length;
        next_state.left_leg_length_dot = left_leg_state.length_dot;
        valid_mask |= WHEEL_LEGGED_STATE_VALID_LEFT_LEG;
    }
    if (WheelLeggedLegReadState(&chassis->right_leg, chassis->state_config.right_leg_relative_direction,
                                &right_leg_state) != 0u)
    {
        next_state.right_leg_relative_theta = right_leg_state.relative_theta;
        next_state.right_leg_relative_theta_dot = right_leg_state.relative_theta_dot;
        next_state.right_leg_length = right_leg_state.length;
        next_state.right_leg_length_dot = right_leg_state.length_dot;
        valid_mask |= WHEEL_LEGGED_STATE_VALID_RIGHT_LEG;
    }

    if ((valid_mask & WHEEL_LEGGED_STATE_VALID_RAW_SOURCES) == WHEEL_LEGGED_STATE_VALID_RAW_SOURCES)
    {
        /* TODO：完成轮径、轮向和 IMU 前向加速度标定后，在此接入独立的纵向状态估计器，
         * 处理打滑、离地和轮端里程计漂移；估计器输出替换 s/s_dot，不得修改 FK、雅可比或电机输出。 */
        const float body_pitch_absolute = chassis->state_config.body_pitch_direction * pitch;
        const float body_pitch_rate = chassis->state_config.body_pitch_direction * pitch_rate;
        left_hip_input.wheel_radius = chassis->left_wheel.wheel_radius;
        left_hip_input.wheel_angle = next_state.left_wheel_angle;
        left_hip_input.wheel_speed = next_state.left_wheel_speed;
        left_hip_input.leg_length = left_leg_state.length;
        left_hip_input.leg_length_dot = left_leg_state.length_dot;
        left_hip_input.leg_theta = left_leg_state.relative_theta +
                                   chassis->state_config.leg_world_body_pitch_gain * body_pitch_absolute +
                                   chassis->state_config.left_leg_world_offset;
        left_hip_input.leg_theta_dot = left_leg_state.relative_theta_dot +
                                       chassis->state_config.leg_world_body_pitch_gain * body_pitch_rate;
        right_hip_input.wheel_radius = chassis->right_wheel.wheel_radius;
        right_hip_input.wheel_angle = next_state.right_wheel_angle;
        right_hip_input.wheel_speed = next_state.right_wheel_speed;
        right_hip_input.leg_length = right_leg_state.length;
        right_hip_input.leg_length_dot = right_leg_state.length_dot;
        right_hip_input.leg_theta = right_leg_state.relative_theta +
                                    chassis->state_config.leg_world_body_pitch_gain * body_pitch_absolute +
                                    chassis->state_config.right_leg_world_offset;
        right_hip_input.leg_theta_dot = right_leg_state.relative_theta_dot +
                                        chassis->state_config.leg_world_body_pitch_gain * body_pitch_rate;
        if (WheelLeggedHipOdometryCalculate(&left_hip_input, &left_hip_output) != 0u &&
            WheelLeggedHipOdometryCalculate(&right_hip_input, &right_hip_output) != 0u)
        {
            next_state.hip_odometry_s_raw = 0.5f * (left_hip_output.raw_position + right_hip_output.raw_position);
            next_state.left_hip_odometry_s_dot = left_hip_output.velocity;
            next_state.right_hip_odometry_s_dot = right_hip_output.velocity;
            next_state.s_dot = 0.5f * (left_hip_output.velocity + right_hip_output.velocity);
            valid_mask |= WHEEL_LEGGED_STATE_VALID_HIP_ODOMETRY;
            if (next_state.origin_captured == 0u)
            {
                next_state.s_origin = next_state.hip_odometry_s_raw;
                next_state.yaw_origin = yaw;
                next_state.pitch_origin = pitch;
                next_state.origin_captured = 1u;
            }
            next_state.s = next_state.hip_odometry_s_raw - next_state.s_origin;
        }
    }

    if (next_state.origin_captured != 0u && (valid_mask & WHEEL_LEGGED_STATE_VALID_IMU) != 0u)
    {
        next_state.phi = chassis->state_config.yaw_direction * (yaw - next_state.yaw_origin);
        next_state.phi_dot = chassis->state_config.yaw_direction * yaw_rate;
        next_state.theta_body = chassis->state_config.body_pitch_direction * (pitch - next_state.pitch_origin);
        next_state.theta_body_dot = chassis->state_config.body_pitch_direction * pitch_rate;
    }

    if ((valid_mask & (WHEEL_LEGGED_STATE_VALID_IMU | WHEEL_LEGGED_STATE_VALID_LEFT_LEG)) ==
            (WHEEL_LEGGED_STATE_VALID_IMU | WHEEL_LEGGED_STATE_VALID_LEFT_LEG) &&
        next_state.origin_captured != 0u)
    {
        next_state.theta_leg_left = next_state.left_leg_relative_theta +
                                    chassis->state_config.leg_world_body_pitch_gain * next_state.theta_body +
                                    chassis->state_config.left_leg_world_offset;
        next_state.theta_leg_left_dot = next_state.left_leg_relative_theta_dot +
                                        chassis->state_config.leg_world_body_pitch_gain * next_state.theta_body_dot;
    }
    if ((valid_mask & (WHEEL_LEGGED_STATE_VALID_IMU | WHEEL_LEGGED_STATE_VALID_RIGHT_LEG)) ==
            (WHEEL_LEGGED_STATE_VALID_IMU | WHEEL_LEGGED_STATE_VALID_RIGHT_LEG) &&
        next_state.origin_captured != 0u)
    {
        next_state.theta_leg_right = next_state.right_leg_relative_theta +
                                     chassis->state_config.leg_world_body_pitch_gain * next_state.theta_body +
                                     chassis->state_config.right_leg_world_offset;
        next_state.theta_leg_right_dot = next_state.right_leg_relative_theta_dot +
                                         chassis->state_config.leg_world_body_pitch_gain * next_state.theta_body_dot;
    }

    next_state.valid_mask = valid_mask;
    WheelLeggedChassisStateFillVector(&next_state);
    chassis->chassis_state = next_state;
}

/**
 * @brief 请求下一次完整有效采样时重新记录底盘状态零点。
 *
 * @param chassis 需要重新置零的底盘对象。
 */
void WheelLeggedChassisStateResetOrigin(WheelLeggedChassisInstance_t *chassis)
{
    if (chassis != NULL)
    {
        chassis->chassis_state.origin_captured = 0u;
    }
}

/**
 * @brief 读取一台轮毂的轮端累计转角和角速度。
 *
 * @param wheel 轮毂运行对象。
 * @param wheel_angle 返回轮端累计转角，单位 rad。
 * @param wheel_speed 返回轮端角速度，单位 rad/s。
 * @return 参数、配置和反馈均有效时返回 1，否则返回 0。
 */
static uint8_t WheelLeggedWheelReadState(WheelLeggedWheelInstance_t *wheel, float *wheel_angle,
                                         float *wheel_speed)
{
    if (wheel == NULL || wheel_angle == NULL || wheel_speed == NULL || wheel->motor == NULL ||
        wheel->configured == 0u || wheel->motor->measure.state != STATE_NORMAL ||
        !isfinite(wheel->wheel_radius) || wheel->wheel_radius <= 0.0f || !isfinite(wheel->reduction_ratio) ||
        wheel->reduction_ratio <= 0.0f || !WheelLeggedIsDirectionValid(wheel->direction) ||
        !isfinite(wheel->motor->measure.total_angle) || !isfinite(wheel->motor->measure.velocity))
    {
        if (wheel != NULL)
        {
            wheel->feedback_ready = 0u;
        }
        return 0u;
    }

    wheel->feedback_ready = 1u;
    *wheel_angle = wheel->direction * wheel->motor->measure.total_angle / wheel->reduction_ratio;
    *wheel_speed = wheel->direction * wheel->motor->measure.velocity / wheel->reduction_ratio;
    if (!isfinite(*wheel_angle) || !isfinite(*wheel_speed))
    {
        wheel->feedback_ready = 0u;
        return 0u;
    }
    return 1u;
}

/**
 * @brief 根据当前腿部 FK、雅可比和关节速度读取同一帧虚拟腿状态。
 *
 * @param leg 待读取的腿对象。
 * @param relative_direction FK 相对腿角到建模相对腿角的符号。
 * @param state 返回腿长、腿长速度、相对机身腿角和角速度。
 * @return 反馈、FK、雅可比和传动均有效时返回 1，否则返回 0。
 */
static uint8_t WheelLeggedLegReadState(const WheelLeggedLegInstance_t *leg, float relative_direction,
                                       WheelLeggedLegStateSnapshot_t *state)
{
    float phi_dot[LEG_KINEMATICS_INPUT_COUNT] = {0.0f};
    const JointTransmissionConfig_t *front_transmission;
    const JointTransmissionConfig_t *rear_transmission;
    const float (*jacobian)[2];

    if (leg == NULL || state == NULL || leg->kinematics.config == NULL ||
        leg->front_joint.motor == NULL || leg->rear_joint.motor == NULL || leg->front_joint.feedback_ready == 0u ||
        leg->rear_joint.feedback_ready == 0u || leg->kinematics.forward_kinematics_status != DOUBLE_CLOSED_LOOP_LEG_OK ||
        leg->kinematics.state.virtual_leg_jacobian_valid == 0u ||
        leg->front_joint_kinematics_input >= LEG_KINEMATICS_INPUT_COUNT ||
        leg->rear_joint_kinematics_input >= LEG_KINEMATICS_INPUT_COUNT ||
        leg->front_joint_kinematics_input == leg->rear_joint_kinematics_input ||
        !WheelLeggedIsDirectionValid(relative_direction))
    {
        return 0u;
    }

    front_transmission = &leg->kinematics.config->transmission[leg->front_joint_kinematics_input];
    rear_transmission = &leg->kinematics.config->transmission[leg->rear_joint_kinematics_input];
    jacobian = leg->kinematics.state.virtual_leg_jacobian;
    if (front_transmission->configured == 0u || rear_transmission->configured == 0u ||
        !isfinite(front_transmission->gain) || !isfinite(rear_transmission->gain) ||
        !WheelLeggedIsDirectionValid(front_transmission->direction) ||
        !WheelLeggedIsDirectionValid(rear_transmission->direction) ||
        !isfinite(leg->front_joint.motor->measure.velocity) || !isfinite(leg->rear_joint.motor->measure.velocity) ||
        !isfinite(leg->kinematics.state.length) || leg->kinematics.state.length < 0.0f ||
        !isfinite(leg->kinematics.state.virtual_leg_theta) || !isfinite(jacobian[0][0]) ||
        !isfinite(jacobian[0][1]) || !isfinite(jacobian[1][0]) || !isfinite(jacobian[1][1]))
    {
        return 0u;
    }

    phi_dot[leg->front_joint_kinematics_input] = front_transmission->direction * front_transmission->gain *
                                                  leg->front_joint.motor->measure.velocity;
    phi_dot[leg->rear_joint_kinematics_input] = rear_transmission->direction * rear_transmission->gain *
                                                 leg->rear_joint.motor->measure.velocity;
    state->length = leg->kinematics.state.length;
    state->length_dot = jacobian[0][LEG_KINEMATICS_INPUT_PHI1] * phi_dot[LEG_KINEMATICS_INPUT_PHI1] +
                        jacobian[0][LEG_KINEMATICS_INPUT_PHI2] * phi_dot[LEG_KINEMATICS_INPUT_PHI2];
    state->relative_theta = relative_direction * leg->kinematics.state.virtual_leg_theta;
    state->relative_theta_dot = relative_direction *
                                (jacobian[1][LEG_KINEMATICS_INPUT_PHI1] * phi_dot[LEG_KINEMATICS_INPUT_PHI1] +
                                 jacobian[1][LEG_KINEMATICS_INPUT_PHI2] * phi_dot[LEG_KINEMATICS_INPUT_PHI2]);
    return isfinite(state->length) && isfinite(state->length_dot) && isfinite(state->relative_theta) &&
           isfinite(state->relative_theta_dot);
}

/**
 * @brief 读取 IMU 的 yaw、pitch 及对应角速度，并统一换算为弧度单位。
 *
 * @param imu 底盘 INS 运行对象。
 * @param yaw 返回展开 yaw，单位 rad。
 * @param yaw_rate 返回 yaw 角速度，单位 rad/s。
 * @param pitch 返回 pitch，单位 rad。
 * @param pitch_rate 返回 pitch 角速度，单位 rad/s。
 * @return IMU 已初始化且数据有限时返回 1，否则返回 0。
 */
static uint8_t WheelLeggedImuReadState(const INS_t *imu, float *yaw, float *yaw_rate, float *pitch,
                                       float *pitch_rate)
{
    if (imu == NULL || yaw == NULL || yaw_rate == NULL || pitch == NULL || pitch_rate == NULL || imu->init == 0u ||
        !isfinite(imu->YawTotalAngle) || !isfinite(imu->Gyro[Z]) || !isfinite(imu->Pitch) ||
        !isfinite(imu->Gyro[X]))
    {
        return 0u;
    }

    *yaw = imu->YawTotalAngle * DEGREE_2_RAD;
    *yaw_rate = imu->Gyro[Z] * DEGREE_2_RAD;
    *pitch = imu->Pitch * DEGREE_2_RAD;
    *pitch_rate = imu->Gyro[X] * DEGREE_2_RAD;
    return isfinite(*yaw) && isfinite(*yaw_rate) && isfinite(*pitch) && isfinite(*pitch_rate);
}

/**
 * @brief 判断一个方向配置是否为有限的 +1 或 -1。
 *
 * @param direction 待判断的方向配置。
 * @return 为 +1 或 -1 时返回 1，否则返回 0。
 */
static uint8_t WheelLeggedIsDirectionValid(float direction)
{
    return isfinite(direction) && (direction == 1.0f || direction == -1.0f);
}

/**
 * @brief 将具名状态字段按唯一固定顺序写入十维状态数组。
 *
 * @param state 已完成计算的底盘状态对象。
 */
static void WheelLeggedChassisStateFillVector(WheelLeggedChassisState_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->state_vector[WHEEL_LEGGED_STATE_S] = state->s;
    state->state_vector[WHEEL_LEGGED_STATE_S_DOT] = state->s_dot;
    state->state_vector[WHEEL_LEGGED_STATE_PHI] = state->phi;
    state->state_vector[WHEEL_LEGGED_STATE_PHI_DOT] = state->phi_dot;
    state->state_vector[WHEEL_LEGGED_STATE_THETA_LEFT] = state->theta_leg_left;
    state->state_vector[WHEEL_LEGGED_STATE_THETA_LEFT_DOT] = state->theta_leg_left_dot;
    state->state_vector[WHEEL_LEGGED_STATE_THETA_RIGHT] = state->theta_leg_right;
    state->state_vector[WHEEL_LEGGED_STATE_THETA_RIGHT_DOT] = state->theta_leg_right_dot;
    state->state_vector[WHEEL_LEGGED_STATE_THETA_BODY] = state->theta_body;
    state->state_vector[WHEEL_LEGGED_STATE_THETA_BODY_DOT] = state->theta_body_dot;
}
