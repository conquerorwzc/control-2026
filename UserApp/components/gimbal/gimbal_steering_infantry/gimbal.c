#include "gimbal.h"
#include "user_lib.h"
/**
******************************************************************************
* @file    gimbal.h
* @author  NeoZeng
* @author  Annotation and Modification By Enhao Zhang
* @date    2025/10/10
* @copyright Copyright (c) SHU SRM 2025 all rights reserved
* @brief   Standard Gimbal Module
******************************************************************************
* @attention
******************************************************************************
*/

static GimbalInstance* gimbal;
static Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd;  // 声明但不初始化

static void gimbal_motor_second_order_linear_controller_init(gimbal_motor_second_order_linear_controller_t *controller,  gimbal_motor_second_order_linear_controller_init_Config_s *config);
static float gimbal_motor_second_order_linear_controller_calc(gimbal_motor_second_order_linear_controller_t *controller, float set_angle, float cur_angle, float set_angle_speed, float cur_angle_speed, float cur_current);
static void GimbalMotorAbsoluteAngleControl(GimbalInstance* gimbal);

// static BMI088Instance *bmi088; // 云台IMU
GimbalInstance* GimbalInit(Gimbal_Init_Config_s* gimbal_init_config) {
  GimbalInstance* gimbal_instance = (GimbalInstance*)zmalloc(sizeof(GimbalInstance));
  gimbal_instance->gimbal_IMU_data = INS_Init();  // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源

  // YAW控制器参数配置
  gimbal_init_config->yaw_motor_config.controller_param_init_config.other_angle_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->YawTotalAngle;
  gimbal_init_config->yaw_motor_config.controller_param_init_config.other_speed_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Gyro[2];

  // YAW控制器设置配置
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;

  // PITCH控制器参数配置
  gimbal_init_config->pitch_motor_config.controller_param_init_config.other_angle_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Pitch;
  // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
  gimbal_init_config->pitch_motor_config.controller_param_init_config.other_speed_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Gyro[0];
  // 设置PITCH轴前馈指针
  //gimbal_init_config->pitch_motor_config.controller_param_init_config.current_feedforward_ptr =

  // PITCH控制器设置配置
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;

  gimbal_instance->yaw_motor = DJIMotorInit(&gimbal_init_config->yaw_motor_config);
  gimbal_instance->pitch_motor = DMMotorInit(&gimbal_init_config->pitch_motor_config);

#if defined(SECOND_ORDER_LINEAR_CONTROLLER_USED)
  gimbal_motor_second_order_linear_controller_init(&gimbal_instance->gimbal_yaw_motor_second_order_linear_controller, &gimbal_init_config->yaw_motor_second_order_linear_controller_init_config);
  gimbal_motor_second_order_linear_controller_init(&gimbal_instance->gimbal_pitch_motor_second_order_linear_controller, &gimbal_init_config->pitch_motor_second_order_linear_controller_init_config);
#endif

  gimbal = gimbal_instance;
  gimbal_ctrl_cmd = &gimbal->gimbal_ctrl_cmd;  // 在运行时初始化指针
  return gimbal_instance;
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask() {
  if (!gimbal) return;
  // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_POWER_OFF) {
    // 停止
    DJIMotorStop(gimbal->yaw_motor);
    DMMotorStop(gimbal->pitch_motor);
    return;
  }

  DJIMotorEnable(gimbal->yaw_motor);
  DMMotorEnable(gimbal->pitch_motor);

  // 调用核心控制函数
#if defined(SECOND_ORDER_LINEAR_CONTROLLER_USED)
  GimbalMotorAbsoluteAngleControl(gimbal);
#endif

#if defined(PID_USED)
  DJIMotorSetPIDRef(gimbal->yaw_motor, gimbal_ctrl_cmd->yaw);  // yaw和pitch会在robot_cmd中处理好多圈和单圈GimbalMotorAbsoluteAngleControl(gimbal);
#endif

  // 在合适的地方添加pitch重力补偿前馈力矩
  // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
  // ...
}

static void GimbalMotorAbsoluteAngleControl(GimbalInstance* gimbal) {
  if (!gimbal || !gimbal->yaw_motor || !gimbal->pitch_motor || !gimbal->gimbal_IMU_data) return;

  // 获取当前IMU数据
  float yaw_current_angle = gimbal->gimbal_IMU_data->YawTotalAngle;
  float yaw_current_speed = gimbal->gimbal_IMU_data->Gyro[2];

  // 使用二阶线性控制器计算YAW输出
  float yaw_output = gimbal_motor_second_order_linear_controller_calc(
      &gimbal->gimbal_yaw_motor_second_order_linear_controller,
      gimbal->gimbal_ctrl_cmd.yaw,    // 设定角度
      yaw_current_angle,              // 当前角度
      0.0f,                           // 设定角速度
      yaw_current_speed,              // 当前角速度
      gimbal->yaw_motor->measure.real_current  // 当前电流
  );
  // PITCH轴：直接角度控制（带软件限位）
  float pitch_set = gimbal->gimbal_ctrl_cmd.pitch;

  // 软件限位
  if (pitch_set > PITCH_MAX_ANGLE) {
    pitch_set = PITCH_MAX_ANGLE;
  } else if (pitch_set < PITCH_MIN_ANGLE) {
    pitch_set = PITCH_MIN_ANGLE;
  }

  // 设置电机输出
  gimbal->yaw_motor->motor_controller.final_output = (int16_t)yaw_output;
  DMMotorSetRef(gimbal->pitch_motor, pitch_set);
}

/**
 * @brief 云台二阶线性控制器初始化
 *
 * @param controller 云台二阶线性控制器结构体
 * @param config
 */
static void gimbal_motor_second_order_linear_controller_init(gimbal_motor_second_order_linear_controller_t *controller, gimbal_motor_second_order_linear_controller_init_Config_s *config)
{
    // 前馈项系数
    controller->k_feed_forward = config->k_feed_forward;
    // 反馈矩阵系数
    controller->k_angle_error = config->k_angle_error;
    controller->k_angle_speed = config->k_angle_speed;
    // 设置最大输出值
    controller->max_out = config->max_out;
    // 设置最小输出值
    controller->min_out = config->min_out;
}

/**
 * @brief 云台二阶线性控制器计算
 * @param controller 云台二阶线性控制器结构体
 * @param set_angle 角度设置值
 * @param cur_angle 当前角度
 * @param cur_angle_speed 当前角速度
 * @param cur_current 当前电流
 * @return 返回系统输入
 */
static float gimbal_motor_second_order_linear_controller_calc(gimbal_motor_second_order_linear_controller_t *controller, float set_angle, float cur_angle, float set_angle_speed, float cur_angle_speed, float cur_current)
{
    // 赋值
    controller->cur_angle = cur_angle;
    controller->set_angle = set_angle;
    controller->cur_angle_speed = cur_angle_speed;
    // 将当前电流值乘以一个小于1的系数当作阻挡系统固有扰动的前馈项
    controller->feed_forward = controller->k_feed_forward * cur_current;
    // if (gimbal->yaw_motor->motor_controller.current_feedforward_ptr != NULL) {
    //   controller->feed_forward = *(gimbal->yaw_motor->motor_controller.current_feedforward_ptr);
    // } else {
    //   controller->feed_forward = 0.0f;
    // }
    // 直接使用电机控制器的电流前馈数据
    controller->set_angle_speed = set_angle_speed;

    // 计算误差 = 设定角度 - 当前角度
    controller->angle_error = controller->set_angle - controller->cur_angle;

    // 将误差值限制 -PI ~ PI 之间
    controller->angle_error = rad_format(controller->angle_error);
    // 计算输出值 = 前馈值 + 角度误差值 * 系数 + 角速度误差 * 系数
    controller->output = controller->feed_forward + controller->angle_error * controller->k_angle_error + (controller->set_angle_speed - controller->cur_angle_speed) * controller->k_angle_speed;

    // 限制输出值，防止出现电机崩溃的情况
    if (controller->output >= controller->max_out)
    {
        controller->output = controller->max_out;
    }
    else if (controller->output <= controller->min_out)
    {
        controller->output = controller->min_out;
    }

    return controller->output;
}
