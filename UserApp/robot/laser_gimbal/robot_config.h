//
// Created by yang6 on 2026/3/3.
//
#include "gimbal.h"
#include "shoot.h"
#include "can_comm.h"
#include "HI05.h"
#ifndef CONTROL_2026_ROBOT_CONFIG_H
#define CONTROL_2026_ROBOT_CONFIG_H

static Gimbal_Init_Config_s gimbal_init_config = {
    .yaw_motor_config =
    {
       .controller_param_init_config =
          {
              .angle_PID =
                  {
                      .Kp = 70,
                      .Ki = 0,
                      .Kd = 0.4,
                      .IntegralLimit = 960,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .MaxOut = 1920,
                  },
              .speed_PID =
                  {
                      .Kp = 9,
                      .Ki = 26,
                      .Kd = 0,
                      .IntegralLimit = 12500,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .MaxOut = 22000,
                  },
              .current_PID =
                  {
                      .Kp = 0,
                      .Ki = 0,
                      .Kd = 0,
                      .IntegralLimit = 3000,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .MaxOut = 15000,
                  },
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,
           .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
              .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
              .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
          },
      .motor_type = GM6020,
      .can_init_config =
        {
            .can_handle = &hcan1,
            .tx_id = 4,
        },
},
    .pitch_motor_config =
    {
       .controller_param_init_config =
          {
              .angle_PID =
                  {
                      .Kp = 70,
                      .Ki = 0,
                      .Kd = 0.4,
                      .IntegralLimit = 960,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .MaxOut = 1920,
                  },
              .speed_PID =
                  {
                      .Kp = 9,
                      .Ki = 26,
                      .Kd = 0,
                      .IntegralLimit = 12500,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .MaxOut = 22000,
                  },
              .current_PID =
                  {
                      .Kp = 0,
                      .Ki = 0,
                      .Kd = 0,
                      .IntegralLimit = 3000,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .MaxOut = 15000,
                  },
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,
           .outer_loop_type = ANGLE_LOOP,
                 .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
              .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
              .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
          },
      .motor_type = GM6020,
      .can_init_config =
        {
            .can_handle = &hcan1,
            .tx_id = 6,
        },
},
};
#endif // CONTROL_2026_ROBOT_CONFIG_H
