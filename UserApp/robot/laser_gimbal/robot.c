//
// Created by yang6 on 2026/3/3.
//

#include "dji_motor.h"
#include "dmmotor.h"
#include "general_def.h"
#include "robot.h"
#include "robot_config.h"
#include "user_lib.h"
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd=NULL;
static RobotInstance* robot;
static  float yaw_max_angle=-40.0f;
static  float yaw_min_angle=-60.0f;
static  float T=2000;
static  float step;
static float x=0;

/* 三角波扫描参数 */
  float SCAN_AMPLITUDE = 0.5f;        /* 扫描幅度 (m) */
  float SCAN_FREQUENCY = 0.3f/1.0f;   /* 扫描频率 (Hz) */
  float BASE_DISTANCE = 14.5f;        /* 云台到目标的基准水平距离 (m) */
 float TARGET_HEIGHT = 1.2f;         /* 目标垂直高度差 (m) */
float factor=-3.6f;
  float TARGET_OFFSET = 4.2f;         /* 目标侧向偏移 (m) */
float offset_pitch=130.0f;
/**
 * @brief 生成三角波信号
 * @param amplitude 幅度 (峰值)
 * @param frequency 频率 (Hz)
 * @param time 当前时间 (秒)
 * @return float 三角波值
 */
static float TriangleWave(float amplitude, float frequency, float time)
{
    return (2.0f * amplitude / PI) * asinf(sinf(2.0f * PI * frequency * time));
}

void RobotInit() {
    robot=(RobotInstance *)zmalloc(sizeof(RobotInstance));
    robot->gimbal = GimbalInit(&gimbal_init_config);
    gimbal_ctrl_cmd=&robot->gimbal->gimbal_ctrl_cmd;
    robot->rc_data = RemoteControlInit(&huart3);
    step = (yaw_max_angle-yaw_min_angle)/T;

}

void RobotTask() {
    if (switch_is_down(robot->rc_data[TEMP].rc.switch_right))
    {
        gimbal_ctrl_cmd->gimbal_mode=GIMBAL_POWER_OFF;
        gimbal_ctrl_cmd->yaw = -15.0f;
        gimbal_ctrl_cmd->pitch = 105.0f;
    }
    else if(switch_is_mid(robot->rc_data[TEMP].rc.switch_right))
    {
        gimbal_ctrl_cmd->gimbal_mode=GIMBAL_ON;
        gimbal_ctrl_cmd->yaw += -0.0005f * (float)robot->rc_data[TEMP].rc.rocker_r_;
        gimbal_ctrl_cmd->pitch -= 0.0003f * (float)robot->rc_data[TEMP].rc.rocker_r1;
    }
    // else if (switch_is_up(robot->rc_data[TEMP].rc.switch_right))
    // {
    //     if (gimbal_ctrl_cmd->yaw>=yaw_max_angle)
    //     {
    //         step=(yaw_min_angle-yaw_max_angle)/T;
    //     }
    //     else if (gimbal_ctrl_cmd->yaw<=yaw_min_angle)
    //     {
    //         step=(yaw_max_angle-yaw_min_angle)/T;
    //     }
    //     gimbal_ctrl_cmd->yaw += step;
    //     gimbal_ctrl_cmd->pitch -= 0.0003f * (float)robot->rc_data[TEMP].rc.rocker_r1;
    // }
    else if (switch_is_up(robot->rc_data[TEMP].rc.switch_right))
    {
        //生成三角波扫描位移
        x = TriangleWave(SCAN_AMPLITUDE, SCAN_FREQUENCY, DWT_GetTimeline_s());
        
        /* 根据当前位置计算云台角度 */
        // yaw 角：水平方向瞄准角度 = arctan(侧向偏移 / 水平距离)
        gimbal_ctrl_cmd->yaw = -RAD_2_DEGREE*atanf(TARGET_OFFSET / (BASE_DISTANCE + x));
        // pitch 角：垂直方向瞄准角度 = arctan(垂直高度 / 斜边距离)
        // 斜边距离 = sqrt(水平距离² + 侧向偏移²)
        gimbal_ctrl_cmd->pitch = factor*RAD_2_DEGREE*atanf(TARGET_HEIGHT / sqrtf(powf(BASE_DISTANCE + x, 2.0f) + powf(TARGET_OFFSET, 2.0f)))+offset_pitch;
    }

     GimbalTask();

    // DMMotorSetPIDRef(J8009P_instance, speed_ref);
    // M3508_instance->motor_controller.final_output = target_torque * q2i_coeff * (16384.0f / 20.0f);
    //DJIMotorSetPIDRef(M3508_instance, speed_ref);
}
