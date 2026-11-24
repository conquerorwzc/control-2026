#include "robot.h"

/**
 ******************************************************************************
 * @file    robot.c
 * @brief   云台 Demo
 *          - 左摇杆左右 -> 云台 yaw 目标角(速度式: 摇杆大小 = 目标角速度)
 *          - 左摇杆上下 -> 云台 pitch 目标角(速度式, 带限位)
 *          - 右拨杆[上]摇杆控制，[中]鼠标控制,[下] 或遥控器掉线 失能
 *          - E : 使能    R : 失能
 *          - 鼠标左右滑动:改变yaw角 鼠标上下滑动:改变pitch角
 *          - 松手后目标角不变 => 云台保持当前指向
 * @note    只初始化云台, 不涉及底盘/发射/视觉/双板通信
 ******************************************************************************
 */

#include <stdlib.h>

#include "bsp_dwt.h"
#include "daemon.h"
#include "robot_config.h"
#include "user_lib.h"

#include <sys/types.h>
/*摇杆参数*/
#define RC_ROCKER_MAX 660.0f      // 摇杆单侧满量程, 用于把原始值归一化
#define YAW_STICK_SPEED 100.0f    // 摇杆打满时 yaw 的目标角速度, deg/s
#define PITCH_STICK_SPEED 100.0f  // 摇杆打满时 pitch 的目标角速度, deg/s
#define STICK_DEADBAND 30         // 摇杆死区(原始 count), 防止回中不到位
/* 鼠标参数 */
#define MOUSE_YAW_SPEED 100.0f    // 鼠标打满时 yaw 的目标角速度, deg/s
#define MOUSE_PITCH_SPEED 100.0f   // 鼠标打满时 pitch 的目标角速度, deg/s
#define RC_MOUSE_MAX 270.0f          // 鼠标值归一化上限
#define MOUSE_DEADBAND 50          // 鼠标静止死区

/*输入源(互斥)*/
typedef enum {
    SRC_NONE = 0,  // 没有任何输入(云台自保持）
    SRC_STICK,     // 当前由遥控器摇杆控制
    SRC_MOUSE,       // 当前由鼠标控制
  } Input_Source_e;

static RobotInstance *robot;
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd;
static RC_ctrl_t *rc_data;
static uint8_t motor_en;         // 最终使能状态
static uint8_t key_en;           //键盘软开关（E使能,R失能）
static uint8_t gimbal_enabled;   // 上一周期的实际使能状态, 用于检测"使能上升沿"
static uint8_t key_e_last;       // 上一周期的 E 键状态, 用于边沿检测
static uint8_t key_r_last;       // 上一周期的 R 键状态
static Input_Source_e input_src; // 当前控制权归谁


static float CalcDt(void) {
  static float last_time;
  float now = DWT_GetTimeline_s();
  float dt = now - last_time;
  last_time = now;
  if (dt <= 0.0f || dt > 0.05f) {
    dt = 0.001f;  // 首周期或异常时退化为标称值(1kHz), 避免大跳变
  }
  return dt;
}

static void PitchLimit(void) {
  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }
}

static void RobotCMDTask(float dt) {
    static uint16_t init_hold = 0;
    if (init_hold < 100) {
        init_hold++;
        gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
        gimbal_enabled = 0;
        return;
    }
    uint8_t sw=rc_data[TEMP].rc.switch_right;
    uint8_t e_now = rc_data[TEMP].key[KEY_PRESS].e;
    uint8_t r_now = rc_data[TEMP].key[KEY_PRESS].r;
    float dx_stick = (float)rc_data[TEMP].rc.rocker_l_;  // 左摇杆 水平
    float dy_stick = (float)rc_data[TEMP].rc.rocker_l1;  // 左摇杆 竖直

    float dx_mouse = float_constrain((float)rc_data[TEMP].mouse.x, -RC_MOUSE_MAX, RC_MOUSE_MAX);
    float dy_mouse = float_constrain((float)rc_data[TEMP].mouse.y, -RC_MOUSE_MAX, RC_MOUSE_MAX);

    if (e_now && !key_e_last) key_en = 1;
    if (r_now && !key_r_last) key_en = 0;
    key_e_last = e_now;
    key_r_last = r_now;

    /* 掉线: 失能, 并且要求恢复后重新按 E */
    if (!RemoteControlIsOnline()) {
        key_en = 0;
    }
    /* 最终使能 = 软开关 && 拨杆不在[下] && 拨杆值有效 */
    motor_en = key_en && !switch_is_down(sw) && !switch_is_off(sw);

    if (!motor_en) {
        key_en=0;
        gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
        gimbal_enabled = 0;
        input_src = SRC_NONE;
        return;
    }
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
  /* 把 yaw和pitch 目标对齐当前姿态 */
  if (!gimbal_enabled) {
      gimbal_ctrl_cmd->yaw = robot->gimbal->gimbal_IMU_data->YawTotalAngle;
      gimbal_ctrl_cmd->pitch =
        float_constrain(robot->gimbal->gimbal_IMU_data->Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
      gimbal_enabled = 1;
      input_src = SRC_NONE;
  }

    /* 拨杆选模式: 上=摇杆, 中=键鼠 */
    if (switch_is_up(sw)) {
        input_src = SRC_STICK;
    } else {
        input_src = SRC_MOUSE;
    }

    if (input_src == SRC_STICK) {
        if (abs((int)dx_stick) > STICK_DEADBAND) {
            gimbal_ctrl_cmd->yaw += -YAW_STICK_SPEED / RC_ROCKER_MAX * dx_stick * dt;
        }
        if (abs((int)dy_stick) > STICK_DEADBAND) {
            gimbal_ctrl_cmd->pitch += -PITCH_STICK_SPEED / RC_ROCKER_MAX * dy_stick * dt;
        }
    } else if (input_src == SRC_MOUSE) {
        if (abs((int)dx_mouse) > MOUSE_DEADBAND) {
            gimbal_ctrl_cmd->yaw += -MOUSE_YAW_SPEED / RC_MOUSE_MAX * dx_mouse * dt;
        }
        if (abs((int)dy_mouse) > MOUSE_DEADBAND) {
            gimbal_ctrl_cmd->pitch += -MOUSE_PITCH_SPEED / RC_MOUSE_MAX * dy_mouse * dt;
        }
    }
  /* pitch限位 */
  PitchLimit();
}


void RobotInit(void) {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

#ifdef STM32F407xx
  robot->rc_data = RemoteControlInit(&huart3);
#elif defined(STM32H723XX)
  robot->rc_data = RemoteControlInit(&huart5);
#endif
  rc_data = robot->rc_data;

  /* 只初始化云台: GimbalInit 内部会把 yaw/pitch 的反馈源设成 IMU 并用陀螺仪闭环 */
  robot->gimbal = GimbalInit(&gimbal_init_config);
  gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;

  /* 上电默认失能, 等操作手拨杆使能; 第一次使能时目标角会对齐当前姿态 */
  gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
  gimbal_ctrl_cmd->yaw = 0.0f;
  gimbal_ctrl_cmd->pitch = 0.0f;
    motor_en = 0;
    gimbal_enabled = 0;
    key_e_last = 0;
    key_r_last = 0;
    key_en = 0;
    input_src = SRC_NONE;
}


void RobotTask(void) {
  float dt = CalcDt();

  RobotCMDTask(dt);
  GimbalTask();
}