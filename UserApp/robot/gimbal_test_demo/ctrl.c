//
// Created by 20211 on 2026/11/27.
//

#include "ctrl.h"

#include "bsp_dwt.h"
#include "robot_config.h"
#include "user_lib.h"

/*  摇杆参数  */
#define RC_ROCKER_MAX 660.0f
#define YAW_STICK_SPEED 100.0f
#define PITCH_STICK_SPEED 100.0f
#define STICK_DEADBAND 30

/*  鼠标参数  */
#define MOUSE_YAW_SPEED 120.0f       // 与灵敏度正相关
#define MOUSE_PITCH_SPEED 100.0f     // 与灵敏度正相关
#define RC_MOUSE_MAX 270.0f
#define MOUSE_DEADBAND 30

/*  发射参数  */
#define TRIGGER_LONG_PRESS_S 1.0f

/*  内部状态  */
static Gimbal_Ctrl_Cmd_s *gimbal_cmd;
static Shoot_Ctrl_Cmd_s *shoot_cmd;

static Ctrl_Mode_e ctrl_mode;
static uint8_t motor_en;        // 云台最终使能
static uint8_t gimbal_enabled;  // 上一周期使能(检测上升沿)

static float trigger_time;      // 左键按下时刻
static uint8_t press_l_last;

/*  工具  */

static void PitchLimit(void) {
  if (gimbal_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_cmd->pitch = PITCH_MIN_ANGLE;
  }
}

static void ShootAllOff(void) {
  shoot_cmd->shoot_mode = SHOOT_OFF;
  shoot_cmd->friction_mode = FRICTION_OFF;
  shoot_cmd->load_mode = LOAD_STOP;
  press_l_last = 0;
}

/*  初始化  */

void CtrlInit(RobotInstance *robot) {
  gimbal_cmd = &robot->gimbal->gimbal_ctrl_cmd;
  shoot_cmd = &robot->shoot->shoot_ctrl_cmd;

  gimbal_cmd->gimbal_mode = GIMBAL_POWER_OFF;
  gimbal_cmd->yaw = 0.0f;
  gimbal_cmd->pitch = 0.0f;

  shoot_cmd->shoot_mode = SHOOT_OFF;
  shoot_cmd->friction_mode = FRICTION_OFF;
  shoot_cmd->load_mode = LOAD_STOP;
  shoot_cmd->heat_mode = NO_CONTROL;
  shoot_cmd->bullet_speed_mode = DISABLE_BULLET_SPEED;

  ctrl_mode = CTRL_MODE_NONE;
  motor_en = 0;
  gimbal_enabled = 0;
  trigger_time = 0.0f;
  press_l_last = 0;
}

/*  摇杆控制  */

void JoyStickCtrl(RobotInstance *robot) {
  /* 摇杆模式下发射强制全停(没有鼠标, 不能打弹) */
  ShootAllOff();

  float dx = (float)robot->rc_data[TEMP].rc.rocker_l_;
  float dy = (float)robot->rc_data[TEMP].rc.rocker_l1;

  if (abs((int)dx) > STICK_DEADBAND) {
    gimbal_cmd->yaw += -YAW_STICK_SPEED / RC_ROCKER_MAX * dx * robot->dt;
  }
  if (abs((int)dy) > STICK_DEADBAND) {
    gimbal_cmd->pitch += PITCH_STICK_SPEED / RC_ROCKER_MAX * dy * robot->dt;
  }

  PitchLimit();
}

/*  键鼠控制  */

void MouseKeyCtrl(RobotInstance *robot) {
  /*  鼠标控云台  */
  float dx = float_constrain((float)robot->rc_data[TEMP].mouse.x, -RC_MOUSE_MAX, RC_MOUSE_MAX);
  float dy = float_constrain((float)robot->rc_data[TEMP].mouse.y, -RC_MOUSE_MAX, RC_MOUSE_MAX);

  if (abs((int)dx) > MOUSE_DEADBAND) {
    gimbal_cmd->yaw += -MOUSE_YAW_SPEED / RC_MOUSE_MAX * dx * robot->dt;
  }
  if (abs((int)dy) > MOUSE_DEADBAND) {
    gimbal_cmd->pitch += MOUSE_PITCH_SPEED / RC_MOUSE_MAX * dy * robot->dt;
  }

  PitchLimit();

  /*  发射: 左拨杆选状态 + 左键单发/长按连发  */
  uint8_t sw_left = robot->rc_data[TEMP].rc.switch_left;

  if (switch_is_off(sw_left) || switch_is_down(sw_left)) {  // 无效 / [下]
    ShootAllOff();
    return;
  }

  if (switch_is_mid(sw_left)) {  // [中]: 摩擦轮 ON, 不拨弹
    shoot_cmd->shoot_mode = SHOOT_ON;
    shoot_cmd->friction_mode = FRICTION_ON;
    shoot_cmd->load_mode = LOAD_STOP;
    press_l_last = 0;
    return;
  }

  /* [上]: 摩擦轮 ON + 左键单发/长按连发 */
  shoot_cmd->shoot_mode = SHOOT_ON;
  shoot_cmd->friction_mode = FRICTION_ON;

  uint8_t press_l = robot->rc_data[TEMP].mouse.press_l;

  if (press_l && !press_l_last) {
    trigger_time = DWT_GetTimeline_s();
  }
  press_l_last = press_l;

  if (!press_l) {
    shoot_cmd->load_mode = LOAD_STOP;
    return;
  }

  if (DWT_GetTimeline_s() - trigger_time >= TRIGGER_LONG_PRESS_S) {
    shoot_cmd->load_mode = LOAD_BURSTFIRE;
  } else {
    shoot_cmd->load_mode = LOAD_1_BULLET;
  }
}

/*  急停与失能保护  */

void EmergencyHandler(RobotInstance *robot) {
    static uint16_t init_hold = 0;
    if (init_hold < 100) {
        init_hold++;
        gimbal_cmd->gimbal_mode = GIMBAL_POWER_OFF;
        gimbal_enabled = 0;
        ShootAllOff();
        return;
    }

    uint8_t sw = robot->rc_data[TEMP].rc.switch_right;

    /* 使能判定: 遥控器在线 && 右拨杆不在[下] && 拨杆值有效  */
    motor_en = RemoteControlIsOnline() && !switch_is_down(sw) && !switch_is_off(sw);

    if (!motor_en) {
        gimbal_cmd->gimbal_mode = GIMBAL_POWER_OFF;
        gimbal_enabled = 0;
        ShootAllOff();
        return;
    }

    gimbal_cmd->gimbal_mode = GIMBAL_ON;

    /* 使能上升沿: 目标角对齐当前姿态 */
    if (!gimbal_enabled) {
        gimbal_cmd->yaw = robot->gimbal->gimbal_IMU_data->YawTotalAngle;
        gimbal_cmd->pitch = float_constrain(robot->gimbal->gimbal_IMU_data->Pitch, PITCH_MIN_ANGLE,
                                            PITCH_MAX_ANGLE);
        gimbal_enabled = 1;
    }
}

/*  主入口  */

void CtrlSolve(RobotInstance *robot) {
  uint8_t sw = robot->rc_data[TEMP].rc.switch_right;

  /*  右拨杆选模式 + 控制  */
  if (switch_is_up(sw)) {
    ctrl_mode = CTRL_MODE_STICK;
    JoyStickCtrl(robot);
  } else if (switch_is_mid(sw)) {
    ctrl_mode = CTRL_MODE_KEYMOUSE;
    MouseKeyCtrl(robot);
  } else {
    ctrl_mode = CTRL_MODE_NONE;  // [下]: 不产生控制量
  }

  /*  急停最后执行, 保证 POWER_OFF 优先级最高  */
  EmergencyHandler(robot);
}

/*  调试接口   */

Ctrl_Mode_e CtrlGetMode(void) { return ctrl_mode; }

uint8_t CtrlIsMotorEnabled(void) { return motor_en; }