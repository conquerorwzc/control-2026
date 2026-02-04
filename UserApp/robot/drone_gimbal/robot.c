#include "robot.h"
#include "robot_config.h"
#include "user_lib.h"

RobotInstance *robot;

// 辅助限幅函数
static void LimitTarget(float *val, float min, float max) {
  if (*val > max) *val = max;
  if (*val < min) *val = min;
}

// ---------------------------------------------------------
// 机器人初始化
// ---------------------------------------------------------
void RobotInit(void) {
  // 1. 分配内存
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

  // 2. 初始化遥控器
  robot->rc = RemoteControlInit(&huart3);

  // 3. 初始化云台
  robot->gimbal = GimbalInit(&gimbal_init_config);

  // 4. 初始化发射机构
  robot->shoot = ShootInit(&shoot_init_config);

  robot->safety_lock = true;

  // 5. 关键步骤：绑定串级 PID 的反馈指针
  // 告诉电机驱动层：外环(角度环)的数据来源是我们自己在 RobotControlLogic 里算出来的 yaw_imu_feed
  if (robot->gimbal) {
    if (robot->gimbal->yaw_motor)
      robot->gimbal->yaw_motor->motor_controller.other_angle_feedback_ptr = &robot->yaw_imu_feed;

    if (robot->gimbal->pitch_motor)
      robot->gimbal->pitch_motor->motor_controller.other_angle_feedback_ptr = &robot->pitch_imu_feed;
  }
}

/*核心控制逻辑 */
static void RobotControlLogic(void) {
  // 安全检查
  if (!robot->gimbal || !robot->gimbal->gimbal_IMU_data) return;
  Gimbal_Ctrl_Cmd_s *cmd = &robot->gimbal->gimbal_ctrl_cmd;

  // 1. 基础数据读取
  float raw_yaw = robot->gimbal->gimbal_IMU_data->YawTotalAngle;
  float raw_pitch = robot->gimbal->gimbal_IMU_data->Pitch;
  int16_t ecd = robot->gimbal->yaw_motor->measure.ecd;

  float current_yaw = -raw_yaw;
  float current_pitch = raw_pitch - 2.0f;

  robot->yaw_imu_feed = current_yaw;
  robot->pitch_imu_feed = current_pitch;

  // ==========================================================
  // 2. 安全与模式裁决 (右侧拨杆)
  // ==========================================================
  // [规则1] 右侧拨杆下 -> 强制上锁
  if (switch_is_down(robot->rc->rc.switch_right)) {
    robot->safety_lock = true;
  }
  // [规则2] 只有拨回中间，才尝试解锁
  else if (switch_is_mid(robot->rc->rc.switch_right)) {
    robot->safety_lock = false;
  }

  // [执行锁定]
  if (robot->safety_lock) {
    cmd->gimbal_mode = GIMBAL_POWER_OFF;
    cmd->yaw = current_yaw;
    cmd->pitch = current_pitch;
    if(robot->shoot) {
      robot->shoot->shoot_ctrl_cmd.shoot_mode = SHOOT_OFF;
      robot->shoot->shoot_ctrl_cmd.friction_mode = FRICTION_OFF;
      robot->shoot->shoot_ctrl_cmd.load_mode = LOAD_STOP;
    }
    return;
  }

  // ==========================================================
  // 3. 运行逻辑 (已解锁)
  // ==========================================================
  cmd->gimbal_mode = GIMBAL_ON;

  float target_yaw = cmd->yaw;
  float target_pitch = cmd->pitch;
  float yaw_step = 0.0f;
  float pitch_step = 0.0f;

  // 发射指令缓存
  uint8_t shoot_mode = SHOOT_ON;
  uint8_t friction_cmd = FRICTION_OFF;
  uint8_t load_cmd = LOAD_STOP;

  // ----------------------------------------------------------
  // [模式 A]: 键鼠模式 (右侧开关【上】)
  // ----------------------------------------------------------
  if (switch_is_up(robot->rc->rc.switch_right)) {

    // >>> 3.1 鼠标控制云台 <<<
    yaw_step   += (float)robot->rc->mouse.x * 0.0005f;
    pitch_step += (float)robot->rc->mouse.y * 0.001f;

    // >>> 3.2 Q键开关摩擦轮 (Toggle逻辑) <<<
    static bool key_friction_active = false;
    static uint8_t last_q_press = 0;

    if (robot->rc->key_count[KEY_PRESS][Key_Q] != last_q_press) {
      key_friction_active = !key_friction_active;
      last_q_press = robot->rc->key_count[KEY_PRESS][Key_Q];
    }
    friction_cmd = key_friction_active ? FRICTION_ON : FRICTION_OFF;

    // >>> 3.3 R键反转 (Unjam) <<<
    if (robot->rc->key[KEY_PRESS].r) {
      load_cmd = LOAD_REVERSE;
    }
    // >>> 3.4 鼠标左键开火 <<<
    else if (key_friction_active && robot->rc->mouse.press_l) {
      load_cmd = LOAD_BURSTFIRE;
    }
    else {
      load_cmd = LOAD_STOP;
    }
  }

  // ----------------------------------------------------------
  // [模式 B]: 遥控器模式 (右侧开关【中】)
  // ----------------------------------------------------------
  else {
    // >>> 3.5 摇杆控制云台 <<<
    float rc_yaw_raw = (float)robot->rc->rc.rocker_l_;
    float rc_pitch_raw = (float)robot->rc->rc.rocker_r1;

    if (fabsf(rc_yaw_raw) < 10.0f) rc_yaw_raw = 0.0f;
    if (fabsf(rc_pitch_raw) < 10.0f) rc_pitch_raw = 0.0f;

    yaw_step = (rc_yaw_raw / 660.0f) * 0.06f;
    pitch_step = -(rc_pitch_raw / 660.0f) * 0.06f;

    // >>> 3.6 左侧拨杆控制发射 <<<
    if (switch_is_down(robot->rc->rc.switch_left)) {
      friction_cmd = FRICTION_OFF;
      load_cmd = LOAD_STOP;
    }
    else if (switch_is_mid(robot->rc->rc.switch_left)) {
      friction_cmd = FRICTION_ON;
      load_cmd = LOAD_STOP;
    }
    else if (switch_is_up(robot->rc->rc.switch_left)) {
      friction_cmd = FRICTION_ON;
      load_cmd = LOAD_BURSTFIRE;
    }
  }

  // ==========================================================
  // 4. 执行控制与限位
  // ==========================================================
  if (robot->shoot) {
    robot->shoot->shoot_ctrl_cmd.shoot_mode = shoot_mode;
    robot->shoot->shoot_ctrl_cmd.friction_mode = friction_cmd;
    robot->shoot->shoot_ctrl_cmd.load_mode = load_cmd;
  }

  if (ecd < 2000 && yaw_step < 0) yaw_step = 0;
  if (ecd > 3400 && yaw_step > 0) yaw_step = 0;

  target_yaw += yaw_step;
  target_pitch += pitch_step;
  LimitTarget(&target_pitch, 0.0f, 28.0f);

  cmd->yaw = target_yaw;
  cmd->pitch = target_pitch;
}

void RobotTask(void) {
  // 1. 计算所有逻辑目标值
  RobotControlLogic();

  // 2. 执行云台底层控制 (计算 PID，发送 CAN)
  GimbalTask();

  // 3. 执行发射机构底层控制
  ShootTask();

}