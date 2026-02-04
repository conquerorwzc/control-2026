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
  // 2. 右侧拨杆逻辑：总电源与安全 (最高优先级)
  // ==========================================================
  // 下: 熄火 / 急停 / 上锁
  // 中: 启动 / 解锁
  // 上: 启动状态下反转 (逻辑在后面处理)

  if (switch_is_down(robot->rc->rc.switch_right)) {
    robot->safety_lock = true; // 拨到下，立刻上锁
  }
  else if (switch_is_mid(robot->rc->rc.switch_right)) {
    robot->safety_lock = false; // 只有拨回中间，才解锁
  }
  // 注意：如果直接从“下”拨到“上”，safety_lock 依然是 true，防止误触反转

  // 锁定状态执行逻辑
  if (robot->safety_lock) {
    cmd->gimbal_mode = GIMBAL_POWER_OFF;
    cmd->yaw = current_yaw;
    cmd->pitch = current_pitch;

    // 强制关停发射
    if(robot->shoot) {
      robot->shoot->shoot_ctrl_cmd.shoot_mode = SHOOT_OFF;
      robot->shoot->shoot_ctrl_cmd.friction_mode = FRICTION_OFF;
      robot->shoot->shoot_ctrl_cmd.load_mode = LOAD_STOP;
    }
    return; // 这里的 return 保证了熄火时绝对没有任何动作
  }

  // ==========================================================
  // 3. 正常运行逻辑 (已解锁)
  // ==========================================================
  cmd->gimbal_mode = GIMBAL_ON;

  // --- A. 云台控制 (摇杆) ---
  float target_yaw = cmd->yaw;
  float target_pitch = cmd->pitch;

  float rc_yaw_raw = (float)robot->rc->rc.rocker_l_;
  float rc_pitch_raw = (float)robot->rc->rc.rocker_r1;

  if (fabsf(rc_yaw_raw) < 10.0f) rc_yaw_raw = 0.0f;
  if (fabsf(rc_pitch_raw) < 10.0f) rc_pitch_raw = 0.0f;

  float yaw_step = (rc_yaw_raw / 660.0f) * 0.12f;
  float pitch_step = -(rc_pitch_raw / 660.0f) * 0.12f;

  // --- B. 发射控制 (左侧拨杆) ---
  // 初始化指令
  uint8_t shoot_mode = SHOOT_ON;
  uint8_t friction_cmd = FRICTION_OFF;
  uint8_t load_cmd = LOAD_STOP;

  if (switch_is_down(robot->rc->rc.switch_left)) {
    // [左下]: 停止
    friction_cmd = FRICTION_OFF;
    load_cmd = LOAD_STOP;
  }
  else if (switch_is_mid(robot->rc->rc.switch_left)) {
    // [左中]: 待机 (只开摩擦轮)
    friction_cmd = FRICTION_ON;
    load_cmd = LOAD_STOP;
  }
  else if (switch_is_up(robot->rc->rc.switch_left)) {
    // [左上]: 开火 (摩擦轮 + 拨弹)
    friction_cmd = FRICTION_ON;
    load_cmd = LOAD_BURSTFIRE;
  }

  // --- C. 反转退弹 (右侧拨杆-上) ---
  // 前提：必须已经解锁 (safety_lock == false)，这在前面已经保证了
  if (switch_is_up(robot->rc->rc.switch_right)) {
    // 强制覆盖拨弹指令为反转
    load_cmd = LOAD_REVERSE;
    // 摩擦轮状态保持左侧拨杆的设定 (通常是为了把卡住的子弹喷出来，或者仅仅是退回弹仓)
  }

  // ==========================================================
  // 4. 执行控制与限位
  // ==========================================================

  // 下发发射指令
  if (robot->shoot) {
    robot->shoot->shoot_ctrl_cmd.shoot_mode = shoot_mode;
    robot->shoot->shoot_ctrl_cmd.friction_mode = friction_cmd;
    robot->shoot->shoot_ctrl_cmd.load_mode = load_cmd;
  }

  // ECD 物理限位
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