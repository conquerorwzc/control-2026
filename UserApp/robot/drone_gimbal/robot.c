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

  // 2. 初始化遥控器 (使用 USART3，请确认你的硬件连接)
  robot->rc = RemoteControlInit(&huart3);

  // 3. 初始化云台 (使用 robot_config.h 中的配置)
  // 注意：gimbal_init_config 在 robot_config.h 中定义
  robot->gimbal = GimbalInit(&gimbal_init_config);

  // 4. 初始化发射机构 (使用 robot_config.h 中的配置)
  // 注意：shoot_init_config 在 robot_config.h 中定义
  robot->shoot = ShootInit(&shoot_init_config);

  // 5. 关键步骤：绑定串级 PID 的反馈指针
  // 告诉电机驱动层：外环(角度环)的数据来源是我们自己在 RobotControlLogic 里算出来的 yaw_imu_feed
  if (robot->gimbal) {
    if (robot->gimbal->yaw_motor)
      robot->gimbal->yaw_motor->motor_controller.other_angle_feedback_ptr = &robot->yaw_imu_feed;

    if (robot->gimbal->pitch_motor)
      robot->gimbal->pitch_motor->motor_controller.other_angle_feedback_ptr = &robot->pitch_imu_feed;
  }
}

// ---------------------------------------------------------
// 核心控制逻辑 (大脑)
// ---------------------------------------------------------
static void RobotControlLogic(void) {
  // 安全检查：如果云台没初始化或没有 IMU 数据，直接返回
  if (!robot->gimbal || !robot->gimbal->gimbal_IMU_data) return;

  Gimbal_Ctrl_Cmd_s *cmd = &robot->gimbal->gimbal_ctrl_cmd;

  // ==========================
  // 1. 数据读取与坐标系修正
  // ==========================
  float raw_yaw = robot->gimbal->gimbal_IMU_data->YawTotalAngle;
  float raw_pitch = robot->gimbal->gimbal_IMU_data->Pitch;

  // [Yaw 修正]
  // 恢复负反馈！左转为正，右转为负，防止疯转
  float current_yaw = -raw_yaw;

  // [Pitch 修正]
  // 归零：把物理水平(5度)映射为逻辑0度
  float current_pitch = raw_pitch - 5.0f;

  // [更新反馈]
  // 把修正后的数据写入 feed 变量，供电机 PID 使用
  robot->yaw_imu_feed = current_yaw;
  robot->pitch_imu_feed = current_pitch;

  // ==========================
  // 2. 上电 1 秒安全逻辑
  // ==========================
  static uint16_t startup_count = 0;
  if (startup_count < 1000) {
    startup_count++;

    // 让目标值(cmd)紧紧跟随当前值(current)，确保误差为 0
    // 这样上电瞬间电机不会猛地跳动
    cmd->yaw = current_yaw;
    cmd->pitch = current_pitch;

    // 前 1 秒可以选择不给电，或者给电但锁死位置
    cmd->gimbal_mode = GIMBAL_POWER_OFF;
    return;
  }

  // 1秒后，开启云台
  cmd->gimbal_mode = GIMBAL_ON;

  // ==========================
  // 3. 遥控器控制逻辑
  // ==========================
  float target_yaw = cmd->yaw;
  float target_pitch = cmd->pitch;

  // 获取原始摇杆值
  float rc_yaw_raw   = (float)robot->rc->rc.rocker_l_;  // 左摇杆水平 (Yaw)
  float rc_pitch_raw = (float)robot->rc->rc.rocker_r1;  // 右摇杆垂直 (Pitch)

  // [死区处理] - 解决漂移的核心
  if (fabsf(rc_yaw_raw) < 10.0f)   rc_yaw_raw = 0.0f;
  if (fabsf(rc_pitch_raw) < 10.0f) rc_pitch_raw = 0.0f;

  // [积分控制]
  // 灵敏度系数：Yaw (0.06) 稍微快点，Pitch (0.02) 慢点更稳
  target_yaw   += (rc_yaw_raw   / 660.0f) * 0.12f;
  target_pitch += (rc_pitch_raw / 660.0f) * 0.12f;

  // [软件限位]
  // Pitch 范围：逻辑 0.0 (物理5度) ~ 逻辑 30.0 (物理35度)
  LimitTarget(&target_pitch, 0.0f, 28.0f);

  // ==========================
  // 4. 发射机构简单控制 (示例)
  // ==========================
  // 假设左侧拨杆 (sw2) 控制发射：下=关，中=摩擦轮开，上=开火
  if (robot->shoot) {
    uint8_t sw_left = robot->rc->rc.switch_left;
    // 状态 1: 开关向下 ->【停止模式】(总开关关)
    if (switch_is_down(sw_left)) {
      robot->shoot->shoot_ctrl_cmd.shoot_mode = SHOOT_OFF;     // 总电源关
      robot->shoot->shoot_ctrl_cmd.friction_mode = FRICTION_OFF; // 摩擦轮关
      robot->shoot->shoot_ctrl_cmd.load_mode = LOAD_STOP;      // 拨盘停
    }
    // 状态 2: 开关中间 ->【准备模式】(摩擦轮转，但不拨弹)
    else if (switch_is_mid(sw_left)) {
      robot->shoot->shoot_ctrl_cmd.shoot_mode = SHOOT_ON;      // 总电源开
      robot->shoot->shoot_ctrl_cmd.friction_mode = FRICTION_ON;// 摩擦轮启动
      robot->shoot->shoot_ctrl_cmd.load_mode = LOAD_STOP;      // 拨盘等待
    }
    // 状态 3: 开关向上 ->【开火模式】(摩擦轮转 + 连发拨弹)
    else if (switch_is_up(sw_left)) {
      robot->shoot->shoot_ctrl_cmd.shoot_mode = SHOOT_ON;      // 总电源开
      robot->shoot->shoot_ctrl_cmd.friction_mode = FRICTION_ON;// 摩擦轮保持转
      robot->shoot->shoot_ctrl_cmd.load_mode = LOAD_BURSTFIRE; // 拨盘连发!
    }
  }

  // ==========================
  // 5. 更新最终命令
  // ==========================
  cmd->yaw = target_yaw;
  cmd->pitch = target_pitch;
}

// ---------------------------------------------------------
// RTOS 任务入口
// ---------------------------------------------------------
void RobotTask(void) {
  // 1. 计算所有逻辑目标值
  RobotControlLogic();

  // 2. 执行云台底层控制 (计算 PID，发送 CAN)
  GimbalTask();

  // 3. 执行发射机构底层控制
  ShootTask();

}