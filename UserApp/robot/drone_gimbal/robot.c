#include "robot.h"
#include "robot_config.h"
#include "user_lib.h"

RobotInstance *robot;

// 辅助限幅
static void LimitTarget(float *val, float min, float max) {
  if (*val > max) *val = max;
  if (*val < min) *val = min;
}

void RobotInit(void) {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

  // 1. 初始化遥控器
  robot->rc = RemoteControlInit(&huart3);

  // 2. 初始化云台组件
  Gimbal_Init_Config_s gimbal_conf = {
      .yaw_motor_config = YAW_CONFIG(&hcan1, YAW_MOTOR_ID),
      .pitch_motor_config = PITCH_CONFIG(&hcan1, PITCH_MOTOR_ID)
  };
  robot->gimbal = GimbalInit(&gimbal_conf);

  // ============================================================
  // 运行时指针劫持 (Runtime Pointer Hijacking)
  // ============================================================
  // GimbalInit 默认把指针指像了原始 IMU 数据，我们要把它改指到我们自己的变量上！
  // 这样我们就可以在不修改 gimbal.c 的情况下，给它喂“修正后”的数据了。

  // 1. 劫持 Yaw 轴反馈指针
  robot->gimbal->yaw_motor->motor_controller.other_angle_feedback_ptr = &robot->yaw_imu_feed;
  robot->gimbal->yaw_motor->motor_controller.other_speed_feedback_ptr = &robot->gimbal->gimbal_IMU_data->Gyro[2]; // 速度暂不修正

  // 2. 劫持 Pitch 轴反馈指针
  robot->gimbal->pitch_motor->motor_controller.other_angle_feedback_ptr = &robot->pitch_imu_feed;
  robot->gimbal->pitch_motor->motor_controller.other_speed_feedback_ptr = &robot->gimbal->gimbal_IMU_data->Gyro[0]; // 速度暂不修正
}

// ---------------------------------------------------------
// IMU 数据修正 (Middleware)
// ---------------------------------------------------------
// 定义静态变量来存储“初始偏差”
static float yaw_offset = 0.0f;
static float pitch_offset = 0.0f;
static bool is_offset_init = false; // 标记是否已经完成归零

static void HackIMUData(void) {
  if (!robot->gimbal || !robot->gimbal->gimbal_IMU_data) return;

  // 1. 读取原始数据
  float raw_yaw = robot->gimbal->gimbal_IMU_data->YawTotalAngle;
  float raw_pitch = robot->gimbal->gimbal_IMU_data->Pitch;

  // 2. 上电自动校准逻辑 (Auto-Zero)
  // 我们利用 RobotControlLogic 循环运行的特性来计时
  // 假设主循环 1ms 一次，我们等 1000 次 (约1秒) 让数据稳下来
  static uint16_t cali_count = 0;

  if (!is_offset_init) {
    cali_count++;

    // 在前 1 秒内，为了防止 PID 乱动，强制把反馈给 0
    robot->yaw_imu_feed = 0;
    robot->pitch_imu_feed = 0;

    if (cali_count > 1000) { // 等待 1 秒 (让温漂飞一会儿)
      // --- 记录 Yaw 的初始偏差 ---
      // 比如此刻它是 -27.0，我们就记下来。以后 raw - (-27) 就等于 0 了
      yaw_offset = raw_yaw;

      // --- 记录 Pitch 的初始偏差 ---
      // 比如此刻它是 187.0 (微抬头)，我们也记下来
      pitch_offset = raw_pitch;

      is_offset_init = true; // 校准完成！锁定！
    }
    return;
  }

  // 3. 应用修正 (实时读数 - 初始偏差)

  // --- Yaw 轴 ---
  // 公式：(当前值 - 初始偏差) * 1.0f (左转正)
  // 比如：初始是 -41，offset 记住了 -41。
  // 现在不动还是 -41。运算： -41 - (-41) = 0。 完美归零！
  robot->yaw_imu_feed = (raw_yaw - yaw_offset) * 1.0f;

  // --- Pitch 轴 ---
  // 公式：(当前值 - 初始偏差)
  // 比如：初始是 -6，offset 记住了 -6。
  // 运算： -6 - (-6) = 0。 完美归零！
  float diff_pitch = raw_pitch - pitch_offset;

  // 处理 0/360 跳变 (防止转一圈数据乱飞)
  if (diff_pitch < -180.0f) diff_pitch += 360.0f;
  if (diff_pitch > 180.0f)  diff_pitch -= 360.0f;

  // Pitch 方向修正 (如果抬头数值变小，前面加个负号： -diff_pitch)
  // 根据你之前的测试，抬头是变大，所以不用加负号
  robot->pitch_imu_feed = diff_pitch;
}

// ---------------------------------------------------------
// 功能逻辑
// ---------------------------------------------------------
static void RobotControlLogic(void) {
  if (!robot->gimbal) return;

  Gimbal_Ctrl_Cmd_s *cmd = &robot->gimbal->gimbal_ctrl_cmd;

  // 读取修正后的反馈值（现在不会闪烁了）
  float current_yaw = robot->yaw_imu_feed;
  float current_pitch = robot->pitch_imu_feed;

  // --- 上电 1 秒静止逻辑 ---
  static uint16_t startup_count = 0;
  if (startup_count < 1000) {
    startup_count++;
    // 同步目标 = 实际
    cmd->yaw = current_yaw;
    cmd->pitch = current_pitch;
    cmd->gimbal_mode = GIMBAL_POWER_OFF;
    return;
  }

  cmd->gimbal_mode = GIMBAL_ON;

  // 计算目标
  float target_yaw = cmd->yaw;
  float target_pitch = cmd->pitch;

  target_yaw -= ((float)robot->rc->rc.rocker_l_ / 660.0f) * 0.3f;
  target_pitch += ((float)robot->rc->rc.rocker_r1 / 660.0f) * 0.3f;

  // Pitch 限位暂时给大一点，防止测试时卡住
  LimitTarget(&target_pitch, -50.0f, 50.0f);

  cmd->yaw = target_yaw;
  cmd->pitch = target_pitch;
}

void RobotTask(void) {
  // 1. 计算修正数据
  HackIMUData();

  // 2. 计算控制目标
  RobotControlLogic();

  // 3. 执行组件
  GimbalTask();
}