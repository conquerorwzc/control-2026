/**
 ******************************************************************************
 * @file    gimbal.c
 * @author  SRM-Control 2026
 * @brief   Dual Yaw Gimbal Module - 虚拟陀螺方案状态机实现
 ******************************************************************************
 * @attention
 *
 *  虚拟陀螺仪:
 *    θ_virtual = θ_gyro - (slave_total - slave_center_total_offset)
 *    ω_virtual = Gyro[2] - SLAVE_YAW_SPEED_DIR * slave->speed_aps
 *
 *    FOLLOW 模式下大yaw ref = θ_gyro, 反馈 = θ_virtual:
 *      error = θ_gyro - θ_virtual = slave_rel → 驱动大yaw缩小偏移
 *
 *  状态切换:
 *    NORMAL → FOLLOW: |θ_rel| ≥ 15°
 *    FOLLOW → NORMAL: |θ_rel| < 10° (迟滞)
 *    FOLLOW → LOCKED: |θ_rel| ≥ 30° (冻结 ref = θ_gyro)
 *    LOCKED → NORMAL: |virtual_gyro - lock_target| < 3°
 *
 ******************************************************************************
 */

#include "gimbal.h"
#include "user_lib.h"
#include "ins_task.h"

static GimbalDualYawInstance *gimbal;
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd;
static Gimbal_Mode_e gimbal_mode_last;
static float last_yaw_cmd;

/* ---- 私有辅助函数 ---- */

static float wrap180(float now, float last) {
  float diff = now - last;
  while (diff > 180.0f)  diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return last + diff;
}

static void ApplyPIDParams(PIDInstance *pid, const PID_Init_Config_s *config) {
  pid->Kp = config->Kp;
  pid->Ki = config->Ki;
  pid->Kd = config->Kd;
  pid->MaxOut = config->MaxOut;
  pid->DeadBand = config->DeadBand;
  pid->Improve = config->Improve;
  pid->IntegralLimit = config->IntegralLimit;
  pid->CoefA = config->CoefA;
  pid->CoefB = config->CoefB;
  pid->Output_LPF_RC = config->Output_LPF_RC;
  pid->Derivative_LPF_RC = config->Derivative_LPF_RC;
  pid->ITerm = 0.0f;
  pid->Last_ITerm = 0.0f;
  pid->Iout = 0.0f;
  pid->Last_Err = 0.0f;
  pid->Err = 0.0f;
}

/**
 * @brief 小yaw相对于机械中心的单圈最短弧偏移 (用于阈值判断)
 * @return [-180, 180]
 */
static float CalcSlaveRelativeShortArc(void) {
  if (gimbal->yaw_slave_motor == NULL) return 0.0f;
  float single = gimbal->yaw_slave_motor->measure.angle_single_round;
  float center = SLAVE_YAW_CENTER_ANGLE;
  float rel = single - center;
  while (rel > 180.0f)  rel -= 360.0f;
  while (rel < -180.0f) rel += 360.0f;
  return rel;
}

/**
 * @brief 小yaw相对于机械中心的连续偏移 (使用 total_angle, 无360°跳变)
 * @return 连续值, 为0时表示小yaw对中
 */
static float CalcSlaveRelativeContinuous(void) {
  if (gimbal->yaw_slave_motor == NULL) return 0.0f;
  return gimbal->yaw_slave_motor->measure.total_angle -
         gimbal->slave_center_total_offset;
}

/**
 * @brief 更新虚拟陀螺仪角度和角速度
 *        θ_virtual = θ_gyro - θ_slave_rel_continuous
 *        ω_virtual = ω_gyro - dir * ω_slave_aps
 */
static void UpdateVirtualGyro(void) {
  float theta_gyro = gimbal->gimbal_IMU_data->YawTotalAngle;
  float slave_rel_cont = CalcSlaveRelativeContinuous();
  gimbal->virtual_gyro_angle = theta_gyro - slave_rel_cont;
  gimbal->virtual_gyro_speed =
      gimbal->gimbal_IMU_data->Gyro[2] -
      (float)SLAVE_YAW_SPEED_DIR * gimbal->yaw_slave_motor->measure.speed_aps;
}

static void SwitchToNormalPID(void) {
  ApplyPIDParams(&gimbal->yaw_master_motor->motor_controller.angle_PID,
                 &gimbal->normal_angle_pid);
  ApplyPIDParams(&gimbal->yaw_master_motor->motor_controller.speed_PID,
                 &gimbal->normal_speed_pid);
}

static void SwitchToFollowPID(void) {
  ApplyPIDParams(&gimbal->yaw_master_motor->motor_controller.angle_PID,
                 &gimbal->follow_angle_pid);
  ApplyPIDParams(&gimbal->yaw_master_motor->motor_controller.speed_PID,
                 &gimbal->follow_speed_pid);
}

/* ---- 初始化 ---- */

GimbalDualYawInstance *GimbalDualYawInit(Gimbal_Dual_Yaw_Init_Config_s *config) {
  GimbalDualYawInstance *instance =
      (GimbalDualYawInstance *)zmalloc(sizeof(GimbalDualYawInstance));

  // 1. 初始化IMU
  instance->gimbal_IMU_data = INS_Init(&config->imu_init_config);

  // 2. 配置小yaw (始终使用IMU反馈, 与标准云台一致)
  config->yaw_slave_motor_config.controller_param_init_config.other_angle_feedback_ptr =
      &instance->gimbal_IMU_data->YawTotalAngle;
  config->yaw_slave_motor_config.controller_param_init_config.other_speed_feedback_ptr =
      &instance->gimbal_IMU_data->Gyro[2];
  config->yaw_slave_motor_config.controller_setting_init_config.angle_feedback_source =
      OTHER_FEED;
  config->yaw_slave_motor_config.controller_setting_init_config.speed_feedback_source =
      OTHER_FEED;
  config->yaw_slave_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  config->yaw_slave_motor_config.controller_setting_init_config.close_loop_type =
      SPEED_LOOP | ANGLE_LOOP;

  // 3. 配置大yaw (使用虚拟陀螺反馈, 与标准云台一致)
  config->yaw_master_motor_config.controller_param_init_config.other_angle_feedback_ptr =
      &instance->virtual_gyro_angle;
  config->yaw_master_motor_config.controller_param_init_config.other_speed_feedback_ptr =
      &instance->virtual_gyro_speed;
  config->yaw_master_motor_config.controller_setting_init_config.angle_feedback_source =
      OTHER_FEED;
  config->yaw_master_motor_config.controller_setting_init_config.speed_feedback_source =
      OTHER_FEED;
  config->yaw_master_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  config->yaw_master_motor_config.controller_setting_init_config.close_loop_type =
      SPEED_LOOP | ANGLE_LOOP;

  // 4. 配置pitch (使用IMU反馈)
  config->pitch_motor_config.controller_param_init_config.other_angle_feedback_ptr =
      &instance->gimbal_IMU_data->Pitch;
  config->pitch_motor_config.controller_param_init_config.other_speed_feedback_ptr =
      &instance->gimbal_IMU_data->Gyro[0];
  config->pitch_motor_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
  config->pitch_motor_config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
  config->pitch_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  config->pitch_motor_config.controller_setting_init_config.close_loop_type =
      SPEED_LOOP | ANGLE_LOOP;

  // 5. 初始化电机
  instance->yaw_master_motor = DJIMotorInit(&config->yaw_master_motor_config);
  instance->yaw_slave_motor = DJIMotorInit(&config->yaw_slave_motor_config);
  instance->pitch_motor = DMMotorInit(&config->pitch_motor_config);

  // 6. 保存两套PID参数
  instance->normal_angle_pid =
      config->yaw_master_motor_config.controller_param_init_config.angle_PID;
  instance->normal_speed_pid =
      config->yaw_master_motor_config.controller_param_init_config.speed_PID;
  instance->follow_angle_pid = config->yaw_master_follow_angle_PID;
  instance->follow_speed_pid = config->yaw_master_follow_speed_PID;

  // 7. 初始化状态变量
  instance->state = DUAL_YAW_NORMAL;
  instance->state_last = DUAL_YAW_NORMAL;
  instance->align_flag = 0;
  instance->virtual_gyro_angle = 0.0f;
  instance->virtual_gyro_speed = 0.0f;
  instance->slave_center_total_offset = 0.0f;
  instance->virtual_gyro_hold_angle = 0.0f;
  instance->virtual_gyro_lock_target = 0.0f;
  instance->yaw_master_feedforward_current = 0.0f;
  instance->yaw_slave_feedforward_current = 0.0f;

  // 8. 阈值
  instance->enter_follow_threshold = DUAL_YAW_ENTER_FOLLOW_THRESHOLD;
  instance->exit_follow_threshold = DUAL_YAW_EXIT_FOLLOW_THRESHOLD;
  instance->lock_threshold = DUAL_YAW_LOCK_THRESHOLD;
  instance->recover_threshold = DUAL_YAW_RECOVER_THRESHOLD;

  gimbal = instance;
  gimbal_ctrl_cmd = &instance->gimbal_ctrl_cmd;
  last_yaw_cmd = 0.0f;
  gimbal_mode_last = GIMBAL_POWER_OFF;

  return instance;
}

/* ---- 前馈接口 ---- */

void GimbalDualYawSetFeedforward(float master_current, float slave_current) {
  if (gimbal == NULL) return;
  gimbal->yaw_master_feedforward_current = master_current;
  gimbal->yaw_slave_feedforward_current = slave_current;
}

/* ---- 核心控制任务 ---- */

void GimbalDualYawTask(void) {
  static uint8_t init_delay = 0;

  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_POWER_OFF) {
    DJIMotorStop(gimbal->yaw_master_motor);
    DJIMotorStop(gimbal->yaw_slave_motor);
    DMMotorStop(gimbal->pitch_motor);

    gimbal_ctrl_cmd->yaw = gimbal->gimbal_IMU_data->YawTotalAngle;
    gimbal_ctrl_cmd->pitch = gimbal->gimbal_IMU_data->Pitch;

    gimbal->yaw_master_feedforward_current = 0.0f;
    gimbal->yaw_slave_feedforward_current = 0.0f;

    gimbal->state = DUAL_YAW_NORMAL;
    gimbal->align_flag = 0;
    init_delay = 0;

    gimbal_mode_last = GIMBAL_POWER_OFF;
    return;
  }

  DJIMotorEnable(gimbal->yaw_master_motor);
  DJIMotorEnable(gimbal->yaw_slave_motor);
  DMMotorEnable(gimbal->pitch_motor);

  /* ---- 上电对齐 ---- */
  if (!gimbal->align_flag) {
    if (++init_delay < 10) {
      gimbal_ctrl_cmd->yaw = gimbal->gimbal_IMU_data->YawTotalAngle;
      gimbal_ctrl_cmd->pitch = gimbal->gimbal_IMU_data->Pitch;
      last_yaw_cmd = gimbal_ctrl_cmd->yaw;
      gimbal_mode_last = gimbal_ctrl_cmd->gimbal_mode;
      return;
    }

    // 基于 SLAVE_YAW_CENTER_ECD 计算 slave_total_angle 对中的连续偏移量
    float slave_rel = CalcSlaveRelativeShortArc();
    float slave_total = gimbal->yaw_slave_motor->measure.total_angle;
    gimbal->slave_center_total_offset = slave_total - slave_rel;

    // 计算初始虚拟陀螺值并记录NORMAL保持目标
    UpdateVirtualGyro();
    gimbal->virtual_gyro_hold_angle = gimbal->virtual_gyro_angle;

    gimbal_ctrl_cmd->yaw = gimbal->gimbal_IMU_data->YawTotalAngle;
    gimbal_ctrl_cmd->pitch = gimbal->gimbal_IMU_data->Pitch;
    last_yaw_cmd = gimbal_ctrl_cmd->yaw;

    SwitchToNormalPID();

    gimbal->align_flag = 1;
    gimbal_mode_last = gimbal_ctrl_cmd->gimbal_mode;
    return;
  }

  /* ---- 更新虚拟陀螺仪 ---- */
  UpdateVirtualGyro();

  /* ---- 从POWER_OFF恢复 ---- */
  if (gimbal_mode_last == GIMBAL_POWER_OFF) {
    gimbal_ctrl_cmd->yaw = gimbal->gimbal_IMU_data->YawTotalAngle;
    gimbal_ctrl_cmd->pitch = gimbal->gimbal_IMU_data->Pitch;
    last_yaw_cmd = gimbal_ctrl_cmd->yaw;
    gimbal->virtual_gyro_hold_angle = gimbal->virtual_gyro_angle;
  }

  /* ---- Yaw指令wrap ---- */
  gimbal_ctrl_cmd->yaw = wrap180(gimbal_ctrl_cmd->yaw, last_yaw_cmd);

  /* ---- 小yaw偏移 (用于阈值判断, 通用最短弧) ---- */
  float slave_rel = CalcSlaveRelativeShortArc();
  float slave_abs = slave_rel > 0.0f ? slave_rel : -slave_rel;

  /* ====== 状态切换 ====== */
  DualYawState_e prev_state = gimbal->state;

  switch (gimbal->state) {
    case DUAL_YAW_NORMAL:
      if (slave_abs >= gimbal->enter_follow_threshold) {
        gimbal->state = DUAL_YAW_FOLLOW;
        SwitchToFollowPID();
      }
      break;

    case DUAL_YAW_FOLLOW:
      if (slave_abs >= gimbal->lock_threshold) {
        gimbal->state = DUAL_YAW_LOCKED;
        // 冻结锁定时真实陀螺的角度作为大yaw目标
        gimbal->virtual_gyro_lock_target = gimbal->gimbal_IMU_data->YawTotalAngle;
      } else if (slave_abs < gimbal->exit_follow_threshold) {
        gimbal->state = DUAL_YAW_NORMAL;
        // 记录新的保持值
        gimbal->virtual_gyro_hold_angle = gimbal->virtual_gyro_angle;
        SwitchToNormalPID();
      }
      break;

    case DUAL_YAW_LOCKED: {
      float lock_err = gimbal->virtual_gyro_angle -
                       gimbal->virtual_gyro_lock_target;
      if (lock_err < 0.0f) lock_err = -lock_err;

      if (lock_err < gimbal->recover_threshold) {
        gimbal->state = DUAL_YAW_NORMAL;
        gimbal->virtual_gyro_hold_angle = gimbal->virtual_gyro_angle;
        SwitchToNormalPID();
        gimbal_ctrl_cmd->yaw = gimbal->gimbal_IMU_data->YawTotalAngle;
      }
    } break;

    default:
      break;
  }

  gimbal->state_last = prev_state;

  /* ====== 各状态控制输出 ====== */

  // Pitch
  DMMotorSetPIDRef(gimbal->pitch_motor, gimbal_ctrl_cmd->pitch);

  switch (gimbal->state) {
    case DUAL_YAW_NORMAL:
      // 小yaw: IMU精跟
      DJIMotorSetPIDRef(gimbal->yaw_slave_motor, gimbal_ctrl_cmd->yaw);
      // 大yaw: 虚拟陀螺保持
      DJIMotorSetPIDRef(gimbal->yaw_master_motor, gimbal->virtual_gyro_hold_angle);
      break;

    case DUAL_YAW_FOLLOW:
      // 小yaw: 继续IMU精跟
      DJIMotorSetPIDRef(gimbal->yaw_slave_motor, gimbal_ctrl_cmd->yaw);
      // 大yaw: 追真实陀螺 → error = θ_gyro - θ_virtual = θ_rel
      DJIMotorSetPIDRef(gimbal->yaw_master_motor,
                        gimbal->gimbal_IMU_data->YawTotalAngle);
      break;

    case DUAL_YAW_LOCKED:
      // 小yaw: 断电
      DJIMotorStop(gimbal->yaw_slave_motor);
      gimbal_ctrl_cmd->yaw = gimbal->gimbal_IMU_data->YawTotalAngle;
      // 大yaw: 追冻结的锁定目标
      DJIMotorSetPIDRef(gimbal->yaw_master_motor,
                        gimbal->virtual_gyro_lock_target);
      break;

    default:
      break;
  }

  /* ---- 前馈叠加 ---- */
  gimbal->yaw_master_motor->motor_controller.final_output +=
      gimbal->yaw_master_feedforward_current;
  gimbal->yaw_slave_motor->motor_controller.final_output +=
      gimbal->yaw_slave_feedforward_current;

  /* ---- 收尾 ---- */
  last_yaw_cmd = gimbal_ctrl_cmd->yaw;
  gimbal_mode_last = gimbal_ctrl_cmd->gimbal_mode;
}
