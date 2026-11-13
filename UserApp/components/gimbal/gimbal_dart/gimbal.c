/**
 * @file gimbal.c
 * @brief dart 云台(gimbal_dart)执行层实现: 发射架 yaw 的角速度指令积分 + 角度环串级速度环
 */
#include "gimbal.h"

#include "bsp_dwt.h"
#include "robot_config.h"
#include "user_lib.h"

/* yaw: 角速度指令积分成角度目标, 角度环串级速度环输出 */
static void ApplyYaw(GimbalDartInstance* inst, float dt_s) {
  inst->yaw_angle_target += inst->yaw_rate_cmd_dps * dt_s;
  // 超前限幅防目标跑飞, 软限位限制机械行程
  float cur = inst->yaw_motor->measure.total_angle;
  VAL_LIMIT(inst->yaw_angle_target, cur - DART_YAW_LEAD_LIMIT_DEG, cur + DART_YAW_LEAD_LIMIT_DEG);
  VAL_LIMIT(inst->yaw_angle_target, inst->yaw_boot_angle - DART_YAW_SOFT_LIMIT_DEG,
            inst->yaw_boot_angle + DART_YAW_SOFT_LIMIT_DEG);
  DJIMotorOuterLoop(inst->yaw_motor, ANGLE_LOOP);
  DJIMotorSetPIDRef(inst->yaw_motor, inst->yaw_angle_target);
}

GimbalDartInstance* GimbalDartInit(void) {
  GimbalDartInstance* inst = (GimbalDartInstance*)zmalloc(sizeof(GimbalDartInstance));

  Motor_Init_Config_s yaw_conf = DART_YAW_MOTOR_CONFIG(DART_YAW_CAN, DART_YAW_CAN_ID);
  inst->yaw_motor = DJIMotorInit(&yaw_conf);

  inst->yaw_boot_angle = inst->yaw_motor->measure.total_angle;
  inst->yaw_angle_target = inst->yaw_boot_angle;
  inst->yaw_rate_cmd_dps = 0.0f;
  inst->enabled = false;
  inst->last_ms = (uint32_t)DWT_GetTimeline_ms();
  return inst;
}

void GimbalDartSetEnable(GimbalDartInstance* inst, bool enable) {
  if (inst == NULL) return;

  if (enable && !inst->enabled) {
    inst->enabled = true;
    DJIMotorEnable(inst->yaw_motor);
    // 重同步(实车教训): 失能期间机构可能被手推动, 重新使能时把 yaw 目标同步到当前位置并清 PID,
    // 避免使能瞬间输出跳变; 正常档位切换(持续使能)不做清零, 否则积分清零会丢保持力矩引起抽动
    inst->yaw_angle_target = inst->yaw_motor->measure.total_angle;
    PIDClear(&inst->yaw_motor->motor_controller.angle_PID);
    PIDClear(&inst->yaw_motor->motor_controller.speed_PID);
  } else if (!enable && inst->enabled) {
    inst->enabled = false;
    DJIMotorStop(inst->yaw_motor);
  }
}

void GimbalDartSetYawRate(GimbalDartInstance* inst, float rate_dps) {
  if (inst == NULL) return;
  inst->yaw_rate_cmd_dps = rate_dps;
}

void GimbalDartStop(GimbalDartInstance* inst) {
  if (inst == NULL) return;
  // 与失能一致: 置 enabled=false 并断输出。恢复必须走 GimbalDartSetEnable(true),
  // 由它重新使能电机/重同步目标/清 PID(电机 stop_flag 是锁存的, 只 SetPIDRef 不会恢复输出)
  inst->enabled = false;
  DJIMotorStop(inst->yaw_motor);
}

void GimbalDartTask(GimbalDartInstance* inst) {
  if (inst == NULL || !inst->enabled) return;

  uint32_t now = (uint32_t)DWT_GetTimeline_ms();
  float dt_s = (float)(now - inst->last_ms) * 0.001f;
  if (dt_s <= 0.0f || dt_s > 0.05f) dt_s = DART_TASK_DT_S;
  inst->last_ms = now;

  ApplyYaw(inst, dt_s);
}
