/**
 * @file gimbal.h
 * @brief dart 云台(gimbal_dart): 发射架 yaw 轴控制接口
 *
 * yaw 控制方式(与拆分前一致):
 *   - 上层只给**角速度指令**(DART_YAW_SENSITIVITY_DPS 缩放), 本模块按任务周期积分成角度目标;
 *   - 角度目标再做"超前限幅 + 软限位", 最后角度环串级速度环输出。
 *
 * 设计要点:
 *   - 失能即停机; 重新使能时把目标重同步到当前角度并清 PID, 防失能期间机构被手推动后输出跳变;
 *   - 故障(FAULT)时由 RobotTask 调用 GimbalDartStop() 断力矩: yaw 无弹簧负载, 不需要像同步带那样保持力矩。
 */
#pragma once

#include "dji_motor.h"
#include <stdbool.h>
#include <stdint.h>

/* 云台(yaw)实例 */
typedef struct {
  DJIMotorInstance* yaw_motor;  // M2006 发射架 yaw

  float yaw_boot_angle;    // yaw 开机角度(软限位基准)
  float yaw_angle_target;  // yaw 目标角 (deg, 相对开机)
  float yaw_rate_cmd_dps;  // yaw 角速度指令 (deg/s), 由任务积分成角度目标
  bool enabled;            // 遥控器使能标志(失能即停机)
  uint32_t last_ms;        // 上次任务时间戳, 用于角速度指令积分
} GimbalDartInstance;

/**
 * @brief 初始化云台: 注册 yaw 电机, 记录开机角度并复位目标
 */
GimbalDartInstance* GimbalDartInit(void);

/**
 * @brief 使能/失能云台; 失能立即停机, 重新使能时重同步 yaw 目标并清 PID
 */
void GimbalDartSetEnable(GimbalDartInstance* inst, bool enable);

/**
 * @brief 设置 yaw 角速度指令(deg/s), 由任务积分成角度目标
 */
void GimbalDartSetYawRate(GimbalDartInstance* inst, float rate_dps);

/**
 * @brief 立即断 yaw 输出并置失能(故障用), 不动其他执行器;
 *        恢复必须再调用 GimbalDartSetEnable(inst, true)(重新使能电机/重同步目标/清 PID)
 */
void GimbalDartStop(GimbalDartInstance* inst);

/**
 * @brief 云台周期任务, 由 RobotTask 以 ~1kHz 调用
 */
void GimbalDartTask(GimbalDartInstance* inst);
