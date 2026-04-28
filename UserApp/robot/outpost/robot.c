//
// Created by PC on 2025/12/22.
//
#include "robot.h"
#include "robot_config.h"
#include "user_lib.h"

RobotInstance* robot = NULL;

void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));
  robot->rc_data = RemoteControlInit(&huart3);

  // 初始化前哨站电机
  Motor_Init_Config_s outpost_conf = OUTPOST_3508_CONFIG(&hcan1, OUTPOST_MOTOR_ID, MOTOR_DIRECTION_NORMAL);
  robot->outpost_motor = DJIMotorInit(&outpost_conf);

  // 初始化状态
  robot->is_first_loop = true;
  robot->is_locked = false;
  robot->lock_angle = 0.0f;
}

void RobotTask(void) {
  if (robot->outpost_motor == NULL || robot->rc_data == NULL) return;

  // ================= 1. 上电反馈检查 (防疯转) =================
  if (robot->is_first_loop) {
    // 确认已收到电机数据（rx_id 不为 0 且 ecd 有值）
    if (robot->outpost_motor->motor_can_instance->rx_id != 0 && robot->outpost_motor->measure.ecd != 0) {
      // 记录初始角度，并将控制目标设为当前角度，防止切环时跳变
      robot->lock_angle = robot->outpost_motor->measure.total_angle;
      robot->outpost_motor->motor_controller.pid_ref = robot->lock_angle;

      robot->is_first_loop = false;
      robot->is_locked = true;
    } else {
      // 没收到反馈前不执行 PID 计算
      DJIMotorTask();
      return;
    }
  }

  // ================= 2. 拨杆控制逻辑 =================
  if (switch_is_mid(robot->rc_data->rc.switch_left)) {
    // 【中档】：0.4转恒速旋转

    robot->is_locked = false; // 解除锁定标记

    DJIMotorOuterLoop(robot->outpost_motor, SPEED_LOOP);
    DJIMotorSetPIDRef(robot->outpost_motor, TARGET_SPEED);
  }
  else {
    // 【上/下档】：角度环锁死

    // 状态切换瞬间：捕获当前绝对位置
    if (robot->is_locked == false) {
      robot->lock_angle = robot->outpost_motor->measure.total_angle;
      robot->is_locked = true;
    }

    DJIMotorOuterLoop(robot->outpost_motor, ANGLE_LOOP);
    DJIMotorSetPIDRef(robot->outpost_motor, robot->lock_angle);
  }

  // 发送控制指令
  DJIMotorTask();
}