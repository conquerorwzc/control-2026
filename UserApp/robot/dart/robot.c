//
// Created by PC on 2025/11/18.
//
#include "robot.h"
#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"
#include "dji_motor.h"

RobotInstance* robot = NULL;
static DartInstance* dart = NULL;
static RC_ctrl_t* rc_data = NULL;
static RC_ctrl_t* rc_data_last = NULL;

#define DART_FRICTION_MAX_SPEED   16000.0f
#define DART_PUSHROD_MAX_SPEED    30000.0f
#define RC_DEADZONE               50

bool IsInDeadzone(int16_t value) {
  return (abs(value) < RC_DEADZONE);
}    //死区检测

float MapStickToSpeed(int16_t stick_value, float max_speed) {
  if (IsInDeadzone(stick_value)) {
    return 0.0f;
  }
  return (stick_value / 660.0f) * max_speed;
}     //映射摇杆值到电机速度

void DartInit(void) {
  // 内存分配与清零: zmalloc 在 user_lib.c 中定义，它会自动执行 memset 清零
  dart = (DartInstance*)zmalloc(sizeof(DartInstance));
  memset(dart, 0, sizeof(DartInstance));

  // 初始化 Yaw 轴电机 (GM6020)
  Motor_Init_Config_s yaw_conf = YAW_MOTOR_CONFIG(&hcan2, YAW_MOTOR_ID);
  dart->yaw_motor = DJIMotorInit(&yaw_conf);

  // 初始化 垂直推杆 (M2006)
  // 这里的PID配置包含了 PID_ErrorHandle，用于校准时的堵转检测
  Motor_Init_Config_s vert_conf = PUSH_ROD_CONFIG(&hcan1, PUSH_VERT_ID, MOTOR_DIRECTION_NORMAL);
  dart->vertical_pushrod = DJIMotorInit(&vert_conf);

  // 初始化 水平推杆 (M2006)
      Motor_Init_Config_s hori_conf = PUSH_ROD_CONFIG(&hcan1, PUSH_HORI_ID, MOTOR_DIRECTION_REVERSE);
  dart->horizontal_pushrod = DJIMotorInit(&hori_conf);

  // 初始化 摩擦轮 (M3508 x4): 左边: LU(1), LD(2) | 右边: RU(3), RD(4)

  // 1. 左上 (Left Up) - ID 1
  Motor_Init_Config_s fric_lu_conf = FRIC_MOTOR_CONFIG(&hcan1, FRIC_MOTOR_LU, MOTOR_DIRECTION_NORMAL);
  dart->friction_motor[0] = DJIMotorInit(&fric_lu_conf);

  // 2. 左下 (Left Down) - ID 2
  Motor_Init_Config_s fric_ld_conf = FRIC_MOTOR_CONFIG(&hcan1, FRIC_MOTOR_LD, MOTOR_DIRECTION_NORMAL);
  dart->friction_motor[1] = DJIMotorInit(&fric_ld_conf);

  // 3. 右上 (Right Up) - ID 3 反转
  Motor_Init_Config_s fric_ru_conf = FRIC_MOTOR_CONFIG(&hcan1, FRIC_MOTOR_RU, MOTOR_DIRECTION_REVERSE);
  dart->friction_motor[2] = DJIMotorInit(&fric_ru_conf);

  // 4. 右下 (Right Down) - ID 4 反转
  Motor_Init_Config_s fric_rd_conf = FRIC_MOTOR_CONFIG(&hcan1, FRIC_MOTOR_RD, MOTOR_DIRECTION_REVERSE);
  dart->friction_motor[3] = DJIMotorInit(&fric_rd_conf);

  /* ================= 状态机初始值 ================= */
  dart->current_mode = DART_MODE_DEBUG;      // 上电默认调试，防误触
  dart->calibration_step = CALI_STEP_IDLE;   // 校准闲置
  dart->is_calibrated = false;               // 未校准

  // 初始命令清零
  dart->dart_ctrl_cmd.fire_command = false;
  dart->dart_ctrl_cmd.friction_speed = 0;
}

void RobotInit(void) {
  // 1. 全局实例分配
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));

  // 2. 初始化遥控器
  rc_data = RemoteControlInit(&huart3);
  robot->rc_data = rc_data;

  // 3. 初始化飞镖系统
  DartInit();
  robot->dart = dart; // 关联指针
  robot->robot_mode = ROBOT_POWER_ON;
}

/* 检查电机是否堵转 (用于校准) */
static bool IsMotorBlocked(DJIMotorInstance *motor) {
  if (motor == NULL) return false;
  // 读取底层PID错误状态
  if (motor->motor_controller.speed_PID.ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR) {
    // 发生堵转后清除错误标志，防止死锁
    motor->motor_controller.speed_PID.ERRORHandler.ERRORType = PID_ERROR_NONE;
    motor->motor_controller.speed_PID.ERRORHandler.ERRORCount = 0;
    return true;
  }
  return false;
}

/* 状态机逻辑 */
void DartStateMachineUpdate(void) {
  if (robot->robot_mode == ROBOT_EMERGENCY_STOP) {
    dart->current_mode = DART_MODE_EMERGENCY_STOP;
    return;
  }

  // 遥控器右侧拨杆 (switch_right) 控制主模式
  // 下档：调试模式 (DEBUG)
  if (switch_is_down(rc_data->rc.switch_right)) {
    dart->current_mode = DART_MODE_DEBUG;
    dart->calibration_step = CALI_STEP_IDLE;
    dart->is_calibrated = false;
  }
  // 中档：自动/就绪模式
  else if (switch_is_mid(rc_data->rc.switch_right)) {
    if (!dart->is_calibrated) {
      // 未校准则强制进入校准模式
      dart->current_mode = DART_MODE_CALIBRATING;
    } else {
      // 已校准则进入就绪状态
      dart->current_mode = DART_MODE_AUTO_READY;
    }
  }
  // 上档：自动打弹
  else if (switch_is_up(rc_data->rc.switch_right)) {
    if (dart->is_calibrated) {
      dart->current_mode = DART_MODE_AUTO_FIRE; // 只有校准过才允许自动打弹
    } else {
      dart->current_mode = DART_MODE_CALIBRATING; // 否则强制去校准
    }
  }
}

/* 调试模式处理(纯手动控制，用于测试电机好坏和方向) */
static float yaw_current_target = 0.0f;
/* 调试模式： */
void DartDebugModeHandler(void) {

  //  if (rc_data == NULL) return;// 检查指针有效性 (防止空指针)
  int16_t stick_val = rc_data->rc.rocker_l_;// 强制类型转换，确保浮点运算
  // --- 1. Yaw 轴 (左摇杆左右 rocker_l_) ---
  // 使用“增量式”位置控制：推杆增加角度，回中保持角度
  float angle_inc = ((float)stick_val / 66.0f) * 0.2f; // 0.5 是灵敏度，可调
  yaw_current_target -= angle_inc;
  float current_angle = dart->yaw_motor->measure.total_angle;
  float max_lead_angle = 15.0f;
  if (yaw_current_target > current_angle + max_lead_angle) {
    yaw_current_target = current_angle + max_lead_angle;
  }
  else if (yaw_current_target < current_angle - max_lead_angle) {
    yaw_current_target = current_angle - max_lead_angle;
  }
  VAL_LIMIT(yaw_current_target, -5000.0f, 5000.0f);
  DJIMotorOuterLoop(dart->yaw_motor, ANGLE_LOOP);
  DJIMotorSetPIDRef(dart->yaw_motor, yaw_current_target);


  // --- 2. 垂直推杆 (左摇杆上下 rocker_l1) ---
  // 速度环控制
  float vert_speed = MapStickToSpeed(rc_data->rc.rocker_l1, DART_PUSHROD_MAX_SPEED);

  DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
  DJIMotorSetPIDRef(dart->vertical_pushrod, vert_speed);


  // --- 3. 水平推杆 (右摇杆左右 rocker_r_) ---
  // 速度环控制
  float hori_speed = MapStickToSpeed(rc_data->rc.rocker_r_, DART_PUSHROD_MAX_SPEED);

  DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
  DJIMotorSetPIDRef(dart->horizontal_pushrod, hori_speed);


  // --- 4. 摩擦轮 (右摇杆上下 rocker_r1) ---
  // 速度环控制，四个轮子同步
  float fric_speed = MapStickToSpeed(rc_data->rc.rocker_r1, DART_FRICTION_MAX_SPEED);

  for(int i=0; i<4; i++) {
    DJIMotorOuterLoop(dart->friction_motor[i], SPEED_LOOP);
    DJIMotorSetPIDRef(dart->friction_motor[i], fric_speed);
  }
}


/* 校准模式处理 */
static void DartCalibrationHandler(void) {
  switch (dart->calibration_step) {
    case CALI_STEP_IDLE:
      // 启动校准，先从垂直推杆开始
      dart->calibration_step = CALI_STEP_VERT_PUSH;
      break;

    // --- 阶段1: 垂直推杆归零 ---
    case CALI_STEP_VERT_PUSH:
      dart->vertical_pushrod->motor_controller.speed_PID.MaxOut = 1000.0f;
      dart->vertical_pushrod->motor_controller.speed_PID.IntegralLimit = 500.0f;
      DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
      DJIMotorSetPIDRef(dart->vertical_pushrod, -CALI_SPEED);

      // 检测堵转
      if (IsMotorBlocked(dart->vertical_pushrod)) {
        // 记录当前编码器值作为零点参考
        dart->vert_zero_ecd = dart->vertical_pushrod->measure.total_angle;
        dart->vertical_pushrod->motor_controller.speed_PID.ITerm = 0;
        dart->vertical_pushrod->motor_controller.speed_PID.Output = 0;
        dart->vertical_pushrod->motor_controller.speed_PID.Iout = 0;
        dart->calibration_step = CALI_STEP_VERT_BACK;
        dart->vertical_pushrod->motor_controller.speed_PID.MaxOut = 10000.0f;
        dart->vertical_pushrod->motor_controller.speed_PID.IntegralLimit = 3000.0f;
      }
      break;

    case CALI_STEP_VERT_BACK:
      // 位置环回退，消除机械形变
      DJIMotorOuterLoop(dart->vertical_pushrod, ANGLE_LOOP);
      // 目标位置 = 撞墙位置 + 回退距离
      DJIMotorSetPIDRef(dart->vertical_pushrod, dart->vert_zero_ecd + CALI_BACK_OFFSET);

      // 判断是否到达回退位置 (误差 < 阈值)
      if (abs(dart->vertical_pushrod->measure.total_angle - (dart->vert_zero_ecd + CALI_BACK_OFFSET)) < CALI_POS_THRESHOLD) {
        // 垂直校准完成，电机停转
        DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->vertical_pushrod, 0);
        // 进入下一阶段：水平推杆
        dart->calibration_step = CALI_STEP_HORI_PUSH;
      }
      break;

    // --- 阶段2: 水平推杆归零 (逻辑同上) ---
    case CALI_STEP_HORI_PUSH:
      dart->horizontal_pushrod->motor_controller.speed_PID.MaxOut = 1000.0f;
      dart->horizontal_pushrod->motor_controller.speed_PID.IntegralLimit = 500.0f;

      DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
      DJIMotorSetPIDRef(dart->horizontal_pushrod, -CALI_SPEED); // 确认方向是否正确

      if (IsMotorBlocked(dart->horizontal_pushrod)) {
        // 记录零点
        dart->hori_zero_ecd = dart->horizontal_pushrod->measure.total_angle;

        // 【新增】清空 PID 积分，防止回退时暴冲
        dart->horizontal_pushrod->motor_controller.speed_PID.ITerm = 0;
        dart->horizontal_pushrod->motor_controller.speed_PID.Output = 0;
        dart->horizontal_pushrod->motor_controller.speed_PID.Iout = 0;

        // 【新增】恢复 PID 满血状态
        dart->horizontal_pushrod->motor_controller.speed_PID.MaxOut = 10000.0f;
        dart->horizontal_pushrod->motor_controller.speed_PID.IntegralLimit = 3000.0f;

        dart->calibration_step = CALI_STEP_HORI_BACK;
      }
      break;

    case CALI_STEP_HORI_BACK:
      DJIMotorOuterLoop(dart->horizontal_pushrod, ANGLE_LOOP);
      DJIMotorSetPIDRef(dart->horizontal_pushrod, dart->hori_zero_ecd + CALI_BACK_OFFSET);

      if (abs(dart->horizontal_pushrod->measure.total_angle - (dart->hori_zero_ecd + CALI_BACK_OFFSET)) < CALI_POS_THRESHOLD) {
        DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
        DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);
        dart->calibration_step = CALI_STEP_DONE;
      }
      break;

    case CALI_STEP_DONE:
      dart->is_calibrated = true;
      // 校准完成，自动切回 Ready 模式
      dart->current_mode = DART_MODE_AUTO_READY;
      break;
  }
}

/* 机器人主任务 */
void RobotTask(void) {
  // 1. 刷新状态机
  DartStateMachineUpdate();

  // 2. 执行对应模式的逻辑
  switch (dart->current_mode) {
    case DART_MODE_DEBUG:
      DartDebugModeHandler();
      break;

    case DART_MODE_CALIBRATING:
      DartCalibrationHandler();
      break;

    case DART_MODE_AUTO_READY:
      // 就绪状态：锁死位置，等待发射指令
      // 这里简单处理：给 0 速度保持静止 (或者写位置环锁住当前位置)
      DJIMotorOuterLoop(dart->vertical_pushrod, SPEED_LOOP);
      DJIMotorSetPIDRef(dart->vertical_pushrod, 0);
      DJIMotorOuterLoop(dart->horizontal_pushrod, SPEED_LOOP);
      DJIMotorSetPIDRef(dart->horizontal_pushrod, 0);
      // 摩擦轮保持怠速或停止
      for(int i=0; i<4; i++) DJIMotorSetPIDRef(dart->friction_motor[i], 0);
      break;

    case DART_MODE_EMERGENCY_STOP:
      // 急停：所有电机断电 (发送 0 电流)
      DJIMotorStop(dart->yaw_motor);
      DJIMotorStop(dart->vertical_pushrod);
      DJIMotorStop(dart->horizontal_pushrod);
      for(int i=0; i<4; i++) DJIMotorStop(dart->friction_motor[i]);
      break;

    default:
      break;
     }

  // 3. 底层发送：将计算好的 PID 输出发送给电调
  DJIMotorTask();
}