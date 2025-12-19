#include "robot.h"
#include "robot_config.h"
#include "user_lib.h"

RobotInstance *robot;

// 辅助限幅函数
static void LimitTarget(float *val, float min, float max) {
  if (*val > max) *val = max;
  if (*val < min) *val = min;
}

void RobotInit(void) {
  // 1. 内存清零
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

  // 2. 初始化遥控器 (直接调，防止宏定义坑)
  robot->rc = RemoteControlInit(&huart3);

  // 3. 初始化电机 (直接用宏，简单粗暴)
  // Yaw (ID 3)
  Motor_Init_Config_s yaw_conf = YAW_CONFIG(&hcan1, YAW_ID);
  robot->yaw_motor = DJIMotorInit(&yaw_conf);

  // Pitch (ID 5)
  Motor_Init_Config_s pitch_conf = PITCH_CONFIG(&hcan1, PITCH_ID);
  robot->pitch_motor = DJIMotorInit(&pitch_conf);

  // 摩擦轮 (左ID1, 右ID2)
  Motor_Init_Config_s fric_l_conf = SHOOT_MOTOR_CONFIG(&hcan1, FRIC_L_ID, MOTOR_DIRECTION_NORMAL);
  robot->fric_l = DJIMotorInit(&fric_l_conf);
  Motor_Init_Config_s fric_r_conf = SHOOT_MOTOR_CONFIG(&hcan1, FRIC_R_ID, MOTOR_DIRECTION_REVERSE);
  robot->fric_r = DJIMotorInit(&fric_r_conf);

  // 4. 初始化目标角度为校准值
  // 必须是这个值，不然上电 PID error 巨大，直接起飞
  robot->target_yaw = YAW_INIT_ANGLE;     // 118.69
  robot->target_pitch = PITCH_INIT_ANGLE; // 98.88

  robot->mode = ROBOT_STOP;
}

static void RobotControlLogic(void) {
  if (robot->rc == NULL) return;

  // [急停逻辑] 右拨杆在下 -> 停止
  if (switch_is_down(robot->rc->rc.switch_right)) {
    robot->mode = ROBOT_STOP;
    DJIMotorStop(robot->yaw_motor);
    DJIMotorStop(robot->pitch_motor);
    DJIMotorStop(robot->fric_l);
    DJIMotorStop(robot->fric_r);

    // 关键：急停恢复后，为了不让云台猛甩回初始位
    // 我们把目标重置为当前实际位置 (或者你希望重置回 INIT 也行，这里选安全策略)
    // 但既然你指定了 INIT 值，也可以每次重置回 INIT，看你习惯
    robot->target_yaw = YAW_INIT_ANGLE;
    robot->target_pitch = PITCH_INIT_ANGLE;
    return;
  }

  // [正常模式]
  robot->mode = ROBOT_RUNNING;
  DJIMotorEnable(robot->yaw_motor);
  DJIMotorEnable(robot->pitch_motor);
  DJIMotorEnable(robot->fric_l);
  DJIMotorEnable(robot->fric_r);

  // 1. Yaw 轴控制 (左摇杆左右)
  // 逻辑：你向左转(摇杆+)，ECD需要变小(去596/29.8度)，所以用减法
  robot->target_yaw -= 0.0005f * (float)robot->rc->rc.rocker_l_;
  // 限位：[29.80, 208.69]
  LimitTarget(&robot->target_yaw, YAW_MIN_ANGLE, YAW_MAX_ANGLE);

  // 2. Pitch 轴控制 (右摇杆上下)
  // 逻辑：你向上推(摇杆+)，需要抬头(去水平/98.88度/ECD变大)，所以用加法
  robot->target_pitch += 0.0002f * (float)robot->rc->rc.rocker_r1;
  // 限位：[54.93, 98.88]
  LimitTarget(&robot->target_pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);

  // 3. 应用目标到电机
  DJIMotorOuterLoop(robot->yaw_motor, ANGLE_LOOP);
  DJIMotorSetPIDRef(robot->yaw_motor, robot->target_yaw);

  DJIMotorOuterLoop(robot->pitch_motor, ANGLE_LOOP);
  DJIMotorSetPIDRef(robot->pitch_motor, robot->target_pitch);

  // 4. 摩擦轮 (右拨杆中/上 -> 开启)
  if (!switch_is_down(robot->rc->rc.switch_right)) {
    robot->target_fric_speed = 0.0f;
  } else {
    robot->target_fric_speed = 0.0f;
  }
  DJIMotorSetPIDRef(robot->fric_l, robot->target_fric_speed);
  DJIMotorSetPIDRef(robot->fric_r, robot->target_fric_speed);
}

void RobotTask(void) {
  RobotControlLogic();
  DJIMotorTask();
}