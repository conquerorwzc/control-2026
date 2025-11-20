#include "motor_task.h"

#include "dji_motor.h"
#include "servo_motor.h"
void MotorControlTask() {
  DJIMotorTask();

  /* 如果有对应的电机则取消注释,可以加入条件编译或者register对应的idx判断是否注册了电机 */
}