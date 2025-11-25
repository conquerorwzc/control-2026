#include "robot.h"

#include "dji_motor.h"
#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"
#include "navigator.h"

static DJIMotorInstance* motor_instance;

void RobotInit() {
  wheel_motor_config.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
  wheel_motor_config.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
  wheel_motor_config.controller_setting_init_config.outer_loop_type = SPEED_LOOP;
  wheel_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP;
  motor_instance = DJIMotorInit(&wheel_motor_config);
}

void RobotTask() {
  uint8_t custom_data[] = {0x40, 0x50, 0x60, 0x70}; //随便给的测试数据，牢恩你改一下啦
  uint32_t system_tick=0xAAAA;  //随便给的值，时间戳后面再说啦
  uint8_t data_id=0x01;
  // uint8_t simple_data = 0xBB; // 只发送一个字节
  // uint32_t system_tick = 0x12345678;
  // uint8_t data_id = 0x91;
  DJIMotorSetPIDRef(motor_instance, 400.0f);
  uint8_t status = protocol_send(
                                  &huart1,         // UART句柄
                                  system_tick, // 时间戳
                                  custom_data,       // 数据指针
                                  sizeof(custom_data), // 数据长度
                                  data_id,           // 数据段ID
                                  HAL_MAX_DELAY
                              );
  if (!status)
  {

  }
  osDelay(100);
}
