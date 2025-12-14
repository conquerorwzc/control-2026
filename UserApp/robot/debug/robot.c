#include "robot.h"

// #include "HI05.h"
#include "dji_motor.h"
#include "general_def.h"
#include "master_process.h"
#include "navigator.h"
#include "robot_config.h"
#include "super_cap.h"
#include "user_lib.h"

static SuperCapInstance* supercap_instance;
// static HI05_t* hi05_instance;  // 保存HI05实例指针

static SuperCap_Init_Config_s supercab_init_config = {
  .can_config = {
    .can_handle = &hcan1,  // 根据实际情况选择CAN接口
    .rx_id = 0x211,        // 接收ID (CAN_SUPERCAP_ID)
    .tx_id = 0X210,        // 发送ID
  }
};

void RobotInit() {
  supercap_instance = SuperCapInit(&supercab_init_config);
  // hi05_instance = HI05_Init(&huart1);


}

void RobotTask() {
  // osDelay(300);

  int16_t power = 10;
  uint16_t buffer = 500;
  uint8_t state = 1;

  SuperCapSendMessage(supercap_instance, power, buffer, state);

  // // 更新IMU运动加速度（在主循环中调用，避免在中断中进行大量浮点运算）
  // if (hi05_instance != NULL) {
  //   HI05_UpdateMotionAccel(hi05_instance);
  // }
}
