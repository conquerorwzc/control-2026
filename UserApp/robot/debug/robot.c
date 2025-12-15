#include "robot.h"

#include "HI05.h"
#include "main.h"
#include "master_process.h"
#include "usart.h"

static GPIOInstance *gpio_5V_EN;
static GPIO_Init_Config_s gpio_init_config_5v = {
    .GPIO_Pin = POWER_5V_Pin,
    .GPIOx = POWER_5V_GPIO_Port,
    .pin_state = GPIO_PIN_SET,
};
static HI05_t *hi05;

void RobotInit() {
  gpio_5V_EN = GPIORegister(&gpio_init_config_5v);
  GPIOSet(gpio_5V_EN);
  hi05 = HI05_Init(&huart10);
}

void RobotTask() {}
