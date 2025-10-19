#include <string.h>


#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "FS-iA10B.hpp"
#include "button.h"
#include "buzzer.h"
#include "tick.h"
#include "ws2812.h"
#include "sample.h"
#include "MotorDriver.hpp"

using namespace std;
using namespace motor_control;

#define RX_BUF_SIZE 25

extern "C" {
static uint8_t rxBuf[RX_BUF_SIZE]; // 全局接收机缓冲区
static SBUS::Receiver sbusReceiver;

uint8_t motorOn = 0;
uint8_t lastButtonState = 0;
uint8_t globalErr=1;

static CANMotorM3508* motor1 = nullptr;
static CANMotorGM6020* motor2 = nullptr;


// volatile int16_t targetDegree = 90; //初始的
// volatile uint16_t servoDegree = 90;
// volatile uint8_t music = 0;
// volatile uint8_t musicLast = 0;


unsigned long tick = 0;
unsigned long lastFsTick = 0;
unsigned long lastlastFsTick = 0;
int differtTickHz = 0; //1000/lastFsTick-lastlastFsTick

/*--------------------------------工具--------------------------------*/

uint8_t readButton(void)
{
    return !HAL_GPIO_ReadPin(BTN_0_GPIO_Port, BTN_0_Pin);
}

int16_t map(int16_t a, int16_t b, int16_t A, int16_t B, int16_t target)
{
    if (a == b)
    {
        return A;
    }
    int32_t result = (int32_t)(target - a) * (B - A);
    result /= (b - a);
    result += A;
    if (result > 0xFFFF) result = 0xFFFF;
    return (int16_t)result;
}

/*--------------------------------回调--------------------------------*/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        uint8_t verifyFlag = SBUS::Receiver::verifyBuffer(rxBuf, Size);
        if (verifyFlag == 0)
        {
            sbusReceiver.getChannels(rxBuf, Size);
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);

            lastlastFsTick = lastFsTick; //正确时更新tick
            lastFsTick = tick;
        }
        else
        {
            sbusReceiver.sendError(&huart1, verifyFlag); //错误-串口回传
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
        }
    }

    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxBuf, RX_BUF_SIZE);
} //接收机回调+校验+报错

// void servoCallback(TIM_HandleTypeDef* htim)
// {
//     if (htim->Instance == TIM15)
//     {
//         if (targetDegree < 0) targetDegree = 0;
//         if (targetDegree > 180) targetDegree = 180; //防止溢出

//         int16_t gap = 500 + (targetDegree * (2000) / 180);
//         __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, gap);
//     }
// }

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
// }
//
//已经开启了Advanced settig - EXTI 使用原来的程序记得关掉

//思路:10-15外部中断线组触发器->EXTI_HandleTypeDef->PendingCallback


/*--------------------------------其他--------------------------------*/


// void Servo_SetDegree(int16_t deg)
// {
//     targetDegree = deg;
// }

// void SERVO_Init()
// {
//     HAL_TIM_RegisterCallback(&htim15, HAL_TIM_PERIOD_ELAPSED_CB_ID, servoCallback);

//     HAL_TIM_PWM_Start_IT(&htim15, TIM_CHANNEL_1);
//     HAL_TIM_Base_Start_IT(&htim15);
// }

/*-==================EXTI Callback==================================*/


    void gpioCallback(void)
{
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
}


    EXTI_HandleTypeDef hexti14;

    void EXTI14_Init(void)
    {
        EXTI_ConfigTypeDef exti_config = {};

        HAL_EXTI_GetHandle(&hexti14, EXTI_LINE_14);//绑定hexiti14->Line

        // 配置 EXTI14
        exti_config.Line    = EXTI_LINE_14;
        exti_config.Mode    = EXTI_MODE_INTERRUPT;
        exti_config.Trigger = EXTI_TRIGGER_RISING_FALLING; // 上升沿 + 下降沿
        exti_config.GPIOSel = EXTI_GPIOC;                  // PC14
        HAL_EXTI_SetConfigLine(&hexti14, &exti_config);

        // 注册回调函数
        HAL_EXTI_RegisterCallback(&hexti14, HAL_EXTI_COMMON_CB_ID, gpioCallback);//绑定hexiti14->PendingCallback
    }





/*--------------------------------判断任务--------------------------------*/

// void ifMotorOnDisplay()
// {
//     if (readButton() != lastButtonState && readButton() == 0)
//     {
//         motorOn = !motorOn;
//         HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
//     }

//     if (motorOn)
//     {
//         if (!globalErr)
//         {
//         // voltage = map(240, 1807, -2000, 2000, sbusReceiver.ch3_Rcol);
//         }
//     }
//     else
//     {
//         // voltage = 0;
//     }


//     lastButtonState = readButton();
// }

// void ifMusicDisplay()
// {

//     if (sbusReceiver.ch9_SwitchC > 250 && sbusReceiver.ch9_SwitchC < 1700)
//     {
//         music = 1;
//     }
//     else if (sbusReceiver.ch9_SwitchC > 1800)
//     {
//         music = 2;
//     }
//     else
//     {
//         music = 0;
//     }

//     if (musicLast != music)
//     {
//         musicLast=music;
//         deleteAllNotes();
//         if (music==1)
//         {
//             playMusicSample(1);
//         }else if (music==2)
//         {
//             playMusicSample(2);
//         }
//     }else
//     {
//         return;
//     }
// }

// void ifWS2812Display()
// {
//     if (sbusReceiver.ch7_SwitchA > 250)
//     {
//         WS2812_Demo(tick, 200);
//     }
//     else
//     {
//         blankAll();
//     }
// }

// void ifServoDisplay()
// {
//     if (sbusReceiver.ch8_SwitchB > 250)
//         servoDegree = map(240, 1807, 0, 180, sbusReceiver.ch4_Rrow);
//     Servo_SetDegree(servoDegree);
// }

void ifErrRebootFs()
{
    if (tick - lastFsTick > 500)
    {
        globalErr=1;

        sbusReceiver.clearAll(); //清空
        // voltage=0;
        // memset(can_tx_buf, 0, 8);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); //不正常-PB7灭
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxBuf, RX_BUF_SIZE);
    }
    else
    {
        globalErr=0;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); //正常-亮PB7
    }
}

/*--------------------------------电机控制--------------------------------*/

void Motor_Init(void)
{
    motor1 = new CANMotorM3508(1);
    motor2 = new CANMotorGM6020(5);

    CANMotorInit(&hfdcan1);
    
    motor1->setSpeedPID(0.2f, 0.0f, 0.001f);
    motor1->setSpeedControl();

    motor2->setSpeedPID(0.2f, 0.0f, 0.001f);
    motor2->setSpeedControl();

    // motor1->setOpenLoop();
    // motor1->setOpenLoopCurrent(0.5f);
    
}

void Motor_Update(void)
{
    if (motor1 != nullptr)
    {
        motor1->update();
    }
}


/*--------------------------------信号生成--------------------------------*/
float generateSineWave(unsigned long tick_ms){
    float frequency = 0.5f; // 频率，单位为Hz 周期为2秒
    float amplitude = 150.0f; // 振幅
    float time = tick_ms / 1000.0f;
    float value = amplitude * sinf(2.0f * 3.14159265f * frequency * time);
    return value;
}

float generateSquareWave(unsigned long tick_ms){
    float frequency = 0.5f; // 频率，单位为Hz 周期为2秒
    float amplitude = 10.0f; // 振幅
    float time = tick_ms / 1000.0f;
    float value = (sinf(2.0f * 3.14159265f * frequency * time) > 0) ? amplitude : -amplitude;
    return value;
}

/*--------------------------------main--------------------------------*/

void mainTask(void)
{
    Tick_Init();
    // WS2812_Init();
    // Buzzer_Init();
    // SERVO_Init();
    // sbusReceiver.FS_Init(rxBuf, RX_BUF_SIZE); //接收机启动

    EXTI14_Init();//PC14灯
    
    // 初始化电机
    Motor_Init();

    while (1)
    {
        differtTickHz = 1000 / (lastFsTick - lastlastFsTick); //接收机成功接收Hz

        motor1->setSpeed(generateSquareWave(tick));
        motor2->setSpeed(generateSineWave(tick));


        // 电机控制循环
        updateAllCAN(globalMotorList, MAX_MOTOR_NUM, &hfdcan1);
        // ifMotorOnDisplay();

        // ifServoDisplay(); //不触发一直是90度

        // ifWS2812Display();

        // ifMusicDisplay();

        // sbusReceiver.debugPrint(&huart3, tick, globalErr,2000); //遥控器数据回报,2000ms发一次
        // ifErrRebootFs(); //如果超时重连遥控器,500ms //正常亮PB7,不正常灭 //不正常清空sbusReceiver
        HAL_Delay(1);
        tick=HAL_GetTick();
    }
}
}
