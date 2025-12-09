#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "main.h"
#include "tim.h"
#include <stdint-gcc.h>
#include "bsp_pwm.h"
#include "bsp_usart.h"

/**********************************************
    * @总线设备基本控制指令表：
    *	1.	#000PID!						//读取ID指令
    *	2.	#000PID001!						//设置ID指令
    *	3.	#000PVER!						//读取版本
    *	4.	#000PBD1!						//设置波特率 默认115200  1:9600 , 2:19200 , 3:38400 , 4:57600 , 5:115200 , 6:128000 7:256000  8:1000000
    *	5.	#000PCLE!						//恢复出厂设置包括ID
    * @总线舵机ZServo控制指令表：
    *	1.	#000P1500T1000!					//舵机角度控制
    *	2.	#000PDST!						//停止
    *	3.	#000PDPT!						//暂停
    *	4.	#000PDCT!						//继续
    *	5.	#000PCSD!						//设置当前值为开机值
    *	6.	#000PCSM!						//开机释放扭力
    *	7.	#000PRAD!						//读取角度
    *	8.	#000PULK!						//释力
    *	9.	#000PULR!						//恢复扭力
    *	10.	#000PSCK!						//设置当前值为1500的偏差值
    *	11.	#000PSCK+050!					//把1500+50作为1500的中间值
    *	12.	#000PSCK-050!					//把1500-50作为1500的中间值
    *	13.	#000PMOD!						//读取模式
    *	14.	#000PMOD1!						//设置模式 舵机模式：2 270 逆时针 1：270 顺 4 180 逆时针 3：180 顺    马达模式：6 360 逆时针 5：360 顺圈 8 360 逆时针 7：360 顺时
    *	15.	#000PRTV!						//读取电压和温度
    *	16.	#000PSTB!						//读取保护值
    *	17.	#000PSTB=60!					//设置保护值 默认60 范围25-80
    *	18.	#000PPAAAIBBB!					//设置KP = AAA, KI = BBB
    *	19.	#000PMIN!						//设置最小值
    *	20.	#000PMAX!						//设置最大值
    *	21.	#000PULM! 						//释力 不带阻力
    *	22.	#000PLN!						//RGB灯开启
    *	23.	#000PLF!						//RGB灯关闭
 **********************************************/

#define SERVO_MOTOR_CNT 7
#define Servo_Frame_First 0x55
#define Servo_Frame_Second 0x55
#define Servo_MAX_BUFF 32
#define SERVO_MOVE_CMD 0x03
#define SERVO_UNLOAD_CMD 0x14
#define SERVO_POS_READ_CMD 0x15

typedef enum
{
    Servo_None_Type = 0,
    Bus_Servo = 1,
    PWM_Servo = 2,
}ServoType_e;

/* 用于初始化不同舵机的结构体,各类舵机通用 */
typedef struct
{
    PWM_Init_Config_s pwm_init_config;
    ServoType_e servo_type;
    UART_HandleTypeDef *_handle;
    uint8_t servo_id;
}Servo_Init_Config_s;

typedef struct
{   
    uint8_t servo_id;
    float angle;
    uint16_t recv_angle;    //原代码中该角度存储为整数形式(放大10倍)，即回调使用时需要除以10
    PWMInstance *pwm_instance;
    USARTInstance *usart_instance;
    ServoType_e servo_type;
}ServoInstance;

ServoInstance *ServoInit(Servo_Init_Config_s *Servo_Init_Config);
void ServoSetAngle(ServoInstance *servo, float angle);

// Bus舵机专用函数
void Bus_Servo_Unload(ServoInstance *servo);
void Bus_Servo_GetAngle(ServoInstance *servo);
float Bus_Servo_ParseAngle(ServoInstance *servo);
void Bus_Servo_SetID(ServoInstance *servo, uint8_t new_id);

#endif // SERVO_MOTOR_H