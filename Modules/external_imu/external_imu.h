//
// Created by zeg on 2025/12/21.
//


#ifndef EXTERNAL_IMU_H
#define EXTERNAL_IMU_H

#include "bsp_can.h"
#include "ins_task.h"
#include "stm32h7xx_hal.h"

// 数据范围定义
#define ACCEL_CAN_MAX     (235.2f)
#define ACCEL_CAN_MIN     (-235.2f)
#define GYRO_CAN_MAX      (34.88f)
#define GYRO_CAN_MIN      (-34.88f)
#define PITCH_CAN_MAX     (90.0f)
#define PITCH_CAN_MIN     (-90.0f)
#define ROLL_CAN_MAX      (180.0f)
#define ROLL_CAN_MIN      (-180.0f)
#define YAW_CAN_MAX       (180.0f)
#define YAW_CAN_MIN       (-180.0f)
#define TEMP_MIN          (0.0f)
#define TEMP_MAX          (60.0f)
#define Quaternion_MIN    (-1.0f)
#define Quaternion_MAX    (1.0f)

// 命令定义
#define CMD_READ          0
#define CMD_WRITE         1

// 通信端口枚举
typedef enum
{
    COM_USB=0,
    COM_RS485,
    COM_CAN,
    COM_VOFA
} imu_com_port_e;

// 波特率枚举
typedef enum
{
    CAN_BAUD_1M=0,
    CAN_BAUD_500K,
    CAN_BAUD_400K,
    CAN_BAUD_250K,
    CAN_BAUD_200K,
    CAN_BAUD_100K,
    CAN_BAUD_50K,
    CAN_BAUD_25K
} imu_baudrate_e;

// 寄存器ID枚举
#define REBOOT_IMU          0x00
#define ACCEL_DATA          0x01
#define GYRO_DATA           0x02
#define EULER_DATA          0x03
#define QUAT_DATA           0x04
#define SET_ZERO            0x05
#define ACCEL_CALI          0x06
#define GYRO_CALI           0x07
#define MAG_CALI            0x08
#define CHANGE_COM          0x09
#define SET_DELAY           0x0A
#define CHANGE_ACTIVE       0x0B
#define SET_BAUD            0x0C
#define SET_CAN_ID          0x0D
#define SET_MST_ID          0x0E
#define DATA_OUTPUT_SELECTION 0x0F
#define SAVE_PARAM          0xFE
#define RESTORE_SETTING     0xFF
typedef struct
{
    CANInstance* can_instance;
    IMU_Data_t imu_data;
    uint8_t can_id;
    uint8_t mst_id;

    FDCAN_HandleTypeDef *can_handle;

    float pitch;
    float roll;
    float yaw;

    float gyro[3];
    float accel[3];

    float q[4];

    float cur_temp;
} external_imu_t;

/**
 * @brief 初始化外置IMU
 * @param can_id CAN ID
 * @param mst_id 主站ID
 * @param can_handle CAN句柄
 * @return 外置IMU实例指针
 */
external_imu_t* ExternalIMUInit(uint8_t can_id, uint8_t mst_id, FDCAN_HandleTypeDef *can_handle);

/**
 * @brief 外置IMU任务
 */
void ExternalIMUTask(void);

/**
 * @brief 写IMU寄存器
 * @param reg_id 寄存器ID
 * @param data 数据
 */
void ExternalIMUWriteReg(uint8_t reg_id, uint32_t data);

/**
 * @brief 读IMU寄存器
 * @param reg_id 寄存器ID
 */
void ExternalIMUReadReg(uint8_t reg_id);

/**
 * @brief 重启IMU
 */
void ExternalIMUReboot(void);

/**
 * @brief 加速度计校准
 */
void ExternalIMUAccelCalibration(void);

/**
 * @brief 陀螺仪校准
 */
void ExternalIMUGyroCalibration(void);

/**
 * @brief 更改通信端口
 * @param port 通信端口
 */
void ExternalIMUChangeComPort(imu_com_port_e port);

/**
 * @brief 设置主动模式延迟
 * @param delay 延迟时间
 */
void ExternalIMUSetActiveModeDelay(uint32_t delay);

/**
 * @brief 设置为主动模式
 */
void ExternalIMUChangeToActive(void);

/**
 * @brief 设置为请求模式
 */
void ExternalIMUChangeToRequest(void);

/**
 * @brief 设置波特率
 * @param baud 波特率
 */
void ExternalIMUSetBaud(imu_baudrate_e baud);

/**
 * @brief 设置CAN ID
 * @param can_id CAN ID
 */
void ExternalIMUSetCanId(uint8_t can_id);

/**
 * @brief 设置主站ID
 * @param mst_id 主站ID
 */
void ExternalIMUSetMstId(uint8_t mst_id);

/**
 * @brief 保存参数
 */
void ExternalIMUSaveParameters(void);

/**
 * @brief 恢复设置
 */
void ExternalIMURestoreSettings(void);

/**
 * @brief 请求加速度数据
 */
void ExternalIMURequestAccel(void);

/**
 * @brief 请求角速度数据
 */
void ExternalIMURequestGyro(void);

/**
 * @brief 请求欧拉角数据
 */
void ExternalIMURequestEuler(void);

/**
 * @brief 请求四元数数据
 */
void ExternalIMURequestQuat(void);

/**
 * @brief 获取IMU数据指针
 * @return IMU数据指针
 */
IMU_Data_t* ExternalIMUGetData(void);

#endif // EXTERNAL_IMU_H
