//
// Created by zhouz on 2025/12/2.
//

#ifndef CONTROL_2026_HI05_H
#define CONTROL_2026_HI05_H

#include <stdint.h>
#include "main.h"

/* HiPNUC protocol constants */
#define HIPNUC_MAX_RAW_SIZE     (512)


/* HiPNUC protocol constants */
#define CHSYNC1                 (0x5A)              /* CHAOHE message sync code 1 */
#define CHSYNC2                 (0xA5)              /* CHAOHE message sync code 2 */
#define CH_HDR_SIZE             (0x06)              /* CHAOHE protocol header size */


/* new HiPNUC standard packet */
#define HIPNUC_ID_HI91        (0x91)
#define HIPNUC_ID_HI81        (0x81)
#define HIPNUC_ID_HI83        (0x83)

#ifndef GRAVITY
#define GRAVITY (9.8F)
#endif

#ifndef D2R
#define D2R (0.0174532925199433F)
#endif

#ifndef R2D
#define R2D (57.2957795130823F)
#endif

/* common type conversion */
#define I2(p) (*((int16_t *)(p)))


/**
 * Packet 0x91: IMU data (floating point)
 */
typedef struct __attribute__((__packed__))
{
    uint8_t         tag;            /* Data packet tag, if tag = 0x00, means that this packet is null */
    uint16_t        main_status;    /* reserved */
    int8_t          temp;           /* Temperature */
    float           air_pressure;   /* Pressure */
    uint32_t        system_time;    /* Timestamp */
    float           acc[3];         /* Accelerometer data (x, y, z) */
    float           gyr[3];         /* Gyroscope data (x, y, z) */
    float           mag[3];         /* Magnetometer data (x, y, z) */
    float           roll;           /* Roll angle */
    float           pitch;          /* Pitch angle */
    float           yaw;            /* Yaw angle */
    float           quat[4];        /* Quaternion (w, x, y, z) */
} HI91_t;

/**
 * 最终传出的数据结构体类型
*/

typedef struct __attribute__((__packed__))
{
  uint8_t         tag;            /* Data packet tag, if tag = 0x00, means that this packet is null */
  uint16_t        main_status;    /* reserved */
  int8_t          temp;           /* Temperature */
  float           air_pressure;   /* Pressure */
  uint32_t        system_time;    /* Timestamp */
  float           acc[3];         /* Accelerometer data (x, y, z) */
  float           gyr[3];         /* Gyroscope data (x, y, z) */
  float           mag[3];         /* Magnetometer data (x, y, z) */
  float           roll;           /* Roll angle */
  float           pitch;          /* Pitch angle */
  float           yaw;            /* Yaw angle */
  float           quat[4];        /* Quaternion (w, x, y, z) */
  float           YawTotalAngle;
  int16_t         YawRoundCount;
  float           YawAngleLast;
  float           AccelLPF;         // 加速度低通滤波系数，用于滤除高频噪声
  float           MotionAccel_b[3];         // 机体坐标系下加速度 [0]-X方向 [1]-Y方向 [2]-Z方向 单位: m/s^2
  float           MotionAccel_n[3];         // 导航坐标系(绝对系)下的加速度 [0]-X方向 [1]-Y方向 [2]-Z方向 单位: m/s^2
} HI05_t;

/**
 * HiPNUC raw data structure，这是用来观测获取到的数据的结构体。
 */
typedef struct
{
    int nbyte;                          /* Number of bytes in message buffer */
    int len;                            /* Message length (bytes) */
    uint8_t *recv_buffer;
    uint16_t recv_buffer_size;
    uint8_t buf[HIPNUC_MAX_RAW_SIZE];   /* 经过crc校验通过的Message buffer */
    HI91_t hi91;                        /* Decoded 0x91 packet data */
} HIPNUC_Raw_t;

/**
 * @brief 初始化HI05 IMU模块
 * @param usart_handle UART句柄
 * @return 返回HI05原始数据结构体指针，用于外部访问解码后的IMU数据
 *
 * @note 数据包格式:
 *       - 帧头: 0x5A 0xA5 (2字节)
 *       - 数据长度: 2字节 (小端序)
 *       - CRC16: 2字节 (小端序)
 *       - 数据: hi81_t结构体 (91字节)
 *       总计: 97字节 (6字节帧头+CRC + 91字节数据)
 */
HI05_t *HI05_Init(UART_HandleTypeDef *usart_handle);

#endif  // CONTROL_2026_HI05_H

