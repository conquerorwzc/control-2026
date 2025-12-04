//
// Created by zhouz on 2025/12/2.
//

#ifndef CONTROL_2026_HI05_H
#define CONTROL_2026_HI05_H

#include <stdint.h>
#include "main.h"

/* HiPNUC protocol constants */
#define HIPNUC_MAX_RAW_SIZE     (512)

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
} hi91_t;


/**
 * Packet 0x81: INS data, including lat, lon, eul, quat, raw IMU data
 */
typedef struct __attribute__((__packed__))
{
    uint8_t         tag;            /* Data packet tag */
    uint16_t        main_status;    /* Status information */
    uint8_t         ins_status;     /* INS status */
    uint16_t        gpst_wn;        /* GPS time: week number */
    uint32_t        gpst_tow;       /* GPS time: time of week */
    uint16_t        reserved;       /* reserved */
    int16_t         gyr_b[3];       /* Gyroscope data (raw) */
    int16_t         acc_b[3];       /* Accelerometer data (raw) */
    int16_t         mag_b[3];       /* Magnetometer data (raw) */
    int16_t         air_pressure;   /* Air pressure */
    int16_t         reserved1;      /* Reserved field */
    int8_t          temperature;    /* Temperature */
    uint8_t         utc_year;       /* UTC year */
    uint8_t         utc_month;      /* UTC month */
    uint8_t         utc_day;        /* UTC day */
    uint8_t         utc_hour;       /* UTC hour */
    uint8_t         utc_min;        /* UTC minute */
    uint16_t        utc_msec;       /* UTC milliseconds */
    int16_t         roll;           /* Roll angle */
    int16_t         pitch;          /* Pitch angle */
    uint16_t        yaw;            /* Yaw angle */
    int16_t         quat[4];        /* Quaternion */
    int32_t         ins_lon;        /* INS longitude */
    int32_t         ins_lat;        /* INS latitude */
    int32_t         ins_msl;        /* INS mean sea level altitude */
    uint8_t         pdop;           /* Position dilution of precision */
    uint8_t         hdop;           /* Horizontal dilution of precision */
    uint8_t         solq_pos;       /* Solution quality for position */
    uint8_t         nv_pos;         /* Number of satellites used for position */
    uint8_t         solq_heading;   /* Solution quality for heading */
    uint8_t         nv_heading;     /* Number of satellites used for heading */
    uint8_t         diff_age;       /* Differential age */
    int16_t         undulation;     /* Undulation */
    uint8_t         ant_status;     /* Reserved field */
    int16_t         vel_enu[3];     /* Velocity in ENU frame */
    int16_t         acc_enu[3];     /* Acceleration in ENU frame */
    int32_t         gnss_lon;       /* GNSS longitude */
    int32_t         gnss_lat;       /* GNSS latitude */
    int32_t         gnss_msl;       /* GNSS mean sea level altitude */
    uint8_t         reserved2[2];   /* Reserved field */
} hi81_t;

/* HI83 bitmap masks */
#define HI83_BMAP_ACC_B              (1u << 0)
#define HI83_BMAP_GYR_B              (1u << 1)
#define HI83_BMAP_MAG_B              (1u << 2)
#define HI83_BMAP_RPY                (1u << 3)
#define HI83_BMAP_QUAT               (1u << 4)
#define HI83_BMAP_SYSTEM_TIME        (1u << 5)
#define HI83_BMAP_UTC                (1u << 6)
#define HI83_BMAP_AIR_PRESSURE       (1u << 7)
#define HI83_BMAP_TEMPERATURE        (1u << 8)
#define HI83_BMAP_INCLINATION        (1u << 9)
#define HI83_BMAP_HSS                (1u << 10)
#define HI83_BMAP_HSS_FRQ            (1u << 11)
#define HI83_BMAP_VEL_ENU            (1u << 12)
#define HI83_BMAP_ACC_ENU            (1u << 13)
#define HI83_BMAP_INS_LON_LAT_MSL    (1u << 14)
#define HI83_BMAP_GNSS_QUALITY_NV    (1u << 15)
#define HI83_BMAP_OD_SPEED           (1u << 16)
#define HI83_BMAP_UNDULATION         (1u << 17)
#define HI83_BMAP_DIFF_AGE           (1u << 18)
#define HI83_BMAP_NODE_ID            (1u << 19)
#define HI83_BMAP_GNSS_LON_LAT_MSL   (1u << 30)
#define HI83_BMAP_GNSS_VEL           (1u << 31)

typedef struct __attribute__((__packed__))
{
    uint8_t  tag;
    uint16_t main_status;
    uint8_t  ins_status;
    uint32_t data_bitmap;

    float    acc_b[3];
    float    gyr_b[3];
    float    mag_b[3];
    float    rpy[3];
    float    quat[4];
    uint32_t system_time;
    struct __attribute__((__packed__)) {
        uint8_t  year;
        uint8_t  month;
        uint8_t  day;
        uint8_t  hour;
        uint8_t  min;
        uint16_t sec_ms;
        uint8_t  rev;
    } utc;
    float    air_pressure;
    float    temperature;
    float    inclination[3];
    float    hss[3];
    float    hss_frq[3];
    float    vel_enu[3];
    float    acc_enu[3];
    double   ins_lon_lat_msl[3];
    uint8_t  solq_pos;
    uint8_t  nv_pos;
    uint8_t  solq_heading;
    uint8_t  nv_heading;
    float    od_speed;
    float    undulation;
    float    diff_age;
    struct __attribute__((__packed__)) {
        uint8_t node_id;
        uint8_t reserved[3];
    } node;
    double   gnss_lon_lat_msl[3];
    float    gnss_vel[3];
} hi83_t;

/**
 * HiPNUC raw data structure
 */
typedef struct
{
    int nbyte;                          /* Number of bytes in message buffer */
    int len;                            /* Message length (bytes) */
    uint8_t buf[HIPNUC_MAX_RAW_SIZE];   /* Message raw buffer */
    hi91_t hi91;                        /* Decoded 0x91 packet data */
    hi81_t hi81;                        /* Decoded 0x81 packet data */
    hi83_t hi83;                        /* Decoded 0x83 packet data */
} hipnuc_raw_t;

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
hi91_t *HI05_Init(UART_HandleTypeDef *usart_handle);

#endif  // CONTROL_2026_HI05_H

