/**
******************************************************************************
 * @file    HI05.c
 * @author  Zhou Zele
 * @version V2.0.0
 * @date    2025/12/3
 * @brief   HI05 IMU数据解析驱动
 *
 * @note    数据包格式 (总计97字节):
 *          +----------+----------+--------+------+-----------+
 *          | 帧头     | 长度     | CRC16  | TAG  | 数据      |
 *          +----------+----------+--------+------+-----------+
 *          | 0x5A 0xA5| 2 bytes  | 2 bytes| 0x81 | 90 bytes  |
 *          | (2字节)  | (小端序) | (小端序)|      | (hi81_t)  |
 *          +----------+----------+--------+------+-----------+
 *
 *          修改说明:
 *          - 原实现采用逐字节解析方式，每次接收1个字节
 *          - 新实现直接接收完整数据包(97字节)，提高解析效率
 *          - CRC校验覆盖: 帧头前4字节 + 完整数据部分
 ******************************************************************************
 */

#include "HI05.h"
#include <string.h>
#include "bsp_dwt.h"
#include "bsp_usart.h"

/* IMU stream read/control struct */
static hipnuc_raw_t hipnuc_raw = {0}; //这个结构体是我从例程中复制过来的，在这个文件里到处传，我懒得改了。反正最后只返回了里面的hi91_t结构体。

static USARTInstance *HI05_usart_instance;

/* HiPNUC protocol constants */
#define CHSYNC1                 (0x5A)              /* CHAOHE message sync code 1 */
#define CHSYNC2                 (0xA5)              /* CHAOHE message sync code 2 */
#define CH_HDR_SIZE             (0x06)              /* CHAOHE protocol header size */


/* new HiPNUC standard packet */
#define HIPNUC_ID_HI91        (0x91)
#define HIPNUC_ID_HI81        (0x81)
#define HIPNUC_ID_HI83        (0x83)


/* common type conversion */
#define I2(p) (*((int16_t *)(p)))
static uint16_t U2(uint8_t *p)
{
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}

static float R4(uint8_t *p)
{
    float r;
    memcpy(&r, p, 4);
    return r;
}

static uint32_t U4(uint8_t *p)
{
    uint32_t u;
    memcpy(&u, p, 4);
    return u;
}

static double D8(uint8_t *p)
{
    double d;
    memcpy(&d, p, 8);
    return d;
}

/* parse the payload of a frame and feed into data section */
static int parse_data(hipnuc_raw_t *raw)
{
    int ofs = 0;
    uint8_t *p = &raw->buf[CH_HDR_SIZE];

    /* ignore all previous data */
    raw->hi91.tag = 0;
    raw->hi81.tag = 0;
    raw->hi83.tag = 0;

    while (ofs < raw->len)
    {
        switch (p[ofs])
        {
        case HIPNUC_ID_HI91://其实只要用这个
            memcpy(&raw->hi91, p + ofs, sizeof(hi91_t));
            ofs += sizeof(hi91_t);
            break;
        case HIPNUC_ID_HI81://其实是用不上的
            memcpy(&raw->hi81, p + ofs, sizeof(hi81_t));
            ofs += sizeof(hi81_t);
            break;
        case HIPNUC_ID_HI83://其实也是用不上的
        {
            raw->hi83.tag = 0x83;
            raw->hi83.main_status = U2(p + ofs + 1);
            raw->hi83.ins_status = p[ofs + 3];
            raw->hi83.data_bitmap = U4(p + ofs + 4);
            int idx = ofs + 8;
            uint32_t bm = raw->hi83.data_bitmap;

            if (bm & HI83_BMAP_ACC_B) { raw->hi83.acc_b[0] = R4(p + idx + 0); raw->hi83.acc_b[1] = R4(p + idx + 4); raw->hi83.acc_b[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_GYR_B) { raw->hi83.gyr_b[0] = R4(p + idx + 0); raw->hi83.gyr_b[1] = R4(p + idx + 4); raw->hi83.gyr_b[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_MAG_B) { raw->hi83.mag_b[0] = R4(p + idx + 0); raw->hi83.mag_b[1] = R4(p + idx + 4); raw->hi83.mag_b[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_RPY) { raw->hi83.rpy[0] = R4(p + idx + 0); raw->hi83.rpy[1] = R4(p + idx + 4); raw->hi83.rpy[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_QUAT) { raw->hi83.quat[0] = R4(p + idx + 0); raw->hi83.quat[1] = R4(p + idx + 4); raw->hi83.quat[2] = R4(p + idx + 8); raw->hi83.quat[3] = R4(p + idx + 12); idx += 16; }
            if (bm & HI83_BMAP_SYSTEM_TIME) { raw->hi83.system_time = U4(p + idx); idx += 4; }
            if (bm & HI83_BMAP_UTC) { raw->hi83.utc.year = p[idx+0]; raw->hi83.utc.month = p[idx+1]; raw->hi83.utc.day = p[idx+2]; raw->hi83.utc.hour = p[idx+3]; raw->hi83.utc.min = p[idx+4]; raw->hi83.utc.sec_ms = U2(p + idx + 5); raw->hi83.utc.rev = p[idx+7]; idx += 8; }
            if (bm & HI83_BMAP_AIR_PRESSURE) { raw->hi83.air_pressure = R4(p + idx); idx += 4; }
            if (bm & HI83_BMAP_TEMPERATURE) { raw->hi83.temperature = R4(p + idx); idx += 4; }
            if (bm & HI83_BMAP_INCLINATION) { raw->hi83.inclination[0] = R4(p + idx + 0); raw->hi83.inclination[1] = R4(p + idx + 4); raw->hi83.inclination[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_HSS) { raw->hi83.hss[0] = R4(p + idx + 0); raw->hi83.hss[1] = R4(p + idx + 4); raw->hi83.hss[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_HSS_FRQ) { raw->hi83.hss_frq[0] = R4(p + idx + 0); raw->hi83.hss_frq[1] = R4(p + idx + 4); raw->hi83.hss_frq[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_VEL_ENU) { raw->hi83.vel_enu[0] = R4(p + idx + 0); raw->hi83.vel_enu[1] = R4(p + idx + 4); raw->hi83.vel_enu[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_ACC_ENU) { raw->hi83.acc_enu[0] = R4(p + idx + 0); raw->hi83.acc_enu[1] = R4(p + idx + 4); raw->hi83.acc_enu[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_INS_LON_LAT_MSL) { raw->hi83.ins_lon_lat_msl[0] = D8(p + idx + 0); raw->hi83.ins_lon_lat_msl[1] = D8(p + idx + 8); raw->hi83.ins_lon_lat_msl[2] = D8(p + idx + 16); idx += 24; }
            if (bm & HI83_BMAP_GNSS_QUALITY_NV) { raw->hi83.solq_pos = p[idx+0]; raw->hi83.nv_pos = p[idx+1]; raw->hi83.solq_heading = p[idx+2]; raw->hi83.nv_heading = p[idx+3]; idx += 4; }
            if (bm & HI83_BMAP_OD_SPEED) { raw->hi83.od_speed = R4(p + idx); idx += 4; }
            if (bm & HI83_BMAP_UNDULATION) { raw->hi83.undulation = R4(p + idx); idx += 4; }
            if (bm & HI83_BMAP_DIFF_AGE) { raw->hi83.diff_age = R4(p + idx); idx += 4; }
            if (bm & HI83_BMAP_NODE_ID) { raw->hi83.node.node_id = p[idx+0]; raw->hi83.node.reserved[0] = p[idx+1]; raw->hi83.node.reserved[1] = p[idx+2]; raw->hi83.node.reserved[2] = p[idx+3]; idx += 4; }
            if (bm & HI83_BMAP_GNSS_LON_LAT_MSL) { raw->hi83.gnss_lon_lat_msl[0] = D8(p + idx + 0); raw->hi83.gnss_lon_lat_msl[1] = D8(p + idx + 8); raw->hi83.gnss_lon_lat_msl[2] = D8(p + idx + 16); idx += 24; }
            if (bm & HI83_BMAP_GNSS_VEL) { raw->hi83.gnss_vel[0] = R4(p + idx + 0); raw->hi83.gnss_vel[1] = R4(p + idx + 4); raw->hi83.gnss_vel[2] = R4(p + idx + 8); idx += 12; }

            ofs = idx;
        }
            break;
        default:
            ofs++;
            break;
        }
    }
    return 1;
}


/**
 * @brief    Calculate HiPNUC CRC16
 *
 * @param    inital is initial value
 * @param    buf    is input buffer pointer
 * @param    len    is length of the buffer
 */
static void hipnuc_crc16(uint16_t *inital, const uint8_t *buf, uint32_t len)
{
    uint32_t crc = *inital;
    uint32_t j;
    for (j=0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = buf[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *inital = crc;
}



/**
 * @brief HI05 IMU解码回调函数
 * @note 该函数在USART接收中断中被调用，直接处理完整的数据包
 */
static void DecodeHI05(void)
{
    // 获取接收缓冲区指针和大小
    uint8_t *buffer = HI05_usart_instance->recv_buff;
    uint16_t buffer_size = HI05_usart_instance->recv_buff_size;

    // 检查帧头是否正确 (0x5A 0xA5)
    if (buffer_size < CH_HDR_SIZE || buffer[0] != CHSYNC1 || buffer[1] != CHSYNC2)
    {
        // 帧头错误，丢弃数据
        return;
    }

    // 提取数据长度 (字节2-3)
    uint16_t data_len = U2(buffer + 2);

    // CRC校验（在这里写了，那个decode_hipnuc就不需要了，所以放到底下去了）
    uint16_t crc = 0;
    hipnuc_crc16(&crc, buffer, CH_HDR_SIZE - 2);              // 校验帧头前4字节
    hipnuc_crc16(&crc, buffer + CH_HDR_SIZE, data_len);       // 校验数据部分
    uint16_t received_crc = U2(buffer + CH_HDR_SIZE - 2);     // 提取接收到的CRC

    if (crc != received_crc)
    {
        // CRC校验失败，丢弃数据
        return;
    }

    // 将数据复制到hipnuc_raw缓冲区
    memcpy(hipnuc_raw.buf, buffer, buffer_size);
    hipnuc_raw.len = data_len;
    hipnuc_raw.nbyte = buffer_size;

    // 解析数据
    parse_data(&hipnuc_raw);
}

/**
 * @brief 初始化HI05 IMU模块
 * @param usart_handle UART句柄
 * @return 返回HI05原始数据结构体指针，用于外部访问解码后的IMU数据
 */
hi91_t *HI05_Init(UART_HandleTypeDef *usart_handle)
{
    // 配置USART初始化参数
    USART_Init_Config_s conf;
    conf.module_callback = DecodeHI05;                              // 设置解码回调函数
    conf.recv_buff_size = sizeof(hi91_t) + CH_HDR_SIZE;             // 接收完整数据包: hi91_t(72字节) + 帧头和CRC(6字节) = 78字节
    conf.usart_handle = usart_handle;                               // 设置UART句柄

    // 注册USART实例
    HI05_usart_instance = USARTRegister(&conf);

    // 初始化hipnuc_raw数据结构
    memset(&hipnuc_raw, 0, sizeof(hipnuc_raw_t));

    // 返回数据结构指针供外部访问
    hi91_t *hi91 = &hipnuc_raw.hi91;
    return hi91;
}


//------------------------------------------------unused definitions------------------------------------------------//
#ifndef GRAVITY
#define GRAVITY (9.8F)
#endif

#ifndef D2R
#define D2R (0.0174532925199433F)
#endif

#ifndef R2D
#define R2D (57.2957795130823F)
#endif

//------------------------------------------------unused functions------------------------------------------------//
/**
 * @brief    Convert packet to string, only dump parts of data
 *
 * @param    raw is struct of decoder
 * @param    buf is the log string buffer, make sure buf is larger than 256
 * @param    buf_size is the size of the log buffer
 * @return   Number of characters written to the buffer
 */
int hipnuc_dump_packet(hipnuc_raw_t *raw, char *buf, size_t buf_size)
{
    int written = 0;
    int ret;

    buf = HI05_usart_instance->recv_buff;

    /* dump 0x91 packet */
    if(raw->hi91.tag == HIPNUC_ID_HI91)
    {
        /* Format:
         * system_time: ms
         * acc: m/s²
         * gyr: deg/s
         * mag: uT
         * pitch/roll/yaw: deg
         * quat: w,x,y,z
         * air_pressure: Pa
         */
        ret = snprintf(buf + written, buf_size - written,
            "{\n"
            "  \"type\": \"HI91\",\n"
            "  \"main_status\": [0x%X],\n"
            "  \"system_time\": %d,\n"
            "  \"acc\": [%.3f, %.3f, %.3f],\n"
            "  \"gyr\": [%.3f, %.3f, %.3f],\n"
            "  \"mag\": [%.3f, %.3f, %.3f],\n"
            "  \"pitch\": %.2f,\n"
            "  \"roll\": %.2f,\n"
            "  \"yaw\": %.2f,\n"
            "  \"quat\": [%.3f, %.3f, %.3f, %.3f],\n"
            "  \"air_pressure\": %.1f\n"
            "}\n",
            raw->hi91.main_status,
            raw->hi91.system_time,
            raw->hi91.acc[0]*GRAVITY, raw->hi91.acc[1]*GRAVITY, raw->hi91.acc[2]*GRAVITY,
            raw->hi91.gyr[0], raw->hi91.gyr[1], raw->hi91.gyr[2],
            raw->hi91.mag[0], raw->hi91.mag[1], raw->hi91.mag[2],
            raw->hi91.pitch, raw->hi91.roll, raw->hi91.yaw,
            raw->hi91.quat[0], raw->hi91.quat[1], raw->hi91.quat[2], raw->hi91.quat[3],
            raw->hi91.air_pressure);
    }



    /* dump 0x81 packet */
    else if(raw->hi81.tag == HIPNUC_ID_HI81)
    {
        /* Format:
         * status: device status
         * ins_status: INS algorithm status
         * gpst_wn/tow: GPS week number and time of week
         * gyr: deg/s
         * acc: m/s²
         * mag: uT
         * air_pressure: Pa
         * temperature: °C
         * utc: YYYY-MM-DD HH:mm:ss.SSS
         * pitch/roll/yaw: deg
         * quat: w,x,y,z
         * ins_lat/lon: deg
         * ins_msl: m
         * pdop/hdop: position/horizontal dilution of precision
         * solq_pos: 0:invalid 1:SPP 2:DGPS 4:RTK-FLOAT 5:RTK-FIXED
         * nv_pos: number of satellites used for position
         * solq_heading: 0:invalid 4:valid
         * nv_heading: number of satellites used for heading
         * diff_age: differential age(s)
         * undulation: geoidal separation(m)
         * vel_enu: east,north,up velocity(m/s)
         * acc_enu: east,north,up acceleration(m/s²)
         */
        ret = snprintf(buf + written, buf_size - written,
            "{\n"
            "  \"type\": \"HI81\",\n"
            "  \"main_status\": %d,\n"
            "  \"ins_status\": %d,\n"
            "  \"gpst_wn\": %d,\n"
            "  \"gpst_tow\": %d,\n"
            "  \"gyr\": [%.3f, %.3f, %.3f],\n"
            "  \"acc\": [%.3f, %.3f, %.3f],\n"
            "  \"mag\": [%.3f, %.3f, %.3f],\n"
            "  \"air_pressure\": %.1f,\n"
            "  \"temperature\": %d,\n"
            "  \"utc\": \"20%02d-%02d-%02d %02d:%02d:%02d.%03d\",\n"
            "  \"pitch\": %.2f,\n"
            "  \"roll\": %.2f,\n"
            "  \"yaw\": %.2f,\n"
            "  \"quat\": [%.3f, %.3f, %.3f, %.3f],\n"
            "  \"ins_lat\": %.7f,\n"
            "  \"ins_lon\": %.7f,\n"
            "  \"ins_msl\": %.2f,\n"
            "  \"pdop\": %.1f,\n"
            "  \"hdop\": %.1f,\n"
            "  \"solq_pos\": %d,\n"
            "  \"nv_pos\": %d,\n"
            "  \"solq_heading\": %d,\n"
            "  \"nv_heading\": %d,\n"
            "  \"diff_age\": %d,\n"
            "  \"undulation\": %.2f,\n"
            "  \"vel_enu\": [%.2f, %.2f, %.2f],\n"
            "  \"acc_enu\": [%.2f, %.2f, %.2f],\n"
            "}\n",
            raw->hi81.main_status,
            raw->hi81.ins_status,
            raw->hi81.gpst_wn,
            raw->hi81.gpst_tow,
            raw->hi81.gyr_b[0]*(0.001*R2D), raw->hi81.gyr_b[1]*(0.001*R2D), raw->hi81.gyr_b[2]*(0.001*R2D),
            raw->hi81.acc_b[0]*0.0048828, raw->hi81.acc_b[1]*0.0048828, raw->hi81.acc_b[2]*0.0048828,
            raw->hi81.mag_b[0]*0.030517, raw->hi81.mag_b[1]*0.030517, raw->hi81.mag_b[2]*0.030517,
            (float)raw->hi81.air_pressure,
            raw->hi81.temperature,
            raw->hi81.utc_year,
            raw->hi81.utc_month,
            raw->hi81.utc_day,
            raw->hi81.utc_hour,
            raw->hi81.utc_min,
            raw->hi81.utc_msec/1000,
            raw->hi81.utc_msec%1000,
            raw->hi81.pitch*0.01,
            raw->hi81.roll*0.01,
            raw->hi81.yaw*0.01,
            raw->hi81.quat[0]*0.0001, raw->hi81.quat[1]*0.0001, raw->hi81.quat[2]*0.0001, raw->hi81.quat[3]*0.0001,
            raw->hi81.ins_lat*1e-7,
            raw->hi81.ins_lon*1e-7,
            raw->hi81.ins_msl*1e-3,
            raw->hi81.pdop*0.1,
            raw->hi81.hdop*0.1,
            raw->hi81.solq_pos,
            raw->hi81.nv_pos,
            raw->hi81.solq_heading,
            raw->hi81.nv_heading,
            raw->hi81.diff_age,
            raw->hi81.undulation*0.01,
            raw->hi81.vel_enu[0]*0.01, raw->hi81.vel_enu[1]*0.01, raw->hi81.vel_enu[2]*0.01,
            raw->hi81.acc_enu[0]*0.0048828, raw->hi81.acc_enu[1]*0.0048828, raw->hi81.acc_enu[2]*0.0048828);
        }

    else if (raw->hi83.tag == HIPNUC_ID_HI83)
    {
        ret = snprintf(buf + written, buf_size - written,
            "{\n"
            "  \"type\": \"HI83\",\n"
            "  \"main_status\": %d,\n"
            "  \"ins_status\": %u,\n"
            "  \"data_bitmap\": %u\n",
            raw->hi83.main_status,
            (unsigned)raw->hi83.ins_status,
            (unsigned)raw->hi83.data_bitmap);
        if (ret > 0) written += ret;

        if (raw->hi83.data_bitmap & HI83_BMAP_ACC_B) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"acc\": [%.3f, %.3f, %.3f]\n", raw->hi83.acc_b[0], raw->hi83.acc_b[1], raw->hi83.acc_b[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_GYR_B) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"gyr\": [%.3f, %.3f, %.3f]\n", raw->hi83.gyr_b[0], raw->hi83.gyr_b[1], raw->hi83.gyr_b[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_MAG_B) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"mag\": [%.3f, %.3f, %.3f]\n", raw->hi83.mag_b[0], raw->hi83.mag_b[1], raw->hi83.mag_b[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_RPY) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"pitch\": %.2f\n  ,\"roll\": %.2f\n  ,\"yaw\": %.2f\n", raw->hi83.rpy[1], raw->hi83.rpy[0], raw->hi83.rpy[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_QUAT) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"quat\": [%.3f, %.3f, %.3f, %.3f]\n", raw->hi83.quat[0], raw->hi83.quat[1], raw->hi83.quat[2], raw->hi83.quat[3]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_SYSTEM_TIME) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"system_time\": %u\n", (unsigned)raw->hi83.system_time);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_UTC) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"utc\": \"20%02u-%02u-%02u %02u:%02u:%02u.%03u\"\n", (unsigned)raw->hi83.utc.year, (unsigned)raw->hi83.utc.month, (unsigned)raw->hi83.utc.day, (unsigned)raw->hi83.utc.hour, (unsigned)raw->hi83.utc.min, (unsigned)(raw->hi83.utc.sec_ms/1000), (unsigned)(raw->hi83.utc.sec_ms%1000));
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_AIR_PRESSURE) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"air_pressure\": %.1f\n", raw->hi83.air_pressure);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_TEMPERATURE) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"temperature\": %.2f\n", raw->hi83.temperature);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_INCLINATION) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"inclination\": [%.2f, %.2f, %.2f]\n", raw->hi83.inclination[0], raw->hi83.inclination[1], raw->hi83.inclination[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_HSS) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"hss\": [%.3f, %.3f, %.3f]\n", raw->hi83.hss[0], raw->hi83.hss[1], raw->hi83.hss[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_HSS_FRQ) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"hss_frq\": [%.3f, %.3f, %.3f]\n", raw->hi83.hss_frq[0], raw->hi83.hss_frq[1], raw->hi83.hss_frq[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_VEL_ENU) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"vel_enu\": [%.3f, %.3f, %.3f]\n", raw->hi83.vel_enu[0], raw->hi83.vel_enu[1], raw->hi83.vel_enu[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_ACC_ENU) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"acc_enu\": [%.3f, %.3f, %.3f]\n", raw->hi83.acc_enu[0], raw->hi83.acc_enu[1], raw->hi83.acc_enu[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_INS_LON_LAT_MSL) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"ins_lon_lat_msl\": [%.7f, %.7f, %.3f]\n", raw->hi83.ins_lon_lat_msl[0], raw->hi83.ins_lon_lat_msl[1], raw->hi83.ins_lon_lat_msl[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_GNSS_QUALITY_NV) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"solq_pos\": %u\n  ,\"nv_pos\": %u\n  ,\"solq_heading\": %u\n  ,\"nv_heading\": %u\n", (unsigned)raw->hi83.solq_pos, (unsigned)raw->hi83.nv_pos, (unsigned)raw->hi83.solq_heading, (unsigned)raw->hi83.nv_heading);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_OD_SPEED) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"od_speed\": %.3f\n", raw->hi83.od_speed);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_UNDULATION) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"undulation\": %.3f\n", raw->hi83.undulation);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_DIFF_AGE) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"diff_age\": %.3f\n", raw->hi83.diff_age);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_NODE_ID) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"node_id\": %u\n", (unsigned)raw->hi83.node.node_id);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_GNSS_LON_LAT_MSL) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"gnss_lon_lat_msl\": [%.7f, %.7f, %.3f]\n", raw->hi83.gnss_lon_lat_msl[0], raw->hi83.gnss_lon_lat_msl[1], raw->hi83.gnss_lon_lat_msl[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_GNSS_VEL) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"gnss_vel\": [%.3f, %.3f, %.3f]\n", raw->hi83.gnss_vel[0], raw->hi83.gnss_vel[1], raw->hi83.gnss_vel[2]);
            if (ret > 0) written += ret;
        }

        ret = snprintf(buf + written, buf_size - written, "}\n");
        if (ret > 0) written += ret;
        ret = 0;
    }

    if (ret > 0) written += ret;
    return written;
}

/* sync code */
static int sync_hipnuc(uint8_t *buf, uint8_t data)
{
    buf[0] = buf[1];
    buf[1] = data;
    return buf[0] == CHSYNC1 && buf[1] == CHSYNC2;
}

// 调用解码函数进行CRC校验和数据解析，返回解码结果（>0成功，<0失败）
static int decode_hipnuc(hipnuc_raw_t *raw)
{
  uint16_t crc = 0;

  /* checksum */
  hipnuc_crc16(&crc, raw->buf, (CH_HDR_SIZE-2));
  hipnuc_crc16(&crc, raw->buf + CH_HDR_SIZE, raw->len);
  if (crc != U2(raw->buf + (CH_HDR_SIZE-2)))
  {
    // NL_TRACE("ch checksum error: frame:0x%X calcuate:0x%X, len:%d\n", U2(raw->buf + 4), crc, raw->len);
    return -1;
  }

  return parse_data(raw);
}

/**
 * @brief     HiPNUC decoder input, read one byte at a time.
 *
 * @param    raw is the decoder struct.
 * @param    data is the one byte read from stream.
 * @return   >0: decoder received a frame successfully, else: receiver did not receive a frame successfully.
 */
int hipnuc_input(hipnuc_raw_t *raw, uint8_t data)
{
    /* synchronize frame */
    if (raw->nbyte == 0)                                                        // 状态机初始状态：等待帧头同步码（0x5A 0xA5）
    {
        if (!sync_hipnuc(raw->buf, data))                                       // 调用同步函数检查是否接收到帧头同步码，未同步则返回0继续等待下一个字节
            return 0;
        raw->nbyte = 2;                                                         // 同步成功，已接收2个字节（帧头），更新接收计数器
        return 0;                                                               // 返回0表示帧未完成，继续接收
    }

    raw->buf[raw->nbyte++] = data;                                              // 将接收到的字节存入缓冲区，同时接收计数器加1

    if (raw->nbyte == CH_HDR_SIZE)                                              // 判断是否接收完协议头（6字节：0x5A 0xA5 + 2字节长度 + 2字节CRC）
    {
        if ((raw->len = U2(raw->buf + 2)) > (HIPNUC_MAX_RAW_SIZE - CH_HDR_SIZE)) // 从协议头中提取数据长度字段（buf[2..3]），并检查长度是否超出最大允许值
        {
            // NL_TRACE("ch length error: len=%d\n",raw->len);
            raw->nbyte = 0;                                                     // 长度异常，重置状态机，丢弃当前帧
            return -1;                                                          // 返回-1表示帧错误
        }
    }

    if (raw->nbyte < CH_HDR_SIZE || raw->nbyte < (raw->len + CH_HDR_SIZE))     // 判断是否接收完整帧：需要接收完协议头+有效载荷数据
    {
        return 0;                                                               // 帧未接收完整，返回0继续接收后续字节
    }

    raw->nbyte = 0;                                                             // 完整帧接收完毕，重置接收计数器，准备接收下一帧

    return decode_hipnuc(raw);                                                  // 调用解码函数进行CRC校验和数据解析，返回解码结果（>0成功，<0失败）
}
