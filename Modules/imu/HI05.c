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
 *          | (2字节)  | (小端序) | (小端序)|      | (hi91_t)  |
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
#include "ins_task.h"

/* IMU stream read/control struct */
static HIPNUC_Raw_t hipnuc_raw = {0}; //这个结构体是我从例程中复制过来的，在这个文件里到处传，我懒得改了。反正最后只返回了里面的hi91_t结构体。
static HI05_t *hi05_t;//这个是要输出的结构体
static USARTInstance *HI05_usart_instance;

const float gravity[3] = {0, 0, 9.81f};
static float dt = 0, t = 0;
static uint32_t HI05_DWT_Count = 0;


static uint16_t U2(uint8_t *p)
{
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}


/* parse the payload of a frame and feed into data section */
static int parse_data(HIPNUC_Raw_t *raw)
{
    int ofs = 0;
    uint8_t *p = &raw->buf[CH_HDR_SIZE];

    /* ignore all previous data */
    raw->hi91.tag = 0;

    while (ofs < raw->len)
    {
        switch (p[ofs])
        {
        case HIPNUC_ID_HI91://其实只要用这个
            memcpy(&raw->hi91, p + ofs, sizeof(HI91_t));
            ofs += sizeof(HI91_t);
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
static void HI05Decode(void)
{
    // 获取接收缓冲区大小
    uint16_t buffer_size = HI05_usart_instance->recv_buff_size;

    // 检查帧头是否正确 (0x5A 0xA5)
    if (buffer_size < CH_HDR_SIZE || hipnuc_raw.recv_buffer[0] != CHSYNC1 || hipnuc_raw.recv_buffer[1] != CHSYNC2)
    {
        // 帧头错误，丢弃数据
        return;
    }

    // 提取数据长度 (字节2-3)
    uint16_t data_len = U2(hipnuc_raw.recv_buffer + 2);

    // CRC校验（在这里写了，那个decode_hipnuc就不需要了，所以放到底下去了）
    uint16_t crc = 0;
    hipnuc_crc16(&crc, hipnuc_raw.recv_buffer, CH_HDR_SIZE - 2);              // 校验帧头前4字节
    hipnuc_crc16(&crc, hipnuc_raw.recv_buffer + CH_HDR_SIZE, data_len);       // 校验数据部分
    uint16_t received_crc = U2(hipnuc_raw.recv_buffer + CH_HDR_SIZE - 2);     // 提取接收到的CRC

    if (crc != received_crc)
    {
        // CRC校验失败，丢弃数据
        return;
    }

    // 将数据复制到hipnuc_raw缓冲区
    memcpy(hipnuc_raw.buf, hipnuc_raw.recv_buffer, buffer_size); // 只有crc 校验成功的数据才复制进来进行解包
    hipnuc_raw.len = data_len;
    hipnuc_raw.nbyte = buffer_size;

    // 解析数据
    parse_data(&hipnuc_raw);

    // 将 HI91_t 的数据逐一赋值给 HI05_t
    hi05_t->tag = hipnuc_raw.hi91.tag;
    hi05_t->main_status = hipnuc_raw.hi91.main_status;
    hi05_t->temp = hipnuc_raw.hi91.temp;
    hi05_t->air_pressure = hipnuc_raw.hi91.air_pressure;
    hi05_t->system_time = hipnuc_raw.hi91.system_time;
    memcpy(hi05_t->acc, hipnuc_raw.hi91.acc, sizeof(hi05_t->acc));
    memcpy(hi05_t->gyr, hipnuc_raw.hi91.gyr, sizeof(hi05_t->gyr));
    memcpy(hi05_t->mag, hipnuc_raw.hi91.mag, sizeof(hi05_t->mag));
    hi05_t->roll = hipnuc_raw.hi91.roll;
    hi05_t->pitch = hipnuc_raw.hi91.pitch;
    hi05_t->yaw = hipnuc_raw.hi91.yaw;
    memcpy(hi05_t->quat, hipnuc_raw.hi91.quat, sizeof(hi05_t->quat));

    // get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
    if (hi05_t->yaw - hi05_t->YawAngleLast > 180.0f)
    {
      hi05_t->YawRoundCount--;
    }
    else if (hi05_t->yaw - hi05_t->YawAngleLast < -180.0f)
    {
      hi05_t->YawRoundCount++;
    }
    hi05_t->YawTotalAngle = 360.0f * hi05_t->YawRoundCount + hi05_t->yaw;
    hi05_t->YawAngleLast = hi05_t->yaw;

    //数据转换
    for (int i = 0; i < 3; i++) {
      hi05_t->acc[i]*= GRAVITY;
      hi05_t->gyr[i]*= D2R;
    }
    hipnuc_raw.hi91.pitch*= D2R;
    hipnuc_raw.hi91.roll*= D2R;
    hipnuc_raw.hi91.yaw*= D2R;

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    dt = DWT_GetDeltaT(&HI05_DWT_Count);
    t += dt;

    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, hi05_t->quat);
    for (uint8_t i = 0; i < 3; ++i)  // 同样过一个低通滤波
    {
      hi05_t->MotionAccel_b[i] = (hi05_t->acc[i] - gravity_b[i]) * dt / (hi05_t->AccelLPF + dt) +
                             hi05_t->MotionAccel_b[i] * hi05_t->AccelLPF / (hi05_t->AccelLPF + dt);
    }
    
    // 直接内联BodyFrameToEarthFrame函数
    hi05_t->MotionAccel_n[0] = 2.0f * ((0.5f - hi05_t->quat[2] * hi05_t->quat[2] - hi05_t->quat[3] * hi05_t->quat[3]) * hi05_t->MotionAccel_b[0] + (hi05_t->quat[1] * hi05_t->quat[2] - hi05_t->quat[0] * hi05_t->quat[3]) * hi05_t->MotionAccel_b[1] +
                     (hi05_t->quat[1] * hi05_t->quat[3] + hi05_t->quat[0] * hi05_t->quat[2]) * hi05_t->MotionAccel_b[2]);

    hi05_t->MotionAccel_n[1] = 2.0f * ((hi05_t->quat[1] * hi05_t->quat[2] + hi05_t->quat[0] * hi05_t->quat[3]) * hi05_t->MotionAccel_b[0] + (0.5f - hi05_t->quat[1] * hi05_t->quat[1] - hi05_t->quat[3] * hi05_t->quat[3]) * hi05_t->MotionAccel_b[1] +
                     (hi05_t->quat[2] * hi05_t->quat[3] - hi05_t->quat[0] * hi05_t->quat[1]) * hi05_t->MotionAccel_b[2]);

    hi05_t->MotionAccel_n[2] = 2.0f * ((hi05_t->quat[1] * hi05_t->quat[3] - hi05_t->quat[0] * hi05_t->quat[2]) * hi05_t->MotionAccel_b[0] + (hi05_t->quat[2] * hi05_t->quat[3] + hi05_t->quat[0] * hi05_t->quat[1]) * hi05_t->MotionAccel_b[1] +
                     (0.5f - hi05_t->quat[1] * hi05_t->quat[1] - hi05_t->quat[2] * hi05_t->quat[2]) * hi05_t->MotionAccel_b[2]);
                     
    // 转换回导航系n (原函数调用已替换为内联代码)，这里不调用下面这个函数是因为会hardfault！！！！所以不要改回去！！！！
    // BodyFrameToEarthFrame(hi05_t->MotionAccel_b, hi05_t->MotionAccel_n, hi05_t->quat);

}

/**
 * @brief 初始化HI05 IMU模块
 * @param usart_handle UART句柄
 * @return 返回HI05原始数据结构体指针，用于外部访问解码后的IMU数据
 */
HI05_t *HI05_Init(UART_HandleTypeDef *usart_handle)
{
    // 配置USART初始化参数
    USART_Init_Config_s conf;
    conf.module_callback = HI05Decode;                              // 设置解码回调函数
    conf.recv_buff_size = sizeof(HI91_t) + CH_HDR_SIZE;             // 接收完整数据包: hi91_t(72字节) + 帧头和CRC(6字节) = 78字节
    conf.usart_handle = usart_handle;                               // 设置UART句柄

    // 注册USART实例
    HI05_usart_instance = USARTRegister(&conf);

    // 初始化hipnuc_raw数据结构
    memset(&hipnuc_raw, 0, sizeof(HIPNUC_Raw_t));
    hipnuc_raw.recv_buffer = HI05_usart_instance->recv_buff;

    // 创建 HI05_t 实例并让指针指向它
    hi05_t = (HI05_t*)malloc(sizeof(HI05_t));
    memset(hi05_t, 0, sizeof(HI05_t));

    // noise of accel is relatively big and of high freq,thus lpf is used
    hi05_t->AccelLPF = 0.0085;
    DWT_GetDeltaT(&HI05_DWT_Count);

    // 返回数据结构指针供外部访问
    return hi05_t;
}
