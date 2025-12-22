//
// Created by zeg on 2025/12/21.
//


#include "external_imu.h"
#include "bsp_dwt.h"
#include "string.h"

static external_imu_t external_imu_instance = {0};

// 添加函数声明以解决编译错误
void IMU_UpdateData(uint8_t* pData);

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief IMU CAN回调函数
 * @param instance CAN实例
 */
static void ExternalIMUCallback(CANInstance* instance)
{
    // 更新IMU数据
    IMU_UpdateData(instance->rx_buff);
}

/**
 * @brief 发送IMU命令
 * @param reg_id 寄存器ID
 * @param cmd 命令类型
 * @param data 数据
 */
static void ExternalIMUSendCmd(uint8_t reg_id, uint8_t cmd, uint32_t data)
{
    if (external_imu_instance.can_instance == NULL)
        return;

    uint8_t buf[8] = {0};
    buf[0] = 0xCC;  // 帧头
    buf[1] = reg_id;
    buf[2] = cmd;
    buf[3] = 0xDD;  // 帧尾
    memcpy(buf + 4, &data, 4);

    memcpy(external_imu_instance.can_instance->tx_buff, buf, 8);
    CANTransmit(external_imu_instance.can_instance, 10); // 10ms超时
}

/**
 * @brief 更新加速度数据
 * @param pData 数据指针
 */
static void IMU_UpdateAccel(uint8_t* pData)
{
    uint16_t accel[3];

    accel[0] = pData[3]<<8 | pData[2];
    accel[1] = pData[5]<<8 | pData[4];
    accel[2] = pData[7]<<8 | pData[6];

    external_imu_instance.accel[0] = uint_to_float(accel[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    external_imu_instance.accel[1] = uint_to_float(accel[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    external_imu_instance.accel[2] = uint_to_float(accel[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);

    // 同步到IMU数据结构体
    external_imu_instance.imu_data.Accel[0] = external_imu_instance.accel[0];
    external_imu_instance.imu_data.Accel[1] = external_imu_instance.accel[1];
    external_imu_instance.imu_data.Accel[2] = external_imu_instance.accel[2];
}

/**
 * @brief 更新角速度数据
 * @param pData 数据指针
 */
static void IMU_UpdateGyro(uint8_t* pData)
{
    uint16_t gyro[3];

    gyro[0] = pData[3]<<8 | pData[2];
    gyro[1] = pData[5]<<8 | pData[4];
    gyro[2] = pData[7]<<8 | pData[6];

    external_imu_instance.gyro[0] = uint_to_float(gyro[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    external_imu_instance.gyro[1] = uint_to_float(gyro[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    external_imu_instance.gyro[2] = uint_to_float(gyro[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);

    // 同步到IMU数据结构体
    external_imu_instance.imu_data.Gyro[0] = external_imu_instance.gyro[0];
    external_imu_instance.imu_data.Gyro[1] = external_imu_instance.gyro[1];
    external_imu_instance.imu_data.Gyro[2] = external_imu_instance.gyro[2];
}

/**
 * @brief 更新欧拉角数据
 * @param pData 数据指针
 */
static void IMU_UpdateEuler(uint8_t* pData)
{
    int euler[3];

    euler[0] = pData[3]<<8 | pData[2];
    euler[1] = pData[5]<<8 | pData[4];
    euler[2] = pData[7]<<8 | pData[6];

    external_imu_instance.pitch = uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
    external_imu_instance.yaw = uint_to_float(euler[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);
    external_imu_instance.roll = uint_to_float(euler[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);
}

/**
 * @brief 更新四元数数据
 * @param pData 数据指针
 */
static void IMU_UpdateQuaternion(uint8_t* pData)
{
    int w = pData[1]<<6 | ((pData[2]&0xF8)>>2);
    int x = (pData[2]&0x03)<<12 | (pData[3]<<4) | ((pData[4]&0xF0)>>4);
    int y = (pData[4]&0x0F)<<10 | (pData[5]<<2) | (pData[6]&0xC0)>>6;
    int z = (pData[6]&0x3F)<<8 | pData[7];

    external_imu_instance.q[0] = uint_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
    external_imu_instance.q[1] = uint_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
    external_imu_instance.q[2] = uint_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
    external_imu_instance.q[3] = uint_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);
}

/**
 * @brief 更新温度数据
 * @param pData 数据指针
 */
static void IMU_UpdateTemp(uint8_t* pData)
{
    uint16_t temp = pData[3]<<8 | pData[2];
    external_imu_instance.cur_temp = uint_to_float(temp, TEMP_MIN, TEMP_MAX, 16);
}

/**
 * @brief 更新IMU数据
 * @param pData 数据指针
 */
void IMU_UpdateData(uint8_t* pData)
{
    switch(pData[0])  // 根据寄存器ID判断数据类型
    {
        case ACCEL_DATA:
            IMU_UpdateAccel(pData);
            break;
        case GYRO_DATA:
            IMU_UpdateGyro(pData);
            break;
        case EULER_DATA:
            IMU_UpdateEuler(pData);
            break;
        case QUAT_DATA:
            IMU_UpdateQuaternion(pData);
            break;
        case 0x05:  // 温度数据
            IMU_UpdateTemp(pData);
            break;
    }
}

external_imu_t* ExternalIMUInit(uint8_t can_id, uint8_t mst_id, FDCAN_HandleTypeDef *can_handle)
{
    external_imu_instance.can_id = can_id;
    external_imu_instance.mst_id = mst_id;
    external_imu_instance.can_handle = can_handle;

    // 初始化CAN实例
    CAN_Init_Config_s can_config = {0};
    can_config.can_handle = can_handle;  // 直接赋值，无需类型转换
    can_config.tx_id = can_id;
    can_config.rx_id = mst_id;
    can_config.can_module_callback = ExternalIMUCallback;
    can_config.id = &external_imu_instance;

    external_imu_instance.can_instance = CANRegister(&can_config);

    // 初始化IMU数据
    external_imu_instance.imu_data.AccelScale = 1.0f;
    external_imu_instance.imu_data.GyroOffset[0] = 0.0f;
    external_imu_instance.imu_data.GyroOffset[1] = 0.0f;
    external_imu_instance.imu_data.GyroOffset[2] = 0.0f;

    return &external_imu_instance;
}

void ExternalIMUTask(void)
{
    // 定期请求数据，例如每10ms请求一次角速度和加速度
    static uint32_t last_request_time = 0;
    uint32_t current_time = DWT_GetTimeline_ms();  // 使用正确的函数名

    if (current_time - last_request_time > 10) {
        ExternalIMURequestGyro();
        ExternalIMURequestAccel();
        last_request_time = current_time;
    }
}

void ExternalIMUWriteReg(uint8_t reg_id, uint32_t data)
{
    ExternalIMUSendCmd(reg_id, CMD_WRITE, data);
}

void ExternalIMUReadReg(uint8_t reg_id)
{
    ExternalIMUSendCmd(reg_id, CMD_READ, 0);
}

void ExternalIMUReboot(void)
{
    ExternalIMUWriteReg(REBOOT_IMU, 0);
}

void ExternalIMUAccelCalibration(void)
{
    ExternalIMUWriteReg(ACCEL_CALI, 0);
}

void ExternalIMUGyroCalibration(void)
{
    ExternalIMUWriteReg(GYRO_CALI, 0);
}

void ExternalIMUChangeComPort(imu_com_port_e port)
{
    ExternalIMUWriteReg(CHANGE_COM, (uint32_t)port);
}

void ExternalIMUSetActiveModeDelay(uint32_t delay)
{
    ExternalIMUWriteReg(SET_DELAY, delay);
}

void ExternalIMUChangeToActive(void)
{
    ExternalIMUWriteReg(CHANGE_ACTIVE, 1);
}

void ExternalIMUChangeToRequest(void)
{
    ExternalIMUWriteReg(CHANGE_ACTIVE, 0);
}

void ExternalIMUSetBaud(imu_baudrate_e baud)
{
    ExternalIMUWriteReg(SET_BAUD, (uint32_t)baud);
}

void ExternalIMUSetCanId(uint8_t can_id)
{
    ExternalIMUWriteReg(SET_CAN_ID, can_id);
}

void ExternalIMUSetMstId(uint8_t mst_id)
{
    ExternalIMUWriteReg(SET_MST_ID, mst_id);
}

void ExternalIMUSaveParameters(void)
{
    ExternalIMUWriteReg(SAVE_PARAM, 0);
}

void ExternalIMURestoreSettings(void)
{
    ExternalIMUWriteReg(RESTORE_SETTING, 0);
}

void ExternalIMURequestAccel(void)
{
    ExternalIMUReadReg(ACCEL_DATA);
}

void ExternalIMURequestGyro(void)
{
    ExternalIMUReadReg(GYRO_DATA);
}

void ExternalIMURequestEuler(void)
{
    ExternalIMUReadReg(EULER_DATA);
}

void ExternalIMURequestQuat(void)
{
    ExternalIMUReadReg(QUAT_DATA);
}

IMU_Data_t* ExternalIMUGetData(void)
{
    return &external_imu_instance.imu_data;
}

