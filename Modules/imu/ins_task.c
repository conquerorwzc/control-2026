/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @author  annotation and modificaiton by neozng
 * @author  modification by Enhao Zhang
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins_task.h"

#include "QuaternionEKF.h"
#include "controller.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "user_lib.h"
#include "cmsis_os.h"
// #include "master_process.h" TODO: 待完善

static INS_t INS;
static IMU_Init_Config_s IMU_Param;
static BMI088Instance* bmi088_device;
static BMI088_Data_t bmi088_data;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

// 用于获取两次采样之间的时间间隔
static uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
static float RefTemp = 40;  // 恒温设定温度

// INS任务句柄
static osThreadId insTaskHandle;

/**
 * @brief IMU温度控制函数
 * @note  通过PID控制器调节加热PWM占空比，使IMU保持恒温状态(默认40°C)
 *        提高IMU在不同环境温度下的测量精度和稳定性
 */
static void IMU_Temperature_Ctrl(void) {
  float pid_ref = PIDCalculate(&bmi088_device->heat_pid, bmi088_data.temperature, RefTemp);
  PWMSetDutyRatio(bmi088_device->heat_pwm, float_constrain(float_rounding(pid_ref), 0, UINT32_MAX));
}

/**
 * @brief 初始化四元数
 * @param[out] init_q4 初始四元数数组指针
 * @note  通过读取加速度计数据计算初始姿态，确定载体相对于重力方向的初始角度
 *        利用加速度计测量的重力方向与理论重力方向的夹角来计算初始四元数
 */
static void InitQuaternion(float* init_q4) {
  float acc_init[3] = {0};
  float gravity_norm[3] = {0, 0, 1};  // 导航系重力加速度矢量,归一化后为(0,0,1)
  float axis_rot[3] = {0};            // 旋转轴
  // 读取100次加速度计数据,取平均值作为初始值
  for (uint8_t i = 0; i < 100; ++i) {
    BMI088Acquire(bmi088_device, &bmi088_data);
    acc_init[X] += bmi088_data.acc[X];
    acc_init[Y] += bmi088_data.acc[Y];
    acc_init[Z] += bmi088_data.acc[Z];
    DWT_Delay(0.001);
  }
  for (uint8_t i = 0; i < 3; ++i) acc_init[i] /= 100;
  Norm3d(acc_init);
  // 计算原始加速度矢量和导航系重力加速度矢量的夹角
  float angle = acosf(Dot3d(acc_init, gravity_norm));
  Cross3d(acc_init, gravity_norm, axis_rot);
  Norm3d(axis_rot);
  init_q4[0] = cosf(angle / 2.0f);
  for (uint8_t i = 0; i < 2; ++i) init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f);  // 轴角公式,第三轴为0(没有z轴分量)
}

/**
 * @brief reserved.IMU修正函数，用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 * @param[in,out] param IMU参数结构体指针
 * @param[in,out] gyro  角速度数组指针
 * @param[in,out] accel 加速度数组指针
 * @note  通过修正矩阵对陀螺仪和加速度计的测量值进行校正
 *        当安装角度或标度因子发生变化时重新计算修正矩阵
 */
static void IMU_Param_Correction(IMU_Init_Config_s* param, float gyro[3], float accel[3]) {
  static float lastYawOffset, lastPitchOffset, lastRollOffset; // 存储上一次设置的偏航角、俯仰角、横滚角
  static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33; // 旋转矩阵的3*3=9个元素
  float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

  // 和上一次设置的偏航角、俯仰角、横滚角的变化大小超过0.001°，或flag是1时重新计算修正矩阵。
  if (fabsf(param->Yaw - lastYawOffset) > 0.001f || fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
      fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag) {
    cosYaw = arm_cos_f32(param->Yaw / 57.295779513f); // 将角度转换为弧度（除以57.295779513，即180/π）并计算对应的正余弦值。
    cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
    cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
    sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
    sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
    sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

    // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
    // 旋转矩阵计算，将传感器坐标系中的数据转换到机体坐标系中。这个矩阵是基于ZYX顺序（偏航-俯仰-横滚）的欧拉角旋转矩阵。
    // 计算完成后，将参数更新标志位清零。
    c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
    c_12 = cosPitch * sinYaw;
    c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
    c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
    c_22 = cosYaw * cosPitch;
    c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
    c_31 = -cosPitch * sinRoll;
    c_32 = sinPitch;
    c_33 = cosPitch * cosRoll;
    param->flag = 0;
  }

  // 应用标度因数校正
  // 将陀螺仪数据与标度因数相乘进行校正，然后通过旋转矩阵将校正后的陀螺仪数据从传感器坐标系转换到机体坐标系。
  float gyro_temp[3];
  for (uint8_t i = 0; i < 3; ++i) gyro_temp[i] = gyro[i] * param->scale[i];

  gyro[X] = c_11 * gyro_temp[X] + c_12 * gyro_temp[Y] + c_13 * gyro_temp[Z];
  gyro[Y] = c_21 * gyro_temp[X] + c_22 * gyro_temp[Y] + c_23 * gyro_temp[Z];
  gyro[Z] = c_31 * gyro_temp[X] + c_32 * gyro_temp[Y] + c_33 * gyro_temp[Z];

  // 对加速度计数据进行同样处理
  // 对加速度计数据也应用相同的旋转矩阵进行坐标变换。
  float accel_temp[3];
  for (uint8_t i = 0; i < 3; ++i) accel_temp[i] = accel[i];

  accel[X] = c_11 * accel_temp[X] + c_12 * accel_temp[Y] + c_13 * accel_temp[Z];
  accel[Y] = c_21 * accel_temp[X] + c_22 * accel_temp[Y] + c_23 * accel_temp[Z];
  accel[Z] = c_31 * accel_temp[X] + c_32 * accel_temp[Y] + c_33 * accel_temp[Z];

  // 保存当前的偏航角、俯仰角和横滚角，供下次调用时比较。
  lastYawOffset = param->Yaw;
  lastPitchOffset = param->Pitch;
  lastRollOffset = param->Roll;
}

__attribute__((noreturn)) void StartINSTASK(void const *argument) {
  static float ins_start;
  static float ins_dt;
  LOGINFO("[freeRTOS] INS Task Start");
  for (;;) {
    // 1kHz
    ins_start = DWT_GetTimeline_ms();// 获取当前时间
    INS_Task();// INS任务
    ins_dt = DWT_GetTimeline_ms() - ins_start; // 计算INS任务执行时间
    if (ins_dt > 1) LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%f]", &ins_dt);// 如果INS任务执行时间超过1ms，则打印错误日志
    // VisionSend();  // 解算完成后发送视觉数据,但是当前的实现不太优雅,后续若添加硬件触发需要重新考虑结构的组织
    osDelay(1);// 1khz运行这个任务
  }
}

/**
 * @brief 初始化惯导解算系统，使用指定参数
 * @param[in] imu_param IMU参数配置指针
 * @retval attitude_t* 返回指向姿态数据的指针
 * @note  初始化内容包括：BMI088硬件配置、IMU参数设置、四元数初始化、加速度低通滤波系数设置等
 * 创建默认的IMU参数配置并调用INS_Param_Init进行初始化
 */
attitude_t* INS_Init(IMU_Init_Config_s* imu_init_config) {
  if (!INS.init)
    INS.init = 1;
  else
    return (attitude_t*)&INS.Gyro;

#ifdef STM32F407xx
  BMI088_Init_Config_s bmi088_config = {
      .spi_acc_config = {.spi_handle = &hspi1, .cs_pin = ACC_CS_Pin, .GPIOx = ACC_CS_GPIO_Port},
      .spi_gyro_config = {.spi_handle = &hspi1, .cs_pin = GYRO_CS_Pin, .GPIOx = GYRO_CS_GPIO_Port},
      .heat_pwm_config = {.htim = &htim10, .channel = TIM_CHANNEL_1},
      .heat_pid_config =
          {.MaxOut = 2000, .IntegralLimit = 300, .DeadBand = 0, .Kp = 1000, .Ki = 20, .Kd = 0, .Improve = 0x01},
      .work_mode = BMI088_BLOCK_PERIODIC_MODE,
      .cali_mode = BMI088_LOAD_PRE_CALI_MODE,
  };
#elifdef STM32H723xx
  // 初始化BMI088
  BMI088_Init_Config_s bmi088_config = {
      .spi_acc_config = {.spi_handle = &hspi2, .cs_pin = ACC_CS_Pin, .GPIOx = ACC_CS_GPIO_Port},
      .spi_gyro_config = {.spi_handle = &hspi2, .cs_pin = GYRO_CS_Pin, .GPIOx = GYRO_CS_GPIO_Port},
      .heat_pwm_config = {.htim = &htim3, .channel = TIM_CHANNEL_4},
      .heat_pid_config =
          {.MaxOut = 2000, .IntegralLimit = 300, .DeadBand = 0, .Kp = 1000, .Ki = 20, .Kd = 0, .Improve = 0x01},
      .work_mode = BMI088_BLOCK_PERIODIC_MODE,
      .cali_mode = BMI088_LOAD_PRE_CALI_MODE,
  };
#endif
  // 注册BMI088设备
  bmi088_device = BMI088Register(&bmi088_config);

  // 使用传入的IMU参数配置
  IMU_Param.scale[X] = imu_init_config->scale[X];
  IMU_Param.scale[Y] = imu_init_config->scale[Y];
  IMU_Param.scale[Z] = imu_init_config->scale[Z];
  IMU_Param.Yaw = imu_init_config->Yaw;
  IMU_Param.Pitch = imu_init_config->Pitch;
  IMU_Param.Roll = imu_init_config->Roll;
  IMU_Param.flag = imu_init_config->flag;

  float init_quaternion[4] = {0};
  InitQuaternion(init_quaternion);
  IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);

  // noise of accel is relatively big and of high freq,thus lpf is used
  INS.AccelLPF = 0.0085;
  DWT_GetDeltaT(&INS_DWT_Count);
  
  // 创建INS任务
  osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
  insTaskHandle = osThreadCreate(osThread(instask), NULL);
  
  return (attitude_t*)&INS.Gyro;  // @todo: 这里偷懒了,不要这样做! 修改INT_t结构体可能会导致异常,待修复.
}

/**
 * @attention 注意以1kHz的频率运行此任务
 * @brief INS任务主函数，在实时系统中以1kHz频率周期性调用
 * @note  主要完成以下工作：
 *        1. 获取时间间隔dt
 *        2. 读取BMI088陀螺仪和加速度计数据
 *        3. 对原始数据进行误差修正
 *        4. 使用扩展卡尔曼滤波器(EKF)进行姿态解算
 *        5. 坐标系变换计算
 *        6. 运动加速度计算
 *        7. IMU温度控制(500Hz)
 */
void INS_Task(void) {
  static uint32_t count = 0;
  const float gravity[3] = {0, 0, 9.81f};

  dt = DWT_GetDeltaT(&INS_DWT_Count);
  t += dt;

  // ins update
  if (BMI088Acquire(bmi088_device, &bmi088_data)) {
    // 更新INS数据
    INS.Accel[X] = bmi088_data.acc[X];
    INS.Accel[Y] = bmi088_data.acc[Y];
    INS.Accel[Z] = bmi088_data.acc[Z];
    INS.Gyro[X] = bmi088_data.gyro[X];
    INS.Gyro[Y] = bmi088_data.gyro[Y];
    INS.Gyro[Z] = bmi088_data.gyro[Z];

    // 用于修正安装误差（修改方式见函数注释&ins_task.md）
    IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

    // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
    // INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
    // INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(xb, INS.xn, INS.q);
    BodyFrameToEarthFrame(yb, INS.yn, INS.q);
    BodyFrameToEarthFrame(zb, INS.zn, INS.q);

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; ++i)  // 同样过一个低通滤波
    {
      INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) +
                             INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
    }
    BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);  // 转换回导航系n

    INS.Yaw = QEKF_INS.Yaw;
    INS.Pitch = QEKF_INS.Pitch;
    INS.Roll = QEKF_INS.Roll;
    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

    // VisionSetAltitude(INS.Yaw, INS.Pitch, INS.Roll);  TODO: 待完善
  }

  // temperature control
  if ((count % 2) == 0) {
    // 500hz
    IMU_Temperature_Ctrl();
  }

  if ((count++ % 1000) == 0) {
    // 1Hz 可以加入monitor函数,检查IMU是否正常运行/离线
  }
}

/**
 * @brief Transform 3dvector from BodyFrame to EarthFrame 机体坐标系向量转换到导航坐标系(地球坐标系)
 * @param[in]  vecBF vector in BodyFrame 机体坐标系下的向量
 * @param[out] vecEF vector in EarthFrame 导航坐标系下的向量
 * @param[in]  q     quaternion 四元数姿态信息
 * @note  使用四元数进行坐标变换，避免万向节死锁问题
 */
void BodyFrameToEarthFrame(const float* vecBF, float* vecEF, float* q) {
  vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] + (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                     (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

  vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                     (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

  vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] + (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                     (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief Transform 3dvector from EarthFrame to BodyFrame导航坐标系(地球坐标系)向量转换到机体坐标系
 * @param[in]  vecEF vector in EarthFrame 导航坐标系下的向量
 * @param[out] vecBF vector in BodyFrame 机体坐标系下的向量
 * @param[in]  q     quaternion 四元数姿态信息
 * @note  使用四元数进行坐标变换，避免万向节死锁问题
 */
void EarthFrameToBodyFrame(const float* vecEF, float* vecBF, float* q) {
  vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] + (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                     (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

  vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                     (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

  vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] + (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                     (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

//------------------------------------functions below are not used in this
// demo-------------------------------------------------
//----------------------------------you can read them for learning or
// programming-----------------------------------------------
//----------------------------------they could also be helpful for further
// design-----------------------------------------------

/**
 * @brief Update quaternion 四元数更新函数,实现微分方程 dq/dt=0.5Ωq 的数值积分
 * @param[in,out] q  四元数数组指针，长度为4
 * @param[in] gx X轴角速度 单位: rad/s
 * @param[in] gy Y轴角速度 单位: rad/s
 * @param[in] gz Z轴角速度 单位: rad/s
 * @param[in] dt 时间间隔 单位: s
 * @note  使用一阶龙格-库塔法进行数值积分更新四元数
 */
void QuaternionUpdate(float* q, float gx, float gy, float gz, float dt) {
  float qa, qb, qc;

  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief Convert quaternion to eular angle 四元数转换成欧拉角(ZYX顺序)
 * @param[in]  q     输入的四元数数组指针
 * @param[out] Yaw   输出的偏航角指针 单位: °
 * @param[out] Pitch 输出的俯仰角指针 单位: °
 * @param[out] Roll  输出的横滚角指针 单位: °
 * @note  使用ZYX旋转顺序进行转换
 */
void QuaternionToEularAngle(float* q, float* Yaw, float* Pitch, float* Roll) {
  *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
  *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
  *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief Convert eular angle to quaternion ZYX欧拉角转换为四元数
 * @param[in]  Yaw   偏航角 单位: °
 * @param[in]  Pitch 俯仰角 单位: °
 * @param[in]  Roll  横滚角 单位: °
 * @param[out] q     输出的四元数数组指针
 * @note  使用ZYX旋转顺序进行转换
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float* q) {
  float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
  Yaw /= 57.295779513f;
  Pitch /= 57.295779513f;
  Roll /= 57.295779513f;
  cosPitch = arm_cos_f32(Pitch / 2);
  cosYaw = arm_cos_f32(Yaw / 2);
  cosRoll = arm_cos_f32(Roll / 2);
  sinPitch = arm_sin_f32(Pitch / 2);
  sinYaw = arm_sin_f32(Yaw / 2);
  sinRoll = arm_sin_f32(Roll / 2);
  q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
  q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
  q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
  q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}