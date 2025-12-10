/**
 * @file bsp_adc.h
 * @author your name (you@domain.com)
 * @brief ADC板级支持包头文件
 * @version 0.1
 * @date 2023-02-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "adc.h"
#include "stdint.h"

#define ADC_DEVICE_CNT 8  // 最大支持的ADC实例数量

/* ADC模式枚举 */
typedef enum {
  ADC_MODE_POLLING,  // 轮询模式
  ADC_MODE_IT,       // 中断模式
  ADC_MODE_DMA,      // DMA模式
} ADC_MODE_e;

/* ADC实例结构体 */
typedef struct adc_ins_temp {
  ADC_HandleTypeDef *hadc;                 // ADC句柄
  uint32_t channel;                        // ADC通道
  ADC_MODE_e mode;                         // ADC工作模式
  float vref;                              // 参考电压(V)
  uint32_t resolution;                     // ADC分辨率对应的最大值,如12位ADC为4096
  uint16_t raw_value;                      // ADC原始值
  float voltage;                           // 转换后的电压值(V)
  void (*callback)(struct adc_ins_temp *); // 转换完成回调函数
  void *id;                                // 实例ID
  
  // EWMA滤波相关成员
  uint16_t filtered_raw_value;             // 滤波后的ADC原始值
  float alpha;                             // EWMA滤波系数 (0.0 - 1.0)
  uint8_t is_first_value;                  // 是否为第一个值标志
} ADCInstance;

/* ADC初始化配置结构体 */
typedef struct {
  ADC_HandleTypeDef *hadc;         // ADC句柄
  uint32_t channel;                // ADC通道
  ADC_MODE_e mode;                 // ADC工作模式
  float vref;                      // 参考电压(V),默认3.3V
  float alpha;                     // EWMA滤波系数 (0.0 - 1.0)，默认0.0表示不使用滤波
  void (*callback)(ADCInstance *); // 转换完成回调函数
  void *id;                        // 实例ID
} ADC_Init_Config_s;

/**
 * @brief 注册一个ADC实例
 *
 * @param config 初始化配置
 * @return ADCInstance* ADC实例指针
 */
ADCInstance *ADCRegister(ADC_Init_Config_s *config);

/**
 * @brief 启动ADC转换(轮询模式)
 *
 * @param adc ADC实例
 * @return float 转换后的电压值(V)
 */
float ADCGetVoltage(ADCInstance *adc);

/**
 * @brief 获取ADC原始值(轮询模式)
 *
 * @param adc ADC实例
 * @return uint16_t ADC原始值
 */
uint16_t ADCGetRawValue(ADCInstance *adc);

/**
 * @brief 获取经过EWMA滤波的ADC原始值(轮询模式)
 *
 * @param adc ADC实例
 * @return uint16_t 滤波后的ADC原始值
 */
uint16_t ADCGetFilteredRawValue(ADCInstance *adc);

/**
 * @brief 设置EWMA滤波系数
 *
 * @param adc ADC实例
 * @param alpha 滤波系数 (0.0 - 1.0)
 */
void ADCSetFilterAlpha(ADCInstance *adc, float alpha);

/**
 * @brief 启动ADC中断模式转换
 *
 * @param adc ADC实例
 */
void ADCStartIT(ADCInstance *adc);

/**
 * @brief 停止ADC中断模式转换
 *
 * @param adc ADC实例
 */
void ADCStopIT(ADCInstance *adc);

/**
 * @brief 启动ADC DMA模式转换
 *
 * @param adc ADC实例
 * @param pData 数据缓冲区地址
 * @param Length 数据长度
 */
void ADCStartDMA(ADCInstance *adc, uint32_t *pData, uint32_t Length);

/**
 * @brief 停止ADC DMA模式转换
 *
 * @param adc ADC实例
 */
void ADCStopDMA(ADCInstance *adc);

/**
 * @brief 将ADC原始值转换为电压值
 *
 * @param adc ADC实例
 * @param raw_value ADC原始值
 * @return float 电压值(V)
 */
float ADCRawToVoltage(ADCInstance *adc, uint16_t raw_value);

#endif // BSP_ADC_H