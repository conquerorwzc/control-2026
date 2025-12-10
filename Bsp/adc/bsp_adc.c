#include "bsp_adc.h"

#include "memory.h"
#include "stdlib.h"
#include "math.h"

// 配合中断以及初始化
static uint8_t idx;
static ADCInstance* adc_instance[ADC_DEVICE_CNT] = {NULL};  // 所有的ADC instance保存于此,用于callback时判断中断来源

/**
 * @brief 获取ADC分辨率对应的最大值
 *
 * @param hadc ADC句柄
 * @return uint32_t 分辨率对应的最大值
 */
static uint32_t ADCGetResolution(ADC_HandleTypeDef* hadc) {
#ifdef STM32F407xx
  switch (hadc->Init.Resolution) {
    case ADC_RESOLUTION_12B:
      return 4096;
    case ADC_RESOLUTION_10B:
      return 1024;
    case ADC_RESOLUTION_8B:
      return 256;
    case ADC_RESOLUTION_6B:
      return 64;
    default:
      return 4096;
  }
#elifdef STM32H723xx
  switch (hadc->Init.Resolution) {
    case ADC_RESOLUTION_16B:
      return 65536;
    case ADC_RESOLUTION_14B:
      return 16384;
    case ADC_RESOLUTION_12B:
      return 4096;
    case ADC_RESOLUTION_10B:
      return 1024;
    case ADC_RESOLUTION_8B:
      return 256;
    default:
      return 65536;
  }
#else
  return 4096;  // 默认12位分辨率
#endif
}

/**
 * @brief 应用EWMA滤波
 *
 * @param adc ADC实例
 * @param raw_value 新的原始值
 * @return uint16_t 滤波后的值
 */
static uint16_t ADCApplyEWMAFilter(ADCInstance* adc, uint16_t raw_value) {
  // 如果alpha为0或者未启用滤波，直接返回原始值
  if (adc->alpha <= 0.0f) {
    return raw_value;
  }

  // 限制alpha在有效范围内
  float alpha = adc->alpha;
  if (alpha > 1.0f) {
    alpha = 1.0f;
  }

  // 如果是第一个值，直接使用该值作为初始值
  if (adc->is_first_value) {
    adc->filtered_raw_value = raw_value;
    adc->is_first_value = 0;
    return raw_value;
  }

  // 应用EWMA滤波公式: y[n] = α * x[n] + (1-α) * y[n-1]
  float filtered_value = alpha * (float)raw_value + (1.0f - alpha) * (float)adc->filtered_raw_value;
  adc->filtered_raw_value = (uint16_t)filtered_value;
  
  return adc->filtered_raw_value;
}

/**
 * @brief ADC转换完成回调函数
 *
 * @param hadc 发生中断的ADC句柄
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  for (uint8_t i = 0; i < idx; i++) {
    if (adc_instance[i]->hadc == hadc) {
      adc_instance[i]->raw_value = HAL_ADC_GetValue(hadc);
      // 应用EWMA滤波
      adc_instance[i]->raw_value = ADCApplyEWMAFilter(adc_instance[i], adc_instance[i]->raw_value);
      adc_instance[i]->voltage = ADCRawToVoltage(adc_instance[i], adc_instance[i]->raw_value);
      if (adc_instance[i]->callback)  // 如果有回调函数
        adc_instance[i]->callback(adc_instance[i]);
      return;
    }
  }
}

ADCInstance* ADCRegister(ADC_Init_Config_s* config) {
  if (idx >= ADC_DEVICE_CNT)  // 超过最大实例数,考虑增加或查看是否有内存泄漏
    while (1);
  ADCInstance* adc = (ADCInstance*)malloc(sizeof(ADCInstance));
  memset(adc, 0, sizeof(ADCInstance));

  adc->hadc = config->hadc;
  adc->channel = config->channel;
  adc->mode = config->mode;
  adc->vref = config->vref > 0 ? config->vref : 3.3f;  // 默认参考电压3.3V
  adc->callback = config->callback;
  adc->id = config->id;
  adc->resolution = ADCGetResolution(adc->hadc);
  adc->raw_value = 0;
  adc->voltage = 0;
  adc->alpha = config->alpha > 0 ? config->alpha : 0.0f;  // 默认不使用滤波
  adc->is_first_value = 1;  // 标记为第一个值

  adc_instance[idx++] = adc;
  return adc;
}

/**
 * @brief 将ADC原始值转换为电压值
 *
 * @param adc ADC实例
 * @param raw_value ADC原始值
 * @return float 电压值(V)
 */
float ADCRawToVoltage(ADCInstance* adc, uint16_t raw_value) {
  return (float)raw_value * adc->vref / (float)adc->resolution;
}

/**
 * @brief 启动ADC转换(轮询模式)并获取电压值
 *
 * @param adc ADC实例
 * @return float 转换后的电压值(V)
 */
float ADCGetVoltage(ADCInstance* adc) {
  HAL_ADC_Start(adc->hadc);
  HAL_ADC_PollForConversion(adc->hadc, HAL_MAX_DELAY);
  adc->raw_value = HAL_ADC_GetValue(adc->hadc);
  // 应用EWMA滤波
  adc->raw_value = ADCApplyEWMAFilter(adc, adc->raw_value);
  adc->voltage = ADCRawToVoltage(adc, adc->raw_value);
  HAL_ADC_Stop(adc->hadc);
  return adc->voltage;
}

/**
 * @brief 获取ADC原始值(轮询模式)
 *
 * @param adc ADC实例
 * @return uint16_t ADC原始值
 */
uint16_t ADCGetRawValue(ADCInstance* adc) {
  HAL_ADC_Start(adc->hadc);
  HAL_ADC_PollForConversion(adc->hadc, HAL_MAX_DELAY);
  adc->raw_value = HAL_ADC_GetValue(adc->hadc);
  // 应用EWMA滤波
  adc->raw_value = ADCApplyEWMAFilter(adc, adc->raw_value);
  HAL_ADC_Stop(adc->hadc);
  return adc->raw_value;
}

/**
 * @brief 获取经过EWMA滤波的ADC原始值(轮询模式)
 *
 * @param adc ADC实例
 * @return uint16_t 滤波后的ADC原始值
 */
uint16_t ADCGetFilteredRawValue(ADCInstance* adc) {
  return adc->filtered_raw_value;
}

/**
 * @brief 设置EWMA滤波系数
 *
 * @param adc ADC实例
 * @param alpha 滤波系数 (0.0 - 1.0)
 */
void ADCSetFilterAlpha(ADCInstance* adc, float alpha) {
  if (alpha >= 0.0f && alpha <= 1.0f) {
    adc->alpha = alpha;
  }
}

/* 启动ADC中断模式转换 */
void ADCStartIT(ADCInstance* adc) { HAL_ADC_Start_IT(adc->hadc); }

/* 停止ADC中断模式转换 */
void ADCStopIT(ADCInstance* adc) { HAL_ADC_Stop_IT(adc->hadc); }

/* 启动ADC DMA模式转换 */
void ADCStartDMA(ADCInstance* adc, uint32_t* pData, uint32_t Length) { HAL_ADC_Start_DMA(adc->hadc, pData, Length); }

/* 停止ADC DMA模式转换 */
void ADCStopDMA(ADCInstance* adc) { HAL_ADC_Stop_DMA(adc->hadc); }