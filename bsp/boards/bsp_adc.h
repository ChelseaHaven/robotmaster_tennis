#ifndef BSP_ADC_H
#define BSP_ADC_H
#include "struct_typedef.h"

/**
  * @brief          采样内部参考电压并计算 ADC 电压换算系数
  * @retval         无
  */
void init_vrefint_reciprocal(void);

/**
  * @brief          读取 STM32 内部温度传感器并换算温度
  * @retval         芯片内部温度，单位 ℃
  */
fp32 get_temprate(void);

/**
  * @brief          读取 ADC3 电池分压并换算电池电压
  * @note           须先调用 init_vrefint_reciprocal。
  * @retval         电池电压，单位 V
  */
fp32 get_battery_voltage(void);
#endif
