#ifndef BSP_ADC_H
#define BSP_ADC_H
#include "struct_typedef.h"

/* 大疆原例程接口：ADC1 内部参考校准、芯片温度、ADC3 电池电压。 */
void init_vrefint_reciprocal(void);
fp32 get_temprate(void);
fp32 get_battery_voltage(void);
#endif
