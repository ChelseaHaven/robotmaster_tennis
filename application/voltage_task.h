#ifndef VOLTAGE_TASK_H
#define VOLTAGE_TASK_H
#include "struct_typedef.h"

// 大疆原例程：每 100 ms 更新电压和估算电量百分比。
extern fp32 battery_voltage;
void battery_voltage_task(void const *argument);
uint16_t get_battery_percentage(void);
#endif
