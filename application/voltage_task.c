/* 版权所有 (C) 2019 DJI。
 * 移植自 Development-Board-C-Examples/20.standard_robot，保留原算法。
 */
#include <stdio.h>
#include "voltage_task.h"
#include "main.h"
#include "cmsis_os.h"

#include "bsp_adc.h"

#define FULL_BATTER_VOLTAGE     25.2f
#define LOW_BATTER_VOLTAGE      22.2f

#define VOLTAGE_DROP            0.00f

static fp32 calc_battery_percentage(float voltage);

fp32 battery_voltage;
fp32 electricity_percentage;

void battery_voltage_task(void const * argument)
{
    osDelay(1000);

    init_vrefint_reciprocal();
    while(1)
    {
        battery_voltage = get_battery_voltage() + VOLTAGE_DROP;
        electricity_percentage = calc_battery_percentage(battery_voltage);
        osDelay(100);
    }
}

static fp32 calc_battery_percentage(float voltage)
{
    fp32 percentage;
    fp32 voltage_2 = voltage * voltage;
    fp32 voltage_3 = voltage_2 * voltage;

    if(voltage < 19.5f)
    {
        percentage = 0.0f;
    }
    else if(voltage < 21.9f)
    {
        percentage = 0.005664f * voltage_3 - 0.3386f * voltage_2 + 6.765f * voltage - 45.17f;
    }
    else if(voltage < 25.5f)
    {
        percentage = 0.02269f * voltage_3 - 1.654f * voltage_2 + 40.34f * voltage - 328.4f;
    }
    else
    {
        percentage = 1.0f;
    }
    if(percentage < 0.0f)
    {
        percentage = 0.0f;
    }
    else if(percentage > 1.0f)
    {
        percentage = 1.0f;
    }

    return percentage;
}

uint16_t get_battery_percentage(void)
{
    return (uint16_t)(electricity_percentage * 100.0f);
}
