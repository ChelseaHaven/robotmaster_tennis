#ifndef VOLTAGE_TASK_H
#define VOLTAGE_TASK_H
#include "struct_typedef.h"

// 大疆原例程：每 100 ms 更新电压和估算电量百分比。
extern fp32 battery_voltage;

/**
  * @brief          周期采集电池电压并更新大疆算法估算的电量
  * @param[in]      argument: 任务参数，未使用
  * @note           启动延时 1 s 后校准内部参考电压，每 100 ms 更新一次。
  * @retval         无
  */
void battery_voltage_task(void const *argument);

/**
  * @brief          读取当前估算电量百分比
  * @retval         电量百分比，范围 0～100
  */
uint16_t get_battery_percentage(void);

/**
  * @brief          查询经过持续时间确认和回差处理的低电量报警状态
  * @note           电压不超过 10 V 时不报警；低于 5% 约 3 s 触发，达到 8% 约 1 s 解除。
  * @retval         1：低电量报警；0：无报警
  */
uint8_t battery_low_alarm_is_active(void);
#endif
