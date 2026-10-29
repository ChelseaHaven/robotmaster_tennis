/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       led_trigger_task.c/h
  * @brief      led RGB show.led RGB灯效。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. rgb led
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef LED_TRIGGER_TASK_H
#define LED_TRIGGER_TASK_H


#include "struct_typedef.h"


/**
  * @brief          周期改变 RGB 灯颜色，形成渐变流水效果
  * @param[in]      argument: 任务参数，未使用
  * @retval         无
  */
extern void led_RGB_flow_task(void const * argument);

#endif
