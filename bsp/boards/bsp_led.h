#ifndef BSP_LED_H
#define BSP_LED_H
#include "struct_typedef.h"


/**
  * @brief          根据颜色和亮度设置 RGB 指示灯 PWM
  * @param[in]      aRGB: 0xAARRGGBB 格式；AA 缩放亮度，RR、GG、BB 为三色分量
  * @retval         无
  */
extern void aRGB_led_show(uint32_t aRGB);


#endif
