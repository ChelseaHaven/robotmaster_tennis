#ifndef BSP_DELAY_H
#define BSP_DELAY_H
#include "struct_typedef.h"

/**
  * @brief          初始化微秒延时使用的处理器周期计数器
  * @retval         无
  */
extern void delay_init(void);

/**
  * @brief          通过处理器周期计数器忙等指定微秒数
  * @param[in]      nus: 延时时间，单位 μs
  * @retval         无
  */
extern void delay_us(uint16_t nus);

/**
  * @brief          通过微秒忙等实现毫秒延时
  * @param[in]      nms: 延时时间，单位 ms
  * @retval         无
  */
extern void delay_ms(uint16_t nms);
#endif
