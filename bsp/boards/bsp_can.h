#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"


/**
  * @brief          配置 CAN1 接收过滤器并启动接收中断
  * @retval         无
  */
extern void can_filter_init(void);

#endif
