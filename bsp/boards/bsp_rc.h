#ifndef BSP_RC_H
#define BSP_RC_H

#include <stdint.h>

#define RC_DMA_BUFFER_SIZE 50U
#define RC_RECEIVE_ERROR_UART 1U
#define RC_RECEIVE_ERROR_DMA 2U
#define RC_RECEIVE_ERROR_CURSOR 4U
#define RC_RECEIVE_ERROR_RESTART 8U

/* 回调运行在中断中：接收回调的数据只在本次调用期间有效。 */
typedef void (*rc_receive_callback_t)(volatile const uint8_t *data, uint16_t length);
typedef void (*rc_receive_error_callback_t)(uint32_t reason);

/**
  * @brief          注册遥控器接收回调并启动双缓冲 DMA
  * @param[in]      receive: 新增数据回调，在中断中调用
  * @param[in]      error: 接收故障回调，在中断或重启失败时调用
  * @retval         1：启动成功；0：启动失败
  */
uint8_t RC_Init(rc_receive_callback_t receive, rc_receive_error_callback_t error);

/**
  * @brief          清除接收状态并重新启动遥控器双缓冲 DMA
  * @retval         1：重启成功；0：DMA 未能停止
  */
uint8_t RC_restart(void);

/**
  * @brief          查询遥控器 DMA 是否需要恢复
  * @retval         1：需要重启；0：无需重启
  */
uint8_t RC_restart_is_needed(void);

/**
  * @brief          处理遥控器串口空闲和错误中断
  * @retval         无
  */
void RC_uart_irq_handler(void);

/**
  * @brief          处理遥控器 DMA 完成和错误中断
  * @retval         无
  */
void RC_dma_irq_handler(void);

#endif
