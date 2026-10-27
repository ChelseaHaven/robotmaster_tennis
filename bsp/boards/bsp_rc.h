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

/* 独占 USART3 和 DMA1 Stream1；不要同时使用 HAL 的 UART 接收或中断处理接口。 */
uint8_t RC_Init(rc_receive_callback_t receive, rc_receive_error_callback_t error);
/* 在主函数或任务中调用，有界等待 DMA 停止后重新启动。 */
uint8_t RC_restart(void);
uint8_t RC_restart_is_needed(void);
void RC_uart_irq_handler(void);
void RC_dma_irq_handler(void);

#endif
