#ifndef BSP_HOST_UART_H
#define BSP_HOST_UART_H
#include <stdint.h>
/* 本模块独占 USART6；以下接口只由通信任务调用。 */
uint8_t bsp_host_uart_init(void);
/* 错误恢复或软件缓冲溢出时返回 1，调用方应清除半帧。 */
uint8_t bsp_host_uart_service(void);
uint16_t bsp_host_uart_read(uint8_t *data, uint16_t capacity);
uint8_t bsp_host_uart_tx_busy(void);
uint8_t bsp_host_uart_send(const uint8_t *data, uint16_t length);
#endif
