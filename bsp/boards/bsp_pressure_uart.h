#ifndef BSP_PRESSURE_UART_H
#define BSP_PRESSURE_UART_H
#include <stdint.h>
/** @brief 初始化 USART1 接收缓冲区并开启逐字节中断接收。 */
void bsp_pressure_uart_init(void);
/** @brief 在任务中恢复接收故障；返回 1 表示发生丢帧，应用需重置解析状态。 */
uint8_t bsp_pressure_uart_service(void);
/** @brief 读取一个缓存字节；成功返回 1，空缓存返回 0。 @param[out] byte 字节地址 */
uint8_t bsp_pressure_uart_read(uint8_t *byte);
/** @brief 阻塞发送请求帧；成功返回 1。 @param[in] data 数据 @param[in] length 字节数 */
uint8_t bsp_pressure_uart_send(const uint8_t *data, uint16_t length);
/** @brief 由 HAL 串口错误回调通知 USART1 故障。 */
void bsp_pressure_uart_error(void);
#endif
