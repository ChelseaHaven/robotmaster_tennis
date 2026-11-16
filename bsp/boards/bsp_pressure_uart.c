#include "bsp_pressure_uart.h"
#include "usart.h"

#define PRESSURE_UART_RING_SIZE 64U
static uint8_t rx_byte;
static uint8_t rx_ring[PRESSURE_UART_RING_SIZE];
static volatile uint16_t head, tail;
static volatile uint8_t fault;
static uint8_t initialized;

void bsp_pressure_uart_init(void)
{
    head = tail = 0;
    initialized = 1;
    fault = HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK;
}

/** @brief USART1 接收中断只缓存字节并重开接收，不解析协议。 @param[in] uart 串口句柄 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart1 || !initialized || fault) return;
    uint16_t next = (head + 1U) % PRESSURE_UART_RING_SIZE;
    if (next == tail) fault = 1;
    else {
        rx_ring[head] = rx_byte;
        __DMB();
        head = next;
    }
    if (!fault && HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK) fault = 1;
}

void bsp_pressure_uart_error(void)
{
    if (initialized) fault = 1;
}

uint8_t bsp_pressure_uart_service(void)
{
    if (!fault) return 0;
    HAL_UART_AbortReceive(&huart1);
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    head = tail = 0;
    fault = 0;
    __set_PRIMASK(mask);
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK) fault = 1;
    return 1;
}

uint8_t bsp_pressure_uart_read(uint8_t *byte)
{
    if (byte == NULL || tail == head) return 0;
    *byte = rx_ring[tail];
    __DMB();
    tail = (tail + 1U) % PRESSURE_UART_RING_SIZE;
    return 1;
}

uint8_t bsp_pressure_uart_send(const uint8_t *data, uint16_t length)
{
    return HAL_UART_Transmit(&huart1, (uint8_t *)data, length, 20U) == HAL_OK;
}
