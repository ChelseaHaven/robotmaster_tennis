#include "bsp_host_uart.h"
#include "usart.h"
#include <string.h>

#define RX_DMA_SIZE 512U
#define RX_RING_SIZE 2048U
#define TX_BUFFER_SIZE 137U
static uint8_t rx_dma[RX_DMA_SIZE] __attribute__((aligned(4)));
static uint8_t rx_ring[RX_RING_SIZE];
static uint8_t tx_buffer[TX_BUFFER_SIZE] __attribute__((aligned(4)));
static uint16_t dma_position;
static volatile uint16_t head, tail;
static volatile uint8_t fault, overflow, tx_busy;
static uint8_t initialized;

uint8_t bsp_host_uart_init(void)
{
    head = tail = dma_position = 0;
    overflow = tx_busy = 0;
    initialized = 1;
    fault = HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_dma, RX_DMA_SIZE) != HAL_OK;
    return !fault;
}

/**
  * @brief          将 USART6 DMA 新增数据搬入接收环形缓冲区
  * @param[in]      uart: 串口句柄
  * @param[in]      size: HAL 事件位置，本实现改读 DMA 实时游标
  * @retval         无
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    (void)size;
    if (uart != &huart6 || !initialized || fault) return;
    /* 读取实时 DMA 游标，避免空闲事件与延迟的半满事件重复搬运。 */
    uint16_t position = (RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx)) % RX_DMA_SIZE;
    while (dma_position != position) {
        uint16_t next = (head + 1U) % RX_RING_SIZE;
        if (next == tail) overflow = 1;
        else if (!overflow) {
            rx_ring[head] = rx_dma[dma_position];
            __DMB();
            head = next;
        }
        dma_position = (dma_position + 1U) % RX_DMA_SIZE;
    }
}

/**
  * @brief          在 USART6 发送完成后释放发送缓冲区
  * @param[in]      uart: 串口句柄
  * @retval         无
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart6) tx_busy = 0;
}

/**
  * @brief          记录 USART6 错误，交由通信任务恢复
  * @param[in]      uart: 串口句柄
  * @retval         无
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart6) fault = 1;
}

uint8_t bsp_host_uart_service(void)
{
    if (fault) {
        /* HAL 的停止与重启在任务上下文执行，失败则下次继续尝试。 */
        HAL_UART_Abort(&huart6);
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        head = tail = dma_position = 0;
        overflow = tx_busy = 0;
        fault = 0;
        __set_PRIMASK(mask);
        if (HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_dma, RX_DMA_SIZE) != HAL_OK)
            fault = 1;
        return 1;
    }
    if (overflow) {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        tail = head;
        overflow = 0;
        __set_PRIMASK(mask);
        return 1;
    }
    return 0;
}
uint16_t bsp_host_uart_read(uint8_t *data, uint16_t capacity)
{
    uint16_t count = 0;
    while (count < capacity && tail != head && !overflow && !fault) {
        data[count++] = rx_ring[tail];
        __DMB();
        tail = (tail + 1U) % RX_RING_SIZE;
    }
    return count;
}
uint8_t bsp_host_uart_tx_busy(void) { return tx_busy || fault; }
uint8_t bsp_host_uart_send(const uint8_t *data, uint16_t length)
{
    if (!data || !length || length > TX_BUFFER_SIZE || bsp_host_uart_tx_busy()) return 0;
    memcpy(tx_buffer, data, length);
    tx_busy = 1;
    if (HAL_UART_Transmit_DMA(&huart6, tx_buffer, length) != HAL_OK) {
        tx_busy = 0;
        fault = 1;
        return 0;
    }
    return 1;
}
