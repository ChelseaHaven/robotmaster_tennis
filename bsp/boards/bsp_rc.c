/* 基于大疆 C 板例程的 USART3 双缓冲 DMA 接收层。
 * 空闲中断不停止 DMA；空闲和传输完成中断共享读取位置，避免丢帧或重复解析。
 * 只负责接收与故障恢复，通过回调交给上层解析，不包含遥控器业务控制。 */
#include "bsp_rc.h"
#include "usart.h"

extern DMA_HandleTypeDef hdma_usart3_rx;

static volatile uint8_t rx_buffers[2][RC_DMA_BUFFER_SIZE] __attribute__((aligned(4)));
static uint16_t read_position;
static volatile uint8_t running;
static volatile uint8_t restart_needed;
static rc_receive_callback_t receive_callback;
static rc_receive_error_callback_t error_callback;

#define RC_DMA_ALL_FLAGS (DMA_LISR_TCIF1 | DMA_LISR_HTIF1 | DMA_LISR_TEIF1 | DMA_LISR_DMEIF1 | DMA_LISR_FEIF1)
#define RC_DMA_ERROR_FLAGS (DMA_LISR_TEIF1 | DMA_LISR_DMEIF1 | DMA_LISR_FEIF1)
#define RC_UART_ERROR_FLAGS (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)

static void stop_receive(void)
{
    CLEAR_BIT(huart3.Instance->CR3, USART_CR3_DMAR | USART_CR3_EIE);
    CLEAR_BIT(huart3.Instance->CR1, USART_CR1_IDLEIE | USART_CR1_PEIE | USART_CR1_RXNEIE);
    CLEAR_BIT(hdma_usart3_rx.Instance->CR,
              DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
    CLEAR_BIT(hdma_usart3_rx.Instance->FCR, DMA_SxFCR_FEIE);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    running = 0;
}

static void receive_error(uint32_t reason)
{
    stop_receive();
    restart_needed = 1;
    __HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx, RC_DMA_ALL_FLAGS);
    __HAL_UART_CLEAR_PEFLAG(&huart3);
    if (error_callback != 0) error_callback(reason);
}

uint8_t RC_restart(void)
{
    uint32_t saved_mask = __get_PRIMASK();
    __disable_irq();
    stop_receive();
    /* 此处也会在调度器启动前运行；关闭中断时采用有界轮询，避免依赖时基而死等。 */
    unsigned polls = 0;
    while ((hdma_usart3_rx.Instance->CR & DMA_SxCR_EN) != 0U && polls < 32U)
        ++polls;
    if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_EN) != 0U)
    {
        receive_error(RC_RECEIVE_ERROR_RESTART);
        __set_PRIMASK(saved_mask);
        return 0;
    }

    CLEAR_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_CT);
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM | DMA_SxCR_CIRC);
    hdma_usart3_rx.Instance->PAR = (uint32_t)&huart3.Instance->DR;
    hdma_usart3_rx.Instance->M0AR = (uint32_t)rx_buffers[0];
    hdma_usart3_rx.Instance->M1AR = (uint32_t)rx_buffers[1];
    hdma_usart3_rx.Instance->NDTR = RC_DMA_BUFFER_SIZE;
    read_position = 0;
    __HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx, RC_DMA_ALL_FLAGS);
    __HAL_UART_CLEAR_PEFLAG(&huart3);
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
    SET_BIT(hdma_usart3_rx.Instance->FCR, DMA_SxFCR_FEIE);
    restart_needed = 0;
    running = 1;
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR | USART_CR3_EIE);
    SET_BIT(huart3.Instance->CR1, USART_CR1_IDLEIE | USART_CR1_PEIE);
    __set_PRIMASK(saved_mask);
    return 1;
}

uint8_t RC_Init(rc_receive_callback_t receive, rc_receive_error_callback_t error)
{
    receive_callback = receive;
    error_callback = error;
    return RC_restart();
}

uint8_t RC_restart_is_needed(void)
{
    return restart_needed;
}

static void drain_receive(void)
{
    uint32_t uart_status = huart3.Instance->SR;
    uint32_t dma_status = __HAL_DMA_GET_FLAG(&hdma_usart3_rx, RC_DMA_ALL_FLAGS);
    if (!running)
    {
        __HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx, RC_DMA_ALL_FLAGS);
        __HAL_UART_CLEAR_PEFLAG(&huart3);
        return;
    }
    if ((uart_status & RC_UART_ERROR_FLAGS) != 0U || (dma_status & RC_DMA_ERROR_FLAGS) != 0U)
    {
        receive_error(((uart_status & RC_UART_ERROR_FLAGS) ? RC_RECEIVE_ERROR_UART : 0U) |
                      ((dma_status & RC_DMA_ERROR_FLAGS) ? RC_RECEIVE_ERROR_DMA : 0U));
        return;
    }

    if ((uart_status & USART_SR_IDLE) != 0U)
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx, dma_status);
    if ((uart_status & USART_SR_IDLE) == 0U && (dma_status & DMA_LISR_TCIF1) == 0U)
        return;

    /* 读取期间 DMA 可能切换缓冲区，重试以保证缓冲区编号和剩余长度相互对应。 */
    uint32_t bank = 0, remaining = 0;
    unsigned attempt;
    for (attempt = 0; attempt < 3U; ++attempt)
    {
        bank = hdma_usart3_rx.Instance->CR & DMA_SxCR_CT;
        remaining = hdma_usart3_rx.Instance->NDTR;
        if (bank == (hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) && remaining <= RC_DMA_BUFFER_SIZE)
            break;
    }
    if (attempt == 3U)
    {
        receive_error(RC_RECEIVE_ERROR_CURSOR);
        return;
    }
    uint16_t position = (bank ? RC_DMA_BUFFER_SIZE : 0U) + RC_DMA_BUFFER_SIZE - remaining;
    if (position == 2U * RC_DMA_BUFFER_SIZE) position = 0;
    __DMB();
    /* 两个中断均为优先级 6，读取实时写入位置，避免延迟的传输完成中断重复提交数据。 */
    while (read_position != position)
    {
        uint16_t offset = read_position % RC_DMA_BUFFER_SIZE;
        uint16_t length = RC_DMA_BUFFER_SIZE - offset;
        uint16_t available = (position + 2U * RC_DMA_BUFFER_SIZE - read_position) %
                              (2U * RC_DMA_BUFFER_SIZE);
        if (length > available) length = available;
        if (receive_callback != 0)
            receive_callback(&rx_buffers[read_position / RC_DMA_BUFFER_SIZE][offset], length);
        read_position = (read_position + length) % (2U * RC_DMA_BUFFER_SIZE);
    }
}

void RC_uart_irq_handler(void)
{
    drain_receive();
}

void RC_dma_irq_handler(void)
{
    drain_receive();
}
