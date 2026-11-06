#include "bsp_can_fifo.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define CAN_FIFO_TX_BURST 3U
#define CAN_FIFO_RX_BURST 32U
#define CAN_FIFO_ISR_BUDGET 8U

static can_fifo_frame_t can_fifo_tx_queue[CAN_FIFO_TX_LENGTH];
static uint8_t can_fifo_tx_head;
static uint8_t can_fifo_tx_tail;
static uint8_t can_fifo_tx_count;
static uint32_t can_fifo_tx_drop_count;

static can_fifo_rx_frame_t can_fifo_rx_queue[CAN_FIFO_RX_LENGTH];
static uint8_t can_fifo_rx_head;
static uint8_t can_fifo_rx_tail;
static uint8_t can_fifo_rx_count;
static uint32_t can_fifo_rx_total_count;
static uint32_t can_fifo_rx_drop_count;

void can_fifo_init(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    can_fifo_tx_head = 0U;
    can_fifo_tx_tail = 0U;
    can_fifo_tx_count = 0U;
    can_fifo_tx_drop_count = 0U;
    can_fifo_rx_head = 0U;
    can_fifo_rx_tail = 0U;
    can_fifo_rx_count = 0U;
    can_fifo_rx_total_count = 0U;
    can_fifo_rx_drop_count = 0U;
    __set_PRIMASK(mask);
}

HAL_StatusTypeDef can_fifo_tx_push(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    if (std_id > 0x7FFU || dlc > 8U || (dlc != 0U && data == NULL))
        return HAL_ERROR;

    can_fifo_frame_t frame = { .std_id = std_id, .dlc = dlc };
    if (dlc != 0U) memcpy(frame.data, data, dlc);

    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (can_fifo_tx_count >= CAN_FIFO_TX_LENGTH)
    {
        ++can_fifo_tx_drop_count;
        __set_PRIMASK(mask);
        return HAL_BUSY;
    }
    can_fifo_tx_queue[can_fifo_tx_tail] = frame;
    can_fifo_tx_tail = (uint8_t)((can_fifo_tx_tail + 1U) % CAN_FIFO_TX_LENGTH);
    ++can_fifo_tx_count;
    __set_PRIMASK(mask);
    return HAL_OK;
}

void can_fifo_tx_process(void)
{
    for (uint8_t sent = 0U; sent < CAN_FIFO_TX_BURST; ++sent)
    {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        if (can_fifo_tx_count == 0U || HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
        {
            __set_PRIMASK(mask);
            break;
        }

        can_fifo_frame_t *frame = &can_fifo_tx_queue[can_fifo_tx_head];
        CAN_TxHeaderTypeDef header = {0};
        uint32_t mailbox;
        header.StdId = frame->std_id;
        header.IDE = CAN_ID_STD;
        header.RTR = CAN_RTR_DATA;
        header.DLC = frame->dlc;
        header.TransmitGlobalTime = DISABLE;
        if (HAL_CAN_AddTxMessage(&hcan1, &header, frame->data, &mailbox) != HAL_OK)
        {
            __set_PRIMASK(mask);
            break;
        }

        can_fifo_tx_head = (uint8_t)((can_fifo_tx_head + 1U) % CAN_FIFO_TX_LENGTH);
        --can_fifo_tx_count;
        __set_PRIMASK(mask);
    }
}

uint8_t can_fifo_tx_get_pending_count(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint8_t count = can_fifo_tx_count;
    __set_PRIMASK(mask);
    return count;
}

uint32_t can_fifo_tx_get_drop_count(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint32_t count = can_fifo_tx_drop_count;
    __set_PRIMASK(mask);
    return count;
}

HAL_StatusTypeDef can_fifo_rx_push(const can_fifo_rx_frame_t *frame)
{
    if (frame == NULL || frame->frame.std_id > 0x7FFU || frame->frame.dlc > 8U)
        return HAL_ERROR;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (can_fifo_rx_count >= CAN_FIFO_RX_LENGTH)
    {
        ++can_fifo_rx_drop_count;
        __set_PRIMASK(mask);
        return HAL_BUSY;
    }
    can_fifo_rx_queue[can_fifo_rx_tail] = *frame;
    can_fifo_rx_tail = (uint8_t)((can_fifo_rx_tail + 1U) % CAN_FIFO_RX_LENGTH);
    ++can_fifo_rx_count;
    ++can_fifo_rx_total_count;
    __set_PRIMASK(mask);
    return HAL_OK;
}

uint8_t can_fifo_rx_pop(can_fifo_rx_frame_t *frame)
{
    if (frame == NULL) return 0U;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (can_fifo_rx_count == 0U)
    {
        __set_PRIMASK(mask);
        return 0U;
    }
    *frame = can_fifo_rx_queue[can_fifo_rx_head];
    can_fifo_rx_head = (uint8_t)((can_fifo_rx_head + 1U) % CAN_FIFO_RX_LENGTH);
    --can_fifo_rx_count;
    __set_PRIMASK(mask);
    return 1U;
}

void can_fifo_rx_process(void)
{
    can_fifo_rx_frame_t frame;
    for (uint8_t count = 0U; count < CAN_FIFO_RX_BURST; ++count)
    {
        if (!can_fifo_rx_pop(&frame)) break;
        can_fifo_rx_callback(&frame);
    }
}

__weak void can_fifo_rx_callback(const can_fifo_rx_frame_t *frame)
{
    (void)frame;
}

uint8_t can_fifo_rx_get_pending_count(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint8_t count = can_fifo_rx_count;
    __set_PRIMASK(mask);
    return count;
}

uint32_t can_fifo_rx_get_count(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint32_t count = can_fifo_rx_total_count;
    __set_PRIMASK(mask);
    return count;
}

uint32_t can_fifo_rx_get_drop_count(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint32_t count = can_fifo_rx_drop_count;
    __set_PRIMASK(mask);
    return count;
}

/**
  * @brief          将 CAN1 硬件 FIFO0 的标准数据帧复制到 RX 软件队列
  * @param[in]      hcan: 触发中断的 CAN 句柄
  * @note           限制每次回调最多读取八帧，不执行电机协议解析。
  * @retval         无
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1) return;
    for (uint8_t count = 0U; count < CAN_FIFO_ISR_BUDGET; ++count)
    {
        if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0U) break;
        CAN_RxHeaderTypeDef header;
        can_fifo_rx_frame_t frame = {0};
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, frame.frame.data) != HAL_OK) break;
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC > 8U) continue;
        frame.frame.std_id = (uint16_t)header.StdId;
        frame.frame.dlc = (uint8_t)header.DLC;
        frame.receive_tick = xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED ?
                             0U : xTaskGetTickCountFromISR();
        (void)can_fifo_rx_push(&frame);
    }
}
