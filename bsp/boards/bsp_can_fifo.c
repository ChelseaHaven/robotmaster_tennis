#include "bsp_can_fifo.h"
#include "can.h"
#include <string.h>

#define CAN_FIFO_TX_BURST 3U

static can_fifo_frame_t can_fifo_tx_queue[CAN_FIFO_TX_LENGTH];
static uint8_t can_fifo_tx_head;
static uint8_t can_fifo_tx_tail;
static uint8_t can_fifo_tx_count;
static uint32_t can_fifo_tx_drop_count;

void can_fifo_init(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    can_fifo_tx_head = 0U;
    can_fifo_tx_tail = 0U;
    can_fifo_tx_count = 0U;
    can_fifo_tx_drop_count = 0U;
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
