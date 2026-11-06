#include <stdio.h>
#include "can_task.h"
#include "bsp_can_fifo.h"
#include "FreeRTOS.h"
#include "task.h"

#define CAN_TASK_PERIOD_MS 1U

void can_task(void const *argument)
{
    (void)argument;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(CAN_TASK_PERIOD_MS);
    for (;;)
    {
        can_fifo_rx_process();
        can_fifo_tx_process();
        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}
