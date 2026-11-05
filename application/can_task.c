#include <stdio.h>
#include "can_task.h"
#include "bsp_can_fifo.h"
#include "cmsis_os.h"

#define CAN_TASK_PERIOD_MS 1U

void can_task(void const *argument)
{
    (void)argument;
    for (;;)
    {
        can_fifo_tx_process();
        osDelay(CAN_TASK_PERIOD_MS);
    }
}
