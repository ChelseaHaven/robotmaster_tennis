#include <stdio.h>
#include "detect_task.h"
#include "INS_task.h"
#include "cmsis_os.h"

static error_t error_list[ERROR_LIST_LENGHT];
static const uint16_t offline_ms[ERROR_LIST_LENGHT] = {10, 10, 10, 10, 5, 5, 40};

void detect_hook(uint8_t device)
{
    if (device < ERROR_LIST_LENGHT)
        error_list[device].new_time = xTaskGetTickCountFromISR();
}

bool_t toe_is_error(uint8_t device)
{
    return device >= ERROR_LIST_LENGHT || error_list[device].error_exist;
}

const error_t *get_error_list_point(void)
{
    return error_list;
}

void detect_task(void const *argument)
{
    (void)argument;
    uint32_t now = xTaskGetTickCount();
    for (unsigned i = 0; i < ERROR_LIST_LENGHT; ++i)
    {
        error_list[i].enable = 1;
        error_list[i].error_exist = 1;
        error_list[i].is_lost = 1;
        error_list[i].new_time = now;
    }
#if !INS_USE_MAGNETOMETER
    error_list[BOARD_MAG_TOE].enable = 0;
    error_list[BOARD_MAG_TOE].error_exist = 0;
#endif
    for (;;)
    {
        now = xTaskGetTickCount();
        for (unsigned i = 0; i < ERROR_LIST_LENGHT; ++i)
        {
            if (!error_list[i].enable) continue;
            uint8_t lost = (now - error_list[i].new_time) > pdMS_TO_TICKS(offline_ms[i]);
            error_list[i].is_lost = lost;
            error_list[i].error_exist = lost;
        }
        osDelay(10);
    }
}
