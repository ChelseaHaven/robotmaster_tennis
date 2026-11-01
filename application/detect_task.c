#include <stdio.h>
#include "detect_task.h"
#include "INS_task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "main.h"

static error_t error_list[ERROR_LIST_LENGHT];
static const uint16_t offline_ms[ERROR_LIST_LENGHT] = {10, 10, 10, 10, 5, 5, 40, RC_FRAME_TIMEOUT_MS};

void detect_hook(uint8_t device)
{
    if (device < ERROR_LIST_LENGHT)
        error_list[device].new_time = xTaskGetTickCountFromISR();
}

bool_t toe_is_error(uint8_t device)
{
    if (device == REMOTE_TOE) return RC_data_is_error();
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
            if (i == REMOTE_TOE)
            {
                RC_ctrl_t remote;
                remote_control_service();
                get_remote_control_snapshot(&remote);
                error_list[i].is_lost = !remote.received ||
                    (uint32_t)(HAL_GetTick() - remote.last_update_ms) >= offline_ms[i];
                error_list[i].error_exist = !remote.valid;
                continue;
            }
            uint8_t lost = (now - error_list[i].new_time) > pdMS_TO_TICKS(offline_ms[i]);
            error_list[i].is_lost = lost;
            error_list[i].error_exist = lost;
        }
        osDelay(10);
    }
}
