/* 大疆工程应用层：解析 HT-10A 的 25 字节 SBUS 数据，提供通道快照与在线状态。 */
#include "remote_control.h"
#include "bsp_rc.h"
#include "detect_task.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>

static RC_ctrl_t rc_ctrl;
static uint8_t frame_buffer[RC_FRAME_LENGTH];
static uint8_t frame_used;

static void receive_error(uint32_t reason)
{
    (void)reason;
    rc_ctrl.valid = 0;
    ++rc_ctrl.receive_errors;
    frame_used = 0;
}

static void parse_byte(uint8_t byte)
{
    if (frame_used == 0U && byte != 0x0FU) return;
    frame_buffer[frame_used++] = byte;
    if (frame_used < RC_FRAME_LENGTH) return;

    if (frame_buffer[24] == 0U && (frame_buffer[23] & 0xF0U) == 0U)
    {
        for (unsigned channel = 0; channel < RC_CHANNEL_COUNT; ++channel)
        {
            unsigned bit = channel * 11U;
            unsigned offset = 1U + bit / 8U;
            uint32_t packed = frame_buffer[offset] | ((uint32_t)frame_buffer[offset + 1U] << 8);
            if (offset + 2U <= 22U) packed |= (uint32_t)frame_buffer[offset + 2U] << 16;
            rc_ctrl.ch[channel] = (uint16_t)((packed >> (bit % 8U)) & 0x07FFU);
        }
        rc_ctrl.flags = frame_buffer[23];
        rc_ctrl.digital[0] = (rc_ctrl.flags & RC_FLAG_DIGITAL_CH17) != 0U;
        rc_ctrl.digital[1] = (rc_ctrl.flags & RC_FLAG_DIGITAL_CH18) != 0U;
        rc_ctrl.last_update_ms = HAL_GetTick();
        ++rc_ctrl.frame_count;
        rc_ctrl.received = 1;
        rc_ctrl.valid = (rc_ctrl.flags & (RC_FLAG_FRAME_LOST | RC_FLAG_FAILSAFE)) == 0U;
        /* 接收可能早于调度器启动；此时不调用 FreeRTOS 的中断接口。 */
        if (rc_ctrl.valid && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
            detect_hook(REMOTE_TOE);
        frame_used = 0;
        return;
    }

    ++rc_ctrl.parse_errors;
    unsigned next = 1U;
    while (next < RC_FRAME_LENGTH && frame_buffer[next] != 0x0FU) ++next;
    frame_used = (uint8_t)(RC_FRAME_LENGTH - next);
    memmove(frame_buffer, frame_buffer + next, frame_used);
}

static void receive_bytes(volatile const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; ++i) parse_byte(data[i]);
}

void remote_control_init(void)
{
    uint32_t saved_mask = __get_PRIMASK();
    __disable_irq();
    memset(&rc_ctrl, 0, sizeof(rc_ctrl));
    frame_used = 0;
    RC_Init(receive_bytes, receive_error);
    __set_PRIMASK(saved_mask);
}

void get_remote_control_snapshot(RC_ctrl_t *snapshot)
{
    if (snapshot == NULL) return;
    uint32_t saved_mask = __get_PRIMASK();
    __disable_irq();
    *snapshot = rc_ctrl;
    uint32_t now = HAL_GetTick();
    __set_PRIMASK(saved_mask);
    snapshot->valid = snapshot->valid && snapshot->received &&
                      (uint32_t)(now - snapshot->last_update_ms) < RC_FRAME_TIMEOUT_MS;
}

uint8_t RC_data_is_error(void)
{
    RC_ctrl_t snapshot;
    get_remote_control_snapshot(&snapshot);
    return !snapshot.valid;
}

void remote_control_service(void)
{
    if (RC_restart_is_needed())
    {
        uint32_t saved_mask = __get_PRIMASK();
        __disable_irq();
        rc_ctrl.valid = 0;
        frame_used = 0;
        RC_restart();
        __set_PRIMASK(saved_mask);
    }
}
