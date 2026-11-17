#include "bsp_usb_debug.h"
#include "main.h"

#define USB_DEBUG_RX_SIZE 512U
static uint8_t rx_buffer[USB_DEBUG_RX_SIZE];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_overflow, rx_reset;

void bsp_usb_debug_rx_reset(void)
{
    uint32_t state = __get_PRIMASK();
    __disable_irq();
    rx_head = rx_tail = 0U;
    rx_overflow = 0U;
    rx_reset = 1U;
    __set_PRIMASK(state);
}

void bsp_usb_debug_rx_push(const uint8_t *data, uint32_t length)
{
    if (!data || rx_overflow) return;
    for (uint32_t i = 0; i < length; ++i) {
        uint16_t next = (uint16_t)((rx_head + 1U) % USB_DEBUG_RX_SIZE);
        if (next == rx_tail) { rx_overflow = 1U; return; }
        rx_buffer[rx_head] = data[i];
        rx_head = next;
    }
}

int bsp_usb_debug_get_byte(uint8_t *byte)
{
    if (!byte) return 0;
    int result = 0;
    uint32_t state = __get_PRIMASK();
    __disable_irq();
    if (rx_overflow) {
        rx_tail = rx_head;
        rx_overflow = rx_reset = 0U;
        result = -1;
    } else if (rx_reset) {
        rx_reset = 0U;
        result = -2;
    } else if (rx_tail != rx_head) {
        *byte = rx_buffer[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % USB_DEBUG_RX_SIZE);
        result = 1;
    }
    __set_PRIMASK(state);
    return result;
}
