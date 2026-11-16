#include "modbus_crc.h"

uint16_t modbus_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    while (length--) {
        crc ^= *data++;
        for (uint8_t bit = 0; bit < 8U; bit++)
            crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) : (uint16_t)(crc >> 1U);
    }
    return crc;
}
