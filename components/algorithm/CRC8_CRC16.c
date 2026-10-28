#include "CRC8_CRC16.h"

/* 参考大疆 CRC8_CRC16 的算法参数，使用逐位计算以减少查表空间。 */
uint8_t get_CRC8_check_sum(const uint8_t *data, uint32_t length, uint8_t crc)
{
    if (!data) return crc;
    while (length--) {
        crc ^= *data++;
        for (unsigned i = 0; i < 8; ++i)
            crc = (crc >> 1) ^ ((crc & 1U) ? 0x8cU : 0U);
    }
    return crc;
}
uint16_t get_CRC16_check_sum(const uint8_t *data, uint32_t length, uint16_t crc)
{
    if (!data) return crc;
    while (length--) {
        crc ^= *data++;
        for (unsigned i = 0; i < 8; ++i)
            crc = (crc >> 1) ^ ((crc & 1U) ? 0x8408U : 0U);
    }
    return crc;
}
uint32_t verify_CRC8_check_sum(const uint8_t *data, uint32_t length)
{
    return data && length >= 2 && get_CRC8_check_sum(data, length-1, 0xff) == data[length-1];
}
uint32_t verify_CRC16_check_sum(const uint8_t *data, uint32_t length)
{
    if (!data || length < 3) return 0;
    uint16_t crc = get_CRC16_check_sum(data, length-2, 0xffff);
    return data[length-2] == (uint8_t)crc && data[length-1] == (uint8_t)(crc >> 8);
}
void append_CRC8_check_sum(uint8_t *data, uint32_t length)
{
    if (data && length >= 2) data[length-1] = get_CRC8_check_sum(data, length-1, 0xff);
}
void append_CRC16_check_sum(uint8_t *data, uint32_t length)
{
    if (!data || length < 3) return;
    uint16_t crc = get_CRC16_check_sum(data, length-2, 0xffff);
    data[length-2] = (uint8_t)crc;
    data[length-1] = (uint8_t)(crc >> 8);
}
