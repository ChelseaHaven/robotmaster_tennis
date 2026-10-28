#ifndef CRC8_CRC16_H
#define CRC8_CRC16_H
#include <stdint.h>
/* 与大疆例程相同：CRC8 初值 FF，CRC16 初值 FFFF，低位优先，无最终异或。 */
uint8_t get_CRC8_check_sum(const uint8_t *data, uint32_t length, uint8_t crc);
uint16_t get_CRC16_check_sum(const uint8_t *data, uint32_t length, uint16_t crc);
uint32_t verify_CRC8_check_sum(const uint8_t *data, uint32_t length);
uint32_t verify_CRC16_check_sum(const uint8_t *data, uint32_t length);
void append_CRC8_check_sum(uint8_t *data, uint32_t length);
void append_CRC16_check_sum(uint8_t *data, uint32_t length);
#endif
