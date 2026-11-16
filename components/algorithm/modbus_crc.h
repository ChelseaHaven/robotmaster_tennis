#ifndef MODBUS_CRC_H
#define MODBUS_CRC_H
#include <stdint.h>
/**
  * @brief          计算 Modbus RTU CRC16，发送时低字节在前
  * @param[in]      data: 数据地址
  * @param[in]      length: 字节数
  * @retval         CRC16 校验值
  */
uint16_t modbus_crc16(const uint8_t *data, uint16_t length);
#endif
