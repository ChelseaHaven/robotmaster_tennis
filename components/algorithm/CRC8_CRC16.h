#ifndef CRC8_CRC16_H
#define CRC8_CRC16_H
#include <stdint.h>

/**
  * @brief          按反射多项式计算 CRC8 校验值
  * @param[in]      data: 待校验数据
  * @param[in]      length: 参与计算的字节数
  * @param[in]      crc: 初始校验值
  * @retval         计算得到的 CRC8
  */
uint8_t get_CRC8_check_sum(const uint8_t *data, uint32_t length, uint8_t crc);

/**
  * @brief          按反射多项式计算 CRC16 校验值
  * @param[in]      data: 待校验数据
  * @param[in]      length: 参与计算的字节数
  * @param[in]      crc: 初始校验值
  * @retval         计算得到的 CRC16
  */
uint16_t get_CRC16_check_sum(const uint8_t *data, uint32_t length, uint16_t crc);

/**
  * @brief          检查数据末尾的 CRC8 是否正确
  * @param[in]      data: 包含校验字段的完整数据
  * @param[in]      length: 总字节数，包含末尾校验字段
  * @retval         1：校验通过；0：参数无效或校验失败
  */
uint32_t verify_CRC8_check_sum(const uint8_t *data, uint32_t length);

/**
  * @brief          检查数据末尾的 CRC16 是否正确
  * @param[in]      data: 包含校验字段的完整数据
  * @param[in]      length: 总字节数，包含末尾校验字段
  * @retval         1：校验通过；0：参数无效或校验失败
  */
uint32_t verify_CRC16_check_sum(const uint8_t *data, uint32_t length);

/**
  * @brief          计算并写入数据末尾的 CRC8 校验字段
  * @param[in,out]  data: 预留末尾校验字段的数据缓冲区
  * @param[in]      length: 总字节数，包含末尾校验字段
  * @retval         无
  */
void append_CRC8_check_sum(uint8_t *data, uint32_t length);

/**
  * @brief          计算并写入数据末尾的 CRC16 校验字段
  * @param[in,out]  data: 预留末尾校验字段的数据缓冲区
  * @param[in]      length: 总字节数，包含末尾校验字段
  * @retval         无
  */
void append_CRC16_check_sum(uint8_t *data, uint32_t length);
#endif
