#ifndef BSP_SPI_H
#define BSP_SPI_H
#include "struct_typedef.h"

/**
  * @brief          配置 SPI1 收发 DMA 缓冲区和传输长度
  * @param[in]      tx_buf: 发送缓冲区地址
  * @param[in]      rx_buf: 接收缓冲区地址
  * @param[in]      num: 传输字节数
  * @retval         无
  */
extern void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);

/**
  * @brief          装载缓冲区并启动一次 SPI1 DMA 收发
  * @param[in]      tx_buf: 发送缓冲区地址
  * @param[in]      rx_buf: 接收缓冲区地址
  * @param[in]      ndtr: 传输字节数
  * @retval         无
  */
extern void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);

#endif
