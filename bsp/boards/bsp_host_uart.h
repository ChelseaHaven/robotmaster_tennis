#ifndef BSP_HOST_UART_H
#define BSP_HOST_UART_H
#include <stdint.h>

/**
  * @brief          初始化上位机串口缓冲区并启动循环 DMA 接收
  * @retval         1：启动成功；0：启动失败
  */
uint8_t bsp_host_uart_init(void);

/**
  * @brief          在通信任务中恢复串口故障或清空溢出的接收数据
  * @retval         1：已发生故障或溢出，协议解析器应复位；0：无需处理
  */
uint8_t bsp_host_uart_service(void);

/**
  * @brief          从上位机串口接收环形缓冲区取出数据
  * @param[out]     data: 目标缓冲区
  * @param[in]      capacity: 目标缓冲区容量，单位字节
  * @note           由通信任务单独调用。
  * @retval         实际取出的字节数
  */
uint16_t bsp_host_uart_read(uint8_t *data, uint16_t capacity);

/**
  * @brief          查询上位机串口是否正在发送或处于故障状态
  * @retval         非 0：暂不可发送；0：空闲
  */
uint8_t bsp_host_uart_tx_busy(void);

/**
  * @brief          复制一帧到专用缓冲区并启动串口 DMA 发送
  * @param[in]      data: 待发送数据
  * @param[in]      length: 字节数，范围 1～137
  * @note           由通信任务单独调用。
  * @retval         1：DMA 启动成功；0：参数错误、忙碌或启动失败
  */
uint8_t bsp_host_uart_send(const uint8_t *data, uint16_t length);
#endif
