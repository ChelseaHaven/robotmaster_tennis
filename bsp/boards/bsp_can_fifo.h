#ifndef BSP_CAN_FIFO_H
#define BSP_CAN_FIFO_H

#include "stm32f4xx_hal.h"

#define CAN_FIFO_TX_LENGTH 64U
#define CAN_FIFO_RX_LENGTH 128U

typedef struct
{
    uint16_t std_id;
    uint8_t dlc;
    uint8_t data[8];
} can_fifo_frame_t;

typedef struct
{
    can_fifo_frame_t frame;
    uint32_t receive_tick;
} can_fifo_rx_frame_t;

/**
  * @brief          初始化 CAN1 收发队列和统计计数
  * @note           在 CAN 启动和任务调度之前调用，运行中不可重复初始化。
  * @retval         无
  */
void can_fifo_init(void);

/**
  * @brief          将标准数据帧复制到 CAN1 发送队列
  * @param[in]      std_id: 标准帧标识符，范围 0～0x7FF
  * @param[in]      data: 数据缓冲区，长度为 0 时允许为 NULL
  * @param[in]      dlc: 数据长度，范围 0～8
  * @note           支持多个任务及中断调用；立即返回，不等待邮箱或队列空间。
  * @retval         HAL_OK 入队成功；HAL_BUSY 队列满；HAL_ERROR 参数错误
  */
HAL_StatusTypeDef can_fifo_tx_push(uint16_t std_id, const uint8_t *data, uint8_t dlc);

/**
  * @brief          将队首报文提交到 CAN1 空闲发送邮箱，每次最多提交三帧
  * @note           由 can_task 周期调用；邮箱忙或提交失败时保留队首，等待下次重试。
  * @retval         无
  */
void can_fifo_tx_process(void);

/**
  * @brief          获取等待提交到硬件邮箱的报文数量
  * @retval         软件发送队列中的帧数，不包含硬件邮箱中的帧
  */
uint8_t can_fifo_tx_get_pending_count(void);

/**
  * @brief          获取因队列满而拒绝入队的累计次数
  * @retval         自初始化以来的队列满计数
  */
uint32_t can_fifo_tx_get_drop_count(void);

/**
  * @brief          将接收帧及实际接收时刻复制到 RX 软件队列
  * @param[in]      frame: 标准数据帧和接收时刻，使用 FreeRTOS tick
  * @retval         HAL_OK 入队成功；HAL_BUSY 队列满；HAL_ERROR 参数错误
  */
HAL_StatusTypeDef can_fifo_rx_push(const can_fifo_rx_frame_t *frame);

/**
  * @brief          从 RX 软件队列按接收顺序取出一帧
  * @param[out]     frame: 接收帧及接收时刻
  * @retval         1 取到一帧；0 队列为空或指针无效
  */
uint8_t can_fifo_rx_pop(can_fifo_rx_frame_t *frame);

/**
  * @brief          每次最多取出 32 帧，调用应用回调进行解析
  * @note           仅由 can_task 调用，不在中断中执行协议解析。
  * @retval         无
  */
void can_fifo_rx_process(void);

/**
  * @brief          处理一帧已出队的标准 CAN 数据帧
  * @param[in]      frame: 接收帧和实际接收时刻，仅在回调期间有效
  * @note           默认弱实现为空，由 CAN_receive.c 实现达妙反馈解析。
  * @retval         无
  */
void can_fifo_rx_callback(const can_fifo_rx_frame_t *frame);

/**
  * @brief          获取等待应用处理的接收帧数量
  * @retval         RX 软件队列中的帧数
  */
uint8_t can_fifo_rx_get_pending_count(void);

/**
  * @brief          获取成功加入 RX 队列的累计帧数
  * @retval         自初始化以来的成功入队帧数
  */
uint32_t can_fifo_rx_get_count(void);

/**
  * @brief          获取因 RX 软件队列满而丢弃的累计帧数
  * @retval         自初始化以来的队列满计数，不包含硬件 FIFO 溢出
  */
uint32_t can_fifo_rx_get_drop_count(void);

#endif
