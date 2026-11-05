#ifndef BSP_CAN_FIFO_H
#define BSP_CAN_FIFO_H

#include "stm32f4xx_hal.h"

#define CAN_FIFO_TX_LENGTH 64U

typedef struct
{
    uint16_t std_id;
    uint8_t dlc;
    uint8_t data[8];
} can_fifo_frame_t;

/**
  * @brief          初始化 CAN1 发送队列和统计计数
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

#endif
