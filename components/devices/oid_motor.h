#ifndef OID_MOTOR_H
#define OID_MOTOR_H

#include "bsp_can_fifo.h"

#define OID_MOTOR_COUNT 2U
#define OID_MOTOR1_TX_ID 0x205U
#define OID_MOTOR2_TX_ID 0x206U
#define OID_MOTOR1_RX_ID 0x15U
#define OID_MOTOR2_RX_ID 0x16U
#define OID_MOTOR_MAX_ERPM 70000L
#define OID_MOTOR_FEEDBACK_TIMEOUT_MS 300U

typedef struct
{
    int32_t speed_erpm;
    uint16_t fault;
    uint8_t speed_received;
    uint8_t fault_received;
} oid_motor_measure_t;

/**
  * @brief          将两个发球电机的电气转速控制帧直接加入 CAN 发送队列
  * @param[in]      motor1_erpm: 电机 1 电气转速，机械 rpm = eRPM / 极对数
  * @param[in]      motor2_erpm: 电机 2 电气转速，正负表示驱动器转向
  * @note           限幅为正负 OID_MOTOR_MAX_ERPM；零目标、反馈未就绪或故障时发送零电流。
  * @note           不缓存目标、不负责周期重发，由调用任务决定发送时机。
  * @retval         HAL_OK 两帧均已入队；其他值表示存在入队失败，可能部分成功
  */
HAL_StatusTypeDef oid_motor_set_speed(int32_t motor1_erpm, int32_t motor2_erpm);

/**
  * @brief          将指定发球电机的心跳帧直接加入 CAN 发送队列
  * @param[in]      index: 电机索引，0 为电机 1，1 为电机 2
  * @retval         HAL_OK 入队成功；HAL_BUSY 队列满；HAL_ERROR 参数无效
  */
HAL_StatusTypeDef oid_motor_send_heartbeat(uint8_t index);

/**
  * @brief          将指定发球电机的故障查询帧直接加入 CAN 发送队列
  * @param[in]      index: 电机索引，0 为电机 1，1 为电机 2
  * @retval         HAL_OK 入队成功；HAL_BUSY 队列满；HAL_ERROR 参数无效
  */
HAL_StatusTypeDef oid_motor_query_fault(uint8_t index);

/**
  * @brief          在 CAN 接收任务中解析两个 OID 电调的转速上报及故障应答
  * @param[in]      received: 包含接收时刻的 CAN 数据帧
  * @retval         无
  */
void oid_motor_receive(const can_fifo_rx_frame_t *received);

/**
  * @brief          复制两个发球电机的反馈，数组顺序为电机 1、2
  * @param[out]     motors: 可容纳 OID_MOTOR_COUNT 个反馈结构体的数组
  * @retval         无
  */
void oid_motor_get_snapshot(oid_motor_measure_t motors[OID_MOTOR_COUNT]);

#endif
