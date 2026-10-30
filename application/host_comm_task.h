#ifndef HOST_COMM_TASK_H
#define HOST_COMM_TASK_H
#include "host_protocol.h"
#include "host_command.h"

#define HOST_VELOCITY_TIMEOUT_MS 500U

/* 最新车体系速度快照；超时只清除 valid，保留最后一次速度。 */
typedef struct {
    float vx;
    float vy;
    float wz;
    uint32_t last_update_ms;  // 单片机收到并解析成功的时间
    uint32_t frame_count;
    uint8_t sequence;
    uint8_t received;
    uint8_t valid;
} host_velocity_t;

/**
  * @brief          在任务中复制上位机最新速度及有效状态
  * @param[out]     velocity: 车体系速度快照；超时仍保留上次速度值
  * @retval         1：已接收且未超时；0：空指针、未接收或超时
  */
uint8_t host_comm_get_velocity(host_velocity_t *velocity);

/**
  * @brief          在调度器启动前创建上位机通信收发队列
  * @retval         1：队列可用；0：队列创建失败
  */
uint8_t host_comm_init(void);

/**
  * @brief          运行上位机串口收包、协议解析、周期上发和 DMA 发送调度
  * @param[in]      argument: 任务参数，未使用
  * @retval         无
  */
void host_comm_task(void const *argument);

/**
  * @brief          复制待发数据到发送队列，由通信任务统一发送
  * @param[in]      command: 命令号
  * @param[in]      payload: 数据载荷，长度为 0 时允许为 NULL
  * @param[in]      length: 载荷字节数，不超过 HOST_MAX_PAYLOAD
  * @note           仅任务调用，非阻塞，允许多个任务调用。
  * @retval         1：成功入队；0：参数错误、队列未创建或队列已满
  */
uint8_t host_comm_send(uint16_t command, const void *payload, uint16_t length);

/**
  * @brief          在任务中非阻塞取出一帧已校验的接收数据
  * @param[out]     packet: 完整接收包
  * @note           接收队列满时丢弃新帧；速度缓存仍会独立更新。
  * @retval         1：取到一帧；0：队列为空、未创建或参数无效
  */
uint8_t host_comm_receive(host_packet_t *packet);

/**
  * @brief          按各自周期将四轮轮端角速度（rad/s）和 IMU 原始数据加入发送队列
  * @param[in]      now_ms: 当前主控时间，单位 ms
  * @note           由通信任务调用，当前两类数据均按 10 ms 周期上发。
  * @retval         无
  */
void host_comm_send_periodic(uint32_t now_ms);
#endif
