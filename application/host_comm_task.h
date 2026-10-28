#ifndef HOST_COMM_TASK_H
#define HOST_COMM_TASK_H
#include "host_protocol.h"
#include "host_command.h"

#define HOST_POSE_TIMEOUT_MS 500U

/* 最新雷达位姿快照；超时只清除 valid，保留最后一次坐标。 */
typedef struct {
    float x;
    float y;
    float yaw;
    uint32_t last_update_ms;  // 单片机收到并解析成功的时间
    uint32_t frame_count;
    uint8_t sequence;
    uint8_t received;
    uint8_t valid;
} host_pose_t;

/* 仅任务调用：复制最新位姿，收到且未超过 500 ms 返回 1，否则返回 0。 */
uint8_t host_comm_get_pose(host_pose_t *pose);

/* 由 main 在调度器启动前创建队列；返回 0 表示资源不足。 */
uint8_t host_comm_init(void);
void host_comm_task(void const *argument);
/* 仅任务调用：复制数据入队，不等待；成功返回 1，未初始化、队列满或参数错误返回 0。 */
uint8_t host_comm_send(uint16_t command, const void *payload, uint16_t length);
/* 仅任务调用：非阻塞取出一帧，成功返回 1。队列满时丢弃新收到的帧。 */
uint8_t host_comm_receive(host_packet_t *packet);
/* 在通信任务中每轮调用，留作周期发送入口；不要阻塞。 */
void host_comm_send_periodic(uint32_t now_ms);
#endif
