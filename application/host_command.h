#ifndef HOST_COMMAND_H
#define HOST_COMMAND_H
#include "host_protocol.h"

#define HOST_CMD_LIDAR_POSE 0x0101U
#define HOST_LIDAR_POSE_PAYLOAD_SIZE 12U
#define HOST_CMD_WHEEL_SPEED 0x0102U

/* 3566 下发的雷达位姿：odom 坐标系下 laser 的位置和朝向。
 * yaw 绕 +Z 逆时针为正，单位 rad，发送端范围为 -pi～pi。
 * 线上格式为三个小端 IEEE 754 float32，不直接映射结构体。 */
typedef struct {
    float x;    // 单位 m
    float y;    // 单位 m
    float yaw;  // 单位 rad
} host_lidar_pose_payload_t;

/* 仅解析命令 0x0101，检查长度及浮点数有效性，失败时不修改输出。 */
uint8_t host_decode_lidar_pose(const host_packet_t *packet, host_lidar_pose_payload_t *pose);
#endif
