#ifndef HOST_COMMAND_H
#define HOST_COMMAND_H
#include "host_protocol.h"

#define HOST_CMD_CHASSIS_VELOCITY 0x0101U
#define HOST_VELOCITY_PAYLOAD_SIZE 12U
// 按传输方向区分：下行三轴目标速度 12 字节，上行四轮反馈速度 16 字节。
#define HOST_CMD_WHEEL_SPEED 0x0101U
#define HOST_CMD_IMU_RAW 0x0102U
#define HOST_IMU_RAW_PAYLOAD_SIZE 36U

/* 3566 下发车体系速度：vx 向前、vy 向左、wz 逆时针为正。
 * 单位依次为 m/s、m/s、rad/s。命令 0x0101 已由位姿改为速度。
 * 线上格式为三个小端 IEEE 754 float32，不直接映射结构体。 */
typedef struct {
    float vx;    // 单位 m/s
    float vy;    // 单位 m/s
    float wz;  // 单位 rad/s
} host_velocity_payload_t;

/* 仅解析命令 0x0101，检查长度及浮点数有效性，失败时不修改输出。 */
uint8_t host_decode_velocity(const host_packet_t *packet, host_velocity_payload_t *velocity);
#endif
