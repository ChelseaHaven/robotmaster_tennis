#ifndef HOST_COMMAND_H
#define HOST_COMMAND_H
#include "host_protocol.h"

/* 下行：3588 → 单片机 */

// 底盘目标速度：vx、vy、wz，三个小端 float32，单位 m/s、m/s、rad/s。
#define HOST_CMD_CHASSIS_VELOCITY 0x0101U
#define HOST_VELOCITY_PAYLOAD_SIZE 12U

// 发球目标：电机 1、2 机械 RPM 和 pitch 指令角度，三个小端 float32。
#define HOST_CMD_LAUNCHER_SETPOINT 0x0102U
#define HOST_LAUNCHER_SETPOINT_PAYLOAD_SIZE 12U

// 拨球开关：uint8_t，仅允许 0/1，0→1 触发一次。
#define HOST_CMD_LAUNCHER_FEED 0x0103U
#define HOST_LAUNCHER_FEED_PAYLOAD_SIZE 1U

/* 上行：单片机 → 3588 */

// 四轮轮端角速度：右前、左前、左后、右后，四个小端 float32，单位 rad/s。
#define HOST_CMD_WHEEL_SPEED 0x0101U

// IMU：gyro XYZ、acc XYZ、mag XYZ，九个小端 float32。
#define HOST_CMD_IMU_RAW 0x0102U
#define HOST_IMU_RAW_PAYLOAD_SIZE 36U

// 四轮输出轴转矩：右前、左前、左后、右后，四个小端 float32，单位 N·m。
#define HOST_CMD_WHEEL_TORQUE 0x0103U
#define HOST_WHEEL_TORQUE_PAYLOAD_SIZE 16U

// 发球电机实际机械转速：电机 1、2，两个小端 float32，单位 RPM，保留符号。
#define HOST_CMD_LAUNCHER_SPEED 0x0104U
#define HOST_LAUNCHER_SPEED_PAYLOAD_SIZE 8U

typedef struct {
    float motor1_rpm;
    float motor2_rpm;
    float pitch_deg;
} host_launcher_payload_t;

/* 3566 下发车体系速度：vx 向前、vy 向左、wz 逆时针为正。
 * 单位依次为 m/s、m/s、rad/s。命令 0x0101 已由位姿改为速度。
 * 线上格式为三个小端 IEEE 754 float32，不直接映射结构体。 */
typedef struct {
    float vx;    // 单位 m/s
    float vy;    // 单位 m/s
    float wz;  // 单位 rad/s
} host_velocity_payload_t;

typedef struct {
    uint8_t trigger;
} host_feed_payload_t;

/**
  * @brief          解析下行车体系速度并检查命令号、长度和有限数值
  * @param[in]      packet: 已校验的协议包
  * @param[out]     velocity: 三轴目标速度，失败时保持原值
  * @retval         1：解析成功；0：参数或数据无效
  */
uint8_t host_decode_velocity(const host_packet_t *packet, host_velocity_payload_t *velocity);
/**
  * @brief          解析发球机构目标，检查命令号、长度、有限值及非负目标转速
  * @param[in]      packet: 已通过 CRC 校验的协议包
  * @param[out]     setpoint: 发球目标；解析失败时保持原值
  * @retval         1 解析成功；0 参数或载荷无效
  */
uint8_t host_decode_launcher_setpoint(const host_packet_t *packet, host_launcher_payload_t *setpoint);
/**
  * @brief          解析拨球开关值，仅接受单字节 0 或 1
  * @param[in]      packet: 已通过 CRC 校验的协议包
  * @param[out]     feed: 拨球命令，解析失败时保持原值
  * @retval         1 解析成功；0 参数、命令号或载荷长度错误
  */
uint8_t host_decode_launcher_feed(const host_packet_t *packet, host_feed_payload_t *feed);
#endif
