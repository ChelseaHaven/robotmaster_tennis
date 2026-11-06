#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "struct_typedef.h"

#define CHASSIS_MOTOR_COUNT 4U
#define CHASSIS_MOTOR_TX_BASE_ID 0x201U
#define CHASSIS_MOTOR_RX_BASE_ID 0x11U

typedef struct
{
    uint16_t position_raw;     // 16 位位置编码
    uint16_t velocity_raw;     // 12 位速度编码
    uint16_t torque_raw;       // 12 位扭矩编码
    fp32 velocity_range;       // 从驱动器读取的 VMAX，rad/s
    fp32 raw_speed_rad_s;      // 官方公式解码的原始角速度，保留零点量化偏差，rad/s
    fp32 speed_rad_s;          // 打印及上发角速度，中间编码归零，按底层及轮序方向处理，rad/s
    uint8_t velocity_range_valid;
    uint8_t feedback_received;
    uint8_t state;             // 0 失能，1 使能，其他值为故障
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
} motor_measure_t;

/**
  * @brief          按四个驱动器的输出轴角速度发送速度模式指令
  * @param[in]      motor1: 1 号目标角速度，rad/s
  * @param[in]      motor2: 2 号目标角速度，rad/s
  * @param[in]      motor3: 3 号目标角速度，rad/s
  * @param[in]      motor4: 4 号目标角速度，rad/s
  * @param[in]      enable_mask: 位 0～3 分别控制四个驱动器是否允许使能
  * @note           所有帧通过 CAN FIFO；未读到 VMAX、无有效反馈或故障时保持失能。
  * @retval         无
  */
void CAN_cmd_chassis(fp32 motor1, fp32 motor2, fp32 motor3, fp32 motor4, uint8_t enable_mask);

/**
  * @brief          周期读取尚未获取的速度映射参数 VMAX
  * @note           在底盘任务中调用，每 20 ms 最多发送一个只读参数请求。
  * @retval         无
  */
void chassis_motor_service(void);

/**
  * @brief          获取指定底盘电机的反馈数据
  * @param[in]      index: 电机索引，0～3 对应 1～4 号
  * @retval         反馈结构体只读指针；索引越界返回 NULL
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t index);
#endif
