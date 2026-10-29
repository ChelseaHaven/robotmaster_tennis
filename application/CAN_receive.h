#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "struct_typedef.h"

typedef struct
{
    // 角度、电流和 raw_speed_rpm 保留电调原始方向。
    uint16_t ecd;
    int16_t raw_speed_rpm; // 原始转速，供现有轮速闭环使用
    int16_t speed_rpm;     // 显示和上发转速，1、4 号取反，前进时四轮同为正
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

/**
  * @brief          通过 CAN1 的 0x200 报文发送四路电调电流指令
  * @param[in]      motor1: 1 号电调电流指令
  * @param[in]      motor2: 2 号电调电流指令
  * @param[in]      motor3: 3 号电调电流指令
  * @param[in]      motor4: 4 号电调电流指令
  * @note           C620 的 ±16384 对应 ±20 A；本函数不再限幅。
  * @retval         无
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          获取指定底盘电机的反馈数据
  * @param[in]      index: 电机索引，0～3 对应 1～4 号
  * @retval         反馈结构体只读指针；索引越界返回 NULL
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t index);
#endif
