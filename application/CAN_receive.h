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

// 四个参数为直接发送给电调的电流指令。
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
const motor_measure_t *get_chassis_motor_measure_point(uint8_t index);
#endif
