#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "struct_typedef.h"

typedef struct
{
    // 电调原始反馈，角度、转速和电流均保留电调方向。
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

// 四个参数为直接发送给电调的电流指令。
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
const motor_measure_t *get_chassis_motor_measure_point(uint8_t index);
#endif
