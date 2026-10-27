#ifndef DETECT_TASK_H
#define DETECT_TASK_H
#include "struct_typedef.h"

enum errorList
{
    CHASSIS_MOTOR1_TOE = 0,
    CHASSIS_MOTOR2_TOE,
    CHASSIS_MOTOR3_TOE,
    CHASSIS_MOTOR4_TOE,
    BOARD_GYRO_TOE,
    BOARD_ACCEL_TOE,
    BOARD_MAG_TOE,
    REMOTE_TOE,
    ERROR_LIST_LENGHT,
};

typedef struct
{
    uint8_t enable;
    uint8_t error_exist;
    uint8_t is_lost;
    uint32_t new_time;
} error_t;

void detect_task(void const *argument);
void detect_hook(uint8_t device);
bool_t toe_is_error(uint8_t device);
const error_t *get_error_list_point(void);
#endif
