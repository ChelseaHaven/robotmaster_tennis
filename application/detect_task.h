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

/**
  * @brief          周期检查设备超时并维护遥控器接收状态
  * @param[in]      argument: 任务参数，未使用
  * @retval         无
  */
void detect_task(void const *argument);

/**
  * @brief          记录指定设备最近一次收到数据的时刻
  * @param[in]      device: 设备索引，小于 ERROR_LIST_LENGHT
  * @retval         无
  */
void detect_hook(uint8_t device);

/**
  * @brief          查询指定设备是否存在检测错误
  * @param[in]      device: 设备索引
  * @retval         设备错误标志；索引越界时返回 1
  */
bool_t toe_is_error(uint8_t device);

/**
  * @brief          获取设备在线检测结果表
  * @retval         错误表只读指针，长度为 ERROR_LIST_LENGHT
  */
const error_t *get_error_list_point(void);
#endif
