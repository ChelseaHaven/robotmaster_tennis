#ifndef TEST_TASK_H
#define TEST_TASK_H
#include "struct_typedef.h"


typedef enum {
    TEST_DEBUG_PITCH,
    TEST_DEBUG_SEPARATOR,
    TEST_DEBUG_SPEED
} test_debug_command_type_t;

typedef struct {
    test_debug_command_type_t type;
    float angle_deg;
    float motor1_rpm;
    float motor2_rpm;
} test_debug_command_t;

/**
  * @brief          解析 USB 调试串口的一行命令，不执行舵机或电机控制
  * @param[out]     command: 命令类型及参数；失败时保持原值
  * @note           仅 test_task 调用，以 CR、LF 或 CRLF 结束一行。
  * @retval         1 取到合法新命令；0 无完整命令或命令无效
  */
uint8_t test_get_debug_command(test_debug_command_t *command);

#endif
