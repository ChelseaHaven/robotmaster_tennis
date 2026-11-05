#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "chassis_behaviour.h"

/**
  * @brief          复制底盘当前控制状态的一致快照
  * @param[out]     control: 模式、目标速度及等待状态；允许为 NULL
  * @retval         无
  */
void chassis_get_control(chassis_control_t *control);

/**
  * @brief          周期执行模式仲裁、麦轮逆解和驱动器速度指令发送
  * @param[in]      argument: 任务参数，未使用
  * @note           轮速反馈异常或急停时给零速并失能驱动器；速度闭环由驱动器执行。
  * @retval         无
  */
void chassis_task(void const *argument);
#endif
