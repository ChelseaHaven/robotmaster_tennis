#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "chassis_behaviour.h"

/* 底盘速度由 SWD 三档统一仲裁，任务外不直接覆盖目标速度。 */
void chassis_get_control(chassis_control_t *control);
void chassis_task(void const *argument);
#endif
