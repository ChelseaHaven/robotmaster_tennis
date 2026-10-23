#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"

// 速度单位为 m/s、m/s、rad/s；未设置时默认为零。
void chassis_set_velocity(fp32 vx, fp32 vy, fp32 wz);
void chassis_task(void const *argument);
#endif
