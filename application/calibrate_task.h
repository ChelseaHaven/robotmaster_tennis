#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H
#include "struct_typedef.h"

void cali_param_init(void);
void calibrate_task(void const *argument);
void request_gyro_calibration(void);
int8_t get_control_temperature(void);
#endif
