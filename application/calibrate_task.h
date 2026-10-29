#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H
#include "struct_typedef.h"
#include "remote_control.h"

void cali_param_init(void);
void calibrate_task(void const *argument);
// CH7 两档：高值 1792 → 低值 192，下降沿请求一次标定。
#define GYRO_CALI_RC_CHANNEL 7U
#define GYRO_CALI_RC_HIGH_VALUE 1792U
#define GYRO_CALI_RC_LOW_VALUE 192U
#define GYRO_CALI_RC_TOLERANCE 120U

// 忙碌或已有请求时忽略，不排队；由主函数或任务调用。
void request_gyro_calibration(void);
uint8_t gyro_calibration_is_busy(void);
// 由 detect_task 每 10 ms 提交遥控器快照，不在中断里执行标定。
void calibrate_remote_control_update(const RC_ctrl_t *remote);
int8_t get_control_temperature(void);
#endif
