#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H
#include "struct_typedef.h"

/**
  * @brief          读取 Flash 中的陀螺仪标定参数，无有效记录时请求首次标定
  * @retval         无
  */
void cali_param_init(void);

/**
  * @brief          执行陀螺仪静态零偏标定并将结果保存到 Flash
  * @param[in]      argument: 任务参数，未使用
  * @note           标定期间保持车体静止，姿态任务继续运行。
  * @retval         无
  */
void calibrate_task(void const *argument);
/**
  * @brief          请求一次陀螺仪标定，已有请求或正在标定时忽略
  * @note           由主函数或任务调用；标定由 calibrate_task 异步执行。
  * @retval         无
  */
void request_gyro_calibration(void);

/**
  * @brief          查询陀螺仪标定是否已请求或正在执行
  * @retval         1：忙碌；0：空闲
  */
uint8_t gyro_calibration_is_busy(void);

/**
  * @brief          获取 IMU 恒温控制目标
  * @retval         目标温度，当前为 40 ℃
  */
int8_t get_control_temperature(void);
#endif
