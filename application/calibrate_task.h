#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H
#include "struct_typedef.h"
#include "remote_control.h"

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
// CH7 两档：高值 1792 → 低值 192，下降沿请求一次标定。
#define GYRO_CALI_RC_CHANNEL 7U
#define GYRO_CALI_RC_HIGH_VALUE 1792U
#define GYRO_CALI_RC_LOW_VALUE 192U
#define GYRO_CALI_RC_TOLERANCE 120U

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
  * @brief          检测 CH7 高值到低值的下降沿并请求一次标定
  * @param[in]      remote: 遥控器快照；无效或忙碌时清除触发准备状态
  * @note           由任务周期调用；完成后需重新回到高值再拨低，才会再次触发。
  * @retval         无
  */
void calibrate_remote_control_update(const RC_ctrl_t *remote);

/**
  * @brief          获取 IMU 恒温控制目标
  * @retval         目标温度，当前为 40 ℃
  */
int8_t get_control_temperature(void);
#endif
