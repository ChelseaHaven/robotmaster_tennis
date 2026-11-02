#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H
#include "struct_typedef.h"
#include "remote_control.h"

#define GYRO_CALI_KNOB_CHANNEL 9U
#define GYRO_CALI_KNOB_MIN 192U
#define GYRO_CALI_KNOB_MAX 1792U
#define GYRO_CALI_KNOB_THRESHOLD ((GYRO_CALI_KNOB_MIN + GYRO_CALI_KNOB_MAX) / 2U)

/**
  * @brief          读取 Flash 中的陀螺仪标定参数，无有效记录时使用零补偿，不自动标定
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
  * @brief          检测 VRA/CH9 越过中值的上升沿并请求一次陀螺仪标定
  * @param[in]      remote: 遥控器一致快照，无效时清除触发准备状态
  * @note           仅任务调用。先见到不超过 992 的值，再超过 992 才触发。
  *                 忙碌时不排队，完成后需重新回到低侧再转高。
  * @retval         无
  */
void calibrate_knob_update(const RC_ctrl_t *remote);

/**
  * @brief          获取 IMU 恒温控制目标
  * @retval         目标温度，当前为 40 ℃
  */
int8_t get_control_temperature(void);
#endif
