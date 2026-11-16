#ifndef LAUNCHER_TASK_H
#define LAUNCHER_TASK_H

#define LAUNCHER_PERIOD_MS              5U
#define LAUNCHER_BALL_SEPARATOR_ANGLE_DEG     135.0f
#define LAUNCHER_PITCH_ANGLE_DEG   135.0f

/**
  * @brief          发球任务，周期处理两路压力传感器并设置分球器及发球俯仰舵机目标角度
  * @param[in]      argument: 任务参数，未使用
  * @retval         无
  */
void launcher_task(void const *argument);
#endif
