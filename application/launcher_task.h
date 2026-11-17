#ifndef LAUNCHER_TASK_H
#define LAUNCHER_TASK_H

#define LAUNCHER_PERIOD_MS              10U
#define LAUNCHER_MOTOR_CONTROL_PERIOD_MS    20U
#define LAUNCHER_MOTOR_HEARTBEAT_PERIOD_MS 100U
#define LAUNCHER_MOTOR_FAULT_QUERY_PERIOD_MS 200U
#define LAUNCHER_MOTOR1_TARGET_ERPM 0
#define LAUNCHER_MOTOR2_TARGET_ERPM 0
#define LAUNCHER_BALL_SEPARATOR_ANGLE_DEG     135.0f
#define LAUNCHER_PITCH_ANGLE_DEG   135.0f

/**
  * @brief          发球任务，周期处理压力、舵机和 OID 发球电机通信
  * @param[in]      argument: 任务参数，未使用
  * @retval         无
  */
void launcher_task(void const *argument);
#endif
