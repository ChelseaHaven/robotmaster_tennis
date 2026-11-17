#ifndef LAUNCHER_TASK_H
#define LAUNCHER_TASK_H

#define LAUNCHER_PERIOD_MS              10U
#define LAUNCHER_MOTOR_CONTROL_PERIOD_MS    20U
#define LAUNCHER_MOTOR_HEARTBEAT_PERIOD_MS 100U
#define LAUNCHER_MOTOR_FAULT_QUERY_PERIOD_MS 200U
// PWM 初始化输出的指令角，上电后缓动到 LAUNCHER_BALL_START_SLOT。
#define LAUNCHER_BALL_SEPARATOR_ANGLE_DEG     135.0f
#define LAUNCHER_PITCH_ANGLE_DEG   135.0f

/* 四个分球工位的舵机指令角，后续按实物分别标定，不使用参考工程脉宽。 */
#define LAUNCHER_BALL_SLOT_COUNT          4U
#define LAUNCHER_BALL_SLOT0_ANGLE_DEG      0.0f
#define LAUNCHER_BALL_SLOT1_ANGLE_DEG     90.0f
#define LAUNCHER_BALL_SLOT2_ANGLE_DEG    180.0f
#define LAUNCHER_BALL_SLOT3_ANGLE_DEG    270.0f
#define LAUNCHER_BALL_START_SLOT          0U
/* 沿用参考工程：地址 0x02 在左，0x01 在右；当前压力数组顺序为 0x01、0x02。 */
#define LAUNCHER_BALL_LEFT_SENSOR         1U
#define LAUNCHER_BALL_RIGHT_SENSOR        0U
#define LAUNCHER_BALL_LEFT_ON            30U
#define LAUNCHER_BALL_LEFT_OFF           20U
#define LAUNCHER_BALL_RIGHT_ON           30U
#define LAUNCHER_BALL_RIGHT_OFF          20U
#define LAUNCHER_BALL_STABLE_SAMPLES      2U
#define LAUNCHER_BALL_HOME_MOVE_MS     1200U
#define LAUNCHER_BALL_MOVE_MS           500U
#define LAUNCHER_BALL_SETTLE_MS        1000U
#define LAUNCHER_BALL_SEARCH_RETRY_MS   500U
#define LAUNCHER_BALL_PREPARE_TIMEOUT_MS 8000U
#define LAUNCHER_BALL_SEARCH_TIMEOUT_MS 8000U
#define LAUNCHER_BALL_AUTO_PREPARE        1U

/**
  * @brief          发球任务，周期处理压力判球、四工位分球、俯仰和 OID 发球电机通信
  * @param[in]      argument: 任务参数，未使用
  * @retval         无
  */
void launcher_task(void const *argument);
#endif
