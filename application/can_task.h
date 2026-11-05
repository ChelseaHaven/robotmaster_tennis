#ifndef CAN_TASK_H
#define CAN_TASK_H

/**
  * @brief          每 1 ms 处理 CAN1 发送队列，将报文提交到硬件邮箱
  * @param[in]      argument: 任务参数，未使用
  * @retval         无
  */
void can_task(void const *argument);

#endif
