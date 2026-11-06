#ifndef CAN_TASK_H
#define CAN_TASK_H

/**
  * @brief          按固定 1 ms 节拍处理 CAN1 收发队列，在任务中分发接收帧并提交发送帧
  * @param[in]      argument: 任务参数，未使用
  * @retval         无
  */
void can_task(void const *argument);

#endif
