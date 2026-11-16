#ifndef PRESSURE_TASK_H
#define PRESSURE_TASK_H
#include <stdint.h>
#define PRESSURE_SENSOR_COUNT 2U
#define PRESSURE_SENSOR_1_ADDR 0x01U
#define PRESSURE_SENSOR_2_ADDR 0x02U

typedef struct {
    uint16_t value;  /* 原始寄存器值，未转换为物理单位。 */
} pressure_data_t;

/**
  * @brief          读取两个压力传感器的一致快照
  * @param[out]     data: 长度为 PRESSURE_SENSOR_COUNT 的数组
  * @retval         无；在线状态通过 toe_is_error(PRESSURE_SENSOR1_TOE/2_TOE) 判断
  */
void pressure_get_snapshot(pressure_data_t data[PRESSURE_SENSOR_COUNT]);
/**
  * @brief          初始化串口并以固定 5 ms 节拍处理双传感器轮询
  * @param[in]      argument: 任务参数，未使用
  * @retval         无
  */
void pressure_task(void const *argument);
#endif
