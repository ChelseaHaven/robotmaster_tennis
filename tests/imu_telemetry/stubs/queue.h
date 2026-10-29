#ifndef IMU_TEST_QUEUE_H
#define IMU_TEST_QUEUE_H
#include <stddef.h>
typedef void *QueueHandle_t;
QueueHandle_t xQueueCreate(unsigned depth, size_t size);
int xQueueSend(QueueHandle_t queue, const void *item, unsigned wait);
int xQueueReceive(QueueHandle_t queue, void *item, unsigned wait);
#endif
