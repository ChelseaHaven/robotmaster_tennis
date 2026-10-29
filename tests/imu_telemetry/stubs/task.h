#ifndef IMU_TEST_TASK_H
#define IMU_TEST_TASK_H
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
void vTaskDelete(void *task);
#endif
