#ifndef CHASSIS_TEST_MAIN_H
#define CHASSIS_TEST_MAIN_H
#include <stddef.h>
#include <stdint.h>
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
typedef struct { unsigned instance; } CAN_HandleTypeDef;
typedef struct { uint32_t StdId, IDE, DLC, RTR; } CAN_RxHeaderTypeDef;
typedef struct { uint32_t StdId, IDE, DLC, RTR; } CAN_TxHeaderTypeDef;
#define HAL_OK 0
#define CAN_RX_FIFO0 0
#define CAN_ID_STD 0
#define CAN_RTR_DATA 0
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, unsigned, CAN_RxHeaderTypeDef *, uint8_t *);
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *, CAN_TxHeaderTypeDef *, uint8_t *, uint32_t *);
#endif
