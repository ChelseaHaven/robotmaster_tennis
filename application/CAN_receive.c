#include "CAN_receive.h"
#include "detect_task.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
static motor_measure_t motor_chassis[4];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    if (hcan != &hcan1 || HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        return;
    if (header.IDE != CAN_ID_STD || header.DLC < 7 || header.StdId < 0x201 || header.StdId > 0x204)
        return;
    unsigned index = header.StdId - 0x201U;
    motor_measure_t *motor = &motor_chassis[index];
    motor->last_ecd = motor->ecd;
    motor->ecd = ((uint16_t)data[0] << 8) | data[1];
    motor->raw_speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    int32_t display_rpm = motor->raw_speed_rpm;
    if (index == 0U || index == 3U) display_rpm = -display_rpm;
    // 极端值 -32768 取反后按 int16_t 正上限饱和，避免显示仍为负。
    if (display_rpm > INT16_MAX) display_rpm = INT16_MAX;
    motor->speed_rpm = (int16_t)display_rpm;
    motor->given_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    motor->temperate = data[6];
    detect_hook(CHASSIS_MOTOR1_TOE + index);
}

void CAN_cmd_chassis(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    int16_t currents[4] = {m1, m2, m3, m4};
    uint8_t data[8];
    header.StdId = 0x200;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    for (unsigned i = 0; i < 4; ++i)
    {
        data[2*i] = (uint8_t)((uint16_t)currents[i] >> 8);
        data[2*i+1] = (uint8_t)currents[i];
    }
    HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
}

const motor_measure_t *get_chassis_motor_measure_point(uint8_t index)
{
    return index < 4 ? &motor_chassis[index] : 0;
}
