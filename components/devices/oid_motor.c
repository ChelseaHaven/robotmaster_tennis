#include "oid_motor.h"
#include "detect_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define OID_CMD_HEARTBEAT 0x00U
#define OID_CMD_CURRENT   0x01U
#define OID_CMD_SPEED     0x02U
#define OID_CMD_QUERY     0x0FU
#define OID_QUERY_FAULT   0x00U
#define OID_QUERY_SPEED   0x01U

static const uint16_t tx_id[OID_MOTOR_COUNT] = {OID_MOTOR1_TX_ID, OID_MOTOR2_TX_ID};
static const uint16_t rx_id[OID_MOTOR_COUNT] = {OID_MOTOR1_RX_ID, OID_MOTOR2_RX_ID};
static oid_motor_measure_t feedback[OID_MOTOR_COUNT];

/**
  * @brief          发送大端有符号电气转速，零目标按参考驱动发送零电流
  * @param[in]      index: 电机索引，0 或 1
  * @param[in]      speed: 电气转速，eRPM
  * @retval         CAN 发送队列入队结果
  */
static HAL_StatusTypeDef oid_send_speed(uint8_t index, int32_t speed)
{
    if (speed == 0) {
        const uint8_t stop[3] = {OID_CMD_CURRENT, 0U, 0U};
        return can_fifo_tx_push(tx_id[index], stop, sizeof(stop));
    }
    if (index == 1U) speed = -speed;
    uint32_t bits = (uint32_t)speed;
    uint8_t data[5] = {OID_CMD_SPEED, (uint8_t)(bits >> 24),
        (uint8_t)(bits >> 16), (uint8_t)(bits >> 8), (uint8_t)bits};
    return can_fifo_tx_push(tx_id[index], data, sizeof(data));
}

HAL_StatusTypeDef oid_motor_set_speed(int32_t motor1_erpm, int32_t motor2_erpm)
{
    int32_t values[OID_MOTOR_COUNT] = {motor1_erpm, motor2_erpm};
    HAL_StatusTypeDef result = HAL_OK;
    for (uint8_t i = 0; i < OID_MOTOR_COUNT; ++i) {
        if (values[i] > OID_MOTOR_MAX_ERPM) values[i] = OID_MOTOR_MAX_ERPM;
        if (values[i] < -OID_MOTOR_MAX_ERPM) values[i] = -OID_MOTOR_MAX_ERPM;
        HAL_StatusTypeDef status = oid_send_speed(i, values[i]);
        if (status != HAL_OK) result = status;
    }
    return result;
}

HAL_StatusTypeDef oid_motor_send_heartbeat(uint8_t index)
{
    if (index >= OID_MOTOR_COUNT) return HAL_ERROR;
    const uint8_t data = OID_CMD_HEARTBEAT;
    return can_fifo_tx_push(tx_id[index], &data, 1U);
}

HAL_StatusTypeDef oid_motor_query_fault(uint8_t index)
{
    if (index >= OID_MOTOR_COUNT) return HAL_ERROR;
    const uint8_t data[2] = {OID_CMD_QUERY, OID_QUERY_FAULT};
    return can_fifo_tx_push(tx_id[index], data, sizeof(data));
}

void oid_motor_get_snapshot(oid_motor_measure_t motors[OID_MOTOR_COUNT])
{
    if (motors == NULL) return;
    taskENTER_CRITICAL();
    memcpy(motors, feedback, sizeof(feedback));
    taskEXIT_CRITICAL();
}

void oid_motor_receive(const can_fifo_rx_frame_t *received)
{
    if (received == NULL) return;
    const can_fifo_frame_t *frame = &received->frame;
    if ((uint32_t)(xTaskGetTickCount() - received->receive_tick) >
        pdMS_TO_TICKS(OID_MOTOR_FEEDBACK_TIMEOUT_MS)) return;
    for (unsigned i = 0; i < OID_MOTOR_COUNT; ++i) {
        if (frame->std_id != rx_id[i] || frame->dlc < 2U ||
            frame->data[0] != OID_CMD_QUERY) continue;
        const uint8_t *data = frame->data;
        if (data[1] == OID_QUERY_SPEED && frame->dlc == 6U) {
            uint32_t bits = ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16) |
                            ((uint32_t)data[4] << 8) | data[5];
            int32_t speed;
            memcpy(&speed, &bits, sizeof(speed));
            if (i == 1U) {
                if (speed == INT32_MIN) return;
                speed = -speed;
            }
            taskENTER_CRITICAL();
            feedback[i].speed_erpm = speed;
            feedback[i].speed_received = 1U;
            taskEXIT_CRITICAL();
            detect_hook_at(LAUNCHER_MOTOR1_TOE + i, received->receive_tick);
        } else if (data[1] == OID_QUERY_FAULT && frame->dlc == 4U) {
            taskENTER_CRITICAL();
            feedback[i].fault = ((uint16_t)data[2] << 8) | data[3];
            feedback[i].fault_received = 1U;
            taskEXIT_CRITICAL();
        }
        return;
    }
}

