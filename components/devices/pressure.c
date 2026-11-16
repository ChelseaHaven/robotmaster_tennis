#include "pressure.h"
#include "main.h"
#include "detect_task.h"
#include "bsp_pressure_uart.h"
#include "modbus_crc.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

#define PRESSURE_FRAME_LENGTH 7U
#define PRESSURE_REQUEST_PERIOD_MS 50U
#define PRESSURE_RESPONSE_TIMEOUT_MS 35U
static pressure_data_t pressure_data[PRESSURE_SENSOR_COUNT];
static const uint8_t pressure_addresses[PRESSURE_SENSOR_COUNT] = {
    PRESSURE_SENSOR_1_ADDR, PRESSURE_SENSOR_2_ADDR
};
static uint8_t rx_frame[PRESSURE_FRAME_LENGTH];
static uint8_t rx_count;
static uint8_t active_sensor;
static uint8_t request_active;
static uint8_t next_sensor;
static uint8_t waiting;
static uint8_t has_sent;
static uint32_t last_request_ms;
static uint32_t request_started_ms;

static uint8_t pressure_request(uint8_t sensor);
static uint8_t pressure_receive(void);

/**
  * @brief          拼接并校验当前从站的应答帧
  * @param[in]      byte: 接收字节
  * @retval         1：有效应答；0：未完成或校验失败
  */
static uint8_t pressure_rx_byte(uint8_t byte)
{
    if (!request_active) return 0;
    if (rx_count == 0 && byte != pressure_addresses[active_sensor]) return 0;
    if ((rx_count == 1 && byte != 0x03U) || (rx_count == 2 && byte != 0x02U)) {
        rx_count = 0;
        return 0;
    }
    rx_frame[rx_count++] = byte;
    if (rx_count < PRESSURE_FRAME_LENGTH) return 0;
    rx_count = 0;
    uint16_t crc = (uint16_t)(rx_frame[5] | ((uint16_t)rx_frame[6] << 8U));
    if (modbus_crc16(rx_frame, 5U) != crc) return 0;
    taskENTER_CRITICAL();
    pressure_data[active_sensor].value = (uint16_t)(((uint16_t)rx_frame[3] << 8U) | rx_frame[4]);
    taskEXIT_CRITICAL();
    detect_hook((uint8_t)(PRESSURE_SENSOR1_TOE + active_sensor));
    request_active = 0;
    return 1;
}

void pressure_get_snapshot(pressure_data_t data[PRESSURE_SENSOR_COUNT])
{
    if (data == NULL) return;
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++) data[i] = pressure_data[i];
    taskEXIT_CRITICAL();
}

void pressure_init(void)
{
    rx_count = active_sensor = request_active = 0;
    next_sensor = waiting = has_sent = 0;
    last_request_ms = request_started_ms = 0;
    for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++) pressure_data[i].value = 0;
    bsp_pressure_uart_init();
}

/** @brief 发送指定传感器请求。 @param[in] sensor 下标 @retval 1：成功；0：失败 */
static uint8_t pressure_request(uint8_t sensor)
{
    if (sensor >= PRESSURE_SENSOR_COUNT) return 0;
    uint8_t request[8] = {pressure_addresses[sensor], 0x03U, 0, 0, 0, 1, 0, 0};
    uint16_t crc = modbus_crc16(request, 6U);
    request[6] = (uint8_t)crc;
    request[7] = (uint8_t)(crc >> 8U);
    active_sensor = sensor;
    rx_count = 0;
    request_active = 1;
    if (!bsp_pressure_uart_send(request, sizeof(request))) {
        request_active = 0;
        return 0;
    }
    return 1;
}

/** @brief 解析缓存中的压力应答。 @retval 1：有效应答；0：未收到 */
static uint8_t pressure_receive(void)
{
    if (bsp_pressure_uart_service()) {
        rx_count = request_active = 0;
        return 0;
    }
    uint8_t byte;
    uint8_t received = 0;
    while (bsp_pressure_uart_read(&byte)) received |= pressure_rx_byte(byte);
    return received;
}

void pressure_update(void)
{
    if (pressure_receive()) waiting = 0;
    uint32_t now = HAL_GetTick();
    if (waiting && (uint32_t)(now - request_started_ms) >= PRESSURE_RESPONSE_TIMEOUT_MS)
        waiting = 0;
    if (!waiting && (!has_sent || (uint32_t)(now - last_request_ms) >= PRESSURE_REQUEST_PERIOD_MS)) {
        last_request_ms = now;
        has_sent = 1;
        waiting = pressure_request(next_sensor);
        request_started_ms = HAL_GetTick();
        next_sensor = (uint8_t)((next_sensor + 1U) % PRESSURE_SENSOR_COUNT);
    }
}
