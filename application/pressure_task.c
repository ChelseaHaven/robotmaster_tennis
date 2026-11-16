#include <stddef.h>
#include "pressure_task.h"
#include "main.h"
#include "detect_task.h"
#include "bsp_pressure_uart.h"
#include "modbus_crc.h"
#include <stdio.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#define PRESSURE_BUS_REQUEST_PERIOD_MS  50U
#define PRESSURE_RESPONSE_TIMEOUT_MS    35U
#define PRESSURE_FRAME_LENGTH           7U
static pressure_data_t pressure_data[PRESSURE_SENSOR_COUNT] = {0};

static void Pressure_RxByte(uint8_t byte);

/* 传感器地址按数组顺序轮询。 */
static const uint8_t pressure_addresses[PRESSURE_SENSOR_COUNT] = {
    PRESSURE_SENSOR_1_ADDR, PRESSURE_SENSOR_2_ADDR
};

static volatile uint8_t pressure_rx_frame[PRESSURE_FRAME_LENGTH];
static volatile uint8_t pressure_rx_count;
static volatile uint8_t pressure_frame_ready;
static volatile uint8_t pressure_request_active;
static volatile uint8_t pressure_waiting;
static volatile uint8_t pressure_active_address;
static uint8_t pressure_next_sensor;
static uint8_t pressure_has_sent;
static uint32_t pressure_last_request_tick;
static uint32_t pressure_request_started_tick;

/**
 * @brief 构造读取一个压力传感器寄存器的 Modbus 请求帧。
 * @param address 压力传感器从站地址。
 * @param[out] request 用于接收 8 字节请求帧的缓冲区。
 */
static void Pressure_BuildReadRequest(uint8_t address, uint8_t request[8])
{
    uint16_t crc;

    request[0] = address;
    request[1] = 0x03U;
    request[2] = 0x00U;
    request[3] = 0x00U;
    request[4] = 0x00U;
    request[5] = 0x01U;
    crc = modbus_crc16(request, 6U);
    request[6] = (uint8_t)(crc & 0x00FFU);
    request[7] = (uint8_t)(crc >> 8U);
}

/**
 * @brief 将压力传感器从站地址映射到数据数组下标。
 * @param address 待查找的从站地址。
 * @return 找到时返回数组下标，否则返回 -1。
 */
static int Pressure_AddressIndex(uint8_t address)
{
    uint8_t sensor_index;

    for (sensor_index = 0U; sensor_index < PRESSURE_SENSOR_COUNT; sensor_index++) {
        if (address == pressure_addresses[sensor_index]) {
            return (int)sensor_index;
        }
    }
    return -1;
}

/**
 * @brief 校验已接收的压力传感器帧并更新对应数据缓存。
 * @param data 用于保存传感器读数和时效信息的数组。
 * @return 收到并处理有效帧时返回 1，否则返回 0。
 */
static uint8_t Pressure_ProcessReceivedFrame(pressure_data_t data[PRESSURE_SENSOR_COUNT])
{
    uint8_t frame[PRESSURE_FRAME_LENGTH];
    uint8_t i;
    uint8_t address;
    int sensor_index;
    uint16_t expected_crc;
    uint16_t received_crc;

    taskENTER_CRITICAL();
    if (pressure_frame_ready == 0U) {
        taskEXIT_CRITICAL();
        return 0U;
    }
    for (i = 0U; i < PRESSURE_FRAME_LENGTH; i++) {
        frame[i] = pressure_rx_frame[i];
    }
    pressure_frame_ready = 0U;
    pressure_rx_count = 0U;
    taskEXIT_CRITICAL();

    address = frame[0];
    sensor_index = Pressure_AddressIndex(address);
    if ((sensor_index < 0) || (address != pressure_active_address) || (frame[1] != 0x03U) || (frame[2] != 0x02U)) {
        return 0U;
    }

    expected_crc = modbus_crc16(frame, 5U);
    received_crc = (uint16_t)(frame[5] | ((uint16_t)frame[6] << 8U));
    if (expected_crc != received_crc) {
        return 0U;
    }

    taskENTER_CRITICAL();
    data[sensor_index].value = (uint16_t)(((uint16_t)frame[3] << 8U) | frame[4]);
    taskEXIT_CRITICAL();
    detect_hook((uint8_t)(PRESSURE_SENSOR1_TOE + sensor_index));
    pressure_waiting = 0U;
    pressure_request_active = 0U;
    return 1U;
}

/** @brief 处理完整应答和下一次轮询。 @param[in,out] data 读数数组 */
static void Pressure_Process(pressure_data_t data[PRESSURE_SENSOR_COUNT])
{
    uint32_t now;
    uint8_t request[8];
    if (data == NULL) {
        return;
    }

    (void)Pressure_ProcessReceivedFrame(data);
    now = HAL_GetTick();

    if (pressure_waiting != 0U && (uint32_t)(now - pressure_request_started_tick) >= PRESSURE_RESPONSE_TIMEOUT_MS) {
        pressure_waiting = 0U;
        pressure_request_active = 0U;
        pressure_rx_count = 0U;
        pressure_frame_ready = 0U;
    }

    if (pressure_waiting == 0U && (pressure_has_sent == 0U || (uint32_t)(now - pressure_last_request_tick) >= PRESSURE_BUS_REQUEST_PERIOD_MS)) {
        pressure_active_address = pressure_addresses[pressure_next_sensor];
        pressure_next_sensor = (uint8_t)((pressure_next_sensor + 1U) % PRESSURE_SENSOR_COUNT);

        pressure_rx_count = 0U;
        pressure_frame_ready = 0U;
        Pressure_BuildReadRequest(pressure_active_address, request);
        pressure_request_active = 1U;
        pressure_waiting = 1U;
        pressure_last_request_tick = now;
        pressure_has_sent = 1U;
        if (!bsp_pressure_uart_send(request, sizeof(request))) {
            pressure_waiting = 0U;
            pressure_request_active = 0U;
        }
        pressure_request_started_tick = HAL_GetTick();
    }

}

/** @brief 在任务内逐字节拼接当前从站应答。 @param[in] byte 接收字节 */
static void Pressure_RxByte(uint8_t byte)
{
    if ((pressure_request_active == 0U) || (pressure_frame_ready != 0U)) {
        return;
    }

    if (pressure_rx_count == 0U) {
        if (byte != pressure_active_address) {
            return;
        }
        pressure_rx_frame[0] = byte;
        pressure_rx_count = 1U;
        return;
    }

    if ((pressure_rx_count == 1U) && (byte != 0x03U)) {
        pressure_rx_count = 0U;
        return;
    }
    if ((pressure_rx_count == 2U) && (byte != 0x02U)) {
        pressure_rx_count = 0U;
        return;
    }

    if (pressure_rx_count < PRESSURE_FRAME_LENGTH) {
        pressure_rx_frame[pressure_rx_count++] = byte;
    }
    if (pressure_rx_count >= PRESSURE_FRAME_LENGTH) {
        pressure_rx_count = 0U;
        pressure_frame_ready = 1U;
    }
}

/** @brief 清空轮询状态及读数。 @param[out] data 读数数组 */
static void Pressure_Init(pressure_data_t data[PRESSURE_SENSOR_COUNT])
{
    uint8_t sensor_index;

    if (data == NULL) {
        return;
    }

    pressure_rx_count = 0U;
    pressure_frame_ready = 0U;
    pressure_request_active = 0U;
    pressure_waiting = 0U;
    pressure_active_address = 0U;
    for (sensor_index = 0U; sensor_index < PRESSURE_SENSOR_COUNT; sensor_index++) {
        data[sensor_index].value = 0U;
    }
    pressure_next_sensor = 0U;
    pressure_has_sent = 0U;
    pressure_last_request_tick = 0U;
    pressure_request_started_tick = 0U;
}

void pressure_get_snapshot(pressure_data_t data[PRESSURE_SENSOR_COUNT])
{
    if (data == NULL) return;
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < PRESSURE_SENSOR_COUNT; i++) data[i] = pressure_data[i];
    taskEXIT_CRITICAL();
}

void pressure_task(void const *argument)
{
    (void)argument;
    Pressure_Init(pressure_data);
    bsp_pressure_uart_init();
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        if (bsp_pressure_uart_service()) {
            pressure_waiting = pressure_request_active = 0U;
            pressure_frame_ready = pressure_rx_count = 0U;
        }
        uint8_t byte;
        while (bsp_pressure_uart_read(&byte)) Pressure_RxByte(byte);
        Pressure_Process(pressure_data);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
    }
}
