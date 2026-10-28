#include "host_comm_task.h"
#include "bsp_host_uart.h"
#include "CAN_receive.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

#define HOST_TX_QUEUE_DEPTH 8U
#define HOST_RX_QUEUE_DEPTH 8U
#define HOST_WHEEL_SPEED_PERIOD_MS 10U
static QueueHandle_t tx_queue, rx_queue;
static host_parser_t parser;
static host_velocity_t latest_velocity;

uint8_t host_comm_get_velocity(host_velocity_t *velocity)
{
    if (!velocity) return 0;
    taskENTER_CRITICAL();
    *velocity = latest_velocity;
    uint32_t now_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
    velocity->valid = velocity->received &&
                  (uint32_t)(now_ms - velocity->last_update_ms) < HOST_VELOCITY_TIMEOUT_MS;
    return velocity->valid;
}

static void handle_received_packet(const host_packet_t *packet, uint32_t now_ms)
{
    host_velocity_payload_t decoded;
    if (!host_decode_velocity(packet, &decoded)) return;

    // 在原始接收队列入队前更新，业务未取队列时速度仍能持续刷新。
    taskENTER_CRITICAL();
    latest_velocity.vx = decoded.vx;
    latest_velocity.vy = decoded.vy;
    latest_velocity.wz = decoded.wz;
    latest_velocity.last_update_ms = now_ms;
    latest_velocity.sequence = packet->sequence;
    latest_velocity.received = 1;
    latest_velocity.valid = 1;
    ++latest_velocity.frame_count;
    taskEXIT_CRITICAL();
}

uint8_t host_comm_init(void)
{
    if (!tx_queue) tx_queue = xQueueCreate(HOST_TX_QUEUE_DEPTH, sizeof(host_packet_t));
    if (!rx_queue) rx_queue = xQueueCreate(HOST_RX_QUEUE_DEPTH, sizeof(host_packet_t));
    return tx_queue && rx_queue;
}
uint8_t host_comm_send(uint16_t command, const void *payload, uint16_t length)
{
    if (!tx_queue || length > HOST_MAX_PAYLOAD || (length && !payload)) return 0;
    host_packet_t packet = { .command = command, .length = length };
    if (length) {
        const uint8_t *bytes = payload;
        for (uint16_t i = 0; i < length; ++i) packet.payload[i] = bytes[i];
    }
    return xQueueSend(tx_queue, &packet, 0) == pdPASS;
}
uint8_t host_comm_receive(host_packet_t *packet)
{
    return rx_queue && packet && xQueueReceive(rx_queue, packet, 0) == pdPASS;
}
void host_comm_send_periodic(uint32_t now_ms)
{
    static uint32_t last_send_ms;
    if ((uint32_t)(now_ms - last_send_ms) < HOST_WHEEL_SPEED_PERIOD_MS)
        return;

    // 127 mm 轮径、19.2 减速比：电机转速换算为轮缘线速度，单位 m/s。
    const float rpm_to_mps = 3.14159265358979323846f * 0.127f / (60.0f * 19.2f);
    float wheel_speed[4];
    for (uint8_t i = 0; i < 4; ++i)
    {
        const motor_measure_t *motor = get_chassis_motor_measure_point(i);
        wheel_speed[i] = motor->speed_rpm * rpm_to_mps;
    }

    // 命令 0x0101：电机 1～4 的四个小端 float32，共 16 字节，保留方向。
    if (host_comm_send(HOST_CMD_WHEEL_SPEED, wheel_speed, sizeof(wheel_speed)))
        last_send_ms = now_ms;
}
void host_comm_task(void const *argument)
{
    (void)argument;
    if (!tx_queue || !rx_queue) { vTaskDelete(NULL); return; }
    bsp_host_uart_init();
    uint8_t sequence = 0, pending = 0;
    host_packet_t tx_packet, rx_packet;
    uint8_t bytes[64], frame[HOST_MAX_FRAME];
    for (;;) {
        if (bsp_host_uart_service()) parser.used = 0;
        uint16_t count;
        /* 每轮最多处理整个软件缓冲区，持续流量下也为其他任务留出时间。 */
        for (unsigned batch = 0; batch < 32; ++batch) {
            count = bsp_host_uart_read(bytes, sizeof(bytes));
            if (!count) break;
            for (uint16_t i = 0; i < count; ++i) {
                if (host_protocol_feed(&parser, bytes[i], HAL_GetTick(), &rx_packet)) {
                    handle_received_packet(&rx_packet, HAL_GetTick());
                    (void)xQueueSend(rx_queue, &rx_packet, 0);
                }
            }
        }
        host_comm_send_periodic(HAL_GetTick());
        if (!pending) pending = xQueueReceive(tx_queue, &tx_packet, 0) == pdPASS;
        if (pending && !bsp_host_uart_tx_busy()) {
            uint16_t length = host_protocol_pack(frame, tx_packet.command, sequence,
                                                tx_packet.payload, tx_packet.length);
            if (bsp_host_uart_send(frame, length)) { ++sequence; pending = 0; }
        }
        osDelay(2);
    }
}
