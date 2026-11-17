#include "host_comm_task.h"
#include "bsp_host_uart.h"
#include "CAN_receive.h"
#include "INS_task.h"
#include "oid_motor.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

#define HOST_TX_QUEUE_DEPTH 8U
#define HOST_RX_QUEUE_DEPTH 8U
#define HOST_WHEEL_SPEED_PERIOD_MS 10U
#define HOST_IMU_RAW_PERIOD_MS 10U
#define HOST_WHEEL_TORQUE_PERIOD_MS 100U
#define HOST_LAUNCHER_SPEED_PERIOD_MS 20U
static QueueHandle_t tx_queue, rx_queue;
static host_parser_t parser;
static host_velocity_t latest_velocity;
static host_launcher_setpoint_t latest_launcher;
static host_feed_payload_t pending_feed;
static uint32_t pending_feed_ms, last_feed_ms;
static uint8_t feed_pending, feed_level, feed_armed;

/**
  * @brief          拨球命令断流时清零电平、重新等待低电平并丢弃待处理事件
  * @param[in]      now_ms: 当前单片机时间，单位 ms
  * @note           调用者须持有任务临界区；仅合法拨球开关帧刷新超时。
  * @retval         无
  */
static void reset_feed_on_timeout(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - last_feed_ms) >= HOST_LAUNCHER_FEED_TIMEOUT_MS) {
        feed_level = 0U;
        feed_armed = 0U;
        feed_pending = 0U;
    }
}

uint8_t host_comm_get_launcher_feed(host_feed_payload_t *feed)
{
    if (!feed) return 0;
    taskENTER_CRITICAL();
    uint32_t now_ms = HAL_GetTick();
    reset_feed_on_timeout(now_ms);
    uint8_t available = feed_pending &&
        (uint32_t)(now_ms - pending_feed_ms) < HOST_LAUNCHER_FEED_TIMEOUT_MS;
    if (available) *feed = pending_feed;
    feed_pending = 0U;
    taskEXIT_CRITICAL();
    return available;
}

uint8_t host_comm_get_launcher_setpoint(host_launcher_setpoint_t *setpoint)
{
    if (!setpoint) return 0;
    taskENTER_CRITICAL();
    *setpoint = latest_launcher;
    uint32_t now_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
    setpoint->valid = setpoint->received &&
                  (uint32_t)(now_ms - setpoint->last_update_ms) < HOST_LAUNCHER_TIMEOUT_MS;
    return setpoint->valid;
}

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

/**
  * @brief          按命令号解析底盘速度、发球目标和一次性拨球事件
  * @param[in]      packet: 通过帧校验的数据包
  * @param[in]      now_ms: 当前主控时间，单位 ms
  * @retval         无
  */
static void handle_received_packet(const host_packet_t *packet, uint32_t now_ms)
{
    switch (packet->command) {
    case HOST_CMD_CHASSIS_VELOCITY: {
        host_velocity_payload_t decoded;
        if (!host_decode_velocity(packet, &decoded)) return;
        taskENTER_CRITICAL();
        latest_velocity.target = decoded;
        latest_velocity.last_update_ms = now_ms;
        latest_velocity.sequence = packet->sequence;
        latest_velocity.received = 1U;
        latest_velocity.valid = 1U;
        ++latest_velocity.frame_count;
        taskEXIT_CRITICAL();
        break;
    }
    case HOST_CMD_LAUNCHER_SETPOINT: {
        host_launcher_payload_t decoded;
        if (!host_decode_launcher_setpoint(packet, &decoded)) return;
        taskENTER_CRITICAL();
        latest_launcher.target = decoded;
        latest_launcher.last_update_ms = now_ms;
        latest_launcher.sequence = packet->sequence;
        latest_launcher.received = 1U;
        latest_launcher.valid = 1U;
        ++latest_launcher.frame_count;
        taskEXIT_CRITICAL();
        break;
    }
    case HOST_CMD_LAUNCHER_FEED: {
        host_feed_payload_t decoded;
        if (!host_decode_launcher_feed(packet, &decoded)) return;
        taskENTER_CRITICAL();
        reset_feed_on_timeout(now_ms);
        last_feed_ms = now_ms;
        if (decoded.trigger == 0U) {
            feed_armed = 1U;
        } else if (feed_armed && feed_level == 0U && !feed_pending) {
            pending_feed = decoded;
            pending_feed_ms = now_ms;
            feed_pending = 1U;
        }
        feed_level = decoded.trigger;
        taskEXIT_CRITICAL();
        break;
    }
    default:
        break;
    }
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
    static uint32_t last_wheel_send_ms;
    static uint32_t last_imu_send_ms;
    static uint32_t last_torque_send_ms;
    static uint32_t last_launcher_send_ms;
    if ((uint32_t)(now_ms - last_wheel_send_ms) >= HOST_WHEEL_SPEED_PERIOD_MS) {
        // 普通达妙反馈已经是减速后输出轴 rad/s，无需再除减速比。
        uint8_t wheel_ready = 1U;
        float wheel_speed[4];
        for (uint8_t i = 0; i < 4; ++i) {
            const motor_measure_t *motor = get_chassis_motor_measure_point(i);
            wheel_speed[i] = motor->speed_rad_s;
            if (!motor->velocity_range_valid || !motor->feedback_received) wheel_ready = 0U;
        }
        if (wheel_ready && host_comm_send(HOST_CMD_WHEEL_SPEED, wheel_speed, sizeof(wheel_speed)))
            last_wheel_send_ms = now_ms;
    }

    if ((uint32_t)(now_ms - last_torque_send_ms) >= HOST_WHEEL_TORQUE_PERIOD_MS) {
        uint8_t torque_ready = 1U;
        float wheel_torque[CHASSIS_MOTOR_COUNT];
        for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; ++i) {
            const motor_measure_t *motor = get_chassis_motor_measure_point(i);
            wheel_torque[i] = motor->torque_nm;
            if (!motor->torque_range_valid || !motor->feedback_received) torque_ready = 0U;
        }
        _Static_assert(sizeof(wheel_torque) == HOST_WHEEL_TORQUE_PAYLOAD_SIZE,
                       "Wheel torque payload must be 16 bytes");
        if (torque_ready && host_comm_send(HOST_CMD_WHEEL_TORQUE, wheel_torque, sizeof(wheel_torque)))
            last_torque_send_ms = now_ms;
    }

    if ((uint32_t)(now_ms - last_launcher_send_ms) >= HOST_LAUNCHER_SPEED_PERIOD_MS) {
        oid_motor_measure_t motors[OID_MOTOR_COUNT];
        oid_motor_get_snapshot(motors);
        if (motors[0].speed_received && motors[1].speed_received) {
            float speeds[OID_MOTOR_COUNT] = {
                (float)motors[0].speed_erpm / OID_MOTOR_POLE_PAIRS,
                (float)motors[1].speed_erpm / OID_MOTOR_POLE_PAIRS,
            };
            _Static_assert(sizeof(speeds) == HOST_LAUNCHER_SPEED_PAYLOAD_SIZE,
                           "Launcher speed payload must be 8 bytes");
            if (host_comm_send(HOST_CMD_LAUNCHER_SPEED, speeds, sizeof(speeds)))
                last_launcher_send_ms = now_ms;
        }
    }

    if ((uint32_t)(now_ms - last_imu_send_ms) >= HOST_IMU_RAW_PERIOD_MS) {
        INS_raw_data_t raw;
        if (get_INS_raw_data_snapshot(&raw)) {
            float values[9];
            for (unsigned i = 0; i < 3; ++i) {
                values[i] = raw.gyro[i];
                values[3+i] = raw.accel[i];
                values[6+i] = raw.mag[i];
            }
            _Static_assert(sizeof(values) == HOST_IMU_RAW_PAYLOAD_SIZE, "IMU payload must be 36 bytes");
            // 0x0102：角速度 XYZ、加速度 XYZ、磁力计 XYZ，九个小端 float32。
            if (host_comm_send(HOST_CMD_IMU_RAW, values, sizeof(values)))
                last_imu_send_ms = now_ms;
        }
    }
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
