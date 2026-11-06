#include "CAN_receive.h"
#include "detect_task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_can_fifo.h"
#include <math.h>
#include <string.h>

#define MOTOR_VMAX_REGISTER 0x16U
#define MOTOR_PARAMETER_PERIOD_MS 20U
#define MOTOR_SWITCH_RETRY_MS 100U

static motor_measure_t motor_chassis[CHASSIS_MOTOR_COUNT];

/**
  * @brief          从小端字节序读取 IEEE 754 单精度浮点数
  * @param[in]      data: 四字节数据
  * @retval         解码后的浮点数
  */
static fp32 read_float_le(const uint8_t *data)
{
    uint32_t bits = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                    ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    fp32 value;
    _Static_assert(sizeof(value) == sizeof(bits), "float must be 32 bits");
    memcpy(&value, &bits, sizeof(value));
    return value;
}

/**
  * @brief          将输出轴角速度编码为小端浮点速度模式帧
  * @param[in]      index: 电机索引，0～3
  * @param[in]      speed: 输出轴角速度，rad/s
  * @retval         FIFO 入队结果
  */
static HAL_StatusTypeDef send_speed(uint8_t index, fp32 speed)
{
    uint32_t bits;
    uint8_t data[4];
    speed = -speed;
    memcpy(&bits, &speed, sizeof(bits));
    for (unsigned i = 0; i < sizeof(data); ++i)
        data[i] = (uint8_t)(bits >> (8U * i));
    return can_fifo_tx_push(CHASSIS_MOTOR_TX_BASE_ID + index, data, sizeof(data));
}

void can_fifo_rx_callback(const can_fifo_rx_frame_t *received)
{
    if (received == NULL) return;
    const can_fifo_frame_t *frame = &received->frame;
    if (frame->dlc != 8U || frame->std_id < CHASSIS_MOTOR_RX_BASE_ID ||
        frame->std_id >= CHASSIS_MOTOR_RX_BASE_ID + CHASSIS_MOTOR_COUNT) return;
    if ((uint32_t)(xTaskGetTickCount() - received->receive_tick) >
        pdMS_TO_TICKS(CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)) return;
    const uint8_t *data = frame->data;
    unsigned index = frame->std_id - CHASSIS_MOTOR_RX_BASE_ID;
    motor_measure_t *motor = &motor_chassis[index];
    if (data[0] == index + 1U && data[1] == 0U && data[2] == 0x33U &&
        data[3] == MOTOR_VMAX_REGISTER)
    {
        fp32 range = read_float_le(&data[4]);
        if (isfinite(range) && range > 0.0f)
        {
            motor->velocity_range = range;
            motor->feedback_received = 0U;
            motor->velocity_range_valid = 1U;
        }
        return;
    }
    // 参数应答不作为电机实时反馈使用。
    if (data[0] == index + 1U && data[1] == 0U &&
        (data[2] == 0x33U || data[2] == 0x55U || data[2] == 0xAAU)) return;
    if ((data[0] & 0x0FU) != index + 1U) return;
    motor->state = data[0] >> 4;
    motor->position_raw = ((uint16_t)data[1] << 8) | data[2];
    motor->velocity_raw = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    motor->torque_raw = ((uint16_t)(data[4] & 0x0FU) << 8) | data[5];
    motor->mos_temperature = data[6];
    motor->rotor_temperature = data[7];
    motor->feedback_received = 1U;
    if (motor->velocity_range_valid)
    {
        motor->raw_speed_rad_s = ((fp32)motor->velocity_raw * 2.0f / 4095.0f - 1.0f) * motor->velocity_range;
        if (motor->velocity_raw == 2047U || motor->velocity_raw == 2048U)
        {
            motor->speed_rad_s = 0.0f;
        }
        else
        {
            motor->speed_rad_s = (index == 0U || index == 3U) ?
                                motor->raw_speed_rad_s : -motor->raw_speed_rad_s;
        }
    }
    detect_hook_at(CHASSIS_MOTOR1_TOE + index, received->receive_tick);
}

void chassis_motor_service(void)
{
    static uint32_t last_request_ms;
    static uint8_t next_motor;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_request_ms) < MOTOR_PARAMETER_PERIOD_MS) return;
    for (unsigned n = 0; n < CHASSIS_MOTOR_COUNT; ++n)
    {
        uint8_t index = next_motor;
        next_motor = (uint8_t)((next_motor + 1U) % CHASSIS_MOTOR_COUNT);
        if (motor_chassis[index].velocity_range_valid) continue;
        uint8_t data[8] = {index + 1U, 0U, 0x33U, MOTOR_VMAX_REGISTER, 0U, 0U, 0U, 0U};
        if (can_fifo_tx_push(0x7FFU, data, sizeof(data)) == HAL_OK) last_request_ms = now;
        break;
    }
}

void CAN_cmd_chassis(fp32 m1, fp32 m2, fp32 m3, fp32 m4, uint8_t enable_mask)
{
    static uint8_t last_enable[CHASSIS_MOTOR_COUNT];
    static uint8_t switch_sent[CHASSIS_MOTOR_COUNT];
    static uint32_t last_switch_ms[CHASSIS_MOTOR_COUNT];
    fp32 speeds[CHASSIS_MOTOR_COUNT] = {m1, m2, m3, m4};
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; ++i)
    {
        const motor_measure_t *motor = &motor_chassis[i];
        uint8_t enable = (enable_mask & (1U << i)) && isfinite(speeds[i]) &&
                         motor->velocity_range_valid && motor->feedback_received &&
                         !toe_is_error(CHASSIS_MOTOR1_TOE + i) && motor->state <= 1U;
        fp32 speed = enable && motor->state == 1U ? speeds[i] : 0.0f;
        if (motor->velocity_range_valid)
        {
            if (speed > motor->velocity_range) speed = motor->velocity_range;
            if (speed < -motor->velocity_range) speed = -motor->velocity_range;
        }
        // 先发送零速再使能，避免恢复使能时沿用驱动器内的旧速度。
        HAL_StatusTypeDef queued = send_speed(i, speed);
        if (queued == HAL_OK && (!switch_sent[i] || enable != last_enable[i] ||
            ((motor->state == 1U) != enable &&
             (uint32_t)(now - last_switch_ms[i]) >= MOTOR_SWITCH_RETRY_MS)))
        {
            uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, enable ? 0xFC : 0xFD};
            if (can_fifo_tx_push(CHASSIS_MOTOR_TX_BASE_ID + i, data, sizeof(data)) == HAL_OK)
            {
                switch_sent[i] = 1U;
                last_enable[i] = enable;
                last_switch_ms[i] = now;
            }
        }
    }
}

const motor_measure_t *get_chassis_motor_measure_point(uint8_t index)
{
    return index < CHASSIS_MOTOR_COUNT ? &motor_chassis[index] : 0;
}
