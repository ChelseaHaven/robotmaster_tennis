#include "launcher_task.h"
#include "pressure.h"
#include "oid_motor.h"
#include "host_comm_task.h"
#include "detect_task.h"
#include "bsp_servo_pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

typedef enum {
    BALL_WAIT_SENSOR, BALL_WAIT_MOTOR, BALL_READY, BALL_MOVING,
    BALL_SETTLING, BALL_WAIT_BALL, BALL_LIMIT_BLOCKED, BALL_STOPPED
} launcher_ball_state_t;

typedef enum {
    BALL_RESULT_NONE, BALL_RESULT_COMPLETE, BALL_RESULT_SENSOR_OFFLINE,
    BALL_RESULT_NO_BALL, BALL_RESULT_TIMEOUT, BALL_RESULT_LIMIT, BALL_RESULT_LINK_LOST
} launcher_ball_result_t;

typedef enum {
    BALL_MOTION_NONE, BALL_MOTION_HOME, BALL_MOTION_FEED,
    BALL_MOTION_SEARCH, BALL_MOTION_REPOSITION
} launcher_ball_motion_t;

typedef struct {
    uint32_t sample_count;
    uint8_t candidate, count, known, has_ball;
} launcher_ball_sensor_t;

typedef struct {
    launcher_ball_state_t state;
    launcher_ball_result_t result;
    launcher_ball_motion_t motion;
    launcher_ball_sensor_t sensor[2];
    float angle_deg, from_deg, target_deg;
    uint32_t motion_ms, phase_start_ms, prepare_start_ms, search_start_ms, retry_start_ms;
    uint32_t completed_count;
    uint8_t slot, target_slot, settling, request_pending, prepare_pending;
    uint8_t search_started, stopped, checked_empty_mask, feed_link_lost;
    int8_t search_direction, feed_direction;
} launcher_ball_t;

static const float ball_angles[LAUNCHER_BALL_SLOT_COUNT] = {
    LAUNCHER_BALL_SLOT0_ANGLE_DEG, LAUNCHER_BALL_SLOT1_ANGLE_DEG,
    LAUNCHER_BALL_SLOT2_ANGLE_DEG, LAUNCHER_BALL_SLOT3_ANGLE_DEG
};
static const uint8_t ball_sensors[2] = {LAUNCHER_BALL_LEFT_SENSOR, LAUNCHER_BALL_RIGHT_SENSOR};
static const uint16_t ball_on[2] = {LAUNCHER_BALL_LEFT_ON, LAUNCHER_BALL_RIGHT_ON};
static const uint16_t ball_off[2] = {LAUNCHER_BALL_LEFT_OFF, LAUNCHER_BALL_RIGHT_OFF};

_Static_assert(LAUNCHER_BALL_START_SLOT < LAUNCHER_BALL_SLOT_COUNT, "Invalid startup slot");
_Static_assert(LAUNCHER_BALL_LEFT_SENSOR < PRESSURE_SENSOR_COUNT &&
               LAUNCHER_BALL_RIGHT_SENSOR < PRESSURE_SENSOR_COUNT &&
               LAUNCHER_BALL_LEFT_SENSOR != LAUNCHER_BALL_RIGHT_SENSOR, "Invalid pressure mapping");
_Static_assert(LAUNCHER_BALL_STABLE_SAMPLES > 0 && LAUNCHER_BALL_STABLE_SAMPLES <= 255,
               "Invalid pressure sample count");

/**
  * @brief          清除判球结果，后续必须取得新的压力采样才能再次动作
  * @param[in,out]  ball: 分球状态
  * @param[in]      pressure: 当前两路压力快照
  * @retval         无
  */
static void launcher_ball_clear_pressure(launcher_ball_t *ball, const pressure_data_t *pressure)
{
    for (unsigned i = 0; i < 2; ++i) {
        ball->sensor[i] = (launcher_ball_sensor_t){.sample_count = pressure[ball_sensors[i]].sample_count};
    }
}

/**
  * @brief          根据上下阈值及连续新采样次数更新左右有球状态
  * @param[in,out]  ball: 分球状态
  * @param[in]      pressure: 当前压力快照
  * @retval         两侧均在线且已取得稳定判定时返回 1
  */
static uint8_t launcher_ball_update_pressure(launcher_ball_t *ball, const pressure_data_t *pressure)
{
    uint8_t ready = 1U;
    for (unsigned i = 0; i < 2; ++i) {
        const pressure_data_t *data = &pressure[ball_sensors[i]];
        launcher_ball_sensor_t *sensor = &ball->sensor[i];
        if (toe_is_error(PRESSURE_SENSOR1_TOE + ball_sensors[i])) {
            sensor->known = sensor->count = 0U;
            sensor->sample_count = data->sample_count;
        } else if (data->sample_count != sensor->sample_count) {
            sensor->sample_count = data->sample_count;
            uint8_t sample = sensor->has_ball;
            if (data->value >= ball_on[i]) sample = 1U;
            else if (data->value <= ball_off[i]) sample = 0U;
            else if (!sensor->known) { sensor->count = 0U; ready = 0U; continue; }
            if (!sensor->count || sensor->candidate != sample) {
                sensor->candidate = sample;
                sensor->count = 1U;
            } else if (sensor->count < LAUNCHER_BALL_STABLE_SAMPLES) ++sensor->count;
            if (sensor->count >= LAUNCHER_BALL_STABLE_SAMPLES) {
                sensor->known = 1U;
                sensor->has_ball = sample;
            }
        }
        if (!sensor->known) ready = 0U;
    }
    return ready;
}

/**
  * @brief          开始一次工位移动，用角度插值控制动作时间
  * @param[in,out]  ball: 分球状态
  * @param[in]      motion: 上电定位、拨球、搜索或端点回移
  * @param[in]      slot: 目标工位，0～3
  * @param[in]      now_ms: 当前时间，单位 ms
  * @param[in]      pressure: 当前压力快照
  * @retval         无
  */
static void launcher_ball_move(launcher_ball_t *ball, launcher_ball_motion_t motion,
                               uint8_t slot, uint32_t now_ms, const pressure_data_t *pressure)
{
    ball->motion = motion;
    ball->target_slot = slot;
    ball->from_deg = ball->angle_deg;
    ball->target_deg = ball_angles[slot];
    ball->motion_ms = motion == BALL_MOTION_HOME ? LAUNCHER_BALL_HOME_MOVE_MS : LAUNCHER_BALL_MOVE_MS;
    ball->phase_start_ms = now_ms;
    ball->settling = 0U;
    ball->state = BALL_MOVING;
    if (motion == BALL_MOTION_FEED) {
        ball->feed_direction = slot > ball->slot ? 1 : -1;
        ball->feed_link_lost = 0U;
        ball->prepare_pending = 0U;
    }
    launcher_ball_clear_pressure(ball, pressure);
}

/**
  * @brief          接受一次上升沿拨球请求，运动中或已有请求时直接忽略
  * @param[in,out]  ball: 分球状态
  * @param[in]      now_ms: 当前时间，单位 ms
  * @param[in]      pressure: 当前压力快照
  * @param[in]      connected: 上位机发球目标是否有效
  * @retval         无
  */
static void launcher_ball_request(launcher_ball_t *ball, uint32_t now_ms,
                                  const pressure_data_t *pressure, uint8_t connected)
{
    if (!connected || ball->motion != BALL_MOTION_NONE || ball->request_pending) return;
    ball->request_pending = ball->prepare_pending = 1U;
    ball->search_started = ball->stopped = ball->checked_empty_mask = 0U;
    ball->prepare_start_ms = now_ms;
    ball->result = BALL_RESULT_NONE;
    launcher_ball_clear_pressure(ball, pressure);
}

/**
  * @brief          锁存搜索终止原因，新的拨球请求才能重新启动搜索
  * @param[in,out]  ball: 分球状态
  * @retval         无
  */
static void launcher_ball_stop_search(launcher_ball_t *ball)
{
    if (toe_is_error(PRESSURE_SENSOR1_TOE) || toe_is_error(PRESSURE_SENSOR2_TOE))
        ball->result = BALL_RESULT_SENSOR_OFFLINE;
    else if (ball->state == BALL_LIMIT_BLOCKED || ball->motion == BALL_MOTION_REPOSITION)
        ball->result = BALL_RESULT_LIMIT;
    else if (ball->search_started && ball->checked_empty_mask == 0x0FU)
        ball->result = BALL_RESULT_NO_BALL;
    else ball->result = BALL_RESULT_TIMEOUT;
    ball->request_pending = ball->prepare_pending = ball->search_started = 0U;
    ball->stopped = 1U;
    if (ball->motion == BALL_MOTION_NONE) ball->state = BALL_STOPPED;
}

/**
  * @brief          非阻塞推进分球动作、压力判定、四工位搜索和单次拨球
  * @param[in,out]  ball: 分球状态；angle_deg 仅为指令角，不是实测角
  * @param[in]      now_ms: 当前时间，单位 ms
  * @param[in]      pressure: 当前压力快照
  * @param[in]      connected: 上位机发球目标是否有效
  * @param[in]      motor_ready: 发球电机反馈是否在线且无故障
  * @retval         无
  */
static void launcher_ball_update(launcher_ball_t *ball, uint32_t now_ms,
                                 const pressure_data_t *pressure, uint8_t connected, uint8_t motor_ready)
{
    if (!connected && ball->motion == BALL_MOTION_FEED) ball->feed_link_lost = 1U;
    if (!connected && (ball->request_pending || ball->prepare_pending)) {
        ball->request_pending = ball->prepare_pending = ball->search_started = 0U;
        ball->result = BALL_RESULT_LINK_LOST;
    }
    if (ball->motion != BALL_MOTION_FEED && (ball->request_pending || ball->prepare_pending)) {
        uint8_t expired = ball->search_started ?
            (uint32_t)(now_ms - ball->search_start_ms) >= LAUNCHER_BALL_SEARCH_TIMEOUT_MS :
            (uint32_t)(now_ms - ball->prepare_start_ms) >= LAUNCHER_BALL_PREPARE_TIMEOUT_MS;
        if (expired) launcher_ball_stop_search(ball);
    }
    if (ball->motion != BALL_MOTION_NONE) {
        uint32_t elapsed = now_ms - ball->phase_start_ms;
        if (!ball->settling) {
            float ratio = elapsed >= ball->motion_ms ? 1.0f : (float)elapsed / ball->motion_ms;
            ball->angle_deg = ball->from_deg + (ball->target_deg - ball->from_deg) * ratio;
            servo_pwm_set_angle(SERVO_BALL_SEPARATOR, ball->angle_deg);
            if (elapsed >= ball->motion_ms) {
                ball->slot = ball->target_slot;
                ball->settling = 1U;
                ball->phase_start_ms = now_ms;
                ball->state = BALL_SETTLING;
            }
            return;
        }
        if (elapsed < LAUNCHER_BALL_SETTLE_MS) return;
        if (ball->motion == BALL_MOTION_FEED) {
            ++ball->completed_count;
            ball->request_pending = 0U;
            ball->result = BALL_RESULT_COMPLETE;
            ball->prepare_pending = connected && !ball->feed_link_lost && LAUNCHER_BALL_AUTO_PREPARE;
            ball->search_direction = ball->feed_direction;
            ball->prepare_start_ms = now_ms;
            ball->search_started = ball->checked_empty_mask = 0U;
        }
        ball->motion = BALL_MOTION_NONE;
        ball->state = ball->stopped ? BALL_STOPPED : BALL_WAIT_SENSOR;
        ball->retry_start_ms = now_ms;
        launcher_ball_clear_pressure(ball, pressure);
        return;
    }
    uint8_t pressure_ready = launcher_ball_update_pressure(ball, pressure);
    if (ball->stopped) { ball->state = BALL_STOPPED; return; }
    if (!pressure_ready) { ball->state = BALL_WAIT_SENSOR; return; }
    if (!connected || !motor_ready) { ball->state = BALL_WAIT_MOTOR; return; }
    if (ball->prepare_pending && !ball->search_started) {
        ball->search_started = 1U;
        ball->search_start_ms = now_ms;
    }
    uint8_t left = ball->sensor[0].has_ball;
    uint8_t right = ball->sensor[1].has_ball;
    uint8_t can_left = left && ball->slot > 0U;
    uint8_t can_right = right && ball->slot + 1U < LAUNCHER_BALL_SLOT_COUNT;
    if (can_left || can_right) {
        int8_t direction = can_left ? -1 : 1;
        if (can_left && can_right) direction = ball->slot <= 1U ? 1 : -1;
        ball->state = BALL_READY;
        ball->prepare_pending = 0U;
        if (ball->request_pending)
            launcher_ball_move(ball, BALL_MOTION_FEED, (uint8_t)(ball->slot + direction), now_ms, pressure);
        return;
    }
    if (left || right) {
        ball->state = BALL_LIMIT_BLOCKED;
        if (ball->prepare_pending && (uint32_t)(now_ms - ball->retry_start_ms) >= LAUNCHER_BALL_SEARCH_RETRY_MS) {
            if (ball->slot == 0U && left && !right)
                launcher_ball_move(ball, BALL_MOTION_REPOSITION, 1U, now_ms, pressure);
            else if (ball->slot == LAUNCHER_BALL_SLOT_COUNT - 1U && right && !left)
                launcher_ball_move(ball, BALL_MOTION_REPOSITION, ball->slot - 1U, now_ms, pressure);
        }
        return;
    }
    ball->state = BALL_WAIT_BALL;
    if (!ball->prepare_pending) return;
    ball->checked_empty_mask |= (uint8_t)(1U << ball->slot);
    if ((uint32_t)(now_ms - ball->retry_start_ms) < LAUNCHER_BALL_SEARCH_RETRY_MS) return;
    if (ball->slot == 0U) ball->search_direction = 1;
    if (ball->slot == LAUNCHER_BALL_SLOT_COUNT - 1U) ball->search_direction = -1;
    launcher_ball_move(ball, BALL_MOTION_SEARCH, (uint8_t)(ball->slot + ball->search_direction), now_ms, pressure);
}

void launcher_task(void const *argument)
{
    (void)argument;
    pressure_init();
    servo_pwm_init();
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_control = last_wake;
    TickType_t last_heartbeat = last_wake;
    TickType_t last_fault_query = last_wake;
    uint8_t query_motor = 0U;
    oid_motor_measure_t motors[OID_MOTOR_COUNT] = {0};
    pressure_data_t pressure[PRESSURE_SENSOR_COUNT] = {0};
    launcher_ball_t ball = {.angle_deg = LAUNCHER_BALL_SEPARATOR_ANGLE_DEG,
                            .search_direction = 1};
    pressure_get_snapshot(pressure);
    launcher_ball_move(&ball, BALL_MOTION_HOME, LAUNCHER_BALL_START_SLOT, HAL_GetTick(), pressure);
    host_launcher_setpoint_t setpoint = {0};
    float pitch_deg = LAUNCHER_PITCH_ANGLE_DEG;
    host_feed_payload_t feed_command = {0};

    for (;;) {
        pressure_update();
        pressure_get_snapshot(pressure);
        oid_motor_get_snapshot(motors);
        uint8_t target_valid = host_comm_get_launcher_setpoint(&setpoint);
        uint32_t now_ms = HAL_GetTick();
        if (host_comm_get_launcher_feed(&feed_command))
            launcher_ball_request(&ball, now_ms, pressure, target_valid);
        uint8_t motor_ready = 1U;
        for (unsigned i = 0; i < OID_MOTOR_COUNT; ++i) {
            if (!motors[i].speed_received || !motors[i].fault_received || motors[i].fault ||
                toe_is_error(LAUNCHER_MOTOR1_TOE + i)) motor_ready = 0U;
        }
        launcher_ball_update(&ball, now_ms, pressure, target_valid, motor_ready);
        if (setpoint.received) {
            pitch_deg = setpoint.target.pitch_deg;
            if (pitch_deg < 0.0f) pitch_deg = 0.0f;
            if (pitch_deg > SERVO_LAUNCHER_PITCH_MAX_ANGLE_DEG) pitch_deg = SERVO_LAUNCHER_PITCH_MAX_ANGLE_DEG;
        }
        TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - last_control) >= pdMS_TO_TICKS(LAUNCHER_MOTOR_CONTROL_PERIOD_MS)) {
            int32_t erpm[OID_MOTOR_COUNT] = {0};
            if (target_valid) {
                float rpm[OID_MOTOR_COUNT] = {setpoint.target.motor1_rpm, setpoint.target.motor2_rpm};
                for (unsigned i = 0; i < OID_MOTOR_COUNT; ++i) {
                    float max_rpm = (float)OID_MOTOR_MAX_ERPM / OID_MOTOR_POLE_PAIRS;
                    if (rpm[i] > max_rpm) rpm[i] = max_rpm;
                    erpm[i] = (int32_t)(rpm[i] * OID_MOTOR_POLE_PAIRS);
                }
            }
            oid_motor_set_speed(erpm[0], erpm[1]);
            last_control = now;
        }
        if ((TickType_t)(now - last_heartbeat) >= pdMS_TO_TICKS(LAUNCHER_MOTOR_HEARTBEAT_PERIOD_MS)) {
            for (uint8_t i = 0; i < OID_MOTOR_COUNT; ++i) oid_motor_send_heartbeat(i);
            last_heartbeat = now;
        }
        if ((TickType_t)(now - last_fault_query) >= pdMS_TO_TICKS(LAUNCHER_MOTOR_FAULT_QUERY_PERIOD_MS)) {
            if (oid_motor_query_fault(query_motor) == HAL_OK) {
                query_motor = (uint8_t)((query_motor + 1U) % OID_MOTOR_COUNT);
                last_fault_query = now;
            }
        }
        servo_pwm_set_angle(SERVO_LAUNCHER_PITCH, pitch_deg);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LAUNCHER_PERIOD_MS));
    }
}
