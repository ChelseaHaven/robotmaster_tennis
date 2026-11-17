#include "launcher_task.h"
#include "pressure.h"
#include "oid_motor.h"
#include "host_comm_task.h"
#include "bsp_servo_pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

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
    // oid_motor_measure_t motors[OID_MOTOR_COUNT] = {0};
    host_launcher_setpoint_t setpoint = {0};
    float pitch_deg = LAUNCHER_PITCH_ANGLE_DEG;
    host_feed_payload_t feed_command = {0};

    for (;;) {
        pressure_update();
        // oid_motor_get_snapshot(motors);
        if (host_comm_get_launcher_feed(&feed_command)) {
            // 在此交给分球状态机；忙时丢弃，本次命令不缓存、不补执行。
            // 分球动作流程尚未移植，当前仅接收一次 0→1 拨球事件。
        }
        uint8_t target_valid = host_comm_get_launcher_setpoint(&setpoint);
        if (setpoint.received) {
            pitch_deg = setpoint.target.pitch_deg;
            if (pitch_deg < 0.0f) pitch_deg = 0.0f;
            if (pitch_deg > SERVO_MAX_ANGLE_DEG) pitch_deg = SERVO_MAX_ANGLE_DEG;
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
        servo_pwm_set_angle(SERVO_BALL_SEPARATOR, LAUNCHER_BALL_SEPARATOR_ANGLE_DEG);
        servo_pwm_set_angle(SERVO_LAUNCHER_PITCH, pitch_deg);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LAUNCHER_PERIOD_MS));
    }
}
