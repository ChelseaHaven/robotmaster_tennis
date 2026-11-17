#include "launcher_task.h"
#include "pressure.h"
#include "oid_motor.h"
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
    oid_motor_measure_t motors[OID_MOTOR_COUNT] = {0};

    for (;;) {
        pressure_update();
        oid_motor_get_snapshot(motors);
        TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - last_control) >= pdMS_TO_TICKS(LAUNCHER_MOTOR_CONTROL_PERIOD_MS)) {
            oid_motor_set_speed(LAUNCHER_MOTOR1_TARGET_ERPM, LAUNCHER_MOTOR2_TARGET_ERPM);
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
        servo_pwm_set_angle(SERVO_LAUNCHER_PITCH, LAUNCHER_PITCH_ANGLE_DEG);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LAUNCHER_PERIOD_MS));
    }
}
