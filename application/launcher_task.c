#include "launcher_task.h"
#include "pressure.h"
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

    for (;;) {
        pressure_update();
        servo_pwm_set_angle(SERVO_BALL_SEPARATOR, LAUNCHER_BALL_SEPARATOR_ANGLE_DEG);
        servo_pwm_set_angle(SERVO_LAUNCHER_PITCH, LAUNCHER_PITCH_ANGLE_DEG);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LAUNCHER_PERIOD_MS));
    }
}
