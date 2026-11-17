#include "bsp_servo_pwm.h"
#include "tim.h"
#include <math.h>

static const uint32_t servo_channels[SERVO_COUNT] = {TIM_CHANNEL_1, TIM_CHANNEL_2};

void servo_pwm_init(void)
{
    for (unsigned i = 0; i < SERVO_COUNT; i++) {
        servo_pwm_off((servo_id_t)i);
        if (HAL_TIM_PWM_Start(&htim1, servo_channels[i]) != HAL_OK) Error_Handler();
    }
}

void servo_pwm_set_us(servo_id_t servo, uint16_t pulse_us)
{
    if ((unsigned)servo >= SERVO_COUNT) return;
    if (pulse_us < SERVO_PWM_MIN_US) pulse_us = SERVO_PWM_MIN_US;
    if (pulse_us > SERVO_PWM_MAX_US) pulse_us = SERVO_PWM_MAX_US;
    __HAL_TIM_SET_COMPARE(&htim1, servo_channels[servo], pulse_us);
}

void servo_pwm_set_angle(servo_id_t servo, float angle_deg)
{
    if ((unsigned)servo >= SERVO_COUNT || !isfinite(angle_deg)) return;
    float max_angle = servo == SERVO_BALL_SEPARATOR ?
        SERVO_BALL_SEPARATOR_MAX_ANGLE_DEG : SERVO_LAUNCHER_PITCH_MAX_ANGLE_DEG;
    if (angle_deg < 0.0f) angle_deg = 0.0f;
    if (angle_deg > max_angle) angle_deg = max_angle;
    uint16_t pulse = (uint16_t)(SERVO_PWM_MIN_US +
        angle_deg / max_angle * (SERVO_PWM_MAX_US - SERVO_PWM_MIN_US) + 0.5f);
    servo_pwm_set_us(servo, pulse);
}

void servo_pwm_off(servo_id_t servo)
{
    if ((unsigned)servo >= SERVO_COUNT) return;
    __HAL_TIM_SET_COMPARE(&htim1, servo_channels[servo], 0U);
}
