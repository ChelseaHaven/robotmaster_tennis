#ifndef BSP_SERVO_PWM_H
#define BSP_SERVO_PWM_H
#include <stdint.h>

typedef enum {
    SERVO_BALL_SEPARATOR = 0,   /* C1：分球器，每次仅放行一个球；DS51150-12V，外接 12 V。 */
    SERVO_LAUNCHER_PITCH,     /* C2：调整发球 pitch 角；DS3230，5 V 供电。 */
    SERVO_COUNT
} servo_id_t;

#define SERVO_PWM_MIN_US       500U
#define SERVO_PWM_CENTER_US    1500U
#define SERVO_PWM_MAX_US       2500U
#define SERVO_MAX_ANGLE_DEG    270.0f
#define SERVO_STARTUP_US       SERVO_PWM_CENTER_US

/**
  * @brief          开启 C1、C2 的 50 Hz PWM，默认中位 135°
  * @note           在 MX_TIM1_Init 后调用一次；硬件持续输出，不需要独立任务
  * @retval         无
  */
void servo_pwm_init(void);
/**
  * @brief          设置舵机高电平脉宽，限幅到 500～2500 μs；停止输出后也可直接恢复
  * @param[in]      servo: SERVO_BALL_SEPARATOR 或 SERVO_LAUNCHER_PITCH
  * @param[in]      pulse_us: 脉宽，单位 μs
  * @retval         无；无效编号忽略
  */
void servo_pwm_set_us(servo_id_t servo, uint16_t pulse_us);
/**
  * @brief          设置舵机目标角度，限幅到 0～270°，135°为中位
  * @param[in]      servo: SERVO_BALL_SEPARATOR 或 SERVO_LAUNCHER_PITCH
  * @param[in]      angle_deg: 角度，单位度，NaN/Inf 忽略
  * @note           角度是指令值，没有实际角度反馈
  * @retval         无
  */
void servo_pwm_set_angle(servo_id_t servo, float angle_deg);
/**
  * @brief          将对应通道脉宽置零，停止输出控制脉冲
  * @param[in]      servo: SERVO_BALL_SEPARATOR 或 SERVO_LAUNCHER_PITCH
  * @note           不切断舵机电源，停止脉冲后的保持行为由舵机决定
  * @retval         无
  */
void servo_pwm_off(servo_id_t servo);
#endif
