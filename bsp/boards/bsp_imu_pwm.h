#ifndef BSP_IMU_PWM_H
#define BSP_IMU_PWM_H
#include "struct_typedef.h"

/**
  * @brief          设置 IMU 加热 PWM 比较值
  * @param[in]      pwm: 定时器比较值，由温控 PID 限幅
  * @retval         无
  */
extern void imu_pwm_set(uint16_t pwm);

#endif
