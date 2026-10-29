#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "struct_typedef.h"

#define BMI088_USE_SPI

/**
  * @brief          保留传感器 GPIO 初始化接口，实际配置由 CubeMX 完成
  * @retval         无
  */
extern void BMI088_GPIO_init(void);

/**
  * @brief          保留传感器通信初始化接口，实际配置由 CubeMX 完成
  * @retval         无
  */
extern void BMI088_com_init(void);

/**
  * @brief          执行传感器所需的毫秒延时
  * @param[in]      ms: 延时时间，单位 ms
  * @retval         无
  */
extern void BMI088_delay_ms(uint16_t ms);

/**
  * @brief          执行传感器所需的微秒延时
  * @param[in]      us: 延时时间，单位 μs
  * @retval         无
  */
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)

/**
  * @brief          拉低并选中 BMI088 加速度计片选
  * @retval         无
  */
extern void BMI088_ACCEL_NS_L(void);

/**
  * @brief          拉高并释放 BMI088 加速度计片选
  * @retval         无
  */
extern void BMI088_ACCEL_NS_H(void);

/**
  * @brief          拉低并选中 BMI088 陀螺仪片选
  * @retval         无
  */
extern void BMI088_GYRO_NS_L(void);

/**
  * @brief          拉高并释放 BMI088 陀螺仪片选
  * @retval         无
  */
extern void BMI088_GYRO_NS_H(void);

/**
  * @brief          通过 SPI1 同时发送和接收一个字节
  * @param[in]      reg: 待发送字节
  * @retval         收到的字节
  */
extern uint8_t BMI088_read_write_byte(uint8_t reg);

#elif defined(BMI088_USE_IIC)

#endif

#endif
