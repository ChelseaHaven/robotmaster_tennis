/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310middleware.c/h
  * @brief      IST8310 磁力计中间层，通过 I2C3 完成寄存器通信。
  * @note       IST8310只支持IIC读取
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef IST8310DRIVER_MIDDLEWARE_H
#define IST8310DRIVER_MIDDLEWARE_H

#include "struct_typedef.h"

#define IST8310_IIC_ADDRESS (0x0E << 1)  //IST8310的IIC地址
#define IST8310_IIC_READ_MSB (0x80) // 寄存器读取位掩码

/**
  * @brief          保留传感器 GPIO 初始化接口，实际配置由 CubeMX 完成
  * @retval         无
  */
extern void ist8310_GPIO_init(void);

/**
  * @brief          保留传感器通信初始化接口，实际配置由 CubeMX 完成
  * @retval         无
  */
extern void ist8310_com_init(void);

/**
  * @brief          通过 I2C 读取 IST8310 单个寄存器
  * @param[in]      reg: 寄存器地址
  * @retval         读到的寄存器值
  */
extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);

/**
  * @brief          通过 I2C 写入 IST8310 单个寄存器
  * @param[in]      reg: 寄存器地址
  * @param[in]      data: 待写值
  * @retval         无
  */
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);

/**
  * @brief          通过 I2C 连续读取 IST8310 寄存器
  * @param[in]      reg: 起始寄存器地址
  * @param[out]     buf: 接收缓冲区
  * @param[in]      len: 读取字节数
  * @retval         无
  */
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

/**
  * @brief          通过 I2C 连续写入 IST8310 寄存器
  * @param[in]      reg: 起始寄存器地址
  * @param[in]      data: 待写缓冲区
  * @param[in]      len: 写入字节数
  * @retval         无
  */
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);

/**
  * @brief          执行传感器所需的毫秒延时
  * @param[in]      ms: 延时时间，单位 ms
  * @retval         无
  */
extern void ist8310_delay_ms(uint16_t ms);

/**
  * @brief          执行传感器所需的微秒延时
  * @param[in]      us: 延时时间，单位 μs
  * @retval         无
  */
extern void ist8310_delay_us(uint16_t us);

/**
  * @brief          拉高 IST8310 复位引脚，释放复位
  * @retval         无
  */
extern void ist8310_RST_H(void);

/**
  * @brief          拉低 IST8310 复位引脚，使器件复位
  * @retval         无
  */
extern void ist8310_RST_L(void);

/**
  * @brief          通过 I2C 读取 IST8310 并检查通信结果
  * @param[in]      reg: 起始寄存器地址
  * @param[out]     buf: 接收缓冲区
  * @param[in]      len: 读取字节数
  * @note           本接口的 HAL 超时为 3 ms。
  * @retval         1：读取成功；0：通信失败
  */
uint8_t ist8310_IIC_read_muli_reg_checked(uint8_t reg, uint8_t *buf, uint8_t len);

#endif
