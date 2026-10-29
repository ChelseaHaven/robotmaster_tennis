/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310.c/h
  * @brief      IST8310磁力计驱动函数，包括初始化函数，处理数据函数，通信读取函数
  *             通过 C 板 I2C3 读取磁场数据。
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

#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H

#include "struct_typedef.h"

#define IST8310_DATA_READY_BIT 2

#define IST8310_NO_ERROR 0x00

#define IST8310_NO_SENSOR 0x40

typedef struct ist8310_real_data_t
{
  uint8_t status;
  fp32 mag[3];
} ist8310_real_data_t;

/**
  * @brief          复位 IST8310 并写入配置，逐项读回校验
  * @retval         0：成功；非 0：设备 ID 或配置错误码
  */
extern uint8_t ist8310_init(void);

/**
  * @brief          解析带状态字节的 IST8310 接收数据
  * @param[in]      status_buf: 状态及三轴磁场，共 7 字节
  * @param[in,out]  mpu6500_real_data: 磁力计数据和就绪标志，磁场单位 μT
  * @retval         无
  */
extern void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *mpu6500_real_data);

/**
  * @brief          读取三轴磁场，通信失败时保留输出原值
  * @param[out]     mag: 三轴磁场，单位 μT
  * @retval         无
  */
extern void ist8310_read_mag(fp32 mag[3]);

/**
  * @brief          读取三轴磁场并返回通信结果
  * @param[out]     mag: 三轴磁场，单位 μT，失败时保持原值
  * @retval         1：读取成功；0：参数无效或通信失败
  */
uint8_t ist8310_read_mag_checked(fp32 mag[3]);
#endif
