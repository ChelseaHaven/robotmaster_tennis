#ifndef AHRS_H
#define AHRS_H

#include "AHRS_middleware.h"

/**
  * @brief          将姿态四元数初始化为单位四元数
  * @param[out]     quat: 四元数，顺序 w、x、y、z
  * @param[in]      accel: 保留的加速度参数，当前未使用
  * @param[in]      mag: 保留的磁场参数，当前未使用
  * @retval         无
  */
extern void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3]);

/**
  * @brief          调用 Mahony 算法更新姿态四元数
  * @param[in,out]  quat: 姿态四元数，顺序 w、x、y、z
  * @param[in]      timing_time: 保留的周期参数，当前实现使用固定 1 kHz
  * @param[in]      gyro: 三轴角速度，单位 rad/s
  * @param[in]      accel: 三轴加速度，单位 m/s²
  * @param[in]      mag: 三轴磁场，单位 μT，全零时使用六轴解算
  * @retval         当前实现固定返回 1
  */
extern bool_t AHRS_update(fp32 quat[4], const fp32 timing_time, const fp32 gyro[3], const fp32 accel[3], const fp32 mag[3]);

/**
  * @brief          由四元数计算偏航角
  * @param[in]      quat: 四元数，顺序 w、x、y、z
  * @retval         偏航角，单位 rad
  */
extern fp32 get_yaw(const fp32 quat[4]);

/**
  * @brief          由四元数计算俯仰角
  * @param[in]      quat: 四元数，顺序 w、x、y、z
  * @retval         俯仰角，单位 rad
  */
extern fp32 get_pitch(const fp32 quat[4]);

/**
  * @brief          由四元数计算横滚角
  * @param[in]      quat: 四元数，顺序 w、x、y、z
  * @retval         横滚角，单位 rad
  */
extern fp32 get_roll(const fp32 quat[4]);

/**
  * @brief          由四元数计算偏航、俯仰和横滚角
  * @param[in]      quat: 四元数，顺序 w、x、y、z
  * @param[out]     yaw: 偏航角，单位 rad，允许为 NULL
  * @param[out]     pitch: 俯仰角，单位 rad，允许为 NULL
  * @param[out]     roll: 横滚角，单位 rad，允许为 NULL
  * @retval         无
  */
extern void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

/**
  * @brief          获取重力加速度常量
  * @retval         重力加速度 9.81 m/s²
  */
extern fp32 get_carrier_gravity(void);

#endif
