
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       AHRS_MiddleWare.c/h
  * @brief      姿态解算中间层，为姿态解算提供相关函数
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef AHRS_MIDDLEWARE_H
#define AHRS_MIDDLEWARE_H

#include "struct_typedef.h"

#ifndef NULL
#define NULL 0
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f
#endif

#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

/**
  * @brief          提供姿态算法使用的高度参数，当前固定为 0
  * @param[out]     high: 高度，单位 m，允许为 NULL
  * @retval         无
  */
extern void AHRS_get_height(fp32 *high);

/**
  * @brief          提供姿态算法使用的纬度参数，当前固定为 22
  * @param[out]     latitude: 纬度，单位度，允许为 NULL
  * @retval         无
  */
extern void AHRS_get_latitude(fp32 *latitude);

/**
  * @brief          计算平方根的倒数
  * @param[in]      num: 输入数值
  * @retval         输入数值的平方根倒数
  */
extern fp32 AHRS_invSqrt(fp32 num);

/**
  * @brief          计算正弦
  * @param[in]      angle: 角度，单位 rad
  * @retval         正弦值
  */
extern fp32 AHRS_sinf(fp32 angle);

/**
  * @brief          计算余弦
  * @param[in]      angle: 角度，单位 rad
  * @retval         余弦值
  */
extern fp32 AHRS_cosf(fp32 angle);

/**
  * @brief          计算正切
  * @param[in]      angle: 角度，单位 rad
  * @retval         正切值
  */
extern fp32 AHRS_tanf(fp32 angle);

/**
  * @brief          计算反正弦
  * @param[in]      sin: 输入值，范围 -1～1
  * @retval         角度，单位 rad
  */
extern fp32 AHRS_asinf(fp32 sin);

/**
  * @brief          计算反余弦
  * @param[in]      cos: 输入值，范围 -1～1
  * @retval         角度，单位 rad
  */
extern fp32 AHRS_acosf(fp32 cos);

/**
  * @brief          根据二维坐标计算象限角
  * @param[in]      y: 纵坐标
  * @param[in]      x: 横坐标
  * @retval         角度，单位 rad
  */
extern fp32 AHRS_atan2f(fp32 y, fp32 x);
#endif
