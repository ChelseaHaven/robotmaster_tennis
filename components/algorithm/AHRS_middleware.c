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

#include "AHRS_middleware.h"
#include "AHRS.h"
#include "arm_math.h"
#include "main.h"


void AHRS_get_height(fp32* high)
{
    if (high != NULL)
    {
        *high = 0.0f;
    }
}


void AHRS_get_latitude(fp32* latitude)
{
    if (latitude != NULL)
    {
        *latitude = 22.0f;
    }
}


fp32 AHRS_invSqrt(fp32 num)
{
    return 1/sqrtf(num);

}


fp32 AHRS_sinf(fp32 angle)
{
    return arm_sin_f32(angle);
}


fp32 AHRS_cosf(fp32 angle)
{
    return arm_cos_f32(angle);
}


fp32 AHRS_tanf(fp32 angle)
{
    return tanf(angle);
}


fp32 AHRS_asinf(fp32 sin)
{

    return asinf(sin);
}


fp32 AHRS_acosf(fp32 cos)
{

    return acosf(cos);
}


fp32 AHRS_atan2f(fp32 y, fp32 x)
{
    return atan2f(y, x);
}
