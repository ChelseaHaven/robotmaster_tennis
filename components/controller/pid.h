/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note
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
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

/**
  * @brief          初始化 PID 参数、模式、限幅及内部状态
  * @param[out]     pid: PID 状态
  * @param[in]      mode: PID_POSITION 位置式或 PID_DELTA 增量式
  * @param[in]      PID: 比例、积分、微分系数数组
  * @param[in]      max_out: 输出绝对值上限
  * @param[in]      max_iout: 积分输出绝对值上限
  * @retval         无
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);


/**
  * @brief          更新一次 PID 计算并限制输出
  * @param[in,out]  pid: PID 参数与状态
  * @param[in]      ref: 反馈值
  * @param[in]      set: 目标值
  * @retval         本次 PID 输出
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);


/**
  * @brief          清零 PID 误差和输出状态，保留系数及限幅
  * @param[in,out]  pid: PID 状态
  * @retval         无
  */
extern void PID_clear(pid_type_def *pid);

#endif
