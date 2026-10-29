#ifndef USER_LIB_H
#define USER_LIB_H
#include "struct_typedef.h"

typedef struct __attribute__((packed))
{
    fp32 input;        //输入数据
    fp32 out;          //输出数据
    fp32 min_value;    //限幅最小值
    fp32 max_value;    //限幅最大值
    fp32 frame_period; //时间间隔
} ramp_function_source_t;

typedef struct __attribute__((packed))
{
    fp32 input;        //输入数据
    fp32 out;          //滤波输出的数据
    fp32 num[1];       //滤波参数
    fp32 frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;

/**
  * @brief          计算平方根的倒数
  * @param[in]      num: 输入数值
  * @retval         输入数值的平方根倒数；非正数返回 0
  */
extern fp32 invSqrt(fp32 num);

/**
  * @brief          初始化斜坡发生器及输出上下限
  * @param[out]     ramp_source_type: 斜坡状态
  * @param[in]      frame_period: 计算周期，单位 s
  * @param[in]      max: 输出上限
  * @param[in]      min: 输出下限
  * @retval         无
  */
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

/**
  * @brief          按变化率和计算周期累加斜坡输出并限幅
  * @param[in,out]  ramp_source_type: 斜坡状态
  * @param[in]      input: 每秒变化率
  * @retval         无
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);

/**
  * @brief          初始化一阶低通滤波器
  * @param[out]     first_order_filter_type: 滤波器状态
  * @param[in]      frame_period: 采样周期，单位 s
  * @param[in]      num: 滤波时间常数，单位 s，数组长度为 1
  * @retval         无
  */
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);

/**
  * @brief          输入一次采样并更新一阶低通滤波输出
  * @param[in,out]  first_order_filter_type: 滤波器状态
  * @param[in]      input: 当前输入值
  * @retval         无
  */
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);

/**
  * @brief          将数值限制在对称区间内
  * @param[in,out]  num: 待限幅数值
  * @param[in]      Limit: 绝对值上限，应为非负数
  * @retval         无
  */
extern void abs_limit(fp32 *num, fp32 Limit);

/**
  * @brief          获取数值的符号
  * @param[in]      value: 输入数值
  * @retval         非负返回 1，负数返回 -1
  */
extern fp32 sign(fp32 value);

/**
  * @brief          对输入执行开区间死区处理
  * @param[in]      Value: 输入值
  * @param[in]      minValue: 死区下界
  * @param[in]      maxValue: 死区上界
  * @retval         位于死区内返回 0，否则返回输入值
  */
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);

/**
  * @brief          对输入执行开区间死区处理
  * @param[in]      Value: 输入值
  * @param[in]      minValue: 死区下界
  * @param[in]      maxValue: 死区上界
  * @retval         位于死区内返回 0，否则返回输入值
  */
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);

/**
  * @brief          将输入限制在给定闭区间内
  * @param[in]      Value: 输入值
  * @param[in]      minValue: 下限
  * @param[in]      maxValue: 上限
  * @retval         限幅后的数值
  */
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);

/**
  * @brief          将输入限制在给定闭区间内
  * @param[in]      Value: 输入值
  * @param[in]      minValue: 下限
  * @param[in]      maxValue: 上限
  * @retval         限幅后的数值
  */
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);

/**
  * @brief          按区间长度循环折返超出范围的输入
  * @param[in]      Input: 输入值
  * @param[in]      minValue: 区间下界
  * @param[in]      maxValue: 区间上界
  * @note           区间长度应大于 0。
  * @retval         折返后的数值；上界小于下界时返回原值
  */
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);

/**
  * @brief          将角度循环归一到 -180～180 度
  * @param[in]      Ang: 输入角度，单位度
  * @retval         归一后的角度，单位度
  */
extern fp32 theta_format(fp32 Ang);

#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
