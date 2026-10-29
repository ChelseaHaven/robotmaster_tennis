/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      IMU 数据采集、温控和姿态解算。
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef INS_Task_H
#define INS_Task_H
#include "struct_typedef.h"


#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4


#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS    2
#define IMU_NOTIFY_SHFITS    3


#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

// IST8310 原始数据在接收缓冲区中的偏移
#define IST8310_RX_BUF_DATA_OFFSET 16


#define TEMPERATURE_PID_KP 1600.0f //温度控制PID的kp
#define TEMPERATURE_PID_KI 0.2f    //温度控制PID的ki
#define TEMPERATURE_PID_KD 0.0f    //温度控制PID的kd

#define TEMPERATURE_PID_MAX_OUT   4500.0f //温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f  //温度控制PID的max_iout

#define MPU6500_TEMP_PWM_MAX 5000 // IMU 加热定时器周期，最大 PWM 比较值为该值减 1


// 采集用于原始数据上发，与是否参与姿态融合独立。
#define INS_SAMPLE_MAGNETOMETER 1
#define INS_USE_MAGNETOMETER 0
#define INS_RAW_DATA_TIMEOUT_MS 100U

typedef struct {
    fp32 gyro[3];   // 传感器轴 XYZ，rad/s，未加软件零偏或安装旋转
    fp32 accel[3];  // 传感器轴 XYZ，m/s²，含重力，未低通或去重力
    fp32 mag[3];    // 传感器轴 XYZ，uT，未做磁场校准
    uint32_t last_update_ms;
    uint8_t ready;
    uint8_t mag_valid;
} INS_raw_data_t;

/**
  * @brief          在任务中复制最新 IMU 原始数据快照
  * @param[out]     snapshot: 原始角速度、加速度、磁场及有效标志
  * @note           磁力计无效时 mag_valid 为 0，磁场三轴填零。
  * @retval         1：BMI088 数据有效；0：空指针、无数据或数据超时
  */
uint8_t get_INS_raw_data_snapshot(INS_raw_data_t *snapshot);

#define INS_TASK_INIT_TIME 7 //任务开始初期 delay 一段时间

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2


/**
  * @brief          初始化 IMU 并完成数据采集、温控和姿态解算
  * @param[in]      pvParameters: 任务参数，未使用
  * @retval         无
  */
extern void INS_task(void const *pvParameters);


/**
  * @brief          执行一次陀螺仪零偏校准迭代
  * @param[out]     cali_scale: 三轴比例因子，当前均置为 1
  * @param[out]     cali_offset: 当前三轴零偏
  * @param[in,out]  time_count: 校准迭代次数，首次调用前置 0
  * @retval         无
  */
extern void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count);


/**
  * @brief          载入已保存的陀螺仪零偏
  * @param[in]      cali_scale: 保留的比例因子参数，当前未使用
  * @param[in]      cali_offset: 三轴零偏
  * @retval         无
  */
extern void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3]);


/**
  * @brief          获取当前姿态四元数
  * @retval         四元数只读指针，顺序 w、x、y、z
  */
extern const fp32 *get_INS_quat_point(void);


/**
  * @brief          获取当前欧拉角
  * @retval         偏航、俯仰、横滚角只读指针，单位 rad
  */
extern const fp32 *get_INS_angle_point(void);


/**
  * @brief          获取安装旋转和零偏补偿后的角速度
  * @retval         三轴角速度只读指针，顺序 X、Y、Z，单位 rad/s
  */
extern const fp32 *get_gyro_data_point(void);


/**
  * @brief          获取安装旋转和零偏补偿后的加速度
  * @retval         三轴加速度只读指针，顺序 X、Y、Z，单位 m/s²
  */
extern const fp32 *get_accel_data_point(void);


/**
  * @brief          获取参与姿态融合的磁场数据
  * @retval         三轴磁场只读指针，单位 μT，关闭磁力计融合时为零
  */
extern const fp32 *get_mag_data_point(void);

#endif
