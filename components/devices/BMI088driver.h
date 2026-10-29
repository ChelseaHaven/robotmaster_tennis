#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#include "struct_typedef.h"
#include "main.h"

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM  6
#define BMI088_WRITE_GYRO_REG_NUM   6

#define BMI088_GYRO_DATA_READY_BIT          0
#define BMI088_ACCEL_DATA_READY_BIT         1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT    2

#define BMI088_LONG_DELAY_TIME      80
#define BMI088_COM_WAIT_SENSOR_TIME 150


#define BMI088_ACCEL_IIC_ADDRESSE   (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE    (0x68 << 1)

#define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
//#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125


#define BMI088_ACCEL_3G_SEN     0.0008974358974f
#define BMI088_ACCEL_6G_SEN     0.00179443359375f
#define BMI088_ACCEL_12G_SEN    0.0035888671875f
#define BMI088_ACCEL_24G_SEN    0.007177734375f


#define BMI088_GYRO_2000_SEN    0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN    0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN     0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN     0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN     0.000066579027251980956150958662738366f


typedef struct __attribute__((packed)) BMI088_RAW_DATA
{
    uint8_t status;
    int16_t accel[3];
    int16_t temp;
    int16_t gyro[3];
} bmi088_raw_data_t;

typedef struct BMI088_REAL_DATA
{
    uint8_t status;
    fp32 accel[3];
    fp32 temp;
    fp32 gyro[3];
    fp32 time;
} bmi088_real_data_t;


enum
{
    BMI088_NO_ERROR                     = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR           = 0x01,
    BMI088_ACC_PWR_CONF_ERROR           = 0x02,
    BMI088_ACC_CONF_ERROR               = 0x03,
    BMI088_ACC_SELF_TEST_ERROR          = 0x04,
    BMI088_ACC_RANGE_ERROR              = 0x05,
    BMI088_INT1_IO_CTRL_ERROR           = 0x06,
    BMI088_INT_MAP_DATA_ERROR           = 0x07,
    BMI088_GYRO_RANGE_ERROR             = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR         = 0x09,
    BMI088_GYRO_LPM1_ERROR              = 0x0A,
    BMI088_GYRO_CTRL_ERROR              = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR        = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR         = 0x40,
    BMI088_NO_SENSOR                    = 0xFF,
};


/**
  * @brief          执行 BMI088 自检并初始化加速度计和陀螺仪
  * @retval         0：成功；非 0：BMI088 错误码或错误位组合
  */
extern uint8_t BMI088_init(void);

/**
  * @brief          执行 BMI088 加速度计硬件自检
  * @retval         0：自检通过；非 0：自检错误码
  */
extern bool_t bmi088_accel_self_test(void);

/**
  * @brief          执行 BMI088 陀螺仪硬件自检
  * @retval         0：自检通过；非 0：自检错误码
  */
extern bool_t bmi088_gyro_self_test(void);

/**
  * @brief          复位并配置 BMI088 加速度计，读回校验寄存器
  * @retval         0：成功；非 0：初始化错误码
  */
extern bool_t bmi088_accel_init(void);

/**
  * @brief          复位并配置 BMI088 陀螺仪，读回校验寄存器
  * @retval         0：成功；非 0：初始化错误码
  */
extern bool_t bmi088_gyro_init(void);

/**
  * @brief          解析 BMI088 加速度和传感器时间
  * @param[in]      rx_buf: 加速度寄存器起始的 9 字节数据
  * @param[out]     accel: 三轴加速度，单位 m/s²
  * @param[out]     time: 传感器时间，单位 μs
  * @retval         无
  */
extern void BMI088_accel_read_over(uint8_t *rx_buf, fp32 accel[3], fp32 *time);

/**
  * @brief          解析 BMI088 陀螺仪接收数据
  * @param[in]      rx_buf: 角速度寄存器起始的 6 字节数据
  * @param[out]     gyro: 三轴角速度，单位 rad/s
  * @retval         无
  */
extern void BMI088_gyro_read_over(uint8_t *rx_buf, fp32 gyro[3]);

/**
  * @brief          解析 BMI088 温度接收数据
  * @param[in]      rx_buf: 温度寄存器起始的 2 字节数据
  * @param[out]     temperate: 温度，单位 ℃
  * @retval         无
  */
extern void BMI088_temperature_read_over(uint8_t *rx_buf, fp32 *temperate);

/**
  * @brief          通过阻塞 SPI 读取 BMI088 角速度、加速度和温度
  * @param[out]     gyro: 三轴角速度，单位 rad/s，芯片 ID 无效时保持原值
  * @param[out]     accel: 三轴加速度，单位 m/s²
  * @param[out]     temperate: 温度，单位 ℃
  * @retval         无
  */
extern void BMI088_read(fp32 gyro[3], fp32 accel[3], fp32 *temperate);

/**
  * @brief          读取 BMI088 传感器时间寄存器
  * @retval         24 位原始计数，每计数约 39.0625 μs
  */
extern uint32_t get_BMI088_sensor_time(void);

/**
  * @brief          读取 BMI088 温度
  * @retval         传感器温度，单位 ℃
  */
extern fp32 get_BMI088_temperate(void);

/**
  * @brief          读取 BMI088 原始三轴陀螺仪计数
  * @param[out]     gyro: 三轴有符号原始计数，尚未换算 rad/s
  * @retval         无
  */
extern void get_BMI088_gyro(int16_t gyro[3]);

/**
  * @brief          读取并换算 BMI088 三轴加速度
  * @param[out]     accel: 三轴加速度，单位 m/s²
  * @retval         无
  */
extern void get_BMI088_accel(fp32 accel[3]);


/**
  * @brief          读取并缓存 BMI088 陀螺仪芯片 ID，供调试查看
  * @retval         无
  */
extern void BMI088_read_gyro_who_am_i(void);

/**
  * @brief          读取并缓存 BMI088 加速度计芯片 ID，供调试查看
  * @retval         无
  */
extern void BMI088_read_accel_who_am_i(void);

#endif
