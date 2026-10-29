// MahonyAHRS.h
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
#ifndef MahonyAHRS_h
#define MahonyAHRS_h


extern volatile float twoKp;			// 比例增益 Kp 的 2 倍
extern volatile float twoKi;			// 积分增益 Ki 的 2 倍
extern volatile float q0, q1, q2, q3;	// 传感器相对参考系的四元数

/**
  * @brief          融合陀螺仪、加速度和磁场更新 Mahony 姿态
  * @param[in,out]  q: 四元数，顺序 w、x、y、z
  * @param[in]      gx: X 轴角速度，rad/s
  * @param[in]      gy: Y 轴角速度，rad/s
  * @param[in]      gz: Z 轴角速度，rad/s
  * @param[in]      ax: X 轴加速度
  * @param[in]      ay: Y 轴加速度
  * @param[in]      az: Z 轴加速度
  * @param[in]      mx: X 轴磁场
  * @param[in]      my: Y 轴磁场
  * @param[in]      mz: Z 轴磁场
  * @note           按 sampleFreq 指定的固定周期调用；积分反馈状态由模块共享。
  * @retval         无
  */
void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

/**
  * @brief          融合陀螺仪和加速度更新六轴 Mahony 姿态
  * @param[in,out]  q: 四元数，顺序 w、x、y、z
  * @param[in]      gx: X 轴角速度，rad/s
  * @param[in]      gy: Y 轴角速度，rad/s
  * @param[in]      gz: Z 轴角速度，rad/s
  * @param[in]      ax: X 轴加速度
  * @param[in]      ay: Y 轴加速度
  * @param[in]      az: Z 轴加速度
  * @note           按 sampleFreq 指定的固定周期调用；积分反馈状态由模块共享。
  * @retval         无
  */
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);

#endif
