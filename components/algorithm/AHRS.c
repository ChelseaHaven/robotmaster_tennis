#include "AHRS.h"
#include "MahonyAHRS.h"

#include <math.h>

/* The reference project ships AHRS only as a Keil library. This GCC adapter
 * keeps its public interface and uses the Mahony source from 18.ins_task. */
void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3])
{
    (void)accel;
    (void)mag;
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

bool_t AHRS_update(fp32 quat[4], const fp32 timing_time,
                   const fp32 gyro[3], const fp32 accel[3], const fp32 mag[3])
{
    (void)timing_time; /* Mahony 固定按 1 kHz 解算。 */
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2],
                     accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
    return 1;
}

fp32 get_yaw(const fp32 quat[4])
{
    return atan2f(2.0f * (quat[0] * quat[3] + quat[1] * quat[2]),
                  1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3]));
}

fp32 get_pitch(const fp32 quat[4])
{
    fp32 value = 2.0f * (quat[0] * quat[2] - quat[1] * quat[3]);
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;
    return asinf(value);
}

fp32 get_roll(const fp32 quat[4])
{
    return atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]),
                  1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]));
}

void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    if (yaw != NULL) *yaw = get_yaw(quat);
    if (pitch != NULL) *pitch = get_pitch(quat);
    if (roll != NULL) *roll = get_roll(quat);
}

fp32 get_carrier_gravity(void)
{
    return 9.81f;
}
