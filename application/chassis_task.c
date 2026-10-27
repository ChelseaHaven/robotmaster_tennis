#include <stdio.h>
#include "chassis_task.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "pid.h"
#include "cmsis_os.h"
#include <math.h>

#define CHASSIS_PERIOD_MS 2U
#define MAX_WHEEL_SPEED 4.0f
#define RPM_TO_MPS 0.0004158097489034945f
#define MOTOR_DISTANCE_TO_CENTER 0.2f

static volatile fp32 cmd_vx;
static volatile fp32 cmd_vy;
static volatile fp32 cmd_wz;

void chassis_set_velocity(fp32 vx, fp32 vy, fp32 wz)
{
    cmd_vx = vx;
    cmd_vy = vy;
    cmd_wz = wz;
}

void chassis_task(void const *argument)
{
    (void)argument;
    static const fp32 gains[3] = {15000.0f, 10.0f, 0.0f};
    pid_type_def speed_pid[4];
    for (unsigned i = 0; i < 4; ++i)
        PID_init(&speed_pid[i], PID_POSITION, gains, 16000.0f, 2000.0f);

    for (;;)
    {
        fp32 vx = cmd_vx, vy = cmd_vy, wz = cmd_wz;
        if (vx == 0.0f && vy == 0.0f && wz == 0.0f)
        {
            for (unsigned i = 0; i < 4; ++i) PID_clear(&speed_pid[i]);
            CAN_cmd_chassis(0, 0, 0, 0);
            osDelay(CHASSIS_PERIOD_MS);
            continue;
        }
        fp32 wheel[4] = {
            -vx - vy - 0.9f * MOTOR_DISTANCE_TO_CENTER * wz,
             vx - vy - 0.9f * MOTOR_DISTANCE_TO_CENTER * wz,
             vx + vy - 1.1f * MOTOR_DISTANCE_TO_CENTER * wz,
            -vx + vy - 1.1f * MOTOR_DISTANCE_TO_CENTER * wz,
        };
        fp32 maximum = 0.0f;
        for (unsigned i = 0; i < 4; ++i)
            if (fabsf(wheel[i]) > maximum) maximum = fabsf(wheel[i]);
        if (maximum > MAX_WHEEL_SPEED)
            for (unsigned i = 0; i < 4; ++i) wheel[i] *= MAX_WHEEL_SPEED / maximum;

        int16_t current[4] = {0};
        for (unsigned i = 0; i < 4; ++i)
        {
            if (toe_is_error(CHASSIS_MOTOR1_TOE + i))
            {
                PID_clear(&speed_pid[i]);
                continue;
            }
            const motor_measure_t *motor = get_chassis_motor_measure_point(i);
            current[i] = (int16_t)PID_calc(&speed_pid[i], motor->speed_rpm * RPM_TO_MPS, wheel[i]);
        }
        CAN_cmd_chassis(current[0], current[1], current[2], current[3]);
        osDelay(CHASSIS_PERIOD_MS);
    }
}
