#include <stdio.h>
#include "chassis_task.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "pid.h"
#include "chassis_behaviour.h"
#include "main.h"
#include "cmsis_os.h"
#include <math.h>

#define CHASSIS_PERIOD_MS 2U
#define MAX_WHEEL_SPEED 4.0f
#define MOTOR_CURRENT_LIMIT (16384.0f * 15.0f / 20.0f) // C620：15 A 对应指令 12288

#define WHEEL_DIAMETER_MM 127.0f       // 轮子直径，单位 mm
#define MOTOR_GEAR_RATIO 19.2f         // 电机转速与轮子转速之比
#define WHEELBASE_MM 320.0f            // 前后轮中心距，单位 mm
#define TRACK_WIDTH_MM 354.6f          // 左右轮中心距，单位 mm

#define RPM_TO_MPS (3.14159265358979323846f * (WHEEL_DIAMETER_MM / 1000.0f) / (60.0f * MOTOR_GEAR_RATIO))
#define MOTOR_DISTANCE_TO_CENTER ((WHEELBASE_MM + TRACK_WIDTH_MM) / (2.0f * 1000.0f))

static chassis_control_t control_state;

void chassis_get_control(chassis_control_t *control)
{
    if (!control) return;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *control = control_state;
    __set_PRIMASK(mask);
}

void chassis_task(void const *argument)
{
    (void)argument;
    static const fp32 gains[3] = {15000.0f, 10.0f, 0.0f};
    pid_type_def speed_pid[4];
    for (unsigned i = 0; i < 4; ++i)
        PID_init(&speed_pid[i], PID_POSITION, gains, MOTOR_CURRENT_LIMIT, 2000.0f);

    chassis_behaviour_t behaviour = {0};
    chassis_mode_t previous_mode = CHASSIS_ESTOP_MODE;
    for (;;)
    {
        RC_ctrl_t remote;
        host_velocity_t host;
        get_remote_control_snapshot(&remote);
        host_comm_get_velocity(&host);
        chassis_control_t control;
        chassis_behaviour_update(&behaviour, &remote, &host, &control);
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        control_state = control;
        __set_PRIMASK(mask);
        if (control.mode != previous_mode) {
            for (unsigned i = 0; i < 4; ++i) PID_clear(&speed_pid[i]);
            previous_mode = control.mode;
        }
        fp32 vx = control.vx, vy = control.vy, wz = control.wz;
        if (vx == 0.0f && vy == 0.0f && wz == 0.0f)
        {
            for (unsigned i = 0; i < 4; ++i) PID_clear(&speed_pid[i]);
            CAN_cmd_chassis(0, 0, 0, 0);
            osDelay(CHASSIS_PERIOD_MS);
            continue;
        }
        fp32 wheel[4] = {
            -vx - vy - 1.0f * MOTOR_DISTANCE_TO_CENTER * wz,
             vx - vy - 1.0f * MOTOR_DISTANCE_TO_CENTER * wz,
             vx + vy - 1.0f * MOTOR_DISTANCE_TO_CENTER * wz,
            -vx + vy - 1.0f * MOTOR_DISTANCE_TO_CENTER * wz,
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
            current[i] = (int16_t)PID_calc(&speed_pid[i], motor->raw_speed_rpm * RPM_TO_MPS, wheel[i]);
        }
        // printf("curent: %d, %d, %d, %d\r\n", (int)current[0], (int)current[1], (int)current[2], (int)current[3]);
        CAN_cmd_chassis(current[0], current[1], current[2], current[3]);
        osDelay(CHASSIS_PERIOD_MS);
    }
}
