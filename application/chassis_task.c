#include <stdio.h>
#include "chassis_task.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "chassis_behaviour.h"
#include "main.h"
#include "cmsis_os.h"
#include <math.h>

#define CHASSIS_PERIOD_MS 2U
#define MAX_WHEEL_SPEED 2.89f

#define WHEEL_DIAMETER_MM 127.0f       // 轮子直径，单位 mm
#define WHEELBASE_MM 320.0f            // 前后轮中心距，单位 mm
#define TRACK_WIDTH_MM 354.6f          // 左右轮中心距，单位 mm

#define WHEEL_RADIUS_M (WHEEL_DIAMETER_MM / 2000.0f)
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
    chassis_behaviour_t behaviour = {0};
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
        chassis_motor_service();
        fp32 vx = control.vx, vy = control.vy, wz = control.wz;
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

        uint8_t enable_mask = 0U;
        uint8_t stop = control.mode == CHASSIS_ESTOP_MODE || control.host_waiting ||
                       (vx == 0.0f && vy == 0.0f && wz == 0.0f &&
                        control.stop_mode == CHASSIS_STOP_DISABLED);
        for (unsigned i = 0; i < CHASSIS_MOTOR_COUNT; ++i)
        {
            wheel[i] = stop ? 0.0f : wheel[i] / WHEEL_RADIUS_M;
            if (!stop && !toe_is_error(CHASSIS_MOTOR1_TOE + i)) enable_mask |= (uint8_t)(1U << i);
        }
        CAN_cmd_chassis(wheel[0], wheel[1], wheel[2], wheel[3], enable_mask);
        osDelay(CHASSIS_PERIOD_MS);
    }
}
