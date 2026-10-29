/**
  **************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      蜂鸣器报警与调试输出任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "test_task.h"
#include <stdio.h>
#include "main.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "host_comm_task.h"
#include "voltage_task.h"

static void buzzer_warn_error(uint8_t num);

const error_t *error_list_test_local;
static const char *const error_name[ERROR_LIST_LENGHT] = {
    "CHASSIS_MOTOR1",
    "CHASSIS_MOTOR2",
    "CHASSIS_MOTOR3",
    "CHASSIS_MOTOR4",
    "GYRO",
    "ACCEL",
    "MAG",
    "REMOTE",
};


/* 以下打印函数仅在本文件使用，按需在任务循环中启用。 */

/**
  * @brief          打印 IMU 偏航、俯仰和横滚角，单位为度
  * @retval         无
  */
static inline void test_print_imu(void)
{
    const fp32 *angle = get_INS_angle_point();
    printf("yaw:%.4f, pitch:%.4f, roll:%.4f\r\n",
           angle[INS_YAW_ADDRESS_OFFSET] * 57.2957795f,
           angle[INS_PITCH_ADDRESS_OFFSET] * 57.2957795f,
           angle[INS_ROLL_ADDRESS_OFFSET] * 57.2957795f);
}

/**
  * @brief          一行打印四轮编码器换算的线速度，单位 m/s
  * @retval         无
  */
static inline void test_print_motor_speed(void)
{
    int16_t speed_rpm[4];
    for (uint8_t i = 0; i < 4; ++i)
    {
        const motor_measure_t *motor = get_chassis_motor_measure_point(i);
        speed_rpm[i] = motor->speed_rpm;
    }
    const float rpm_to_mps = 3.14159265358979323846f * 0.127f / (60.0f * 19.2f);
    printf("motor1:%.3f, motor2:%.3f, motor3:%.3f, motor4:%.3f m/s\r\n",
           speed_rpm[0] * rpm_to_mps,
           speed_rpm[1] * rpm_to_mps,
           speed_rpm[2] * rpm_to_mps,
           speed_rpm[3] * rpm_to_mps);
}

/**
  * @brief          遥控器有效时一行打印全部 SBUS 原始通道
  * @retval         无
  */
static inline void test_print_remote(void)
{
    RC_ctrl_t remote;
    get_remote_control_snapshot(&remote);
    if (!remote.valid) return;
    printf("RC CH1:%u CH2:%u CH3:%u CH4:%u CH5:%u CH6:%u CH7:%u CH8:%u "
           "CH9:%u CH10:%u CH11:%u CH12:%u CH13:%u CH14:%u CH15:%u CH16:%u "
           "CH17:%u CH18:%u\r\n",
           (unsigned)remote.ch[0], (unsigned)remote.ch[1],
           (unsigned)remote.ch[2], (unsigned)remote.ch[3],
           (unsigned)remote.ch[4], (unsigned)remote.ch[5],
           (unsigned)remote.ch[6], (unsigned)remote.ch[7],
           (unsigned)remote.ch[8], (unsigned)remote.ch[9],
           (unsigned)remote.ch[10], (unsigned)remote.ch[11],
           (unsigned)remote.ch[12], (unsigned)remote.ch[13],
           (unsigned)remote.ch[14], (unsigned)remote.ch[15],
           (unsigned)remote.digital[0], (unsigned)remote.digital[1]);
}

/**
  * @brief          打印有效的上位机目标速度，单位 m/s、m/s、rad/s
  * @retval         无
  */
static inline void test_print_host_velocity(void)
{
    host_velocity_t velocity;
    if (!host_comm_get_velocity(&velocity)) return;
    printf("vx:%.3f, vy:%.3f, wz:%.3f\r\n",
           velocity.vx, velocity.vy, velocity.wz);
}

/**
  * @brief          打印电池电压和估算电量百分比
  * @retval         无
  */
static inline void test_print_battery(void)
{
    printf("battery: %.2f V, %u%%\r\n",
           (double)battery_voltage,
           (unsigned)get_battery_percentage());
}

/**
  * @brief          打印当前启用设备的错误，无错误时保持静默
  * @retval         无
  */
static inline void test_print_errors(void)
{
    const error_t *error_list = get_error_list_point();
    for (uint8_t i = 0; i < ERROR_LIST_LENGHT; ++i)
    {
        if (error_list[i].enable && error_list[i].error_exist)
        {
            printf("ERROR: %s (id=%u, lost=%u)\r\n",
                   error_name[i], (unsigned)i, (unsigned)error_list[i].is_lost);
        }
    }
}

/**
  * @brief          按错误编号输出对应次数的周期蜂鸣提示
  * @param[in]      num: 每组提示的蜂鸣次数
  * @retval         无
  */
static void buzzer_warn_error(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    if(show_num == 0 && stop_num == 0)
    {
        show_num = num;
        stop_num = 100;
    }
    else if(show_num == 0)
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if(tick < 50)
        {
            buzzer_off();
        }
        else if(tick < 100)
        {
            buzzer_on(BUZZER_L3, 100);
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}

/**
  * @brief          播放开机旋律，周期检查报警并执行选中的串口打印
  * @param[in]      argument: 任务参数，未使用
  * @retval         无
  */
void test_task(void const * argument)
{
    static uint8_t error;
    static uint8_t error_num;
    uint32_t last_print_tick = 0;
    uint8_t battery_alarm_active = 0;
    error_list_test_local = get_error_list_point();
    buzzer_play_startup_melody();

    while(1)
    {
        // 大疆 voltage_task 的估算电量低于 20% 时，沿用已有蜂鸣报警。
        if (battery_voltage > 1.0f && get_battery_percentage() < 20U) {
            buzzer_warn_error(1);
            battery_alarm_active = 1;
        } else if (battery_alarm_active) {
            buzzer_off();
            battery_alarm_active = 0;
        }
        error = 0;

        for(error_num = 0; error_num < ERROR_LIST_LENGHT; error_num++)
        {
            if(error_list_test_local[error_num].enable && error_list_test_local[error_num].error_exist)
            {
                error = 1;
                break;
            }
        }

        if(error)
        {
            // buzzer_warn_error(error_num+1);
        }

        if (xTaskGetTickCount() - last_print_tick >= pdMS_TO_TICKS(100))
        {
            last_print_tick = xTaskGetTickCount();


            // 按需取消注释，每组打印保持在 100 ms 周期内。
            // test_print_imu();
            test_print_motor_speed();
            // test_print_remote();
            // test_print_host_velocity();
            // test_print_battery();
            // test_print_errors();
        }
        osDelay(10);
    }
}
