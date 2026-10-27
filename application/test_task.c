/**
  **************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.蜂鸣器报警任务
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
};




/**
  * @brief          test任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void test_task(void const * argument)
{
    static uint8_t error;
    static uint8_t error_num;
    uint32_t last_print_tick = 0;
    error_list_test_local = get_error_list_point();
    buzzer_play_startup_melody();

    while(1)
    {
        error = 0;

        //发现错误
        for(error_num = 0; error_num < ERROR_LIST_LENGHT; error_num++)
        {
            if(error_list_test_local[error_num].enable && error_list_test_local[error_num].error_exist)
            {
                error = 1;
                break;
            }
        }

        //有错误
        if(error)
        {
            // buzzer_warn_error(error_num+1);
        }
        
        if (xTaskGetTickCount() - last_print_tick >= pdMS_TO_TICKS(100))
        {
            last_print_tick = xTaskGetTickCount();
            // const fp32 *angle = get_INS_angle_point();
            // printf("yaw:%.4f, pitch:%.4f, roll:%.4f\r\n",
            //        angle[INS_YAW_ADDRESS_OFFSET] * 57.2957795f,
            //        angle[INS_PITCH_ADDRESS_OFFSET] * 57.2957795f,
            //        angle[INS_ROLL_ADDRESS_OFFSET] * 57.2957795f);

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

            uint8_t has_error = 0;
            for (uint8_t i = 0; i < ERROR_LIST_LENGHT; i++)
            {
                if (error_list_test_local[i].enable && error_list_test_local[i].error_exist)
                {
                    printf("ERROR: %s (id=%u, lost=%u)\r\n",
                           error_name[i], (unsigned)i,
                           (unsigned)error_list_test_local[i].is_lost);
                    has_error = 1;
                }
            }
            if (!has_error)
            {
                // printf("ERROR: none\r\n");
            }
        }
        osDelay(10);
    }
}



/**
  * @brief          使得蜂鸣器响
  * @param[in]      num:响声次数
  * @retval         none
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


	
