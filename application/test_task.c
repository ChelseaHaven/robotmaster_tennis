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
#include "bsp_usb_debug.h"
#include "bsp_servo_pwm.h"
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
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
#include "calibrate_task.h"
#include "pressure.h"
#include "oid_motor.h"


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
    "PRESSURE_SENSOR1",
    "PRESSURE_SENSOR2",
    "LAUNCHER_MOTOR1",
    "LAUNCHER_MOTOR2",
};


/* 以下打印函数仅在本文件使用，按需在任务循环中启用。 */

/**
  * @brief          按上发顺序打印补偿后的 gyro（rad/s）、acc（m/s²）、mag（μT）
  * @retval         无
  */
static inline void test_print_imu(void)
{
    INS_raw_data_t raw;
    if (!get_INS_raw_data_snapshot(&raw)) return;
    printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
           raw.gyro[0], raw.gyro[1], raw.gyro[2],
           raw.accel[0], raw.accel[1], raw.accel[2],
           raw.mag[0], raw.mag[1], raw.mag[2]);
}

/**
  * @brief          一行打印四轮反馈角速度，与上发方向一致，单位 rad/s
  * @retval         无
  */
static inline void test_print_motor_speed(void)
{
    fp32 speed[CHASSIS_MOTOR_COUNT];
    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; ++i)
    {
        const motor_measure_t *motor = get_chassis_motor_measure_point(i);
        // if (!motor->velocity_range_valid || !motor->feedback_received) return;
        speed[i] = motor->speed_rad_s;
    }
    printf("motor1:%.3f, motor2:%.3f, motor3:%.3f, motor4:%.3f rad/s\r\n",
           speed[0], speed[1], speed[2], speed[3]);
}

/**
  * @brief          一行打印两个 OID 发球电机反馈的电气转速，单位 eRPM
  * @retval         无
  */
static inline void test_print_launcher_speed(void)
{
    oid_motor_measure_t motors[OID_MOTOR_COUNT];
    oid_motor_get_snapshot(motors);
    printf("launcher1:%ld, launcher2:%ld eRPM\r\n",
           (long)motors[0].speed_erpm, (long)motors[1].speed_erpm);
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
           velocity.target.vx, velocity.target.vy, velocity.target.wz);
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
  * @brief          打印两路压力原始读数
  * @retval         无
  */
static inline void test_print_pressure(void)
{
    pressure_data_t pressure[PRESSURE_SENSOR_COUNT];
    pressure_get_snapshot(pressure);
    printf("pressure1:%u, pressure2:%u\r\n",
           (unsigned)pressure[0].value, (unsigned)pressure[1].value);
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
  * @param[in]      num: 每组提示的蜂鸣次数；0 表示关闭并复位提示计数
  * @retval         无
  */
static void buzzer_warn_error(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    static uint8_t tick = 0;
    if (num == 0U) {
        show_num = tick = 0;
        stop_num = 100;
        buzzer_off();
        return;
    }
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

#define TEST_DEBUG_LINE_SIZE 96U

/**
  * @brief          解析以空白分隔的有限浮点参数，并检查参数数量及范围
  * @param[in]      text: 参数字符串
  * @param[out]     values: 可容纳 count 个参数的数组
  * @param[in]      count: 期望参数数量
  * @param[in]      minimum: 参数下限
  * @param[in]      maximum: 参数上限
  * @retval         1 参数合法；0 格式、数量或范围错误
  */
static uint8_t test_debug_parse_values(const char *text, float *values, unsigned count,
                                       float minimum, float maximum)
{
    for (unsigned i = 0; i < count; ++i) {
        while (isspace((unsigned char)*text)) ++text;
        char *end;
        float value = strtof(text, &end);
        if (end == text || !isfinite(value) || value < minimum || value > maximum ||
            (*end && !isspace((unsigned char)*end))) return 0;
        values[i] = value;
        text = end;
    }
    while (isspace((unsigned char)*text)) ++text;
    return *text == '\0';
}

/**
  * @brief          将一行文本解码为调试命令，仅填参数而不执行硬件动作
  * @param[in]      line: 以零结尾的一行文本
  * @param[out]     command: 合法命令及参数
  * @retval         1 解析成功；0 未知命令或参数错误
  */
static uint8_t test_debug_parse_line(const char *line, test_debug_command_t *command)
{
    while (isspace((unsigned char)*line)) ++line;
    const char *arguments = line;
    while (*arguments && !isspace((unsigned char)*arguments)) ++arguments;
    size_t size = (size_t)(arguments - line);
    test_debug_command_t decoded = {0};
    float values[2];
    if (size == 5U && strncmp(line, "pitch", size) == 0) {
        if (!test_debug_parse_values(arguments, values, 1U, 0.0f, SERVO_LAUNCHER_PITCH_MAX_ANGLE_DEG)) return 0;
        decoded.type = TEST_DEBUG_PITCH;
        decoded.angle_deg = values[0];
    } else if (size == 9U && strncmp(line, "separator", size) == 0) {
        if (!test_debug_parse_values(arguments, values, 1U, 0.0f, SERVO_BALL_SEPARATOR_MAX_ANGLE_DEG)) return 0;
        decoded.type = TEST_DEBUG_SEPARATOR;
        decoded.angle_deg = values[0];
    } else if (size == 5U && strncmp(line, "speed", size) == 0) {
        float maximum = (float)OID_MOTOR_MAX_ERPM / OID_MOTOR_POLE_PAIRS;
        if (!test_debug_parse_values(arguments, values, 2U, -maximum, maximum)) return 0;
        decoded.type = TEST_DEBUG_SPEED;
        decoded.motor1_rpm = values[0];
        decoded.motor2_rpm = values[1];
    } else return 0;
    *command = decoded;
    return 1;
}

uint8_t test_get_debug_command(test_debug_command_t *command)
{
    static char line[TEST_DEBUG_LINE_SIZE];
    static unsigned used;
    static uint8_t discard;
    if (!command) return 0;
    for (unsigned i = 0; i < 512U; ++i) {
        uint8_t byte;
        int result = bsp_usb_debug_get_byte(&byte);
        if (result == 0) return 0;
        if (result < 0) {
            used = 0U;
            discard = result == -1;
            continue;
        }
        if (byte == '\r' || byte == '\n') {
            if (discard) { discard = 0U; used = 0U; continue; }
            line[used] = '\0';
            used = 0U;
            if (test_debug_parse_line(line, command)) return 1;
            continue;
        }
        if (discard) continue;
        if (byte == '\b' || byte == 0x7FU) {
            if (used) --used;
        } else if ((byte >= 0x20U && byte <= 0x7EU) || byte == '\t') {
            if (used + 1U < sizeof(line)) line[used++] = (char)byte;
            else { discard = 1U; used = 0U; }
        } else {
            discard = 1U;
            used = 0U;
        }
    }
    return 0;
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
    enum { BUZZER_IDLE, BUZZER_BATTERY, BUZZER_CALIBRATION };
    uint8_t buzzer_mode = BUZZER_IDLE;
    test_debug_command_t debug_command = {0};
    error_list_test_local = get_error_list_point();
    buzzer_play_startup_melody();

    while(1)
    {
        // 统一控制蜂鸣器：确认后的低电量报警优先于标定提示。
        uint8_t next_buzzer_mode = battery_low_alarm_is_active() ? BUZZER_BATTERY :
                                  gyro_calibration_is_busy() ? BUZZER_CALIBRATION : BUZZER_IDLE;
        if (next_buzzer_mode != buzzer_mode) {
            buzzer_warn_error(0);
            buzzer_mode = next_buzzer_mode;
        }
        if (buzzer_mode == BUZZER_BATTERY) {
            buzzer_warn_error(1);
        } else if (buzzer_mode == BUZZER_CALIBRATION) {
            buzzer_on(1000, 30);
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

        if (test_get_debug_command(&debug_command)) {
            switch (debug_command.type) {
            case TEST_DEBUG_PITCH:
                servo_pwm_set_angle(SERVO_LAUNCHER_PITCH, debug_command.angle_deg);
                break;
            case TEST_DEBUG_SEPARATOR:
                servo_pwm_set_angle(SERVO_BALL_SEPARATOR, debug_command.angle_deg);
                break;
            case TEST_DEBUG_SPEED:
                oid_motor_set_speed((int32_t)(debug_command.motor1_rpm * OID_MOTOR_POLE_PAIRS), (int32_t)(debug_command.motor2_rpm * OID_MOTOR_POLE_PAIRS));
                break;
            }
        }

        if (xTaskGetTickCount() - last_print_tick >= pdMS_TO_TICKS(100))
        {
            last_print_tick = xTaskGetTickCount();


            // 按需取消注释，每组打印保持在 100 ms 周期内。
            // test_print_imu();
            // test_print_motor_speed();
            // test_print_launcher_speed();
            // test_print_remote();
            // test_print_host_velocity();
            // test_print_battery();
            // test_print_pressure();
            // test_print_errors();
        }
        osDelay(10);
    }
}
