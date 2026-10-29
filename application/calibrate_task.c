#include <stdio.h>
#include "calibrate_task.h"
#include "INS_task.h"
#include "bsp_flash.h"
#include "bsp_buzzer.h"
#include "voltage_task.h"
#include "cmsis_os.h"
#include "main.h"

#define GYRO_CALI_FLASH_ADDR ADDR_FLASH_SECTOR_9
#define GYRO_CALI_MAGIC 0x47595231U
#define GYRO_CALI_SAMPLES 20000U

typedef struct
{
    uint32_t magic;
    fp32 scale[3];
    fp32 offset[3];
} gyro_cali_store_t;

static gyro_cali_store_t gyro_cali;
static volatile uint8_t calibration_requested;
static volatile uint8_t calibration_running;
static uint8_t remote_calibration_armed;

int8_t get_control_temperature(void)
{
    return 40;
}

void cali_param_init(void)
{
    flash_read(GYRO_CALI_FLASH_ADDR, (uint32_t *)&gyro_cali, sizeof(gyro_cali) / 4U);
    if (gyro_cali.magic == GYRO_CALI_MAGIC)
        INS_set_cali_gyro(gyro_cali.scale, gyro_cali.offset);
    else
        calibration_requested = 1;
}

void request_gyro_calibration(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!calibration_running && !calibration_requested) {
        calibration_requested = 1;
        remote_calibration_armed = 0;
    }
    __set_PRIMASK(mask);
}

uint8_t gyro_calibration_is_busy(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint8_t busy = calibration_running || calibration_requested;
    __set_PRIMASK(mask);
    return busy;
}

void calibrate_remote_control_update(const RC_ctrl_t *remote)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!remote || !remote->valid || calibration_requested || calibration_running) {
        // 忙碌、失联期间不记录触发；完成后重新见到高值才允许下一次下降沿。
        remote_calibration_armed = 0;
    } else {
        int value = remote->ch[GYRO_CALI_RC_CHANNEL - 1U];
        int high_error = value - (int)GYRO_CALI_RC_HIGH_VALUE;
        int low_error = value - (int)GYRO_CALI_RC_LOW_VALUE;
        if (high_error >= -(int)GYRO_CALI_RC_TOLERANCE && high_error <= (int)GYRO_CALI_RC_TOLERANCE) {
            remote_calibration_armed = 1;
        } else if (low_error >= -(int)GYRO_CALI_RC_TOLERANCE && low_error <= (int)GYRO_CALI_RC_TOLERANCE) {
            if (remote_calibration_armed) calibration_requested = 1;
            remote_calibration_armed = 0;
        } else {
            remote_calibration_armed = 0;
        }
    }
    __set_PRIMASK(mask);
}

void calibrate_task(void const *argument)
{
    (void)argument;
    for (;;)
    {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        uint8_t start = calibration_requested && !calibration_running;
        if (start) {
            calibration_requested = 0;
            calibration_running = 1;
            remote_calibration_armed = 0;
        }
        __set_PRIMASK(mask);
        if (start)
        {
            uint16_t samples = 0;
            gyro_cali.magic = GYRO_CALI_MAGIC;
            do
            {
                INS_cali_gyro(gyro_cali.scale, gyro_cali.offset, &samples);
                if (!(battery_voltage > 1.0f && get_battery_percentage() < 20U)) buzzer_on(1000, 30);
                osDelay(1);
            } while (samples <= GYRO_CALI_SAMPLES);
            if (!(battery_voltage > 1.0f && get_battery_percentage() < 20U)) buzzer_off();
            flash_erase_address(GYRO_CALI_FLASH_ADDR, 1);
            flash_write_single_address(GYRO_CALI_FLASH_ADDR,
                                       (uint32_t *)&gyro_cali, sizeof(gyro_cali) / 4U);
            mask = __get_PRIMASK();
            __disable_irq();
            remote_calibration_armed = 0;
            calibration_running = 0;
            __set_PRIMASK(mask);
        }
        osDelay(10);
    }
}
