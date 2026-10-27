#include <stdio.h>
#include "calibrate_task.h"
#include "INS_task.h"
#include "bsp_flash.h"
#include "bsp_buzzer.h"
#include "cmsis_os.h"

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
    calibration_requested = 1;
}

void calibrate_task(void const *argument)
{
    (void)argument;
    for (;;)
    {
        if (calibration_requested)
        {
            uint16_t samples = 0;
            calibration_requested = 0;
            gyro_cali.magic = GYRO_CALI_MAGIC;
            do
            {
                INS_cali_gyro(gyro_cali.scale, gyro_cali.offset, &samples);
                buzzer_on(1000, 30);
                osDelay(1);
            } while (samples <= GYRO_CALI_SAMPLES);
            buzzer_off();
            flash_erase_address(GYRO_CALI_FLASH_ADDR, 1);
            flash_write_single_address(GYRO_CALI_FLASH_ADDR,
                                       (uint32_t *)&gyro_cali, sizeof(gyro_cali) / 4U);
        }
        osDelay(10);
    }
}
