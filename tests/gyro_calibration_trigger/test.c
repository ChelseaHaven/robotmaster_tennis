#include "calibrate_task.h"
#include "bsp_flash.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
static RC_ctrl_t remote={.valid=1};
static unsigned runs, sample_calls;
static jmp_buf finished;
float battery_voltage=24.0f;
uint16_t get_battery_percentage(void) { return 70; }
void buzzer_on(uint32_t frequency,uint8_t loudness) { (void)frequency;(void)loudness; }
void buzzer_off(void) {}
static void channel(uint16_t value)
{
    remote.ch[6]=value;calibrate_remote_control_update(&remote);
}
void flash_read(uint32_t address,uint32_t *data,uint32_t count)
{
    assert(address==ADDR_FLASH_SECTOR_9);memset(data,0,count*4U);data[0]=0x47595231U;
}
void INS_set_cali_gyro(float scale[3],float offset[3]) { (void)scale;(void)offset; }
void INS_cali_gyro(float scale[3],float offset[3],uint16_t *count)
{
    (void)scale;(void)offset;
    assert(gyro_calibration_is_busy());
    ++sample_calls;++*count;
    // 标定期间反复上升/下降及直接请求，均不能排队。
    channel(1792);channel(192);request_gyro_calibration();
}
void flash_erase_address(uint32_t address,uint16_t length)
{
    assert(address==ADDR_FLASH_SECTOR_9 && length==1);assert(gyro_calibration_is_busy());
}
int8_t flash_write_single_address(uint32_t address,uint32_t *data,uint32_t count)
{
    (void)data;(void)count;assert(address==ADDR_FLASH_SECTOR_9);
    assert(gyro_calibration_is_busy());
    channel(1792);channel(192);request_gyro_calibration();++runs;return 0;
}
void osDelay(uint32_t milliseconds)
{
    if(milliseconds==1) { assert(gyro_calibration_is_busy());return; }
    assert(milliseconds==10 && !gyro_calibration_is_busy());
    assert(sample_calls==runs*20001U);
    for(unsigned i=0;i<100;i++) channel(192);
    assert(!gyro_calibration_is_busy());
    if(runs==2) longjmp(finished,1);
    assert(runs==1);
    // 标定结束后明确重新拨高、再拨低，才能开始下一次。
    channel(1792);assert(!gyro_calibration_is_busy());
    channel(192);assert(gyro_calibration_is_busy());
}
int main(void)
{
    cali_param_init();assert(!gyro_calibration_is_busy());
    channel(192);channel(192);assert(!gyro_calibration_is_busy());
    channel(1792);remote.valid=0;channel(192);remote.valid=1;channel(192);
    assert(!gyro_calibration_is_busy());
    channel(1792);channel(992);channel(192);assert(!gyro_calibration_is_busy());
    channel(1792);channel(192);assert(gyro_calibration_is_busy());
    channel(1792);channel(192);request_gyro_calibration();
    if(!setjmp(finished)) calibrate_task(NULL);
    assert(runs==2 && sample_calls==40002);
    puts("PASS: CH7 falling-edge single-shot, startup-low/reconnect rejection, busy toggles ignored, no deferred repeat and explicit rearm after completion.");
}
