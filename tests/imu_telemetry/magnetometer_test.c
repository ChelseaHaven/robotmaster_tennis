#include "ist8310driver.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static uint8_t read_ok=1;
uint8_t ist8310_IIC_read_muli_reg_checked(uint8_t reg,uint8_t *buf,uint8_t len)
{
    assert(reg==0x03 && len==6);
    const uint8_t bytes[6]={0x9c,0xff,0xc8,0x00,0xd4,0xfe};
    if (read_ok) memcpy(buf,bytes,6);
    return read_ok;
}
int main(void)
{
    float mag[3]={0};
    assert(ist8310_read_mag_checked(mag));
    assert(fabsf(mag[0]+30)<0.001f && fabsf(mag[1]-60)<0.001f && fabsf(mag[2]+90)<0.001f);
    read_ok=0;
    assert(!ist8310_read_mag_checked(mag));
    assert(fabsf(mag[0]+30)<0.001f && fabsf(mag[1]-60)<0.001f && fabsf(mag[2]+90)<0.001f);
    assert(!ist8310_read_mag_checked(0));
    puts("PASS: magnetometer signed raw conversion to uT and failed-read output preservation.");
}
