#include "bsp_delay.h"
#include "main.h"

static uint32_t cycles_per_us;

void delay_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cycles_per_us = SystemCoreClock / 1000000U;
}

void delay_us(uint16_t us)
{
    if (cycles_per_us == 0U)
    {
        delay_init();
    }
    const uint32_t start = DWT->CYCCNT;
    const uint32_t cycles = (uint32_t)us * cycles_per_us;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
    }
}

void delay_ms(uint16_t ms)
{
    while (ms-- != 0U)
    {
        delay_us(1000U);
    }
}
