#include "bsp_buzzer.h"
#include "bsp_delay.h"
#include "main.h"
extern TIM_HandleTypeDef htim4;
void buzzer_on(uint32_t frequency_hz, uint8_t loudness)
{
    if (frequency_hz == 0U || loudness == 0U)
    {
        buzzer_off();
        return;
    }
    if (loudness > 100U)
    {
        loudness = 100U;
    }

    uint64_t timer_clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
    {
        timer_clock *= 2U;
    }

    // 自动选择分频系数，使周期能放入 16 位定时器。
    uint64_t divider = (timer_clock + (uint64_t)frequency_hz * 65536U - 1U) /
                       ((uint64_t)frequency_hz * 65536U);
    if (divider == 0U) divider = 1U;
    if (divider > 65536U) divider = 65536U;

    uint64_t ticks = (timer_clock + divider * frequency_hz / 2U) /
                     (divider * frequency_hz);
    if (ticks < 2U) ticks = 2U;
    if (ticks > 65536U) ticks = 65536U;

    // 100% 响度对应 50% 占空比，避免输出恒定电平。
    uint32_t pulse = (uint32_t)((ticks * loudness + 100U) / 200U);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0U);
    __HAL_TIM_PRESCALER(&htim4, (uint32_t)(divider - 1U));
    __HAL_TIM_SET_AUTORELOAD(&htim4, (uint32_t)(ticks - 1U));
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    HAL_TIM_GenerateEvent(&htim4, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pulse);
}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

void buzzer_play_startup_melody(void)
{
    static const uint16_t notes[] = {
        BUZZER_L1, BUZZER_L3, BUZZER_L5, BUZZER_M1,
        BUZZER_L5, BUZZER_L3, BUZZER_L1,
    };
    static const uint16_t durations_ms[] = {160, 160, 160, 300, 160, 160, 320};

    for (uint8_t i = 0; i < sizeof(notes) / sizeof(notes[0]); ++i)
    {
        buzzer_on(notes[i], 100);
        delay_ms(durations_ms[i]);
        buzzer_off();
        delay_ms(30);
    }
}
