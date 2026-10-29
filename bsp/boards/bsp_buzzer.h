#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "struct_typedef.h"

// 与 bicycle 工程相同的低、中、高音阶；数值为频率，单位 Hz。
#define BUZZER_L1 262U  // 低音 do
#define BUZZER_L2 296U  // 低音 re
#define BUZZER_L3 330U  // 低音 mi
#define BUZZER_L4 349U  // 低音 fa
#define BUZZER_L5 392U  // 低音 sol
#define BUZZER_L6 440U  // 低音 la
#define BUZZER_L7 494U  // 低音 si

#define BUZZER_M1 523U  // 中音 do
#define BUZZER_M2 587U  // 中音 re
#define BUZZER_M3 659U  // 中音 mi
#define BUZZER_M4 699U  // 中音 fa
#define BUZZER_M5 784U  // 中音 sol
#define BUZZER_M6 880U  // 中音 la
#define BUZZER_M7 988U  // 中音 si

#define BUZZER_H1 1048U // 高音 do
#define BUZZER_H2 1176U // 高音 re
#define BUZZER_H3 1320U // 高音 mi
#define BUZZER_H4 1480U // 高音 fa
#define BUZZER_H5 1640U // 高音 sol
#define BUZZER_H6 1760U // 高音 la
#define BUZZER_H7 1976U // 高音 si

/**
  * @brief          设置蜂鸣器频率和响度并启动 PWM
  * @param[in]      frequency_hz: 频率，单位 Hz；0 表示关闭
  * @param[in]      loudness: 响度范围 0～100；0 表示关闭
  * @note           响度 100 对应 50% 占空比，响度参数与实际声压并非线性关系。
  * @retval         无
  */
extern void buzzer_on(uint32_t frequency_hz, uint8_t loudness);

/**
  * @brief          关闭蜂鸣器 PWM 输出
  * @retval         无
  */
extern void buzzer_off(void);

/**
  * @brief          播放开机旋律，完成后关闭蜂鸣器
  * @note           包含阻塞延时，应在任务中调用。
  * @retval         无
  */
extern void buzzer_play_startup_melody(void);

#endif
