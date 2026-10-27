#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>

#define RC_CHANNEL_COUNT 16U
#define RC_FRAME_LENGTH 25U
#define RC_FRAME_TIMEOUT_MS 100U
#define RC_FLAG_DIGITAL_CH17 0x01U
#define RC_FLAG_DIGITAL_CH18 0x02U
#define RC_FLAG_FRAME_LOST 0x04U
#define RC_FLAG_FAILSAFE 0x08U

/* HT-10A 的 SBUS 原始通道，与大疆 DBUS 的居中通道值不同。
 * 摇杆、旋钮、拨杆及按键对应哪个通道，由遥控器设置决定，不在接收层绑定功能。
 */
typedef struct
{
    uint16_t ch[RC_CHANNEL_COUNT]; /* CH1～CH16，11 位原始值 0～2047，不减中值 */
    uint8_t digital[2];            /* CH17、CH18，值为 0 或 1，接收机未提供时无有效控制含义 */
    uint8_t flags;                 /* 原始状态字：数字通道、丢帧、失控保护 */
    uint8_t received;              /* 上电后是否收到过格式正确的完整帧 */
    uint8_t valid;                 /* 无丢帧/失控标志且最近 100 ms 内有数据 */
    uint32_t last_update_ms;       /* 最近完整帧的主控时间，单位 ms */
    uint32_t frame_count;          /* 格式正确的完整帧计数，包含失控保护帧 */
    uint32_t parse_errors;         /* 帧格式错误计数 */
    uint32_t receive_errors;       /* UART/DMA 接收错误计数 */
} RC_ctrl_t;

/* USART3、DMA 初始化后、调度器启动前调用一次。 */
void remote_control_init(void);
/* 在主函数或任务中获取一致快照；即使 valid 为 0，也保留上一帧通道供诊断。 */
void get_remote_control_snapshot(RC_ctrl_t *snapshot);
/* 返回 1 表示未接收、超时、丢帧或失控保护；返回 0 表示可用。 */
uint8_t RC_data_is_error(void);
/* 由现有 detect_task 周期调用，在接收故障后尝试恢复。 */
void remote_control_service(void);

#endif
