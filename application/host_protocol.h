#ifndef HOST_PROTOCOL_H
#define HOST_PROTOCOL_H
#include <stdint.h>
#define HOST_MAX_PAYLOAD 128U
#define HOST_FRAME_OVERHEAD 9U
#define HOST_MAX_FRAME (HOST_MAX_PAYLOAD + HOST_FRAME_OVERHEAD)
#define HOST_FRAME_TIMEOUT_MS 100U

typedef struct {
    uint16_t command;
    uint16_t length;
    uint8_t sequence;
    uint8_t payload[HOST_MAX_PAYLOAD];
} host_packet_t;

typedef struct {
    uint8_t buffer[HOST_MAX_FRAME];
    uint16_t used;
    uint32_t last_byte_ms;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t frames;
} host_parser_t;

/* 纯协议函数，不依赖 HAL 和操作系统；多字节字段均为小端。 */
uint16_t host_protocol_pack(uint8_t *out, uint16_t command, uint8_t sequence,
                             const void *payload, uint16_t length);
/* 每次输入一个字节，成功解出一帧时返回 1。 */
uint8_t host_protocol_feed(host_parser_t *parser, uint8_t byte, uint32_t now_ms,
                            host_packet_t *packet);
#endif
