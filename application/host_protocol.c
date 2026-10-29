#include "host_protocol.h"
#include "CRC8_CRC16.h"
#include <string.h>

uint16_t host_protocol_pack(uint8_t *out, uint16_t command, uint8_t sequence,
                             const void *payload, uint16_t length)
{
    if (!out || length > HOST_MAX_PAYLOAD || (length && !payload)) return 0;
    out[0] = 0xa5;
    out[1] = (uint8_t)length;
    out[2] = (uint8_t)(length >> 8);
    out[3] = sequence;
    append_CRC8_check_sum(out, 5);
    out[5] = (uint8_t)command;
    out[6] = (uint8_t)(command >> 8);
    if (length) memcpy(out+7, payload, length);
    append_CRC16_check_sum(out, length + HOST_FRAME_OVERHEAD);
    return length + HOST_FRAME_OVERHEAD;
}

/**
  * @brief          丢弃解析缓冲区前部字节并前移剩余数据
  * @param[in,out]  p: 解析器状态
  * @param[in]      count: 丢弃字节数，不超过当前已缓存长度
  * @retval         无
  */
static void discard(host_parser_t *p, uint16_t count)
{
    p->used -= count;
    memmove(p->buffer, p->buffer+count, p->used);
}
uint8_t host_protocol_feed(host_parser_t *p, uint8_t byte, uint32_t now_ms,
                            host_packet_t *packet)
{
    if (!p || !packet) return 0;
    if (p->used && (uint32_t)(now_ms-p->last_byte_ms) >= HOST_FRAME_TIMEOUT_MS)
        p->used = 0;
    p->last_byte_ms = now_ms;
    if (p->used >= HOST_MAX_FRAME) discard(p, 1);
    p->buffer[p->used++] = byte;
    while (p->used) {
        if (p->buffer[0] != 0xa5) { discard(p, 1); continue; }
        if (p->used < 5) return 0;
        if (!verify_CRC8_check_sum(p->buffer, 5)) {
            ++p->crc_errors; discard(p, 1); continue;
        }
        uint16_t length = p->buffer[1] | ((uint16_t)p->buffer[2] << 8);
        if (length > HOST_MAX_PAYLOAD) {
            ++p->length_errors; discard(p, 1); continue;
        }
        uint16_t total = length + HOST_FRAME_OVERHEAD;
        if (p->used < total) return 0;
        if (!verify_CRC16_check_sum(p->buffer, total)) {
            ++p->crc_errors; discard(p, 1); continue;
        }
        packet->command = p->buffer[5] | ((uint16_t)p->buffer[6] << 8);
        packet->sequence = p->buffer[3];
        packet->length = length;
        memcpy(packet->payload, p->buffer+7, length);
        discard(p, total);
        ++p->frames;
        return 1;
    }
    return 0;
}
