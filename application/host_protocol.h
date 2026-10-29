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

/**
  * @brief          封装含帧头 CRC8 和整帧 CRC16 的通信帧
  * @param[out]     out: 输出缓冲区，容量至少为 length + HOST_FRAME_OVERHEAD
  * @param[in]      command: 命令号
  * @param[in]      sequence: 帧序号
  * @param[in]      payload: 载荷，长度为 0 时允许为 NULL
  * @param[in]      length: 载荷字节数
  * @retval         完整帧字节数；参数错误时返回 0
  */
uint16_t host_protocol_pack(uint8_t *out, uint16_t command, uint8_t sequence,
                             const void *payload, uint16_t length);

/**
  * @brief          逐字节接收并校验协议帧，自动处理错帧和半帧超时
  * @param[in,out]  parser: 解析器状态，首次使用前清零
  * @param[in]      byte: 新收到的字节
  * @param[in]      now_ms: 当前时间，单位 ms
  * @param[out]     packet: 完整且校验通过的数据包
  * @retval         1：输出一帧；0：尚无完整有效帧
  */
uint8_t host_protocol_feed(host_parser_t *parser, uint8_t byte, uint32_t now_ms,
                            host_packet_t *packet);
#endif
