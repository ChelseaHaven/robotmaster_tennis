#include "host_command.h"
#include <float.h>
#include <math.h>
#include <string.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
               FLT_MAX_EXP == 128, "Host velocity requires IEEE 754 binary32");

/**
  * @brief          将四字节小端数据还原为单精度浮点数
  * @param[in]      data: 至少 4 字节的小端数据
  * @retval         还原后的浮点数
  */
static float read_float_le(const uint8_t *data)
{
    uint32_t bits = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                    ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

uint8_t host_decode_velocity(const host_packet_t *packet, host_velocity_payload_t *velocity)
{
    if (!packet || !velocity || packet->command != HOST_CMD_CHASSIS_VELOCITY ||
        packet->length != HOST_VELOCITY_PAYLOAD_SIZE)
        return 0;
    host_velocity_payload_t decoded = {
        .vx = read_float_le(packet->payload),
        .vy = read_float_le(packet->payload + 4),
        .wz = read_float_le(packet->payload + 8),
    };
    if (!isfinite(decoded.vx) || !isfinite(decoded.vy) || !isfinite(decoded.wz))
        return 0;
    *velocity = decoded;
    return 1;
}
