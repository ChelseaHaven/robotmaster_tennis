#include "host_command.h"
#include <float.h>
#include <math.h>
#include <string.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
               FLT_MAX_EXP == 128, "Host pose requires IEEE 754 binary32");

static float read_float_le(const uint8_t *data)
{
    uint32_t bits = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                    ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

uint8_t host_decode_lidar_pose(const host_packet_t *packet, host_lidar_pose_payload_t *pose)
{
    if (!packet || !pose || packet->command != HOST_CMD_LIDAR_POSE ||
        packet->length != HOST_LIDAR_POSE_PAYLOAD_SIZE)
        return 0;
    host_lidar_pose_payload_t decoded = {
        .x = read_float_le(packet->payload),
        .y = read_float_le(packet->payload + 4),
        .yaw = read_float_le(packet->payload + 8),
    };
    if (!isfinite(decoded.x) || !isfinite(decoded.y) || !isfinite(decoded.yaw))
        return 0;
    *pose = decoded;
    return 1;
}
