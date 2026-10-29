#include "host_comm_task.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include "CRC8_CRC16.h"
#include "queue.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static host_packet_t sent[16];
static unsigned count;
static uint8_t raw_ready=1;
static motor_measure_t motors[4];
QueueHandle_t xQueueCreate(unsigned depth,size_t size)
{
    assert(depth==8 && size==sizeof(host_packet_t));return (void *)1;
}
int xQueueSend(QueueHandle_t queue,const void *item,unsigned wait)
{
    (void)queue;assert(!wait);assert(count<16);sent[count++]=*(const host_packet_t *)item;return 1;
}
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i) { return &motors[i]; }
uint8_t get_INS_raw_data_snapshot(INS_raw_data_t *data)
{
    *data=(INS_raw_data_t){.gyro={1,2,3},.accel={4,5,6},.mag={7,8,9},.ready=raw_ready};
    return raw_ready;
}
static void check_imu(const host_packet_t *packet)
{
    assert(packet->command==0x0102 && packet->length==36);
    float fields[9];memcpy(fields,packet->payload,sizeof fields);
    for(unsigned i=0;i<9;i++) assert(fields[i]==(float)(i+1));
    uint8_t wire[HOST_MAX_FRAME];
    uint16_t length=host_protocol_pack(wire,packet->command,19,packet->payload,packet->length);
    assert(length==45 && verify_CRC8_check_sum(wire,5) && verify_CRC16_check_sum(wire,length));
    host_parser_t parser={0};host_packet_t decoded;unsigned frames=0;
    for(unsigned i=0;i<length;i++) frames+=host_protocol_feed(&parser,wire[i],10,&decoded);
    assert(frames==1 && decoded.command==0x0102 && decoded.length==36);
    assert(!memcmp(decoded.payload,packet->payload,36));
}
int main(void)
{
    assert(host_comm_init());
    host_comm_send_periodic(0);host_comm_send_periodic(9);assert(!count);
    host_comm_send_periodic(10);assert(count==2);
    assert(sent[0].command==0x0101 && sent[0].length==16);check_imu(&sent[1]);
    host_comm_send_periodic(11);assert(count==2);
    host_comm_send_periodic(20);assert(count==4);check_imu(&sent[3]);
    raw_ready=0;host_comm_send_periodic(30);assert(count==5 && sent[4].length==16);
    raw_ready=1;host_comm_send_periodic(31);assert(count==6);check_imu(&sent[5]);
    puts("PASS: 10 ms wheel/IMU schedules, raw-ready gate, nine-axis order, 36-byte little-endian payload and CRC frame round-trip.");
}
