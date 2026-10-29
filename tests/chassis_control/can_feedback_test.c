#include "main.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
CAN_HandleTypeDef hcan1;
static CAN_RxHeaderTypeDef incoming;
static uint8_t rx_data[8], tx_data[8];
static unsigned hook_count;
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, unsigned fifo,
                        CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)hcan;(void)fifo;*header=incoming;memcpy(data,rx_data,8);return HAL_OK;
}
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *header,
                        uint8_t *data, uint32_t *mailbox)
{
    (void)hcan;assert(header->StdId==0x200);memcpy(tx_data,data,8);*mailbox=0;return HAL_OK;
}
void detect_hook(uint8_t device) { assert(device<4);++hook_count; }
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
static void receive(unsigned index,int16_t rpm)
{
    incoming=(CAN_RxHeaderTypeDef){.StdId=0x201+index,.DLC=8,.IDE=CAN_ID_STD};
    uint16_t raw=(uint16_t)rpm;
    rx_data[0]=1;rx_data[1]=244;rx_data[2]=raw>>8;rx_data[3]=raw;
    rx_data[4]=0xff;rx_data[5]=0x9c;rx_data[6]=55;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
}
int main(void)
{
    int16_t raw[4]={-500,600,700,-800};
    for(unsigned i=0;i<4;i++) {
        receive(i,raw[i]);const motor_measure_t *m=get_chassis_motor_measure_point(i);
        assert(m->raw_speed_rpm==raw[i]);
        assert(m->speed_rpm==(i==0||i==3 ? -raw[i]:raw[i]));
        assert(m->ecd==500 && m->given_current==-100 && m->temperate==55);
        receive(i,-raw[i]);m=get_chassis_motor_measure_point(i);
        assert(m->raw_speed_rpm==-raw[i]);
        assert(m->speed_rpm==(i==0||i==3 ? raw[i]:-raw[i]));
    }
    receive(0,INT16_MIN);assert(get_chassis_motor_measure_point(0)->speed_rpm==INT16_MAX);
    assert(get_chassis_motor_measure_point(0)->raw_speed_rpm==INT16_MIN);
    assert(hook_count==9);
    CAN_cmd_chassis(100,-200,300,-400);
    const uint8_t expected[8]={0,100,0xff,0x38,1,0x2c,0xfe,0x70};
    assert(!memcmp(tx_data,expected,8));
    puts("PASS: CAN RX direction normalization for wheels 1/4, raw PID feedback preservation and unchanged TX current signs.");
}
