#include "chassis_task.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
static RC_ctrl_t remote;
static host_velocity_t host;
static motor_measure_t motors[4];
static int16_t currents[4];
static jmp_buf done;
static unsigned step;
void get_remote_control_snapshot(RC_ctrl_t *out) { *out=remote; }
uint8_t host_comm_get_velocity(host_velocity_t *out) { *out=host;return out->valid; }
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i) { return &motors[i]; }
bool_t toe_is_error(uint8_t device) { (void)device;return 0; }
void CAN_cmd_chassis(int16_t a,int16_t b,int16_t c,int16_t d)
{
    currents[0]=a;currents[1]=b;currents[2]=c;currents[3]=d;
    for(unsigned i=0;i<4;i++) assert(abs(currents[i])<=12288);
}
static void stopped(void) { for(unsigned i=0;i<4;i++) assert(!currents[i]); }
void osDelay(uint32_t period)
{
    assert(period==2);
    chassis_control_t state;
    chassis_get_control(&state);
    switch(step++) {
    case 0:
        assert(state.mode==CHASSIS_ESTOP_MODE);stopped();remote.ch[7]=992;break;
    case 1:
        assert(state.mode==CHASSIS_REMOTE_MODE);stopped();remote.ch[2]=1792;
        motors[1].speed_rpm=-32768;break;
    case 2:
        assert(currents[0]<0 && currents[1]>0 && currents[2]>0 && currents[3]<0);
        assert(currents[1]==12288);motors[1].speed_rpm=0;
        remote.ch[2]=992;remote.ch[3]=1792;break;
    case 3:
        assert(currents[0]>0 && currents[1]>0 && currents[2]<0 && currents[3]<0);
        remote.ch[3]=992;remote.ch[0]=192;break;
    case 4:
        for(unsigned i=0;i<4;i++) assert(currents[i]<0);
        remote.ch[7]=192;break;
    case 5:
        assert(state.mode==CHASSIS_HOST_MODE && state.host_waiting);stopped();host.frame_count++;break;
    case 6:
        assert(!state.host_waiting && state.vx==.3f);
        assert(currents[0]<0 && currents[1]>0);host.valid=0;break;
    case 7:
        assert(state.host_waiting);stopped();host.valid=1;remote.valid=0;break;
    case 8:
        assert(state.mode==CHASSIS_ESTOP_MODE);stopped();remote.valid=1;host.frame_count++;break;
    case 9:
        assert(state.host_waiting);stopped();host.frame_count++;break;
    case 10:
        assert(!state.host_waiting && state.vx==.3f);remote.ch[7]=1792;break;
    case 11:
        assert(state.mode==CHASSIS_ESTOP_MODE);stopped();longjmp(done,1);
    default:assert(0);
    }
}
int main(void)
{
    remote.valid=1;
    for(unsigned i=0;i<16;i++) remote.ch[i]=992;
    remote.ch[7]=1792;
    host=(host_velocity_t){.valid=1,.frame_count=7,.vx=.3f};
    if (!setjmp(done)) chassis_task(NULL);
    puts("PASS: actual chassis task RC/host/stop currents, 15 A clamp, stale-command gate and disconnect stop.");
}
