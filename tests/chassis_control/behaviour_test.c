#include "chassis_behaviour.h"
#include "host_command.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static void near(float a,float b) { assert(fabsf(a-b)<0.0001f); }
int main(void) {
    chassis_behaviour_t b={0};chassis_control_t out;
    RC_ctrl_t rc={.valid=1};host_velocity_t host={.valid=1,.frame_count=9,.vx=.2f,.vy=.1f,.wz=.3f};
    for (unsigned i=0;i<16;i++) rc.ch[i]=992;
#if CHASSIS_RC_MAPPING_CONFIRMED
    rc.ch[7]=1792;chassis_behaviour_update(&b,&rc,&host,&out);
    assert(out.mode==CHASSIS_ESTOP_MODE);near(out.vx,0);
    rc.ch[7]=992;chassis_behaviour_update(&b,&rc,&host,&out);
    assert(out.mode==CHASSIS_REMOTE_MODE);near(out.vx,0);near(out.vy,0);near(out.wz,0);
    rc.ch[2]=1792;chassis_behaviour_update(&b,&rc,&host,&out);near(out.vx,.5f);
    rc.ch[2]=992;rc.ch[3]=1792;chassis_behaviour_update(&b,&rc,&host,&out);near(out.vy,-.5f);
    rc.ch[3]=992;rc.ch[0]=192;chassis_behaviour_update(&b,&rc,&host,&out);near(out.wz,1);
    rc.ch[0]=1792;chassis_behaviour_update(&b,&rc,&host,&out);near(out.wz,-1);
    rc.ch[0]=992;rc.ch[1]=192;chassis_behaviour_update(&b,&rc,&host,&out);near(out.vx,0);near(out.wz,0);
    rc.ch[7]=192;chassis_behaviour_update(&b,&rc,&host,&out);
    assert(out.mode==CHASSIS_HOST_MODE && out.host_waiting);near(out.vx,0);
    host.frame_count++;chassis_behaviour_update(&b,&rc,&host,&out);near(out.vx,.2f);assert(!out.host_waiting);
    host.valid=0;chassis_behaviour_update(&b,&rc,&host,&out);near(out.vx,0);assert(out.host_waiting);
    host.valid=1;rc.valid=0;chassis_behaviour_update(&b,&rc,&host,&out);assert(out.mode==CHASSIS_ESTOP_MODE);near(out.vx,0);
    rc.valid=1;rc.ch[7]=992;rc.ch[3]=1792;rc.ch[2]=1792;
    chassis_behaviour_update(&b,&rc,&host,&out);near(hypotf(out.vx,out.vy),.5f);
    rc.ch[7]=500;chassis_behaviour_update(&b,&rc,&host,&out);assert(out.mode==CHASSIS_ESTOP_MODE);
#else
    for(unsigned v=0;v<=2047;v++) {
        rc.ch[7]=v;chassis_behaviour_update(&b,&rc,&host,&out);
        assert(out.mode==CHASSIS_ESTOP_MODE);near(out.vx,0);near(out.vy,0);near(out.wz,0);
    }
#endif
    host_packet_t packet={.command=HOST_CMD_CHASSIS_VELOCITY,.length=12};
    uint8_t data[12]={0,0,0,0x3e,0,0,0x80,0xbe,0,0,0x80,0x3f};
    memcpy(packet.payload,data,12);host_velocity_payload_t speed;
    assert(host_decode_velocity(&packet,&speed));near(speed.vx,.125f);near(speed.vy,-.25f);near(speed.wz,1);
    packet.command=0x0103;assert(!host_decode_velocity(&packet,&speed));
    packet.command=HOST_CMD_CHASSIS_VELOCITY;packet.length=11;assert(!host_decode_velocity(&packet,&speed));
    packet.length=12;packet.payload[2]=0xc0;packet.payload[3]=0x7f;assert(!host_decode_velocity(&packet,&speed));
    puts("PASS: mode arbitration, RC axes/deadband, unused stick, stale host gate, disconnect, limits, unknown-command rejection and velocity decoding.");
}
