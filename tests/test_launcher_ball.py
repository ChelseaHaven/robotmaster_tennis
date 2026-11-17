"""从实际 launcher_task.c 提取纯状态机，在主机验证动作及故障路径。
运行：python3 tests/test_launcher_ball.py
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'application/launcher_task.c').read_text()
helpers = source[source.index('typedef enum {'):source.index('void launcher_task(')]
prelude = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "launcher_task.h"
#include "pressure.h"
#define PRESSURE_SENSOR1_TOE 0
#define PRESSURE_SENSOR2_TOE 1
#define SERVO_BALL_SEPARATOR 0
static uint8_t offline[2];
static float last_angle;
static uint8_t toe_is_error(unsigned index) { return offline[index]; }
static void servo_pwm_set_angle(unsigned servo, float angle) {
    (void)servo;
    assert(angle >= 0 && angle <= 270);
    last_angle = angle;
}
'''
cases = r'''
static void sample(pressure_data_t *p, uint16_t left, uint16_t right) {
    p[LAUNCHER_BALL_LEFT_SENSOR].value = left;
    p[LAUNCHER_BALL_RIGHT_SENSOR].value = right;
    for (unsigned i = 0; i < 2; ++i) ++p[i].sample_count;
}
static launcher_ball_t idle(unsigned slot) {
    launcher_ball_t b = {.slot = slot, .angle_deg = ball_angles[slot], .search_direction = 1};
    return b;
}
int main(void) {
    pressure_data_t p[2] = {0};
    launcher_ball_t b = idle(1);
    launcher_ball_request(&b, 0, p, 1);
    sample(p, 50, 0);
    launcher_ball_update(&b, 100, p, 1, 1);
    assert(b.motion == BALL_MOTION_NONE);
    for (unsigned i = 0; i < 20; ++i) launcher_ball_update(&b, 101+i, p, 1, 1);
    assert(b.motion == BALL_MOTION_NONE);  /* 重复快照不能充当新采样。 */
    sample(p, 50, 0);
    launcher_ball_update(&b, 200, p, 1, 1);
    assert(b.motion == BALL_MOTION_FEED && b.target_slot == 0);
    launcher_ball_request(&b, 201, p, 1);
    assert(b.prepare_start_ms == 0);      /* 忙时不保留新请求。 */
    launcher_ball_update(&b, 450, p, 1, 1);
    assert(last_angle == 45.0f);
    launcher_ball_update(&b, 700, p, 1, 1);
    assert(b.state == BALL_SETTLING && b.completed_count == 0);
    launcher_ball_update(&b, 1700, p, 1, 1);
    assert(b.completed_count == 1 && !b.request_pending && b.prepare_pending);
    launcher_ball_update(&b, 1710, p, 1, 1);
    assert(b.motion == BALL_MOTION_NONE); /* 动作前采样不能启动下一次移动。 */

    /* 两边有球时在中间工位之间来回，自动准备不等于再次发球。 */
    p[0].sample_count = p[1].sample_count = 0;
    b = idle(1);
    launcher_ball_request(&b, 0, p, 1);
    sample(p, 50, 50); launcher_ball_update(&b, 100, p, 1, 1);
    sample(p, 50, 50); launcher_ball_update(&b, 200, p, 1, 1);
    assert(b.motion == BALL_MOTION_FEED && b.target_slot == 2);
    launcher_ball_update(&b, 700, p, 1, 1);
    launcher_ball_update(&b, 1700, p, 1, 1);
    sample(p, 50, 50); launcher_ball_update(&b, 1800, p, 1, 1);
    sample(p, 50, 50); launcher_ball_update(&b, 1900, p, 1, 1);
    assert(b.state == BALL_READY && b.motion == BALL_MOTION_NONE && b.completed_count == 1);

    /* 端点占球方向不可继续转时，向内回移，不计作拨球。 */
    b = idle(0);
    launcher_ball_request(&b, 0, p, 1);
    sample(p, 50, 0); launcher_ball_update(&b, 100, p, 1, 1);
    sample(p, 50, 0); launcher_ball_update(&b, 600, p, 1, 1);
    assert(b.motion == BALL_MOTION_REPOSITION && b.target_slot == 1 && !b.completed_count);

    /* 准备超时与实际搜索超时分开；条件恢复不会解除停止锁存。 */
    b = idle(0);
    launcher_ball_request(&b, 0, p, 1);
    sample(p, 0, 0); launcher_ball_update(&b, 100, p, 1, 0);
    sample(p, 0, 0); launcher_ball_update(&b, 200, p, 1, 0);
    assert(!b.search_started);
    launcher_ball_update(&b, 8000, p, 1, 0);
    assert(b.stopped && b.result == BALL_RESULT_TIMEOUT);
    sample(p, 0, 50); launcher_ball_update(&b, 8100, p, 1, 1);
    assert(b.motion == BALL_MOTION_NONE && b.stopped);
    launcher_ball_request(&b, 8200, p, 1);
    assert(!b.stopped);

    /* 全部工位确认无球后超时，返回无球；搜索始终处于合法工位。 */
    b = idle(0);
    launcher_ball_request(&b, 0, p, 1);
    for (uint32_t t = 100; t < 12000; t += 100) {
        sample(p, 0, 0);
        launcher_ball_update(&b, t, p, 1, 1);
        assert(b.target_slot < 4 && b.slot < 4);
    }
    assert(b.stopped && b.result == BALL_RESULT_NO_BALL && b.completed_count == 0);

    /* 掉线取消未提交请求，重连后不补拨。 */
    b = idle(1);
    launcher_ball_request(&b, 0, p, 1);
    launcher_ball_update(&b, 10, p, 0, 1);
    sample(p, 50, 50); launcher_ball_update(&b, 100, p, 1, 1);
    sample(p, 50, 50); launcher_ball_update(&b, 200, p, 1, 1);
    assert(!b.request_pending && b.motion == BALL_MOTION_NONE);

    /* 已提交拨球走完；中途掉线后即使重连，也不自动备下一颗。 */
    launcher_ball_request(&b, 300, p, 1);
    sample(p, 50, 50); launcher_ball_update(&b, 400, p, 1, 1);
    sample(p, 50, 50); launcher_ball_update(&b, 500, p, 1, 1);
    assert(b.motion == BALL_MOTION_FEED);
    launcher_ball_update(&b, 600, p, 0, 1);
    launcher_ball_update(&b, 1000, p, 1, 1);
    launcher_ball_update(&b, 2000, p, 1, 1);
    assert(b.completed_count == 1 && !b.prepare_pending);

    /* 压力掉线不能启动移动，上电定位按独立动作时间完成。 */
    b = idle(1); offline[0] = 1;
    launcher_ball_request(&b, 0, p, 1);
    sample(p, 50, 50); launcher_ball_update(&b, 100, p, 1, 1);
    sample(p, 50, 50); launcher_ball_update(&b, 200, p, 1, 1);
    launcher_ball_update(&b, 8000, p, 1, 1);
    assert(b.motion == BALL_MOTION_NONE && b.result == BALL_RESULT_SENSOR_OFFLINE);
    offline[0] = 0;
    b = idle(1); b.angle_deg = 135;
    launcher_ball_move(&b, BALL_MOTION_HOME, 0, 0, p);
    launcher_ball_update(&b, 1200, p, 0, 0);
    launcher_ball_update(&b, 2200, p, 0, 0);
    assert(b.slot == 0 && b.motion == BALL_MOTION_NONE && !b.completed_count);
    puts("launcher ball: all behavior checks passed");
    return 0;
}
'''
with tempfile.TemporaryDirectory() as directory:
    c_file = Path(directory) / 'test.c'
    binary = Path(directory) / 'test'
    c_file.write_text(prelude + helpers + cases)
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(ROOT / 'application'), '-I', str(ROOT / 'components/devices'),
                    str(c_file), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
