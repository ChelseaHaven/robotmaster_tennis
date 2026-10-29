#include "chassis_behaviour.h"
#include <math.h>
#include <stddef.h>

/**
  * @brief          判断遥控器通道是否落在指定档位的容差内
  * @param[in]      value: 通道原始值
  * @param[in]      expected: 档位标称值
  * @retval         1：匹配；0：不匹配
  */
static uint8_t matches(uint16_t value, uint16_t expected)
{
    int difference = (int)value - (int)expected;
    return difference >= -(int)CHASSIS_RC_SWD_TOLERANCE &&
           difference <= (int)CHASSIS_RC_SWD_TOLERANCE;
}

/**
  * @brief          按遥控器有效状态和 CH8 档位选择底盘模式
  * @param[in]      remote: 遥控器快照
  * @retval         急停、遥控或上位机控制模式
  */
static chassis_mode_t get_mode(const RC_ctrl_t *remote)
{
    if (!remote || !remote->valid || !CHASSIS_RC_MAPPING_CONFIRMED ||
        CHASSIS_RC_SWD_CHANNEL < 1 || CHASSIS_RC_SWD_CHANNEL > RC_CHANNEL_COUNT)
        return CHASSIS_ESTOP_MODE;
    unsigned channel = CHASSIS_RC_SWD_CHANNEL;
    if (channel == 0) return CHASSIS_ESTOP_MODE;
    uint16_t value = remote->ch[channel-1];
    // 急停优先；不在任何已确认档位附近时同样停止。
    if (matches(value, CHASSIS_RC_SWD_INNER_VALUE)) return CHASSIS_ESTOP_MODE;
    if (matches(value, CHASSIS_RC_SWD_MIDDLE_VALUE)) return CHASSIS_REMOTE_MODE;
    if (matches(value, CHASSIS_RC_SWD_OUTER_VALUE)) return CHASSIS_HOST_MODE;
    return CHASSIS_ESTOP_MODE;
}

/**
  * @brief          将摇杆原始通道转换为带死区的归一化输入
  * @param[in]      remote: 遥控器快照
  * @param[in]      channel: 通道号，从 1 开始
  * @retval         归一化输入，范围 -1～1
  */
static float axis(const RC_ctrl_t *remote, unsigned channel)
{
    if (channel < 1 || channel > RC_CHANNEL_COUNT) return 0;
    float delta = (float)remote->ch[channel-1] - CHASSIS_RC_CENTER;
    float magnitude = fabsf(delta);
    if (magnitude <= CHASSIS_RC_DEADBAND) return 0;
    float span = delta > 0 ? CHASSIS_RC_MAX-CHASSIS_RC_CENTER : CHASSIS_RC_CENTER-CHASSIS_RC_MIN;
    float value = (magnitude-CHASSIS_RC_DEADBAND)/(span-CHASSIS_RC_DEADBAND);
    if (value > 1) value = 1;
    return delta > 0 ? value : -value;
}

/**
  * @brief          限制车体平移合速度和角速度
  * @param[in,out]  control: 待限幅的车体控制指令
  * @retval         无
  */
static void limit_velocity(chassis_control_t *control)
{
    float speed = hypotf(control->vx, control->vy);
    if (speed > CHASSIS_MAX_LINEAR_SPEED) {
        control->vx *= CHASSIS_MAX_LINEAR_SPEED/speed;
        control->vy *= CHASSIS_MAX_LINEAR_SPEED/speed;
    }
    if (control->wz > CHASSIS_MAX_YAW_SPEED) control->wz = CHASSIS_MAX_YAW_SPEED;
    if (control->wz < -CHASSIS_MAX_YAW_SPEED) control->wz = -CHASSIS_MAX_YAW_SPEED;
}
void chassis_behaviour_update(chassis_behaviour_t *behaviour,
                              const RC_ctrl_t *remote,
                              const host_velocity_t *host,
                              chassis_control_t *control)
{
    if (!control) return;
    *control = (chassis_control_t){0};
    if (!behaviour) return;
    control->mode = get_mode(remote);
    if (control->mode != behaviour->previous_mode) {
        behaviour->host_entry_frame = host ? host->frame_count : 0;
        behaviour->previous_mode = control->mode;
    }
    if (control->mode == CHASSIS_REMOTE_MODE) {
        control->vx = CHASSIS_RC_FORWARD_SIGN * axis(remote, CHASSIS_RC_TRANSLATION_Y_CHANNEL) * CHASSIS_MAX_LINEAR_SPEED;
        control->vy = CHASSIS_RC_LEFT_SIGN * axis(remote, CHASSIS_RC_TRANSLATION_X_CHANNEL) * CHASSIS_MAX_LINEAR_SPEED;
        control->wz = CHASSIS_RC_CCW_SIGN * axis(remote, CHASSIS_RC_ROTATION_X_CHANNEL) * CHASSIS_MAX_YAW_SPEED;
    } else if (control->mode == CHASSIS_HOST_MODE) {
        control->host_waiting = 1;
        if (host && host->valid && host->frame_count != behaviour->host_entry_frame &&
            isfinite(host->vx) && isfinite(host->vy) && isfinite(host->wz)) {
            control->vx = host->vx;
            control->vy = host->vy;
            control->wz = host->wz;
            control->host_waiting = 0;
        }
    }
    limit_velocity(control);
}
