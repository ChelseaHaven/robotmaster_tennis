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
  * @brief          优先检查 SWD 急停，再按 SWC 档位选择遥控或上位机模式
  * @param[in]      remote: 遥控器快照
  * @retval         急停、遥控或上位机控制模式
  */
static chassis_mode_t get_mode(const RC_ctrl_t *remote)
{
    if (!remote || !remote->valid || !CHASSIS_RC_MAPPING_CONFIRMED ||
        CHASSIS_RC_SWD_CHANNEL < 1 || CHASSIS_RC_SWD_CHANNEL > RC_CHANNEL_COUNT ||
        CHASSIS_RC_SWC_CHANNEL < 1 || CHASSIS_RC_SWC_CHANNEL > RC_CHANNEL_COUNT)
        return CHASSIS_ESTOP_MODE;
    uint16_t stop_value = remote->ch[CHASSIS_RC_SWD_CHANNEL - 1U];
    if (matches(stop_value, CHASSIS_RC_SWD_INNER_VALUE)) return CHASSIS_ESTOP_MODE;
    if (!matches(stop_value, CHASSIS_RC_SWD_MIDDLE_VALUE) &&
        !matches(stop_value, CHASSIS_RC_SWD_OUTER_VALUE)) return CHASSIS_ESTOP_MODE;
    uint16_t source_value = remote->ch[CHASSIS_RC_SWC_CHANNEL - 1U];
    if (matches(source_value, CHASSIS_RC_SWC_REMOTE_VALUE)) return CHASSIS_REMOTE_MODE;
    if (matches(source_value, CHASSIS_RC_SWC_HOST_VALUE)) return CHASSIS_HOST_MODE;
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
  * @param[in]      max_linear_speed: 平移合速度上限，m/s
  * @param[in]      max_yaw_speed: 角速度绝对值上限，rad/s
  * @retval         无
  */
static void limit_velocity(chassis_control_t *control, float max_linear_speed, float max_yaw_speed)
{
    float speed = hypotf(control->vx, control->vy);
    if (speed > max_linear_speed) {
        control->vx *= max_linear_speed/speed;
        control->vy *= max_linear_speed/speed;
    }
    if (control->wz > max_yaw_speed) control->wz = max_yaw_speed;
    if (control->wz < -max_yaw_speed) control->wz = -max_yaw_speed;
}

/**
  * @brief          根据 SWA 档位选择遥控器平移和旋转速度上限
  * @param[in]      remote: 有效的遥控器快照
  * @param[out]     linear: 平移合速度上限，m/s
  * @param[out]     yaw: 角速度上限，rad/s
  * @note           未匹配的档位按低档处理。
  * @retval         无
  */
static void get_remote_limits(const RC_ctrl_t *remote, float *linear, float *yaw)
{
    uint16_t value = remote->ch[CHASSIS_RC_SWA_CHANNEL - 1U];
    if (matches(value, CHASSIS_RC_SWA_HIGH_VALUE)) {
        *linear = CHASSIS_RC_HIGH_MAX_LINEAR_SPEED;
        *yaw = CHASSIS_RC_HIGH_MAX_YAW_SPEED;
    } else if (matches(value, CHASSIS_RC_SWA_MIDDLE_VALUE)) {
        *linear = CHASSIS_RC_MIDDLE_MAX_LINEAR_SPEED;
        *yaw = CHASSIS_RC_MIDDLE_MAX_YAW_SPEED;
    } else {
        *linear = CHASSIS_RC_LOW_MAX_LINEAR_SPEED;
        *yaw = CHASSIS_RC_LOW_MAX_YAW_SPEED;
    }
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
    if (remote && remote->valid &&
        matches(remote->ch[CHASSIS_RC_SWB_CHANNEL - 1U], CHASSIS_RC_SWB_HOLD_VALUE))
        control->stop_mode = CHASSIS_STOP_SPEED_HOLD;
    if (control->mode != behaviour->previous_mode) {
        behaviour->host_entry_frame = host ? host->frame_count : 0;
        behaviour->previous_mode = control->mode;
    }
    if (control->mode == CHASSIS_REMOTE_MODE) {
        float max_linear_speed, max_yaw_speed;
        get_remote_limits(remote, &max_linear_speed, &max_yaw_speed);
        control->vx = CHASSIS_RC_FORWARD_SIGN * axis(remote, CHASSIS_RC_TRANSLATION_Y_CHANNEL) * max_linear_speed;
        control->vy = CHASSIS_RC_LEFT_SIGN * axis(remote, CHASSIS_RC_TRANSLATION_X_CHANNEL) * max_linear_speed;
        control->wz = CHASSIS_RC_CCW_SIGN * axis(remote, CHASSIS_RC_ROTATION_X_CHANNEL) * max_yaw_speed;
        limit_velocity(control, max_linear_speed, max_yaw_speed);
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
    limit_velocity(control, CHASSIS_MAX_LINEAR_SPEED, CHASSIS_MAX_YAW_SPEED);
}
