#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "remote_control.h"
#include "host_comm_task.h"

/* HT-10A 实测映射，通道编号从 1 开始。
 * CH8：朝内 1792 急停，中间 992 遥控，朝外 192 上位机。 */
#define CHASSIS_RC_MAPPING_CONFIRMED 1U
#define CHASSIS_RC_SWD_CHANNEL 8U
#define CHASSIS_RC_SWD_INNER_VALUE 1792U
#define CHASSIS_RC_SWD_MIDDLE_VALUE 992U
#define CHASSIS_RC_SWD_OUTER_VALUE 192U
#define CHASSIS_RC_SWD_TOLERANCE 120U

// 左摇杆 CH4/CH3 控制平移，右摇杆 CH1 控制旋转，CH2 忽略。
// 实测：左右轴右增左减，前后轴前增后减，中值均为 992。
#define CHASSIS_RC_TRANSLATION_X_CHANNEL 4U
#define CHASSIS_RC_TRANSLATION_Y_CHANNEL 3U
#define CHASSIS_RC_ROTATION_X_CHANNEL 1U
#define CHASSIS_RC_FORWARD_SIGN 1.0f
#define CHASSIS_RC_LEFT_SIGN (-1.0f)
#define CHASSIS_RC_CCW_SIGN (-1.0f)
#define CHASSIS_RC_MIN 192U
#define CHASSIS_RC_CENTER 992U
#define CHASSIS_RC_MAX 1792U
#define CHASSIS_RC_DEADBAND 30U
#define CHASSIS_MAX_LINEAR_SPEED 0.5f  // 遥控/上位机的平移合速度上限，m/s
#define CHASSIS_MAX_YAW_SPEED 1.0f     // 角速度上限，rad/s

typedef enum {
    CHASSIS_ESTOP_MODE = 0,
    CHASSIS_REMOTE_MODE,
    CHASSIS_HOST_MODE,
} chassis_mode_t;
typedef struct {
    chassis_mode_t mode;
    float vx, vy, wz;
    uint8_t host_waiting;
} chassis_control_t;
typedef struct {
    chassis_mode_t previous_mode;
    uint32_t host_entry_frame;
} chassis_behaviour_t;

/* 无独立线程，由底盘任务每轮调用；遥控失联、未知档位输出急停。
 * 最外档进入后必须收到新速度帧，才允许使用上位机命令。 */
void chassis_behaviour_update(chassis_behaviour_t *behaviour,
                              const RC_ctrl_t *remote,
                              const host_velocity_t *host,
                              chassis_control_t *control);
#endif
