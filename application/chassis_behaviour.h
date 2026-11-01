#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "remote_control.h"
#include "host_comm_task.h"

/* HT-10A 实测映射，通道编号从 1 开始。
 * SWC/CH7：下档 1792 遥控，上档 192 上位机。
 * SWD/CH8：下档 1792 急停，中档 992 和上档 192 均放行。 */
#define CHASSIS_RC_MAPPING_CONFIRMED 1U
#define CHASSIS_RC_SWC_CHANNEL 7U
#define CHASSIS_RC_SWC_REMOTE_VALUE 1792U
#define CHASSIS_RC_SWC_HOST_VALUE 192U
#define CHASSIS_RC_SWD_CHANNEL 8U
#define CHASSIS_RC_SWD_INNER_VALUE 1792U
#define CHASSIS_RC_SWD_MIDDLE_VALUE 992U
#define CHASSIS_RC_SWD_OUTER_VALUE 192U
#define CHASSIS_RC_SWD_TOLERANCE 120U

// SWA 从里到外选择低、中、高档；SWB 仅决定目标零速时的停车方式。
#define CHASSIS_RC_SWA_CHANNEL 5U
#define CHASSIS_RC_SWA_LOW_VALUE 1792U
#define CHASSIS_RC_SWA_MIDDLE_VALUE 992U
#define CHASSIS_RC_SWA_HIGH_VALUE 192U
#define CHASSIS_RC_SWB_CHANNEL 6U
#define CHASSIS_RC_SWB_ZERO_CURRENT_VALUE 192U
#define CHASSIS_RC_SWB_HOLD_VALUE 1792U
#define CHASSIS_RC_LOW_MAX_LINEAR_SPEED 0.5f
#define CHASSIS_RC_LOW_MAX_YAW_SPEED 1.0f
#define CHASSIS_RC_MIDDLE_MAX_LINEAR_SPEED 1.0f
#define CHASSIS_RC_MIDDLE_MAX_YAW_SPEED 1.5f
#define CHASSIS_RC_HIGH_MAX_LINEAR_SPEED 1.5f
#define CHASSIS_RC_HIGH_MAX_YAW_SPEED 2.0f

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
#define CHASSIS_MAX_LINEAR_SPEED 3.21f    // 车体平移合速度总上限，上位机也受此限制，m/s
#define CHASSIS_MAX_YAW_SPEED 9.52f       // 车体角速度总上限，rad/s

typedef enum {
    CHASSIS_ESTOP_MODE = 0,
    CHASSIS_REMOTE_MODE,
    CHASSIS_HOST_MODE,
} chassis_mode_t;
typedef enum {
    CHASSIS_STOP_ZERO_CURRENT = 0,
    CHASSIS_STOP_SPEED_HOLD,
} chassis_stop_mode_t;
typedef struct {
    chassis_mode_t mode;
    chassis_stop_mode_t stop_mode;
    float vx, vy, wz;
    uint8_t host_waiting;
} chassis_control_t;
typedef struct {
    chassis_mode_t previous_mode;
    uint32_t host_entry_frame;
} chassis_behaviour_t;

/**
  * @brief          仲裁急停、遥控和上位机速度指令并进行限幅
  * @param[in,out]  behaviour: 模式切换状态
  * @param[in]      remote: 遥控器快照
  * @param[in]      host: 上位机速度快照
  * @param[out]     control: 当前模式及车体速度指令
  * @note           SWA 选择遥控限速，SWB 选择零速停车方式；SWC 选控制源，SWD 下档急停。
  *                 遥控失联时急停并给零电流。
  *                 切入上位机模式后等待一帧新的有效速度指令。
  * @retval         无
  */
void chassis_behaviour_update(chassis_behaviour_t *behaviour,
                              const RC_ctrl_t *remote,
                              const host_velocity_t *host,
                              chassis_control_t *control);
#endif
