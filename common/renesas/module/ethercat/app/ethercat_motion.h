#ifndef ETHERCAT_MOTION_H
#define ETHERCAT_MOTION_H

#include <stdint.h>

/* 往返次数为0时表示持续往返，直到上位机发送停止命令。 */
#define ETHERCAT_MOTION_RECIP_FOREVER (0U)

/* 运动接口统一返回值。 */
typedef enum {
    ETHERCAT_MOTION_OK = 0,
    ETHERCAT_MOTION_ERR_PARAM = -1,
    ETHERCAT_MOTION_ERR_BUSY = -2,
    ETHERCAT_MOTION_ERR_NOT_READY = -3,
    ETHERCAT_MOTION_ERR_LIMIT = -4,
} ethercat_motion_result_t;

/* 当前运动模式。 */
typedef enum {
    ETHERCAT_MOTION_MODE_IDLE = 0,
    ETHERCAT_MOTION_MODE_MOVE_ABS,
    ETHERCAT_MOTION_MODE_MOVE_REL,
    ETHERCAT_MOTION_MODE_JOG,
    ETHERCAT_MOTION_MODE_RECIP,
    ETHERCAT_MOTION_MODE_STOP,
} ethercat_motion_mode_t;

/* 供上位机读取的运动状态。 */
typedef struct {
    ethercat_motion_mode_t mode;
    uint8_t busy;
    uint8_t done;
    uint8_t error;
    int32_t command_position_counts;
    int32_t target_position_counts;
    int32_t actual_position_counts;
    uint32_t recip_completed_count;
} ethercat_motion_status_t;

/* 设置电机和机械参数；运动过程中不允许修改。 */
int ethercat_motion_motor_params_set(float encoder_counts_per_motor_rev,
                                     float lead_mm_per_screw_rev,
                                     float gear_ratio,
                                     float reducer_ratio,
                                     float max_motor_rpm);

/* 提交绝对、相对或有限距离点动命令。 */
int ethercat_motion_command_set(ethercat_motion_mode_t mode,
                                float position_mm,
                                float velocity_mm_s,
                                float acceleration_mm_s2);

/* 回到机械零点；机械零点对应目标位置0。 */
int ethercat_motion_home_start(float velocity_mm_s,
                               float acceleration_mm_s2);

/* 提交往返命令；repeat_count为0表示一直往返。 */
int ethercat_motion_recip_start(float offset_mm,
                                float velocity_mm_s,
                                float acceleration_mm_s2,
                                uint32_t first_interval_ms,
                                uint32_t second_interval_ms,
                                uint32_t repeat_count);

/* 停止当前运动并保持当前位置。 */
void ethercat_motion_stop(void);

/* 每个PDO周期调用一次，生成下一周期的CSP目标位置。 */
void ethercat_motion_process(void);

/* 获取当前运动状态。 */
void ethercat_motion_status_get(ethercat_motion_status_t *status);

#endif /* ETHERCAT_MOTION_H */