#ifndef ETHERCAT_MOTION_H
#define ETHERCAT_MOTION_H

#include <stdint.h>

/* 往返次数为0时表示持续往返，直到上位机发送停止命令。 */
#define ETHERCAT_MOTION_RECIP_FOREVER (0U)
#define CSP_LOCAL_JERK_MM_S3      (500.0f)
/*
 * 运动接口统一返回值。
 * 注意：返回ETHERCAT_MOTION_OK只表示命令已被接受，不表示电机已经运动完成；
 * 是否完成应通过ethercat_motion_status_get()读取busy、done和error判断。
 */
typedef enum {
    ETHERCAT_MOTION_OK = 0,            /* 命令或参数设置已成功接受。 */
    ETHERCAT_MOTION_ERR_PARAM = -1,    /* 参数或运动模式不合法。 */
    ETHERCAT_MOTION_ERR_BUSY = -2,     /* 当前正在运动，或已有待处理命令。 */
    ETHERCAT_MOTION_ERR_NOT_READY = -3, /* 电机参数或PDO数据尚未准备好。 */
    ETHERCAT_MOTION_ERR_LIMIT = -4,    /* 位置、速度或数值超出允许范围。 */
} ethercat_motion_result_t;

/* 当前运动模式。 */
typedef enum {
    ETHERCAT_MOTION_MODE_IDLE = 0, /* 空闲，当前没有正在执行的运动。 */
    ETHERCAT_MOTION_MODE_MOVE_ABS, /* 绝对运动，位置相对软件机械零点。 */
    ETHERCAT_MOTION_MODE_MOVE_REL, /* 相对运动，位置相对接收命令时的位置。 */
    ETHERCAT_MOTION_MODE_JOG,      /* 有限距离点动，当前位置加指定偏移。 */
    ETHERCAT_MOTION_MODE_RECIP,    /* 在起点和偏移终点之间往返运动。 */
    ETHERCAT_MOTION_MODE_STOP,     /* 已停止并保持当前位置。 */
} ethercat_motion_mode_t;

/* 供上位机读取的运动状态。 */
typedef struct {
    ethercat_motion_mode_t mode;       /* 当前运动模式。 */
    uint8_t busy;                      /* 1：运动、到位确认或往返等待仍在进行。 */
    uint8_t done;                      /* 1：上一条命令已正常完成或已执行停止。 */
    uint8_t error;                     /* 1：运动执行过程中发生错误。 */
    int32_t command_position_counts;   /* 当前S曲线生成的CSP位置指令，单位counts。 */
    int32_t target_position_counts;    /* 当前运动段的最终目标位置，单位counts。 */
    int32_t actual_position_counts;    /* 驱动器0x6064反馈的实际位置，单位counts。 */
    uint32_t recip_completed_count;    /* 已完成的完整“起点-终点-起点”次数。 */
} ethercat_motion_status_t;

/**
 * @brief 设置电机编码器和机械传动参数。
 *
 * 这些参数用于完成mm、mm/s与编码器counts之间的换算；运动过程中不允许修改。
 * gear_ratio和reducer_ratio均按“电机转数/丝杠转数”填写。
 *
 * @param encoder_counts_per_motor_rev 电机旋转一圈对应的编码器计数，单位counts/rev。
 * @param lead_mm_per_screw_rev        丝杠旋转一圈的直线位移，单位mm/rev。
 * @param gear_ratio                   齿轮传动比，无单位，按电机转数/丝杠转数填写。
 * @param reducer_ratio                减速机传动比，无单位，按输入转数/输出转数填写。
 * @param max_motor_rpm                电机允许的最大转速，单位r/min。
 * @return ethercat_motion_result_t，成功返回ETHERCAT_MOTION_OK。
 */
int ethercat_motion_motor_params_set(float encoder_counts_per_motor_rev,
                                     float lead_mm_per_screw_rev,
                                     float gear_ratio,
                                     float reducer_ratio,
                                     float max_motor_rpm);

/**
 * @brief 提交绝对、相对或有限距离点动命令。
 *
 * MOVE_ABS模式下position_mm是相对软件机械零点的绝对位置；
 * MOVE_REL和JOG模式下position_mm是相对当前位置的偏移量。
 * velocity_mm_s、acceleration_mm_s2和jerk_mm_s3均为七段式S曲线的最大值，
 * 短距离运动可能达不到给定的最大速度或最大加速度。
 *
 * @param mode               运动模式，只允许MOVE_ABS、MOVE_REL或JOG。
 * @param position_mm        目标绝对位置或相对偏移量，单位mm，可为负数。
 * @param velocity_mm_s      最大直线速度，单位mm/s，必须大于0。
 * @param acceleration_mm_s2 最大直线加速度，单位mm/s²，必须大于0。
 * @param jerk_mm_s3         最大加加速度（加速度变化率），单位mm/s³，必须大于0。
 * @return ethercat_motion_result_t，成功返回ETHERCAT_MOTION_OK。
 */
/* 【七段式-接口1】相比原接口新增jerk_mm_s3参数。 */
int ethercat_motion_command_set(ethercat_motion_mode_t mode,
                                float position_mm,
                                float velocity_mm_s,
                                float acceleration_mm_s2,
                                float jerk_mm_s3);

/**
 * @brief 以S形曲线返回软件机械零点0mm。
 *
 * 当前实现不是搜索原点开关的CiA 402回零，而是执行目标位置为0mm的绝对运动。
 *
 * @param velocity_mm_s      最大回零速度，单位mm/s，必须大于0。
 * @param acceleration_mm_s2 最大回零加速度，单位mm/s²，必须大于0。
 * @param jerk_mm_s3         最大回零加加速度，单位mm/s³，必须大于0。
 * @return ethercat_motion_result_t，成功返回ETHERCAT_MOTION_OK。
 */
/* 【七段式-接口2】回零接口新增jerk_mm_s3参数。 */
int ethercat_motion_home_start(float velocity_mm_s,
                               float acceleration_mm_s2,
                               float jerk_mm_s3);

/**
 * @brief 提交往返运动命令。
 *
 * 往返起点取接收命令时的实际位置，终点为起点加offset_mm；
 * 完成“起点到终点再返回起点”后，完整往返次数加1。
 *
 * @param offset_mm          往返终点相对起点的偏移量，单位mm，可为负数但不能为0。
 * @param velocity_mm_s      每个单程的最大直线速度，单位mm/s。
 * @param acceleration_mm_s2 每个单程的最大直线加速度，单位mm/s²。
 * @param jerk_mm_s3         每个单程的最大加加速度，单位mm/s³。
 * @param first_interval_ms  到达偏移终点后的等待时间，单位ms。
 * @param second_interval_ms 返回起点后的等待时间，单位ms。
 * @param repeat_count       完整往返次数；0表示持续往返直到收到停止命令。
 * @return ethercat_motion_result_t，成功返回ETHERCAT_MOTION_OK。
 */
/* 【七段式-接口3】往返接口新增jerk_mm_s3参数。 */
int ethercat_motion_recip_start(float offset_mm,
                                float velocity_mm_s,
                                float acceleration_mm_s2,
                                float jerk_mm_s3,
                                uint32_t first_interval_ms,
                                uint32_t second_interval_ms,
                                uint32_t repeat_count);

/**
 * @brief 立即取消当前轨迹，并把当前实际位置作为新的保持位置。
 * @note 当前是立即保持停止，不是按S形曲线减速停止。
 */
void ethercat_motion_stop(void);

/**
 * @brief 运动调度器，每个PDO周期必须调用一次。
 *
 * 函数负责接收待处理命令、推进S形轨迹、判断到位状态，并生成下一周期的CSP目标位置。
 * 当前PDO周期为2ms；该函数应在EtherCAT实时周期任务中调用，不能放入普通低速任务。
 */
void ethercat_motion_process(void);

/**
 * @brief 获取供应用层或上位机读取的运动状态快照。
 * @param status 用于接收状态的结构体指针；传入NULL时函数直接返回。
 */
void ethercat_motion_status_get(ethercat_motion_status_t *status);

#endif /* ETHERCAT_MOTION_H */
