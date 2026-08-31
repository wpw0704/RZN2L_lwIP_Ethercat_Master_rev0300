#ifndef ETHERCAT_MOTION_H
#define ETHERCAT_MOTION_H

#include <stdint.h>
#include <stdbool.h>

/**
 * 笔记
 *  | 状态                               |        含义                       |
    | `running=1`                       | 仍在每2ms生成新的 S 曲线位置         |
    | `running=0, waiting_actual=1`     | 指令已经到终点，正在等待电机实际到位   |
    | `running=0, waiting_actual=0`     | 当前运动段已经完成                   |
    | `busy=1`                          | 整个运动命令还没有结束                |
    | `paused=1`                        | 轨迹被暂停，内部状态仍然保留           |
 */

/* 往返次数为0时表示持续往返，直到上位机发送取消命令。 */
#define ETHERCAT_MOTION_RECIP_FOREVER (0U)
#define CSP_LOCAL_JERK_MM_S3      (500.0f)

/* 无参数回零接口使用的固定七段式S曲线参数。 */
#define ETHERCAT_MOTION_ZERO_RETURN_VELOCITY_MM_S      (10.0f)
#define ETHERCAT_MOTION_ZERO_RETURN_ACCELERATION_MM_S2 (10.0f)
#define ETHERCAT_MOTION_ZERO_RETURN_JERK_MM_S3         CSP_LOCAL_JERK_MM_S3

/* 电机参数只在运动模块内部保存，应用层直接传入各项数值。 */
typedef struct {
    float encoder_counts_per_motor_rev; /* 电机每转编码器计数，单位counts/rev。 */
    float lead_mm_per_screw_rev; /* 丝杠每转直线位移，单位mm/rev。 */
    float gear_ratio; /* 齿轮比，按电机转数/丝杠转数填写。 */
    float reducer_ratio; /* 减速机比，按输入转数/输出转数填写。 */
    float max_motor_rpm; /* 电机允许的最大转速，单位r/min。 */
} motion_motor_params_t;

/*
 * 运动接口统一返回值。
 * 注意：返回ETHERCAT_MOTION_OK只表示命令已被接受，不表示电机已经运动完成；
 * 是否完成应通过ethercat_motion_status_get()读取busy、done和error判断。
 */
typedef enum {
    ETHERCAT_MOTION_OK = 0, /* 命令或参数设置已成功接受。 */
    ETHERCAT_MOTION_ERR_PARAM = -1, /* 参数或运动模式不合法。 */
    ETHERCAT_MOTION_ERR_BUSY = -2, /* 未暂停的运动正在执行，或已有待处理命令。 */
    ETHERCAT_MOTION_ERR_NOT_READY = -3, /* 电机参数或PDO数据尚未准备好。 */
    ETHERCAT_MOTION_ERR_LIMIT = -4, /* 位置、速度或数值超出允许范围。 */
    ETHERCAT_MOTION_ERR_STATE = -5, /* 当前运动状态不允许执行该操作。 */
} ethercat_motion_result_t;

/* 当前运动模式。 */
typedef enum {
    ETHERCAT_MOTION_MODE_IDLE = 0, /* 空闲，当前没有正在执行的运动。 */
    ETHERCAT_MOTION_MODE_MOVE_ABS, /* 绝对运动，位置相对软件机械零点。 */
    ETHERCAT_MOTION_MODE_MOVE_REL, /* 相对运动，位置相对接收命令时的位置。 */
    ETHERCAT_MOTION_MODE_JOG, /* 有限距离点动，当前位置加指定偏移。 */
    ETHERCAT_MOTION_MODE_RECIP, /* 在起点和偏移终点之间往返运动。 */
    ETHERCAT_MOTION_MODE_STOP, /* 已停止并保持当前位置。 */
} ethercat_motion_mode_t;

/* 供上位机读取的运动状态。 */
typedef struct {
    ethercat_motion_mode_t mode; /* 当前运动模式。 */
    uint8_t busy; /* 1：运动、到位确认或往返等待仍在进行。 */
    uint8_t done; /* 1：上一条命令已正常完成或已执行停止。 */
    uint8_t error; /* 1：运动执行过程中发生错误。 */
    uint8_t paused; /* 1：运动已暂停，可调用continue继续。 */
    int32_t command_position_counts; /* 当前S曲线生成的CSP位置指令，单位counts。 */
    int32_t target_position_counts; /* 当前运动段的最终目标位置，单位counts。 */
    int32_t actual_position_counts; /* 驱动器0x6064反馈的实际位置，单位counts。 */
    uint32_t recip_completed_count; /* 已完成的完整“起点-终点-起点”次数。 */
} ethercat_motion_status_t;

/* 运动调度状态。 */
typedef struct {
    ethercat_motion_mode_t mode; /* 当前运动模式。 */
    uint8_t busy; /* 1：命令尚未完整结束。 */
    uint8_t done; /* 1：上一条命令已正常完成或已执行停止。 */
    uint8_t error; /* 1：运动执行过程中发生错误。 */
    uint8_t paused; /* 1：当前运动已暂停，内部轨迹状态仍保留。 */
    int32_t command_counts; /* 当前输出的S曲线位置指令，单位counts。 */
    int32_t target_counts; /* 当前运动段最终目标位置，单位counts。 */
} motion_control_t;

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
 * 当前运动暂停时允许提交，新命令被PDO任务取出后会覆盖原轨迹并开始运行。
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
 * @brief 把当前位置设置为软件零点。
 *
 * 接口在临界区读取0x6064当前位置，并将该计数保存为相对位置0mm。
 * 设零只保存在RAM，不写驱动器参数且掉电不保持。
 * 伺服未进入CSP运行状态、当前正在运动、已经暂停或存在其他待处理请求时拒绝设零。
 *
 * @return ethercat_motion_result_t；
 *         成功设置返回ETHERCAT_MOTION_OK，
 *         位置数据尚未准备好返回ETHERCAT_MOTION_ERR_NOT_READY，
 *         当前忙返回ETHERCAT_MOTION_ERR_BUSY。
 */
int ethercat_motion_software_zero_set(void);

/**
 * @brief 以S形曲线返回机械零点，即绝对值编码器计数0。
 *
 * 目标位置固定为0x6064绝对计数0，不叠加软件零点偏移。
 * 速度、加速度和Jerk使用ETHERCAT_MOTION_ZERO_RETURN_*固定配置。
 * 当前实现不搜索原点开关，也不执行CiA 402 Homing模式。
 *
 * @return ethercat_motion_result_t，成功入队返回ETHERCAT_MOTION_OK。
 */
int ethercat_motion_mechanical_zero_return(void);

/**
 * @brief 以S形曲线返回软件设置的零点。
 *
 * 目标位置是最近一次ethercat_motion_software_zero_set()记录的0x6064计数；
 * 上电后若尚未调用设零接口，软件零点默认为绝对值编码器计数0。
 * 速度、加速度和Jerk使用ETHERCAT_MOTION_ZERO_RETURN_*固定配置。
 * 当前实现不搜索原点开关，也不执行CiA 402 Homing模式。
 *
 * @return ethercat_motion_result_t，成功入队返回ETHERCAT_MOTION_OK。
 */
int ethercat_motion_software_zero_return(void);

/**
 * @brief 提交往返运动命令。
 *
 * 往返起点取接收命令时的实际位置，终点为起点加offset_mm；
 * 完成“起点到终点再返回起点”后，完整往返次数加1。
 * 当前运动暂停时允许提交，新往返命令被PDO任务取出后会覆盖原轨迹。
 *
 * @param offset_mm          往返终点相对起点的偏移量，单位mm，可为负数但不能为0。
 * @param velocity_mm_s      每个单程的最大直线速度，单位mm/s。
 * @param acceleration_mm_s2 每个单程的最大直线加速度，单位mm/s²。
 * @param jerk_mm_s3         每个单程的最大加加速度，单位mm/s³。
 * @param first_interval_ms  到达偏移终点后的等待时间，单位ms。
 * @param second_interval_ms 返回起点后的等待时间，单位ms。
 * @param repeat_count       完整往返次数；0表示持续往返直到收到取消命令。
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
 * @brief 暂停当前运动并保留完整轨迹状态。
 *
 * 暂停期间保持最后一个已生成的CSP目标位置，不推进轨迹周期、往返阶段或等待计数；
 * 后续调用ethercat_motion_continue()可从保存状态继续执行。
 * 暂停期间也可以提交新运动命令；新命令生效后原轨迹将被覆盖，不能再继续。
 *
 * @note 该接口不是安全急停，也不会丢弃当前轨迹。
 * @return ethercat_motion_result_t；成功入队返回ETHERCAT_MOTION_OK，
 *         没有可暂停的运动返回ETHERCAT_MOTION_ERR_STATE，
 *         已有待处理请求返回ETHERCAT_MOTION_ERR_BUSY。
 */
int ethercat_motion_stop(void);

/**
 * @brief 继续执行由ethercat_motion_stop()暂停的运动。
 *
 * 普通轨迹从保留的周期继续，往返运动从保留的单程、等待时间和完成次数继续。
 * 仅当busy=1、paused=1且没有其他待处理请求时接受；否则不改变当前状态。
 *
 * @return ethercat_motion_result_t；成功入队返回ETHERCAT_MOTION_OK，
 *         没有未完成的暂停运动返回ETHERCAT_MOTION_ERR_STATE，
 *         已有待处理请求返回ETHERCAT_MOTION_ERR_BUSY。
 */
int ethercat_motion_continue(void);

/**
 * @brief 取消当前运动，并把当前实际位置作为新的保持位置。
 *
 * 该操作会丢弃保存的轨迹和往返状态，调用后不能继续原运动。
 *
 * @note 该接口不是硬件急停，也不会切断伺服使能。
 * @return ethercat_motion_result_t；成功入队返回ETHERCAT_MOTION_OK，
 *         没有可取消的运动或请求返回ETHERCAT_MOTION_ERR_STATE。
 */
int ethercat_motion_abort(void);

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


/**
 * @brief 将运动控制器的目标位置同步到当前0x6064位置。
 *
 * 清除旧轨迹和待处理运动命令，并把当前位置作为新的保持位置和软件零点。
 *
 * @return ETHERCAT_MOTION_OK：同步成功；
 *         ETHERCAT_MOTION_ERR_NOT_READY：PDO尚未绑定。
 */
int ethercat_motion_position_sync(void);

float get_motor_position_mm(void);

uint8_t get_motion_request_pending(void);

float n_get_motor_position_mm(void);
#endif /* ETHERCAT_MOTION_H */
