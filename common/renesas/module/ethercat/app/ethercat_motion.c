#include "ethercat_motion.h"

#include "ethercat_master.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ethercat_app_common.h"

#include <float.h>
#include <limits.h>

/* 当前EtherCAT PDO周期为2ms。 */
#define MOTION_PDO_PERIOD_S                  (0.002f)

/* 【七段式-1】经典七段式Jerk受限S曲线固定包含7个时间段。 */
#define MOTION_S7_SEGMENT_COUNT              (7U)

/* 轨迹结束后，实际位置连续落入误差范围才报告完成。 */
#define MOTION_POSITION_TOLERANCE_COUNTS     (500LL)
#define MOTION_POSITION_STABLE_CYCLES        (10U)

/* 毫秒转换为2ms PDO周期数，向上取整。 */
#define MOTION_MS_TO_CYCLES(ms) \
    (((uint32_t) (ms) / 2U) + (((uint32_t) (ms) % 2U) ? 1U : 0U))

typedef enum {
    MOTION_SEGMENT_RUNNING = 0, /* 当前运动段仍在运行或等待实际位置到位。 */
    MOTION_SEGMENT_DONE = 1, /* 当前运动段已完成并通过到位稳定判断。 */
    MOTION_SEGMENT_ERROR = -1, /* 当前运动段执行失败。 */
} motion_segment_result_t;

/* 应用任务提交给PDO任务的内部请求动作。 */
typedef enum {
    MOTION_REQUEST_ACTION_START = 0, /* 启动request中描述的新运动。 */
    MOTION_REQUEST_ACTION_PAUSE, /* 暂停并保留当前运动状态。 */
    MOTION_REQUEST_ACTION_CONTINUE, /* 从保留状态继续当前运动。 */
    MOTION_REQUEST_ACTION_ABORT, /* 取消当前运动并保持实际位置。 */
} motion_request_action_t;

/* MOVE_ABS请求使用的绝对坐标参考系。 */
typedef enum {
    MOTION_ABSOLUTE_REFERENCE_SOFTWARE_ZERO = 0, /* 相对软件设置零点。 */
    MOTION_ABSOLUTE_REFERENCE_ENCODER_ZERO, /* 相对绝对值编码器计数0。 */
} motion_absolute_reference_t;

/* 往返运动内部阶段。 */
typedef enum {
    RECIP_STAGE_IDLE = 0, /* 往返状态机空闲。 */
    RECIP_STAGE_MOVE_OUT, /* 从起点向偏移终点运动。 */
    RECIP_STAGE_WAIT_FIRST, /* 到达偏移终点后的第一次等待。 */
    RECIP_STAGE_MOVE_BACK, /* 从偏移终点返回起点。 */
    RECIP_STAGE_WAIT_SECOND, /* 返回起点后的第二次等待。 */
} recip_stage_t;

/* 上位机命令先写入单槽请求区，再由PDO周期任务统一接收。 */
typedef struct {
    uint8_t pending; /* 1：存在尚未被PDO任务取走的命令。 */
    motion_request_action_t action; /* 启动、暂停、继续或取消动作。 */
    ethercat_motion_mode_t mode; /* 新运动请求的运动模式。 */
    motion_absolute_reference_t absolute_reference; /* MOVE_ABS的坐标参考系。 */
    float position_mm; /* 绝对位置、相对偏移或往返偏移，单位mm。 */
    float velocity_mm_s; /* 允许的最大直线速度，单位mm/s。 */
    float acceleration_mm_s2; /* 允许的最大直线加速度，单位mm/s²。 */
    /* 【七段式-2】命令区增加用户可设置的Jerk上限。 */
    float jerk_mm_s3; /* 允许的最大加加速度，单位mm/s³。 */
    uint32_t first_interval_ms; /* 到达往返偏移终点后的等待时间，单位ms。 */
    uint32_t second_interval_ms; /* 返回往返起点后的等待时间，单位ms。 */
    uint32_t repeat_count; /* 完整往返次数，0表示持续往返。 */
} motion_request_t;

/* 【七段式-3开始】新增七段轨迹的数据结构。 */
/* 七段式轨迹中单个恒定Jerk时间段的起始状态。 */
typedef struct {
    float start_time_s; /* 本段相对整条轨迹的开始时间，单位s。 */
    float duration_s; /* 本段持续时间，单位s；可为0。 */
    float start_position_counts; /* 本段开始时相对起点的位移，单位counts。 */
    float start_velocity_counts_s; /* 本段开始速度，单位counts/s。 */
    float start_acceleration_counts_s2; /* 本段开始加速度，单位counts/s²。 */
    float jerk_counts_s3; /* 本段恒定Jerk，单位counts/s³。 */
} motion_s7_segment_t;

/* 经典七段式Jerk受限S形轨迹运行数据。 */
typedef struct {
    uint8_t running; /* 1：七段式目标位置仍在生成。 */
    uint8_t waiting_actual; /* 1：目标已生成完，正在等待实际位置稳定到位。 */
    uint32_t cycle_index; /* 当前已经执行的S曲线PDO周期序号。 */
    uint32_t total_cycles; /* 当前运动段总PDO周期数。 */
    uint32_t actual_stable_cycles; /* 实际位置连续落入误差范围的周期数。 */
    int32_t start_counts; /* 当前运动段起点，单位编码器counts。 */
    int32_t target_counts; /* 当前运动段最终终点，单位编码器counts。 */
    int32_t command_counts; /* 当前周期计算出的CSP位置指令，单位counts。 */
    float total_time_s; /* 七个时间段的总运动时间，单位s。 */
    motion_s7_segment_t segment[MOTION_S7_SEGMENT_COUNT]; /* 七段起始状态。 */
} motion_trajectory_t;

/* 【七段式-3结束】七段轨迹的数据结构到此结束。 */

/* 往返运动状态。 */
typedef struct {
    uint8_t active; /* 1：往返状态机已启动。 */
    recip_stage_t stage; /* 当前正向、返回或等待阶段。 */
    int32_t start_counts; /* 接收命令时的往返起点，单位counts。 */
    int32_t end_counts; /* 起点加偏移后的往返终点，单位counts。 */
    float velocity_mm_s; /* 每个单程的最大速度，单位mm/s。 */
    float acceleration_mm_s2; /* 每个单程的最大加速度，单位mm/s²。 */
    float jerk_mm_s3; /* 每个单程的最大加加速度，单位mm/s³。 */
    uint32_t first_interval_cycles; /* 偏移终点等待时间换算后的PDO周期数。 */
    uint32_t second_interval_cycles; /* 起点等待时间换算后的PDO周期数。 */
    uint32_t wait_cycles; /* 当前等待阶段剩余PDO周期数。 */
    uint32_t repeat_count; /* 要执行的完整往返次数，0表示持续往返。 */
    uint32_t completed_count; /* 已完成的“起点-终点-起点”次数。 */
} motion_recip_t;

static motion_motor_params_t s_motor_params; /* 当前电机与机械换算参数。 */
static uint8_t s_motor_params_ready; /* 1：电机机械参数已经设置。 */
static uint8_t s_motion_initialized; /* 1：命令位置已与首次实际位置对齐。 */
static int32_t s_software_zero_counts; /* 软件零点对应的0x6064绝对计数。 */
static motion_request_t s_request; /* 应用任务到PDO任务的单槽命令区。 */
static motion_trajectory_t s_trajectory; /* 当前S形运动段的运行数据。 */

/* 当前运动调度状态，上电默认空闲且目标位置为0。 */
motion_control_t s_control = {
    .mode = ETHERCAT_MOTION_MODE_IDLE,
    .busy = 0U,
    .done = 0U,
    .error = 0U,
    .paused = 0U,
    .command_counts = 0,
    .target_counts = 0,
};
static motion_recip_t s_recip; /* 当前往返运动状态机数据。 */

/* 计算正数平方根，不依赖libm。 */
static float motion_positive_sqrt(float value);

/* 计算正数立方根，不依赖libm。 */
static float motion_positive_cbrt(float value);

/* 判断浮点数是否为有限值。 */
static uint8_t motion_float_is_finite(float value);

/* 判断浮点数是否为有限正数。 */
static uint8_t motion_float_is_positive(float value);

/* 获取1mm对应的编码器计数。 */
static float motion_counts_per_mm_get(void);

/* 获取最大电机转速对应的直线速度。 */
static float motion_max_linear_velocity_mm_s_get(void);

/* 把毫米位置或偏移量转换成编码器计数。 */
static int motion_mm_to_counts(float position_mm, int32_t *counts);

/* 根据绝对、相对或点动模式计算最终绝对目标位置。 */
static int motion_position_target_get(ethercat_motion_mode_t mode,
                                      motion_absolute_reference_t absolute_reference,
                                      float position_mm,
                                      int32_t *target_counts);

/* 检查运动模式、速度、加速度、Jerk和转速限制。 */
static int motion_request_validate(ethercat_motion_mode_t mode,
                                   float velocity_mm_s,
                                   float acceleration_mm_s2,
                                   float jerk_mm_s3);

/* 把应用层命令写入单槽请求区。 */
static int motion_request_submit(const motion_request_t *request);

/* 由PDO任务读取并清除一条待处理命令。 */
static uint8_t motion_request_fetch(motion_request_t *request);

/* 根据起点、终点、速度、加速度和Jerk创建一段七段式S形轨迹。 */
static int motion_trajectory_start(int32_t start_counts,
                                   int32_t target_counts,
                                   float velocity_mm_s,
                                   float acceleration_mm_s2,
                                   float jerk_mm_s3);

/* 每个PDO周期推进一次当前S形运动段。 */
static int motion_segment_process(void);

/* 启动绝对、相对或点动的单段运动。 */
static int motion_single_start(const motion_request_t *request);

/* 启动完整往返运动状态机。 */
static int motion_recip_start_internal(const motion_request_t *request);

/* 启动往返运动中的一个单程。 */
static int motion_recip_move_start(int32_t target_counts);

/* 每个PDO周期推进一次往返状态机。 */
static void motion_recip_process(void);

/* 将取出的应用层请求转换为内部运动状态。 */
static void motion_request_apply(const motion_request_t *request);

/* 取消当前运动并把实际位置设置为新的保持位置。 */
static void motion_abort_apply(void);

/* 标记当前命令正常结束。 */
static void motion_finish_success(void);

/* 停止轨迹并标记当前命令执行错误。 */
static void motion_finish_error(void);

/* 获取后续压力或位置闭环产生的位置修正量。 */
static int32_t motion_closed_loop_correction_get(void);

/* 合成开环轨迹和闭环修正量，并写入RxPDO。 */
static void motion_target_output_write(void);

/*
 * 不依赖libm的正数平方根，用于根据最大加速度计算S曲线时间。
 * 该函数只在新命令开始时运行一次，不会在每个轨迹周期重复计算。
 */
static float motion_positive_sqrt(float value) {
    if (value <= 0.0f) {
        return 0.0f;
    }

    float estimate = (value > 1.0f) ? value : 1.0f;

    for (uint32_t i = 0U; i < 24U; i++) {
        estimate = 0.5f * (estimate + value / estimate);
    }

    return estimate;
}

/*
 * 不依赖libm的正数立方根，用于短距离七段式轨迹的时间计算。
 * 该函数只在接收新运动段时运行，不占用每个PDO周期的实时计算时间。
 */
static float motion_positive_cbrt(float value) {
    if (value <= 0.0f) {
        return 0.0f;
    }

    /* 先按2倍缩放到立方根附近，避免直接用value作为初值造成溢出。 */
    float estimate = 1.0f;
    if (value > 1.0f) {
        while (estimate < value / estimate / estimate) {
            estimate *= 2.0f;
        }
    } else {
        while (estimate > value / estimate / estimate) {
            estimate *= 0.5f;
        }
    }

    for (uint32_t i = 0U; i < 16U; i++) {
        estimate = (2.0f * estimate +
                    value / estimate / estimate) /
                   3.0f;
    }

    return estimate;
}

/*
 * 检查浮点参数是否为有限值。
 * 返回1表示有效普通数值，返回0表示NaN或正负无穷。
 */
static uint8_t motion_float_is_finite(float value) {
    return (uint8_t) ((value <= FLT_MAX) &&
                      (value >= -FLT_MAX));
}

/* 检查浮点参数是否为大于0的有限值，合法返回1，否则返回0。 */
static uint8_t motion_float_is_positive(float value) {
    return (uint8_t) (motion_float_is_finite(value) &&
                      (value > 0.0f));
}

/*
 * 计算1mm对应的编码器计数：
 * counts/mm = 编码器每转计数 × 齿轮比 × 减速机比 ÷ 丝杠导程。
 */
static float motion_counts_per_mm_get(void) {
    return (s_motor_params.encoder_counts_per_motor_rev *
            s_motor_params.gear_ratio *
            s_motor_params.reducer_ratio) /
           s_motor_params.lead_mm_per_screw_rev;
}

/*
 * 根据最大电机转速和传动参数计算机构允许的最大直线速度，返回单位mm/s。
 */
static float motion_max_linear_velocity_mm_s_get(void) {
    const float screw_rps = (s_motor_params.max_motor_rpm / 60.0f) /
                            (s_motor_params.gear_ratio *
                             s_motor_params.reducer_ratio);

    const float linear_mm_s = screw_rps *
                              s_motor_params.lead_mm_per_screw_rev;

    return linear_mm_s;
}

/*
 * 将毫米位置或偏移量转换为编码器计数，并检查int32_t范围。
 * position_mm可以为负数；转换结果通过counts返回。
 */
static int motion_mm_to_counts(float position_mm, int32_t *counts) {
    if ((counts == NULL) ||
        (!motion_float_is_finite(position_mm))) {
        return ETHERCAT_MOTION_ERR_PARAM;
    }

    const float counts_float = position_mm * motion_counts_per_mm_get();

    if ((counts_float > (float) INT32_MAX) ||
        (counts_float < (float) INT32_MIN)) {
        return ETHERCAT_MOTION_ERR_LIMIT;
    }

    if (counts_float >= 0.0f) {
        *counts = (int32_t) (counts_float + 0.5f);
    } else {
        *counts = (int32_t) (counts_float - 0.5f);
    }

    return ETHERCAT_MOTION_OK;
}

/*
 * 根据运动模式把应用层位置参数转换为最终绝对counts：
 * MOVE_ABS按请求选择软件零点或绝对值编码器零点作为坐标参考；
 * MOVE_REL和JOG在当前实际位置上叠加偏移。
 */
static int motion_position_target_get(ethercat_motion_mode_t mode,
                                      motion_absolute_reference_t absolute_reference,
                                      float position_mm,
                                      int32_t *target_counts) {
    int32_t offset_counts;
    int64_t target_64;

    if ((input1s == NULL) || (target_counts == NULL)) {
        return ETHERCAT_MOTION_ERR_NOT_READY;
    }

    if ((mode != ETHERCAT_MOTION_MODE_MOVE_ABS) &&
        (mode != ETHERCAT_MOTION_MODE_MOVE_REL) &&
        (mode != ETHERCAT_MOTION_MODE_JOG)) {
        return ETHERCAT_MOTION_ERR_PARAM;
    }

    const int result = motion_mm_to_counts(position_mm, &offset_counts);
    if (result != ETHERCAT_MOTION_OK) {
        return result;
    }

    if (mode == ETHERCAT_MOTION_MODE_MOVE_ABS) {
        if (absolute_reference ==
            MOTION_ABSOLUTE_REFERENCE_SOFTWARE_ZERO) {
            target_64 = (int64_t) s_software_zero_counts +
                        (int64_t) offset_counts;
        } else if (absolute_reference ==
                   MOTION_ABSOLUTE_REFERENCE_ENCODER_ZERO) {
            target_64 = (int64_t) offset_counts;
        } else {
            return ETHERCAT_MOTION_ERR_PARAM;
        }
    } else {
        target_64 = (int64_t) input1s->CurrentPosition +
                    (int64_t) offset_counts;
    }

    if ((target_64 > INT32_MAX) || (target_64 < INT32_MIN)) {
        return ETHERCAT_MOTION_ERR_LIMIT;
    }

    *target_counts = (int32_t) target_64;
    return ETHERCAT_MOTION_OK;
}

/*
 * 检查运动模式、速度、加速度、Jerk以及电机最大转速限制。
 * 三个运动学限制必须是有限正数，速度不得超过机械最大直线速度。
 */
static int motion_request_validate(ethercat_motion_mode_t mode,
                                   float velocity_mm_s,
                                   float acceleration_mm_s2,
                                   float jerk_mm_s3) {
    if (!s_motor_params_ready) {
        return ETHERCAT_MOTION_ERR_NOT_READY;
    }

    if ((mode != ETHERCAT_MOTION_MODE_MOVE_ABS) &&
        (mode != ETHERCAT_MOTION_MODE_MOVE_REL) &&
        (mode != ETHERCAT_MOTION_MODE_JOG) &&
        (mode != ETHERCAT_MOTION_MODE_RECIP)) {
        return ETHERCAT_MOTION_ERR_PARAM;
    }

    if ((!motion_float_is_positive(velocity_mm_s)) ||
        (!motion_float_is_positive(acceleration_mm_s2)) ||
        (!motion_float_is_positive(jerk_mm_s3))) {
        return ETHERCAT_MOTION_ERR_PARAM;
    }

    if (velocity_mm_s > motion_max_linear_velocity_mm_s_get()) {
        return ETHERCAT_MOTION_ERR_LIMIT;
    }

    return ETHERCAT_MOTION_OK;
}

/*
 * 单槽命令提交接口。
 * 应用任务只写s_request，不直接修改PDO周期状态。
 * 已有待处理命令或正在执行未暂停的运动时返回BUSY；暂停时允许新命令覆盖原轨迹。
 */
static int motion_request_submit(const motion_request_t *request) {
    int result = ETHERCAT_MOTION_OK;

    if (request == NULL) {
        return ETHERCAT_MOTION_ERR_PARAM;
    }

    taskENTER_CRITICAL();

    if (s_request.pending ||
        (s_control.busy && !s_control.paused)) {
        result = ETHERCAT_MOTION_ERR_BUSY;
    } else {
        s_request = *request;
        s_request.pending = 1U;
    }

    taskEXIT_CRITICAL();
    return result;
}

/*
 * PDO任务读取并清除应用层请求。
 * 返回1表示成功取到命令，返回0表示当前没有待处理命令。
 */
static uint8_t motion_request_fetch(motion_request_t *request) {
    uint8_t has_request = 0U;

    taskENTER_CRITICAL();

    if (s_request.pending) {
        *request = s_request;
        s_request.pending = 0U;
        has_request = 1U;
    }

    taskEXIT_CRITICAL();
    return has_request;
}

/* 【七段式-4开始】下面是七段式轨迹规划器，替换原五次多项式。 */
/*
 * 创建经典七段式Jerk受限S曲线。七段依次为：
 * 1正Jerk、2恒加速、3负Jerk、4匀速、
 * 5负Jerk、6恒减速、7正Jerk。
 *
 * 对短距离运动，恒加速段或匀速段的时间可能自动变成0，但仍使用同一套
 * 七段数据结构。这样速度、加速度和Jerk三个上限都不会被规划器主动超过。
 *
 * start_counts     本次单程起点位置
 * target_counts    本次单程最终位置
 * velocity_mm_s    最大速度
 * acceleration_mm_s2   最大加速度
 * jerk_mm_s3           最大 Jerk
 *
 * 返回值：
 * ETHERCAT_MOTION_OK：规划成功。
 * ETHERCAT_MOTION_ERR_LIMIT：计算结果无效或超出范围。
 */
static int motion_trajectory_start(int32_t start_counts,
                                   int32_t target_counts,
                                   float velocity_mm_s,
                                   float acceleration_mm_s2,
                                   float jerk_mm_s3) {
    /**
    *| 段 | Jerk符号 | 作用 |
        |---|---:|---|
        | 1 | `+1` | 加速度从0增大 |
        | 2 | `0` | 保持恒加速度 |
        | 3 | `-1` | 加速度下降到0 |
        | 4 | `0` | 保持匀速 |
        | 5 | `-1` | 进入负加速度 |
        | 6 | `0` | 保持恒减速度 |
        | 7 | `+1` | 负加速度回到0 |
     */
    static const float jerk_sign[MOTION_S7_SEGMENT_COUNT] = {
        1.0f, 0.0f, -1.0f, 0.0f, -1.0f, 0.0f, 1.0f
    };
    int64_t delta_64; //目标位置减起点位置
    float distance_counts; //运动距离的绝对值，单位 counts。

    /* 输入的 mm/s、mm/s²、mm/s³ 转换成编码器单位。*/
    float max_velocity_counts_s; //mm/s
    float max_acceleration_counts_s2; //mm/s²
    float max_jerk_counts_s3; // mm/s³

    // 判断短距离轨迹能否达到最大加速度所需的速度和距离临界值。
    float acceleration_transition_velocity;
    float acceleration_transition_distance;

    float time_jerk; //Jerk段时间，即加速度正在变化的时间
    float time_constant_acceleration; //恒加速度段时间，即加速度保持不变的时间
    float time_constant_velocity; //匀速段时间，即速度保持不变的时间
    float velocity_profile_distance; //达到指定最大速度，最少需要多少距离
    float segment_duration[MOTION_S7_SEGMENT_COUNT]; //保存七段各自的持续时间
    float direction; //+1：正方向;-1：反方向

    // 积分过程中保存每一段开始时的完整运动状态
    float position;
    float velocity;
    float acceleration;
    float start_time;
    float duration;

    float jerk; //当前段的 Jerk
    float duration2; //时间平方
    float duration3; //时间立方

    // 把轨迹总时间换算成2ms PDO周期数
    float cycles_float;
    uint32_t total_cycles;
    uint32_t i;

    // 计算有方向的位置差
    delta_64 = (int64_t) target_counts - (int64_t) start_counts;

    // 起点等于终点
    if (delta_64 == 0) {
        s_trajectory.running = 0U;
        s_trajectory.waiting_actual = 1U;
        s_trajectory.actual_stable_cycles = 0U;
        s_trajectory.start_counts = start_counts;
        s_trajectory.target_counts = target_counts;
        s_trajectory.command_counts = target_counts;
        s_trajectory.total_time_s = 0.0f;
        s_control.command_counts = target_counts;
        s_control.target_counts = target_counts;
        return ETHERCAT_MOTION_OK;
    }

    // 提取方向和绝对距离
    // 根据位置差确定运动方向
    direction = (delta_64 > 0) ? 1.0f : -1.0f;
    // 把距离转换为正数
    distance_counts = (delta_64 > 0) ? (float) delta_64 : (float) -delta_64;
    // mm/s 转换为 counts/s
    max_velocity_counts_s = velocity_mm_s * motion_counts_per_mm_get();
    // mm/s² 转换为 counts/s²
    max_acceleration_counts_s2 = acceleration_mm_s2 * motion_counts_per_mm_get();
    // mm/s³ 转换为 counts/s³
    max_jerk_counts_s3 = jerk_mm_s3 * motion_counts_per_mm_get();

    // 检查
    if ((!motion_float_is_positive(max_velocity_counts_s)) ||
        (!motion_float_is_positive(max_acceleration_counts_s2)) ||
        (!motion_float_is_positive(max_jerk_counts_s3))) {
        return ETHERCAT_MOTION_ERR_LIMIT;
    }

    /* 达到最大加速度时的速度和最短位移，用于区分短距离轨迹类型。 */
    acceleration_transition_velocity =
            (max_acceleration_counts_s2 *
             max_acceleration_counts_s2) /
            max_jerk_counts_s3;
    acceleration_transition_distance =
            (2.0f * max_acceleration_counts_s2 *
             max_acceleration_counts_s2 *
             max_acceleration_counts_s2) /
            (max_jerk_counts_s3 * max_jerk_counts_s3);

    if ((!motion_float_is_positive(acceleration_transition_velocity)) ||
        (!motion_float_is_positive(acceleration_transition_distance))) {
        return ETHERCAT_MOTION_ERR_LIMIT;
    }

    /* 先计算在速度上限处所需的加速段时间。 */
    if (max_velocity_counts_s <= acceleration_transition_velocity) {
        /*
         * 如果当前设定最大速度 小于 达到最大加速度时的速度，因此可以说明达到最大速度之前，加速度还来不及达到最大加速度。
         * 结论：不存在恒加速段和恒减速段。即：2段和6段
         */
        // V = J × Tj² => Tj = √(V/J)  => 获得Tj Jerk段运动时间
        time_jerk = motion_positive_sqrt(max_velocity_counts_s / max_jerk_counts_s3);
        // 没有恒加速度段，即第2段和第6段持续时间为0。
        time_constant_acceleration = 0.0f;
    } else {
        /*
         *  说明可以先达到最大加速度，然后继续保持恒加速度。
         */
        // A = J × Tj => Tj = A/J => 获得Tj Jerk段运动时间
        time_jerk = max_acceleration_counts_s2 / max_jerk_counts_s3;
        // 计算达到最大速度还需要保持多长时间的恒加速度
        time_constant_acceleration = max_velocity_counts_s / max_acceleration_counts_s2 - time_jerk;
    }

    velocity_profile_distance = max_velocity_counts_s * (2.0f * time_jerk + time_constant_acceleration);

    if (distance_counts >= velocity_profile_distance) {
        /* 距离足够：能够达到给定最大速度，第4段存在匀速时间。 */
        time_constant_velocity = (distance_counts - velocity_profile_distance) / max_velocity_counts_s;
    } else if ((max_velocity_counts_s >= acceleration_transition_velocity) &&
               (distance_counts >= acceleration_transition_distance)) {
        /* 能达到最大加速度，但距离不足以达到最大速度。 */
        time_jerk = max_acceleration_counts_s2 / max_jerk_counts_s3;
        /*
         * Ta = time_constant_acceleration  恒加速度时间，未知量
         * Tj = time_jerk                  每个Jerk段时间，已经算出
         * D  = distance_counts            总运动距离
         * A  = max_acceleration_counts_s2 最大加速度
         * 公式：Ta = (√(Tj² + 4D/A) - 3Tj) / 2
         */
        time_constant_acceleration =
        (motion_positive_sqrt(
             time_jerk * time_jerk +
             4.0f * distance_counts /
             max_acceleration_counts_s2) -
         3.0f * time_jerk) / 2.0f;
        // 防止浮点误差导致一个很小的负数
        if (time_constant_acceleration < 0.0f) {
            time_constant_acceleration = 0.0f;
        }
        // 第4段匀速时间为0，因为距离不足以达到最大速度
        time_constant_velocity = 0.0f;
    } else {
        /* 极短距离：只使用Jerk段，达不到给定最大加速度和最大速度。 */
        /*
         * 根据D = 2 × J × Tj³
         * Tj = ∛(D / 2J)
         */
        time_jerk = motion_positive_cbrt(distance_counts / (2.0f * max_jerk_counts_s3));
        time_constant_acceleration = 0.0f;
        time_constant_velocity = 0.0f;
    }

    if ((!motion_float_is_positive(time_jerk)) ||
        (!motion_float_is_finite(time_constant_acceleration)) ||
        (!motion_float_is_finite(time_constant_velocity)) ||
        (time_constant_acceleration < 0.0f) ||
        (time_constant_velocity < 0.0f)) {
        return ETHERCAT_MOTION_ERR_LIMIT;
    }

    /*
     * 填写七段持续时间
     * 第1段时间 = 第3段 = 第5段 = 第7段
     * 第2段时间 = 第6段
     * 第4段是匀速时间
     */
    segment_duration[0] = time_jerk;
    segment_duration[1] = time_constant_acceleration;
    segment_duration[2] = time_jerk;
    segment_duration[3] = time_constant_velocity;
    segment_duration[4] = time_jerk;
    segment_duration[5] = time_constant_acceleration;
    segment_duration[6] = time_jerk;

    /* 从x=0、v=0、a=0积分，保存每一段开始时的完整运动状态。 */
    position = 0.0f; // 初始位置
    velocity = 0.0f; // 初始速度
    acceleration = 0.0f; //初始加速度
    start_time = 0.0f; //初始时间

    for (i = 0U; i < MOTION_S7_SEGMENT_COUNT; i++) {
        // 取得本段持续时间
        duration = segment_duration[i];
        /**
         *计算本段带方向的实际 Jerk
         *正方向时：+J, 0, -J, 0, -J, 0, +J
         *反方向时：-J, 0, +J, 0, +J, 0, -J
         */
        jerk = direction * jerk_sign[i] * max_jerk_counts_s3;

        s_trajectory.segment[i].start_time_s = start_time;
        s_trajectory.segment[i].duration_s = duration;
        s_trajectory.segment[i].start_position_counts = position;
        s_trajectory.segment[i].start_velocity_counts_s = velocity;
        s_trajectory.segment[i].start_acceleration_counts_s2 = acceleration;
        s_trajectory.segment[i].jerk_counts_s3 = jerk;

        duration2 = duration * duration;
        duration3 = duration2 * duration;
        // x末 = x初 + v初·t + 1/2·a初·t² + 1/6·J·t³
        position += velocity * duration +
                0.5f * acceleration * duration2 +
                jerk * duration3 / 6.0f;
        // v末 = v初 + a初·t + 1/2·J·t²
        velocity += acceleration * duration +
                0.5f * jerk * duration2;
        // a末 = a初 + J·t
        acceleration += jerk * duration;
        start_time += duration; //累加时间
    }

    if (!motion_float_is_positive(start_time)) {
        return ETHERCAT_MOTION_ERR_LIMIT;
    }
    // 总时间转换为 PDO 周期数
    cycles_float = start_time / MOTION_PDO_PERIOD_S;
    if ((!motion_float_is_finite(cycles_float)) ||
        (cycles_float >= (float) UINT32_MAX)) {
        return ETHERCAT_MOTION_ERR_LIMIT;
    }
    // 先截断成整数
    total_cycles = (uint32_t) cycles_float;
    // 如果截断后的周期不足以覆盖总时间，就加1，相当于向上取整
    if (((float) total_cycles * MOTION_PDO_PERIOD_S) < start_time) {
        total_cycles++;
    }
    // 确保有效运动至少执行一个 PDO 周期
    if (total_cycles == 0U) {
        total_cycles = 1U;
    }

    // 更新
    s_trajectory.running = 1U; //后续可以开始每2ms生成位置点
    s_trajectory.waiting_actual = 0U; //还没有进入实际位置到位确认阶段
    s_trajectory.cycle_index = 0U; //轨迹从第0周期开始
    s_trajectory.total_cycles = total_cycles; //保存总周期数
    s_trajectory.actual_stable_cycles = 0U; //清零实际位置稳定计数
    s_trajectory.start_counts = start_counts; // 绝对起点
    s_trajectory.target_counts = target_counts; //  绝对终点
    s_trajectory.command_counts = start_counts;
    s_trajectory.total_time_s = start_time; //保存七段总理论时间

    s_control.command_counts = start_counts;
    s_control.target_counts = target_counts;
    return ETHERCAT_MOTION_OK;
}

/* 【七段式-5开始】下面按2ms PDO周期计算七段式的下一目标位置。 */
/*
 * 每个PDO周期按恒定Jerk运动方程计算目标位置：
 * x=x₀ + v₀t + 1/2·a₀t² + 1/6·Jt³。
 * 七段目标生成结束后，继续等待实际位置连续稳定到位。
 */
static int motion_segment_process(void) {
    const motion_s7_segment_t *segment; //当前七段运行轨迹
    float elapsed_time; //整条轨迹已经运行的时间，单位s：已运行周期数*0.002
    float segment_time; //记录当前轨迹段后已经经过的时间点
    float segment_end_time; //  某个轨迹的结束时间
    /* 用于判断当前时刻属于七段轨迹中的哪一段  */
    float segment_time2; //segment_time² = t²
    float segment_time3; //segment_time³ = t³
    float relative_position; //保存相对于本次运动起点的位移。 单位为编码器 counts
    float command_float; //保存浮点形式的绝对命令位置
    int64_t actual_error; //保存实际位置与目标位置之间的误差。
    uint32_t segment_index; //保存当前位于七段 S 曲线中的第几段。:0~6
    uint32_t i;

    // 如果S曲线还没有走完，就计算本周期的新位置指令。
    if (s_trajectory.running) {
        // 检查当前周期序号是否还没有达到总周期数。
        if (s_trajectory.cycle_index < s_trajectory.total_cycles) {
            s_trajectory.cycle_index++;
        }
        // 计算整条轨迹已经运行的时间
        elapsed_time = (float) s_trajectory.cycle_index * MOTION_PDO_PERIOD_S;
        if (elapsed_time > s_trajectory.total_time_s) {
            // 最后一个 PDO 周期可能因为向上取整而超过理论轨迹时间，所以这里进行限幅。
            elapsed_time = s_trajectory.total_time_s;
        }

        /* 跳过持续时间为0的退化段，定位当前有效时间段。 */
        segment_index = MOTION_S7_SEGMENT_COUNT - 1U;
        for (i = 0U; i < MOTION_S7_SEGMENT_COUNT; i++) {
            // 计算当前被检查轨迹段的结束时间。
            segment_end_time = s_trajectory.segment[i].start_time_s + s_trajectory.segment[i].duration_s;
            /**
             *同时满足两个条件，就说明找到了当前轨迹段：
             *当前段持续时间大于0。
             *整条轨迹的已运行时间没有超过当前段的结束时间。
             */
            if ((s_trajectory.segment[i].duration_s > 0.0f) && (elapsed_time <= segment_end_time)) {
                segment_index = i;
                break;
            }
        }
        // 计算当前段内部时间
        segment = &s_trajectory.segment[segment_index]; //获取当前点的指针
        segment_time = elapsed_time - segment->start_time_s;
        // 如果因为浮点误差得到负数，将它限制为0。
        if (segment_time < 0.0f) {
            segment_time = 0.0f;
        } else if (segment_time > segment->duration_s) {
            // 如果段内时间超过当前段持续时间，就限制在当前段的结束时刻。
            segment_time = segment->duration_s;
        }

        segment_time2 = segment_time * segment_time;
        segment_time3 = segment_time2 * segment_time;
        /**
         * x = x₀ + v₀t + 1/2·a₀t² + 1/6·Jt³
         * x₀：当前段开始时的相对位置。
         * v₀：当前段开始时的速度。
         * a₀：当前段开始时的加速度。
         * J：当前段恒定的 Jerk。
         * t：进入当前段后经过的时间。
         * x：本周期相对于整条轨迹起点的位移。
         */
        relative_position =
                segment->start_position_counts +
                segment->start_velocity_counts_s * segment_time +
                0.5f * segment->start_acceleration_counts_s2 *
                segment_time2 +
                segment->jerk_counts_s3 * segment_time3 / 6.0f;
        command_float = (float) s_trajectory.start_counts + relative_position;

        /* 抑制浮点积分误差造成的起点/终点轻微越界。 */
        if (s_trajectory.target_counts >= s_trajectory.start_counts) {
            //正方向运动
            if (command_float < (float) s_trajectory.start_counts) {
                // 计算位置小于起点时，强制等于起点。
                command_float = (float) s_trajectory.start_counts;
            } else if (command_float > (float) s_trajectory.target_counts) {
                // 计算位置超过终点时，强制等于终点。
                command_float = (float) s_trajectory.target_counts;
            }
        } else {
            //反方向运动
            if (command_float > (float) s_trajectory.start_counts) {
                command_float = (float) s_trajectory.start_counts;
            } else if (command_float < (float) s_trajectory.target_counts) {
                command_float = (float) s_trajectory.target_counts;
            }
        }
        // 浮点位置四舍五入成整数
        if (command_float >= 0.0f) {
            s_trajectory.command_counts = (int32_t) (command_float + 0.5f);
        } else {
            s_trajectory.command_counts = (int32_t) (command_float - 0.5f);
        }
        // 判断指令轨迹是否生成结束
        // 如果当前周期已经达到总周期数，说明整条七段式目标位置已经生成结束。
        if (s_trajectory.cycle_index >= s_trajectory.total_cycles) {
            // 最后一个周期不再依赖浮点计算结果，直接强制输出精确目标位置
            s_trajectory.command_counts = s_trajectory.target_counts;
            s_trajectory.running = 0U; //标记“不再继续生成 S 曲线位置”。
            s_trajectory.waiting_actual = 1U; //进入“等待实际位置稳定到位”阶段。
            s_trajectory.actual_stable_cycles = 0U;
        }
        // 把本周期计算出的轨迹位置写入运动控制状态
        s_control.command_counts = s_trajectory.command_counts;
    }

    // 判断是否进入到位确认阶段
    if (!s_trajectory.waiting_actual) {
        return MOTION_SEGMENT_RUNNING;
    }

    // 计算实际位置误差绝对值
    actual_error = (int64_t) input1s->CurrentPosition - (int64_t) s_trajectory.target_counts;

    if (actual_error < 0) {
        actual_error = -actual_error;
    }
    // 连续稳定到位判断
    if (actual_error <= MOTION_POSITION_TOLERANCE_COUNTS) {
        // 实际位置每连续一次落入误差范围，稳定计数加1。
        if (++s_trajectory.actual_stable_cycles >= MOTION_POSITION_STABLE_CYCLES) {
            // 连续稳定次数满足要求，退出实际位置等待状态。
            s_trajectory.waiting_actual = 0U;
            // 报告当前运动段已经完成。
            return MOTION_SEGMENT_DONE;
        }
    } else {
        // 只要有一个周期的位置误差超过500 counts，就把连续稳定计数清零。
        s_trajectory.actual_stable_cycles = 0U;
    }
    // S 曲线仍在生成 或者 S 曲线已经生成完成，但实际位置还没连续稳定10个周期
    return MOTION_SEGMENT_RUNNING;
}

/* 【七段式-5结束】每周期七段式目标位置计算到此结束。 */

/*
 * 启动绝对、相对或有限距离点动运动。
 * request中保存模式、位置、最大速度、最大加速度和最大Jerk；成功后置busy=1。
 */
static int motion_single_start(const motion_request_t *request) {
    int32_t target_counts;
    int result;

    if (request->mode == ETHERCAT_MOTION_MODE_MOVE_ABS) {
        result = motion_position_target_get(
            ETHERCAT_MOTION_MODE_MOVE_ABS,
            request->absolute_reference,
            request->position_mm,
            &target_counts);
    } else if ((request->mode == ETHERCAT_MOTION_MODE_MOVE_REL) ||
               (request->mode == ETHERCAT_MOTION_MODE_JOG)) {
        result = motion_position_target_get(
            request->mode,
            request->absolute_reference,
            request->position_mm,
            &target_counts);
    } else {
        /* HOME命令在请求区使用MOVE_ABS并把位置固定为0。 */
        return ETHERCAT_MOTION_ERR_PARAM;
    }

    if (result != ETHERCAT_MOTION_OK) {
        return result;
    }

    result = motion_trajectory_start(
        s_control.command_counts,
        target_counts,
        request->velocity_mm_s,
        request->acceleration_mm_s2,
        request->jerk_mm_s3);

    if (result == ETHERCAT_MOTION_OK) {
        s_control.mode = request->mode;
        s_control.busy = 1U;
        s_control.done = 0U;
        s_control.error = 0U;
        s_control.paused = 0U;
    }

    return result;
}

/*
 * 启动往返运动的一个单程。
 * 起点使用当前命令位置，终点由target_counts指定，正反向复用同一套S形算法。
 */
static int motion_recip_move_start(int32_t target_counts) {
    return motion_trajectory_start(
        s_control.command_counts,
        target_counts,
        s_recip.velocity_mm_s,
        s_recip.acceleration_mm_s2,
        s_recip.jerk_mm_s3);
}

/*
 * 接收往返命令并启动第一段运动。
 * 往返起点取接收命令时的实际位置，终点等于起点加offset_mm换算后的counts。
 */
static int motion_recip_start_internal(const motion_request_t *request) {
    int32_t offset_counts;
    int64_t end_64;
    int result;

    result = motion_mm_to_counts(request->position_mm,
                                 &offset_counts);
    if (result != ETHERCAT_MOTION_OK) {
        return result;
    }

    if (offset_counts == 0) {
        return ETHERCAT_MOTION_ERR_PARAM;
    }

    end_64 = (int64_t) input1s->CurrentPosition +
             (int64_t) offset_counts;

    if ((end_64 > INT32_MAX) || (end_64 < INT32_MIN)) {
        return ETHERCAT_MOTION_ERR_LIMIT;
    }

    s_recip.active = 1U;
    s_recip.stage = RECIP_STAGE_MOVE_OUT;
    s_recip.start_counts = input1s->CurrentPosition;
    s_recip.end_counts = (int32_t) end_64;
    s_recip.velocity_mm_s = request->velocity_mm_s;
    s_recip.acceleration_mm_s2 = request->acceleration_mm_s2;
    s_recip.jerk_mm_s3 = request->jerk_mm_s3;
    s_recip.first_interval_cycles =
            MOTION_MS_TO_CYCLES(request->first_interval_ms);
    s_recip.second_interval_cycles =
            MOTION_MS_TO_CYCLES(request->second_interval_ms);
    s_recip.wait_cycles = 0U;
    s_recip.repeat_count = request->repeat_count;
    s_recip.completed_count = 0U;

    result = motion_recip_move_start(s_recip.end_counts);
    if (result != ETHERCAT_MOTION_OK) {
        s_recip.active = 0U;
        s_recip.stage = RECIP_STAGE_IDLE;
        return result;
    }

    s_control.mode = ETHERCAT_MOTION_MODE_RECIP;
    s_control.busy = 1U;
    s_control.done = 0U;
    s_control.error = 0U;
    s_control.paused = 0U;
    return ETHERCAT_MOTION_OK;
}

/*
 * 往返状态机，所有单程都复用同一套S形轨迹。
 * 状态顺序为：移向终点→终点等待→返回起点→起点等待→下一次往返。
 */
static void motion_recip_process(void) {
    /**
     * MOTION_SEGMENT_RUNNING：单程还没完成。
     * MOTION_SEGMENT_DONE：单程已经完成并且实际位置稳定到位。
     * MOTION_SEGMENT_ERROR：预留的错误结果。
     */
    int segment_result; // 保存单程轨迹的运行结果
    int result; //保存启动下一段轨迹时的返回值。

    // 根据当前的往返阶段，选择这次 PDO 周期需要执行的操作。
    switch (s_recip.stage) {
        case RECIP_STAGE_MOVE_OUT: // 向终点运动
            segment_result = motion_segment_process();
            //判断是否到达终点并稳定
            if (segment_result == MOTION_SEGMENT_DONE) {
                // 检查终点等待时间是否大于零。
                if (s_recip.first_interval_cycles > 0U) {
                    // 把终点等待周期数装入 wait_cycles
                    s_recip.wait_cycles = s_recip.first_interval_cycles;
                    // 状态切换为“终点等待”
                    s_recip.stage = RECIP_STAGE_WAIT_FIRST;
                } else {
                    // 如果不需要在终点等待，就立即创建一条“从当前终点返回起点”的新 S 曲线。
                    result = motion_recip_move_start(s_recip.start_counts);
                    if (result != ETHERCAT_MOTION_OK) {
                        motion_finish_error();
                        return;
                    }
                    // 状态切换为“返回起点”
                    s_recip.stage = RECIP_STAGE_MOVE_BACK;
                }
            }
            break;

        case RECIP_STAGE_WAIT_FIRST: // 在终点等待
            if (s_recip.wait_cycles > 0U) {
                s_recip.wait_cycles--;
            }
            // 当等待计数变为0，创建返回起点的 S 曲线。
            if (s_recip.wait_cycles == 0U) {
                result = motion_recip_move_start(
                    s_recip.start_counts);
                if (result != ETHERCAT_MOTION_OK) {
                    motion_finish_error();
                    return;
                }
                // 切换到“返回起点”状态
                s_recip.stage = RECIP_STAGE_MOVE_BACK;
            }
            break;

        case RECIP_STAGE_MOVE_BACK: //返回起点
            segment_result = motion_segment_process();
            // 判断是否完成一次往返动作
            if (segment_result == MOTION_SEGMENT_DONE) {
                s_recip.completed_count++;

                if ((s_recip.repeat_count != ETHERCAT_MOTION_RECIP_FOREVER) &&
                    (s_recip.completed_count >= s_recip.repeat_count)) {
                    s_recip.active = 0U;
                    s_recip.stage = RECIP_STAGE_IDLE; //整个往返运动结束
                    motion_finish_success();
                    return;
                }
                // 如果还需要继续往返，并且配置了起点等待时间
                if (s_recip.second_interval_cycles > 0U) {
                    s_recip.wait_cycles = s_recip.second_interval_cycles;
                    // 切换为“起点等待”
                    s_recip.stage = RECIP_STAGE_WAIT_SECOND;
                } else {
                    // 创建下一条“起点到终点”的轨迹
                    result = motion_recip_move_start(s_recip.end_counts);
                    if (result != ETHERCAT_MOTION_OK) {
                        //检查
                        motion_finish_error();
                        return;
                    }
                    // “向终点运动”状态
                    s_recip.stage = RECIP_STAGE_MOVE_OUT;
                }
            }
            break;

        case RECIP_STAGE_WAIT_SECOND: //在起点等待
            if (s_recip.wait_cycles > 0U) {
                s_recip.wait_cycles--;
            }
            if (s_recip.wait_cycles == 0U) {
                // 创建下一条“起点到终点”的轨迹
                result = motion_recip_move_start(s_recip.end_counts);
                if (result != ETHERCAT_MOTION_OK) {
                    //检查
                    motion_finish_error();
                    return;
                }
                // “向终点运动”状态
                s_recip.stage = RECIP_STAGE_MOVE_OUT;
            }
            break;

        case RECIP_STAGE_IDLE: //异常状态保护
        default:
            motion_finish_error();
            break;
    }
}

/*
 * 取消当前运动并保持取消时的实际位置。
 * 该函数会丢弃轨迹和往返状态，取消后不能继续原运动。
 */
static void motion_abort_apply(void) {
    s_trajectory.running = 0U;
    s_trajectory.waiting_actual = 0U;
    s_recip.active = 0U;
    s_recip.stage = RECIP_STAGE_IDLE;
    s_control.mode = ETHERCAT_MOTION_MODE_STOP;
    s_control.command_counts = input1s->CurrentPosition;
    s_control.target_counts = input1s->CurrentPosition;
    s_control.busy = 0U;
    s_control.done = 1U;
    s_control.error = 0U;
    s_control.paused = 0U;
}

/*
 * 把应用层请求转换为内部运动。
 * 暂停和继续只改变paused；取消会丢弃轨迹；START启动新运动。
 */
static void motion_request_apply(const motion_request_t *request) {
    int result;

    switch (request->action) {
        case MOTION_REQUEST_ACTION_PAUSE:
            if (s_control.busy) {
                s_control.busy = 1U;
                s_control.paused = 1U;
                s_control.done = 0U;
                s_control.error = 0U;
            } else {
                motion_abort_apply();
            }
            return;

        case MOTION_REQUEST_ACTION_CONTINUE:
            if (s_control.busy && s_control.paused) {
                s_control.busy = 1U;
                s_control.paused = 0U;
                s_control.done = 0U;
                s_control.error = 0U;
            }
            return;

        case MOTION_REQUEST_ACTION_ABORT:
            motion_abort_apply();
            return;

        case MOTION_REQUEST_ACTION_START:
            break;

        default:
            motion_finish_error();
            return;
    }

    /*
     * 暂停期间提交START表示放弃原轨迹。
     * 保留command_counts作为新轨迹起点，但清除旧轨迹和往返活动状态。
     */
    if (s_control.paused) {
        s_trajectory.running = 0U;
        s_trajectory.waiting_actual = 0U;
        s_recip.active = 0U;
        s_recip.stage = RECIP_STAGE_IDLE;
        s_control.paused = 0U;
    }

    if (request->mode == ETHERCAT_MOTION_MODE_RECIP) {
        result = motion_recip_start_internal(request);
    } else {
        result = motion_single_start(request);
    }

    if (result != ETHERCAT_MOTION_OK) {
        motion_finish_error();
    }
}

/* 当前命令正常结束：回到IDLE，清除busy并置done。 */
static void motion_finish_success(void) {
    s_control.mode = ETHERCAT_MOTION_MODE_IDLE;
    s_control.busy = 0U;
    s_control.done = 1U;
    s_control.error = 0U;
    s_control.paused = 0U;
}

/* 当前命令异常结束：取消轨迹、保持实际位置并置error。 */
static void motion_finish_error(void) {
    s_trajectory.running = 0U;
    s_trajectory.waiting_actual = 0U;
    s_recip.active = 0U;
    s_recip.stage = RECIP_STAGE_IDLE;
    s_control.mode = ETHERCAT_MOTION_MODE_STOP;
    s_control.command_counts = input1s->CurrentPosition;
    s_control.target_counts = input1s->CurrentPosition;
    s_control.busy = 0U;
    s_control.done = 0U;
    s_control.error = 1U;
    s_control.paused = 0U;
}

/*
 * 后续压力/位置闭环扩展点。
 * 当前应用层开环运行，修正量固定为0，不改变S形轨迹目标。
 * 增加压力采集后，可在这里根据反馈生成小范围位置修正量。
 */
static int32_t motion_closed_loop_correction_get(void) {
    return 0;
}

/*
 * 将开环轨迹目标和未来闭环修正量统一写入RxPDO的0x607A目标位置。
 * 使用int64_t完成加法和限幅，避免int32_t溢出。
 */
static void motion_target_output_write(void) {
    int64_t target_64;

    target_64 = (int64_t) s_control.command_counts +
                (int64_t) motion_closed_loop_correction_get();

    if (target_64 > INT32_MAX) {
        target_64 = INT32_MAX;
    } else if (target_64 < INT32_MIN) {
        target_64 = INT32_MIN;
    }
    // if (current_state == SET_0RIGIN) {
    //     output1s->TargetPos = 0;
    //     ec_receive_processdata(EC_TIMEOUTRET);
    // }else {
    //     output1s->TargetPos = (int32_t) target_64;
    // }
    output1s->TargetPos = (int32_t) target_64;
}

/*
 * 设置编码器、丝杠、传动比和最大转速参数。
 * 所有参数必须为有限正数；正在运动或存在待处理命令时拒绝修改。
 */
int ethercat_motion_motor_params_set(float encoder_counts_per_motor_rev,
                                     float lead_mm_per_screw_rev,
                                     float gear_ratio,
                                     float reducer_ratio,
                                     float max_motor_rpm) {
    float counts_per_mm;
    float max_linear_velocity_mm_s;

    if ((!motion_float_is_positive(
            encoder_counts_per_motor_rev)) ||
        (!motion_float_is_positive(
            lead_mm_per_screw_rev)) ||
        (!motion_float_is_positive(gear_ratio)) ||
        (!motion_float_is_positive(reducer_ratio)) ||
        (!motion_float_is_positive(max_motor_rpm))) {
        return ETHERCAT_MOTION_ERR_PARAM;
    }

    counts_per_mm =
            (encoder_counts_per_motor_rev *
             gear_ratio * reducer_ratio) /
            lead_mm_per_screw_rev;

    max_linear_velocity_mm_s =
            ((max_motor_rpm / 60.0f) /
             (gear_ratio * reducer_ratio)) *
            lead_mm_per_screw_rev;

    if ((!motion_float_is_positive(counts_per_mm)) ||
        (!motion_float_is_positive(max_linear_velocity_mm_s))) {
        return ETHERCAT_MOTION_ERR_LIMIT;
    }

    taskENTER_CRITICAL();

    if (s_control.busy || s_request.pending) {
        taskEXIT_CRITICAL();
        return ETHERCAT_MOTION_ERR_BUSY;
    }

    s_motor_params.encoder_counts_per_motor_rev =
            encoder_counts_per_motor_rev;
    s_motor_params.lead_mm_per_screw_rev =
            lead_mm_per_screw_rev;
    s_motor_params.gear_ratio = gear_ratio;
    s_motor_params.reducer_ratio = reducer_ratio;
    s_motor_params.max_motor_rpm = max_motor_rpm;
    s_motor_params_ready = 1U;

    taskEXIT_CRITICAL();
    return ETHERCAT_MOTION_OK;
}

/*
 * 提交绝对、相对或有限距离点动命令。
 * 函数只负责参数检查和命令入队，返回OK不代表运动已经完成。
 */
int ethercat_motion_command_set(ethercat_motion_mode_t mode,
                                float position_mm,
                                float velocity_mm_s,
                                float acceleration_mm_s2,
                                /* 【七段式-6】新增Jerk入参。 */
                                float jerk_mm_s3) {
    motion_request_t request = {0};
    int result;

    result = motion_request_validate(mode,
                                     velocity_mm_s,
                                     acceleration_mm_s2,
                                     jerk_mm_s3);
    if (result != ETHERCAT_MOTION_OK) {
        return result;
    }

    if ((mode != ETHERCAT_MOTION_MODE_MOVE_ABS) &&
        (mode != ETHERCAT_MOTION_MODE_MOVE_REL) &&
        (mode != ETHERCAT_MOTION_MODE_JOG)) {
        return ETHERCAT_MOTION_ERR_PARAM;
    }

    // 保存参数
    request.action = MOTION_REQUEST_ACTION_START;
    request.mode = mode;
    request.absolute_reference =
            MOTION_ABSOLUTE_REFERENCE_SOFTWARE_ZERO;
    request.position_mm = position_mm;
    request.velocity_mm_s = velocity_mm_s;
    request.acceleration_mm_s2 = acceleration_mm_s2;
    request.jerk_mm_s3 = jerk_mm_s3;

    return motion_request_submit(&request);
}

/*
 * 在临界区读取当前实际位置并设置为软件零点。
 * 仅在运动模块已经取得有效位置且当前没有运动或待处理请求时接受。
 */
int ethercat_motion_software_zero_set(void) {
    int result = ETHERCAT_MOTION_OK;

    taskENTER_CRITICAL();

    if ((input1s == NULL) ||
        (output1s == NULL) ||
        (!s_motion_initialized) ||
        ((input1s->StatusWord & CIA402_SW_MASK) !=
         CIA402_SW_OPERATION_ENABLED) ||
        (input1s->OpModeNow != 8)) {
        result = ETHERCAT_MOTION_ERR_NOT_READY;
    } else if (s_request.pending || s_control.busy) {
        result = ETHERCAT_MOTION_ERR_BUSY;
    } else {
        s_software_zero_counts = input1s->CurrentPosition;
    }

    taskEXIT_CRITICAL();
    return result;
}

/*
 * 返回绝对值编码器计数0。
 * 使用固定回零运动参数，不搜索硬件原点开关。
 */
int ethercat_motion_mechanical_zero_return(void) {
    motion_request_t request = {0};
    int result;

    result = motion_request_validate(
        ETHERCAT_MOTION_MODE_MOVE_ABS,
        ETHERCAT_MOTION_ZERO_RETURN_VELOCITY_MM_S,
        ETHERCAT_MOTION_ZERO_RETURN_ACCELERATION_MM_S2,
        ETHERCAT_MOTION_ZERO_RETURN_JERK_MM_S3);
    if (result != ETHERCAT_MOTION_OK) {
        return result;
    }

    request.action = MOTION_REQUEST_ACTION_START;
    request.mode = ETHERCAT_MOTION_MODE_MOVE_ABS;
    request.absolute_reference =
            MOTION_ABSOLUTE_REFERENCE_ENCODER_ZERO;
    request.position_mm = 0.0f;
    request.velocity_mm_s =
            ETHERCAT_MOTION_ZERO_RETURN_VELOCITY_MM_S;
    request.acceleration_mm_s2 =
            ETHERCAT_MOTION_ZERO_RETURN_ACCELERATION_MM_S2;
    request.jerk_mm_s3 =
            ETHERCAT_MOTION_ZERO_RETURN_JERK_MM_S3;

    return motion_request_submit(&request);
}

/*
 * 返回软件设置的零点。
 * 使用固定回零运动参数，不搜索硬件原点开关。
 */
int ethercat_motion_software_zero_return(void) {
    return ethercat_motion_command_set(
        ETHERCAT_MOTION_MODE_MOVE_ABS,
        0.0f,
        ETHERCAT_MOTION_ZERO_RETURN_VELOCITY_MM_S,
        ETHERCAT_MOTION_ZERO_RETURN_ACCELERATION_MM_S2,
        ETHERCAT_MOTION_ZERO_RETURN_JERK_MM_S3);
}

/*
 * 提交往返运动命令。
 * offset_mm是终点相对接收命令时实际位置的偏移；repeat_count为0表示持续往返。
 */
int ethercat_motion_recip_start(float offset_mm,
                                float velocity_mm_s,
                                float acceleration_mm_s2,
                                /* 【七段式-8】新增Jerk入参。 */
                                float jerk_mm_s3,
                                uint32_t first_interval_ms,
                                uint32_t second_interval_ms,
                                uint32_t repeat_count) {
    motion_request_t request = {0};
    int result;

    result = motion_request_validate(
        ETHERCAT_MOTION_MODE_RECIP,
        velocity_mm_s,
        acceleration_mm_s2,
        jerk_mm_s3);
    if (result != ETHERCAT_MOTION_OK) {
        return result;
    }

    request.mode = ETHERCAT_MOTION_MODE_RECIP;
    request.position_mm = offset_mm;
    request.velocity_mm_s = velocity_mm_s;
    request.acceleration_mm_s2 = acceleration_mm_s2;
    request.jerk_mm_s3 = jerk_mm_s3;
    request.first_interval_ms = first_interval_ms;
    request.second_interval_ms = second_interval_ms;
    request.repeat_count = repeat_count;

    return motion_request_submit(&request);
}

/*
 * 提交暂停请求。
 * PDO任务收到请求后冻结当前轨迹状态，并保持最后一个规划位置。
 */
int ethercat_motion_stop(void) {
    int result = ETHERCAT_MOTION_OK;

    taskENTER_CRITICAL();

    if (s_request.pending) {
        result = ETHERCAT_MOTION_ERR_BUSY;
    } else if (!s_control.busy || s_control.paused) {
        result = ETHERCAT_MOTION_ERR_STATE;
    } else {
        s_request.action = MOTION_REQUEST_ACTION_PAUSE;
        s_request.mode = ETHERCAT_MOTION_MODE_STOP;
        s_request.pending = 1U;
    }

    taskEXIT_CRITICAL();
    return result;
}

/*
 * 提交继续请求。
 * 仅当前运动未完成、已经暂停且没有其他待处理请求时接受。
 * 返回OK表示请求已入队，实际继续由PDO任务完成。
 */
int ethercat_motion_continue(void) {
    int result = ETHERCAT_MOTION_OK;

    taskENTER_CRITICAL();

    if (s_request.pending) {
        result = ETHERCAT_MOTION_ERR_BUSY;
    } else if ((!s_control.busy) || (!s_control.paused)) {
        result = ETHERCAT_MOTION_ERR_STATE;
    } else {
        s_request.action = MOTION_REQUEST_ACTION_CONTINUE;
        s_request.mode = ETHERCAT_MOTION_MODE_STOP;
        s_request.pending = 1U;
    }

    taskEXIT_CRITICAL();
    return result;
}

/*
 * 提交取消请求。
 * PDO任务收到请求后丢弃轨迹，并把实际位置设置为保持位置。
 */
int ethercat_motion_abort(void) {
    int result = ETHERCAT_MOTION_OK;

    taskENTER_CRITICAL();

    if ((!s_request.pending) && (!s_control.busy)) {
        result = ETHERCAT_MOTION_ERR_STATE;
    } else {
        s_request.action = MOTION_REQUEST_ACTION_ABORT;
        s_request.mode = ETHERCAT_MOTION_MODE_STOP;
        s_request.pending = 1U;
    }

    taskEXIT_CRITICAL();
    return result;
}

/*
 * 2ms PDO周期运动调度入口。
 * 设置CSP模式、接收应用命令、推进S形轨迹，并写出下一周期TargetPos。
 */
void ethercat_motion_process(void) {
    motion_request_t request;
    int result;

    if ((input1s == NULL) || (output1s == NULL)) {
        return;
    }

    /* 当前所有运动都使用CSP模式。 */
    output1s->OpModeSet = 8;
    output1s->TargetVelocity = 0;

    /*
     * 调度器只在伺服已使能且模式显示为CSP时推进轨迹。
     * 模式暂未切换成功时保持当前指令位置，不消耗S曲线周期。
     */
    if (((input1s->StatusWord & CIA402_SW_MASK) !=
         CIA402_SW_OPERATION_ENABLED) || (input1s->OpModeNow != 8)) {
        /* 首次运行时把命令位置对齐实际位置，防止目标位置从0突跳。 */
        // 情况一：第一次调用时伺服尚未就绪
        if (!s_motion_initialized) {
            s_control.command_counts = input1s->CurrentPosition;
            s_control.target_counts = input1s->CurrentPosition;
            s_motion_initialized = 1U;
        }
        motion_target_output_write();
        return;
    }

    /* 首次运行时把命令位置对齐实际位置，防止目标位置从0突跳。 */
    // 情况二：第一次调用时伺服已经就绪
    if (!s_motion_initialized) {
        s_control.command_counts = input1s->CurrentPosition;
        s_control.target_counts = input1s->CurrentPosition;
        s_motion_initialized = 1U;
    }

    if (motion_request_fetch(&request)) {
        motion_request_apply(&request);
    }

    if (s_control.busy && !s_control.paused) {
        if (s_control.mode == ETHERCAT_MOTION_MODE_RECIP) {
            motion_recip_process();
        } else {
            result = motion_segment_process();

            if (result == MOTION_SEGMENT_DONE) {
                motion_finish_success();
            } else if (result == MOTION_SEGMENT_ERROR) {
                motion_finish_error();
            }
        }
    }

    motion_target_output_write();
}

/*
 * 获取运动状态快照。
 * 临界区保证应用任务不会读到PDO任务更新到一半的数据。
 */
void ethercat_motion_status_get(ethercat_motion_status_t *status) {
    if (status == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    status->mode = s_control.mode;
    status->busy = s_control.busy;
    status->done = s_control.done;
    status->error = s_control.error;
    status->paused = s_control.paused;
    status->command_position_counts = s_control.command_counts;
    status->target_position_counts = s_control.target_counts;
    status->actual_position_counts = (input1s != NULL) ? input1s->CurrentPosition : 0;
    status->recip_completed_count = s_recip.completed_count;
    taskEXIT_CRITICAL();
}

int ethercat_motion_position_sync(void) {
    int32_t position_counts;

    if ((input1s == NULL) || (output1s == NULL)) {
        return ETHERCAT_MOTION_ERR_NOT_READY;
    }

    taskENTER_CRITICAL();

    position_counts = input1s->CurrentPosition;

    /* 清除还没有被PDO任务取走的旧运动命令。 */
    s_request.pending = 0U;

    /* 停止旧轨迹，并把内部目标同步到新的0x6064位置。 */
    motion_abort_apply();

    /*
     * 驱动器原点已经改变，因此同步软件坐标原点，
     * 防止后续MOVE_ABS仍使用旧的软件零点偏移。
     */
    s_software_zero_counts = position_counts;
    s_motion_initialized = 1U;

    /* 下一帧RxPDO直接保持新的当前位置。 */
    output1s->TargetPos = position_counts;

    taskEXIT_CRITICAL();

    return ETHERCAT_MOTION_OK;
}


float get_motor_position_mm(void) {
    float result;
    int32_t position_counts;
    float counts_per_mm;

    taskENTER_CRITICAL();

    if ((input1s == NULL) || (s_motor_params_ready == 0U)) {
        taskEXIT_CRITICAL();
        return ETHERCAT_MOTION_ERR_NOT_READY;
    }
    /*
     * 在临界区内取得同一个时刻的位置和机械参数快照，
     * 避免PDO任务更新位置时应用任务读取到不一致的数据。
     */
    position_counts = input1s->CurrentPosition;
    counts_per_mm = motion_counts_per_mm_get();

    taskEXIT_CRITICAL();

    if (position_counts <= 100) {
        return 0;
    }
    if (!motion_float_is_positive(counts_per_mm)) {
        return ETHERCAT_MOTION_ERR_LIMIT;
    }
    result = (float) position_counts / counts_per_mm;

    return result;
}

