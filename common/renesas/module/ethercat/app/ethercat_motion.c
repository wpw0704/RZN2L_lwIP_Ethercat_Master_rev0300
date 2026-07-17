#include "ethercat_motion.h"

#include "ethercat_master.h"
#include "FreeRTOS.h"
#include "task.h"

#include <float.h>
#include <limits.h>
#include <stddef.h>

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
    MOTION_SEGMENT_DONE = 1,    /* 当前运动段已完成并通过到位稳定判断。 */
    MOTION_SEGMENT_ERROR = -1,  /* 当前运动段执行失败。 */
} motion_segment_result_t;

/* 往返运动内部阶段。 */
typedef enum {
    RECIP_STAGE_IDLE = 0,    /* 往返状态机空闲。 */
    RECIP_STAGE_MOVE_OUT,    /* 从起点向偏移终点运动。 */
    RECIP_STAGE_WAIT_FIRST,  /* 到达偏移终点后的第一次等待。 */
    RECIP_STAGE_MOVE_BACK,   /* 从偏移终点返回起点。 */
    RECIP_STAGE_WAIT_SECOND, /* 返回起点后的第二次等待。 */
} recip_stage_t;

/* 上位机命令先写入单槽请求区，再由PDO周期任务统一接收。 */
typedef struct {
    uint8_t pending;                  /* 1：存在尚未被PDO任务取走的命令。 */
    ethercat_motion_mode_t mode;      /* 本次请求的运动模式。 */
    float position_mm;                /* 绝对位置、相对偏移或往返偏移，单位mm。 */
    float velocity_mm_s;              /* 允许的最大直线速度，单位mm/s。 */
    float acceleration_mm_s2;         /* 允许的最大直线加速度，单位mm/s²。 */
    /* 【七段式-2】命令区增加用户可设置的Jerk上限。 */
    float jerk_mm_s3;                  /* 允许的最大加加速度，单位mm/s³。 */
    uint32_t first_interval_ms;        /* 到达往返偏移终点后的等待时间，单位ms。 */
    uint32_t second_interval_ms;       /* 返回往返起点后的等待时间，单位ms。 */
    uint32_t repeat_count;             /* 完整往返次数，0表示持续往返。 */
} motion_request_t;

/* 【七段式-3开始】新增七段轨迹的数据结构。 */
/* 七段式轨迹中单个恒定Jerk时间段的起始状态。 */
typedef struct {
    float start_time_s;                /* 本段相对整条轨迹的开始时间，单位s。 */
    float duration_s;                  /* 本段持续时间，单位s；可为0。 */
    float start_position_counts;       /* 本段开始时相对起点的位移，单位counts。 */
    float start_velocity_counts_s;     /* 本段开始速度，单位counts/s。 */
    float start_acceleration_counts_s2; /* 本段开始加速度，单位counts/s²。 */
    float jerk_counts_s3;              /* 本段恒定Jerk，单位counts/s³。 */
} motion_s7_segment_t;

/* 经典七段式Jerk受限S形轨迹运行数据。 */
typedef struct {
    uint8_t running;                  /* 1：七段式目标位置仍在生成。 */
    uint8_t waiting_actual;           /* 1：目标已生成完，正在等待实际位置稳定到位。 */
    uint32_t cycle_index;              /* 当前已经执行的S曲线PDO周期序号。 */
    uint32_t total_cycles;             /* 当前运动段总PDO周期数。 */
    uint32_t actual_stable_cycles;     /* 实际位置连续落入误差范围的周期数。 */
    int32_t start_counts;              /* 当前运动段起点，单位编码器counts。 */
    int32_t target_counts;             /* 当前运动段最终终点，单位编码器counts。 */
    int32_t command_counts;            /* 当前周期计算出的CSP位置指令，单位counts。 */
    float total_time_s;                 /* 七个时间段的总运动时间，单位s。 */
    motion_s7_segment_t segment[MOTION_S7_SEGMENT_COUNT]; /* 七段起始状态。 */
} motion_trajectory_t;
/* 【七段式-3结束】七段轨迹的数据结构到此结束。 */

/* 运动调度状态。 */
typedef struct {
    ethercat_motion_mode_t mode; /* 当前运动模式。 */
    uint8_t busy;                /* 1：命令尚未完整结束。 */
    uint8_t done;                /* 1：上一条命令已正常完成或已执行停止。 */
    uint8_t error;               /* 1：运动执行过程中发生错误。 */
    int32_t command_counts;      /* 当前输出的S曲线位置指令，单位counts。 */
    int32_t target_counts;       /* 当前运动段最终目标位置，单位counts。 */
} motion_control_t;

/* 往返运动状态。 */
typedef struct {
    uint8_t active;                    /* 1：往返状态机已启动。 */
    recip_stage_t stage;               /* 当前正向、返回或等待阶段。 */
    int32_t start_counts;              /* 接收命令时的往返起点，单位counts。 */
    int32_t end_counts;                /* 起点加偏移后的往返终点，单位counts。 */
    float velocity_mm_s;               /* 每个单程的最大速度，单位mm/s。 */
    float acceleration_mm_s2;          /* 每个单程的最大加速度，单位mm/s²。 */
    float jerk_mm_s3;                  /* 每个单程的最大加加速度，单位mm/s³。 */
    uint32_t first_interval_cycles;     /* 偏移终点等待时间换算后的PDO周期数。 */
    uint32_t second_interval_cycles;    /* 起点等待时间换算后的PDO周期数。 */
    uint32_t wait_cycles;               /* 当前等待阶段剩余PDO周期数。 */
    uint32_t repeat_count;              /* 要执行的完整往返次数，0表示持续往返。 */
    uint32_t completed_count;           /* 已完成的“起点-终点-起点”次数。 */
} motion_recip_t;

/* 电机参数只在运动模块内部保存，应用层直接传入各项数值。 */
typedef struct {
    float encoder_counts_per_motor_rev; /* 电机每转编码器计数，单位counts/rev。 */
    float lead_mm_per_screw_rev;        /* 丝杠每转直线位移，单位mm/rev。 */
    float gear_ratio;                   /* 齿轮比，按电机转数/丝杠转数填写。 */
    float reducer_ratio;                /* 减速机比，按输入转数/输出转数填写。 */
    float max_motor_rpm;                /* 电机允许的最大转速，单位r/min。 */
} motion_motor_params_t;

static motion_motor_params_t s_motor_params; /* 当前电机与机械换算参数。 */
static uint8_t s_motor_params_ready;         /* 1：电机机械参数已经设置。 */
static uint8_t s_motion_initialized;         /* 1：命令位置已与首次实际位置对齐。 */
static motion_request_t s_request;           /* 应用任务到PDO任务的单槽命令区。 */
static motion_trajectory_t s_trajectory;     /* 当前S形运动段的运行数据。 */
/* 当前运动调度状态，上电默认空闲且目标位置为0。 */
static motion_control_t s_control = {
    .mode = ETHERCAT_MOTION_MODE_IDLE,
    .busy = 0U,
    .done = 0U,
    .error = 0U,
    .command_counts = 0,
    .target_counts = 0,
};
static motion_recip_t s_recip;               /* 当前往返运动状态机数据。 */

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
    float estimate;
    uint32_t i;

    if (value <= 0.0f) {
        return 0.0f;
    }

    estimate = (value > 1.0f) ? value : 1.0f;

    for (i = 0U; i < 24U; i++) {
        estimate = 0.5f * (estimate + value / estimate);
    }

    return estimate;
}

/*
 * 不依赖libm的正数立方根，用于短距离七段式轨迹的时间计算。
 * 该函数只在接收新运动段时运行，不占用每个PDO周期的实时计算时间。
 */
static float motion_positive_cbrt(float value) {
    float estimate;
    uint32_t i;

    if (value <= 0.0f) {
        return 0.0f;
    }

    /* 先按2倍缩放到立方根附近，避免直接用value作为初值造成溢出。 */
    estimate = 1.0f;
    if (value > 1.0f) {
        while (estimate < value / estimate / estimate) {
            estimate *= 2.0f;
        }
    } else {
        while (estimate > value / estimate / estimate) {
            estimate *= 0.5f;
        }
    }

    for (i = 0U; i < 16U; i++) {
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
    float screw_rps;
    float linear_mm_s;

    screw_rps = (s_motor_params.max_motor_rpm / 60.0f) /
                (s_motor_params.gear_ratio *
                 s_motor_params.reducer_ratio);

    linear_mm_s = screw_rps *
                  s_motor_params.lead_mm_per_screw_rev;

    return linear_mm_s;
}

/*
 * 将毫米位置或偏移量转换为编码器计数，并检查int32_t范围。
 * position_mm可以为负数；转换结果通过counts返回。
 */
static int motion_mm_to_counts(float position_mm, int32_t *counts) {
    float counts_float;

    if ((counts == NULL) ||
        (!motion_float_is_finite(position_mm))) {
        return ETHERCAT_MOTION_ERR_PARAM;
    }

    counts_float = position_mm * motion_counts_per_mm_get();

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
 * MOVE_ABS直接相对软件零点换算；MOVE_REL和JOG在当前实际位置上叠加偏移。
 */
static int motion_position_target_get(ethercat_motion_mode_t mode,
                                      float position_mm,
                                      int32_t *target_counts) {
    int32_t offset_counts;
    int64_t target_64;
    int result;

    if ((input1s == NULL) || (target_counts == NULL)) {
        return ETHERCAT_MOTION_ERR_NOT_READY;
    }

    if (mode == ETHERCAT_MOTION_MODE_MOVE_ABS) {
        return motion_mm_to_counts(position_mm, target_counts);
    }

    if ((mode != ETHERCAT_MOTION_MODE_MOVE_REL) &&
        (mode != ETHERCAT_MOTION_MODE_JOG)) {
        return ETHERCAT_MOTION_ERR_PARAM;
    }

    result = motion_mm_to_counts(position_mm, &offset_counts);
    if (result != ETHERCAT_MOTION_OK) {
        return result;
    }

    target_64 = (int64_t) input1s->CurrentPosition +
                (int64_t) offset_counts;

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
 * 应用任务只写s_request，不直接修改PDO周期状态；已有待处理命令或正在运动时返回BUSY。
 */
static int motion_request_submit(const motion_request_t *request) {
    int result = ETHERCAT_MOTION_OK;

    if (request == NULL) {
        return ETHERCAT_MOTION_ERR_PARAM;
    }

    taskENTER_CRITICAL();

    if (s_request.pending || s_control.busy) {
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
 */
static int motion_trajectory_start(int32_t start_counts,
                                   int32_t target_counts,
                                   float velocity_mm_s,
                                   float acceleration_mm_s2,
                                   float jerk_mm_s3) {
    static const float jerk_sign[MOTION_S7_SEGMENT_COUNT] = {
        1.0f, 0.0f, -1.0f, 0.0f, -1.0f, 0.0f, 1.0f
    };
    int64_t delta_64;
    float distance_counts;
    float max_velocity_counts_s;
    float max_acceleration_counts_s2;
    float max_jerk_counts_s3;
    float acceleration_transition_velocity;
    float acceleration_transition_distance;
    float time_jerk;
    float time_constant_acceleration;
    float time_constant_velocity;
    float velocity_profile_distance;
    float segment_duration[MOTION_S7_SEGMENT_COUNT];
    float direction;
    float position;
    float velocity;
    float acceleration;
    float start_time;
    float duration;
    float jerk;
    float duration2;
    float duration3;
    float cycles_float;
    uint32_t total_cycles;
    uint32_t i;

    delta_64 = (int64_t) target_counts - (int64_t) start_counts;

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

    direction = (delta_64 > 0) ? 1.0f : -1.0f;
    distance_counts = (delta_64 > 0) ?
                      (float) delta_64 :
                      (float) -delta_64;

    max_velocity_counts_s = velocity_mm_s *
                            motion_counts_per_mm_get();
    max_acceleration_counts_s2 = acceleration_mm_s2 *
                                 motion_counts_per_mm_get();
    max_jerk_counts_s3 = jerk_mm_s3 *
                         motion_counts_per_mm_get();

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
        time_jerk = motion_positive_sqrt(
                max_velocity_counts_s / max_jerk_counts_s3);
        time_constant_acceleration = 0.0f;
    } else {
        time_jerk = max_acceleration_counts_s2 /
                    max_jerk_counts_s3;
        time_constant_acceleration =
                max_velocity_counts_s / max_acceleration_counts_s2 -
                time_jerk;
    }

    velocity_profile_distance =
            max_velocity_counts_s *
            (2.0f * time_jerk + time_constant_acceleration);

    if (distance_counts >= velocity_profile_distance) {
        /* 距离足够：能够达到给定最大速度，第4段存在匀速时间。 */
        time_constant_velocity =
                (distance_counts - velocity_profile_distance) /
                max_velocity_counts_s;
    } else if ((max_velocity_counts_s >=
                acceleration_transition_velocity) &&
               (distance_counts >=
                acceleration_transition_distance)) {
        /* 能达到最大加速度，但距离不足以达到最大速度。 */
        time_jerk = max_acceleration_counts_s2 /
                    max_jerk_counts_s3;
        time_constant_acceleration =
                (motion_positive_sqrt(
                    time_jerk * time_jerk +
                    4.0f * distance_counts /
                    max_acceleration_counts_s2) -
                 3.0f * time_jerk) /
                2.0f;
        if (time_constant_acceleration < 0.0f) {
            time_constant_acceleration = 0.0f;
        }
        time_constant_velocity = 0.0f;
    } else {
        /* 极短距离：只使用Jerk段，达不到给定最大加速度和最大速度。 */
        time_jerk = motion_positive_cbrt(
                distance_counts / (2.0f * max_jerk_counts_s3));
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

    segment_duration[0] = time_jerk;
    segment_duration[1] = time_constant_acceleration;
    segment_duration[2] = time_jerk;
    segment_duration[3] = time_constant_velocity;
    segment_duration[4] = time_jerk;
    segment_duration[5] = time_constant_acceleration;
    segment_duration[6] = time_jerk;

    /* 从x=0、v=0、a=0积分，保存每一段开始时的完整运动状态。 */
    position = 0.0f;
    velocity = 0.0f;
    acceleration = 0.0f;
    start_time = 0.0f;

    for (i = 0U; i < MOTION_S7_SEGMENT_COUNT; i++) {
        duration = segment_duration[i];
        jerk = direction * jerk_sign[i] * max_jerk_counts_s3;

        s_trajectory.segment[i].start_time_s = start_time;
        s_trajectory.segment[i].duration_s = duration;
        s_trajectory.segment[i].start_position_counts = position;
        s_trajectory.segment[i].start_velocity_counts_s = velocity;
        s_trajectory.segment[i].start_acceleration_counts_s2 = acceleration;
        s_trajectory.segment[i].jerk_counts_s3 = jerk;

        duration2 = duration * duration;
        duration3 = duration2 * duration;
        position += velocity * duration +
                    0.5f * acceleration * duration2 +
                    jerk * duration3 / 6.0f;
        velocity += acceleration * duration +
                    0.5f * jerk * duration2;
        acceleration += jerk * duration;
        start_time += duration;
    }

    if (!motion_float_is_positive(start_time)) {
        return ETHERCAT_MOTION_ERR_LIMIT;
    }

    cycles_float = start_time / MOTION_PDO_PERIOD_S;
    if ((!motion_float_is_finite(cycles_float)) ||
        (cycles_float >= (float) UINT32_MAX)) {
        return ETHERCAT_MOTION_ERR_LIMIT;
    }

    total_cycles = (uint32_t) cycles_float;
    if (((float) total_cycles * MOTION_PDO_PERIOD_S) < start_time) {
        total_cycles++;
    }
    if (total_cycles == 0U) {
        total_cycles = 1U;
    }

    s_trajectory.running = 1U;
    s_trajectory.waiting_actual = 0U;
    s_trajectory.cycle_index = 0U;
    s_trajectory.total_cycles = total_cycles;
    s_trajectory.actual_stable_cycles = 0U;
    s_trajectory.start_counts = start_counts;
    s_trajectory.target_counts = target_counts;
    s_trajectory.command_counts = start_counts;
    s_trajectory.total_time_s = start_time;

    s_control.command_counts = start_counts;
    s_control.target_counts = target_counts;
    return ETHERCAT_MOTION_OK;
}
/* 【七段式-4结束】七段式轨迹规划器到此结束。 */

/* 【七段式-5开始】下面按2ms PDO周期计算七段式的下一目标位置。 */
/*
 * 每个PDO周期按恒定Jerk运动方程计算目标位置：
 * x=x0+v0*t+0.5*a0*t²+(1/6)*J*t³。
 * 七段目标生成结束后，继续等待实际位置连续稳定到位。
 */
static int motion_segment_process(void) {
    const motion_s7_segment_t *segment;
    float elapsed_time;
    float segment_time;
    float segment_end_time;
    float segment_time2;
    float segment_time3;
    float relative_position;
    float command_float;
    int64_t actual_error;
    uint32_t segment_index;
    uint32_t i;

    if (s_trajectory.running) {
        if (s_trajectory.cycle_index < s_trajectory.total_cycles) {
            s_trajectory.cycle_index++;
        }

        elapsed_time = (float) s_trajectory.cycle_index *
                       MOTION_PDO_PERIOD_S;
        if (elapsed_time > s_trajectory.total_time_s) {
            elapsed_time = s_trajectory.total_time_s;
        }

        /* 跳过持续时间为0的退化段，定位当前有效时间段。 */
        segment_index = MOTION_S7_SEGMENT_COUNT - 1U;
        for (i = 0U; i < MOTION_S7_SEGMENT_COUNT; i++) {
            segment_end_time =
                    s_trajectory.segment[i].start_time_s +
                    s_trajectory.segment[i].duration_s;
            if ((s_trajectory.segment[i].duration_s > 0.0f) &&
                (elapsed_time <= segment_end_time)) {
                segment_index = i;
                break;
            }
        }

        segment = &s_trajectory.segment[segment_index];
        segment_time = elapsed_time - segment->start_time_s;
        if (segment_time < 0.0f) {
            segment_time = 0.0f;
        } else if (segment_time > segment->duration_s) {
            segment_time = segment->duration_s;
        }

        segment_time2 = segment_time * segment_time;
        segment_time3 = segment_time2 * segment_time;
        relative_position =
                segment->start_position_counts +
                segment->start_velocity_counts_s * segment_time +
                0.5f * segment->start_acceleration_counts_s2 *
                segment_time2 +
                segment->jerk_counts_s3 * segment_time3 / 6.0f;
        command_float = (float) s_trajectory.start_counts +
                        relative_position;

        /* 抑制浮点积分误差造成的起点/终点轻微越界。 */
        if (s_trajectory.target_counts >= s_trajectory.start_counts) {
            if (command_float < (float) s_trajectory.start_counts) {
                command_float = (float) s_trajectory.start_counts;
            } else if (command_float >
                       (float) s_trajectory.target_counts) {
                command_float = (float) s_trajectory.target_counts;
            }
        } else {
            if (command_float > (float) s_trajectory.start_counts) {
                command_float = (float) s_trajectory.start_counts;
            } else if (command_float <
                       (float) s_trajectory.target_counts) {
                command_float = (float) s_trajectory.target_counts;
            }
        }

        if (command_float >= 0.0f) {
            s_trajectory.command_counts =
                    (int32_t) (command_float + 0.5f);
        } else {
            s_trajectory.command_counts =
                    (int32_t) (command_float - 0.5f);
        }

        if (s_trajectory.cycle_index >=
            s_trajectory.total_cycles) {
            s_trajectory.command_counts =
                    s_trajectory.target_counts;
            s_trajectory.running = 0U;
            s_trajectory.waiting_actual = 1U;
            s_trajectory.actual_stable_cycles = 0U;
        }

        s_control.command_counts =
                s_trajectory.command_counts;
    }

    if (!s_trajectory.waiting_actual) {
        return MOTION_SEGMENT_RUNNING;
    }

    actual_error =
            (int64_t) input1s->CurrentPosition -
            (int64_t) s_trajectory.target_counts;

    if (actual_error < 0) {
        actual_error = -actual_error;
    }

    if (actual_error <= MOTION_POSITION_TOLERANCE_COUNTS) {
        if (++s_trajectory.actual_stable_cycles >=
            MOTION_POSITION_STABLE_CYCLES) {
            s_trajectory.waiting_actual = 0U;
            return MOTION_SEGMENT_DONE;
        }
    } else {
        s_trajectory.actual_stable_cycles = 0U;
    }

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
            request->position_mm,
            &target_counts);
    } else if ((request->mode == ETHERCAT_MOTION_MODE_MOVE_REL) ||
               (request->mode == ETHERCAT_MOTION_MODE_JOG)) {
        result = motion_position_target_get(
            request->mode,
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
    return ETHERCAT_MOTION_OK;
}

/*
 * 往返状态机，所有单程都复用同一套S形轨迹。
 * 状态顺序为：移向终点→终点等待→返回起点→起点等待→下一次往返。
 */
static void motion_recip_process(void) {
    int segment_result;
    int result;

    switch (s_recip.stage) {
        case RECIP_STAGE_MOVE_OUT:
            segment_result = motion_segment_process();
            if (segment_result == MOTION_SEGMENT_DONE) {
                if (s_recip.first_interval_cycles > 0U) {
                    s_recip.wait_cycles =
                            s_recip.first_interval_cycles;
                    s_recip.stage = RECIP_STAGE_WAIT_FIRST;
                } else {
                    result = motion_recip_move_start(
                        s_recip.start_counts);
                    if (result != ETHERCAT_MOTION_OK) {
                        motion_finish_error();
                        return;
                    }
                    s_recip.stage = RECIP_STAGE_MOVE_BACK;
                }
            }
            break;

        case RECIP_STAGE_WAIT_FIRST:
            if (s_recip.wait_cycles > 0U) {
                s_recip.wait_cycles--;
            }

            if (s_recip.wait_cycles == 0U) {
                result = motion_recip_move_start(
                    s_recip.start_counts);
                if (result != ETHERCAT_MOTION_OK) {
                    motion_finish_error();
                    return;
                }
                s_recip.stage = RECIP_STAGE_MOVE_BACK;
            }
            break;

        case RECIP_STAGE_MOVE_BACK:
            segment_result = motion_segment_process();
            if (segment_result == MOTION_SEGMENT_DONE) {
                s_recip.completed_count++;

                if ((s_recip.repeat_count !=
                     ETHERCAT_MOTION_RECIP_FOREVER) &&
                    (s_recip.completed_count >=
                     s_recip.repeat_count)) {
                    s_recip.active = 0U;
                    s_recip.stage = RECIP_STAGE_IDLE;
                    motion_finish_success();
                    return;
                }

                if (s_recip.second_interval_cycles > 0U) {
                    s_recip.wait_cycles =
                            s_recip.second_interval_cycles;
                    s_recip.stage = RECIP_STAGE_WAIT_SECOND;
                } else {
                    result = motion_recip_move_start(
                        s_recip.end_counts);
                    if (result != ETHERCAT_MOTION_OK) {
                        motion_finish_error();
                        return;
                    }
                    s_recip.stage = RECIP_STAGE_MOVE_OUT;
                }
            }
            break;

        case RECIP_STAGE_WAIT_SECOND:
            if (s_recip.wait_cycles > 0U) {
                s_recip.wait_cycles--;
            }

            if (s_recip.wait_cycles == 0U) {
                result = motion_recip_move_start(
                    s_recip.end_counts);
                if (result != ETHERCAT_MOTION_OK) {
                    motion_finish_error();
                    return;
                }
                s_recip.stage = RECIP_STAGE_MOVE_OUT;
            }
            break;

        case RECIP_STAGE_IDLE:
        default:
            motion_finish_error();
            break;
    }
}

/*
 * 把应用层请求转换为内部运动。
 * STOP请求立即保持实际位置；RECIP进入往返状态机；其他模式启动单段轨迹。
 */
static void motion_request_apply(const motion_request_t *request) {
    int result;

    if (request->mode == ETHERCAT_MOTION_MODE_STOP) {
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
        return;
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

    request.mode = mode;
    request.position_mm = position_mm;
    request.velocity_mm_s = velocity_mm_s;
    request.acceleration_mm_s2 = acceleration_mm_s2;
    request.jerk_mm_s3 = jerk_mm_s3;

    return motion_request_submit(&request);
}

/*
 * 返回软件机械零点0mm。
 * 当前实现等价于提交目标位置为0mm的绝对运动，不执行原点开关搜索。
 */
int ethercat_motion_home_start(float velocity_mm_s,
                               float acceleration_mm_s2,
                               /* 【七段式-7】新增Jerk入参。 */
                               float jerk_mm_s3) {
    return ethercat_motion_command_set(
        ETHERCAT_MOTION_MODE_MOVE_ABS,
        0.0f,
        velocity_mm_s,
        acceleration_mm_s2,
        jerk_mm_s3);
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
 * 提交停止请求。
 * PDO任务收到请求后取消当前轨迹，并以当前实际位置作为新的保持位置。
 */
void ethercat_motion_stop(void) {
    taskENTER_CRITICAL();

    s_request.pending = 1U;
    s_request.mode = ETHERCAT_MOTION_MODE_STOP;

    taskEXIT_CRITICAL();
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
         CIA402_SW_OPERATION_ENABLED) ||
        (input1s->OpModeNow != 8)) {
        if (!s_motion_initialized) {
            s_control.command_counts = input1s->CurrentPosition;
            s_control.target_counts = input1s->CurrentPosition;
            s_motion_initialized = 1U;
        }
        motion_target_output_write();
        return;
    }

    /* 首次运行时把命令位置对齐实际位置，防止目标位置从0突跳。 */
    if (!s_motion_initialized) {
        s_control.command_counts = input1s->CurrentPosition;
        s_control.target_counts = input1s->CurrentPosition;
        s_motion_initialized = 1U;
    }

    if (motion_request_fetch(&request)) {
        motion_request_apply(&request);
    }

    if (s_control.busy) {
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
    status->command_position_counts =
            s_control.command_counts;
    status->target_position_counts =
            s_control.target_counts;
    status->actual_position_counts =
            (input1s != NULL) ? input1s->CurrentPosition : 0;
    status->recip_completed_count =
            s_recip.completed_count;

    taskEXIT_CRITICAL();
}
