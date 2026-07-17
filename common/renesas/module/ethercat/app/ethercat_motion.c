#include "ethercat_motion.h"

#include "ethercat_master.h"
#include "FreeRTOS.h"
#include "task.h"

#include <float.h>
#include <limits.h>
#include <stddef.h>

/* 当前EtherCAT PDO周期为2ms。 */
#define MOTION_PDO_PERIOD_S                  (0.002f)

/* 五次多项式S曲线的速度、加速度归一化峰值。 */
#define MOTION_S_CURVE_MAX_DERIVATIVE        (1.875f)
#define MOTION_S_CURVE_MAX_SECOND_DERIVATIVE (5.773503f)

/* 轨迹结束后，实际位置连续落入误差范围才报告完成。 */
#define MOTION_POSITION_TOLERANCE_COUNTS     (500LL)
#define MOTION_POSITION_STABLE_CYCLES        (10U)

/* 毫秒转换为2ms PDO周期数，向上取整。 */
#define MOTION_MS_TO_CYCLES(ms) \
    (((uint32_t) (ms) / 2U) + (((uint32_t) (ms) % 2U) ? 1U : 0U))

typedef enum {
    MOTION_SEGMENT_RUNNING = 0,
    MOTION_SEGMENT_DONE = 1,
    MOTION_SEGMENT_ERROR = -1,
} motion_segment_result_t;

typedef enum {
    RECIP_STAGE_IDLE = 0,
    RECIP_STAGE_MOVE_OUT,
    RECIP_STAGE_WAIT_FIRST,
    RECIP_STAGE_MOVE_BACK,
    RECIP_STAGE_WAIT_SECOND,
} recip_stage_t;

/* 上位机命令先写入单槽请求区，再由PDO周期任务统一接收。 */
typedef struct {
    uint8_t pending;
    ethercat_motion_mode_t mode;
    float position_mm;
    float velocity_mm_s;
    float acceleration_mm_s2;
    uint32_t first_interval_ms;
    uint32_t second_interval_ms;
    uint32_t repeat_count;
} motion_request_t;

/* 五次多项式S形轨迹运行数据。 */
typedef struct {
    uint8_t running;
    uint8_t waiting_actual;
    uint32_t cycle_index;
    uint32_t total_cycles;
    uint32_t actual_stable_cycles;
    int32_t start_counts;
    int32_t target_counts;
    int32_t command_counts;
    float delta_counts;
} motion_trajectory_t;

/* 运动调度状态。 */
typedef struct {
    ethercat_motion_mode_t mode;
    uint8_t busy;
    uint8_t done;
    uint8_t error;
    int32_t command_counts;
    int32_t target_counts;
} motion_control_t;

/* 往返运动状态。 */
typedef struct {
    uint8_t active;
    recip_stage_t stage;
    int32_t start_counts;
    int32_t end_counts;
    float velocity_mm_s;
    float acceleration_mm_s2;
    uint32_t first_interval_cycles;
    uint32_t second_interval_cycles;
    uint32_t wait_cycles;
    uint32_t repeat_count;
    uint32_t completed_count;
} motion_recip_t;

/* 电机参数只在运动模块内部保存，应用层直接传入各项数值。 */
typedef struct {
    float encoder_counts_per_motor_rev;
    float lead_mm_per_screw_rev;
    float gear_ratio;
    float reducer_ratio;
    float max_motor_rpm;
} motion_motor_params_t;

static motion_motor_params_t s_motor_params;
static uint8_t s_motor_params_ready;
static uint8_t s_motion_initialized;
static motion_request_t s_request;
static motion_trajectory_t s_trajectory;
static motion_control_t s_control = {
    .mode = ETHERCAT_MOTION_MODE_IDLE,
    .busy = 0U,
    .done = 0U,
    .error = 0U,
    .command_counts = 0,
    .target_counts = 0,
};
static motion_recip_t s_recip;

static float motion_positive_sqrt(float value);
static uint8_t motion_float_is_finite(float value);
static uint8_t motion_float_is_positive(float value);
static float motion_counts_per_mm_get(void);
static float motion_max_linear_velocity_mm_s_get(void);
static int motion_mm_to_counts(float position_mm, int32_t *counts);
static int motion_position_target_get(ethercat_motion_mode_t mode,
                                      float position_mm,
                                      int32_t *target_counts);
static int motion_request_validate(ethercat_motion_mode_t mode,
                                   float velocity_mm_s,
                                   float acceleration_mm_s2);
static int motion_request_submit(const motion_request_t *request);
static uint8_t motion_request_fetch(motion_request_t *request);
static int motion_trajectory_start(int32_t start_counts,
                                   int32_t target_counts,
                                   float velocity_mm_s,
                                   float acceleration_mm_s2);
static int motion_segment_process(void);
static int motion_single_start(const motion_request_t *request);
static int motion_recip_start_internal(const motion_request_t *request);
static int motion_recip_move_start(int32_t target_counts);
static void motion_recip_process(void);
static void motion_request_apply(const motion_request_t *request);
static void motion_finish_success(void);
static void motion_finish_error(void);
static int32_t motion_closed_loop_correction_get(void);
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

/* 不依赖libm检查上位机浮点参数，拒绝NaN和正负无穷。 */
static uint8_t motion_float_is_finite(float value) {
    return (uint8_t) ((value <= FLT_MAX) &&
                     (value >= -FLT_MAX));
}

static uint8_t motion_float_is_positive(float value) {
    return (uint8_t) (motion_float_is_finite(value) &&
                     (value > 0.0f));
}

/* 计算1mm对应的编码器计数。 */
static float motion_counts_per_mm_get(void) {
    return (s_motor_params.encoder_counts_per_motor_rev *
            s_motor_params.gear_ratio *
            s_motor_params.reducer_ratio) /
           s_motor_params.lead_mm_per_screw_rev;
}

/* 根据最大电机转速计算机构允许的最大直线速度。 */
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

/* mm转换为counts，并检查int32范围。 */
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

/* 根据运动模式把上位机位置参数转换为最终绝对counts。 */
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

/* 检查速度、加速度以及电机最大转速限制。 */
static int motion_request_validate(ethercat_motion_mode_t mode,
                                   float velocity_mm_s,
                                   float acceleration_mm_s2) {
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
        (!motion_float_is_positive(acceleration_mm_s2))) {
        return ETHERCAT_MOTION_ERR_PARAM;
    }

    if (velocity_mm_s > motion_max_linear_velocity_mm_s_get()) {
        return ETHERCAT_MOTION_ERR_LIMIT;
    }

    return ETHERCAT_MOTION_OK;
}

/* 单槽命令提交接口，避免上位机任务直接修改PDO周期状态。 */
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

/* PDO任务读取并清除上位机请求。 */
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

/*
 * 启动五次多项式S形轨迹：
 * s(u)=10u^3-15u^4+6u^5，u范围为0到1。
 *
 * 轨迹时间同时满足上位机给出的最大速度和最大加速度。
 * Jerk由最终轨迹时间自动确定，因此接口无需额外传入Jerk。
 */
static int motion_trajectory_start(int32_t start_counts,
                                   int32_t target_counts,
                                   float velocity_mm_s,
                                   float acceleration_mm_s2) {
    int64_t delta_64;
    float distance_counts;
    float max_velocity_counts_s;
    float max_acceleration_counts_s2;
    float time_by_velocity;
    float time_by_acceleration;
    float total_time;
    float cycles_float;
    uint32_t total_cycles;

    delta_64 = (int64_t) target_counts - (int64_t) start_counts;

    if (delta_64 == 0) {
        s_trajectory.running = 0U;
        s_trajectory.waiting_actual = 1U;
        s_trajectory.actual_stable_cycles = 0U;
        s_trajectory.start_counts = start_counts;
        s_trajectory.target_counts = target_counts;
        s_trajectory.command_counts = target_counts;
        s_control.command_counts = target_counts;
        s_control.target_counts = target_counts;
        return ETHERCAT_MOTION_OK;
    }

    distance_counts = (delta_64 > 0) ?
                      (float) delta_64 :
                      (float) -delta_64;

    max_velocity_counts_s = velocity_mm_s *
                            motion_counts_per_mm_get();

    max_acceleration_counts_s2 = acceleration_mm_s2 *
                                 motion_counts_per_mm_get();

    if ((max_velocity_counts_s <= 0.0f) ||
        (max_acceleration_counts_s2 <= 0.0f)) {
        return ETHERCAT_MOTION_ERR_PARAM;
    }

    time_by_velocity =
            MOTION_S_CURVE_MAX_DERIVATIVE *
            distance_counts /
            max_velocity_counts_s;

    time_by_acceleration = motion_positive_sqrt(
        MOTION_S_CURVE_MAX_SECOND_DERIVATIVE *
        distance_counts /
        max_acceleration_counts_s2);

    total_time = (time_by_velocity > time_by_acceleration) ?
                 time_by_velocity :
                 time_by_acceleration;

    if (total_time < MOTION_PDO_PERIOD_S) {
        total_time = MOTION_PDO_PERIOD_S;
    }

    cycles_float = total_time / MOTION_PDO_PERIOD_S;

    if (cycles_float >= (float) UINT32_MAX) {
        return ETHERCAT_MOTION_ERR_LIMIT;
    }

    total_cycles = (uint32_t) cycles_float;
    if (((float) total_cycles * MOTION_PDO_PERIOD_S) < total_time) {
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
    s_trajectory.delta_counts = (float) delta_64;

    s_control.command_counts = start_counts;
    s_control.target_counts = target_counts;
    return ETHERCAT_MOTION_OK;
}

/*
 * 执行一段S形轨迹，并等待实际位置稳定到目标附近。
 * 返回1表示该段真正完成，返回0表示仍在运行。
 */
static int motion_segment_process(void) {
    float u;
    float u2;
    float u3;
    float u4;
    float u5;
    float blend;
    float command_float;
    int64_t actual_error;

    if (s_trajectory.running) {
        if (s_trajectory.cycle_index < s_trajectory.total_cycles) {
            s_trajectory.cycle_index++;
        }

        u = (float) s_trajectory.cycle_index /
            (float) s_trajectory.total_cycles;

        if (u > 1.0f) {
            u = 1.0f;
        }

        u2 = u * u;
        u3 = u2 * u;
        u4 = u3 * u;
        u5 = u4 * u;
        blend = 10.0f * u3 - 15.0f * u4 + 6.0f * u5;

        command_float =
                (float) s_trajectory.start_counts +
                s_trajectory.delta_counts * blend;

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

/* 启动绝对、相对、点动或回零运动。 */
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
        request->acceleration_mm_s2);

    if (result == ETHERCAT_MOTION_OK) {
        s_control.mode = request->mode;
        s_control.busy = 1U;
        s_control.done = 0U;
        s_control.error = 0U;
    }

    return result;
}

/* 启动往返运动的某一段。 */
static int motion_recip_move_start(int32_t target_counts) {
    return motion_trajectory_start(
        s_control.command_counts,
        target_counts,
        s_recip.velocity_mm_s,
        s_recip.acceleration_mm_s2);
}

/* 接收往返命令并启动第一段正向运动。 */
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

/* 往返状态机，所有单程都复用同一套S形轨迹。 */
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

/* 把上位机请求转换为内部运动。 */
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

static void motion_finish_success(void) {
    s_control.mode = ETHERCAT_MOTION_MODE_IDLE;
    s_control.busy = 0U;
    s_control.done = 1U;
    s_control.error = 0U;
}

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

/* 将开环轨迹目标和未来闭环修正量统一写入PDO。 */
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

int ethercat_motion_command_set(ethercat_motion_mode_t mode,
                                float position_mm,
                                float velocity_mm_s,
                                float acceleration_mm_s2) {
    motion_request_t request = {0};
    int result;

    result = motion_request_validate(mode,
                                     velocity_mm_s,
                                     acceleration_mm_s2);
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

    return motion_request_submit(&request);
}

int ethercat_motion_home_start(float velocity_mm_s,
                               float acceleration_mm_s2) {
    return ethercat_motion_command_set(
        ETHERCAT_MOTION_MODE_MOVE_ABS,
        0.0f,
        velocity_mm_s,
        acceleration_mm_s2);
}

int ethercat_motion_recip_start(float offset_mm,
                                float velocity_mm_s,
                                float acceleration_mm_s2,
                                uint32_t first_interval_ms,
                                uint32_t second_interval_ms,
                                uint32_t repeat_count) {
    motion_request_t request = {0};
    int result;

    result = motion_request_validate(
        ETHERCAT_MOTION_MODE_RECIP,
        velocity_mm_s,
        acceleration_mm_s2);
    if (result != ETHERCAT_MOTION_OK) {
        return result;
    }

    request.mode = ETHERCAT_MOTION_MODE_RECIP;
    request.position_mm = offset_mm;
    request.velocity_mm_s = velocity_mm_s;
    request.acceleration_mm_s2 = acceleration_mm_s2;
    request.first_interval_ms = first_interval_ms;
    request.second_interval_ms = second_interval_ms;
    request.repeat_count = repeat_count;

    return motion_request_submit(&request);
}

void ethercat_motion_stop(void) {
    taskENTER_CRITICAL();

    s_request.pending = 1U;
    s_request.mode = ETHERCAT_MOTION_MODE_STOP;

    taskEXIT_CRITICAL();
}

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
