#include "ethercat_motion.h"


static csp_motion_ctrl_t s_csp_motion_ctrl = {
    .mode = CSP_MOTION_MODE_IDLE,
    .last_mode = CSP_MOTION_MODE_IDLE,
    .enable = 0,
    .busy = 0,
    .done = 0,
    .error = 0,
    .command_pos = 0,
    .zero_pos = 0,
};

void ethercat_csp_motion_mode_set(csp_motion_mode_t mode) {
    s_csp_motion_ctrl.mode = mode;
    s_csp_motion_ctrl.enable = 1;
    s_csp_motion_ctrl.done = 0;
    s_csp_motion_ctrl.error = 0;
}

/*
 * 默认机械参数。
 * 当前：电机一圈 262144 counts，外部导程 50mm，按直连处理。
 */
static csp_mech_param_t s_csp_mech_param = {
    .motor_counts_per_rev = 262144.0f,
    .lead_mm_per_load_rev = 50.0f,
    .gear_ratio_motor_per_load = 1.0f,
};

static csp_traj_t s_csp_traj;
static csp_position_command_t s_csp_position_command;
static int csp_traj_process(csp_traj_t *traj);
static void csp_traj_start(csp_traj_t *traj,
                           int32 start_pos,
                           int32 target_pos,
                           float max_velocity_counts_s,
                           float accel_counts_s2);
static void csp_motion_position_process(void);
/*
 * 获取 1mm 对应多少 counts。
 *
 * 计算公式：
 * counts/mm = 电机一圈 counts * 齿轮比 / 导程
 */
static float csp_counts_per_mm_get(void) {
    return (s_csp_mech_param.motor_counts_per_rev *
            s_csp_mech_param.gear_ratio_motor_per_load) /
           s_csp_mech_param.lead_mm_per_load_rev;
}

/* mm 转换为 counts */
static int32 csp_mm_to_counts(float mm) {
    return (int32) (mm * csp_counts_per_mm_get());
}

/* m/s 转换为 counts/s */
static float csp_mps_to_counts_s(float mps) {
    return mps * 1000.0f * csp_counts_per_mm_get();
}

/* m/s^2 转换为 counts/s^2 */
static float csp_mps2_to_counts_s2(float mps2) {
    return mps2 * 1000.0f * csp_counts_per_mm_get();
}

static int csp_position_move_submit(csp_position_type_t type,
                                    float position_mm,
                                    float velocity_mps,
                                    float accel_mps2)
{
    if ((velocity_mps <= 0.0f) ||
        (velocity_mps > CSP_MAX_VELOCITY_MPS)) {
        return -1;
        }

    if ((accel_mps2 <= 0.0f) ||
        (accel_mps2 > CSP_MAX_ACCEL_MPS2)) {
        return -2;
        }

    /* 绝对位置必须位于机械有效行程内 */
    if ((type == CSP_POSITION_ABSOLUTE) &&
        ((position_mm < 0.0f) ||
         (position_mm > CSP_MACHINE_MAX_POSITION_MM))) {
        return -3;
         }

    /* 当前运动未结束时拒绝新命令 */
    if (s_csp_position_command.pending ||
        s_csp_position_command.active) {
        return -4;
        }

    taskENTER_CRITICAL();
    s_csp_position_command.type = type;
    s_csp_position_command.position_mm = position_mm;
    s_csp_position_command.velocity_mps = velocity_mps;
    s_csp_position_command.accel_mps2 = accel_mps2;
    s_csp_position_command.pending = 1U;
    taskEXIT_CRITICAL();

    ethercat_csp_motion_mode_set(CSP_MOTION_MODE_MOVE_POSITION);
    return 0;
}

int ethercat_csp_move_abs_start_mm(float target_mm,
                                   float velocity_mps,
                                   float accel_mps2)
{
    return csp_position_move_submit(CSP_POSITION_ABSOLUTE,
                                    target_mm,
                                    velocity_mps,
                                    accel_mps2);
}

int ethercat_csp_move_rel_start_mm(float delta_mm,
                                   float velocity_mps,
                                   float accel_mps2)
{
    return csp_position_move_submit(CSP_POSITION_RELATIVE,
                                    delta_mm,
                                    velocity_mps,
                                    accel_mps2);
}

/*
 * CSP 运动模式总调度函数。
 *
 * 调用位置：
 * - 该函数运行在 2ms PDO 周期任务中。
 * - 必须在伺服已经 Operation Enabled 后调用。
 *
 * 作用：
 * - 根据当前运动模式，调用对应的运动处理逻辑。
 * - 最终更新 output1s->TargetPos，作为下一周期发送给驱动器的目标位置。
 *
 * 注意：
 * - 本函数只能做快速计算和 PDO 输出赋值。
 * - 禁止在本函数中打印日志、读写 SDO、延时、等待上位机命令。
 * - 本周期计算出的 TargetPos，会在下一次 ec_send_processdata() 时发送出去。
 */
void ethercat_csp_motion_process(void) {
    /*
     * PDO 指针未准备好时直接返回。
    * 避免访问 input1s/output1s 导致 HardFault。
    */
    if ((input1s == NULL) || (output1s == NULL)) {
        return;
    }
    /*
     * 只有驱动器进入 Operation Enabled 后才允许运动。
     * StatusWord & 0x006F == 0x0027 表示 CiA402 Operation Enabled。
     */
    if ((input1s->StatusWord & CIA402_SW_MASK) != CIA402_SW_OPERATION_ENABLED) {
        return;
    }
    /*
     * 本运动调度器基于 CSP 模式。
     * 如果驱动器当前还不是 CSP，则写 0x6060=8，并先把目标位置对齐当前位置，
     * 避免 TargetPos 突然跳变造成报警或冲击。
     */
    if (input1s->OpModeNow != 8) {
        output1s->OpModeSet = 8;
        // output1s->TargetPos = input1s->CurrentPosition;
        return;
    }

    /*
     * 检测运动模式是否发生切换。
     * 模式切换时必须把命令位置对齐当前位置，防止旧模式目标位置残留。
     */
    if (s_csp_motion_ctrl.mode != s_csp_motion_ctrl.last_mode) {
        s_csp_motion_ctrl.last_mode = s_csp_motion_ctrl.mode;

        /*
         * command_pos 是主站内部轨迹命令位置。
         * 切换模式时从当前位置开始生成新轨迹。
         */
        // !!!!!!!!!!!!!!!!!
        s_csp_motion_ctrl.command_pos = input1s->CurrentPosition;
        output1s->TargetPos = s_csp_motion_ctrl.command_pos;

        /*
         * 清除上一模式的运行状态。
         */
        s_csp_motion_ctrl.busy = 0;
        s_csp_motion_ctrl.done = 0;
        s_csp_motion_ctrl.error = 0;
    }

    /*
    * 如果运动未使能，则保持当前位置。
    * 这里不做刹车曲线，只是简单把 TargetPos 对齐当前位置。
    */
    if (!s_csp_motion_ctrl.enable) {
        s_csp_motion_ctrl.command_pos = input1s->CurrentPosition;
        // output1s->TargetPos = s_csp_motion_ctrl.command_pos;
        return;
    }

    /*
     * 根据当前运动模式执行对应逻辑。
     * 后续每种模式只需要实现自己的 process 函数，
     * 并在这里调用即可。
     */
    switch (s_csp_motion_ctrl.mode) {
        case CSP_MOTION_MODE_IDLE:
            /*
             * 空闲模式：
             * 不产生新轨迹，保持当前 command_pos。
             */
            s_csp_motion_ctrl.busy = 0;
            output1s->TargetPos = s_csp_motion_ctrl.command_pos;
            break;

        case CSP_MOTION_MODE_JOG:
            /*
             * 点动模式：
             * 后续在这里调用点动处理函数。
             * 例如：csp_motion_jog_process();
             */
            break;

        case CSP_MOTION_MODE_MOVE_POSITION:
            /* 绝对和相对位置指令统一进入该处理函数 */
            csp_motion_position_process();
            break;

        case CSP_MOTION_MODE_HOME_ZERO:
            /*
             * 回零模式：
             * 后续在这里调用回零处理函数。
             * 可以是软件零点，也可以扩展为驱动器 Homing。
             */
            break;

        case CSP_MOTION_MODE_RECIP:
            /*
             * 往返模式：
             * 后续在这里调用往返运动处理函数。
             */
            break;

        case CSP_MOTION_MODE_SEQUENCE:
            /*
             * 序列模式：
             * 后续在这里调用多点位序列处理函数。
             */
            break;

        case CSP_MOTION_MODE_STOP:
            /*
             * 停止模式：
             * 当前先采用简单停止：目标位置对齐当前位置。
             * 后续可以扩展为按减速度平滑停止。
             */
            s_csp_motion_ctrl.command_pos = input1s->CurrentPosition;
            output1s->TargetPos = s_csp_motion_ctrl.command_pos;
            s_csp_motion_ctrl.busy = 0;
            break;

        case CSP_MOTION_MODE_FAULT:
        default:
            /*
             * 异常模式：
             * 置错误标志，并保持当前位置，避免继续发送旧目标。
             */
            s_csp_motion_ctrl.error = 1;
            s_csp_motion_ctrl.command_pos = input1s->CurrentPosition;
            output1s->TargetPos = s_csp_motion_ctrl.command_pos;
            break;
    }

    /*
     * 持续写 0x6060=8，确保驱动器保持 CSP 模式。
     */
    output1s->OpModeSet = 8;
}

/*
 * 到目标位置模式周期处理。
 *
 * 功能：
 * - 将上位机下发的 mm、m/s、m/s^2 转换为 counts 单位。
 * - 启动通用梯形轨迹。
 * - 每个 PDO 周期调用 csp_traj_process() 更新目标位置。
 *
 * 注意：
 * - 本函数运行在 PDO 周期中，不能打印、不能延时、不能读写 SDO。
 */
static void csp_motion_position_process(void)
{
    int64_t target_counts;
    int result;

    if (s_csp_position_command.pending &&
        !s_csp_position_command.active) {

        if (s_csp_position_command.type == CSP_POSITION_ABSOLUTE) {
            /* CurrentPosition=0就是机械零点 */
            target_counts =
                csp_mm_to_counts(s_csp_position_command.position_mm);
        } else {
            /* 相对运动：当前位置加上相对位移 */
            target_counts =
                (int64_t) input1s->CurrentPosition +
                csp_mm_to_counts(s_csp_position_command.position_mm);
        }

        /* 相对运动计算后也不能超出机械绝对行程 */
        if ((target_counts < 0) ||
            (target_counts >
             csp_mm_to_counts(CSP_MACHINE_MAX_POSITION_MM))) {
            s_csp_position_command.pending = 0U;
            s_csp_motion_ctrl.error = 1U;
            return;
             }

        s_csp_position_command.target_counts = (int32) target_counts;

        /* 绝对和相对运动从这里开始共用同一套梯形轨迹 */
        csp_traj_start(
            &s_csp_traj,
            input1s->CurrentPosition,
            s_csp_position_command.target_counts,
            csp_mps_to_counts_s(s_csp_position_command.velocity_mps),
            csp_mps2_to_counts_s2(s_csp_position_command.accel_mps2));

        s_csp_position_command.pending = 0U;
        s_csp_position_command.active = 1U;
        s_csp_motion_ctrl.busy = 1U;
        s_csp_motion_ctrl.done = 0U;
        }

    if (!s_csp_position_command.active) {
        return;
    }

    result = csp_traj_process(&s_csp_traj);
    output1s->TargetPos = s_csp_traj.command_pos;

    if (result != 0) {
        s_csp_position_command.active = 0U;
        s_csp_motion_ctrl.busy = 0U;
        s_csp_motion_ctrl.done = (result > 0) ? 1U : 0U;
        s_csp_motion_ctrl.error = (result < 0) ? 1U : 0U;
        s_csp_motion_ctrl.mode = CSP_MOTION_MODE_IDLE;
        s_csp_motion_ctrl.enable = 0U;
    }
}



/*
 * 启动一段通用位置轨迹。
 *
 * 参数：
 * - traj：轨迹对象
 * - start_pos：起始位置，单位 counts
 * - target_pos：目标位置，单位 counts
 * - max_velocity_counts_s：最大速度，单位 counts/s
 * - accel_counts_s2：加速度，单位 counts/s^2
 */
static void csp_traj_start(csp_traj_t *traj,
                           int32 start_pos,
                           int32 target_pos,
                           float max_velocity_counts_s,
                           float accel_counts_s2) {
    traj->running = 1U;
    traj->done = 0U;

    traj->command_pos = start_pos;
    traj->target_pos = target_pos;

    traj->velocity = 0.0f;
    traj->max_velocity = max_velocity_counts_s;
    traj->accel = accel_counts_s2;
}

/*
 * 通用梯形轨迹周期处理函数。
 *
 * 调用周期：
 * - 每 2ms PDO 周期调用一次。
 *
 * 功能：
 * - 根据最大速度和加速度更新 command_pos。
 * - 接近目标时自动减速。
 *
 * 注意：
 * - 这里只做计算，不打印、不延时、不读写 SDO。
 *
 * 返回值：
 *  1 = 到达目标
 *  0 = 正在运行
 * -1 = 参数异常
 */
static int csp_traj_process(csp_traj_t *traj) {
    int32 remaining;
    float distance;
    float direction;
    float stop_distance;
    float step;

    if ((traj->max_velocity <= 0.0f) || (traj->accel <= 0.0f)) {
        traj->running = 0U;
        traj->done = 0U;
        return -1;
    }

    remaining = traj->target_pos - traj->command_pos;

    if (remaining == 0) {
        traj->velocity = 0.0f;
        traj->running = 0U;
        traj->done = 1U;
        return 1;
    }

    direction = (remaining > 0) ? 1.0f : -1.0f;
    distance = (remaining > 0) ? (float) remaining : (float) -remaining;

    /*
     * 刹停距离公式：
     * stop_distance = v^2 / (2a)
     *
     * 如果剩余距离小于等于刹停距离，就开始减速；
     * 否则继续加速，直到最大速度。
     */
    stop_distance = (traj->velocity * traj->velocity) / (2.0f * traj->accel);

    if (distance <= stop_distance) {
        /* 减速阶段 */
        traj->velocity -= traj->accel * CSP_PDO_PERIOD_S;

        if (traj->velocity < 0.0f) {
            traj->velocity = 0.0f;
        }
    } else {
        /* 加速阶段 */
        traj->velocity += traj->accel * CSP_PDO_PERIOD_S;

        if (traj->velocity > traj->max_velocity) {
            traj->velocity = traj->max_velocity;
        }
    }

    /*
     * 本周期位置增量：
     * step = velocity * period
     */
    step = traj->velocity * CSP_PDO_PERIOD_S;

    /*
     * 防止低速时 step 小于 1 count，导致命令位置长时间不变化。
     */
    if (step < 1.0f) {
        step = 1.0f;
    }

    /*
     * 如果本周期步长已经超过剩余距离，直接落到目标点。
     */
    if (step >= distance) {
        traj->command_pos = traj->target_pos;
        traj->velocity = 0.0f;
        traj->running = 0U;
        traj->done = 1U;
        return 1;
    }

    traj->command_pos += (int32) (direction * step);

    return 0;
}

/*
 * 设置机械参数。
 * 后续上位机下发导程、齿轮比、电机一圈 counts 时调用。
 */
int ethercat_csp_mech_param_set(float motor_counts_per_rev,
                                float lead_mm_per_load_rev,
                                float gear_ratio_motor_per_load) {
    if ((motor_counts_per_rev <= 0.0f) ||
        (lead_mm_per_load_rev <= 0.0f) ||
        (gear_ratio_motor_per_load <= 0.0f)) {
        return 0;
    }

    s_csp_mech_param.motor_counts_per_rev = motor_counts_per_rev;
    s_csp_mech_param.lead_mm_per_load_rev = lead_mm_per_load_rev;
    s_csp_mech_param.gear_ratio_motor_per_load = gear_ratio_motor_per_load;
    return 1;
}
