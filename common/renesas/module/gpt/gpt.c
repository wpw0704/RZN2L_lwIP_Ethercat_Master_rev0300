#include "gpt.h"

#include "um_common.h"
#include <stdbool.h>

#define ETHERCAT_DC_CYCLE_NS          (2000000LL) /* EtherCAT周期：2ms */
#define GPT_NOMINAL_PERIOD_COUNTS     (800000U)   /* 当前GPT的2ms计数值 */
#define GPT_MAX_ADJUST_NS             (2000LL)    /* 单周期最多调整2us */

SemaphoreHandle_t s_gpt_cycle_semaphore = NULL;
static bool s_gpt_opened = false;
static bool s_gpt_started = false;

fsp_err_t gpt_init(void) {
    fsp_err_t err;

    if (s_gpt_cycle_semaphore == NULL) {
        s_gpt_cycle_semaphore = xSemaphoreCreateBinary();
        if (s_gpt_cycle_semaphore == NULL) {
            USR_LOG_ERROR("GPT semaphore create failed");
            return FSP_ERR_NOT_INITIALIZED;
        }
    }

    if (!s_gpt_opened) {
        err = R_GPT_Open(&g_timer0_ctrl,
                         &g_timer0_cfg);

        if (err != FSP_SUCCESS) {
            USR_LOG_ERROR(
                "GPT open failed: %d",
                err);

            return err;
        }

        s_gpt_opened = true;
    }

    if (s_gpt_started) {
        return FSP_SUCCESS;
    }

    err = R_GPT_Start(&g_timer0_ctrl);

    if (err != FSP_SUCCESS) {
        USR_LOG_ERROR(
            "GPT start failed: %d",
            err);

        return err;
    }

    s_gpt_started = true;

    return FSP_SUCCESS;
}

fsp_err_t gpt_stop(void) {
    fsp_err_t err;

    if (!s_gpt_started) {
        return FSP_SUCCESS;
    }

    err = R_GPT_Stop(&g_timer0_ctrl);

    if (err == FSP_SUCCESS) {
        s_gpt_started = false;
    }

    return err;
}

void gpt0_callback(timer_callback_args_t *p_args) {
    if (p_args != NULL && p_args->event == TIMER_EVENT_CYCLE_END) {
        /*
* The ISR only releases the cycle semaphore. PDO exchange,
* logging and motion calculation stay in task context.
*/
        BaseType_t higher_priority_task_woken =
                pdFALSE;
        xSemaphoreGiveFromISR(
            s_gpt_cycle_semaphore,
            &higher_priority_task_woken);
        portYIELD_FROM_ISR(
            higher_priority_task_woken);
    }
}


/**
 * @brief 根据 EtherCAT DC 时间微调 GPT 周期
 *
 * 第一次调用时记录当前相位，后续保持该相位不再缓慢漂移。
 * 只微调 GPT 周期，不改变 EtherCAT 的2ms通信周期。
 *
 * @param dc_time_ns SOEM提供的分布式时钟时间 ec_DCtime，单位ns
 */
void gpt_dc_sync_adjust(int64_t dc_time_ns)
{
    static bool initialized = false;
    static int64_t target_phase_ns = 0;
    static int64_t integral = 0;

    int64_t phase_ns;
    int64_t error_ns;
    int64_t adjust_ns;
    int64_t period_counts;

    /* 计算当前PDO执行时刻在2ms DC周期内的位置 */
    phase_ns = dc_time_ns % ETHERCAT_DC_CYCLE_NS;
    if (phase_ns < 0) {
        phase_ns += ETHERCAT_DC_CYCLE_NS;
    }

    /*
     * 第一次运行时，以当前相位作为目标。
     * 这样不会突然把相位拉到其他位置，能够避免电机受到冲击。
     */
    if (!initialized) {
        target_phase_ns = phase_ns;
        integral = 0;
        initialized = true;
        return;
    }

    error_ns = phase_ns - target_phase_ns;

    /* 处理2ms周期边界，取得最短方向的相位误差 */
    if (error_ns > (ETHERCAT_DC_CYCLE_NS / 2)) {
        error_ns -= ETHERCAT_DC_CYCLE_NS;
    } else if (error_ns < -(ETHERCAT_DC_CYCLE_NS / 2)) {
        error_ns += ETHERCAT_DC_CYCLE_NS;
    }

    /*
     * 缓慢积分，用于补偿GPT和EtherCAT DC之间的长期频率误差。
     * 不直接累加error_ns，避免积分变化过快。
     */
    if (error_ns > 0) {
        integral++;
    } else if (error_ns < 0) {
        integral--;
    }

    /* 防止积分无限增长 */
    if (integral > 20000) {
        integral = 20000;
    } else if (integral < -20000) {
        integral = -20000;
    }

    /*
     * PI形式的周期修正：
     * 相位偏大时缩短下一周期，相位偏小时延长下一周期。
     */
    adjust_ns = -(error_ns / 100) - (integral / 20);

    /* 限制单周期修正量，避免PDO周期发生突变 */
    if (adjust_ns > GPT_MAX_ADJUST_NS) {
        adjust_ns = GPT_MAX_ADJUST_NS;
    } else if (adjust_ns < -GPT_MAX_ADJUST_NS) {
        adjust_ns = -GPT_MAX_ADJUST_NS;
    }

    /*
     * 当前配置中：
     * 2,000,000ns对应800,000个GPT计数。
     */
    period_counts =
        GPT_NOMINAL_PERIOD_COUNTS +
        ((adjust_ns * GPT_NOMINAL_PERIOD_COUNTS) /
         ETHERCAT_DC_CYCLE_NS);

    (void) R_GPT_PeriodSet(&g_timer0_ctrl, (uint32_t) period_counts);
}
