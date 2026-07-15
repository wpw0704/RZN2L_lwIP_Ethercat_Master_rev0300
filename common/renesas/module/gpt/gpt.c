#include "gpt.h"

#include "um_common.h"
#include <stdbool.h>

SemaphoreHandle_t s_gpt_cycle_semaphore = NULL;
static bool s_gpt_opened = false;
static bool s_gpt_started = false;

// void gpt_set_cycle_semaphore(SemaphoreHandle_t semaphore_handle)
// {
//     /*
//      * Set the binary semaphore before starting GPT.
//      * Stop GPT before clearing the semaphore handle.
//      */
//     s_gpt_cycle_semaphore = semaphore_handle;
// }

fsp_err_t gpt_init(void) {
    fsp_err_t err;

    if (!s_gpt_opened) {
        err = R_GPT_Open(&g_timer0_ctrl, &g_timer0_cfg);

        if (err != FSP_SUCCESS) {
            USR_LOG_ERROR("GPT open failed: %d", err);
            return err;
        }

        s_gpt_opened = true;
    }

    if (s_gpt_started) {
        return FSP_SUCCESS;
    }

    /*
     * 必须先创建信号量再启动GPT。
     * 否则定时器中断可能先进入回调，导致ISR里操作空指针。
     */
    if (s_gpt_cycle_semaphore == NULL) {
        s_gpt_cycle_semaphore = xSemaphoreCreateBinary();
        if (s_gpt_cycle_semaphore == NULL) {
            USR_LOG_ERROR("GPT semaphore create failed");
            return FSP_ERR_OUT_OF_MEMORY;
        }
    }

    err = R_GPT_Start(&g_timer0_ctrl);

    if (err != FSP_SUCCESS) {
        USR_LOG_ERROR("GPT start failed: %d", err);
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
    if ((p_args != NULL) &&
        (p_args->event == TIMER_EVENT_CYCLE_END) &&
        (s_gpt_cycle_semaphore != NULL)) {
        /*
         * ISR只释放周期信号量。
         * PDO交换、日志和运动计算都放在任务上下文中执行。
         */
        BaseType_t higher_priority_task_woken = pdFALSE;

        xSemaphoreGiveFromISR(
            s_gpt_cycle_semaphore,
            &higher_priority_task_woken);

        portYIELD_FROM_ISR(higher_priority_task_woken);
        }
}
