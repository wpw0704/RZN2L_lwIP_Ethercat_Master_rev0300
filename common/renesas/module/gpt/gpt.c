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
    s_gpt_cycle_semaphore = xSemaphoreCreateBinary();
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
