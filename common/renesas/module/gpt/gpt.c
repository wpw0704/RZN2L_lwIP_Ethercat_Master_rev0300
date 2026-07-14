#include "gpt.h"

#include "um_common.h"
#include <stdbool.h>

static TaskHandle_t s_gpt_notify_task = NULL;
static bool s_gpt_opened = false;
static bool s_gpt_started = false;

void gpt_set_notify_task(TaskHandle_t task_handle)
{
    /*
     * 调用顺序必须是：
     * 设置任务句柄 -> 启动GPT。
     *
     * 停止时必须是：
     * 停止GPT -> 清空任务句柄。
     */
    s_gpt_notify_task = task_handle;
}

fsp_err_t gpt_init(void)
{
    fsp_err_t err;

    if (!s_gpt_opened)
    {
        err = R_GPT_Open(&g_timer0_ctrl,
                         &g_timer0_cfg);

        if (err != FSP_SUCCESS)
        {
            USR_LOG_ERROR(
                "GPT open failed: %d",
                err);

            return err;
        }

        s_gpt_opened = true;
    }

    if (s_gpt_started)
    {
        return FSP_SUCCESS;
    }

    err = R_GPT_Start(&g_timer0_ctrl);

    if (err != FSP_SUCCESS)
    {
        USR_LOG_ERROR(
            "GPT start failed: %d",
            err);

        return err;
    }

    s_gpt_started = true;

    return FSP_SUCCESS;
}

fsp_err_t gpt_stop(void)
{
    fsp_err_t err;

    if (!s_gpt_started)
    {
        return FSP_SUCCESS;
    }

    err = R_GPT_Stop(&g_timer0_ctrl);

    if (err == FSP_SUCCESS)
    {
        s_gpt_started = false;
    }

    return err;
}

void gpt0_callback(timer_callback_args_t *p_args)
{
    BaseType_t higher_priority_task_woken =
        pdFALSE;

    if ((p_args == NULL) ||
        (p_args->event != TIMER_EVENT_CYCLE_END))
    {
        return;
    }

    if (s_gpt_notify_task != NULL)
    {
        /*
         * GPT每4 ms增加一次任务通知计数。
         * ISR中只通知，不进行PDO、日志或运动计算。
         */
        vTaskNotifyGiveFromISR(
            s_gpt_notify_task,
            &higher_priority_task_woken);
    }

    portYIELD_FROM_ISR(
        higher_priority_task_woken);
}