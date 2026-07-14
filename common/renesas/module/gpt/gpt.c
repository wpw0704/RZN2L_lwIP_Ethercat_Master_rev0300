//
// Created by wpw07 on 2026/7/14.
//
#include "gpt.h"

fsp_err_t gpt_init() {
    static fsp_err_t err;
    err = R_GPT_Open(&g_timer0_ctrl, &g_timer0_cfg);

    if (FSP_SUCCESS != err) {
        USR_LOG_INFO("GPT Open Error: %d", err);
        return err;
    }
    err = R_GPT_Start(&g_timer0_ctrl);

    if (FSP_SUCCESS != err) {
        USR_LOG_INFO("GPT Open Error: %d", err);
        return err;
    }
    return err;
}

void gpt0_callback(timer_callback_args_t *p_args) {
}
