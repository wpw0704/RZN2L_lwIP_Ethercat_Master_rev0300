//
// Created by wpw07 on 2026/7/14.
//

#ifndef RZN2L_LWIP_ETHERCAT_MASTER_REV0300_GPT_H
#define RZN2L_LWIP_ETHERCAT_MASTER_REV0300_GPT_H

#include "hal_data.h"
#include "FreeRTOS.h"
#include "task.h"
#include "um_common.h"

fsp_err_t gpt_init(void);
fsp_err_t gpt_stop(void);

void gpt_set_notify_task(TaskHandle_t task_handle);

// void gpt0_callback(timer_callback_args_t *p_args);

#endif //RZN2L_LWIP_ETHERCAT_MASTER_REV0300_GPT_H
