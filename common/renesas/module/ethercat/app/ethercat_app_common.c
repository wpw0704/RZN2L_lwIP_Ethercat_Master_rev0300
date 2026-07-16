#include "ethercat_app_common.h"

static ethercat_app_notify_t g_ethercat_app_notify;
static uint8_t g_ethercat_app_common_opened;

/*
 * 初始化 EtherCAT app 层通用通知资源。
 * 返回值：
 * - USR_SUCCESS：初始化成功，或之前已经初始化过。
 * - USR_ERR_NOT_INITIALIZED：FreeRTOS 信号量创建失败。
 */
usr_err_t ethercat_app_common_open(void)
{
    if (g_ethercat_app_common_opened)
    {
        return USR_SUCCESS;
    }

    g_ethercat_app_notify.master_scan_done_sem = xSemaphoreCreateBinary();
    if (NULL == g_ethercat_app_notify.master_scan_done_sem)
    {
        return USR_ERR_NOT_INITIALIZED;
    }

    g_ethercat_app_notify.master_scan_state = ETHERCAT_MASTER_SCAN_STATE_IDLE;
    g_ethercat_app_notify.master_scan_slave_count = 0;
    g_ethercat_app_common_opened = true;
    g_ethercat_app_notify.master_run_state = ETHERCAT_MASTER_RUN_STATE_IDLE;
    g_ethercat_app_notify.master_last_wkc = 0;
    g_ethercat_app_notify.master_expected_wkc = 0;
    g_ethercat_app_notify.master_al_state = 0;
    g_ethercat_app_notify.servo_status_word = 0;
    g_ethercat_app_notify.servo_control_word = 0;

    return USR_SUCCESS;
}

/*
 * 获取全局通知对象。
 * 返回值：
 * - ethercat_app_notify_t *：内部静态全局对象，保存任务句柄、信号量和扫描状态。
 */
ethercat_app_notify_t *ethercat_app_notify_get(void)
{
    return &g_ethercat_app_notify;
}

/*
 * 设置主站扫描状态。
 * 参数：
 * - state：新的扫描状态。
 * - slave_count：本次扫描到的从站数量；失败时传 0。
 *
 * 信号作用：
 * - 当 state 为 DONE 或 FAILED 时释放 master_scan_done_sem，
 *   后续任务可以等待该信号来判断扫描阶段已经结束。
 */
void ethercat_app_master_scan_set_state(ethercat_master_scan_state_t state, int slave_count)
{
    g_ethercat_app_notify.master_scan_state = state;
    g_ethercat_app_notify.master_scan_slave_count = slave_count;

    if ((ETHERCAT_MASTER_SCAN_STATE_DONE == state) || (ETHERCAT_MASTER_SCAN_STATE_FAILED == state))
    {
        (void) xSemaphoreGive(g_ethercat_app_notify.master_scan_done_sem);
    }
}

void ethercat_app_master_run_set_state(ethercat_master_run_state_t state)
{
    g_ethercat_app_notify.master_run_state = state;
}

ethercat_master_run_state_t ethercat_app_master_run_get_state(void)
{
    return g_ethercat_app_notify.master_run_state;
}

void ethercat_app_master_status_update(int last_wkc,
                                       int expected_wkc,
                                       uint16_t al_state,
                                       uint16_t status_word,
                                       uint16_t control_word)
{
    g_ethercat_app_notify.master_last_wkc = last_wkc;
    g_ethercat_app_notify.master_expected_wkc = expected_wkc;
    g_ethercat_app_notify.master_al_state = al_state;
    g_ethercat_app_notify.servo_status_word = status_word;
    g_ethercat_app_notify.servo_control_word = control_word;
}
