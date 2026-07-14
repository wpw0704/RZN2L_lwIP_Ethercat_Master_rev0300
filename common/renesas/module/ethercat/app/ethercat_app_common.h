#ifndef ETHERCAT_APP_COMMON_H
#define ETHERCAT_APP_COMMON_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "um_common.h"

typedef enum e_ethercat_master_scan_state
{
    ETHERCAT_MASTER_SCAN_STATE_IDLE = 0,  /* 尚未启动主站扫描。 */
    ETHERCAT_MASTER_SCAN_STATE_RUNNING,   /* SOEM 扫描任务已经创建，正在枚举从站。 */
    ETHERCAT_MASTER_SCAN_STATE_DONE,      /* 扫描完成，至少找到一个从站。 */
    ETHERCAT_MASTER_SCAN_STATE_FAILED,    /* 扫描失败，未找到从站或初始化失败。 */
} ethercat_master_scan_state_t;

typedef struct st_ethercat_app_notify
{
    /* port1 Link 状态监控任务句柄，用于防止重复创建监控任务，后续也可用于任务通知。 */
    TaskHandle_t port_monitor_task;

    /* SOEM 主站扫描任务句柄，用于防止重复创建扫描任务，后续可扩展为主站状态机任务。 */
    TaskHandle_t master_scan_task;

    /* 主站扫描完成信号量：DONE/FAILED 时释放，其他任务可等待该信号后进入下一阶段。 */
    SemaphoreHandle_t master_scan_done_sem;

    /* 当前主站扫描状态，供其他任务判断是否可进入 PDO 映射、状态切换等后续流程。 */
    ethercat_master_scan_state_t master_scan_state;

    /* 最近一次扫描到的从站数量，DONE 时为有效数量，FAILED 时为 0。 */
    int master_scan_slave_count;
} ethercat_app_notify_t;

/* 初始化 EtherCAT app 层公共通知对象。调用本模块其他接口前应先调用该函数。 */
usr_err_t ethercat_app_common_open(void);

/* 获取公共通知对象指针。返回值为模块内部静态对象，不需要释放。 */
ethercat_app_notify_t *ethercat_app_notify_get(void);

/* 更新主站扫描状态和从站数量；状态进入 DONE/FAILED 时会释放 master_scan_done_sem。 */
void ethercat_app_master_scan_set_state(ethercat_master_scan_state_t state, int slave_count);

#endif
