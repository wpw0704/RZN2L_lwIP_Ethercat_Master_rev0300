#include "osal.h"
#include "FreeRTOS.h"
#include "task.h"

#define UNIX_TO_ECAT_EPOCH_SECONDS    (946684800ULL)


/* 启动 OSAL 定时器，按指定微秒超时时间计算停止 tick。 */
// void osal_timer_start(osal_timert * self, uint32 timeout_us) {
//     uint32 delta_ms = (timeout_us + 999) / 1000;
//     self->stop_time.sec = 0;
//     self->stop_time.usec = xTaskGetTickCount() + pdMS_TO_TICKS(delta_ms);
// }

void osal_timer_start(osal_timert *self, uint32 timeout_us)
{
    TickType_t timeout_ticks;
    uint32 delta_ms;

    if (self == NULL)
    {
        return;
    }

    delta_ms = (timeout_us + 999U) / 1000U;
    timeout_ticks = pdMS_TO_TICKS(delta_ms);

    if ((timeout_us > 0U) && (timeout_ticks == 0U))
    {
        timeout_ticks = 1U;
    }

    /*
     * stop_time.sec  保存启动tick
     * stop_time.usec 保存持续tick
     */
    self->stop_time.sec  = (uint32)xTaskGetTickCount();
    self->stop_time.usec = (uint32)timeout_ticks;
}

/* 判断 OSAL 定时器是否已经到达超时时间。 */
// boolean osal_timer_is_expired(osal_timert * self) {
//     uint32 now = xTaskGetTickCount();
//     return (now >= self->stop_time.usec) ? TRUE : FALSE;
// }

boolean osal_timer_is_expired(osal_timert *self)
{
    TickType_t now;
    TickType_t start;
    TickType_t timeout_ticks;
    TickType_t elapsed;

    if (self == NULL)
    {
        return TRUE;
    }

    now           = xTaskGetTickCount();
    start         = (TickType_t)self->stop_time.sec;
    timeout_ticks = (TickType_t)self->stop_time.usec;
    elapsed       = now - start;

    return (elapsed >= timeout_ticks) ? TRUE : FALSE;
}

/* 以微秒为单位延时，内部转换为 FreeRTOS tick 延时。 */
int osal_usleep(uint32 usec) {
    vTaskDelay(pdMS_TO_TICKS((usec + 999)/1000));
    return 1;
}

/**
 * @brief 获取供 SOEM 使用的当前时间。
 *
 * 当前系统没有提供真实 Unix/RTC 时间，因此使用：
 *
 * Unix到EtherCAT纪元偏移 + FreeRTOS启动运行时间
 *
 * ecx_configdc() 内部会再减去 946684800 秒，
 * 最终得到以系统启动时刻为零点的 EtherCAT DC 时间。
 */
ec_timet osal_current_time(void)
{
    ec_timet t;
    TickType_t ticks;
    uint64 uptime_us;
    uint64 time_sec;

    ticks = xTaskGetTickCount();

    /* FreeRTOS tick转换成系统启动后的微秒数 */
    uptime_us =
        ((uint64)ticks * 1000000ULL) /
        (uint64)configTICK_RATE_HZ;

    /*
     * 提前加上Unix到EtherCAT纪元的偏移。
     * ecx_configdc()会减去相同的946684800秒。
     */
    time_sec =
        UNIX_TO_ECAT_EPOCH_SECONDS +
        (uptime_us / 1000000ULL);

    t.sec  = (uint32)time_sec;
    t.usec = (uint32)(uptime_us % 1000000ULL);

    return t;
}

/* 计算两个 SOEM 时间点之间的时间差。 */
void osal_time_diff(ec_timet *start, ec_timet *end, ec_timet *diff) {
    uint64 s = (uint64)start->sec * 1000000ULL + start->usec;
    uint64 e = (uint64)end->sec   * 1000000ULL + end->usec;
    uint64 d = (e >= s) ? (e - s) : 0;
    diff->sec = (uint32)(d / 1000000ULL);
    diff->usec = (uint32)(d % 1000000ULL);
}

/* 创建普通线程的 OSAL 接口，本工程当前未启用线程创建功能。 */
int osal_thread_create(void *thandle, int stacksize, void *func, void *param) {
    (void)thandle; (void)stacksize; (void)func; (void)param;
    return 0; // Not supported / Not needed for pure loop
}

/* 创建实时线程的 OSAL 接口，本工程当前未启用实时线程创建功能。 */
int osal_thread_create_rt(void *thandle, int stacksize, void *func, void *param) {
    (void)thandle; (void)stacksize; (void)func; (void)param;
    return 0;
}
