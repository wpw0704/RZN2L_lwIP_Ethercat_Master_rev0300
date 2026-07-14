#include "osal.h"
#include "FreeRTOS.h"
#include "task.h"

/* 启动 OSAL 定时器，按指定微秒超时时间计算停止 tick。 */
void osal_timer_start(osal_timert * self, uint32 timeout_us) {
    uint32 delta_ms = (timeout_us + 999) / 1000;
    self->stop_time.sec = 0;
    self->stop_time.usec = xTaskGetTickCount() + pdMS_TO_TICKS(delta_ms);
}

/* 判断 OSAL 定时器是否已经到达超时时间。 */
boolean osal_timer_is_expired(osal_timert * self) {
    uint32 now = xTaskGetTickCount();
    return (now >= self->stop_time.usec) ? TRUE : FALSE;
}

/* 以微秒为单位延时，内部转换为 FreeRTOS tick 延时。 */
int osal_usleep(uint32 usec) {
    vTaskDelay(pdMS_TO_TICKS((usec + 999)/1000));
    return 1;
}

/* 获取当前系统运行时间，并转换为 SOEM 使用的秒和微秒格式。 */
ec_timet osal_current_time(void) {
    ec_timet t;
    uint32 ticks = xTaskGetTickCount();
    t.sec = ticks / configTICK_RATE_HZ;
    t.usec = (ticks % configTICK_RATE_HZ) * (1000000 / configTICK_RATE_HZ);
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
