#ifndef OSAL_DEFS_H
#define OSAL_DEFS_H

#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

#define OSAL_THREAD_HANDLE void*
#define OSAL_MUTEX_HANDLE  SemaphoreHandle_t

#ifndef PACKED
  #if defined(__ICCARM__)
    #define PACKED_BEGIN  _Pragma("pack(push, 1)")
    #define PACKED        /* IAR: packing via PACKED_BEGIN/END pragma */
    #define PACKED_END    _Pragma("pack(pop)")
  #elif defined(__GNUC__)
    #define PACKED_BEGIN
    #define PACKED  __attribute__((__packed__))
    #define PACKED_END
  #else
    #define PACKED_BEGIN
    #define PACKED
    #define PACKED_END
  #endif
#endif

// define if debug printf is needed
//#define EC_DEBUG

#ifdef EC_DEBUG
#include <stdio.h>
#define EC_PRINT printf
#else
#define EC_PRINT(...) do {} while (0)
#endif

static inline int osal_getsysinfo(char *sysinfo) {
    strcpy(sysinfo, "FreeRTOS_RZN2L");
    return 1;
}

static inline int osal_mutex_init(OSAL_MUTEX_HANDLE *mutex) {
    if (mutex) {
        *mutex = xSemaphoreCreateMutex();
        return (*mutex != NULL) ? 1 : 0;
    }
    return 0;
}

static inline void osal_mutex_lock(OSAL_MUTEX_HANDLE *mutex) {
    if (mutex && *mutex) {
        xSemaphoreTake(*mutex, portMAX_DELAY);
    }
}

static inline void osal_mutex_unlock(OSAL_MUTEX_HANDLE *mutex) {
    if (mutex && *mutex) {
        xSemaphoreGive(*mutex);
    }
}

#endif /* OSAL_DEFS_H */
