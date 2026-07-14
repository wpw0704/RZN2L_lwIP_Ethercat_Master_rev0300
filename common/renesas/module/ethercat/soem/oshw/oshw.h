#ifndef _oshw_
#define _oshw_

#ifdef __cplusplus
extern "C" {
#endif

#include "ethercattype.h"
#include "nicdrv.h"

uint16 oshw_htons(uint16 hostshort);
uint16 oshw_ntohs(uint16 networkshort);
void* oshw_find_adapters(void);
void oshw_free_adapters(void* adapter);

#ifdef __cplusplus
}
#endif

#endif
