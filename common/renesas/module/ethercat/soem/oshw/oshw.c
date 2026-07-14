#include "oshw.h"
#include <stddef.h>

uint16 oshw_htons(uint16 host)
{
    /* Cortex-R52 is little endian, Ethernet is big endian */
    return (uint16)(((host & 0xFF) << 8) | ((host >> 8) & 0xFF));
}

uint16 oshw_ntohs(uint16 network)
{
    return (uint16)(((network & 0xFF) << 8) | ((network >> 8) & 0xFF));
}

void* oshw_find_adapters(void)
{
    return NULL;
}

void oshw_free_adapters(void* adapter)
{
    (void)adapter;
}
