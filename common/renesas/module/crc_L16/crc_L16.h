//
// Created by wpw07 on 2026/8/13.
//

#ifndef RZN2L_LWIP_ETHERNET_REV0300_CRC_L16_H
#define RZN2L_LWIP_ETHERNET_REV0300_CRC_L16_H
#include "main_thread.h"
#include "hal_data.h"
#include "um_common.h"

void crc_init(void);

uint32_t crc_calc(uint8_t *p_data, uint32_t length);

#endif //RZN2L_LWIP_ETHERNET_REV0300_CRC_L16_H
