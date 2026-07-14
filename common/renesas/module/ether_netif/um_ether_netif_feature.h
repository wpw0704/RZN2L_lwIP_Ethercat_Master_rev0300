/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_ether_netif_feature.h
 * Version      :1
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History :
 *
 *
 *********************************************************************************************************************/

#ifndef UM_ETHER_NETIF_FEATURE_H_
#define UM_ETHER_NETIF_FEATURE_H_

#include "bsp_mcu_family_cfg.h"

#if defined(BSP_MCU_GROUP_RA6M3)

/** This module supports RA6M3 series with RA FSP. */
#define UM_ETHER_NETIF_FEATURE_R_ETHER_IMPL_R_ETHER         (1)
#define UM_ETHER_NETIF_FEATURE_R_ETHER_IMPL_R_GMAC          (0)
#define UM_ETHER_NETIF_FEATURE_R_ETHER_DEPEND_ON_R_ETHSW    (0)
#define UM_ETHER_NETIF_FEATURE_R_ETHSW_SUPPORT_SPECIFIC_TAG (0)
#define UM_ETHER_NETIF_FEATURE_NUMBER_OF_ETHER_PORTS        (1)

#elif (defined(BSP_MCU_GROUP_RZT2M) || defined(BSP_MCU_GROUP_RZN2L) || \
defined(BSP_MCU_GROUP_RZT2ME))

/** This module supports RZT2M/RZT2L/RZN2L/RZT2ME. */
#define UM_ETHER_NETIF_FEATURE_R_ETHER_IMPL_R_ETHER         (0)
#define UM_ETHER_NETIF_FEATURE_R_ETHER_IMPL_R_GMAC          (1)
#define UM_ETHER_NETIF_FEATURE_R_ETHER_DEPEND_ON_R_ETHSW    (1)
#define UM_ETHER_NETIF_FEATURE_R_ETHSW_SUPPORT_SPECIFIC_TAG (1)
#define UM_ETHER_NETIF_FEATURE_NUMBER_OF_ETHER_PORTS        (3)

#elif defined(BSP_MCU_GROUP_RZT2L)

/** This module supports RZT2L series with RZT FSP. */
#define UM_ETHER_NETIF_FEATURE_R_ETHER_IMPL_R_ETHER         (0)
#define UM_ETHER_NETIF_FEATURE_R_ETHER_IMPL_R_GMAC          (1)
#define UM_ETHER_NETIF_FEATURE_R_ETHER_DEPEND_ON_R_ETHSW    (0)
#define UM_ETHER_NETIF_FEATURE_R_ETHSW_SUPPORT_SPECIFIC_TAG (0)
#define UM_ETHER_NETIF_FEATURE_NUMBER_OF_ETHER_PORTS        (3)

#else

#error "Unsupported device."

#endif

#endif /** UM_ETHER_NETIF_FEATURE_H_ */
