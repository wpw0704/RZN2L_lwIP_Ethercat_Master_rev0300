#ifndef ETHERCAT_PORT_CFG_H
#define ETHERCAT_PORT_CFG_H

#include "um_ether_netif_api.h"

/*
 * EtherCAT/lwIP physical port mapping:
 *   0U: port0 = EtherCAT master, port1 = lwIP (default)
 *   1U: port1 = EtherCAT master, port0 = lwIP
 */
#define ETHERCAT_LWIP_PORT_SWAP (1U)

#if (0U == ETHERCAT_LWIP_PORT_SWAP)
#define ETHERCAT_MASTER_PORT_NUMBER (0U)
#define ETHERCAT_MASTER_IFNAME      "port0"
#define LWIP_ETHERNET_PORT_NUMBER   (1U)
#elif (1U == ETHERCAT_LWIP_PORT_SWAP)
#define ETHERCAT_MASTER_PORT_NUMBER (1U)
#define ETHERCAT_MASTER_IFNAME      "port1"
#define LWIP_ETHERNET_PORT_NUMBER   (2U)
#else
#error "ETHERCAT_LWIP_PORT_SWAP must be 0U or 1U"
#endif

#define ETHERCAT_MASTER_PORT_MASK   ETHER_NETIF_CFG_PORT_BIT(ETHERCAT_MASTER_PORT_NUMBER)
#define LWIP_ETHERNET_PORT_MASK     ETHER_NETIF_CFG_PORT_BIT(LWIP_ETHERNET_PORT_NUMBER)

#endif
