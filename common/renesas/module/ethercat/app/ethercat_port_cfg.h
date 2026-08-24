#ifndef ETHERCAT_PORT_CFG_H
#define ETHERCAT_PORT_CFG_H

#include "um_ether_netif_api.h"

/*
 * EtherCAT/lwIP physical port selection.
 *
 * Only change ETHERCAT_LWIP_PORT_SELECTION. The first parameter selects the
 * SOEM EtherCAT master port and the second parameter selects the lwIP port.
 * Valid physical port numbers are 0U, 1U and 2U; the two ports must differ.
 *
 * Example: ETHERCAT_LWIP_PORTS(1U, 2U) selects SOEM port1 and lwIP port2.
 */
#define ETHERCAT_LWIP_PORTS(ethercat_port, lwip_port) (((ethercat_port) << 2U) | (lwip_port))
#define ETHERCAT_LWIP_PORT_SELECTION                  ETHERCAT_LWIP_PORTS(0U, 1U)

#define ETHERCAT_MASTER_PORT_NUMBER ((ETHERCAT_LWIP_PORT_SELECTION >> 2U) & 0x03U)
#define LWIP_ETHERNET_PORT_NUMBER   (ETHERCAT_LWIP_PORT_SELECTION & 0x03U)

#if (ETHERCAT_MASTER_PORT_NUMBER > 2U) || (LWIP_ETHERNET_PORT_NUMBER > 2U)
#error "EtherCAT and lwIP port numbers must be 0U, 1U or 2U"
#elif (ETHERCAT_MASTER_PORT_NUMBER == LWIP_ETHERNET_PORT_NUMBER)
#error "EtherCAT and lwIP must use different physical ports"
#endif

#if (0U == ETHERCAT_MASTER_PORT_NUMBER)
#define ETHERCAT_MASTER_IFNAME      "port0"
#define ETHERCAT_MASTER_PHY_INSTANCE g_ether_phy0
#elif (1U == ETHERCAT_MASTER_PORT_NUMBER)
#define ETHERCAT_MASTER_IFNAME      "port1"
#define ETHERCAT_MASTER_PHY_INSTANCE g_ether_phy1
#else
#define ETHERCAT_MASTER_IFNAME      "port2"
#define ETHERCAT_MASTER_PHY_INSTANCE g_ether_phy2
#endif

#if (0U == LWIP_ETHERNET_PORT_NUMBER)
#define LWIP_ETHERNET_PHY_INSTANCE g_ether_phy0
#elif (1U == LWIP_ETHERNET_PORT_NUMBER)
#define LWIP_ETHERNET_PHY_INSTANCE g_ether_phy1
#else
#define LWIP_ETHERNET_PHY_INSTANCE g_ether_phy2
#endif

#define ETHERCAT_MASTER_PORT_MASK   ETHER_NETIF_CFG_PORT_BIT(ETHERCAT_MASTER_PORT_NUMBER)
#define LWIP_ETHERNET_PORT_MASK     ETHER_NETIF_CFG_PORT_BIT(LWIP_ETHERNET_PORT_NUMBER)

#endif
