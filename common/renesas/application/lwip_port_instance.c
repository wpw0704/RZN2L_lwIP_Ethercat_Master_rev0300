/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :lwip_port_instance.c
 * Version      :1
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History :
 *
 *
 *********************************************************************************************************************/

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
/** FSP module instances. */
#include "hal_data.h"
#include "common_data.h"
#include "main_thread.h"

/** User module instance APIs. */
#include "um_common.h"

#include "um_serial_io_api.h"

#include "um_lwip_port_api.h"
#include "um_lwip_port.h"

#include "um_ether_netif_api.h"
#include "um_ether_netif.h"

#include "lwip/def.h"
#include "lwip/inet.h"
#include "um_ether_netif_api.h"
#include "lwip_port_main_api.h"

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * Private constants
 **********************************************************************************************************************/
/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * Global Variables
 **********************************************************************************************************************/
/**
 * Serial IO module instance
 */

/**
 * Ethernet network interface module instance
 */
static ether_netif_instance_ctrl_t g_ether_netif0_ctrl;
static ether_netif_cfg_t const     g_ether_netif0_cfg =
{
    .p_ether_instance = &g_ether0,
};
ether_netif_instance_t const g_ether_netif0 =
{
    .p_ctrl = &g_ether_netif0_ctrl,
    .p_cfg  = &g_ether_netif0_cfg,
    .p_api  = &g_ether_netif_on_ether_netif
};

/**
 * lwIP network interface module instance
 */
static lwip_port_instance_ctrl_t g_lwip_port0_ctrl;
static lwip_port_common_ctrl_t   g_lwip_port0_common_ctrl;

static uint8_t               gp_lwip_port0_hostname[16] = "LWIP_NETIF0";

static lwip_port_netif_cfg_t g_lwip_port0_netif_cfg =
{
        .dhcp            = LWIP_PORT_DHCP_DISABLE,
        .ip_address      = PP_HTONL(LWIP_MAKEU32(192,168,  10,170)),
        .subnet_mask     = PP_HTONL(LWIP_MAKEU32(255,255,255,  0)),
        .gateway_address = PP_HTONL(LWIP_MAKEU32(192,168,  10,  1)),
        .p_host_name     = gp_lwip_port0_hostname
};

static lwip_port_common_cfg_t const g_lwip_port0_common_cfg =
{
        .p_ether_netif_instance = &g_ether_netif0,
        .p_common_ctrl = &g_lwip_port0_common_ctrl,
};

static lwip_port_cfg_t const g_lwip_port0_cfg =
{
        .p_netif_cfg  = &g_lwip_port0_netif_cfg,
        .p_common_cfg = &g_lwip_port0_common_cfg,
};

lwip_port_instance_t const g_lwip_port0 =
{
        .p_ctrl = &g_lwip_port0_ctrl,
        .p_cfg  = &g_lwip_port0_cfg,
        .p_api  = &g_lwip_port_on_lwip_port
};

/**
 * Extern pointers.
 */
ether_netif_instance_t const * gp_ether_netif0 = &g_ether_netif0;
lwip_port_instance_t   const * gp_lwip_port0   = &g_lwip_port0;

/**********************************************************************************************************************
* Function Name: lwip_port_user_instance_init
* Description  : Initialize your device and make it available for use
* Arguments    : None
* Return Value : usr_err_t
*********************************************************************************************************************/
usr_err_t lwip_port_user_instance_init(void)
{
    /** Error status */
    usr_err_t usr_err = USR_SUCCESS;

    /******************************************************************************************************************
     * Startup Serial I/O module for debug print
     ******************************************************************************************************************/
    /** Open Serial I/O module. */
    usr_err = um_serial_io_uart_open( &g_uart0);
    if( USR_SUCCESS != usr_err )
    {
        /** Note:Error Handling  */
        return usr_err;
    }

    USR_LOG_INFO( "Started Serial I/O interface." );

    /******************************************************************************************************************
     * Startup Ethernet network interface.
     ******************************************************************************************************************/
    /** Open Ethernet network interface. */

    usr_err = gp_ether_netif0->p_api->open(gp_ether_netif0->p_ctrl, gp_ether_netif0->p_cfg);
    if ( USR_SUCCESS != usr_err )
    {
        USR_LOG_ERROR( "Failed to initialize Ethernet network interface." );
        return usr_err;
    }
    /** Setting the receive buffer to use lwip */
    USR_LOG_INFO( "Initialized Ethernet network interface." );

    /******************************************************************************************************************
     * Startup lwIP TCP/IP network interface.
     ******************************************************************************************************************/
    /** Open lwIP TCP/IP network interface. */
    usr_err = gp_lwip_port0->p_api->open(gp_lwip_port0->p_ctrl, gp_lwip_port0->p_cfg);
    if ( USR_SUCCESS != usr_err )
    {
        USR_LOG_ERROR( "Failed to initialize lwIP TCP/IP stack." );
        return usr_err;
    }
    USR_LOG_INFO( "Initialized lwIP TCP/IP stack." );

    /** Start lwIP TCP/IP network interface. */
    usr_err = gp_lwip_port0->p_api->start(gp_lwip_port0->p_ctrl);
    if ( USR_SUCCESS != usr_err )
    {
        USR_LOG_ERROR( "Failed to start the lwIP TCP/IP stack." );
        return usr_err;
    }

    /** Start Ethernet network interface. */
    usr_err = gp_ether_netif0->p_api->start(gp_ether_netif0->p_ctrl);
    if ( USR_SUCCESS != usr_err )
    {
        USR_LOG_ERROR( "Failed to Ethernet network interface." );
        return usr_err;
    }
    USR_LOG_INFO( "Started Ethernet network interface." );
    return usr_err;

}
