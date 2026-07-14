/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_lwip_port_ether_netif.c
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
#include "um_lwip_port_api.h"
#include "um_lwip_port_cfg.h"
#include "um_lwip_port.h"
#include "um_lwip_port_internal.h"
#include "um_ether_netif_api.h"
#include "um_ether_netif.h"


#include "FreeRTOS.h"

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
 * Functions
 **********************************************************************************************************************/

/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_open
* Description  : Initialize the controller.
* Arguments    : [in] p_ctrl                    Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*                USR_ERR_NOT_INITIALIZED        Initialization has been failed.
*                USR_ERR_ALREADY_OPEN            Specified module has been already opened.
*********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_open( lwip_port_ether_netif_ctrl_t * const p_ctrl,
        lwip_port_launcher_ctrl_t * p_launcher_ctrl,
        lwip_port_receiver_ctrl_t * p_reader_ctrl,
        ether_netif_instance_t const * const p_ether_netif_instance)
{
    /** Error codes */
    usr_err_t usr_err;

    /** Set related module controls. */
    p_ctrl->p_ether_netif_instance = (ether_netif_instance_t *)p_ether_netif_instance;
    p_ctrl->p_launcher_ctrl        = p_launcher_ctrl;
    p_ctrl->p_receiver_ctrl        = p_reader_ctrl;

    /** Try opening Ethernet network interface module */
    usr_err = p_ctrl->p_ether_netif_instance->p_api->open(
            p_ctrl->p_ether_netif_instance->p_ctrl, p_ctrl->p_ether_netif_instance->p_cfg );
    USR_ERROR_RETURN( USR_SUCCESS == usr_err || USR_ERR_ALREADY_OPEN == usr_err ||
            USR_ERR_ALREADY_RUNNING == usr_err , USR_ERR_NOT_INITIALIZED );

    /** RX callback is intentionally not registered in the current EtherCAT port monitor stage. */
    (void) p_launcher_ctrl;
    (void) p_reader_ctrl;
    /** Return success code */
    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_start
* Description  : Start the network interface submodule.
* Arguments    : [in] p_ctrl                    Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_start( lwip_port_ether_netif_ctrl_t * const p_ctrl )
{
    /** link status of all ports */
    uint32_t link_status;

    /** Start Ethernet network interface module. */
    (void) p_ctrl->p_ether_netif_instance->p_api->start(p_ctrl->p_ether_netif_instance->p_ctrl);

    /** Get link state. */
    (void) p_ctrl->p_ether_netif_instance->p_api->linkStatusGet(
            p_ctrl->p_ether_netif_instance->p_ctrl, &link_status, true );

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_stop
* Description  : Stop the network interface submodule.
* Arguments    : [in] p_ctrl                    Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_stop( lwip_port_ether_netif_ctrl_t * const p_ctrl )
{
    /** Error codes */
    usr_err_t usr_err;

    /** Start Ethernet network interface module. */
    usr_err = p_ctrl->p_ether_netif_instance->p_api->stop(p_ctrl->p_ether_netif_instance->p_ctrl);
    USR_ERROR_RETURN( USR_SUCCESS == usr_err, USR_SUCCESS /** Already stopped */ );

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_close
* Description  : Stop the network interface submodule.
* Arguments    : [in] p_ctrl                    Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_close( lwip_port_ether_netif_ctrl_t * const p_ctrl )
{
    /** Error codes */
    usr_err_t usr_err;

    /** Start Ethernet network interface module. */
    usr_err = p_ctrl->p_ether_netif_instance->p_api->close(p_ctrl->p_ether_netif_instance->p_ctrl);
    USR_ERROR_RETURN( USR_SUCCESS == usr_err, USR_SUCCESS /** Already closed */ );

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_send
* Description  : Request the Ethernet Link module to send Ethernet frame.
* Arguments    : [in] p_ctrl                    Pointer to the controller
*                [in] p_packet_buffer    Pointer to packet buffer.
* Return Value : USR_SUCCESS                    Process has been done successfully.
*                USR_ERR_ABORTED        Process has been aborted.
*********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_send( lwip_port_ether_netif_ctrl_t * const p_ctrl,
        ether_netif_frame_t * const p_packet_buffer )
{
    usr_err_t usr_err;

    /** lwIP uses port0 only; EtherCAT master uses port1. */
    p_packet_buffer->port = ETHER_NETIF_CFG_PORT_BIT(0);

    /** Send the frame */
    usr_err = p_ctrl->p_ether_netif_instance->p_api->send( p_ctrl->p_ether_netif_instance->p_ctrl, p_packet_buffer );
    USR_ERROR_RETURN( USR_SUCCESS == usr_err, USR_ERR_ABORTED );

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_get_local_mac_address
* Description  : Get the device host local MAC address
* Arguments    : [in] p_ctrl                    Pointer to the controller
*                [out] p_mac_address    Pointer to MAC address array pointer to be set the host MAC address
* Return Value : USR_SUCCESS                    Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_get_local_mac_address(
        lwip_port_ether_netif_ctrl_t * const p_ctrl, uint8_t * const p_mac_address )
{
    /** Get Mac address configuration */
    memcpy( p_mac_address, p_ctrl->p_ether_netif_instance->p_cfg->p_ether_instance->p_cfg->p_mac_address,
            sizeof(uint8_t) * ETHER_NETIF_CFG_MAC_ADDRESS_BYTES);

    return USR_SUCCESS;
}

/***********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/
