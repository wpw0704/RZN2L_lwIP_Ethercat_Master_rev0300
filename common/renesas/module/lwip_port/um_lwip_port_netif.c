/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_lwip_port_netif.c
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
#include "um_ether_netif_cfg.h"
#include "um_lwip_port.h"
#include "um_lwip_port_cfg.h"
#include "um_lwip_port_internal.h"

/** FreeRTOS related */
#include "FreeRTOS.h"        /** require including FreeRTOS.h before the following FreeRTOS resources. */
#include "event_groups.h"
#include "queue.h"

/** lwIP related */
#include "lwip/inet.h"
#include "lwip/netifapi.h"
#include "lwip/etharp.h"

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
/** Callback function when adding lwIP network interface. */
static err_t _netif_add_callback(struct netif *p_netif);

/** Callback functions called by lwIP network interface. */
static void  _status_changed(struct netif *netif);
static err_t _link_output(struct netif *p_netif, struct pbuf *p_packet);
struct netif *gp_lwip_netif;

/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_open
* Description  : Unlock updating instance chain list
* Arguments    : [in] p_args     Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*                USR_ERR_NOT_INITIALIZED        Initialization has been failed.
*********************************************************************************************************************/
usr_err_t um_lwip_port_netif_open( lwip_port_netif_ctrl_t * const p_ctrl,
        lwip_port_netif_cfg_t const * const p_cfg,
        lwip_port_ether_netif_ctrl_t * const p_ether_netif_ctrl,
        lwip_port_callback_ctrl_t * const p_callback_ctrl,
        lwip_port_launcher_ctrl_t * const p_launcher_ctrl )
{
    /** Status */
    err_t lwip_err;

    /** Set dependent module */
    p_ctrl->p_ether_netif_ctrl    = p_ether_netif_ctrl;
    p_ctrl->p_callback_ctrl       = p_callback_ctrl;
    p_ctrl->p_launcher_ctrl       = p_launcher_ctrl;
    p_ctrl->p_netif_cfg           = p_cfg;
    p_ctrl->p_ether_netif_frame   = NULL;
    gp_lwip_netif                 = &p_ctrl->netif;

    /** Set down state to network state. */
    p_ctrl->netif_state = LWIP_PORT_NETIF_STATE_DOWN;

    /** Add lwIP netif */
    lwip_err = netifapi_netif_add(
            &(p_ctrl->netif),
            /** CODE CHECKER, this is OK as a comment aligns with the cast*/
            (LWIP_PORT_DHCP_ENABLE == p_ctrl->p_netif_cfg->dhcp) ? NULL : (ip4_addr_t*) &p_cfg->ip_address,
            /** CODE CHECKER, this is OK as a comment aligns with the cast*/
            (LWIP_PORT_DHCP_ENABLE == p_ctrl->p_netif_cfg->dhcp) ? NULL : (ip4_addr_t*) &p_cfg->subnet_mask,
            /** CODE CHECKER, this is OK as a comment aligns with the cast*/
            (LWIP_PORT_DHCP_ENABLE == p_ctrl->p_netif_cfg->dhcp) ? NULL : (ip4_addr_t*) &p_cfg->gateway_address,
            p_ctrl, // Pass instance control to "netif->state" controlled in TCP/IP task context.
            (netif_init_fn) _netif_add_callback, // Pass callback function for setting network interface.
            /** CODE CHECKER, this is OK as a comment aligns with the cast*/
            (netif_input_fn) tcpip_input);

    USR_ERROR_RETURN( ERR_OK == lwip_err, USR_ERR_NOT_INITIALIZED );

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_get_netif_config
* Description  : Get the network interface configuration.
* Arguments    : [in] p_args     Pointer to the controller
*                [out] p_netif_cfg              Pointer to the netif configuration to be written.
* Return Value : USR_SUCCESS                    Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_netif_get_netif_config( lwip_port_netif_ctrl_t * const p_ctrl,
        lwip_port_netif_cfg_t * p_netif_cfg )
{
    p_netif_cfg->ip_address      = p_ctrl->netif.ip_addr.addr;
    p_netif_cfg->subnet_mask     = p_ctrl->netif.netmask.addr;
    p_netif_cfg->gateway_address = p_ctrl->netif.gw.addr;
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    p_netif_cfg->p_host_name     = (const uint8_t *) p_ctrl->netif.hostname;

    return USR_SUCCESS;
}


/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_set_netif_state
* Description  : Set the state of netif and notify application of it by callback.
* Arguments    : [in] p_args     Pointer to the controller
*                [in] state                     New state of netif.
* Return Value : USR_SUCCESS                    Process has been done successfully.
*                USR_ERR_ABORTED            Failed to set.
*********************************************************************************************************************/
usr_err_t um_lwip_port_netif_set_netif_state( lwip_port_netif_ctrl_t * const p_ctrl, lwip_port_netif_state_t state )
{
    /** Error status */
    usr_err_t usr_err = USR_SUCCESS;

    /** If new state is same as current state, return success code. */
    if( p_ctrl->netif_state == state )
    {
        return USR_SUCCESS;
    }

    /** Set new state */
    p_ctrl->netif_state = state;

    /** Notify the state change */
    switch ( p_ctrl->netif_state )
    {
        case LWIP_PORT_NETIF_STATE_UP:
            usr_err = um_lwip_port_callback_request(p_ctrl->p_callback_ctrl, LWIP_PORT_CALLBACK_EVENT_NETIF_UP );
            break;
        case LWIP_PORT_NETIF_STATE_DOWN:
            usr_err = um_lwip_port_callback_request(p_ctrl->p_callback_ctrl, LWIP_PORT_CALLBACK_EVENT_NETIF_DOWN );
            break;

        default:
            break;
    }
    USR_ERROR_RETURN( USR_SUCCESS == usr_err, USR_ERR_ABORTED );

    /** return success code */
    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_get_netif_state
* Description  : Get the network interface state, and notify callback it.
* Arguments    : [in] p_args     Pointer to the controller
*                [out] p_netif_state            Pointer to the netif state to be written.
* Return Value : USR_SUCCESS                    Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_netif_get_netif_state( lwip_port_netif_ctrl_t * const p_ctrl,
        lwip_port_netif_state_t * p_netif_state )
{
    /** Get netif state */
    *p_netif_state = p_ctrl->netif_state;

    return USR_SUCCESS;
}
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_set_default
* Description  : Set the specified lwIP network interface to default
* Arguments    : [in] p_args     Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*                USR_ERR_ABORTED            Failed to set.
*********************************************************************************************************************/
usr_err_t um_lwip_port_netif_set_default( lwip_port_netif_ctrl_t * const p_ctrl )
{
    /** Status */
    err_t lwip_err = ERR_OK;

    /** Set specified netif as default. */
    lwip_err = netifapi_netif_set_default(&(p_ctrl->netif));
    USR_ERROR_RETURN( ERR_OK == lwip_err, USR_ERR_ABORTED );

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_set_up
* Description  : Setup lwIP network interface.
* Arguments    : [in] p_args     Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*                USR_ERR_ABORTED            Failed to set.
*********************************************************************************************************************/
usr_err_t um_lwip_port_netif_set_up( lwip_port_netif_ctrl_t * const p_ctrl )
{
    /** Status */
    err_t lwip_err = ERR_OK;

    /** If netif is not up. */
    if( !netif_is_up( &( p_ctrl->netif ) ) )
    {
        /** Make netif up */
        lwip_err = netifapi_netif_set_up( &(p_ctrl->netif) );
        USR_ERROR_RETURN( ERR_OK == lwip_err, USR_ERR_ABORTED );
    }

    /** If netif is not link up. */
    if( !netif_is_link_up( &(p_ctrl->netif) ) )
    {
        /** Tell lwIP that the physical link is up. */
        lwip_err = netifapi_netif_set_link_up(&(p_ctrl->netif));
        USR_ERROR_RETURN( ERR_OK == lwip_err, USR_ERR_ABORTED );
    }

    if( LWIP_PORT_DHCP_ENABLE == p_ctrl->p_netif_cfg->dhcp )
    {
        /** Start DHCP */
        lwip_err = netifapi_dhcp_start(&(p_ctrl->netif));
        USR_ERROR_RETURN( ERR_OK == lwip_err, USR_ERR_ABORTED );
    }

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_set_down
* Description  : Set down lwIP network interface.
* Arguments    : [in] p_args     Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*                USR_ERR_ABORTED            Failed to set.
*********************************************************************************************************************/
usr_err_t um_lwip_port_netif_set_down( lwip_port_netif_ctrl_t * const p_ctrl )
{
    /** Status */
    err_t lwip_err = ERR_OK;

    /** Stop DHCP */
    if( LWIP_PORT_DHCP_ENABLE == p_ctrl->p_netif_cfg->dhcp )
    {
        lwip_err = netifapi_dhcp_release_and_stop( &(p_ctrl->netif) );
        USR_ERROR_RETURN( ERR_OK == lwip_err, USR_ERR_ABORTED );
    }

    /** Notify lwIP of that Ethernet interface is link down */
    if( netif_is_link_up( &(p_ctrl->netif) ) )
    {
        /** Tell lwIP that the physical link is down. */
        lwip_err = netifapi_netif_set_link_down( &(p_ctrl->netif) );
        USR_ERROR_RETURN( ERR_OK == lwip_err, USR_ERR_ABORTED );
    }

    /** Set down lwIP */
    if( netif_is_up( &(p_ctrl->netif) ) )
    {
        lwip_err = netifapi_netif_set_down( &(p_ctrl->netif) );
        USR_ERROR_RETURN( ERR_OK == lwip_err, USR_ERR_ABORTED );
    }

    return USR_SUCCESS;
}

/***********************************************************************************************************************
 * Private Functions [lwip IF]
 **********************************************************************************************************************/
/**********************************************************************************************************************
* Function Name: _netif_add_callback
* Description  :  Callback function that initializes the lwIP network interface.
* Arguments    : p_netif    Pointer to lwIP network interface structure
* Return Value : ERR_OK         Initialization was successful
*********************************************************************************************************************/
static err_t _netif_add_callback(struct netif *p_netif)
{
    /** Resolve the callback context pointer type. */
    lwip_port_netif_ctrl_t * p_ctrl = (lwip_port_netif_ctrl_t * ) p_netif->state;

    /** Set the interface name RE for Renesas Electronics */
    p_netif->name[0U] = 'R';
    p_netif->name[1U] = 'E';

    /** Set the host name */
#if LWIP_NETIF_HOSTNAME == (1)
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    p_netif->hostname = (const char*) p_ctrl->p_netif_cfg->p_host_name;
#endif

    /** Set hardware address information. */
    (void) um_lwip_port_ether_netif_get_local_mac_address(p_ctrl->p_ether_netif_ctrl, (uint8_t*) &p_netif->hwaddr);
    p_netif->hwaddr_len = NETIF_MAX_HWADDR_LEN;

    /** Use the lwIP Ethernet output function to resolve addresses.
        This then send the data to the driver. Use the function supplied by lwIP for Ethernet */
    p_netif->output = etharp_output;

    /** Set the device capabilities */
    p_netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

    /** Set the maximum transfer unit */
    p_netif->mtu = UM_ETHER_NETIF_CFG_MTU_BYTES;

    /** Specify the actual output function for V4 */
    p_netif->linkoutput = (netif_linkoutput_fn) _link_output;

    /** Set the callback when */
    p_netif->status_callback = (netif_status_callback_fn) _status_changed;

    return ERR_OK;
}

/**********************************************************************************************************************
* Function Name: _link_output
* Description  : Callback function outputting Ethernet frame called by lwIP.
* Arguments    : [in] p_netif    Pointer to lwIP network interface structure
* Return Value : ERR_OK         Initialization was successful
*********************************************************************************************************************/
static err_t _link_output(struct netif *p_netif, struct pbuf *p_packet)
{
    /** temp Buff */
    static ether_netif_frame_t ether_netif_frame;
    /** Resolve context pointer type */
    lwip_port_netif_ctrl_t * p_ctrl = (lwip_port_netif_ctrl_t * ) p_netif->state;


    /** Status */
    usr_err_t  usr_err;

    if(p_packet->next != NULL)
    {
        /** For concatenating the lwIP packet chain. */
        struct pbuf *p_packet_block   = p_packet;
        /** frame */
        uint8_t  *p_concat_frame;
        /** frame length */
        uint32_t  concat_frame_len = 0;

        /** Allocate buffer */

        /** Get the head address of frame buffer */
        p_concat_frame = ether_netif_frame.buffer;

        /** Integrate chained buffers into output buffer. */
        for( p_packet_block = p_packet; p_packet_block != NULL; p_packet_block = p_packet_block->next)
        {
            /** Copy into the destination buffer */
            memcpy(p_concat_frame, p_packet_block->payload, p_packet_block->len);

            /** Shift pointer of buffer according to */
            p_concat_frame += p_packet_block->len;             /** += means shifting pointer */

            /** Calculate the sum of packet length. */
            concat_frame_len += p_packet_block->len;        /** += means adding value */
        }

        /** If the concatenated packet length does not match to total length written in p_packet */
        if( concat_frame_len != p_packet->tot_len )
        {
            /** TODO: Implement Error handling in TCPIP task. */
        }
        ether_netif_frame.length      = concat_frame_len;
        ether_netif_frame.buffer_mode = 0;
        ether_netif_frame.p_buffer    = NULL;

    }
    else
    {
        ether_netif_frame.length      = p_packet->tot_len;
        ether_netif_frame.buffer_mode = ETHER_NETIF_CFG_FB_MODE_POINTER;
        /** CODE CHECKER, this is OK as a comment aligns with the cast*/
        ether_netif_frame.p_buffer   = (uint8_t *)p_packet->payload;
    }

    /** Send the frame to queue for buffering Ethernet frame to be send. */
    usr_err = um_lwip_port_ether_netif_send(p_ctrl->p_ether_netif_ctrl, &ether_netif_frame);
    if (USR_SUCCESS != usr_err)
    {
        /** TODO: Implement Error handling in TCPIP task. */
    }

    return ERR_OK;
}

/*******************************************************************************************************************//**
 * Callback function called by lwIP when changed network link state.
 *
 * @param[in] p_netif   Pointer to lwIP network interface structure
 **********************************************************************************************************************/
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_open
* Description  : Unlock updating instance chain list
* Arguments    : [in] p_args     Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*********************************************************************************************************************/
static void _status_changed(struct netif *p_netif)
{
    /** Resolve context pointer type */
    lwip_port_netif_ctrl_t * p_ctrl = (lwip_port_netif_ctrl_t * ) p_netif->state;

    /** The netif is up and link-up and IP address is set. */
    if ( netif_is_up( p_netif ) && (netif_ip4_addr( p_netif )->addr != INADDR_ANY) )
    {
        /** Request the link up event for launch callback when the IP address is set by DHCP */
        (void) um_lwip_port_task_launcher_request(p_ctrl->p_launcher_ctrl, LWIP_PORT_LAUNCHER_EVENT_IP_UP, p_ctrl);
    }
    /** other case */
    else
    {
        /** Request the link up event for launch callback when the IP address is set by DHCP */
        (void) um_lwip_port_task_launcher_request(p_ctrl->p_launcher_ctrl, LWIP_PORT_LAUNCHER_EVENT_IP_DOWN, p_ctrl);
    }
}
