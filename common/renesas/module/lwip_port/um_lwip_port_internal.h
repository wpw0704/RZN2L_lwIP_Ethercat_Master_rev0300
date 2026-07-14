/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_lwip_port_internal.h
 * Version      :1
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History :
 *
 *
 *********************************************************************************************************************/

#ifndef UM_LWIP_PORT_INTERNAL_H
#define UM_LWIP_PORT_INTERNAL_H

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "um_lwip_port.h"
#include "um_lwip_port_api.h"

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
/**
 * Enumeration for the state indicating whether netif link is up or down
 */
typedef enum e_lwip_port_launcher_event
{
    LWIP_PORT_LAUNCHER_EVENT_LINK_UP,     ///< Event when link is up
    LWIP_PORT_LAUNCHER_EVENT_LINK_DOWN,   ///< Event when link is down
    LWIP_PORT_LAUNCHER_EVENT_IP_UP,       ///< Event when IP is up
    LWIP_PORT_LAUNCHER_EVENT_IP_DOWN,     ///< Event when IP is down
} lwip_port_launcher_event_t;

typedef struct st_lwip_port_launcher_request
{
    lwip_port_launcher_event_t  event;
    lwip_port_netif_ctrl_t    * p_netif_ctrl;
} lwip_port_launcher_request_t;

/***********************************************************************************************************************
 * Private constants
 **********************************************************************************************************************/
/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * Function prototypes
 **********************************************************************************************************************/
/** lwIP interface */
/**********************************************************************************************************************
* Function Name: um_lwip_port_core_open
**********************************************************************************************************************/
usr_err_t um_lwip_port_core_open( lwip_port_core_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_core_task_sync
**********************************************************************************************************************/
usr_err_t um_lwip_port_core_task_sync( void );

/** lwIP network interface */
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_open
**********************************************************************************************************************/
usr_err_t um_lwip_port_netif_open( lwip_port_netif_ctrl_t * const p_ctrl,
        lwip_port_netif_cfg_t const * const p_cfg,
        lwip_port_ether_netif_ctrl_t * const p_ether_netif_ctrl,
        lwip_port_callback_ctrl_t * const p_callback_ctrl,
        lwip_port_launcher_ctrl_t * const p_launcher_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_input
**********************************************************************************************************************/
usr_err_t um_lwip_port_netif_input( lwip_port_netif_ctrl_t * const p_ctrl, ether_netif_frame_t * p_frame_packet );
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_set_default
**********************************************************************************************************************/
usr_err_t um_lwip_port_netif_set_default( lwip_port_netif_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_set_up
**********************************************************************************************************************/
usr_err_t um_lwip_port_netif_set_up( lwip_port_netif_ctrl_t *const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_set_down
**********************************************************************************************************************/
usr_err_t um_lwip_port_netif_set_down( lwip_port_netif_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_get_netif_config
**********************************************************************************************************************/
usr_err_t um_lwip_port_netif_get_netif_config(
        lwip_port_netif_ctrl_t * const p_ctrl, lwip_port_netif_cfg_t * p_netif_cfg );
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_set_netif_state
**********************************************************************************************************************/
usr_err_t um_lwip_port_netif_set_netif_state( lwip_port_netif_ctrl_t * const p_ctrl, lwip_port_netif_state_t state );
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_get_netif_state
**********************************************************************************************************************/
usr_err_t um_lwip_port_netif_get_netif_state( lwip_port_netif_ctrl_t * const p_ctrl,
        lwip_port_netif_state_t * p_netif_state );

/** list interface of lwIP network interface */
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_list_open
**********************************************************************************************************************/
usr_err_t um_lwip_port_netif_list_open( lwip_port_netif_list_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_list_add
**********************************************************************************************************************/
usr_err_t um_lwip_port_netif_list_add( lwip_port_netif_list_ctrl_t * const p_ctrl,
        lwip_port_netif_ctrl_t * const p_netif_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_list_lock
**********************************************************************************************************************/
usr_err_t um_lwip_port_netif_list_lock( lwip_port_netif_list_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_list_unlock
**********************************************************************************************************************/
usr_err_t um_lwip_port_netif_list_unlock( lwip_port_netif_list_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_netif_list_map
**********************************************************************************************************************/
usr_err_t um_lwip_port_netif_list_map( lwip_port_netif_list_ctrl_t * const p_ctrl,
        usr_err_t (* p_callback)( lwip_port_netif_ctrl_t * p_netif_ctrl, void * p_context), void * p_context );

/** Ethernet Link interface */
/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_open
**********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_open( lwip_port_ether_netif_ctrl_t * const p_ctrl,
        lwip_port_launcher_ctrl_t * p_launcher_ctrl,
        lwip_port_receiver_ctrl_t * p_reader_ctrl,
        ether_netif_instance_t const * const p_ether_netif_instance);
/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_close
**********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_close( lwip_port_ether_netif_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_start
**********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_start( lwip_port_ether_netif_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_stop
**********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_stop( lwip_port_ether_netif_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_send
**********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_send( lwip_port_ether_netif_ctrl_t * const p_ctrl,
        ether_netif_frame_t * const p_packet_buffer );
/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_get_local_mac_address
**********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_get_local_mac_address( lwip_port_ether_netif_ctrl_t * const p_ctrl,
        uint8_t * const p_mac_address );

/** Callback interface */
/**********************************************************************************************************************
* Function Name: um_lwip_port_callback_open
**********************************************************************************************************************/
usr_err_t um_lwip_port_callback_open( lwip_port_callback_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_callback_add
**********************************************************************************************************************/
usr_err_t um_lwip_port_callback_add( lwip_port_callback_ctrl_t * const p_ctrl,
        lwip_port_callback_link_node_t * p_node );
/**********************************************************************************************************************
* Function Name: um_lwip_port_callback_request
**********************************************************************************************************************/
usr_err_t um_lwip_port_callback_request( lwip_port_callback_ctrl_t * const p_ctrl,
        lwip_port_callback_event_t event );

/** Launcher task interface */
/**********************************************************************************************************************
* Function Name: um_lwip_port_task_launcher_open
**********************************************************************************************************************/
usr_err_t um_lwip_port_task_launcher_open( lwip_port_launcher_ctrl_t * const p_ctrl,
        lwip_port_netif_list_ctrl_t * const p_netif_list_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_task_launcher_start
**********************************************************************************************************************/
usr_err_t um_lwip_port_task_launcher_start( lwip_port_launcher_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_task_launcher_stop
**********************************************************************************************************************/
usr_err_t um_lwip_port_task_launcher_stop( lwip_port_launcher_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_task_launcher_request
**********************************************************************************************************************/
usr_err_t um_lwip_port_task_launcher_request( lwip_port_launcher_ctrl_t * const p_ctrl,
        lwip_port_launcher_event_t const event,lwip_port_netif_ctrl_t * const p_netif_ctrl);

/** Reader task interface */
/**********************************************************************************************************************
* Function Name: um_lwip_port_task_receiver_open
**********************************************************************************************************************/
usr_err_t um_lwip_port_task_receiver_open( lwip_port_receiver_ctrl_t * const p_ctrl,
        lwip_port_netif_list_ctrl_t * const p_instance_manager_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_task_receiver_start
**********************************************************************************************************************/
usr_err_t um_lwip_port_task_receiver_start( lwip_port_receiver_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_task_receiver_stop
**********************************************************************************************************************/
usr_err_t um_lwip_port_task_receiver_stop( lwip_port_receiver_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_lwip_port_task_receiver_request
**********************************************************************************************************************/
usr_err_t um_lwip_port_task_receiver_request( lwip_port_receiver_ctrl_t * const p_ctrl,
        ether_netif_frame_t * p_packet_buffer );

#endif /** UM_LWIP_PORT_INTERNAL_H */
