/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_ether_netif_internal.h
 * Version      :1
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History :
 *
 *
 *********************************************************************************************************************/

#ifndef UM_ETHER_NETIF_INTERNAL_H
#define UM_ETHER_NETIF_INTERNAL_H

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "lwip/etharp.h"
#include "um_ether_netif.h"
#include "um_ether_netif_api.h"

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
 * Function prototypes
 **********************************************************************************************************************/
/** Callback control functions */
/**********************************************************************************************************************
* Function Name: um_ether_netif_callback_open
*********************************************************************************************************************/
usr_err_t um_ether_netif_callback_open( ether_netif_callback_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_ether_netif_callback_add
*********************************************************************************************************************/
usr_err_t um_ether_netif_callback_add( ether_netif_callback_ctrl_t * const p_ctrl,
        ether_netif_callback_link_node_t * const p_node );
/**********************************************************************************************************************
* Function Name: um_ether_netif_callback_request
*********************************************************************************************************************/
usr_err_t um_ether_netif_callback_request( ether_netif_callback_ctrl_t * const p_ctrl,
        ether_netif_callback_event_t event, ether_netif_frame_t * p_frame_packet);

/** Target driver control functions */
/**********************************************************************************************************************
* Function Name: um_ether_netif_ether_open
*********************************************************************************************************************/
usr_err_t um_ether_netif_ether_open(ether_netif_ether_ctrl_t * const p_ctrl,
        ether_instance_t const * const p_ether_instance, ether_netif_reader_ctrl_t * const p_reader_ctrl);
/**********************************************************************************************************************
* Function Name: um_ether_netif_ether_read
*********************************************************************************************************************/
usr_err_t um_ether_netif_ether_read(ether_netif_ether_ctrl_t * const p_ctrl, ether_netif_frame_t * const p_frame );
/**********************************************************************************************************************
* Function Name: um_ether_netif_ether_write
*********************************************************************************************************************/
usr_err_t um_ether_netif_ether_write(ether_netif_ether_ctrl_t * const p_ctrl, ether_netif_frame_t * const p_frame);
/**********************************************************************************************************************
* Function Name: um_ether_netif_ether_monitor
*********************************************************************************************************************/
usr_err_t um_ether_netif_ether_monitor (ether_netif_ether_ctrl_t * const p_ctrl, uint32_t * p_link_status );
/**********************************************************************************************************************
* Function Name: um_ether_netif_ether_set_callback_context
*********************************************************************************************************************/
usr_err_t um_ether_netif_ether_set_callback_context(ether_netif_ether_ctrl_t * const p_ctrl,
        void * const p_callback_context );

/** Monitor task control functions */
/**********************************************************************************************************************
* Function Name: um_ether_netif_task_monitor_open
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_monitor_open( ether_netif_monitor_ctrl_t * const p_ctrl,
        ether_netif_ether_ctrl_t * const p_target_ctrl, ether_netif_callback_ctrl_t * const p_callback_ctrl );
/**********************************************************************************************************************
* Function Name: um_ether_netif_task_monitor_start
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_monitor_start( ether_netif_monitor_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_ether_netif_task_monitor_stop
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_monitor_stop( ether_netif_monitor_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_ether_netif_task_monitor_get_link_status
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_monitor_get_link_status ( ether_netif_monitor_ctrl_t * const p_ctrl,
        uint32_t * p_link_status);

/** Reader task control functions */
/**********************************************************************************************************************
* Function Name: um_ether_netif_task_reader_open
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_reader_open( ether_netif_reader_ctrl_t * const p_ctrl,
        ether_netif_ether_ctrl_t * const p_target_ctrl, ether_netif_callback_ctrl_t * const p_callback_ctrl );
/**********************************************************************************************************************
* Function Name: um_ether_netif_task_reader_start
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_reader_start( ether_netif_reader_ctrl_t * const p_instance_ctrl );
/**********************************************************************************************************************
* Function Name: um_ether_netif_task_reader_stop
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_reader_stop( ether_netif_reader_ctrl_t * const p_instance_ctrl );
/**********************************************************************************************************************
* Function Name: um_ether_netif_task_reader_get_task_handle
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_reader_get_task_handle( ether_netif_reader_ctrl_t * const p_ctrl,
        TaskHandle_t * pp_task_handle );

/** Writer task control functions */
/**********************************************************************************************************************
* Function Name: um_ether_netif_task_writer_open
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_writer_open( ether_netif_writer_ctrl_t * const p_ctrl,
        ether_netif_ether_ctrl_t * const p_target_ctrl );
/**********************************************************************************************************************
* Function Name: um_ether_netif_task_writer_start
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_writer_start( ether_netif_writer_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_ether_netif_task_writer_stop
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_writer_stop( ether_netif_writer_ctrl_t * const p_ctrl );
/**********************************************************************************************************************
* Function Name: um_ether_netif_task_writer_request
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_writer_request( ether_netif_writer_ctrl_t * const p_ctrl,
        ether_netif_frame_t * p_packet_buffer );

#endif /** UM_ETHER_NETIF_INTERNAL_H */
