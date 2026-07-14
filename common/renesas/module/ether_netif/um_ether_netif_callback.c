/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_ether_netif_callback.c
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
#include "um_ether_netif.h"
#include "um_ether_netif_cfg.h"
#include "um_ether_netif_internal.h"

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
* Function Name: um_ether_netif_callback_open
* Description  : Initialize the controller.
* Arguments    : Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*                USR_ERR_NOT_INITIALIZED        Initialization has been failed.
*********************************************************************************************************************/
usr_err_t um_ether_netif_callback_open( ether_netif_callback_ctrl_t * const p_ctrl )
{
    /** Create the mutex */
    p_ctrl->p_mutex_handle = xSemaphoreCreateMutex();
    USR_ERROR_RETURN( p_ctrl->p_mutex_handle, USR_ERR_NOT_INITIALIZED );

    /** Initialize the head/tail of node list. */
    p_ctrl->p_head_node = NULL;
    p_ctrl->p_tail_node = NULL;

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_ether_netif_callback_add
* Description  : Specify the callback function and the argument memory.
* Arguments    : Pointer to the controller, Pointer to the callback node
* Return Value : USR_SUCCESS           Process has been done successfully.
*                USR_ERR_IN_USE        Resources are busy.
*********************************************************************************************************************/
usr_err_t um_ether_netif_callback_add( ether_netif_callback_ctrl_t * const p_ctrl,
        ether_netif_callback_link_node_t * const p_new_nodes )
{
    ether_netif_callback_link_node_t * p_node;

    /** Lock section */
    xSemaphoreTake(p_ctrl->p_mutex_handle,portMAX_DELAY);

    /** If it's your first time setting */
    if ( NULL == p_ctrl->p_head_node )
    {
        /** Set the nodes as head node. */
        p_ctrl->p_head_node = p_new_nodes;
    }
    else
    {
        /** Add the new nodes after tail node */
        p_ctrl->p_tail_node->p_next = p_new_nodes;
    }

    /** Scan last node until next node is null. */
    for( p_node = p_ctrl->p_head_node; p_node->p_next != NULL; p_node = p_node->p_next );

    /** Update tail node */
    p_ctrl->p_tail_node = p_node;

    /** UNLOCK */
    xSemaphoreGive(p_ctrl->p_mutex_handle);

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_ether_netif_callback_request
* Description  : Request callback with specifying event and arguments.
* Arguments    : Pointer to the controller
*                Event to be set to callback memory.
*                Pointer to the frame packet to be set to callback memory.
*
* Return Value : USR_SUCCESS           Process has been done successfully.
*                USR_ERR_IN_USE        Resources are busy.
*********************************************************************************************************************/
usr_err_t um_ether_netif_callback_request( ether_netif_callback_ctrl_t * const p_ctrl,
        ether_netif_callback_event_t event, ether_netif_frame_t * p_frame_packet)
{
    /** Pointer for iterating node of link list. */
    ether_netif_callback_link_node_t *p_node;
    ether_netif_callback_link_node_t *p_node_lwip = NULL;

    /** Lock section */
    xSemaphoreTake(p_ctrl->p_mutex_handle,portMAX_DELAY);
    /** Iterate along link list */
    for (p_node = p_ctrl->p_head_node; p_node != NULL; p_node = p_node->p_next )
    {
        /** Set callback arguments with event. */
        p_node->p_memory->event     = event;
        p_node->p_memory->p_context = p_node->p_context;

        /** Set callback packet. */
        if ( p_frame_packet )
        {
            /** lwip ? */
            if((p_node_lwip == NULL) && (p_node->callback_buffer_mode & ETHER_NETIF_CFG_FB_MODE_LWIP))
            {
                p_node_lwip = p_node;
                continue;
            }
            /** If the callback node is last. */
            USR_HEAP_ALLOCATE( p_node->p_memory->p_frame_packet, sizeof(ether_netif_frame_t) );
            /** Send the copy packet. */
            if(p_node->callback_buffer_mode & ETHER_NETIF_CFG_FB_MODE_POINTER)
            {
                p_node->p_memory->p_frame_packet->p_buffer = p_frame_packet->buffer;
            }
            else
            {
                memcpy( p_node->p_memory->p_frame_packet->buffer,
                        p_frame_packet->p_buffer, p_frame_packet->length );
                p_node->p_memory->p_frame_packet->p_buffer = NULL;
            }
            p_node->p_memory->p_frame_packet->buffer_mode =
                    ETHER_NETIF_CFG_FB_MODE_LWIP | ETHER_NETIF_CFG_FB_MODE_POINTER;
            p_node->p_memory->p_frame_packet->length = p_frame_packet->length;
            p_node->p_memory->p_frame_packet->port   = p_frame_packet->port;
        }
        else
        {
            p_node->p_memory->p_frame_packet = NULL;
        }

        /** Execute callback function with the argument. */
        (void) p_node->p_func( p_node->p_memory );
    }
    if(p_node_lwip)
    {
        /** lwip call back */
        p_node_lwip->p_memory->p_frame_packet = p_frame_packet;
        (void) p_node_lwip->p_func( p_node_lwip->p_memory );
    }
    /** UNLOCK */
    xSemaphoreGive(p_ctrl->p_mutex_handle);

    return USR_SUCCESS;
}

/***********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/
