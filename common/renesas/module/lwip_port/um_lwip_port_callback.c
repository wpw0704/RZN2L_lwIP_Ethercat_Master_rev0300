/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_lwip_port_callback.c
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
#include "FreeRTOS.h"
#include "um_lwip_port_internal.h"

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
* Function Name: um_lwip_port_callback_open
* Description  : Initialize the controller.
* Arguments    : [in] p_ctrl               Pointer to the callback controller
* Return Value : USR_SUCCESS                Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_callback_open( lwip_port_callback_ctrl_t * const p_ctrl )
{
    /** Create the mutex */
    p_ctrl->p_mutex_handle = xSemaphoreCreateMutex();
    USR_ERROR_RETURN( p_ctrl->p_mutex_handle, USR_ERR_NOT_INITIALIZED );

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_callback_add
* Description  : Specify the callback function and the argument memory.
* Arguments    : [in] p_ctrl               Pointer to the callback controller
*                [in] p_node               Pointer to the callback link
* Return Value : USR_SUCCESS                Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_callback_add( lwip_port_callback_ctrl_t * const p_ctrl,
        lwip_port_callback_link_node_t * p_new_nodes )
{
    lwip_port_callback_link_node_t * p_node;

    /** Lock section */
    xSemaphoreTake(p_ctrl->p_mutex_handle,portMAX_DELAY);

    /** Set callback by adding callback node into link list. */
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
* Function Name: um_lwip_port_callback_request
* Description  : Request callback with specifying event and arguments.
* Arguments    : p_ctrl            Pointer to the controller
*                event             Event to be set to callback memory.
* Return Value : USR_SUCCESS                Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_callback_request( lwip_port_callback_ctrl_t * const p_ctrl,
        lwip_port_callback_event_t event )
{
    /** Pointer for iterating node of link list. */
    lwip_port_callback_link_node_t * p_node;

    /** Lock section */
    xSemaphoreTake(p_ctrl->p_mutex_handle,portMAX_DELAY);

    /** Iterate along link list */
    for ( p_node = p_ctrl->p_head_node; p_node != NULL; p_node = p_node->p_next )
    {
        /** Set callback arguments with event. */
        p_node->p_memory->event     = event;
        p_node->p_memory->p_context = p_node->p_context;

        /** Execute callback function with the argument. */
        (void) p_node->p_func( p_node->p_memory );
    }

    /** UNLOCK */
    xSemaphoreGive(p_ctrl->p_mutex_handle);

    return USR_SUCCESS;
}

/***********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/
