/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_lwip_port_core.c
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
#include "um_lwip_port_internal.h"

#include "FreeRTOS.h"
/** require including FreeRTOS.h before the following FreeRTOS resources. */
#include "event_groups.h"
#include "queue.h"

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
/** Callback functions when initializing lwIP TCP/IP stack task. */
static void tcpip_init_callback(void * arg);
static void tcpip_task_sync_callback( void *ctx );

/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/
/**********************************************************************************************************************
* Function Name: um_lwip_port_core_open
* Description  : Initialize the controller.
* Arguments    : [in] p_ctrl               Pointer to the controller
* Return Value : USR_SUCCESS                Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_core_open( lwip_port_core_ctrl_t * const p_ctrl )
{
    /** Set the handler of current task. */
    p_ctrl->p_parent_task_handle = xTaskGetCurrentTaskHandle();

    /** Power on lwIP and create task of lwIP whose mode is not NO_SYS. */
    (void) tcpip_init( (tcpip_init_done_fn) tcpip_init_callback, (void*) p_ctrl);

    /** Wait for notification indicating the created task is initialized */
    (void) ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

    return USR_SUCCESS;
}

/*******************************************************************************************************************//**
 * Synchronize with TCPIP task.
 **********************************************************************************************************************/
/**********************************************************************************************************************
* Function Name: um_lwip_port_core_task_sync
* Description  : Synchronize with TCPIP task.
* Arguments    : None
* Return Value : USR_SUCCESS                Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_core_task_sync( void )
{
    tcpip_callback(tcpip_task_sync_callback, xTaskGetCurrentTaskHandle());
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    return USR_SUCCESS;
}

/***********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/
/**********************************************************************************************************************
* Function Name: tcpip_init_callback
* Description  : Callback function called by lwIP TCP/IP API tcpip_init() on TCP/IP stack task context.
* Arguments    : [in] arg              Pointer to lwIP network interface structure
* Return Value : None
*********************************************************************************************************************/
static void tcpip_init_callback(void * arg)
{
    /** Resolve callback context pointer */
    lwip_port_core_ctrl_t * p_ctrl = arg;

    /** Get lwIP TCP/IP Task handler. */
    p_ctrl->p_tcpip_task_handle = xTaskGetCurrentTaskHandle();

    /** Notify the parenet task launch of this task. */
    xTaskNotifyGive( p_ctrl->p_parent_task_handle );
}

/**********************************************************************************************************************
* Function Name: tcpip_task_sync_callback
* Description  : Synchronize with TCPIP task.
* Arguments    : [in] ctx     Callback context.
* Return Value : None
*********************************************************************************************************************/
static void tcpip_task_sync_callback( void *ctx )
{
    /** Resolve context */
    TaskHandle_t sync_task = (TaskHandle_t) ctx;

    /** Notify the task asking the TCPIP task call this functions. */
    xTaskNotifyGive( sync_task );
}
