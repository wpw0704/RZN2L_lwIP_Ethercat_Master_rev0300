/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_ether_netif_task_monitor.c
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
/** Ethernet link check intervals */
#define ETHER_LINK_CHECK_INTERVAL_MS        ( 1000 )
/** [ticks] = [ms] * [ticks/ms] */
#define ETHER_LINK_CHECK_INTERVAL_TICKS     ( (TickType_t)( ETHER_LINK_CHECK_INTERVAL_MS * \
        configTICK_RATE_HZ / 1000 ) )

/***********************************************************************************************************************
 * Private constants
 **********************************************************************************************************************/
/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
/** Task functions to handling Ethernet input, output and link. */
static void task_code(void * pvParameter);

/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/
/**********************************************************************************************************************
* Function Name: um_ether_netif_task_monitor_open
* Description  : Create the task and initialize its controller.
* Arguments    : [in] p_ctrl               Pointer to the controller
*                [in] p_target_ctrl        Pointer to target controller
*                [in] p_callback_ctrl      Pointer to callback controller
* Return Value : USR_SUCCESS                Process has been done successfully.
*                USR_ERR_NOT_INITIALIZED    Initialization has been failed.
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_monitor_open( ether_netif_monitor_ctrl_t * const p_ctrl,
                                            ether_netif_ether_ctrl_t * const p_target_ctrl,
                                            ether_netif_callback_ctrl_t * const p_callback_ctrl )
{
    BaseType_t rtos_err;

    /** Set target and callback interface controller. */
    p_ctrl->p_ether_ctrl         = p_target_ctrl;
    p_ctrl->p_callback_ctrl      = p_callback_ctrl;
    p_ctrl->p_parent_task_handle = xTaskGetCurrentTaskHandle();

    /** Create network link task */
    rtos_err = xTaskCreate(task_code,
        UM_ETHER_NETIF_CFG_MONITOR_TASK_NAME,
        UM_ETHER_NETIF_CFG_MONITOR_TASK_STACK_BTYE_SIZE / sizeof(StackType_t),
        p_ctrl, UM_ETHER_NETIF_CFG_MONITOR_TASK_PRIORITY,
        &(p_ctrl->p_task_handle) );
    USR_ERROR_RETURN( pdPASS == rtos_err, USR_ERR_NOT_INITIALIZED );
    
    /** Wait for notification indicating the created task is initialized. */
    (void) ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    
    /** Suspend the created task. */
    vTaskSuspend( p_ctrl->p_task_handle );

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_ether_netif_task_monitor_start
* Description  : Start the task.
* Arguments    : [in] p_ctrl               Pointer to the controller
* Return Value : USR_SUCCESS                Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_monitor_start( ether_netif_monitor_ctrl_t * const p_ctrl )
{
    vTaskResume(p_ctrl->p_task_handle);
    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_ether_netif_task_monitor_stop
* Description  : Stop the task.
* Arguments    : [in] p_ctrl               Pointer to the controller
* Return Value : USR_SUCCESS                Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_monitor_stop( ether_netif_monitor_ctrl_t * const p_ctrl )
{
    vTaskSuspend(p_ctrl->p_task_handle);
    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_ether_netif_task_monitor_get_link_status
* Description  : Get link status of all ports
* Arguments    : [in] p_ctrl               Pointer to the controller
*                [out] p_link_status            Pointer to link status of all ports
* Return Value : USR_SUCCESS                Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_monitor_get_link_status ( ether_netif_monitor_ctrl_t * const p_ctrl,
        uint32_t * p_link_status)
{
    * p_link_status = p_ctrl->link_status;
    return USR_SUCCESS;
}

/***********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/
/**********************************************************************************************************************
* Function Name: task_code
* Description  : RTOS task.
* Arguments    : [in] pvParameter       Pointer to task parameters.
* Return Value : None
*********************************************************************************************************************/
static void task_code (void * pvParameter)
{
    /** Resolve task parameter */
    ether_netif_monitor_ctrl_t * const p_ctrl = (ether_netif_monitor_ctrl_t * const) pvParameter;

    /** Status */
    BaseType_t usr_err;

    /** Callback event */
    ether_netif_callback_event_t callback_event = ETHER_NETIF_CALLBACK_EVENT_LINK_DOWN;
    /** */
    ether_netif_callback_event_t callback_event_prev = ETHER_NETIF_CALLBACK_EVENT_LINK_DOWN;

    /** Notify the parent task launch of this task. */
    xTaskNotifyGive( p_ctrl->p_parent_task_handle );

    while(true)
    {
        /** Delay task */
        vTaskDelay(ETHER_LINK_CHECK_INTERVAL_TICKS);

        /** Check link state */
        usr_err = um_ether_netif_ether_monitor( p_ctrl->p_ether_ctrl, &p_ctrl->link_status );

        /** Decide link event */
        callback_event = ( USR_SUCCESS == usr_err ) ?
            ETHER_NETIF_CALLBACK_EVENT_LINK_UP : ETHER_NETIF_CALLBACK_EVENT_LINK_DOWN;

        /** Issue the event with callback when the link state changes. */
        if( callback_event != callback_event_prev )
        {
            /** Request the callback with lock for notifying the frame reception with the frame. */
            usr_err = um_ether_netif_callback_request(
                (ether_netif_callback_ctrl_t * ) p_ctrl->p_callback_ctrl, callback_event, NULL );
        }

        /** Update previous callback event. */
        callback_event_prev = callback_event;
    }

    /** vTaskDelete(NULL); */
}
