/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_lwip_port_task_launcher.c
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
#include "um_lwip_port.h"
#include "um_lwip_port_cfg.h"
#include "um_lwip_port_internal.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

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
/** Task functions to handling Ethernet input, output and link. */
static void task_code(void * pvParameter);

static usr_err_t netif_link_on( lwip_port_netif_ctrl_t * p_ctrl, void * p_context );
static usr_err_t netif_link_off( lwip_port_netif_ctrl_t * p_ctrl, void * p_context );

/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/
/**********************************************************************************************************************
* Function Name: um_lwip_port_task_launcher_open
* Description  : Create the task and initialize its controller.
* Arguments    : [in] p_ctrl     Pointer to the controller
*                [in] p_netif_list_ctrl    Pointer to callback controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_task_launcher_open( lwip_port_launcher_ctrl_t * const p_ctrl,
        lwip_port_netif_list_ctrl_t * const p_netif_list_ctrl )
{
    BaseType_t rtos_err;

    /** Set the related submodules. */
    p_ctrl->p_netif_list_ctrl    = p_netif_list_ctrl;
    p_ctrl->p_parent_task_handle = xTaskGetCurrentTaskHandle();

    /** Create frame T. */
    p_ctrl->p_queue_handle = xQueueCreate(UM_LWIP_PORT_CFG_LAUNCHER_QUEUE_LENGTH, sizeof(lwip_port_launcher_request_t));
    USR_ERROR_RETURN( NULL != p_ctrl->p_queue_handle, USR_ERR_NOT_INITIALIZED);

    /** Create netif launcher task */
    rtos_err = xTaskCreate( task_code,
                            UM_LWIP_PORT_CFG_LAUNCHER_TASK_NAME,
                            UM_LWIP_PORT_CFG_LAUNCHER_TASK_STACK_BTYE_SIZE / sizeof(StackType_t),
                            p_ctrl,
                            UM_LWIP_PORT_CFG_LAUNCHER_TASK_PRIORITY,
                            &(p_ctrl->p_task_handle) );
    USR_ERROR_RETURN( pdPASS == rtos_err, USR_ERR_NOT_INITIALIZED );

    /** Wait for notification indicating the created task is initialized. */
    (void) ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    
    /** Suspend the created task. */
    vTaskSuspend( p_ctrl->p_task_handle );
    
    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_task_launcher_start
* Description  : Start the task.
* Arguments    : [in] p_ctrl     Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_task_launcher_start( lwip_port_launcher_ctrl_t * const p_ctrl )
{
    /** Check if the resource is already enabled. */
    USR_ERROR_RETURN( NULL != p_ctrl->p_task_handle, USR_ERR_NOT_ENABLED );

    /** Resume task. */
    vTaskResume(p_ctrl->p_task_handle);

    /** Return error. */
    return USR_SUCCESS;
}
/**********************************************************************************************************************
* Function Name: um_lwip_port_task_launcher_stop
* Description  : Stop the task.
* Arguments    : [in] p_ctrl     Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_task_launcher_stop( lwip_port_launcher_ctrl_t * const p_ctrl )
{
    /** Check if the resource is already enabled. */
    USR_ERROR_RETURN( NULL != p_ctrl->p_task_handle, USR_ERR_NOT_ENABLED );

    /** Suspend task. */
    vTaskSuspend(p_ctrl->p_task_handle);

    /** Return error. */
    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_task_launcher_request
* Description  : Request the RTOS task to launch lwIP network interface.
* Arguments    : [in] p_ctrl     Pointer to the controller
*                [in] event            Ethernet link callback event.
*                [in] p_netif_ctrl Pointer to the netif controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_task_launcher_request( lwip_port_launcher_ctrl_t * const p_ctrl,
        lwip_port_launcher_event_t const event, lwip_port_netif_ctrl_t * const p_netif_ctrl)
{
    /** Status */
    BaseType_t rtos_err;

    /** Check if the resource is already enabled. */
    USR_ERROR_RETURN( NULL != p_ctrl->p_queue_handle, USR_ERR_NOT_ENABLED );

    /** Luncher request */
    lwip_port_launcher_request_t request;
    request.event        = event;
    request.p_netif_ctrl = p_netif_ctrl;

    /** Send Event. */
    rtos_err = xQueueSend(p_ctrl->p_queue_handle, &request, 0);
    USR_ERROR_RETURN( rtos_err == pdTRUE, USR_ERR_IN_USE );

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
    lwip_port_launcher_ctrl_t * const p_ctrl = (lwip_port_launcher_ctrl_t * const) pvParameter;

    /** Status */
    BaseType_t rtos_err;
    /** FSP status */
    usr_err_t  usr_err;

    /** Luncher event */
    lwip_port_launcher_request_t request;

    /** Notify the parenet task launch of this task. */
    xTaskNotifyGive( p_ctrl->p_parent_task_handle );

    while(true)
    {
        /** Notified from Ethernet Link callback. */
        rtos_err = xQueueReceive(p_ctrl->p_queue_handle, &request, portMAX_DELAY);
        if( pdPASS != rtos_err )
        {
            /** TODO: Implement task assertion */
        }

        switch( request.event )
        {
            case LWIP_PORT_LAUNCHER_EVENT_LINK_UP:
                usr_err = um_lwip_port_netif_list_map(p_ctrl->p_netif_list_ctrl, netif_link_on, NULL);
                if( USR_SUCCESS != usr_err)
                {
                    /** TODO: Implement error handling in task. */
                }
                break;

            case LWIP_PORT_LAUNCHER_EVENT_LINK_DOWN:
                usr_err = um_lwip_port_netif_list_map(p_ctrl->p_netif_list_ctrl, netif_link_off, NULL);
                if (USR_SUCCESS != usr_err)
                {
                    /** TODO: Implement error handling in task. */
                }
                break;

            case LWIP_PORT_LAUNCHER_EVENT_IP_UP:
                if( request.p_netif_ctrl )
                {
                    usr_err = um_lwip_port_netif_set_netif_state(request.p_netif_ctrl, LWIP_PORT_NETIF_STATE_UP);
                    if (USR_SUCCESS != usr_err)
                    {
                        /** TODO: Implement error handling in task. */
                    }
                }
                break;

            case LWIP_PORT_LAUNCHER_EVENT_IP_DOWN:
                if( request.p_netif_ctrl )
                {
                    usr_err = um_lwip_port_netif_set_netif_state(request.p_netif_ctrl, LWIP_PORT_NETIF_STATE_DOWN);
                    if (USR_SUCCESS != usr_err)
                    {
                        /** TODO: Implement error handling in task. */
                    }
                }
                break;

            default:
                /** TODO: Implement error handling in task. */
                break;
        }
    }

    /** vTaskDelete(NULL); */
}

/**********************************************************************************************************************
* Function Name: netif_link_on
* Description  : Process for link up process when the ethernet link state becomes down from up.
* Arguments    : [in] p_ctrl     Pointer to the controller
*                [in] p_context  Pointer to the context data
* Return Value : USR_SUCCESS                    Process has been done successfully.
*                USR_ERR_ABORTED            Failed to set.
*********************************************************************************************************************/
static usr_err_t netif_link_on( lwip_port_netif_ctrl_t * p_ctrl, void * p_context )
{
    /** No context */
    (void) p_context;

    /** Status */
    usr_err_t usr_err;

    /** Set the network interface to up */
    usr_err = um_lwip_port_netif_set_up( p_ctrl );

    return usr_err;
}

/**********************************************************************************************************************
* Function Name: netif_link_off
* Description  : PProcess for link down process when the Ethernet link state becomes down from up.
* Arguments    : [in] p_ctrl     Pointer to the controller
*                [in] p_context  Pointer to the context data
* Return Value : USR_SUCCESS                    Process has been done successfully.
*                USR_ERR_ABORTED            Failed to set.
*********************************************************************************************************************/
static usr_err_t netif_link_off( lwip_port_netif_ctrl_t * p_ctrl, void * p_context )
{
    (void) p_context;

    /** Status */
    usr_err_t usr_err;

    /** Set the network interface to down */
    usr_err = um_lwip_port_netif_set_down( p_ctrl );

    return usr_err;
}
