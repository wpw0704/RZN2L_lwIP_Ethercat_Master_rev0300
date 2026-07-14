/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_ether_netif_task_reader.c
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
#include "lwip/etharp.h"
#include "um_ether_netif.h"
#include "um_ether_netif_cfg.h"
#include "um_ether_netif_internal.h"
#include "r_ether_api.h"
#include "r_ether_cfg.h"
#include "fsp_common_api.h"

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
static void task_code(void * pvParameter);

/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/
/**********************************************************************************************************************
* Function Name: um_ether_netif_task_reader_open
* Description  : Create the task and initialize its controller.
* Arguments    : [in] p_ctrl            Pointer to the controller
*                [in] p_target_ctrl        Pointer to target controller
*                [in] p_callback_ctrl    Pointer to callback controller
* Return Value : USR_SUCCESS                Process has been done successfully.
*                USR_ERR_NOT_INITIALIZED        Initialization has been failed.
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_reader_open( ether_netif_reader_ctrl_t * const p_ctrl,
    ether_netif_ether_ctrl_t * const p_target_ctrl,
    ether_netif_callback_ctrl_t * const p_callback_ctrl )
{
    BaseType_t rtos_err;

    /** Set target and callback control. */
    p_ctrl->p_ether_ctrl         = p_target_ctrl;
    p_ctrl->p_callback_ctrl      = p_callback_ctrl;
    p_ctrl->p_parent_task_handle = xTaskGetCurrentTaskHandle();
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    USR_HEAP_ALLOCATE( p_ctrl->p_ether_netif_frame, sizeof(ether_netif_frame_t));
    memset(p_ctrl->p_ether_netif_frame, 0,  sizeof(ether_netif_frame_t));

    /** Create the task. */
    rtos_err = xTaskCreate(task_code,
        UM_ETHER_NETIF_CFG_READER_TASK_NAME,
        UM_ETHER_NETIF_CFG_READER_TASK_STACK_BYTE_SIZE / sizeof(StackType_t),
        p_ctrl, UM_ETHER_NETIF_CFG_READER_TASK_PRIORITY,
        &(p_ctrl->p_task_handle) );
    USR_ERROR_RETURN( pdPASS == rtos_err, USR_ERR_NOT_INITIALIZED );
    
    /** Wait for notification indicating the created task is initialized. */
    (void) ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    
    /** Suspend the created task. */
    vTaskSuspend( p_ctrl->p_task_handle );
    
    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_ether_netif_task_reader_start
* Description  : Start the task.
* Arguments    : [in] p_ctrl               Pointer to the controller
* Return Value : USR_SUCCESS                Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_reader_start( ether_netif_reader_ctrl_t * const p_ctrl )
{
    vTaskResume(p_ctrl->p_task_handle);
    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_ether_netif_task_reader_stop
* Description  : Stop the task.
* Arguments    : [in] p_ctrl               Pointer to the controller
* Return Value : USR_SUCCESS                Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_reader_stop( ether_netif_reader_ctrl_t * const p_ctrl )
{
    vTaskSuspend(p_ctrl->p_task_handle);
    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_ether_netif_task_reader_get_task_handle
* Description  : Get the task handle
* Arguments    : [in] p_ctrl               Pointer to the controller
*                [out] pp_task_handle      Pointer to task handle of all ports
* Return Value : USR_SUCCESS                Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_reader_get_task_handle(
        ether_netif_reader_ctrl_t * const p_ctrl, TaskHandle_t * pp_task_handle )
{
    * pp_task_handle = p_ctrl->p_task_handle;
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
static void task_code(void * pvParameter)
{
    /** Resolve task parameter */
    ether_netif_reader_ctrl_t * const p_ctrl = (ether_netif_reader_ctrl_t * const) pvParameter;

    /** Status */
    uint32_t  notices;
    /** lwip buffer */
    struct pbuf *p_netif_packet_pbuf = NULL;

    /** Notify the parent task launch of this task. */
    xTaskNotifyGive( p_ctrl->p_parent_task_handle );

    /** Task loop */
    while ( true )
    {
        /** Notified from Ethernet module interrupt */
        notices = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        if ( notices == 0 )
        {
            continue;
            /** TODO: Implement task error handling */
        }
        /** Read from the Ethernet driver interface with lock */
        for( ;; )
        {

            /** If receiving to lwip */
            if(p_ctrl->p_ether_netif_frame->buffer_mode & ETHER_NETIF_CFG_FB_MODE_LWIP)
            {

                /** Get the receive buffer according to the used buffer */
                if(p_netif_packet_pbuf == NULL)
                {
                    p_netif_packet_pbuf = pbuf_alloc(PBUF_RAW,  1546, PBUF_RAM);
                    if ( p_netif_packet_pbuf == NULL )
                    {
                        taskYIELD();
                        continue;
                    }


                }
                /** Save received data to a pointer buffer */
                p_ctrl->p_ether_netif_frame->p_buffer = p_netif_packet_pbuf->payload;

                if(USR_SUCCESS != um_ether_netif_ether_read(p_ctrl->p_ether_ctrl, p_ctrl->p_ether_netif_frame ))
                {
                    break;
                }
                /** If using lwip buffer, change to buffer structure address */
                p_netif_packet_pbuf->tot_len =
                /** CODE CHECKER, this is OK as a comment aligns with the cast*/
                p_netif_packet_pbuf->len = (uint16_t)p_ctrl->p_ether_netif_frame->length;
                /** CODE CHECKER, this is OK as a comment aligns with the cast*/
                p_ctrl->p_ether_netif_frame->p_buffer = (uint8_t *)p_netif_packet_pbuf;
                /** CODE CHECKER, this is OK as a comment aligns with the cast*/
                um_ether_netif_callback_request((ether_netif_callback_ctrl_t * ) p_ctrl->p_callback_ctrl,
                    ETHER_NETIF_CALLBACK_EVENT_RECEIVE_ETHER_FRAME, p_ctrl->p_ether_netif_frame );

                /** CODE CHECKER, this is OK as a comment aligns with the cast*/
                p_netif_packet_pbuf = (struct pbuf *)p_ctrl->p_ether_netif_frame->p_buffer;
                taskYIELD();
            }
            else
            {
                /** Ethernet driver common buffer management */
                if(USR_SUCCESS != um_ether_netif_ether_read(p_ctrl->p_ether_ctrl, p_ctrl->p_ether_netif_frame ))
                {
                    break;
                }
                /** CODE CHECKER, this is OK as a comment aligns with the cast*/
                um_ether_netif_callback_request((ether_netif_callback_ctrl_t * ) p_ctrl->p_callback_ctrl,
                    ETHER_NETIF_CALLBACK_EVENT_RECEIVE_ETHER_FRAME, p_ctrl->p_ether_netif_frame );
                taskYIELD();

            }
        }
    }
}
