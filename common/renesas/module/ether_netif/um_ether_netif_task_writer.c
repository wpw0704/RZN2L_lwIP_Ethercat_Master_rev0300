/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_ether_netif_task_writer.c
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
/** Ethernet frame size restrictions */
#define ETHER_FRAME_MAXIMUM_BYTES        ( 1514U )   /** without CRC 4byte */
#define ETHER_FRAME_MINIMUM_BYTES        ( 60U )     /** without CRC 4byte */

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
* Function Name: um_ether_netif_task_writer_open
* Description  : initialize its controller.
* Arguments    : [in] p_ctrl               Pointer to the controller
*                [in] p_target_ctrl        Pointer to target controller
* Return Value : USR_SUCCESS                Process has been done successfully.
*                USR_ERR_NOT_INITIALIZED    Initialization has been failed.
*********************************************************************************************************************/
usr_err_t um_ether_netif_task_writer_open( ether_netif_writer_ctrl_t * const p_ctrl,
        ether_netif_ether_ctrl_t * const p_target_ctrl )
{

    /** Set target controller. */
    p_ctrl->p_ether_ctrl = p_target_ctrl;
    /** task handle */
    p_ctrl->p_parent_task_handle = xTaskGetCurrentTaskHandle();
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    USR_HEAP_ALLOCATE( p_ctrl->p_ether_netif_frame, sizeof(ether_netif_frame_t) );
    memset(p_ctrl->p_ether_netif_frame, 0,  sizeof(ether_netif_frame_t));

    return USR_SUCCESS;
}

