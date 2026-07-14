/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_ether_netif.c
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
/** "UETM" in ASCII, used to determine if channel is open. */
#define ETHER_NETIF_OPEN    (0x45544E49ULL)
#define ETHER_NETIF_CLOSE   (ETHER_NETIF_OPEN - 1)
#define ETHER_NETIF_RUN     (ETHER_NETIF_OPEN + 1)

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
 * Private global variables
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * Global Variables
 **********************************************************************************************************************/

/** UM_ETHER_NETIF Implementation of General UM_ETHER_NETIF Driver  */
const ether_netif_api_t g_ether_netif_on_ether_netif =
{
    .open          = UM_ETHER_NETIF_Open,
    .close         = UM_ETHER_NETIF_Close,
    .start           = UM_ETHER_NETIF_Start,
    .stop           = UM_ETHER_NETIF_Stop,
    .send             = UM_ETHER_NETIF_Send,
    .callbackAdd   = UM_ETHER_NETIF_CallbackAdd,
    .linkStatusGet = UM_ETHER_NETIF_LinkStatusGet
};

/*******************************************************************************************************************//**
 * @addtogroup UM_ETHER_NETIF
 * @{
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/
/**********************************************************************************************************************
* Function Name: UM_ETHER_NETIF_Open
* Description  : Initializes the UM_ETHER_NETIF instance.
* Arguments    : [in] p_ctrl               Pointer to the controller
*                [in] p_target_ctrl        Pointer to target controller
* Return Value : USR_SUCCESS                Process has been done successfully.
*                USR_ERR_ASSERTION          The argument parameter are invalid.
*                USR_ERR_ALREADY_OPEN       The instance has been already opened.
*                USR_ERR_NOT_INITIALIZED    Initialization has been failed.
*********************************************************************************************************************/
usr_err_t UM_ETHER_NETIF_Open(ether_netif_ctrl_t * const p_ctrl, ether_netif_cfg_t const * const p_cfg)
{
#if defined(UM_ETHER_NETIF_CFG_PARAM_CHECKING_ENABLE) && ((1) == UM_ETHER_NETIF_CFG_PARAM_CHECKING_ENABLE)
    USR_ASSERT(p_ctrl);
    USR_ASSERT(p_cfg);
#endif

    /** Status */
    usr_err_t   usr_err;

    /** Resolve control instance type. */
    ether_netif_instance_ctrl_t * const p_instance_ctrl = (ether_netif_ctrl_t * const) p_ctrl;

    /** Check if the instance is already started. */
    USR_ERROR_RETURN( ETHER_NETIF_RUN != p_instance_ctrl->open, USR_ERR_ALREADY_RUNNING );

    /** Check if the instance is already opened. */
    USR_ERROR_RETURN( ETHER_NETIF_OPEN != p_instance_ctrl->open, USR_ERR_ALREADY_OPEN );

    /** Initialize instance control */
    p_instance_ctrl->p_cfg = p_cfg;

    /** Initialize internal module */
    {
        /** Open Ethernet driver interface and its controller. */
        usr_err = um_ether_netif_ether_open(&p_instance_ctrl->ether_ctrl,
            p_instance_ctrl->p_cfg->p_ether_instance, &p_instance_ctrl->reader_ctrl);
        USR_ERROR_RETURN( USR_SUCCESS == usr_err , USR_ERR_NOT_INITIALIZED );

        /** Open callback module  */
        usr_err = um_ether_netif_callback_open(&p_instance_ctrl->callback_ctrl);
        USR_ERROR_RETURN( USR_SUCCESS == usr_err , USR_ERR_NOT_INITIALIZED );

        /** Create link monitor task. */
        usr_err = um_ether_netif_task_monitor_open( &p_instance_ctrl->monitor_ctrl,
                                                    &p_instance_ctrl->ether_ctrl,
                                                    &p_instance_ctrl->callback_ctrl );
        USR_ERROR_RETURN( USR_SUCCESS == usr_err , USR_ERR_NOT_INITIALIZED );

        /** Create network writer task */
        usr_err = um_ether_netif_task_writer_open( &p_instance_ctrl->writer_ctrl,
            &p_instance_ctrl->ether_ctrl );
        USR_ERROR_RETURN( USR_SUCCESS == usr_err , USR_ERR_NOT_INITIALIZED );

        /** Create network reader task */
        usr_err = um_ether_netif_task_reader_open( &p_instance_ctrl->reader_ctrl,
            &p_instance_ctrl->ether_ctrl, &p_instance_ctrl->callback_ctrl );
        USR_ERROR_RETURN( USR_SUCCESS == usr_err , USR_ERR_NOT_INITIALIZED );
    }

    /** Set open state to the instance control. */
    p_instance_ctrl->open = ETHER_NETIF_OPEN;

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: UM_ETHER_NETIF_Close
* Description  : Finalize the UM_ETHER_MW frame handling instance.
* Arguments    : [in] p_ctrl               Pointer to the controller
* Return Value : USR_SUCCESS                Finalizing was successful and each task has started.
*                USR_ERR_ASSERTION           The argument parameter are invalid.
*********************************************************************************************************************/
usr_err_t UM_ETHER_NETIF_Close(ether_netif_ctrl_t * const p_ctrl)
{
#if defined(UM_ETHER_NETIF_CFG_PARAM_CHECKING_ENABLE) && ((1) == UM_ETHER_NETIF_CFG_PARAM_CHECKING_ENABLE)
    USR_ASSERT(p_ctrl);
#endif
    /**
     * TODO: Add Close Sequence.
     */
    (void) p_ctrl;
    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: UM_ETHER_NETIF_Start
* Description  : Resume each task.
* Arguments    : [in] p_ctrl               Pointer to the controller
* Return Value : USR_SUCCESS                Finalizing was successful and each task has started.
*********************************************************************************************************************/
usr_err_t UM_ETHER_NETIF_Start(ether_netif_ctrl_t * const p_ctrl)
{
#if defined(UM_ETHER_NETIF_CFG_PARAM_CHECKING_ENABLE) && ((1) == UM_ETHER_NETIF_CFG_PARAM_CHECKING_ENABLE)
    USR_ASSERT(p_ctrl);
#endif
    /** Resolve control instance type. */
    ether_netif_instance_ctrl_t * const p_instance_ctrl = (ether_netif_instance_ctrl_t * const) p_ctrl;

    /** Resume each Task. */
    (void) um_ether_netif_task_monitor_start(&p_instance_ctrl->monitor_ctrl);
    (void) um_ether_netif_task_reader_start(&p_instance_ctrl->reader_ctrl);

    /** Set the status to be started. */
    p_instance_ctrl->open = ETHER_NETIF_RUN;

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: UM_ETHER_NETIF_Stop
* Description  : Suspend each task.
* Arguments    : [in] p_ctrl               Pointer to the controller
* Return Value : USR_SUCCESS                Finalizing was successful and each task has started.
*********************************************************************************************************************/
usr_err_t UM_ETHER_NETIF_Stop(ether_netif_ctrl_t * const p_ctrl)
{
#if defined(UM_ETHER_NETIF_CFG_PARAM_CHECKING_ENABLE) && ((1) == UM_ETHER_NETIF_CFG_PARAM_CHECKING_ENABLE)
    USR_ASSERT(p_ctrl);
#endif

    /** Resolve control instance type. */
    ether_netif_instance_ctrl_t * const p_instance_ctrl = (ether_netif_instance_ctrl_t * const) p_ctrl;

    /** Suspend each Task. */
    (void) um_ether_netif_task_monitor_stop(&p_instance_ctrl->monitor_ctrl);
    (void) um_ether_netif_task_reader_stop(&p_instance_ctrl->reader_ctrl);

    p_instance_ctrl->open = ETHER_NETIF_OPEN;

    return USR_SUCCESS;
}

/*********************************************************************************************************************
* Function Name: UM_ETHER_NETIF_CallbackAdd
* Description  : Updates the user callback with the option to provide memory for the callback argument structure.
* Arguments    : [in] p_ctrl               Pointer to the controller
*                [in] p_node               Pointer to the callback controller
* Return Value : USR_SUCCESS               Callback was updated successfully.
*                USR_ERR_IN_USE    Resources are busy.
*********************************************************************************************************************/
usr_err_t UM_ETHER_NETIF_CallbackAdd (ether_netif_ctrl_t * const p_ctrl,
        ether_netif_callback_link_node_t * const p_node )
{
#if defined(UM_ETHER_NETIF_CFG_PARAM_CHECKING_ENABLE) && ((1) == UM_ETHER_NETIF_CFG_PARAM_CHECKING_ENABLE)
    USR_ASSERT(p_ctrl);
#endif

    /** Resolve control instance type. */
    ether_netif_instance_ctrl_t * p_instance_ctrl = (ether_netif_instance_ctrl_t *) p_ctrl;

    /** Status */
    usr_err_t usr_err;

    /** Set callback callback function. */
    usr_err = um_ether_netif_callback_add( &p_instance_ctrl->callback_ctrl, p_node );

    /** Return the error without change. */
    return usr_err;
}

/**********************************************************************************************************************
* Function Name: UM_ETHER_NETIF_Send
* Description  : Send Ethernet frame to be sent via Ethernet driver.
* Arguments    : [in] p_ctrl               Pointer to the controller
*                [in] p_packet_buffer      Pointer to the packet buffer
* Return Value : USR_SUCCESS               Send the frame to the TX queue successfully.
*                USR_ERR_IN_USE              The TX queue is full.
*********************************************************************************************************************/
usr_err_t UM_ETHER_NETIF_Send(ether_netif_ctrl_t * const p_ctrl, ether_netif_frame_t * const p_packet_buffer)
{
#if defined(UM_ETHER_NETIF_CFG_PARAM_CHECKING_ENABLE) && ((1) == UM_ETHER_NETIF_CFG_PARAM_CHECKING_ENABLE)
    USR_ASSERT(p_ctrl);
    USR_ASSERT(p_packet_buffer);
#endif

    /** Resolve control instance type. */
    ether_netif_instance_ctrl_t * const p_instance_ctrl = (ether_netif_instance_ctrl_t * const) p_ctrl;

    /** Status */
    usr_err_t usr_err;

    /** Request to enqueue Ethernet frame. */
    usr_err = um_ether_netif_ether_write( &p_instance_ctrl->ether_ctrl, p_packet_buffer );

    /** Return the error without change */
    return usr_err;
}

/**********************************************************************************************************************
* Function Name: UM_ETHER_NETIF_LinkStatusGet
* Description  : Get link status of all ports
* Arguments    : [in] p_ctrl               Pointer to the controller
*                [out] p_packet_buffer      Pointer to the link status
* Return Value : USR_SUCCESS               Send the frame to the TX queue successfully.
*********************************************************************************************************************/
usr_err_t UM_ETHER_NETIF_LinkStatusGet(
        ether_netif_ctrl_t * const p_ctrl, uint32_t * p_link_status, uint8_t notify_callback)
{
#if defined(UM_ETHER_NETIF_CFG_PARAM_CHECKING_ENABLE) && ((1) == UM_ETHER_NETIF_CFG_PARAM_CHECKING_ENABLE)
    USR_ASSERT(p_ctrl);
    USR_ASSERT(p_link_status);
#endif

    /** Resolve control instance type. */
    ether_netif_instance_ctrl_t * const p_instance_ctrl = (ether_netif_instance_ctrl_t * const) p_ctrl;

    /** Status */
    usr_err_t usr_err;

    /** Get the link status of all ports */
    usr_err = um_ether_netif_task_monitor_get_link_status (&p_instance_ctrl->monitor_ctrl, p_link_status);

    if(notify_callback)
    {
        usr_err = um_ether_netif_callback_request(&p_instance_ctrl->callback_ctrl,
            ((*p_link_status) & ETHER_NETIF_PORT_MASK) ? ETHER_NETIF_CALLBACK_EVENT_LINK_UP :
            ETHER_NETIF_CALLBACK_EVENT_LINK_DOWN , NULL);
    }

    /** Return the error without change */
    return usr_err;
}

/** @} (end addtogroup UM_ETHER_NETIF) */
