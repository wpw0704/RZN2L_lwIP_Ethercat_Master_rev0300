/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_ether_netif_ether.c
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
/** Includes user modules */
#include "um_common.h"
#include "um_ether_netif_api.h"
#include "um_ether_netif_cfg.h"
#include "um_ether_netif_feature.h"
#include "um_ether_netif.h"
#include "um_ether_netif_internal.h"

/** Includes target FSP modules */
#include "fsp_common_api.h"

/** Includes r_ether module */
#include "r_ether_api.h"
#include "r_ether_cfg.h"
#include "main_thread.h"


#if UM_ETHER_NETIF_FEATURE_R_ETHER_IMPL_R_ETHER
#include "r_ether.h"
#endif
#if UM_ETHER_NETIF_FEATURE_R_ETHER_IMPL_R_GMAC
#include "r_gmac.h"
#endif
#if UM_ETHER_NETIF_FEATURE_R_ETHER_DEPEND_ON_R_ETHSW
#include "r_ether_switch_api.h"
#endif

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#if UM_ETHER_NETIF_FEATURE_R_ETHER_IMPL_R_ETHER
#define ETHER_NETIF_INTERRUPT_FACTOR_RECEPTION          (0x01070000)
#endif /** defined(BSP_MCU_GROUP_RA6M3) */

#if UM_ETHER_NETIF_FEATURE_R_ETHER_IMPL_R_GMAC
#define ETHER_NETIF_INTERRUPT_FACTOR_RECEPTION            (0x000000C0)
#endif

#if UM_ETHER_NETIF_FEATURE_R_ETHSW_SUPPORT_SPECIFIC_TAG
/** Define management tag for reception. */
#define GMAC_RX_MGTAG_DATA1_PORT_SHIFT      (0U)
#define GMAC_RX_MGTAG_DATA1_PORT_MASK       (0x03 << GMAC_RX_MGTAG_DATA1_PORT_SHIFT )
#define GMAC_RX_MGTAG_DATA1_TIMER           (1U << 4)
#define GMAC_RX_MGTAG_DATA1_RED             (1U << 6)

/** Define management tag for transmit */
#define GMAC_TX_MGTAG_DATA1_FORWARD_FORCE   (1U << 0)
#define GMAC_TX_MGTAG_DATA1_TIMESTAMP       (1U << 3)
#define GMAC_TX_MGTAG_DATA1_ONE_STEP        (1U << 4)
#define GMAC_TX_MGTAG_DATA1_PRP_SUPPRESS    (1U << 5)
#define GMAC_TX_MGTAG_DATA1_PRP_FORCE       (1U << 6)
#define GMAC_TX_MGTAG_DATA1_PRP_SEQUENCE    (1U << 7)
#define GMAC_TX_MGTAG_DATA1_QUE_NUM         (1U << 9)
#define GMAC_TX_MGTAG_DATA1_TIM_NUM         (1U << 13)

#define GMAC_TX_MGTAG_DATA1_QUE_NUM_SHIFT   (10U)
#define GMAC_TX_MGTAG_DATA1_QUE_NUM_MASK    (0x07 << GMAC_TX_MGTAG_DATA1_QUE_NUM_SHIFT)
#define GMAC_TX_MGTAG_DATA1_TIM_NUM_SHIFT   (14U)
#define GMAC_TX_MGTAG_DATA1_TIM_NUM_MASK    (0x01 << GMAC_TX_MGTAG_DATA1_TIM_NUM_SHIFT)

#define GMAC_TX_MGTAG_DATA2_PORT_SHIFT      (0U)
#define GMAC_TX_MGTAG_DATA2_PORT_MASK       (0x07 << GMAC_TX_MGTAG_DATA2_PORT_SHIFT)
#define GMAC_TX_MGTAG_DATA2_TSID_SHIFT      (9U)
#define GMAC_TX_MGTAG_DATA2_TSID_MASK       (0x7F << GMAC_TX_MGTAG_DATA2_TSID_SHIFT)
#endif

/***********************************************************************************************************************
 * Private constants
 **********************************************************************************************************************/
/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

#if UM_ETHER_NETIF_FEATURE_R_ETHSW_SUPPORT_SPECIFIC_TAG
typedef struct st_ether_frame
{
    uint8_t dst_addr[6];               /* destination addres */
    uint8_t src_addr[6];               /* source address */
    uint8_t type[2];                   /* type */
    uint8_t data[2];                   /* data */
} ether_frame_t;

typedef struct st_ether_mgtag
{
    uint8_t control_tag[2];            /* control_tag of management-tag  */
    uint8_t control_data[2];           /* control_data of management-tag  */
    uint8_t control_data2[4];          /* control_data2 of management-tag  */
} ether_mgtag_t;

typedef struct st_ether_frame_mgtag
{
    uint8_t       dst_addr[6];         /* destination address */
    uint8_t       src_addr[6];         /* source address */
    ether_mgtag_t mgtag;               /* management tag */
    uint8_t       type[2];             /* type */
    uint8_t       data[2];             /* data */
} ether_frame_mgtag_t;
#endif

/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
#if UM_ETHER_NETIF_FEATURE_R_ETHSW_SUPPORT_SPECIFIC_TAG
static usr_err_t build_tx_management_tag ( ether_netif_ether_ctrl_t * const p_ctrl,
        ether_netif_frame_t * const p_frame, ether_mgtag_t * p_mgtag );
static usr_err_t fetch_rx_management_tag ( ether_netif_ether_ctrl_t * const p_ctrl,
        ether_netif_frame_t * const p_frame, ether_mgtag_t const * p_mgtag );
static usr_err_t insert_management_tag ( ether_netif_frame_t *p_frame, ether_mgtag_t const * p_mgtag );
static usr_err_t remove_management_tag (  ether_netif_frame_t *p_frame, ether_mgtag_t * p_mgtag );

extern const ether_switch_instance_t g_ethsw0;

#endif

/***********************************************************************************************************************
 * Imported function prototypes
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/
/**
 * Context for callback of r_ether module.
 * TODO:
 *  r_ether module has user context interface but it does not work;
 *    RA FSP v2.x.x:  p_context is not used.
 *    RA FSP v3.x.x:  p_context cannot be set by FSP configuration.
 *    RZT FSP v1.x.x: p_context cannot be set by FSP configuration.
 *  Therefore, this is instead of the user context interface.
 */
static void * sp_r_ether_callback_context = NULL;

/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/
/**********************************************************************************************************************
* Function Name: um_ether_netif_ether_open
* Description  : Initialize the controller.
* Arguments    : Pointer to the controller, Pointer to the Ethernet instance, Pointer to the read instance
* Return Value : USR_SUCCESS           Process has been done successfully.
*                USR_ERR_IN_USE        Initialization has been failed.
*********************************************************************************************************************/
usr_err_t um_ether_netif_ether_open(ether_netif_ether_ctrl_t * const p_ctrl,
                                    ether_instance_t const * const p_ether_instance,
                                    ether_netif_reader_ctrl_t * const p_reader_ctrl)
{
    /** Error codes */
    fsp_err_t fsp_err;

    /** Set target control. */
    p_ctrl->p_ether_instance = p_ether_instance;

    /** Set internal submodule control. */
    p_ctrl->p_reader_ctrl = p_reader_ctrl;

    /** Create a mutex for r_ether */
    p_ctrl->p_mutex_handle    = xSemaphoreCreateMutex();
    p_ctrl->p_rx_mutex_handle = xSemaphoreCreateMutex();
    p_ctrl->p_tx_mutex_handle = xSemaphoreCreateMutex();
    USR_ERROR_RETURN( p_ctrl->p_mutex_handle, USR_ERR_NOT_INITIALIZED );

    /** Open the Ethernet instance */
    fsp_err = p_ctrl->p_ether_instance->p_api->open(p_ctrl->p_ether_instance->p_ctrl, p_ctrl->p_ether_instance->p_cfg);
    USR_ERROR_RETURN( FSP_SUCCESS == fsp_err || FSP_ERR_ALREADY_OPEN == fsp_err , USR_ERR_NOT_INITIALIZED );

    /** Set context */
    sp_r_ether_callback_context = p_ctrl;

    /** Return success code */
    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_ether_netif_ether_read
* Description  : Read Ethernet frame from Ethernet peripherals via driver.
* Arguments    : Pointer to the controller, Pointer to the frame buffer to be set to the read Ethernet frame.
* Return Value : USR_SUCCESS           Process has been done successfully.
*                USR_ERR_IN_USE        Resources are busy.
*********************************************************************************************************************/
usr_err_t um_ether_netif_ether_read( ether_netif_ether_ctrl_t * const p_ctrl, ether_netif_frame_t * const p_frame )
{
    /** Error codes */
    fsp_err_t fsp_err;

    /** Initialize the port number */
    p_frame->port = (uint8_t)ETHER_NETIF_CFG_PORT_RECV_PORT_ANY;

    /** Lock statement. */
    xSemaphoreTake( p_ctrl->p_rx_mutex_handle , portMAX_DELAY);
    {
        if(p_frame->buffer_mode & ETHER_NETIF_CFG_FB_MODE_POINTER)
        {
            /** Read Ethernet frame. */
            if(FSP_SUCCESS != (fsp_err = p_ctrl->p_ether_instance->p_api->read
                    (p_ctrl->p_ether_instance->p_ctrl, p_frame->p_buffer, (uint32_t *)&p_frame->length)))
            {
                xSemaphoreGive( p_ctrl->p_rx_mutex_handle );
                /** CODE CHECKER, this is OK as a comment aligns with the cast*/
                return (usr_err_t)fsp_err;
            }
        }
        else
        {
            /** Read Ethernet frame. */
            if(FSP_SUCCESS != (fsp_err = p_ctrl->p_ether_instance->p_api->read
                    (p_ctrl->p_ether_instance->p_ctrl, p_frame->buffer,
                    (uint32_t *)&p_frame->length)))
            {
                xSemaphoreGive( p_ctrl->p_rx_mutex_handle );
                /** CODE CHECKER, this is OK as a comment aligns with the cast*/
                return (usr_err_t)fsp_err;
            }

        }

    }
    xSemaphoreGive( p_ctrl->p_rx_mutex_handle );

#if UM_ETHER_NETIF_FEATURE_R_ETHSW_SUPPORT_SPECIFIC_TAG

    /** Resolve the gmac's extended configuration. */
    ethsw_extend_cfg_t const * p_gmac_extend_cfg = g_ethsw0.p_cfg->p_extend;

    /** The received Ethernet frame does not have the management tag. */
    if( p_gmac_extend_cfg->specific_tag == ETHSW_SPECIFIC_TAG_ENABLE )
    {
        ether_mgtag_t mgtag;
        (void) remove_management_tag(p_frame, &mgtag);
        (void) fetch_rx_management_tag(p_ctrl, p_frame, &mgtag);
    }

#endif

    /** Return success code */
    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_ether_netif_ether_write
* Description  : Write Ethernet frame to Ethernet peripherals via driver.
* Arguments    : Pointer to the controller, Pointer to the frame buffer to be set to the send Ethernet frame.
* Return Value : USR_SUCCESS           Process has been done successfully.
*                USR_ERR_IN_USE        Resources are busy.
*********************************************************************************************************************/
usr_err_t um_ether_netif_ether_write( ether_netif_ether_ctrl_t * const p_ctrl, ether_netif_frame_t * const p_frame )
{
    /** Error codes */
    fsp_err_t fsp_err = FSP_SUCCESS;

#if UM_ETHER_NETIF_FEATURE_R_ETHSW_SUPPORT_SPECIFIC_TAG

    /** Management tag */
    ether_mgtag_t mgtag;

    /** Resolve the gmac extended configuration. */
    ethsw_extend_cfg_t const * p_gmac_extend_cfg = g_ethsw0.p_cfg->p_extend;

    /** The received Ethernet frame does not have the management tag. */
    if( ETHSW_SPECIFIC_TAG_ENABLE == p_gmac_extend_cfg->specific_tag )
    {
        (void) build_tx_management_tag(p_ctrl, p_frame, &mgtag);
        (void) insert_management_tag(p_frame, &mgtag);
    }

#endif

    /** Lock statement. */
    xSemaphoreTake( p_ctrl->p_tx_mutex_handle , portMAX_DELAY);
    {
        while(1)
        {
        /** Write Ethernet frame. */
        if(ETHER_NETIF_CFG_FB_MODE_POINTER & p_frame->buffer_mode)
        {
            fsp_err = p_ctrl->p_ether_instance->p_api->write(
                    p_ctrl->p_ether_instance->p_ctrl, p_frame->p_buffer, p_frame->length );
        }
        else
        {
            fsp_err = p_ctrl->p_ether_instance->p_api->write(
                    p_ctrl->p_ether_instance->p_ctrl, p_frame->buffer, p_frame->length );
        }
            /** Resend until possible */
            if(fsp_err == FSP_ERR_ETHER_ERROR_TRANSMIT_BUFFER_FULL)
            {
                continue;
            }
            break;
        }
    }
    /** UNLOCK */
    xSemaphoreGive( p_ctrl->p_tx_mutex_handle );

    /** Return FSP error of r_ether. */
    return (usr_err_t)fsp_err;
}

/*******************************************************************************************************************//**
 * @brief Monitor the link state of Ethernet peripherals via driver.
 *
 * @param[in] p_ctrl            Pointer to the controller
 *
 * @retval USR_SUCCESS            Process has been done successfully.
 * @retval USR_ERR_ABORTED        Fail to the process.
 **********************************************************************************************************************/
/**********************************************************************************************************************
* Function Name: um_ether_netif_ether_monitor
* Description  : Monitor the link state of Ethernet peripherals via driver.
* Arguments    : Pointer to the controller, Pointer to the link status buffer
* Return Value : USR_SUCCESS           Process has been done successfully.
*                USR_ERR_IN_USE        Resources are busy.
*********************************************************************************************************************/
usr_err_t um_ether_netif_ether_monitor (ether_netif_ether_ctrl_t * const p_ctrl, uint32_t * p_link_status )
{
    /** Error status */
    fsp_err_t fsp_err;

    /** Lock statement. */
    xSemaphoreTake( p_ctrl->p_mutex_handle , portMAX_DELAY);

    /** Try linking the process */
    fsp_err = p_ctrl->p_ether_instance->p_api->linkProcess(p_ctrl->p_ether_instance->p_ctrl);

    /** UNLOCK */
    xSemaphoreGive( p_ctrl->p_mutex_handle );

#if UM_ETHER_NETIF_FEATURE_R_ETHER_IMPL_R_ETHER

    /** Record port status */
    *p_link_status = ( FSP_SUCCESS == fsp_err ) ? ETHER_NETIF_PORT_BIT(0) : ETHER_NETIF_PORT_NONE;

#endif

#if UM_ETHER_NETIF_FEATURE_R_ETHER_IMPL_R_GMAC

    /** Check the link state of multiple ports */
    uint8_t port = 0;
    /** link status */
    gmac_link_status_t port_status;

    /** Clear link status. */
    (*p_link_status) &= (~ETHER_NETIF_CFG_PORT_ALL);

    for( port = 0; port < BSP_FEATURE_GMAC_MAX_PORTS; port++ )
    {
        /** Check the link state of single port. */
        xSemaphoreTake( p_ctrl->p_mutex_handle , portMAX_DELAY);

        fsp_err = R_GMAC_GetLinkStatus(p_ctrl->p_ether_instance->p_ctrl, port, &port_status);

        /** UNLOCK */
        xSemaphoreGive( p_ctrl->p_mutex_handle );

        /** If link status can be gotten correctly and is ready, the port is linking. */
        if( (FSP_SUCCESS == fsp_err) && (GMAC_LINK_STATUS_READY == port_status) )
        {
            /** CODE CHECKER, this is OK as a comment aligns with the cast*/
            (*p_link_status) |= ETHER_NETIF_CFG_PORT_BIT(port);
        }

        /** Otherwise, the port is not linking. */
        else
        {
            /** CODE CHECKER, this is OK as a comment aligns with the cast*/
            (*p_link_status) &= (~ETHER_NETIF_CFG_PORT_BIT(port));
        }
    }
#endif

    /** If there are linking port, return success.*/
    return ( (*p_link_status) & ETHER_NETIF_PORT_MASK ) ?  USR_SUCCESS : USR_ERR_ABORTED ;
}

/**********************************************************************************************************************
* Function Name: vEtherISRCallback
* Description  : Callback handler to set to Ethernet module instance
* Arguments    : Pointer to ether instance callback arguments
* Return Value : None
*********************************************************************************************************************/
void vEtherISRCallback (ether_callback_args_t * p_args)
{
    /** Flag for requesting the RTOS to switch the context to highest priority task. */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /** Get the instance as the context. */
    ether_netif_ether_ctrl_t * const p_ctrl = sp_r_ether_callback_context;

    /** Task handle for notification */
    TaskHandle_t p_task_handle;

    /** Check Callback event */
    switch ( p_args->event )
    {

#if UM_ETHER_NETIF_FEATURE_R_ETHER_IMPL_R_ETHER
        case ETHER_EVENT_INTERRUPT:
            if (p_args->status_eesr & ETHER_NETIF_INTERRUPT_FACTOR_RECEPTION)
#endif
#if UM_ETHER_NETIF_FEATURE_R_ETHER_IMPL_R_GMAC
        case ETHER_EVENT_SBD_INTERRUPT: /** Case that Ether module interrupts. */
            if (p_args->status_ether & ETHER_NETIF_INTERRUPT_FACTOR_RECEPTION)
#endif
            {
                /** Get task handle of reader task */
                (void) um_ether_netif_task_reader_get_task_handle(p_ctrl->p_reader_ctrl, &p_task_handle);

                if ( p_task_handle != NULL )
                {
                    vTaskNotifyGiveFromISR( p_task_handle, &xHigherPriorityTaskWoken);
                }
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            break;

        /** Break when other cases occur */
        default:
            break;
    }
}

/***********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

#if UM_ETHER_NETIF_FEATURE_R_ETHSW_SUPPORT_SPECIFIC_TAG
/**********************************************************************************************************************
* Function Name: build_tx_management_tag
* Description  : Build management tag for Ethernet frame to be sent.
* Arguments    : p_ctrl        Pointer to controller
*                p_frame       Pointer to Ethernet frame structure of network interface.
*                p_mgtag       Pointer to management tag to be built.
* Return Value : USR_SUCCESS
*********************************************************************************************************************/
static usr_err_t build_tx_management_tag ( ether_netif_ether_ctrl_t * const p_ctrl,
        ether_netif_frame_t * const p_frame, ether_mgtag_t * p_mgtag )
{
    /** Resolve the gmac extended configuration. */
    ethsw_extend_cfg_t const * p_gmac_extend_cfg = g_ethsw0.p_cfg->p_extend;

    /** Control datum of management tag. */
    uint32_t control_data  = 0x0000U;
    uint32_t control_data2 = 0x00000000UL;

    /** dummy */
    (void)(&p_ctrl);
    /** Set ports for forwarding. */
    switch( p_frame->port )
    {
        /** Disable Forced Forwarding. */
        case ETHER_NETIF_CFG_PORT_SEND_PORT_ANY:
            /** CODE CHECKER, this is OK as a comment aligns with the cast*/
            control_data &= ((uint32_t) ~GMAC_TX_MGTAG_DATA1_FORWARD_FORCE);
            break;

        /** Enable Forced Forwarding, and set the all ports as targets */
        case ETHER_NETIF_CFG_PORT_SEND_PORT_ALL:
            /** CODE CHECKER, this is OK as a comment aligns with the cast*/
            control_data |= (uint32_t) GMAC_TX_MGTAG_DATA1_FORWARD_FORCE;
            /** CODE CHECKER, this is OK as a comment aligns with the cast*/
            control_data2 |= (uint32_t) GMAC_TX_MGTAG_DATA2_PORT_MASK;
            break;

        /** Enable Forced Forwarding, and Set the specified ports. */
        default:
            control_data  |= GMAC_TX_MGTAG_DATA1_FORWARD_FORCE;
            control_data2 |= ((p_frame->port << GMAC_TX_MGTAG_DATA2_PORT_SHIFT ) & GMAC_TX_MGTAG_DATA2_PORT_MASK );
            break;
    }

    /** Build management tag. */
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    p_mgtag->control_tag[0]   = (uint8_t)( ( p_gmac_extend_cfg->specific_tag_id >> 8 ) & 0x00FF ) ;
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    p_mgtag->control_tag[1]   = (uint8_t)( ( p_gmac_extend_cfg->specific_tag_id      ) & 0x00FF ) ;
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    p_mgtag->control_data[0]  = (uint8_t)( ( control_data >> 8 ) & 0xFFU );
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    p_mgtag->control_data[1]  = (uint8_t)( ( control_data      ) & 0xFFU );
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    p_mgtag->control_data2[0] = (uint8_t)( ( control_data2 >> 24 ) & 0xFFUL );
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    p_mgtag->control_data2[1] = (uint8_t)( ( control_data2 >> 16 ) & 0xFFUL );
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    p_mgtag->control_data2[2] = (uint8_t)( ( control_data2 >> 8  ) & 0xFFUL );
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    p_mgtag->control_data2[3] = (uint8_t)( ( control_data2       ) & 0xFFUL );

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: fetch_rx_management_tag
* Description  : Fetch management tag information from received Ethernet frame
* Arguments    : p_ctrl        Pointer to controller
*                p_frame       Pointer to Ethernet frame structure of network interface.
*                p_mgtag       Pointer to management tag to be built.
* Return Value : USR_SUCCESS
*********************************************************************************************************************/
static usr_err_t fetch_rx_management_tag ( ether_netif_ether_ctrl_t * const p_ctrl,
        ether_netif_frame_t * const p_frame, ether_mgtag_t const * p_mgtag )
{
    /** Currently unused parameter. */
    (void) p_ctrl;

    /** Load control data */
    uint32_t control_data = (uint32_t) ( p_mgtag->control_data[0] << 8U ) + ( p_mgtag->control_data[1] );

    /** Get port where the received frame was received from. */
    p_frame->port = (uint32_t) ( (0x01) << (control_data & GMAC_RX_MGTAG_DATA1_PORT_MASK) );

    /** Return success code */
    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: insert_management_tag
* Description  : Insert management tag from Ethernet frame to be sent.
* Arguments    : p_frame       Pointer to Ethernet frame structure of network interface.
*                p_mgtag       Pointer to management tag to be built.
* Return Value : USR_SUCCESS
*********************************************************************************************************************/
static usr_err_t insert_management_tag ( ether_netif_frame_t *p_frame , ether_mgtag_t const * p_mgtag )
{

    /** buffer mode select */
    if(ETHER_NETIF_CFG_FB_MODE_POINTER & p_frame->buffer_mode)
    {
        /** Copy the management tag into Ethernet frame */
        memcpy(p_frame->buffer , p_frame->p_buffer , 12);
        /** Shift the buffer after Ether Type field to insert management tag by overwriting. */
        memcpy(&p_frame->buffer[12] , p_mgtag , sizeof(ether_mgtag_t));
        memcpy(&p_frame->buffer[12 + sizeof(ether_mgtag_t)] , &p_frame->p_buffer[12] , p_frame->length - 12);
        /** CODE CHECKER, this is OK as a comment aligns with the cast*/
        p_frame->buffer_mode &= (~(uint32_t)ETHER_NETIF_CFG_FB_MODE_POINTER);
    }
    else
    {
        /** Copy the management tag into Ethernet frame */
        /** Shift the buffer after Ether Type field to insert management tag by overwriting. */
        memmove(&p_frame->buffer[12 + sizeof(ether_mgtag_t)] , &p_frame->buffer[12] , p_frame->length - 12);
        memcpy(&p_frame->buffer[12] , p_mgtag , sizeof(ether_mgtag_t));
    }

    /** Add the length of management tag. */
    p_frame->length =  p_frame->length + sizeof(ether_mgtag_t);

    /** Return success code */
    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: remove_management_tag
* Description  : Remove management tag from received Ethernet frame
* Arguments    : p_frame       Pointer to Ethernet frame structure of network interface.
*                p_mgtag       Pointer to management tag to be built.
* Return Value : USR_SUCCESS
*********************************************************************************************************************/
static usr_err_t remove_management_tag ( ether_netif_frame_t *p_frame , ether_mgtag_t * p_mgtag )
{
    ether_frame_mgtag_t * p_mgtag_frame;
    ether_frame_t *       p_ether_frame;

    /** buffer mode select */
    if(ETHER_NETIF_CFG_FB_MODE_POINTER & p_frame->buffer_mode)
    {
        /** Resolve the Ethernet frame as it has a management tag. */
        p_mgtag_frame = (ether_frame_mgtag_t *) p_frame->p_buffer;

        /** Resolve the Ethernet frame as it does not have a management tag. */
        p_ether_frame = (ether_frame_t* ) p_frame->p_buffer;
    }
    else
    {
                /** Resolve the Ethernet frame as it has a management tag. */
        p_mgtag_frame = (ether_frame_mgtag_t *) p_frame->buffer;

        /** Resolve the Ethernet frame as it does not have a management tag. */
        p_ether_frame = (ether_frame_t* ) p_frame->buffer;
    }
    /** Copy the management tag from Ethernet frame */
    memcpy( p_mgtag, &p_mgtag_frame->mgtag, sizeof(ether_mgtag_t) );

    /** Shift the buffer after Ether Type field to remove management tag by overwriting. */
    memmove(p_ether_frame->type, p_mgtag_frame->type,
            p_frame->length - ((sizeof(p_mgtag_frame->dst_addr)) +
            (sizeof(p_mgtag_frame->src_addr)) + (sizeof(p_mgtag_frame->mgtag))));

    /** Subtract the length of management tag. */
    p_frame->length =  (uint16_t)(p_frame->length - sizeof(ether_mgtag_t));

    /** Return success code. */
    return USR_SUCCESS;
}
#endif
