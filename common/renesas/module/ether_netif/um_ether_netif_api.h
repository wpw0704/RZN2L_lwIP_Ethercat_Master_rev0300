/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_ether_netif_api.h
 * Version      :1
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History :
 *
 *
 *********************************************************************************************************************/
#ifndef UM_ETHER_NETIF_API_H
#define UM_ETHER_NETIF_API_H

/*******************************************************************************************************************//**
 * @defgroup UM_ETHER_NETIF_API
 * @ingroup RENESAS_INTERFACES
 * @brief Interface for Ethernet network
 *
 * @section UM_ETHER_NETIF_API_SUMMARY Summary
 * The Ethernet network interface provides the tasks and APIs for the link layer of TCP/IP model.
 * - Provide the task for periodically checking the Ethernet Link
 * - Provide the task for receiving the Ethernet Frame
 * - Provide the task for sending the Ethernet Frame
 * - Require the FSP and Ethernet Interface (r_ether_api) to handling the above features.
 *
 * Implemented by:
 * - @ref UM_ETHER_NETIF
 *
 * @{
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
/** Include APIs */
#include "um_common.h"

/** Dependencies on FSP module. */
#include "r_ether_api.h"

/**********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
/** Ethernet port macros */
#define ETHER_NETIF_CFG_PORT_BIT(x)         ((uint32_t)(1)<<(x))

/** CODE CHECKER, this is OK as a comment aligns with the cast*/
#define ETHER_NETIF_CFG_PORT_NONE           ((uint32_t)(0x00000000))
/** CODE CHECKER, this is OK as a comment aligns with the cast*/
#define ETHER_NETIF_CFG_PORT_ALL            ((uint32_t)(0xFFFFFFFF))
/** CODE CHECKER, this is OK as a comment aligns with the cast*/
#define ETHER_NETIF_CFG_PORT_RECV_PORT_ANY  ((uint32_t)(0x00000000))
/** CODE CHECKER, this is OK as a comment aligns with the cast*/
#define ETHER_NETIF_CFG_PORT_SEND_PORT_ALL  ((uint32_t)(0xFFFFFFFF))
/** CODE CHECKER, this is OK as a comment aligns with the cast*/
#define ETHER_NETIF_CFG_PORT_SEND_PORT_ANY  ((uint32_t)(0x00000000))

/** Ethernet MAC address length */
#define ETHER_NETIF_CFG_MAC_ADDRESS_BYTES (6)
#define ETHER_NETIF_CFG_802_1Q_TAG_BYTES  (4)
#define ETHER_NETIF_CFG_ETHER_TYPE_BYTES  (2)

/** Ethernet frame header length */
#define ETHER_NETIF_CFG_FRAME_HEADER_BYTES ( \
        ETHER_NETIF_MAC_ADDRESS_BYTES * 2 + \
        ETHER_NETIF_802_1Q_TAG_BYTES + ETHER_NETIF_ETHER_TYPE_BYTES )

/** Ethernet MAC frame buffer size */
#define ETHER_NETIF_CFG_BUFFER_BYTES         (\
        BSP_STACK_ALIGNMENT * ( ( ( UM_ETHER_NETIF_CFG_MTU_SIZE +\
        ETHER_NETIF_FRAME_HEADER_BYTES ) / BSP_STACK_ALIGNMENT ) + 1 ) )

/**
 * ether_netif_frame_buffer_mode
 */
#define ETHER_NETIF_CFG_FB_MODE_POINTER (1)           /*  Array or pointer setting bit 0:Array  1:pointer */
#define ETHER_NETIF_CFG_FB_MODE_LWIP    (2)           /*  Buffer format when pointer is set 0:Byte  1:lwip(pbuf) */



/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/
/**
 * Enumerators for callback event.
 */
typedef enum e_ether_netif_callback_event
{
    ETHER_NETIF_CALLBACK_EVENT_LINK_UP,                ///< Event issued when any port linked up.
    ETHER_NETIF_CALLBACK_EVENT_LINK_DOWN,              ///< Event issued when all port linked down.
    ETHER_NETIF_CALLBACK_EVENT_RECEIVE_ETHER_FRAME,    ///< Event issued when any port receive Ethernet frame.
} ether_netif_callback_event_t;

/**
 * Structure for Ethernet frame packet.
 */
typedef struct st_ether_netif_frame
{
    uint32_t length;                                       ///< Length of the Ethernet frame.
    /* < Ethernet port where the Ethernet frame will be transmit or was received */
    uint32_t port;
    uint8_t  buffer[1600] __attribute__((aligned(8)));   ///< Buffer of the Ethernet frame.
    uint32_t buffer_mode;                                  ///< Usage status of array buffer or pointer buffer.
    uint8_t *p_buffer;                                   ///< pointer buffers of the Ethernet frame.
} ether_netif_frame_t;

/**
 * Structure for callback argument.
 */
typedef struct st_ether_netif_callback_args
{
    ether_netif_callback_event_t event;        ///< Event when the callback issued.
    ether_netif_frame_t * p_frame_packet;    ///< Ethernet frame packet the callback issued.
    void const * p_context;                 ///< Placeholder for user data.
} ether_netif_callback_args_t;

/**
 * Structure for callback add
 */
typedef struct st_ether_netif_callback_link_node
{
    struct st_ether_netif_callback_link_node * p_next;     ///< Pointer to next elements
    void (* p_func)(ether_netif_callback_args_t *);        ///< Pointer to callback function.
    ether_netif_callback_args_t * p_memory;                ///< Pointer to callback arguments.
    /* < Bit0 = buffer mode, Bit1 = buffer format. For the "0 or 1" setting, refer to the macro. */
    int32_t callback_buffer_mode;
    /*  Pointer to user context set to p_callback_memory when callback is issued. */
    void const *                  p_context;
} ether_netif_callback_link_node_t;

/** ETHER_NETIF control block.
 * Allocate an instance specific control block to pass into the LINK API calls.
 *
 * @par Implemented as
 * - ether_netif_ctrl_t
 */
typedef void ether_netif_ctrl_t;

/**
 * User configuration structure, used in open function
 */
typedef struct st_ether_netif_cfg
{
    ether_instance_t const * p_ether_instance; ///< Pointer to ether instance
} ether_netif_cfg_t;

/** LINK API structure.
 * General LINK functions implemented at the HAL layer follow this API.
 */
typedef struct st_ether_netif_api
{
    /** Initialize the instance.
     * @par Implemented as
     * - @ref UM_ETHER_NETIF_Open()
     *
     * @param[in]   p_ctrl     Pointer to control block. Must be declared by user. Elements set here.
     * @param[in]   p_cfg      Pointer to configuration structure. All elements of this structure must be set by user.
     */
    usr_err_t (* open)(ether_netif_ctrl_t * const p_ctrl, ether_netif_cfg_t const * const p_cfg);
    /** Start tasks.
     * @par Implemented as
     * - @ref UM_ETHER_NETIF_Start()
     *
     * @param[in]   p_ctrl     Pointer to control block. Must be declared by user.
     */
    usr_err_t (* start)(ether_netif_ctrl_t * const p_ctrl );
    /** Stop tasks.
     * @par Implemented as
     * - @ref UM_ETHER_NETIF_Stop()
     *
     * @param[in]   p_ctrl     Pointer to control block. Must be declared by user.
     */
    usr_err_t (* stop)(ether_netif_ctrl_t * const p_ctrl );
    /** Allows driver to be reconfigured and may reduce power consumption.
     * @par Implemented as
     * - @ref UM_ETHER_NETIF_Close()
     *
     * @param[in]   p_ctrl     Control block set in @ref um_link_api_t::open call for this ether_.
     */
    usr_err_t (* close)(ether_netif_ctrl_t * const p_ctrl);
    /** Specify callback function and optional context pointer and working memory pointer.
     * @par Implemented as
     * - @ref UM_ETHER_NETIF_CallbackAdd()

     * @param[in]  p_ctrl            Pointer to control structure.
     * @param[in]  p_callback_node   Pointer to callback node
     */
    usr_err_t (* callbackAdd)(ether_netif_ctrl_t * const p_ctrl, ether_netif_callback_link_node_t * p_callback_node );
    /** Enqueue the Ethernet frame to be sent via Ethernet driver.
     * @par Implemented as
     * - @ref UM_ETHER_NETIF_Send()
     *
     * @param[in]  p_ctrl            Pointer to control structure.
     * @param[in]  p_packet_buffer   Pointer to data to write.
     */
    usr_err_t (* send)(ether_netif_ctrl_t * const p_ctrl, ether_netif_frame_t * const p_packet_buffer);
    /** Get the link status of all ports.
     * @par Implemented as
     * - @ref UM_ETHER_NETIF_Enqueue()
     *
     * @param[in]  p_api_ctrl       Pointer to control structure.
     * @param[out] p_link_status    Pointer to port status information of all ports.
     */
    usr_err_t (* linkStatusGet)(ether_netif_ctrl_t * const p_ctrl, uint32_t * p_link_status, uint8_t notify_callback);

} ether_netif_api_t;

/** This structure encompasses everything that is needed to use an instance of this interface. */
typedef struct st_ether_netif_instance
{
    ether_netif_ctrl_t      * p_ctrl;        ///< Pointer to the control structure for this instance
    ether_netif_cfg_t const * p_cfg;         ///< Pointer to the configuration structure for this instance
    ether_netif_api_t const * p_api;         ///< Pointer to the API structure for this instance
} ether_netif_instance_t;

/*******************************************************************************************************************//**
 * @} (end defgroup UM_ETHER_NETIF_API)
 **********************************************************************************************************************/

/** r_ether callback function set by r_ether_cfg. */


#endif  /** UM_ETHER_NETIF_API_H */
