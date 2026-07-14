/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_serial_io_api.h
 * Version      :1
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History :
 *
 *
 *********************************************************************************************************************/

#ifndef UM_SERIAL_IO_API_H
#define UM_SERIAL_IO_API_H

/*******************************************************************************************************************//**
 * @defgroup TCPIP network interface
 * @ingroup RENESAS_INTERFACES
 * @brief Interface for UM_SERIAL_IO functions.
 *
 * @section UM_SERIAL_IO_API_SUMMARY Summary
 * The UM_SERIAL_IO interface provides the network interface for utilizing TCP/IP communication.
 * - Provide the network interface for TCP/IP communication with TCP/IP protocol stack.
 *
 * Implemented by:
 * - @ref UM_SERIAL_IO
 *
 * @{
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
/** Include APIs */
#include "um_common.h"
#include "r_uart_api.h"

/**********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
/** Buffer of string size */
#define SERIAL_IO_CFG_MAX_DATA_LENGTH       (1024U)


/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/
/**
 * Structure for Ethernet frame packet.
 */
typedef struct serial_io_data_t
{
    uint32_t length;              ///< Length of string.
    uint8_t  buffer[SERIAL_IO_CFG_MAX_DATA_LENGTH];     ///< Buffer of string.
} serial_io_data_t;

/** UM_SERIAL_IO control block. Allocate an instance specific control block to pass into the UM_SERIAL_IO API calls.
 * @par Implemented as
 * - lwip_instance_ctrl_t
 */
typedef void serial_io_ctrl_t;

/**
 * Structure for instance configuration.
 */
typedef struct st_serial_io_cfg
{
    uart_instance_t const * const p_uart_instance;
} serial_io_cfg_t;

/** UM_SERIAL_IO API structure. General net functions implemented at the HAL layer follow this API. */
typedef struct st_um_net_api
{
    /** Open network interface.
    * @par Implemented as
    * - @ref UM_LWIP_PORT_Open()
    * @return
    */
    usr_err_t (* open )( serial_io_ctrl_t * const p_ctrl, serial_io_cfg_t const * const p_cfg );

    /** Open network interface.
    * @par Implemented as
    * - @ref UM_LWIP_PORT_Close()
    * @return
    */
    usr_err_t (* close)( serial_io_ctrl_t * const p_ctrl );

    /** Start tasks.
    * @par Implemented as
    * - @ref UM_LWIP_PORT_Start()
    * @return
    */
    usr_err_t (* start)( serial_io_ctrl_t * const p_ctrl );

    /** Stop tasks
    * @par Implemented as
    * - @ref UM_LWIP_PORT_Stop()
    * @return
     */
    usr_err_t (* stop)( serial_io_ctrl_t * const p_ctrl );

    /** Write character
     * @par Implemented as
     * - @ref UM_LWIP_PORT_Write()
     * @return
     */
    usr_err_t (* write)( serial_io_ctrl_t * const p_ctrl, void const * const buffer, uint32_t length );

} serial_io_api_t;

/** This structure encompasses everything that is needed to use an instance of this interface. */
typedef struct st_serial_io_instance
{
    serial_io_ctrl_t      * p_ctrl;        ///< Pointer to the control structure for this instance
    serial_io_cfg_t const * p_cfg;         ///< Pointer to the configuration structure for this instance
    serial_io_api_t const * p_api;           ///< Pointer to the API structure for this instance
} serial_io_instance_t;

/**********************************************************************************************************************
 * Function Name: um_serial_io_uart_open
*********************************************************************************************************************/
usr_err_t um_serial_io_uart_open(uart_instance_t const*);
/**********************************************************************************************************************
 * Function Name: user_uart_callback
*********************************************************************************************************************/
void user_uart_callback (uart_callback_args_t * p_args);

/*******************************************************************************************************************//**
 * @} (end defgroup UM_SERIAL_IO_API)
 **********************************************************************************************************************/

#endif        /* UM_SERIAL_IO_API_H */
