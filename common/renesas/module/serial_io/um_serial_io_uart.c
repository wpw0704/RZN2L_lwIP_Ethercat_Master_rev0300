/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_serial_io_uart.c
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
#include "um_serial_io_api.h"
#include "um_common.h"

/** For FSP error codes */
#include "fsp_common_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/** For FSP Ethernet module. */
#include "r_uart_api.h"
#include "r_sci_uart.h"
#include "r_sci_uart_cfg.h"
#include <stdarg.h>
#include <stdio.h>

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define SCI_UART_BAUDRATE    (115200)

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
 * Imported function prototypes
 **********************************************************************************************************************/
/** r_uart_api callback function set by r_uart_cfg. */
/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/
static SemaphoreHandle_t  mutex_handle;
static SemaphoreHandle_t  tx_semaphore_handle;

static uart_instance_t const *p_ginst;
/**********************************************************************************************************************
 * Function Name: um_serial_io_uart_write
*********************************************************************************************************************/
static usr_err_t um_serial_io_uart_write(serial_io_data_t * const p_data_buffer );

/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/

/***********************************************************************************************************************
* Function Name: um_serial_io_uart_open
* Description  : Initialize the controller.
* Arguments    : Pointer to the controller
*                Pointer to the instance of uart interface
* Return Value : USR_SUCCESS              Process has been done successfully.
*                USR_ERR_NOT_INITIALIZED  Initialization has been failed.
*********************************************************************************************************************/
usr_err_t um_serial_io_uart_open(uart_instance_t const *p_inst)
{
    /** Error codes */
    fsp_err_t fsp_err;

    /** Create a mutex for r_uart */
    mutex_handle = xSemaphoreCreateMutex();
    USR_ERROR_RETURN( mutex_handle, USR_ERR_NOT_INITIALIZED );

    /** Create a semaphore for transmission control. */
    tx_semaphore_handle = xSemaphoreCreateBinary();
    USR_ERROR_RETURN( tx_semaphore_handle, USR_ERR_NOT_INITIALIZED );
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    (void) xSemaphoreGive( tx_semaphore_handle );

    /** Open the target interface. */
    fsp_err = p_inst->p_api->open( p_inst->p_ctrl, p_inst->p_cfg );
    USR_ERROR_RETURN( FSP_SUCCESS == fsp_err || FSP_ERR_ALREADY_OPEN == fsp_err , USR_ERR_NOT_INITIALIZED );
    /** Set callback */
#if defined(BSP_MCU_GROUP_RA6M3)
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    fsp_err = p_ctrl->p_uart_instance->p_api->callbackSet(
            p_ctrl->p_uart_instance->p_ctrl, user_uart_callback,
            p_ctrl, &p_ctrl->callback_memory );
    USR_ERROR_RETURN( FSP_SUCCESS == fsp_err, USR_ERR_NOT_INITIALIZED );
#endif /* defined(BSP_MCU_GROUP_RA6M3) */
    /** RZT2M FSP v1.0.0 does NOT support callbackSet API. */
    p_ginst = p_inst;

    /** Return success code */
    return USR_SUCCESS;
}
/*******************************************************************************************************************//**
 * @brief Output character via UART interface
 *
 * @param[in] p_ctrl        Pointer to the controller
 *
 * @retval USR_SUCCESS        Process has been done successfully.
 **********************************************************************************************************************/
/***********************************************************************************************************************
* Function Name: um_serial_io_uart_write
* Description  : Output character via UART interface
* Arguments    : Pointer to the Send data
* Return Value : USR_SUCCESS              Process has been done successfully.
*********************************************************************************************************************/
static usr_err_t um_serial_io_uart_write(serial_io_data_t * const p_data_buffer )
{
/**
 * To improve transmission efficiency, the transmission has been changed
 * from 1 byte to character string transmission.
 * Please enable this if you need to convert "CR+LF" and "LF" to "CR".
 * UART_SEND_MODE 0 : disable   1 : enabled
 */
    /** Enter lock section. */
    xSemaphoreTake(mutex_handle,portMAX_DELAY);

    p_ginst->p_api->write( p_ginst->p_ctrl, p_data_buffer->buffer, p_data_buffer->length);
    /** Waiting for transmission to complete */
    (void) xSemaphoreTake( tx_semaphore_handle, portMAX_DELAY );

    /** UNLOCK */
    xSemaphoreGive(mutex_handle);

    /** Return success code. */
    return USR_SUCCESS;
}

/***********************************************************************************************************************
* Function Name: user_uart_callback
* Description  : callback function registered to uart interface
* Arguments    : Pointer to the callback memory
* Return Value :
*********************************************************************************************************************/
void user_uart_callback (uart_callback_args_t * p_args)
{

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /** Handle the UART event */
    switch (p_args->event)
    {
        /** Detect transmission completed. */
        case UART_EVENT_TX_COMPLETE:
        {
            /** Release the resource for transmission. */
            (void) xSemaphoreGiveFromISR( tx_semaphore_handle, &xHigherPriorityTaskWoken );
            break;
        }

        /** Other cases are not handled. TODO: Implement the reception sequence. */
        default:
            break;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*******************************************************************************************************************//**
 * @brief Override write function of standard library.
 **********************************************************************************************************************/
/***********************************************************************************************************************
* Function Name: printf
* Description  : Override write function of standard library.
* Arguments    : Conforming to the library
* Return Value : Number of characters sent
*********************************************************************************************************************/
int printf (const char *__restrict format, ...)
{
    /** Resolve context */
    serial_io_data_t *p_tx_data = NULL;

    int32_t ret_len;

    /** Allocate memory */
    USR_HEAP_ALLOCATE( p_tx_data, sizeof(serial_io_data_t) );

    /** Resolve formats */
    va_list args;
    va_start (args, format);
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    ret_len = vsprintf( (char *) p_tx_data->buffer, format, args);

    /** Check the error */
    if( ret_len < 0 )
    {
        return ret_len;
    }

    /** Request the task to write buffer. */
    p_tx_data->length = (uint32_t) ret_len;
    (void) um_serial_io_uart_write( p_tx_data );
    /** Close the va_list variable. */
    va_end(args);
    vPortFree(p_tx_data);

    /** return the length. */
    return (int) ret_len;
}

