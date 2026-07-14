/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_common.h
 * Version      :1
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History :
 *
 *
 *********************************************************************************************************************/

#ifndef UM_COMMON_H
#define UM_COMMON_H

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/** Common error codes */
typedef enum e_usr_err
{
    USR_SUCCESS                 = 0,
    USR_ERR_ABORTED                ,   /** Aborted */
    USR_ERR_ASSERTION           ,   /** Assertion error */

    USR_ERR_NOT_INITIALIZED     ,   /** Not initialized */
    USR_ERR_ALREADY_OPEN        ,   /** Already open    */
    USR_ERR_ALREADY_RUNNING     ,   /** Already running */

    USR_ERR_IN_USE                ,   /** Resource is lacked or locked */
    USR_ERR_NOT_ENABLED            ,   /** Resource is not enabled */

} usr_err_t;

/**********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
/** Return with error */
#define USR_ERROR_RETURN(a, err)                        \
    {                                                   \
        if ((a))                                        \
        {                                               \
            (void) 0;                  /* Do nothing */ \
        }                                               \
        else                                            \
        {                                               \
            return (err);                               \
        }                                               \
    }
/** */
#define USR_ERROR_RETURN_VOID(a)                        \
    {                                                   \
        if ((a))                                        \
        {                                               \
            (void) 0;                  /* Do nothing */ \
        }                                               \
        else                                            \
        {                                               \
            return ;                                    \
        }                                               \
    }


/** Heap Allocation */
#define USR_HEAP_ALLOCATE( pv, size )      {     \
    (pv) = pvPortMalloc( (size) );               \
    while( NULL == (pv) )                        \
    {                                            \
        (pv) = pvPortMalloc( (size) );             \
        vTaskDelay(1);                           \
    }                                            \
}

/** Heap release */
#define USR_HEAP_RELEASE( pv )  {                \
    if( pv )                                     \
    {                                            \
        vPortFree( pv );                         \
        (pv) = NULL;                             \
    }                                            \
}



/** Error return in lock section */
#define USR_LOCK_ERROR_RETURN(mutex, a, err)            \
    {                                                   \
        if ((a))                                        \
        {                                               \
            (void) 0; /* Do nothing */                     \
        }                                               \
        else                                            \
        {                                               \
            (void) xSemaphoreGive( (mutex) );                \
            return (err);                                 \
        }                                               \
    }

/** Assertion */
#define USR_ASSERT(a)              {        \
    USR_ERROR_RETURN((a), USR_ERR_ASSERTION)\
    }


/** Output Log */
/** Debug Print */
#define USR_DEBUG_PRINT(...)        (printf(__VA_ARGS__))
/** */
#define USR_LOG_INFO(...)              {   \
        USR_DEBUG_PRINT("\r\n>>Info : ");    \
        USR_DEBUG_PRINT(__VA_ARGS__);      \
        }
/** */
#define USR_LOG_WARN(...)               {  \
        USR_DEBUG_PRINT("\r\n>>Warning : "); \
        USR_DEBUG_PRINT(__VA_ARGS__);      \
        }
/** */
#define USR_LOG_ERROR(...)            {    \
        USR_DEBUG_PRINT("\r\n>>Error : ");   \
        USR_DEBUG_PRINT(__VA_ARGS__);      \
        }


#endif
