/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :lwip_add_on_main.c
 * Version      :1
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History :
 *
 *
 *********************************************************************************************************************/

/** lwip Add-on application */
#if (defined _LWIP_ADD_ON_APP) && ((_LWIP_ADD_ON_APP & 7 ) != 0)


/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
/** User module instance APIs. */
#include "um_lwip_port_api.h"     /** API of lwIP porting module */
#include "um_serial_io_api.h"       /** API of serial output for debugging */
#include "um_common.h"

/** For handling application task */
#include "FreeRTOS.h"
#include "queue.h"
#include <time.h>
#include <stdio.h>

/** lwIP modules */
#include "lwip/sockets.h"
#include "lwip/errno.h"
#include "lwip/opt.h"
#include "lwip/apps/sntp.h"
#include "lwip/netif.h"
#include "lwip/apps/lwiperf.h"
#include "lwip_add_on_main_api.h"


/** Standard library */
#include <stdlib.h>

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
/** configurable constants */
/** IP address of the NTP server used when running the SNTP client */
#define SNTP_SERVER_IP   "192.168.10.1"
/** IP address of the iperf2 server used when running the iperf2 client */
#define IPERF_SERVER_IP   "192.168.10.1"
/** Response wait timeout (seconds) when running Iperf2 server and client */
#define IPERF_TIME_OUT  (180)
/************************************************************************/


#define STATE_DISABLE    (0)

#define STATE_ENABIE    (1)

#define LWIP_APP_LWIPERF_CL (_LWIP_ADD_ON_APP & 4)
#define LWIP_APP_LWIPERF_SR (_LWIP_ADD_ON_APP & 2)
#define LWIP_APP_SNTP       (_LWIP_ADD_ON_APP & 1)

/***********************************************************************************************************************
 * Private constants
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Global function prototypes
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
static void iperf_sr_report_cb(
        void *, enum lwiperf_report_type ,
        const ip_addr_t*, u16_t,
        const ip_addr_t*, u16_t, u32_t,
        u32_t, u32_t);
static void iperf_cl_report_cb(
        void *, enum lwiperf_report_type ,
        const ip_addr_t*, u16_t,
        const ip_addr_t*, u16_t, u32_t,
        u32_t, u32_t);

/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/
static uint8_t iperf_sr_report_buff[512];

static uint8_t iperf_cl_report_buff[512];
static uint8_t sntp_buff[36];


/***********************************************************************************************************************
 * Global Variables
 **********************************************************************************************************************/


/**********************************************************************************************************************
* Function Name: lwip_AddOn_main
* Description  : Some of the lwIP add-on applications run here.
* Arguments    : None
* Return Value : None
*
*********************************************************************************************************************/
void lwip_AddOn_main(void)
{
    char host_ip[32];
    int timeOut = 0;

    /** Starting the boot process for each add-on function
     * Bit0 SNTP Start the client function and synchronize with the system clock (rtc)
    */
    if(LWIP_APP_SNTP)
    {
        /** operating_mode  one of the available operating modes */
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        /** Specify the server by IP address */
        inet_pton(AF_INET,SNTP_SERVER_IP,host_ip);
        sntp_setserver(0, (const ip_addr_t *)host_ip); /** CODE CHECKER, this is OK as a comment aligns with the cast*/
        sntp_init();
        USR_LOG_INFO( "<< SNTP client application start >>" );
        /** If SNTP is enabled and the clock is not displayed, get it */
        /** LOG output if it can be acquired normally */
        timeOut = (SNTP_RETRY_TIMEOUT_MAX / 1000);
        sntp_buff[0] = STATE_DISABLE;
        while(1)
        {
            /** SNTP timeout standard 150S*/
            if(sntp_buff[0] != STATE_DISABLE)
            {
                USR_LOG_INFO( "<< SNTP client application completed [ %s ] >> \n",&sntp_buff[4]);
                break;
            }
            --timeOut;
            if(timeOut == 0)
            {
                /** end with error */
                USR_LOG_INFO( "<< SNTP client application end with error >>");
                break;
            }
            /** Wait 1s */
            vTaskDelay( 1000 / portTICK_PERIOD_MS );
        }
        /** SNTP end processing */
        sntp_stop();
        vTaskDelay( 500 / portTICK_PERIOD_MS );
    }

    /** Bit1 lwiperf tart the server function */
    if(LWIP_APP_LWIPERF_SR)
    {
        /** Specify the server by IP address */
        iperf_sr_report_buff[0] = STATE_DISABLE;
        lwiperf_start_tcp_server_default(iperf_sr_report_cb, NULL);
        USR_LOG_INFO( "<< IPERF2 server application start >>");
        /** Timeout for waiting for report is 3 minutes */
        timeOut = IPERF_TIME_OUT;
        while(1)
        {
            /* SNTP timeout standard 150S*/
            if(iperf_sr_report_buff[0] != STATE_DISABLE)
            {
                USR_LOG_INFO( "<< IPERF2 server application completed >>\n%s",&iperf_sr_report_buff[4]);
                break;
            }
            --timeOut;
            if(timeOut == 0)
            {
                /* end with error */
                USR_LOG_INFO( "<< IPERF2 server application end with error >>");
                break;
            }
            /** Wait 1s */
            vTaskDelay( 1000 / portTICK_PERIOD_MS );
        }

    /* Bit1 lwipef tart the client function */
    }
    if(LWIP_APP_LWIPERF_CL)
    {
        /** Specify the server by IP address */
        iperf_cl_report_buff[0] = STATE_DISABLE;
        inet_pton(AF_INET,IPERF_SERVER_IP,host_ip);
        /** CODE CHECKER, this is OK as a comment aligns with the cast*/
        lwiperf_start_tcp_client_default((const ip_addr_t *)host_ip, iperf_cl_report_cb, NULL);
        /** Timeout for waiting for report is 3 minutes */
        timeOut = IPERF_TIME_OUT;
        USR_LOG_INFO( "<< IPERF2 client application start >>");
        while(1)
        {
            /* SNTP timeout standard 150S*/
            if(iperf_cl_report_buff[0] != STATE_DISABLE)
            {
                USR_LOG_INFO( "<< IPERF2 client application completed >>\n%s",&iperf_cl_report_buff[4]);
                break;
            }
            --timeOut;
            if(timeOut == 0)
            {
                /* end with error */
                USR_LOG_INFO( "<< IPERF2 client application end with error >>");
                break;
            }
            /** Wait 1s */
            vTaskDelay( 1000 / portTICK_PERIOD_MS );
        }
    }

}

/**
 * Data storage processing when receiving SNTP
 * Convert calendar data acquired by SNTP to character strin
 * @param sec Data at time of reception
 */
/**********************************************************************************************************************
* Function Name: lwip_sntp_set_system_time_cb
* Description  : lwIP add-on
*********************************************************************************************************************/
void lwip_sntp_set_system_time_cb(u32_t sec)
{
    struct tm *current_time_val;
    time_t temp_time;

    /* if not received */
    if(sec == 0)
    {
        return;
    }
    /* Convert binary data to calendar format */
    temp_time = (time_t)sec;
    current_time_val= localtime(&temp_time);
    /* convert to string */
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    strftime((char *)&sntp_buff[4], 32, "%d.%m.%Y %H:%M:%S", current_time_val);
    sntp_buff[0] = STATE_ENABIE;
}

/**
 * In the report data callback process,
 * save the received data in LOG format.
 */
/**********************************************************************************************************************
* Function Name: iperf_sr_report_cb
* Description  : lwIP add-on
*********************************************************************************************************************/
static void iperf_sr_report_cb(void *arg, enum lwiperf_report_type report_type,
        const ip_addr_t* local_addr, u16_t local_port,
        const ip_addr_t* remote_addr, u16_t remote_port,
        u32_t bytes_transferred, u32_t ms_duration,
        u32_t bandwidth_kbitpsec)
{
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(local_addr);
    LWIP_UNUSED_ARG(local_port);

    if(bytes_transferred == 0)
    {
        return;
    }
    /* convert and save */
    sprintf((char *)&iperf_sr_report_buff[4],
        "IPERF report: type=%d, remote: %s:%d, total bytes: %"U32_F", duration in ms: %"U32_F", kbits/s: %"U32_F"\n",
    (int)report_type, ipaddr_ntoa(remote_addr), (int)remote_port, bytes_transferred, ms_duration, bandwidth_kbitpsec);
    
    iperf_sr_report_buff[0] = STATE_ENABIE;

}
/**********************************************************************************************************************
* Function Name: iperf_cl_report_cb
* Description  : lwIP add-on
*********************************************************************************************************************/
static void iperf_cl_report_cb(void *arg, enum lwiperf_report_type report_type,
        const ip_addr_t* local_addr, u16_t local_port,
        const ip_addr_t* remote_addr, u16_t remote_port,
        u32_t bytes_transferred, u32_t ms_duration,
        u32_t bandwidth_kbitpsec)
{
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(local_addr);
    LWIP_UNUSED_ARG(local_port);

    if(bytes_transferred == 0)
    {
        return;
    }
    /* convert and save */
    sprintf((char *)&iperf_cl_report_buff[4],
        "IPERF report: type=%d, remote: %s:%d, total bytes: %"U32_F", duration in ms: %"U32_F", kbits/s: %"U32_F"\n",
    (int)report_type, ipaddr_ntoa(remote_addr), (int)remote_port, bytes_transferred, ms_duration, bandwidth_kbitpsec);
    
    iperf_cl_report_buff[0] = STATE_ENABIE;

}

#endif /* _LWIP_ADD_ONAPP */

