/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :lwip_port_main.c
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
/** User module instance APIs. */
#include "um_lwip_port_api.h"     /** API of lwIP porting module */
#include "um_serial_io_api.h"       /** API of serial output for debugging */
#include "um_common.h"

/** For handling application task */
#include "FreeRTOS.h"
#include "queue.h"

/** lwIP modules */
#include "lwip/sockets.h"
#include "lwip/errno.h"
#include "lwip_add_on_main_api.h"
#include "lwip_port_main_api.h"
/** Standard library */
#include <stdlib.h>

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define TCP_SERVER_RECV_BUFFER_SIZE     (1600)
#define TCP_SERVER_PORT                 (8000)
#define TCP_SERVER_TASK_PRIORITY        (4)
#define TCP_SERVER_TASK_STACK_SIZE      (1024)
#define TCP_SERVER_TASK_NAME            "TCP Server task"
#define TCP_SERVER_INVALID_SOCKET       (-1)

/***********************************************************************************************************************
 * Private constants
 **********************************************************************************************************************/
#define SAMPLE_APP_VERSION_MAJOR (1)
#define SAMPLE_AAP_VERSION_MINOR (5)

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/
/**
 * Application (TCP server) control.
 */
typedef struct st_tcp_server_ctrl
{
    TaskHandle_t                    p_server_task_handle;                          ///< TCP listner Task handler.
    TaskHandle_t                    p_parent_task_handle;                          ///< Task handle of parent task.
    uint8_t                         recv_buffer[TCP_SERVER_RECV_BUFFER_SIZE];      ///< Receive buffer.

    int32_t                         max_fd;
    uint32_t                        num_of_socket;
    fd_set                          fdset_listners;
    fd_set                          fdset_clients;
    fd_set                          fdset_all;

    lwip_port_netif_state_t         lwip_netif_state;
    lwip_port_instance_t
    const * p_lwip_port_instance;             ///< The controller of lwIP port module
    lwip_port_callback_args_t       callback_memory;
    lwip_port_callback_link_node_t
    callback_link_node;
} tcp_server_ctrl_t;

/**********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
static void      tcp_server_lwip_port_callback( lwip_port_callback_args_t * p_arg );
static usr_err_t tcp_server_add_listener_socket( tcp_server_ctrl_t * p_ctrl );
static usr_err_t tcp_server_remove_listener_socket( tcp_server_ctrl_t * p_ctrl );
static void      tcp_server_task( void* pvParameter );
static usr_err_t tcp_server_handle_listner_socket( tcp_server_ctrl_t * p_ctrl, int32_t listner_socket_fd );
static usr_err_t tcp_server_handle_connected_socket( tcp_server_ctrl_t * p_ctrl, int32_t connected_socket_fd );
static int32_t   tcp_server_get_socket_by_ip_address( uint32_t ip_address, int max_fd );
static int32_t   tcp_server_create_new_listner_socket( uint32_t ip_address, uint16_t port );

/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * Global Variables
 **********************************************************************************************************************/
/**
 * Platform module instances
 */
extern lwip_port_instance_t   const * gp_lwip_port0;

/**
 * TCP server module instance.
 */
static tcp_server_ctrl_t g_tcp_server0_ctrl;

static tcp_server_ctrl_t * gp_tcp_server0_ctrl = &g_tcp_server0_ctrl;

/**
 * Define the errno for lwIP BSD socket interface (EWARM only)
 */
#if defined(__ICCARM__)
int errno;
#endif

/*******************************************************************************
* Function Name: lwip_port_user_main
* Description  : Example codes for initializing and handling lwIP port module.
* Arguments    : None
* Return Value : None
*******************************************************************************/
void lwip_port_user_main(void)
{
    /** Status */
    usr_err_t   usr_err;      /** User status */

    BaseType_t  rtos_err;    /** Rtos status */

    /** Enabled the printf() function via SCI UART. */
    USR_LOG_INFO( "/** Started sample application (v%d.%d) for lwIP Port. **/",
            SAMPLE_APP_VERSION_MAJOR, SAMPLE_AAP_VERSION_MINOR);

    /******************************************************************************************************************
     * Check the link state
     ******************************************************************************************************************/
    /** Executes any add-on application option settings */
#if (defined _LWIP_ADD_ON_APP) && ((_LWIP_ADD_ON_APP & 7) != 0)
    static tcp_server_ctrl_t *tcp_server0_ctrl = NULL;
    USR_LOG_INFO( "Waiting for link-up..." );
    /* for LOG output */
    if(NULL == (tcp_server0_ctrl = pvPortMalloc(sizeof(tcp_server_ctrl_t))))
    {
        USR_LOG_ERROR( "Unable to get heap memory." );
        return;
    }

    /* wait for link up */
    while(1)
    {
        /* get linu-up status */
        usr_err =  gp_lwip_port0->p_api->netifStateGet(
                gp_lwip_port0->p_ctrl, &tcp_server0_ctrl->lwip_netif_state, false );
        if((USR_SUCCESS == usr_err) && (LWIP_PORT_NETIF_STATE_UP == tcp_server0_ctrl->lwip_netif_state))
        {
            /** heap free */
            vPortFree(tcp_server0_ctrl);
            break;
        }
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
    /* add-on */
    lwip_AddOn_main();
#endif /* _LWIP_ADD_ON_APP */

    /******************************************************************************************************************
     * Startup TCP Server application.
     ******************************************************************************************************************/
    /** Set lwIP port module instance. */
    gp_tcp_server0_ctrl->p_lwip_port_instance = gp_lwip_port0;
    /** Set current task handle for controlling the start of the task. */
    gp_tcp_server0_ctrl->p_parent_task_handle = xTaskGetCurrentTaskHandle();

    /** Clear socket descriptor sets */
    FD_ZERO(&gp_tcp_server0_ctrl->fdset_listners);
    FD_ZERO(&gp_tcp_server0_ctrl->fdset_clients);
    FD_ZERO(&gp_tcp_server0_ctrl->fdset_all);
    gp_tcp_server0_ctrl->max_fd = TCP_SERVER_INVALID_SOCKET;

    /** Set callback utilities into TCPIP network interface. */
    gp_tcp_server0_ctrl->callback_link_node.p_context = gp_tcp_server0_ctrl;
    gp_tcp_server0_ctrl->callback_link_node.p_memory  = &gp_tcp_server0_ctrl->callback_memory;
    gp_tcp_server0_ctrl->callback_link_node.p_func    = tcp_server_lwip_port_callback;
    gp_tcp_server0_ctrl->callback_link_node.p_next    = NULL;

    usr_err =  gp_lwip_port0->p_api->callbackAdd(
            gp_lwip_port0->p_ctrl, &gp_tcp_server0_ctrl->callback_link_node );
    if ( USR_SUCCESS != usr_err )
    {
        USR_LOG_ERROR( "Failed to set the callback function into TCP/IP stack." );
        return;
    }
    USR_LOG_INFO( "Set the callback function into TCP/IP stack." );

    /** Create application task. */
    rtos_err = xTaskCreate(
            tcp_server_task, TCP_SERVER_TASK_NAME,
            TCP_SERVER_TASK_STACK_SIZE / sizeof(StackType_t),
            gp_tcp_server0_ctrl, TCP_SERVER_TASK_PRIORITY,
            & gp_tcp_server0_ctrl->p_server_task_handle);

    if ( pdTRUE != rtos_err )
    {
        USR_LOG_ERROR( "Failed to create application task." );
        return;
    }
    USR_LOG_INFO( "Created sample application task." );

    /** Wait for notification indicating the created task is initialized. */
    (void) ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

    /** Suspend the created task. */
    vTaskSuspend( gp_tcp_server0_ctrl->p_server_task_handle );

    /** Resume the created task */
    vTaskResume( gp_tcp_server0_ctrl->p_server_task_handle );

    while( 1 )
    {
        /** Check the netif state */
        usr_err =  gp_lwip_port0->p_api->netifStateGet(
                gp_lwip_port0->p_ctrl,
                &gp_tcp_server0_ctrl->lwip_netif_state, true );
        /** If the netif is up. */
        if( LWIP_PORT_NETIF_STATE_UP == gp_tcp_server0_ctrl->lwip_netif_state )
        {
            break;
        }
        /** Wait 1s */
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}

/***********************************************************************************************************************
* Function Name: tcp_server_lwip_port_callback
* Description  : Handling callback from device
* Arguments    : Callback details
* Return Value : None
 **********************************************************************************************************************/
static void tcp_server_lwip_port_callback( lwip_port_callback_args_t * p_arg )
{
    /** Resolve context */
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    tcp_server_ctrl_t * p_ctrl = (tcp_server_ctrl_t *) p_arg->p_context;


    switch ( p_arg->event )
    {
        case LWIP_PORT_CALLBACK_EVENT_NETIF_UP:
            tcp_server_add_listener_socket(p_ctrl);
            break;

        case LWIP_PORT_CALLBACK_EVENT_NETIF_DOWN:

            tcp_server_remove_listener_socket(p_ctrl);
            break;

        default:
            break;
    }
}


/***********************************************************************************************************************
* Function Name: tcp_server_task
* Description  : TCP server task handling the BSD sockets.
* Arguments    : Information on the device used, etc.
* Return Value : None
***********************************************************************************************************************/
static void tcp_server_task( void* pvParameter )
{
    /** Resolve parameter */
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    tcp_server_ctrl_t * p_ctrl = (tcp_server_ctrl_t *) pvParameter;

    /** Notify the parenet task launch of this task. */
    xTaskNotifyGive( p_ctrl->p_parent_task_handle );

    /** fdsets for select */
    fd_set fdset_read;
    fd_set fdset_except;

    /** For scanning fd sets */
    int32_t num_sockets = 0;
    int32_t socket_fd   = -1;     /** Iterator */

    for(;;)
    {
        if ( p_ctrl->num_of_socket == 0 )
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        /** Get fdsets for detecting the read and except event. */
        memcpy(&fdset_read, &p_ctrl->fdset_all, sizeof(fd_set));
        memcpy(&fdset_except, &p_ctrl->fdset_all, sizeof(fd_set));

        /** Try select to detect event */
        num_sockets = (int32_t) lwip_select( p_ctrl->max_fd + 1, &fdset_read, NULL, &fdset_except, NULL );

        /** Continue if any events are detected. */
        if ( num_sockets <= 0 )
        {
            continue;
        }

        /** Check events */
        for( socket_fd = 0; socket_fd <= p_ctrl->max_fd ; socket_fd++ )
        {
            /** Check if the socket has event. */
            if ( (!(FD_ISSET(socket_fd, &fdset_read))) && (!(FD_ISSET(socket_fd, &fdset_except))) )
            {
                continue;
            }

            /** Check if the socket is listener socket. */
            if ( FD_ISSET(socket_fd, &p_ctrl->fdset_listners) )
            {
                (void)tcp_server_handle_listner_socket(p_ctrl, socket_fd);
                continue;
            }

            /** Check if the socket is client socket. */
            if ( FD_ISSET(socket_fd, &p_ctrl->fdset_clients ) )
            {
                (void) tcp_server_handle_connected_socket(p_ctrl, socket_fd);
                continue;
            }
        }
    }
}


/***********************************************************************************************************************
* Function Name: tcp_server_add_listener_socket
* Description  : Add listener socket
* Arguments    : Server Feature Management Data
* Return Value : usr_err_t
 **********************************************************************************************************************/
static usr_err_t tcp_server_add_listener_socket( tcp_server_ctrl_t * p_ctrl )
{
    /** Status */
    usr_err_t usr_err;

    /** For getting IP information */
    lwip_port_netif_cfg_t netif_cfg;

    /** Listener socket */
    int32_t listener_socket_fd;

    /** Get IP address */
    usr_err = p_ctrl->p_lwip_port_instance->p_api->netifConfigGet( p_ctrl->p_lwip_port_instance->p_ctrl, &netif_cfg );
    USR_ERROR_RETURN( USR_SUCCESS == usr_err, USR_ERR_ABORTED );

    /** Check if the IP address is enabled. */
    listener_socket_fd = tcp_server_get_socket_by_ip_address( netif_cfg.ip_address, p_ctrl->max_fd );
    USR_ERROR_RETURN( TCP_SERVER_INVALID_SOCKET == listener_socket_fd, USR_ERR_ABORTED );

    /** Create new listener socket */
    listener_socket_fd = tcp_server_create_new_listner_socket( netif_cfg.ip_address , TCP_SERVER_PORT );
    USR_ERROR_RETURN( TCP_SERVER_INVALID_SOCKET  != listener_socket_fd, USR_ERR_ABORTED );

    /** Update listener socket informations */
    FD_SET( listener_socket_fd, &p_ctrl->fdset_listners );
    FD_SET( listener_socket_fd, &p_ctrl->fdset_all );
    if ( listener_socket_fd > p_ctrl->max_fd ) { p_ctrl->max_fd = listener_socket_fd; }
    p_ctrl->num_of_socket++;

    return USR_SUCCESS;
}

/***********************************************************************************************************************
* Function Name: tcp_server_remove_listener_socket
* Description  : Remove listener socket
* Arguments    : Server Feature Management Data
* Return Value : usr_err_t
 **********************************************************************************************************************/
static usr_err_t tcp_server_remove_listener_socket( tcp_server_ctrl_t * p_ctrl )
{
    /** Status */
    usr_err_t usr_err;
    /** Status */
    int32_t   soc_err;

    /** For getting IP information */
    lwip_port_netif_cfg_t netif_cfg;

    /** Listener socket */
    int32_t listener_socket_fd;

    /** Get IP address */
    usr_err = p_ctrl->p_lwip_port_instance->p_api->netifConfigGet( p_ctrl->p_lwip_port_instance->p_ctrl, &netif_cfg );
    USR_ERROR_RETURN( USR_SUCCESS == usr_err, USR_ERR_ABORTED );

    /** Get the target socket by referencing the IP address */
    listener_socket_fd = tcp_server_get_socket_by_ip_address( netif_cfg.ip_address, p_ctrl->max_fd );
    USR_ERROR_RETURN( TCP_SERVER_INVALID_SOCKET != listener_socket_fd, USR_ERR_ABORTED );

    /** Try closing the socket. */
    soc_err = lwip_close( listener_socket_fd );
    USR_ERROR_RETURN( TCP_SERVER_INVALID_SOCKET != soc_err, USR_ERR_ABORTED );

    /** Update listener socket informations */
    FD_CLR( listener_socket_fd, &p_ctrl->fdset_listners );
    FD_CLR( listener_socket_fd, &p_ctrl->fdset_all );
    p_ctrl->num_of_socket--;

    /** TODO: Update mac_fd */
    return USR_SUCCESS;
}


/***********************************************************************************************************************
* Function Name: tcp_server_handle_listner_socket
* Description  : Handle listener socket
* Arguments    : Server Feature Management Data,Client data
* Return Value : usr_err_t
 **********************************************************************************************************************/
static usr_err_t tcp_server_handle_listner_socket( tcp_server_ctrl_t * p_ctrl, int32_t listner_socket_fd )
{
    /** new client socket information  */
    int32_t client_socket_fd;
    /** client addr */
    struct sockaddr_in client_addr;
    /** addr len */
    socklen_t client_addr_len;

    /** Accept */
    client_socket_fd = lwip_accept(listner_socket_fd, (struct sockaddr*) &client_addr, &client_addr_len);
    USR_ERROR_RETURN( -1 != client_socket_fd, USR_ERR_ABORTED );

    /** Update fd sets */
    FD_SET( client_socket_fd, &p_ctrl->fdset_clients );
    FD_SET( client_socket_fd, &p_ctrl->fdset_all );
    if ( client_socket_fd > p_ctrl->max_fd ) { p_ctrl->max_fd = client_socket_fd; }
    p_ctrl->num_of_socket++;

    /** Return success code. */
    return USR_SUCCESS;
}

/***********************************************************************************************************************
* Function Name: tcp_server_handle_connected_socket
* Description  : Handle connected socket
* Arguments    : Server Feature Management Data,Socket Handle
* Return Value : usr_err_t
 **********************************************************************************************************************/
static usr_err_t tcp_server_handle_connected_socket( tcp_server_ctrl_t * p_ctrl, int32_t connected_socket_fd )
{
    /** return values of recv and send */
    ssize_t recv_size;
    ssize_t sent_size;

    /** socket error */
    int32_t soc_err;

    /** Receive TCP packet. */
    recv_size = lwip_recv(connected_socket_fd, p_ctrl->recv_buffer, TCP_SERVER_RECV_BUFFER_SIZE, 0);

    /** If the packet is not received, check each error check by seeing errno. */
    if ( recv_size <= 0 )
    {
        /** Check if recv() is timed out.*/
        /** This block must be not reachable because it is ensured by select() that the client socket is readable. */
        USR_ERROR_RETURN( errno != EWOULDBLOCK, USR_ERR_ABORTED );
        /** Check if the socket is disconnected. */
        if (errno == ENOTCONN)
        {
            /** Close the disconnected client socket. */
            soc_err = lwip_close( connected_socket_fd );
            USR_ERROR_RETURN( TCP_SERVER_INVALID_SOCKET != soc_err, USR_ERR_ABORTED );

            /** Update listener socket informations */
            FD_CLR( connected_socket_fd, &p_ctrl->fdset_clients );
            FD_CLR( connected_socket_fd, &p_ctrl->fdset_all );
            p_ctrl->num_of_socket--;
            /** TODO: Update max_fd */
        }
    }

    /** Response with TCP packet. Here, it is simple echo. */
    sent_size = lwip_send( connected_socket_fd, p_ctrl->recv_buffer, (size_t) recv_size, 0);
    USR_ERROR_RETURN( sent_size > 0, USR_ERR_ABORTED);

    return USR_SUCCESS;
}

/***********************************************************************************************************************
* Function Name: tcp_server_get_socket_by_ip_address
* Description  : Check if the IP address is already used by existing sockets
* Arguments    : Client IP address, Last connection handle
* Return Value : connection handle
 **********************************************************************************************************************/
static int32_t tcp_server_get_socket_by_ip_address( uint32_t ip_address, int max_fd )
{
    /** lwip Function parameters */
    int socket_err = TCP_SERVER_INVALID_SOCKET;
    /** lwip Function parameters */
    int socket_fd  = 0;
    /** socket address */
    struct sockaddr_in socket_addr;
    /**  socket length */
    socklen_t socket_addr_len;

    /** Check if the IP address is already used in existing sockets */
    for ( socket_fd = 0; socket_fd <= max_fd ; socket_fd++ )
    {
        /** CODE CHECKER, this is OK as a comment aligns with the cast*/
        socket_err = lwip_getsockname( socket_fd, (struct sockaddr*) &socket_addr, &socket_addr_len );
        if ( TCP_SERVER_INVALID_SOCKET == socket_err )
        {
            /** Is the socket is already closed? */
            continue;
        }

        if ( socket_addr.sin_addr.s_addr == ip_address )
        {
            /** The IP address is already used. */
            return socket_fd;
        }
    }

    /** The IP address is not used yet. */
    return TCP_SERVER_INVALID_SOCKET;
}

/***********************************************************************************************************************
* Function Name: tcp_server_create_new_listner_socket
* Description  : Create new listner socket
* Arguments    : Client IP address, Ports Used
* Return Value : connection handle
 **********************************************************************************************************************/
static int32_t tcp_server_create_new_listner_socket( uint32_t ip_address, uint16_t port )
{
    /** lwip Function parameters */
    int socket_fd;
    /** lwip Function parameters */
    int socket_err;
    /** socket address */
    struct sockaddr_in socket_addr;

    /** Create listener socket */
    socket_fd = lwip_socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if ( TCP_SERVER_INVALID_SOCKET == socket_fd )
    {
        return TCP_SERVER_INVALID_SOCKET;
    }

    /** Bind the socket */
    socket_addr.sin_port        = htons(port);
    socket_addr.sin_family      = AF_INET;
    socket_addr.sin_addr.s_addr = ip_address;
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    socket_err = lwip_bind( socket_fd, (struct sockaddr*) &socket_addr, (socklen_t) sizeof(struct sockaddr));
    if ( TCP_SERVER_INVALID_SOCKET == socket_err )
    {
        socket_err = lwip_close(socket_fd);
        return TCP_SERVER_INVALID_SOCKET;
    }

    /** Start listening */
    lwip_listen(socket_fd, 20);
    if ( TCP_SERVER_INVALID_SOCKET == socket_err )
    {
        socket_err = lwip_close(socket_fd);
        return TCP_SERVER_INVALID_SOCKET;
    }

    return socket_fd;
}
