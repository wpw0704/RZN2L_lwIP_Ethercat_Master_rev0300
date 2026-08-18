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
#include "crc_L16.h"
#include "io.h"
#include "ethercat_motion.h"
#include "ethercat_master.h"
/** For handling application task */
#include "FreeRTOS.h"
#include "queue.h"

/** lwIP modules */
#include "lwip/sockets.h"
#include "lwip/errno.h"
#include "lwip_add_on_main_api.h"
#include "lwip_port_main_api.h"
#include "host_protocol.h"
#include "host_command.h"
/** Standard library */
#include <stdlib.h>

#include "hal_data.h"
#include "r_crc_api.h"
/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define TCP_SERVER_RECV_BUFFER_SIZE     (1600)
#define TCP_SERVER_PORT                 (8000)
#define TCP_SERVER_TASK_PRIORITY        (3)
#define SEQUENCE_TASK_PRIORITY        (4)
#define TCP_SERVER_TASK_STACK_SIZE      (8192U)
#define TCP_SERVER_TASK_NAME            "TCP Server task"
#define TCP_SEND_TASK_NAME            "TCP Send task"
#define SEQUENCE_TASK_NAME           "Sequence task"
#define TCP_SERVER_INVALID_SOCKET       (-1)
#define TCP_SERVER_SEND_BUFFER_SIZE (100)

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
typedef struct st_tcp_server_ctrl {
    TaskHandle_t p_server_task_handle; ///< TCP listner Task handler.
    TaskHandle_t p_parent_task_handle; ///< Task handle of parent task.
    uint8_t recv_buffer[TCP_SERVER_RECV_BUFFER_SIZE]; ///< Receive buffer.

    int32_t max_fd;
    uint32_t num_of_socket;
    fd_set fdset_listners;
    fd_set fdset_clients;
    fd_set fdset_all;
    host_protocol_parser_t protocol_parser[MEMP_NUM_NETCONN];

    lwip_port_netif_state_t lwip_netif_state;
    lwip_port_instance_t
    const *p_lwip_port_instance; ///< The controller of lwIP port module
    lwip_port_callback_args_t callback_memory;
    lwip_port_callback_link_node_t
    callback_link_node;
} tcp_server_ctrl_t;

typedef enum {
    manual_control = 0,
    Sequence_control,
    PLC_control
}Servo_control_mode;

Servo_control_mode servo_control_mode = manual_control;
static uint8_t sequence_parameter[TCP_SERVER_RECV_BUFFER_SIZE] = {0};

/**********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
static void tcp_server_lwip_port_callback(lwip_port_callback_args_t *p_arg);

static usr_err_t tcp_server_add_listener_socket(tcp_server_ctrl_t *p_ctrl);

static usr_err_t tcp_server_remove_listener_socket(tcp_server_ctrl_t *p_ctrl);

static void tcp_server_task(void *pvParameter);

static void tcp_send_task(void *pvParameter);

static void sequence_task(void *pvParameter);

static void send_frame(uint8_t socket_fd);

static usr_err_t tcp_server_handle_listner_socket(tcp_server_ctrl_t *p_ctrl, int32_t listner_socket_fd);

static usr_err_t tcp_server_handle_connected_socket(tcp_server_ctrl_t *p_ctrl, int32_t connected_socket_fd);

static int32_t tcp_server_get_socket_by_ip_address(uint32_t ip_address, int max_fd);

static int32_t tcp_server_create_new_listner_socket(uint32_t ip_address, uint16_t port);


/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * Global Variables
 **********************************************************************************************************************/
/**
 * Platform module instances
 */
extern lwip_port_instance_t const *gp_lwip_port0;

extern ether_netif_instance_t const *gp_ether_netif0;
extern uint8_t sn595_data[SN595_DATA_COUNT];
static TaskHandle_t tcp_send_handle = NULL;
static TaskHandle_t sequence_handle = NULL;

extern uint8_t s_servo_enable_request;

extern control_state_t current_state;

extern motion_control_t s_control;

/**
 * TCP server module instance.
 */
static tcp_server_ctrl_t g_tcp_server0_ctrl;

static tcp_server_ctrl_t *gp_tcp_server0_ctrl = &g_tcp_server0_ctrl;

static uint8_t send_buf[TCP_SERVER_SEND_BUFFER_SIZE] = {0};

static float real_time_position = 0;

static float zero_init_position = 0;

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
void lwip_port_user_main(void) {
    /** Status */
    usr_err_t usr_err; /** User status */

    BaseType_t rtos_err; /** Rtos status */
    uint32_t link_status = 0U;
    uint32_t previous_link_status = 0U;
    uint32_t down_ports;
    uint32_t up_ports;
    uint8_t status_initialized = 0U;

    /** Enabled the printf() function via SCI UART. */
    USR_LOG_INFO("/** Started sample application (v%d.%d) for lwIP Port. **/",
                 SAMPLE_APP_VERSION_MAJOR, SAMPLE_AAP_VERSION_MINOR);

    /******************************************************************************************************************
     * Check the link state
     ******************************************************************************************************************/
    /** Executes any add-on application option settings */
#if (defined _LWIP_ADD_ON_APP) && ((_LWIP_ADD_ON_APP & 7) != 0)
    static tcp_server_ctrl_t *tcp_server0_ctrl = NULL;
    USR_LOG_INFO("Waiting for link-up...");
    /* for LOG output */
    if (NULL == (tcp_server0_ctrl = pvPortMalloc(sizeof(tcp_server_ctrl_t)))) {
        USR_LOG_ERROR("Unable to get heap memory.");
        return;
    }

    /* wait for link up */
    while (1) {
        /* get linu-up status */
        usr_err = gp_lwip_port0->p_api->netifStateGet(
            gp_lwip_port0->p_ctrl, &tcp_server0_ctrl->lwip_netif_state, false);
        if ((USR_SUCCESS == usr_err) && (LWIP_PORT_NETIF_STATE_UP == tcp_server0_ctrl->lwip_netif_state)) {
            /** heap free */
            vPortFree(tcp_server0_ctrl);
            break;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
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
    gp_tcp_server0_ctrl->callback_link_node.p_memory = &gp_tcp_server0_ctrl->callback_memory;
    gp_tcp_server0_ctrl->callback_link_node.p_func = tcp_server_lwip_port_callback;
    gp_tcp_server0_ctrl->callback_link_node.p_next = NULL;

    usr_err = gp_lwip_port0->p_api->callbackAdd(
        gp_lwip_port0->p_ctrl, &gp_tcp_server0_ctrl->callback_link_node);
    if (USR_SUCCESS != usr_err) {
        USR_LOG_ERROR("Failed to set the callback function into TCP/IP stack.");
        return;
    }
    USR_LOG_INFO("Set the callback function into TCP/IP stack.");
    current_state = IDLE_STATE;
    /** Create application task. */
    rtos_err = xTaskCreate(
        tcp_server_task, TCP_SERVER_TASK_NAME,
        TCP_SERVER_TASK_STACK_SIZE / sizeof(StackType_t),
        gp_tcp_server0_ctrl, TCP_SERVER_TASK_PRIORITY,
        &gp_tcp_server0_ctrl->p_server_task_handle);

    if (pdTRUE != rtos_err) {
        USR_LOG_ERROR("Failed to create application task.");
        return;
    }
    USR_LOG_INFO("Created sample application task.");

    rtos_err = xTaskCreate(
        tcp_send_task, TCP_SEND_TASK_NAME,
        TCP_SERVER_TASK_STACK_SIZE / sizeof(StackType_t),
        gp_tcp_server0_ctrl, TCP_SERVER_TASK_PRIORITY,
        &tcp_send_handle);
    if (pdTRUE != rtos_err) {
        USR_LOG_ERROR("Failed to create send task.");
        return;
    }
    USR_LOG_INFO("Created sample send task.");
    rtos_err = xTaskCreate(
        sequence_task, SEQUENCE_TASK_NAME,
        TCP_SERVER_TASK_STACK_SIZE / sizeof(StackType_t),
        sequence_handle, SEQUENCE_TASK_PRIORITY,
        &tcp_send_handle);
    if (pdTRUE != rtos_err) {
        USR_LOG_ERROR("Failed to create sequence_task.");
        return;
    }
    USR_LOG_INFO("Created sample send task.");

    /** Wait for notification indicating the created task is initialized. */
    (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // while (1) {
    /*
    * Replay the current netif state after registering the application callback.
    *
    * If the cable was already connected during power-on, NETIF_UP might have
    * occurred before tcp_server_lwip_port_callback() was registered. Setting
    * notify_callback to true replays the current UP/DOWN state to the callback.
    */
    usr_err = gp_lwip_port0->p_api->netifStateGet(
        gp_lwip_port0->p_ctrl,
        &gp_tcp_server0_ctrl->lwip_netif_state,
        true);

    if (USR_SUCCESS != usr_err) {
        USR_LOG_ERROR("Failed to notify current lwIP netif state: %d", usr_err);
    }
    /*
     * netif state changes after this point are handled by
     * tcp_server_lwip_port_callback().
     */
    vTaskDelete(NULL);
    // }
}

/***********************************************************************************************************************
* Function Name: tcp_server_lwip_port_callback
* Description  : Handling callback from device
* Arguments    : Callback details
* Return Value : None
 **********************************************************************************************************************/
static void tcp_server_lwip_port_callback(lwip_port_callback_args_t *p_arg) {
    /** Resolve context */
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    tcp_server_ctrl_t *p_ctrl = (tcp_server_ctrl_t *) p_arg->p_context;


    switch (p_arg->event) {
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
static void tcp_server_task(void *pvParameter) {
    /** Resolve parameter */
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    tcp_server_ctrl_t *p_ctrl = (tcp_server_ctrl_t *) pvParameter;

    /** Notify the parenet task launch of this task. */
    xTaskNotifyGive(p_ctrl->p_parent_task_handle);

    /** fdsets for select */
    fd_set fdset_read;
    fd_set fdset_except;

    /** For scanning fd sets */
    int32_t num_sockets = 0;
    int32_t socket_fd = -1; /** Iterator */

    for (;;) {
        if (p_ctrl->num_of_socket == 0) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        /** Get fdsets for detecting the read and except event. */
        memcpy(&fdset_read, &p_ctrl->fdset_all, sizeof(fd_set));
        memcpy(&fdset_except, &p_ctrl->fdset_all, sizeof(fd_set));
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; //100ms
        /** Try select to detect event */
        num_sockets = (int32_t) lwip_select(p_ctrl->max_fd + 1, &fdset_read, NULL, &fdset_except, NULL);

        /** Continue if any events are detected. */
        if (num_sockets <= 0) {
            continue;
        }

        /** Check events */
        for (socket_fd = 0; socket_fd <= p_ctrl->max_fd; socket_fd++) {
            /** Check if the socket has event. */
            if ((!(FD_ISSET(socket_fd, &fdset_read))) && (!(FD_ISSET(socket_fd, &fdset_except)))) {
                continue;
            }

            /** Check if the socket is listener socket. */
            if (FD_ISSET(socket_fd, &p_ctrl->fdset_listners)) {
                (void) tcp_server_handle_listner_socket(p_ctrl, socket_fd);
                continue;
            }

            /** Check if the socket is client socket. */
            if (FD_ISSET(socket_fd, &p_ctrl->fdset_clients)) {
                (void) tcp_server_handle_connected_socket(p_ctrl, socket_fd);
                continue;
            }
        }
    }
}

/***********************************************************************************************************************
* Function Name: tcp_server_handle_connected_socket
* Description  : Handle connected socket
* Arguments    : Server Feature Management Data,Socket Handle
* Return Value : usr_err_t
 **********************************************************************************************************************/
static usr_err_t tcp_server_handle_connected_socket(tcp_server_ctrl_t *p_ctrl, int32_t connected_socket_fd) {
    /** return values of recv and send */
    ssize_t recv_size;
    ssize_t sent_size;
    /** socket error */
    int32_t soc_err;
    static uint16_t command;


    /** Receive TCP packet. */
    recv_size = lwip_recv(connected_socket_fd, p_ctrl->recv_buffer, TCP_SERVER_RECV_BUFFER_SIZE, 0);

    /** 无数据可读时保留连接；连接关闭或其他错误时释放socket和解析状态。 */
    if (recv_size <= 0) {
        /** Check if recv() is timed out.*/
        /** This block must be not reachable because it is ensured by select() that the client socket is readable. */
        USR_ERROR_RETURN(errno != EWOULDBLOCK, USR_ERR_ABORTED);
        /** Check if the socket is disconnected. */
        if (errno == ENOTCONN) {
            soc_err = lwip_close(connected_socket_fd);
            USR_ERROR_RETURN(TCP_SERVER_INVALID_SOCKET != soc_err, USR_ERR_ABORTED);


            FD_CLR(connected_socket_fd, &p_ctrl->fdset_clients);
            FD_CLR(connected_socket_fd, &p_ctrl->fdset_all);
            p_ctrl->num_of_socket--;
        }
    }

    USR_LOG_INFO("socket %ld, max socket %ld,len %d", connected_socket_fd, p_ctrl->max_fd, recv_size);
    if (p_ctrl->recv_buffer[6] != 0x77 || p_ctrl->recv_buffer[7] != 0x44 ||
        p_ctrl->recv_buffer[recv_size - 4] != 0x77 || p_ctrl->recv_buffer[recv_size - 3] != 0x44) {
        USR_LOG_INFO("6 %02x 7 %02x %d %02x %d %02x", p_ctrl->recv_buffer[6], p_ctrl->recv_buffer[7], recv_size - 4,
                     p_ctrl->recv_buffer[recv_size - 4]
                     , recv_size - 3, p_ctrl->recv_buffer[recv_size - 3]);
        return USR_ERR_ABORTED;
        }

    uint32_t crc = crc_calc(p_ctrl->recv_buffer, (uint32_t) recv_size - 2);
    USR_LOG_INFO("Calculated CRC: 0x%04lX", crc);
    if (((uint32_t) p_ctrl->recv_buffer[recv_size - 2] << 8 | (uint32_t) p_ctrl->recv_buffer[recv_size - 1]) != crc) {
        USR_LOG_INFO("CRC ERR");
        return USR_ERR_ABORTED;
    }
    command = ((uint16_t) p_ctrl->recv_buffer[10] << 8) | (uint16_t) p_ctrl->recv_buffer[11];
    USR_LOG_INFO("Command: %04x", command);
    switch (command) {
        case 0x0001:
            uint8_t res = servo_enable_allowed();
            if (res == 1) {
                servo_control_mode = manual_control;
                USR_LOG_INFO("Enable servo");
            } else if (res == 0) {
                USR_LOG_INFO("Disable servo");
            } else {
                USR_LOG_INFO("Enable servo err %d", res);
            }
            break;
        case 0x0002:
            if ((input1s->StatusWord & CIA402_SW_MASK) == CIA402_SW_OPERATION_ENABLED && s_servo_enable_request == 1) {
                break;
            }
            current_state = SET_0RIGIN;
            /*
             * 让PDO任务先观察到SET_0RIGIN，停止旧轨迹计算。
             * 当前PDO周期为2ms。
             */
            zero_init_position = (double) (((uint32_t) p_ctrl->recv_buffer[15] << 24) | (
                                               (uint32_t) p_ctrl->recv_buffer[16] << 16)
                                           |
                                           ((uint32_t) p_ctrl->recv_buffer[17] << 8) | (
                                               (uint32_t) p_ctrl->recv_buffer[18] << 0)) / 10000.0;
            vTaskDelay(pdMS_TO_TICKS(10U));
            write16(1U, 0x2005U, 0x1FU, 6U);
            ethercat_motion_position_sync();
            USR_LOG_INFO("position sync actual=%ld target=%ld",
                         (long) input1s->CurrentPosition,
                         (long) output1s->TargetPos);
            current_state = IDLE_STATE;
            ethercat_motion_software_zero_set();
            real_time_position = zero_init_position;
            // USR_LOG_INFO("zero_init_position = %f real_time_position %f", zero_init_position, real_time_position);
            // USR_LOG_INFO("Set Zero Position");
            break;
        case 0x0003:
            if (s_servo_enable_request != 1 || servo_control_mode != manual_control) {
                break;
            }
            USR_LOG_INFO("Jog Move");
            if (p_ctrl->recv_buffer[14] == 1 && zero_init_position == 0.0) {
                ethercat_motion_command_set(ETHERCAT_MOTION_MODE_MOVE_REL, 5, 5, 10,CSP_LOCAL_JERK_MM_S3);
                USR_LOG_INFO("++++++++++++++++++++++++++++++");
                break;
            }
            if (p_ctrl->recv_buffer[14] == 2) {
                if (get_motor_position_mm() > 0.0) {
                    // 5 为实际出入位置
                    if (get_motor_position_mm() < 5) {
                        ethercat_motion_software_zero_return();
                        USR_LOG_INFO("Zero Position");
                        break;
                    }
                    // 5 为实际出入位置
                    ethercat_motion_command_set(ETHERCAT_MOTION_MODE_MOVE_REL, -4, 5, 10,CSP_LOCAL_JERK_MM_S3);
                    USR_LOG_INFO("-------------------------------");
                    break;
                }
                if (zero_init_position == 0.0) {
                    break;
                }
                // test 5 为实际出入位置
                if (real_time_position < 5.0) {
                    ethercat_motion_command_set(ETHERCAT_MOTION_MODE_MOVE_REL, -real_time_position, 5, 10,
                                                CSP_LOCAL_JERK_MM_S3);
                    real_time_position = 0;
                } else {
                    // 4 为实际出入位置
                    ethercat_motion_command_set(ETHERCAT_MOTION_MODE_MOVE_REL, -4, 5, 10,CSP_LOCAL_JERK_MM_S3);
                    real_time_position -= 4;
                }
            }
            break;
        case 0x0004:
            break;
        case 0x0005:
            if (s_servo_enable_request == 1 || get_motor_position_mm() != 0.0) {
                break;
            }
            float lead = (float) ((uint32_t) p_ctrl->recv_buffer[15] << 24 | (uint32_t) p_ctrl->recv_buffer[16] << 16 |
                                  (uint32_t) p_ctrl->recv_buffer[17] << 8 | (uint32_t) p_ctrl->recv_buffer[18] << 0) /
                         10000.0;
            float reduction_ratio = (float) ((uint32_t) p_ctrl->recv_buffer[20] << 24 | (uint32_t) p_ctrl->recv_buffer[
                                                 21] << 16 |
                                             (uint32_t) p_ctrl->recv_buffer[22] << 8 | (uint32_t) p_ctrl->recv_buffer[
                                                 23] << 0) /
                                    10000.0;
            // USR_LOG_INFO("lead=%f reduction_ratio=%f", lead, reduction_ratio);
            // ethercat_motion_motor_params_set(262144, 5, 1, 1, 3000);
            ethercat_motion_motor_params_set(262144, lead, 1, reduction_ratio, 3000);
            USR_LOG_INFO("Set Motor Parameters");
            break;
        case 0x0006:
            break;
        case 0x0007:
            break;
        case 0x000B:
            if (s_servo_enable_request != 1 || s_control.busy != 0 || s_control.done != 1) {
                break;
            }
            FAST_CLEAR_ARRAY(sequence_parameter,TCP_SERVER_RECV_BUFFER_SIZE);
            uint32_t farm_length = ((uint32_t)p_ctrl->recv_buffer[12] << 8 | (uint32_t)p_ctrl->recv_buffer[13]);
            USR_LOG_INFO("farm_length=%ld", farm_length);
            memcpy(sequence_parameter, &p_ctrl->recv_buffer[14], farm_length); // 获取序列控制数据
            servo_control_mode = Sequence_control;
            USR_LOG_INFO("Sequence Enable");
            break;
        case 0x000C:
            USR_LOG_INFO("Sequence Control");
            break;
        case 0x000D:
            USR_LOG_INFO("Extract signal");
            break;
        default:
            USR_LOG_INFO("Unknown Command");
            break;
    }

    return USR_SUCCESS;
}

static void sequence_task(void *pvParameter) {
    tcp_server_ctrl_t *p_ctrl = (tcp_server_ctrl_t *) pvParameter;

    for (;;) {
        vTaskDelay(100);
        if (servo_control_mode != Sequence_control) {
            continue;
        }

    }
}

static void tcp_send_task(void *pvParameter) {
    tcp_server_ctrl_t *p_ctrl = (tcp_server_ctrl_t *) pvParameter;
    static uint8_t i = 0;
    for (;;) {
        if (input1s != NULL) {
            if ((input1s->StatusWord & CIA402_SW_MASK) == CIA402_SW_OPERATION_ENABLED) {
                if (i++ > 2) {
                    sn595_data_set(10, 1);
                    i = 0;
                }
            } else {
                sn595_data_set(10, 0);
                i = 0;
            }
        }
        // sn595_data_update();
        // sn595_outdata();

        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (p_ctrl->num_of_socket == 0) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        send_frame((uint8_t) p_ctrl->max_fd);
    }
}

static void send_frame(uint8_t socket_fd) {
    static uint32_t position_send = 0;
    send_buf[0] = 0x73;
    send_buf[1] = 0x4C;
    send_buf[2] = 0x47;
    send_buf[3] = 0x57;
    send_buf[4] = 0x56;
    send_buf[5] = 0x01;
    send_buf[6] = 0x44;
    send_buf[7] = 0x77;
    send_buf[8] = 0x07;
    send_buf[9] = 0xE9; // 年
    send_buf[10] = 0x09; // 月
    send_buf[11] = 0x03; // 日
    send_buf[12] = 0x0A; // 时
    send_buf[13] = 0x22; // 分
    send_buf[14] = 0x22; // 秒
    send_buf[15] = 0x1F;
    send_buf[16] = 0x40; // 毫秒
    send_buf[17] = 0x02;
    send_buf[18] = 0xBC; // 微秒
    if (zero_init_position != 0.0) {
        position_send = (uint32_t) (real_time_position * 10000);
    } else {
        position_send = (uint32_t) (get_motor_position_mm() * 10000);
    }

    // position_send /= 10000;
    // if (position_send > 100) {
    //     position_send = 1;
    // } else {
    //     position_send++;
    // }
    // position_send *= 10000;
    send_buf[19] = 0x00;
    send_buf[20] = (position_send >> 24) & 0xFF;
    send_buf[21] = (position_send >> 16) & 0xFF;
    send_buf[22] = (position_send >> 8) & 0xFF;
    send_buf[23] = position_send & 0xFF;
    send_buf[25] = 0x00;;
    send_buf[26] = 0x98;
    send_buf[27] = 0x96;
    send_buf[28] = 0x80;
    send_buf[29] =
            (uint8_t) (((sn595_data[14] & 0x01) << 0) | ((sn595_data[12] & 0x01) << 1) | ((sn595_data[10] & 0x01) << 2)
                       | (
                           (sn595_data[8] & 0x01) << 3)
                       | ((sn595_data[22] & 0x01) << 4) | ((sn595_data[20] & 0x01) << 5) | (
                           (sn595_data[18] & 0x01) << 6) | (
                           (sn595_data[16] & 0x01) << 7));
    send_buf[30] =
            (uint8_t) (((sn595_data[6] & 0x01) << 0) | ((sn595_data[4] & 0x01) << 1) | ((sn595_data[2] & 0x01) << 2) |
                       (
                           (sn595_data[15] & 0x01) << 3)
                       | ((sn595_data[13] & 0x01) << 4) | ((sn595_data[11] & 0x01) << 5) | (
                           (sn595_data[9] & 0x01) << 6) | (
                           (sn595_data[23] & 0x01) << 7));
    send_buf[31] =
            (uint8_t) (((sn595_data[21] & 0x01) << 0) | ((sn595_data[19] & 0x01) << 1) | ((sn595_data[17] & 0x01) << 2)
                       |
                       (
                           (sn595_data[7] & 0x01) << 3)
                       | ((sn595_data[5] & 0x01) << 4) | ((sn595_data[3] & 0x01) << 5) | (
                           (sn595_data[0] & 0x01) << 6) | (
                           (sn595_data[1] & 0x01) << 7));
    send_buf[74] = 0x00;
    send_buf[75] = 0x0c;
    send_buf[76] = 0x01;
    send_buf[77] = 0x12;
    send_buf[78] = 0x01;
    send_buf[79] = 0x01;
    send_buf[80] = 0x01;
    send_buf[81] = 0x49;
    send_buf[82] = 0x6e;
    send_buf[83] = 0x6f;
    send_buf[84] = 0x53;
    send_buf[85] = 0x56;
    send_buf[86] = 0x36;
    send_buf[87] = 0x33;
    send_buf[88] = 0x30;
    send_buf[89] = 0x4e;
    send_buf[96] = 0x44;
    send_buf[97] = 0x77;
    const uint16_t crc = (uint16_t) crc_calc(send_buf, 98);
    send_buf[98] = (crc >> 8) & 0xFF;
    send_buf[99] = crc & 0xFF;
    lwip_send(socket_fd, send_buf, 100, 0);
}


/***********************************************************************************************************************
* Function Name: tcp_server_add_listener_socket
* Description  : Add listener socket
* Arguments    : Server Feature Management Data
* Return Value : usr_err_t
 **********************************************************************************************************************/
static usr_err_t tcp_server_add_listener_socket(tcp_server_ctrl_t *p_ctrl) {
    /** Status */
    usr_err_t usr_err;

    /** For getting IP information */
    lwip_port_netif_cfg_t netif_cfg;

    /** Listener socket */
    int32_t listener_socket_fd;

    /** Get IP address */
    usr_err = p_ctrl->p_lwip_port_instance->p_api->netifConfigGet(p_ctrl->p_lwip_port_instance->p_ctrl, &netif_cfg);
    USR_ERROR_RETURN(USR_SUCCESS == usr_err, USR_ERR_ABORTED);

    /** Check if the IP address is enabled. */
    listener_socket_fd = tcp_server_get_socket_by_ip_address(netif_cfg.ip_address, p_ctrl->max_fd);
    USR_ERROR_RETURN(TCP_SERVER_INVALID_SOCKET == listener_socket_fd, USR_ERR_ABORTED);

    /** Create new listener socket */
    listener_socket_fd = tcp_server_create_new_listner_socket(netif_cfg.ip_address, TCP_SERVER_PORT);
    USR_ERROR_RETURN(TCP_SERVER_INVALID_SOCKET != listener_socket_fd, USR_ERR_ABORTED);

    /** Update listener socket informations */
    FD_SET(listener_socket_fd, &p_ctrl->fdset_listners);
    FD_SET(listener_socket_fd, &p_ctrl->fdset_all);
    if (listener_socket_fd > p_ctrl->max_fd) { p_ctrl->max_fd = listener_socket_fd; }
    p_ctrl->num_of_socket++;

    return USR_SUCCESS;
}

/***********************************************************************************************************************
* Function Name: tcp_server_remove_listener_socket
* Description  : Remove listener socket
* Arguments    : Server Feature Management Data
* Return Value : usr_err_t
 **********************************************************************************************************************/
static usr_err_t tcp_server_remove_listener_socket(tcp_server_ctrl_t *p_ctrl) {
    /** Status */
    usr_err_t usr_err;
    /** Status */
    int32_t soc_err;

    /** For getting IP information */
    lwip_port_netif_cfg_t netif_cfg;

    /** Listener socket */
    int32_t listener_socket_fd;

    /** Get IP address */
    usr_err = p_ctrl->p_lwip_port_instance->p_api->netifConfigGet(p_ctrl->p_lwip_port_instance->p_ctrl, &netif_cfg);
    USR_ERROR_RETURN(USR_SUCCESS == usr_err, USR_ERR_ABORTED);

    /** Get the target socket by referencing the IP address */
    listener_socket_fd = tcp_server_get_socket_by_ip_address(netif_cfg.ip_address, p_ctrl->max_fd);
    USR_ERROR_RETURN(TCP_SERVER_INVALID_SOCKET != listener_socket_fd, USR_ERR_ABORTED);

    /** Try closing the socket. */
    soc_err = lwip_close(listener_socket_fd);
    USR_ERROR_RETURN(TCP_SERVER_INVALID_SOCKET != soc_err, USR_ERR_ABORTED);

    /** Update listener socket informations */
    FD_CLR(listener_socket_fd, &p_ctrl->fdset_listners);
    FD_CLR(listener_socket_fd, &p_ctrl->fdset_all);
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
static usr_err_t tcp_server_handle_listner_socket(tcp_server_ctrl_t *p_ctrl, int32_t listner_socket_fd) {
    /** new client socket information  */
    int32_t client_socket_fd;
    /** client addr */
    struct sockaddr_in client_addr;
    /** addr len */
    socklen_t client_addr_len;

    /** Accept */
    client_socket_fd = lwip_accept(listner_socket_fd, (struct sockaddr *) &client_addr, &client_addr_len);
    USR_ERROR_RETURN(-1 != client_socket_fd, USR_ERR_ABORTED);

    /** Update fd sets */
    FD_SET(client_socket_fd, &p_ctrl->fdset_clients);
    FD_SET(client_socket_fd, &p_ctrl->fdset_all);
    if (client_socket_fd > p_ctrl->max_fd) { p_ctrl->max_fd = client_socket_fd; }
    p_ctrl->num_of_socket++;

    /* 每个TCP连接独立保存拆包和粘包解析状态。 */

    /** Return success code. */
    return USR_SUCCESS;
}

/***********************************************************************************************************************
* Function Name: tcp_server_get_socket_by_ip_address
* Description  : Check if the IP address is already used by existing sockets
* Arguments    : Client IP address, Last connection handle
* Return Value : connection handle
 **********************************************************************************************************************/
static int32_t tcp_server_get_socket_by_ip_address(uint32_t ip_address, int max_fd) {
    /** lwip Function parameters */
    int socket_err = TCP_SERVER_INVALID_SOCKET;
    /** lwip Function parameters */
    int socket_fd = 0;
    /** socket address */
    struct sockaddr_in socket_addr;
    /**  socket length */
    socklen_t socket_addr_len;

    /** Check if the IP address is already used in existing sockets */
    for (socket_fd = 0; socket_fd <= max_fd; socket_fd++) {
        /** CODE CHECKER, this is OK as a comment aligns with the cast*/
        socket_err = lwip_getsockname(socket_fd, (struct sockaddr *) &socket_addr, &socket_addr_len);
        if (TCP_SERVER_INVALID_SOCKET == socket_err) {
            /** Is the socket is already closed? */
            continue;
        }

        if (socket_addr.sin_addr.s_addr == ip_address) {
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
static int32_t tcp_server_create_new_listner_socket(uint32_t ip_address, uint16_t port) {
    /** lwip Function parameters */
    int socket_fd;
    /** lwip Function parameters */
    int socket_err;
    /** socket address */
    struct sockaddr_in socket_addr;

    /** Create listener socket */
    socket_fd = lwip_socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (TCP_SERVER_INVALID_SOCKET == socket_fd) {
        return TCP_SERVER_INVALID_SOCKET;
    }

    /** Bind the socket */
    socket_addr.sin_port = htons(port);
    socket_addr.sin_family = AF_INET;
    socket_addr.sin_addr.s_addr = ip_address;
    /** CODE CHECKER, this is OK as a comment aligns with the cast*/
    socket_err = lwip_bind(socket_fd, (struct sockaddr *) &socket_addr, (socklen_t) sizeof(struct sockaddr));
    if (TCP_SERVER_INVALID_SOCKET == socket_err) {
        socket_err = lwip_close(socket_fd);
        return TCP_SERVER_INVALID_SOCKET;
    }

    /** Start listening */
    lwip_listen(socket_fd, 20);
    if (TCP_SERVER_INVALID_SOCKET == socket_err) {
        socket_err = lwip_close(socket_fd);
        return TCP_SERVER_INVALID_SOCKET;
    }

    return socket_fd;
}
