/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_lwip_port_ether_netif.c
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
#include "um_lwip_port_api.h"
#include "um_lwip_port_cfg.h"
#include "um_lwip_port.h"
#include "um_lwip_port_internal.h"
#include "um_ether_netif_api.h"
#include "um_ether_netif.h"


#include "FreeRTOS.h"
/********************************USER**********************************************/
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "lwip/err.h"

#include <string.h>

#define LWIP_ETHERNET_PORT_NUMBER  (2U)
#define LWIP_ETHERNET_PORT_MASK    ETHER_NETIF_CFG_PORT_BIT(LWIP_ETHERNET_PORT_NUMBER)

#define ETHERNET_HEADER_SIZE       (14U)
#define ETHER_TYPE_IPV4            (0x0800U)
#define ETHER_TYPE_ARP             (0x0806U)

/* 在 um_lwip_port_netif.c 中定义。 */
extern struct netif *gp_lwip_netif;

static void lwip_ether_netif_callback(ether_netif_callback_args_t *p_args);

static uint8_t const *lwip_frame_data_get(ether_netif_frame_t const *p_frame);

/********************************USER**********************************************/

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
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
* Function Name: um_lwip_port_ether_netif_open
* Description  : Initialize the controller.
* Arguments    : [in] p_ctrl                    Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*                USR_ERR_NOT_INITIALIZED        Initialization has been failed.
*                USR_ERR_ALREADY_OPEN            Specified module has been already opened.
*********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_open(lwip_port_ether_netif_ctrl_t *const p_ctrl,
                                        lwip_port_launcher_ctrl_t *p_launcher_ctrl,
                                        lwip_port_receiver_ctrl_t *p_reader_ctrl,
                                        ether_netif_instance_t const *const p_ether_netif_instance) {
    /** Error codes */
    usr_err_t usr_err;

    /** Set related module controls. */
    p_ctrl->p_ether_netif_instance = (ether_netif_instance_t *) p_ether_netif_instance;
    p_ctrl->p_launcher_ctrl = p_launcher_ctrl;
    p_ctrl->p_receiver_ctrl = p_reader_ctrl;

    /** Try opening Ethernet network interface module */
    usr_err = p_ctrl->p_ether_netif_instance->p_api->open(
        p_ctrl->p_ether_netif_instance->p_ctrl, p_ctrl->p_ether_netif_instance->p_cfg);
    USR_ERROR_RETURN(USR_SUCCESS == usr_err || USR_ERR_ALREADY_OPEN == usr_err ||
                     USR_ERR_ALREADY_RUNNING == usr_err, USR_ERR_NOT_INITIALIZED);

    // /** RX callback is intentionally not registered in the current EtherCAT port monitor stage. */
    // (void) p_launcher_ctrl;
    // (void) p_reader_ctrl;
    /********************************USER**********************************************/
    /*
    * 注册 lwIP 接收回调。
     *
    * LWIP 模式回调会最后执行，并直接收到 Reader 任务持有的原始帧。
    * 本回调只复制数据到 pbuf，不释放 p_frame_packet。
    */
    memset(&p_ctrl->callback_memory, 0, sizeof(p_ctrl->callback_memory));
    memset(&p_ctrl->callback_node, 0, sizeof(p_ctrl->callback_node));

    p_ctrl->callback_node.p_func = lwip_ether_netif_callback;
    p_ctrl->callback_node.p_memory = &p_ctrl->callback_memory;
    p_ctrl->callback_node.p_context = p_ctrl;
    p_ctrl->callback_node.callback_buffer_mode =
            ETHER_NETIF_CFG_FB_MODE_LWIP;

    usr_err = p_ctrl->p_ether_netif_instance->p_api->callbackAdd(
        p_ctrl->p_ether_netif_instance->p_ctrl,
        &p_ctrl->callback_node);

    USR_ERROR_RETURN(USR_SUCCESS == usr_err,
                     USR_ERR_NOT_INITIALIZED);
    /********************************USER**********************************************/
    /** Return success code */
    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_start
* Description  : Start the network interface submodule.
* Arguments    : [in] p_ctrl                    Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_start(lwip_port_ether_netif_ctrl_t *const p_ctrl) {
    /** link status of all ports */
    uint32_t link_status;

    /** Start Ethernet network interface module. */
    (void) p_ctrl->p_ether_netif_instance->p_api->start(p_ctrl->p_ether_netif_instance->p_ctrl);

    /** Get link state. */
    (void) p_ctrl->p_ether_netif_instance->p_api->linkStatusGet(
        p_ctrl->p_ether_netif_instance->p_ctrl, &link_status, true);

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_stop
* Description  : Stop the network interface submodule.
* Arguments    : [in] p_ctrl                    Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_stop(lwip_port_ether_netif_ctrl_t *const p_ctrl) {
    /** Error codes */
    usr_err_t usr_err;

    /** Start Ethernet network interface module. */
    usr_err = p_ctrl->p_ether_netif_instance->p_api->stop(p_ctrl->p_ether_netif_instance->p_ctrl);
    USR_ERROR_RETURN(USR_SUCCESS == usr_err, USR_SUCCESS /** Already stopped */);

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_close
* Description  : Stop the network interface submodule.
* Arguments    : [in] p_ctrl                    Pointer to the controller
* Return Value : USR_SUCCESS                    Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_close(lwip_port_ether_netif_ctrl_t *const p_ctrl) {
    /** Error codes */
    usr_err_t usr_err;

    /** Start Ethernet network interface module. */
    usr_err = p_ctrl->p_ether_netif_instance->p_api->close(p_ctrl->p_ether_netif_instance->p_ctrl);
    USR_ERROR_RETURN(USR_SUCCESS == usr_err, USR_SUCCESS /** Already closed */);

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_send
* Description  : Request the Ethernet Link module to send Ethernet frame.
* Arguments    : [in] p_ctrl                    Pointer to the controller
*                [in] p_packet_buffer    Pointer to packet buffer.
* Return Value : USR_SUCCESS                    Process has been done successfully.
*                USR_ERR_ABORTED        Process has been aborted.
*********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_send(lwip_port_ether_netif_ctrl_t *const p_ctrl,
                                        ether_netif_frame_t *const p_packet_buffer) {
    usr_err_t usr_err;

    // /** lwIP uses port2 only; EtherCAT master uses port1. */
    // p_packet_buffer->port = ETHER_NETIF_CFG_PORT_BIT(0);

    /* 所有lwIP发送报文都从port2发出。 */
    p_packet_buffer->port = LWIP_ETHERNET_PORT_MASK;

    /** Send the frame */
    usr_err = p_ctrl->p_ether_netif_instance->p_api->send(p_ctrl->p_ether_netif_instance->p_ctrl, p_packet_buffer);
    USR_ERROR_RETURN(USR_SUCCESS == usr_err, USR_ERR_ABORTED);

    return USR_SUCCESS;
}

/**********************************************************************************************************************
* Function Name: um_lwip_port_ether_netif_get_local_mac_address
* Description  : Get the device host local MAC address
* Arguments    : [in] p_ctrl                    Pointer to the controller
*                [out] p_mac_address    Pointer to MAC address array pointer to be set the host MAC address
* Return Value : USR_SUCCESS                    Process has been done successfully.
*********************************************************************************************************************/
usr_err_t um_lwip_port_ether_netif_get_local_mac_address(
    lwip_port_ether_netif_ctrl_t *const p_ctrl, uint8_t *const p_mac_address) {
    /** Get Mac address configuration */
    memcpy(p_mac_address, p_ctrl->p_ether_netif_instance->p_cfg->p_ether_instance->p_cfg->p_mac_address,
           sizeof(uint8_t) * ETHER_NETIF_CFG_MAC_ADDRESS_BYTES);

    return USR_SUCCESS;
}

/***********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

/*
 * Ethernet 共用接收回调。
 *
 * port0：交给 lwIP；
 * port1：由 SOEM EtherCAT 回调处理。
 */
static void lwip_ether_netif_callback(
    ether_netif_callback_args_t *p_args)
{
    lwip_port_ether_netif_ctrl_t *p_ctrl;
    ether_netif_frame_t *p_frame;
    uint8_t const *p_data;
    uint16_t ether_type;
    uint32_t link_status = 0U;
    struct pbuf *p_packet;
    err_t lwip_err;

    if (NULL == p_args)
    {
        return;
    }

    p_ctrl = (lwip_port_ether_netif_ctrl_t *)p_args->p_context;
    if (NULL == p_ctrl)
    {
        return;
    }

    /*
     * 底层 LINK_UP 表示任意端口连接。
     * lwIP 只使用 port0/por2，因此重新检查 port0/por2 的状态。
     */
    if ((ETHER_NETIF_CALLBACK_EVENT_LINK_UP == p_args->event) ||
        (ETHER_NETIF_CALLBACK_EVENT_LINK_DOWN == p_args->event))
    {
        if (USR_SUCCESS ==
            p_ctrl->p_ether_netif_instance->p_api->linkStatusGet(
                p_ctrl->p_ether_netif_instance->p_ctrl,
                &link_status,
                false))
        {
            (void)um_lwip_port_task_launcher_request(
                (lwip_port_launcher_ctrl_t *)p_ctrl->p_launcher_ctrl,
                (0U != (link_status & LWIP_ETHERNET_PORT_MASK)) ?
                    LWIP_PORT_LAUNCHER_EVENT_LINK_UP :
                    LWIP_PORT_LAUNCHER_EVENT_LINK_DOWN,
                NULL);
        }

        return;
    }

    if ((ETHER_NETIF_CALLBACK_EVENT_RECEIVE_ETHER_FRAME !=
         p_args->event) ||
        (NULL == p_args->p_frame_packet))
    {
        return;
    }

    p_frame = p_args->p_frame_packet;

    /* 明确过滤：lwIP 只处理从 port0/por2 收到的帧。 */
    if ((p_frame->port != ETHER_NETIF_CFG_PORT_RECV_PORT_ANY) &&
        (0U == (p_frame->port & LWIP_ETHERNET_PORT_MASK)))
    {
        return;
    }

    if (p_frame->length < ETHERNET_HEADER_SIZE)
    {
        return;
    }

    p_data = lwip_frame_data_get(p_frame);
    if (NULL == p_data)
    {
        return;
    }

    ether_type = (uint16_t)(((uint16_t)p_data[12] << 8) |
                            (uint16_t)p_data[13]);

    /* 当前只接收 IPv4 和 ARP，EtherCAT 0x88A4 不交给 lwIP。 */
    if ((ETHER_TYPE_IPV4 != ether_type) &&
        (ETHER_TYPE_ARP != ether_type))
    {
        return;
    }

    if ((NULL == gp_lwip_netif) ||
        (NULL == gp_lwip_netif->input))
    {
        return;
    }

    p_packet = pbuf_alloc(PBUF_RAW,
                          (u16_t)p_frame->length,
                          PBUF_POOL);
    if (NULL == p_packet)
    {
        return;
    }

    lwip_err = pbuf_take(p_packet,
                         p_data,
                         (u16_t)p_frame->length);
    if (ERR_OK != lwip_err)
    {
        pbuf_free(p_packet);
        return;
    }

    /*
     * 成功后 pbuf 所有权交给 lwIP；
     * 失败时由本函数释放。
     */
    lwip_err = gp_lwip_netif->input(
        p_packet,
        gp_lwip_netif);

    if (ERR_OK != lwip_err)
    {
        pbuf_free(p_packet);
    }
}

static uint8_t const *lwip_frame_data_get(
    ether_netif_frame_t const *p_frame)
{
    if (NULL == p_frame)
    {
        return NULL;
    }

    if (0U !=
        (p_frame->buffer_mode & ETHER_NETIF_CFG_FB_MODE_POINTER))
    {
        return p_frame->p_buffer;
    }

    return p_frame->buffer;
}