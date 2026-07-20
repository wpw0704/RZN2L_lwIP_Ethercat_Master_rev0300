#include "nicdrv.h"

#include "ethercat_port_cfg.h"
#include "osal.h"
#include "task.h"
#include "um_ether_netif_api.h"

#include <stdint.h>
#include <string.h>

#define ETHERCAT_ETHERTYPE_HIGH (0x88U)
#define ETHERCAT_ETHERTYPE_LOW  (0xA4U)
#define ETHERCAT_FRAME_HEADER   (14U)
#define ETHERCAT_RX_QUEUE_SIZE  (8U)

const uint16 priMAC[3] = {0x1102, 0x3322, 0x5544};
const uint16 secMAC[3] = {0x1104, 0x3322, 0x5544};

extern ether_netif_instance_t const *gp_ether_netif0;

typedef struct st_ethercat_rx_frame {
    uint32_t length;
    uint8_t buffer[1600];
} ethercat_rx_frame_t;

static ether_netif_callback_args_t s_rx_callback_args;
static ether_netif_callback_link_node_t s_rx_callback_node;
static ethercat_rx_frame_t s_rx_queue[ETHERCAT_RX_QUEUE_SIZE];
static OSAL_MUTEX_HANDLE s_rx_mutex;
static uint8_t s_rx_head;
static uint8_t s_rx_tail;
static uint8_t s_rx_count;
static uint8_t s_rx_mutex_ready;
static uint8_t s_port_mutexes_ready;
static uint8_t s_callback_added;

static void ethercat_netif_callback(ether_netif_callback_args_t *p_args);

static uint8_t *ethercat_frame_buffer(ether_netif_frame_t const *p_frame);

static int ethercat_frame_is_for_master(ether_netif_frame_t const *p_frame);

static void ethercat_rx_queue_push(uint8_t const *p_buffer, uint32_t length);

static int ethercat_rx_queue_pop(ethercat_rx_frame_t *p_frame);

/* 初始化 EtherCAT 发送缓冲区中的以太网帧头。 */
void ec_setupheader(void *p) {
    ecx_portt *port = (ecx_portt *) p;

    for (int i = 0; i < EC_MAXBUF; i++) {
        unsigned char *buf = (unsigned char *) &port->txbuf[i];

        buf[0] = 0xff;
        buf[1] = 0xff;
        buf[2] = 0xff;
        buf[3] = 0xff;
        buf[4] = 0xff;
        buf[5] = 0xff;

        buf[6] = (unsigned char) (priMAC[0] & 0xFF);
        buf[7] = (unsigned char) (priMAC[0] >> 8);
        buf[8] = (unsigned char) (priMAC[1] & 0xFF);
        buf[9] = (unsigned char) (priMAC[1] >> 8);
        buf[10] = (unsigned char) (priMAC[2] & 0xFF);
        buf[11] = (unsigned char) (priMAC[2] >> 8);

        buf[12] = ETHERCAT_ETHERTYPE_HIGH;
        buf[13] = ETHERCAT_ETHERTYPE_LOW;
    }
}

/* 初始化 SOEM 主站端口、互斥锁和 EtherCAT 接收回调。 */
int ecx_setupnic(ecx_portt *port, const char *ifname, int secondary) {
    usr_err_t usr_err;

    (void) ifname;

    if (secondary) {
        return 0;
    }

    if (!s_port_mutexes_ready) {
        osal_mutex_init(&port->getindex_mutex);
        osal_mutex_init(&port->tx_mutex);
        osal_mutex_init(&port->rx_mutex);
        s_port_mutexes_ready = 1U;
    }

    if (!s_rx_mutex_ready) {
        osal_mutex_init(&s_rx_mutex);
        s_rx_mutex_ready = 1U;
    }

    if (!s_callback_added) {
        memset(&s_rx_callback_args, 0, sizeof(s_rx_callback_args));
        memset(&s_rx_callback_node, 0, sizeof(s_rx_callback_node));
        s_rx_callback_node.p_func = ethercat_netif_callback;
        s_rx_callback_node.p_memory = &s_rx_callback_args;
        s_rx_callback_node.callback_buffer_mode = ETHER_NETIF_CFG_FB_MODE_POINTER;

        usr_err = gp_ether_netif0->p_api->callbackAdd(gp_ether_netif0->p_ctrl, &s_rx_callback_node);
        if (USR_SUCCESS != usr_err) {
            return 0;
        }

        s_callback_added = 1U;
    }

    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_count = 0U;

    port->sockhandle = 1;
    port->lastidx = 0;
    ec_setupheader(port);

    return 1;
}

/* 关闭主站网卡端口并标记 socket 句柄无效。 */
int ecx_closenic(ecx_portt *port) {
    port->sockhandle = 0;
    return 0;
}

/* 设置指定索引接收缓冲区的状态。 */
void ecx_setbufstat(ecx_portt *port, int idx, int bufstat) {
    port->rxbufstat[idx] = bufstat;
}

/* 分配一个空闲的 SOEM 缓冲区索引用于发送和接收匹配。 */
int ecx_getindex(ecx_portt *port) {
    int idx;
    int cnt;

    osal_mutex_lock(&port->getindex_mutex);

    idx = port->lastidx + 1;
    if (idx >= EC_MAXBUF) {
        idx = 1;
    }

    cnt = 0;
    while ((port->rxbufstat[idx] != EC_BUF_EMPTY) && (cnt < EC_MAXBUF)) {
        idx++;
        if (idx >= EC_MAXBUF) {
            idx = 1;
        }
        cnt++;
    }

    if (cnt < EC_MAXBUF) {
        port->rxbufstat[idx] = EC_BUF_ALLOC;
        port->lastidx = idx;
    } else {
        idx = 0;
    }

    osal_mutex_unlock(&port->getindex_mutex);

    return idx;
}

/* 通过底层网口发送指定索引的 EtherCAT 以太网帧。 */
int ecx_outframe(ecx_portt *port, int idx, int sock) {
    ether_netif_frame_t frame;
    uint32_t length;
    int ret = 0;

    (void) sock;

    osal_mutex_lock(&port->tx_mutex);

    length = (uint32_t) port->txbuflength[idx];
    if ((length > 0U) && (port->sockhandle != 0)) {
        if (length < 60U) {
            memset(&port->txbuf[idx][length], 0, 60U - length);
            length = 60U;
        }

        memset(&frame, 0, sizeof(frame));
        frame.length = length;
        frame.port = ETHERCAT_MASTER_PORT_MASK;
        memcpy(frame.buffer, &port->txbuf[idx], length);

        if (USR_SUCCESS == gp_ether_netif0->p_api->send(gp_ether_netif0->p_ctrl, &frame)) {
            ret = 1;
        }
    }

    port->rxbufstat[idx] = EC_BUF_TX;
    osal_mutex_unlock(&port->tx_mutex);

    return ret;
}

/* 发送冗余模式使用的 EtherCAT 帧，本工程复用普通发送路径。 */
int ecx_outframe_red(ecx_portt *port, int idx) {
    return ecx_outframe(port, idx, 0);
}

/* 从接收队列中等待并取出一帧 EtherCAT 数据。 */
static int eth_receive(ecx_portt *port, int timeout_us) {
    osal_timert timer;
    ethercat_rx_frame_t frame;

    osal_timer_start(&timer, timeout_us);

    do {
        if (ethercat_rx_queue_pop(&frame)) {
            memcpy(&port->tempinbuf, frame.buffer, frame.length);
            port->tempinbufs = (int) frame.length;
            return (int) frame.length;
        }

        taskYIELD();
    } while (!osal_timer_is_expired(&timer));

    return 0;
}

/* 等待指定缓冲区索引对应的 EtherCAT 响应帧并写入接收缓冲区。 */
int ecx_waitinframe(ecx_portt *port, int idx, int timeout) {
    int bytes_read;
    int rx_idx;

    osal_mutex_lock(&port->rx_mutex);

    if (port->rxbufstat[idx] == EC_BUF_RCVD) {
        osal_mutex_unlock(&port->rx_mutex);
        return port->rxbufstat[idx];
    }

    bytes_read = eth_receive(port, timeout);
    if (bytes_read > (int) ETHERCAT_FRAME_HEADER) {
        uint8_t *rxbuf_ptr = (uint8_t *) &port->tempinbuf;
        uint8_t *ecat_header = rxbuf_ptr + ETHERCAT_FRAME_HEADER;

        if ((rxbuf_ptr[12] == ETHERCAT_ETHERTYPE_HIGH) && (rxbuf_ptr[13] == ETHERCAT_ETHERTYPE_LOW)) {
            rx_idx = ecat_header[3];
            if ((rx_idx > 0) && (rx_idx < EC_MAXBUF) &&
                ((port->rxbufstat[rx_idx] == EC_BUF_TX) || (port->rxbufstat[rx_idx] == EC_BUF_ALLOC))) {
                int payload_len = bytes_read - (int) ETHERCAT_FRAME_HEADER;

                if ((payload_len > 0) && (payload_len <= (int) sizeof(ec_bufT))) {
                    memcpy(&port->rxbuf[rx_idx], ecat_header, (size_t) payload_len);
                    port->rxbufstat[rx_idx] = EC_BUF_RCVD;
                }
            }
        }
    }

    osal_mutex_unlock(&port->rx_mutex);

    return port->rxbufstat[idx];
}

/* 发送 EtherCAT 帧并等待响应确认，返回工作计数器或接收结果。 */
int ecx_srconfirm(ecx_portt *port, int idx, int timeout) {
    int rx = 0;
    osal_timert timer;

    ecx_outframe(port, idx, 0);
    osal_timer_start(&timer, timeout);

    do {
        if (port->rxbufstat[idx] == EC_BUF_RCVD) {
            int len = port->txbuflength[idx] - (int) ETHERCAT_FRAME_HEADER;

            if (len >= 2) {
                rx = port->rxbuf[idx][len - 2] | (port->rxbuf[idx][len - 1] << 8);
            } else {
                rx = 1;
            }

            break;
        }

        (void) ecx_waitinframe(port, idx, 5000);
    } while (!osal_timer_is_expired(&timer));

    return rx;
}

/* 底层以太网接收回调，筛选 EtherCAT 帧并放入主站接收队列。 */
static void ethercat_netif_callback(ether_netif_callback_args_t *p_args) {
    uint8_t const *p_buffer;

    if ((NULL == p_args) ||
        (ETHER_NETIF_CALLBACK_EVENT_RECEIVE_ETHER_FRAME != p_args->event) ||
        (NULL == p_args->p_frame_packet)) {
        return;
    }

    if (!ethercat_frame_is_for_master(p_args->p_frame_packet)) {
        /*
     * 该帧对象由 um_ether_netif_callback_request() 为本回调动态分配。
     * lwIP 启用后会收到大量 ARP/IP 帧；这里不释放会持续消耗堆内存。
     */
        USR_HEAP_RELEASE(p_args->p_frame_packet);
        return;
    }

    p_buffer = ethercat_frame_buffer(p_args->p_frame_packet);
    if (NULL == p_buffer) {
        USR_HEAP_RELEASE(p_args->p_frame_packet);
        return;
    }

    ethercat_rx_queue_push(p_buffer, p_args->p_frame_packet->length);
    USR_HEAP_RELEASE(p_args->p_frame_packet);
}

/* 根据底层帧缓冲模式获取实际帧数据指针。 */
static uint8_t *ethercat_frame_buffer(ether_netif_frame_t const *p_frame) {
    if (p_frame->buffer_mode & ETHER_NETIF_CFG_FB_MODE_POINTER) {
        return p_frame->p_buffer;
    }

    return (uint8_t *) p_frame->buffer;
}

/* 判断接收到的以太网帧是否为当前 EtherCAT 主站需要处理的帧。 */
static int ethercat_frame_is_for_master(ether_netif_frame_t const *p_frame) {
    uint8_t const *p_buffer = ethercat_frame_buffer(p_frame);

    if ((NULL == p_buffer) || (p_frame->length < ETHERCAT_FRAME_HEADER)) {
        return 0;
    }

    if ((p_frame->port != ETHER_NETIF_CFG_PORT_RECV_PORT_ANY) &&
        (0U == (p_frame->port & ETHERCAT_MASTER_PORT_MASK))) {
        return 0;
    }

    return ((p_buffer[12] == ETHERCAT_ETHERTYPE_HIGH) && (p_buffer[13] == ETHERCAT_ETHERTYPE_LOW));
}

/* 将接收到的 EtherCAT 帧复制并压入环形接收队列。 */
static void ethercat_rx_queue_push(uint8_t const *p_buffer, uint32_t length) {
    if ((NULL == p_buffer) || (length > sizeof(s_rx_queue[0].buffer))) {
        return;
    }

    osal_mutex_lock(&s_rx_mutex);

    if (s_rx_count >= ETHERCAT_RX_QUEUE_SIZE) {
        s_rx_tail = (uint8_t) ((s_rx_tail + 1U) % ETHERCAT_RX_QUEUE_SIZE);
        s_rx_count--;
    }

    s_rx_queue[s_rx_head].length = length;
    memcpy(s_rx_queue[s_rx_head].buffer, p_buffer, length);
    s_rx_head = (uint8_t) ((s_rx_head + 1U) % ETHERCAT_RX_QUEUE_SIZE);
    s_rx_count++;

    osal_mutex_unlock(&s_rx_mutex);
}

/* 从环形接收队列中弹出一帧 EtherCAT 数据。 */
static int ethercat_rx_queue_pop(ethercat_rx_frame_t *p_frame) {
    int ret = 0;

    osal_mutex_lock(&s_rx_mutex);

    if (s_rx_count > 0U) {
        memcpy(p_frame, &s_rx_queue[s_rx_tail], sizeof(*p_frame));
        s_rx_tail = (uint8_t) ((s_rx_tail + 1U) % ETHERCAT_RX_QUEUE_SIZE);
        s_rx_count--;
        ret = 1;
    }

    osal_mutex_unlock(&s_rx_mutex);

    return ret;
}

/* 使用全局 SOEM 端口初始化网卡。 */
int ec_setupnic(const char *ifname, int secondary) { return ecx_setupnic(&ecx_port, ifname, secondary); }
/* 使用全局 SOEM 端口关闭网卡。 */
int ec_closenic(void) { return ecx_closenic(&ecx_port); }
/* 使用全局 SOEM 端口设置接收缓冲区状态。 */
void ec_setbufstat(int idx, int bufstat) { ecx_setbufstat(&ecx_port, idx, bufstat); }
/* 使用全局 SOEM 端口分配缓冲区索引。 */
int ec_getindex(void) { return ecx_getindex(&ecx_port); }
/* 使用全局 SOEM 端口发送 EtherCAT 帧。 */
int ec_outframe(int idx, int sock) { return ecx_outframe(&ecx_port, idx, sock); }
/* 使用全局 SOEM 端口发送冗余 EtherCAT 帧。 */
int ec_outframe_red(int idx) { return ecx_outframe_red(&ecx_port, idx); }
/* 使用全局 SOEM 端口等待指定索引的响应帧。 */
int ec_waitinframe(int idx, int timeout) { return ecx_waitinframe(&ecx_port, idx, timeout); }
/* 使用全局 SOEM 端口发送帧并等待确认。 */
int ec_srconfirm(int idx, int timeout) { return ecx_srconfirm(&ecx_port, idx, timeout); }
