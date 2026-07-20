#include "ethercat_port_monitor.h"

#include "ethercat_app_common.h"
#include "ethercat_port_cfg.h"
#include "ethercat_master.h"
#include "FreeRTOS.h"
#include "main_thread.h"
#include "r_ether_phy_api.h"
#include "r_ethsw.h"
#include "task.h"
#include "um_ether_netif.h"
#include "um_ether_netif_api.h"

#define ETHERCAT_MONITOR_TASK_NAME       "ECAT port mon"
#define ETHERCAT_MONITOR_TASK_STACK_SIZE (1024U)
#define ETHERCAT_MONITOR_TASK_PRIORITY   (tskIDLE_PRIORITY + 2U)
#define ETHERCAT_LINK_POLL_MS            (100U)
/* Start SOEM scan only after port1 has stayed Link Up for more than 500 ms. */
#define ETHERCAT_LINK_STABLE_MS          (500U)
#define ETHERCAT_LINK_STABLE_TICKS       pdMS_TO_TICKS(ETHERCAT_LINK_STABLE_MS)
#define ETHERCAT_MONITOR_PORT_COUNT     (3U)

extern ether_netif_instance_t const *gp_ether_netif0;

static void ethercat_port_monitor_task(void *pvParameters);

static void ethercat_port_configure_ethsw_speed(void);

/* 成功返回true，失败返回false，失败后下一周期继续尝试。 */
static uint8_t lwip_port2_configure_ethsw_speed(void);

static ethsw_link_speed_t ethercat_port_phy_speed_to_ethsw(uint32_t phy_speed);

static void ethercat_port_log_link_changes(uint32_t link_status);

uint8_t port2_speed_configured = false;

usr_err_t ethercat_port_monitor_start(void) {
    usr_err_t usr_err;
    ethercat_app_notify_t *p_notify;

    usr_err = ethercat_app_common_open();
    if (USR_SUCCESS != usr_err) {
        USR_LOG_ERROR("EtherCAT app common open failed: %d", usr_err);
        return usr_err;
    }

    p_notify = ethercat_app_notify_get();
    if (NULL != p_notify->port_monitor_task) {
        return USR_SUCCESS;
    }

    usr_err = gp_ether_netif0->p_api->open(gp_ether_netif0->p_ctrl, gp_ether_netif0->p_cfg);
    if ((USR_SUCCESS != usr_err) && (USR_ERR_ALREADY_OPEN != usr_err) && (USR_ERR_ALREADY_RUNNING != usr_err)) {
        USR_LOG_ERROR("EtherCAT netif open failed: %d", usr_err);
        return usr_err;
    }

    usr_err = gp_ether_netif0->p_api->start(gp_ether_netif0->p_ctrl);
    if (USR_SUCCESS != usr_err) {
        USR_LOG_ERROR("EtherCAT netif start failed: %d", usr_err);
        return usr_err;
    }

    if (pdPASS != xTaskCreate(ethercat_port_monitor_task,
                              ETHERCAT_MONITOR_TASK_NAME,
                              ETHERCAT_MONITOR_TASK_STACK_SIZE / sizeof(StackType_t),
                              NULL,
                              ETHERCAT_MONITOR_TASK_PRIORITY,
                              &p_notify->port_monitor_task)) {
        USR_LOG_ERROR("EtherCAT port monitor task create failed.");
        return USR_ERR_NOT_INITIALIZED;
    }

    return USR_SUCCESS;
}

/*
 * 监控各物理网口状态：
 *
 * port1：EtherCAT伺服网口；
 * port2：lwIP上位机网口。
 *
 * 底层Ethernet Monitor只报告“是否存在任意Link Up端口”，
 * 无法区分port1和port2的独立变化。
 * 因此这里单独检测port2状态，在port2变化时强制通知lwIP。
 */
static void ethercat_port_monitor_task(void *pvParameters) {
    TickType_t port1_link_up_start_tick = 0U;

    uint8_t port1_stable_reported = false;

    /* port2速度配置状态。 */
    uint8_t port2_speed_configured = false;

    /* 记录port2上一次链路状态。 */
    uint8_t port2_link_initialized = false;
    uint8_t port2_link_previous = false;

    FSP_PARAMETER_NOT_USED(pvParameters);

    for (;;) {
        uint32_t link_status = 0U;
        uint8_t port1_link_up;
        uint8_t port2_link_up;
        usr_err_t usr_err;

        /*
         * 获取port0、port1、port2当前链路位图。
         * notify_callback=false，避免每100ms都产生回调。
         */
        usr_err = gp_ether_netif0->p_api->linkStatusGet(
            gp_ether_netif0->p_ctrl,
            &link_status,
            false);

        if (USR_SUCCESS == usr_err) {
            ethercat_port_log_link_changes(link_status);
        }

        /*
         * 分别取得port1和port2状态。
         */
        port1_link_up =
        ((USR_SUCCESS == usr_err) &&
         (0U !=
          (link_status & ETHERCAT_MASTER_PORT_MASK)));

        port2_link_up =
        ((USR_SUCCESS == usr_err) &&
         (0U !=
          (link_status &
           ETHER_NETIF_CFG_PORT_BIT(2U))));

        /*
         * port2连接后，根据PHY2协商结果配置ETHSW port2。
         *
         * 只有配置成功才置位。
         * 如果PHY协商结果暂时没有准备好，下一次100ms循环继续尝试。
         */
        if (port2_link_up) {
            if (!port2_speed_configured) {
                port2_speed_configured =
                        lwip_port2_configure_ethsw_speed();
            }
        } else {
            /*
             * 拔掉网线后清除标志。
             * 下次重新连接时重新读取PHY协商速度。
             */
            port2_speed_configured = false;
        }

        /*
         * 单独检测port2状态变化。
         *
         * 例如：
         * port1已经Link Up以后再插入port2，底层总体状态仍是Link Up，
         * 默认Ethernet Monitor不会再次产生LINK_UP。
         *
         * 这里检测port2自身变化，强制产生一次回调。
         * lwIP回调会重新检查port2，并把netif设置成Up或Down。
         */
        if ((!port2_link_initialized) ||
            (port2_link_up != port2_link_previous)) {
            uint32_t notify_link_status = 0U;
            usr_err_t notify_err;

            notify_err =
                    gp_ether_netif0->p_api->linkStatusGet(
                        gp_ether_netif0->p_ctrl,
                        &notify_link_status,
                        true);

            if (USR_SUCCESS == notify_err) {
                USR_LOG_INFO(
                    "lwIP port2 link changed: up=%u status=0x%08lx",
                    port2_link_up,
                    (unsigned long)notify_link_status);
            } else {
                USR_LOG_WARN(
                    "lwIP port2 link notify failed: %d",
                    notify_err);
            }

            port2_link_previous = port2_link_up;
            port2_link_initialized = true;
        }

        /*
         * port1连续稳定500ms后启动EtherCAT主站扫描。
         */
        if (port1_link_up) {
            if (0U == port1_link_up_start_tick) {
                port1_link_up_start_tick =
                        xTaskGetTickCount();
            }

            if ((!port1_stable_reported) &&
                ((xTaskGetTickCount() -
                  port1_link_up_start_tick) >=
                 ETHERCAT_LINK_STABLE_TICKS)) {
                port1_stable_reported = true;

                ethercat_port_configure_ethsw_speed();

                USR_LOG_INFO(
                    "EtherCAT port%u link up > %ums, "
                    "start SOEM slave scan.",
                    ETHERCAT_MASTER_PORT_NUMBER,
                    ETHERCAT_LINK_STABLE_MS);

                (void) ethercat_master_scan_start();
            }
        } else {
            port1_link_up_start_tick = 0U;
            port1_stable_reported = false;
        }

        vTaskDelay(
            pdMS_TO_TICKS(ETHERCAT_LINK_POLL_MS));
    }
}

static void ethercat_port_configure_ethsw_speed(void) {
    uint32_t phy_speed = 0U;
    uint32_t local_pause = 0U;
    uint32_t partner_pause = 0U;
    fsp_err_t fsp_err;
    ethsw_link_speed_t ethsw_speed;

    /* SV630 接在 port1 时读取 g_ether_phy1 的协商结果。
     *
     * RZ/N2L ETHSW 的外部 PHY 口和内部 ESC 口速率需要一致，否则可能链路亮但帧转发失败。 */
    fsp_err = g_ether_phy1.p_api->linkPartnerAbilityGet(g_ether_phy1.p_ctrl,
                                                        &phy_speed,
                                                        &local_pause,
                                                        &partner_pause);
    if (FSP_SUCCESS != fsp_err) {
        USR_LOG_WARN("EtherCAT PHY%u speed read failed: 0x%lx.",
                     ETHERCAT_MASTER_PORT_NUMBER,
                     (uint32_t) fsp_err);
        return;
    }

    ethsw_speed = ethercat_port_phy_speed_to_ethsw(phy_speed);

    // (void) R_ETHSW_SpeedCfg(&g_ethsw0_ctrl, 0U, ethsw_speed);
    (void) R_ETHSW_SpeedCfg(&g_ethsw0_ctrl, 1U, ethsw_speed);
    // (void) R_ETHSW_SpeedCfg(&g_ethsw0_ctrl, 2U, ethsw_speed);

    USR_LOG_INFO("EtherCAT ETHSW speed configured from PHY%u: phy=%lu ethsw=%d.",
                 ETHERCAT_MASTER_PORT_NUMBER,
                 phy_speed,
                 ethsw_speed);
}

static ethsw_link_speed_t ethercat_port_phy_speed_to_ethsw(uint32_t phy_speed) {
    switch (phy_speed) {
        case ETHER_PHY_LINK_SPEED_10H:
            return ETHSW_LINK_SPEED_10H;

        case ETHER_PHY_LINK_SPEED_10F:
            return ETHSW_LINK_SPEED_10F;

        case ETHER_PHY_LINK_SPEED_100H:
            return ETHSW_LINK_SPEED_100H;

        case ETHER_PHY_LINK_SPEED_100F:
            return ETHSW_LINK_SPEED_100F;

        case ETHER_PHY_LINK_SPEED_1000H:
            return ETHSW_LINK_SPEED_1000H;

        case ETHER_PHY_LINK_SPEED_1000F:
            return ETHSW_LINK_SPEED_1000F;

        default:
            return ETHSW_LINK_SPEED_100F;
    }
}

static void ethercat_port_log_link_changes(uint32_t link_status) {
    static uint8_t s_initialized = false;
    static uint32_t s_last_link_status = 0U;

    uint32_t changed;

    if (!s_initialized) {
        s_last_link_status = link_status;
        s_initialized = true;

        for (uint32_t port = 0U; port < ETHERCAT_MONITOR_PORT_COUNT; port++) {
            uint32_t port_mask = ETHER_NETIF_CFG_PORT_BIT(port);

            if (0U != (link_status & port_mask)) {
                USR_LOG_INFO("Ethernet port%lu link up.", (unsigned long) port);
            } else {
                USR_LOG_WARN("Ethernet port%lu link down.", (unsigned long) port);
            }
        }

        return;
    }

    changed = link_status ^ s_last_link_status;
    if (0U == changed) {
        return;
    }

    for (uint32_t port = 0U; port < ETHERCAT_MONITOR_PORT_COUNT; port++) {
        uint32_t port_mask = ETHER_NETIF_CFG_PORT_BIT(port);

        if (0U == (changed & port_mask)) {
            continue;
        }

        if (0U != (link_status & port_mask)) {
            USR_LOG_INFO("Ethernet port%lu link up.", (unsigned long) port);
        } else {
            USR_LOG_WARN("Ethernet port%lu link down.", (unsigned long) port);
        }
    }

    s_last_link_status = link_status;
}

/*
 * 根据port2外部PHY的实际协商结果配置ETHSW port2。
 *
 * 返回值：
 * true：配置成功；
 * false：PHY协商结果尚未准备好或配置失败。
 */
static uint8_t lwip_port2_configure_ethsw_speed(void)
{
    uint32_t phy_speed = 0U;
    uint32_t local_pause = 0U;
    uint32_t partner_pause = 0U;

    ethsw_link_speed_t ethsw_speed;
    fsp_err_t fsp_err;

    /*
     * 读取PHY2与电脑网卡的协商速度。
     */
    fsp_err =
        g_ether_phy2.p_api->linkPartnerAbilityGet(
            g_ether_phy2.p_ctrl,
            &phy_speed,
            &local_pause,
            &partner_pause);

    if (FSP_SUCCESS != fsp_err)
    {
        USR_LOG_WARN(
            "lwIP PHY2 speed read failed: 0x%lx",
            (unsigned long)fsp_err);

        return false;
    }

    /*
     * 将PHY速度转换成ETHSW使用的速度类型。
     */
    ethsw_speed =
        ethercat_port_phy_speed_to_ethsw(phy_speed);

    /*
     * 只配置port2，不影响EtherCAT使用的port1。
     */
    fsp_err = R_ETHSW_SpeedCfg(
        &g_ethsw0_ctrl,
        2U,
        ethsw_speed);

    if (FSP_SUCCESS != fsp_err)
    {
        USR_LOG_WARN(
            "ETHSW port2 speed configure failed: 0x%lx",
            (unsigned long)fsp_err);

        return false;
    }

    USR_LOG_INFO(
        "lwIP port2 speed configured: "
        "phy=%lu ethsw=%d",
        (unsigned long)phy_speed,
        (int)ethsw_speed);

    return true;
}
