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

static ethsw_link_speed_t ethercat_port_phy_speed_to_ethsw(uint32_t phy_speed);

static void ethercat_port_log_link_changes(uint32_t link_status);

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

    /* Open the low-level Ethernet driver only. Do not start lwIP or old TX test code. */
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

static void ethercat_port_monitor_task(void *pvParameters) {
    TickType_t link_up_start_tick = 0U;
    uint8_t stable_reported = false;
    TickType_t link_down_start_tick = 0U;
    uint8_t down_reported = false;
    (void) pvParameters;

    for (;;) {
        uint32_t link_status = 0U;
        usr_err_t usr_err = gp_ether_netif0->p_api->linkStatusGet(gp_ether_netif0->p_ctrl, &link_status, false);

        if (USR_SUCCESS == usr_err) {
            ethercat_port_log_link_changes(link_status);
        }

        uint8_t port1_link_up = ((USR_SUCCESS == usr_err) &&
                                 (0U != (link_status & ETHERCAT_MASTER_PORT_MASK)));

        if (port1_link_up) {
            /* First Link Up sample starts the stable timer; any Link Down restarts the timer. */
            if (0U == link_up_start_tick) {
                link_up_start_tick = xTaskGetTickCount();
            }

            if ((!stable_reported) && ((xTaskGetTickCount() - link_up_start_tick) >= ETHERCAT_LINK_STABLE_TICKS)) {
                stable_reported = true;
                ethercat_port_configure_ethsw_speed();
                USR_LOG_INFO("EtherCAT port%u link up > %ums, start SOEM slave scan.",
                             ETHERCAT_MASTER_PORT_NUMBER,
                             ETHERCAT_LINK_STABLE_MS);
                (void) ethercat_master_scan_start();
            }
        } else {
            link_up_start_tick = 0U;
            stable_reported = false;
            if (0U == link_down_start_tick) {
                link_down_start_tick = xTaskGetTickCount();
            }
            if ((!down_reported) &&
                ((xTaskGetTickCount() - link_down_start_tick) >= ETHERCAT_LINK_STABLE_TICKS)) {
                down_reported = true;

                USR_LOG_WARN("EtherCAT port%u link down > %ums, stop master.",
                             ETHERCAT_MASTER_PORT_NUMBER,
                             ETHERCAT_LINK_STABLE_MS);

                /* 这里执行删除/停止函数 */
                /* ethercat_master_stop_request(); */
            }

            vTaskDelay(pdMS_TO_TICKS(ETHERCAT_LINK_POLL_MS));
        }
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

    /* port0/port1 是外部 PHY 口，port2 是 ESC 侧口。port3 是 GMAC 内部口，保持 FSP/GMAC 默认配置。 */
    (void) R_ETHSW_SpeedCfg(&g_ethsw0_ctrl, 0U, ethsw_speed);
    (void) R_ETHSW_SpeedCfg(&g_ethsw0_ctrl, 1U, ethsw_speed);
    (void) R_ETHSW_SpeedCfg(&g_ethsw0_ctrl, 2U, ethsw_speed);

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
