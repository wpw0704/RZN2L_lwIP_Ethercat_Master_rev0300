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
/* Start SOEM scan only after the configured master port has stayed Link Up for more than 500 ms. */
#define ETHERCAT_LINK_STABLE_MS          (500U)
#define ETHERCAT_LINK_STABLE_TICKS       pdMS_TO_TICKS(ETHERCAT_LINK_STABLE_MS)
#define ETHERCAT_MONITOR_PORT_COUNT     (3U)

#define ETHSW_MANAGEMENT_PORT_MASK    (1U << 3)

extern ether_netif_instance_t const *gp_ether_netif0;

static void ethercat_port_monitor_task(void *pvParameters);

static void ethercat_port_configure_ethsw_speed(void);

static fsp_err_t ethernet_port_isolation_configure(void);

/* 成功返回true，失败返回false，失败后下一周期继续尝试。 */
static uint8_t lwip_port_configure_ethsw_speed(uint32_t port, ether_phy_instance_t const *p_phy_instance);

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
    fsp_err_t fsp_err = ethernet_port_isolation_configure();

    if (FSP_SUCCESS != fsp_err) {
        USR_LOG_ERROR(
            "ETHSW isolation configure failed: 0x%lx",
            (unsigned long) fsp_err);

        return USR_ERR_NOT_INITIALIZED;
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
 * EtherCAT 与 lwIP 的物理端口由 ETHERCAT_LWIP_PORT_SELECTION 配置。
 *
 * 底层Ethernet Monitor只报告“是否存在任意Link Up端口”，
 * 无法区分各物理端口的独立变化。
 * 因此这里单独检测主站端口和 lwIP 端口状态。
 */
static void ethercat_port_monitor_task(void *pvParameters) {
    TickType_t master_link_up_start_tick = 0U;

    uint8_t master_stable_reported = false;

    uint8_t lwip_speed_configured = false;

    uint8_t lwip_link_initialized = false;
    uint8_t lwip_link_previous = false;

    uint8_t master_link_initialized = false;
    uint8_t master_link_previous = false;

    FSP_PARAMETER_NOT_USED(pvParameters);

    for (;;) {
        uint32_t link_status = 0U;
        uint8_t master_link_up;
        uint8_t lwip_link_up;
        usr_err_t usr_err;

        /*
         * 获取各物理端口当前链路位图。
         * notify_callback=false，避免每100ms都产生回调。
         */
        usr_err = gp_ether_netif0->p_api->linkStatusGet(
            gp_ether_netif0->p_ctrl,
            &link_status,
            false);

        if (USR_SUCCESS == usr_err) {
            ethercat_port_log_link_changes(link_status);
        }

        master_link_up =
        ((USR_SUCCESS == usr_err) &&
         (0U != (link_status & ETHERCAT_MASTER_PORT_MASK)));
        if ((!master_link_initialized) ||
            (master_link_up != master_link_previous)) {
            if (master_link_up) {
                ethercat_app_master_run_set_state(
                    ETHERCAT_MASTER_RUN_STATE_LINK_UP);
            } else {
                ethercat_app_master_run_set_state(
                    ETHERCAT_MASTER_RUN_STATE_LINK_DOWN);

                ethercat_master_link_down_notify();
            }

            master_link_previous = master_link_up;
            master_link_initialized = true;
        }

        lwip_link_up =
        ((USR_SUCCESS == usr_err) &&
         (0U != (link_status & LWIP_ETHERNET_PORT_MASK)));

        if (lwip_link_up) {
            if (!lwip_speed_configured) {
                lwip_speed_configured =
                        lwip_port_configure_ethsw_speed(
                            LWIP_ETHERNET_PORT_NUMBER,
                            &LWIP_ETHERNET_PHY_INSTANCE);
            }
        } else {
            lwip_speed_configured = false;
        }

        /*
         * The lower Ethernet monitor reports only whether any physical port is up.
         * Notify lwIP explicitly when its configured port state changes.
         */
        if ((!lwip_link_initialized) ||
            (lwip_link_up != lwip_link_previous)) {
            uint32_t notify_link_status = 0U;
            usr_err_t notify_err;

            notify_err =
                    gp_ether_netif0->p_api->linkStatusGet(
                        gp_ether_netif0->p_ctrl,
                        &notify_link_status,
                        true);

            if (USR_SUCCESS == notify_err) {
                USR_LOG_INFO(
                    "lwIP link changed: up=%u "
                    "port%u=%u status=0x%08lx",
                    lwip_link_up,
                    LWIP_ETHERNET_PORT_NUMBER,
                    lwip_link_up,
                    (unsigned long)notify_link_status);
            } else {
                USR_LOG_WARN(
                    "lwIP link notify failed: %d",
                    notify_err);
            }

            lwip_link_previous = lwip_link_up;
            lwip_link_initialized = true;
        }

        /*
         * 主站端口连续稳定500ms后启动EtherCAT主站扫描。
         */
        if (master_link_up) {
            if (0U == master_link_up_start_tick) {
                master_link_up_start_tick =
                        xTaskGetTickCount();
            }

            if ((!master_stable_reported) &&
                ((xTaskGetTickCount() -
                  master_link_up_start_tick) >=
                 ETHERCAT_LINK_STABLE_TICKS)) {
                /*
                 * SOEM扫描前先根据PHY协商结果配置ETHSW端口速率。
                 */
                ethercat_port_configure_ethsw_speed();

                /*
                 * 只有任务实际创建成功后才置位。
                 * 如果旧主站任务正在退出，保持false，
                 * 下一次100ms监控周期会继续尝试。
                 */
                usr_err = ethercat_master_scan_start();

                if (USR_SUCCESS == usr_err) {
                    master_stable_reported = true;

                    USR_LOG_INFO(
                        "EtherCAT port%u link up > %ums, "
                        "SOEM scan started.",
                        ETHERCAT_MASTER_PORT_NUMBER,
                        ETHERCAT_LINK_STABLE_MS);
                } else if (USR_ERR_ALREADY_RUNNING != usr_err) {
                    /*
                     * ALREADY_RUNNING表示旧主站任务尚未完成退出，
                     * 属于重连过程中的正常暂态，不重复打印警告。
                     */
                    USR_LOG_WARN(
                        "SOEM scan start failed: %d",
                        usr_err);
                }
            }
        } else {
            master_link_up_start_tick = 0U;
            master_stable_reported = false;
        }

        vTaskDelay(
            pdMS_TO_TICKS(ETHERCAT_LINK_POLL_MS));
    }
}

static void ethercat_port_configure_ethsw_speed(void) {
    uint32_t phy_speed = 0U;
    uint32_t local_pause = 0U;
    uint32_t partner_pause = 0U;
    ether_phy_instance_t const *p_phy_instance;
    fsp_err_t fsp_err;
    ethsw_link_speed_t ethsw_speed;

    /* 读取 EtherCAT 主站端口对应 PHY 的协商结果。
     *
     * RZ/N2L ETHSW 的外部 PHY 口和内部 ESC 口速率需要一致，否则可能链路亮但帧转发失败。 */
    p_phy_instance = &ETHERCAT_MASTER_PHY_INSTANCE;

    fsp_err = p_phy_instance->p_api->linkPartnerAbilityGet(
        p_phy_instance->p_ctrl,
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

    (void) R_ETHSW_SpeedCfg(&g_ethsw0_ctrl,
                            ETHERCAT_MASTER_PORT_NUMBER,
                            ethsw_speed);

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
 * Synchronize an lwIP Ethernet switch port with its negotiated PHY speed.
 *
 * Parameters:
 *   port           ETHSW physical port number.
 *   p_phy_instance PHY instance associated with the port.
 *
 * Return:
 *   true  Configuration succeeded.
 *   false PHY negotiation is incomplete or ETHSW configuration failed.
 */
static uint8_t lwip_port_configure_ethsw_speed(uint32_t port, ether_phy_instance_t const *p_phy_instance) {
    uint32_t phy_speed = 0U;
    uint32_t local_pause = 0U;
    uint32_t partner_pause = 0U;
    ethsw_link_speed_t ethsw_speed;
    fsp_err_t fsp_err;

    if (NULL == p_phy_instance) {
        return false;
    }

    fsp_err =
            p_phy_instance->p_api->linkPartnerAbilityGet(
                p_phy_instance->p_ctrl,
                &phy_speed,
                &local_pause,
                &partner_pause);

    if (FSP_SUCCESS != fsp_err) {
        USR_LOG_WARN(
            "lwIP PHY%lu speed read failed: 0x%lx",
            (unsigned long)port,
            (unsigned long)fsp_err);

        return false;
    }

    ethsw_speed =
            ethercat_port_phy_speed_to_ethsw(phy_speed);

    fsp_err =
            R_ETHSW_SpeedCfg(
                &g_ethsw0_ctrl,
                port,
                ethsw_speed);

    if (FSP_SUCCESS != fsp_err) {
        USR_LOG_WARN(
            "ETHSW port%lu speed configure failed: 0x%lx",
            (unsigned long)port,
            (unsigned long)fsp_err);

        return false;
    }

    USR_LOG_INFO(
        "lwIP port%lu speed configured: "
        "phy=%lu ethsw=%d",
        (unsigned long)port,
        (unsigned long)phy_speed,
        (int)ethsw_speed);

    return true;
}


static fsp_err_t ethernet_port_isolation_configure(void) {
    ethsw_flood_unknown_config_t flood_config =
    {
        .port_mask_bcast = {
            .mask = ETHSW_MANAGEMENT_PORT_MASK
        },
        .port_mask_mcast = {
            .mask = ETHSW_MANAGEMENT_PORT_MASK
        },
        .port_mask_ucast = {
            .mask = ETHSW_MANAGEMENT_PORT_MASK
        },
    };

    return R_ETHSW_FloodUnknownSet(
        &g_ethsw0_ctrl,
        &flood_config);
}
