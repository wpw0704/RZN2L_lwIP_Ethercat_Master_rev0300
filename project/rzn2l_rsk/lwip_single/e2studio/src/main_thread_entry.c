#include "main_thread.h"
#include "um_common.h"
#include "um_serial_io_api.h"
#include "ethercat_port_monitor.h"
#include "gpt.h"
#include "ethercat_master.h"
/* Main Thread entry function */

static bool key_pressed(bsp_io_port_pin_t pin);

/* pvParameters contains TaskHandle_t */
void main_thread_entry(void *pvParameters) {
    FSP_PARAMETER_NOT_USED(pvParameters);
    /** Error status */
    usr_err_t usr_err = USR_SUCCESS;
    usr_err = um_serial_io_uart_open(&g_uart0);
    if (USR_SUCCESS != usr_err) {
        /** Note:Error Handling  */
        while (1) {
            vTaskSuspend(NULL);
        }
    }
    USR_LOG_INFO("Started Serial I/O interface.");
    // gpt_init();

    /* 当前阶段不运行发包验证函数，只启动 port1 链路稳定监控，为后续 SOEM 扫描从站做准备。 */
    usr_err = ethercat_port_monitor_start();
    if (USR_SUCCESS != usr_err) {
        USR_LOG_ERROR("EtherCAT port monitor start failed: %d", usr_err);
        while (1) {
            vTaskSuspend(NULL);
        }
    }

    /** TODO: add your own code here */
    while (1) {
        static int i = 0;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // vTaskDelete(NULL);
        if (key_pressed(KEY1)) {
            /* KEY1按下 */
            i = 1;
        }

        if (key_pressed(KEY2)) {
            /* KEY2按下 */
            i = 2;
        }
        if (i == 0) {
            USR_LOG_INFO("Started Serial I/O interface.");
        }else if (i == 1) {
            USR_LOG_INFO("KEY1111");
        }else if (i == 2) {
            USR_LOG_INFO("KEY2222");
        }

    }

}


void phy_8211(ether_phy_instance_ctrl_t *p_instance_ctrl) {
    /* RTL8211F extended-page and LED control registers. */
#define RTL_8211F_PAGE_SELECT 0x1F
#define RTL_8211F_EEELCR_ADDR 0x11
#define RTL_8211F_LED_PAGE 0xD04
#define RTL_8211F_LCR_ADDR 0x10

    uint32_t val1, val2 = 0;

    /* Switch to the RTL8211F LED configuration page. */
    R_ETHER_PHY_Write(p_instance_ctrl, RTL_8211F_PAGE_SELECT, RTL_8211F_LED_PAGE);

    /* Configure green LED as link status and yellow LED as link/activity. */
    R_ETHER_PHY_Read(p_instance_ctrl, RTL_8211F_LCR_ADDR, &val1);
    /* LED1: link at 10/100/1000 Mbps. */
    val1 |= 1U << 5;
    val1 |= 1U << 8;
    val1 &= ~(1U << 9);
    /* LED2: link at 10/100/1000 Mbps plus activity indication. */
    val1 |= 1U << 10;
    val1 |= 1U << 11;
    R_ETHER_PHY_Write(p_instance_ctrl, RTL_8211F_LCR_ADDR, val1);

    /* Disable EEE LED mode so the green LED stays on while the link is up. */
    R_ETHER_PHY_Read(p_instance_ctrl, RTL_8211F_EEELCR_ADDR, &val2);
    val2 &= ~(1U << 2);
    R_ETHER_PHY_Write(p_instance_ctrl, RTL_8211F_EEELCR_ADDR, val2);

    /* Return to the PHY default register page used by the driver. */
    R_ETHER_PHY_Write(p_instance_ctrl, RTL_8211F_PAGE_SELECT, 0xa42);
    // R_ETHER_PHY_Write(p_instance_ctrl, RTL_8211F_PAGE_SELECT, 0x0000);

    R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);

    R_ETHER_PHY_Read(p_instance_ctrl, 0x02, &val2);

    (void) val2;
}

static bool key_pressed(bsp_io_port_pin_t pin)
{
    bsp_io_level_t level;

    if (R_IOPORT_PinRead(&g_ioport_ctrl,
                         pin,
                         &level) != FSP_SUCCESS) {
        return false;
                         }

    /* 原理图外部上拉，按下时为低电平。 */
    return level == BSP_IO_LEVEL_LOW;
}
