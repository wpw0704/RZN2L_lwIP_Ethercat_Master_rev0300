#include "main_thread.h"
#include "um_common.h"
#include "um_serial_io_api.h"
#include "ethercat_port_monitor.h"
#include "gpt.h"
#include "ethercat_master.h"
#include "hal_data.h"
#include "crc_L16.h"

#include "host_command.h"

#include "um_lwip_port_api.h"
#include "um_ether_netif_api.h"
#include "lwip_port_main_api.h"
#include "task.h"
#include "lwip/sockets.h"

#define LWIP_APP_TASK_NAME        "lwIP app"
#define LWIP_APP_TASK_PRIORITY    (3U)
#define LWIP_APP_TASK_STACK_BYTES (2048U)

extern usr_err_t app_lwip_stack_start(void);

extern void app_lwip_task(void *pvParameters);
extern uint8_t test;
/* Main Thread entry function */
#define KEYTEST 1

static bool key_pressed(bsp_io_port_pin_t pin);

typedef struct {
    uint8_t pressed_count;
    bool latched;
} key_filter_t;


static bool key_press_event(bsp_io_port_pin_t pin,
                            key_filter_t *filter) {
    bool pressed = key_pressed(pin);

    if (pressed) {
        if (filter->pressed_count < 3U) {
            filter->pressed_count++;
        }

        /*
         * 连续检测到3次低电平，确认按下。
         * 一次按下只返回一次true。
         */
        if ((filter->pressed_count >= 3U) &&
            (!filter->latched)) {
            filter->latched = true;
            return true;
        }
    } else {
        filter->pressed_count = 0U;
        filter->latched = false;
    }

    return false;
}


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
    /* 按 ETHERCAT_LWIP_PORT_SELECTION 配置启动 EtherCAT 与 lwIP 链路监控。 */
    crc_init();
    usr_err = ethercat_port_monitor_start();
    if (USR_SUCCESS != usr_err) {
        USR_LOG_ERROR("EtherCAT port monitor start failed: %d", usr_err);
        while (1) {
            vTaskSuspend(NULL);
        }
    }
    /* 启动lwIP协议栈。 */
    usr_err = app_lwip_stack_start();
    if (USR_SUCCESS != usr_err) {
        USR_LOG_ERROR("lwIP stack start failed: %d", usr_err);

        while (1) {
            vTaskSuspend(NULL);
        }
    }

    USR_LOG_INFO("lwIP stack started.");

    /* 创建低优先级的TCP应用任务。 */
    if (pdPASS != xTaskCreate(
            app_lwip_task,
            LWIP_APP_TASK_NAME,
            LWIP_APP_TASK_STACK_BYTES / sizeof(StackType_t),
            NULL,
            LWIP_APP_TASK_PRIORITY,
            NULL)) {
        USR_LOG_ERROR("lwIP application task create failed.");

        while (1) {
            vTaskSuspend(NULL);
        }
    }
    /** TODO: add your own code here */
    while (1) {



#if  KEYTEST
        static key_filter_t key1_filter;
        static key_filter_t key2_filter;

        ethercat_motion_status_t status;
        int result;

        vTaskDelay(pdMS_TO_TICKS(100));

#if 0
        /*
         * KEY1：
         * 从当前位置正向移动1mm；
         * 速度2mm/s；
         * 加速度10mm/s²。
         */
        if (key_press_event(KEY1, &key1_filter)) {
            result = ethercat_motion_command_set(
                ETHERCAT_MOTION_MODE_MOVE_REL,
                10.0f,
                2.0f,
                10.0f);

            USR_LOG_INFO("KEY1 motion result=%d", result);
        }
#else
        /*
     * KEY1：启动持续往复运动。
     *
     * 行程：当前位置到正方向1mm
     * 速度：2mm/s
     * 加速度：10mm/s²
     * 到达终点等待：500ms
     * 返回起点后等待：500ms
     * 往返次数：0表示一直往返
     */
        if (key_press_event(KEY1, &key1_filter)) {
            result = ethercat_motion_recip_start(
                50.0f,
                10.0f,
                10.0f,
                CSP_LOCAL_JERK_MM_S3,
                100U,
                1000U,
                ETHERCAT_MOTION_RECIP_FOREVER);

            if (result == ETHERCAT_MOTION_OK) {
                USR_LOG_INFO("KEY1 reciprocation started.");
            } else {
                USR_LOG_ERROR(
                    "KEY1 reciprocation start failed: %d",
                    result);
            }
        }
#endif

        /*
         * KEY2：
         * 如果正在运动则暂停；
         * 如果已经暂停则继续；
         * 如果当前空闲则不操作。
         */
        if (key_press_event(KEY2, &key2_filter)) {
            ethercat_motion_status_get(&status);

            if (status.busy && !status.paused) {
                ethercat_motion_stop();
                USR_LOG_INFO("KEY2 motion pause");
            } else if (status.busy && status.paused) {
                result = ethercat_motion_continue();
                USR_LOG_INFO(
                    "KEY2 motion continue result=%d",
                    result);
            } else {
                USR_LOG_INFO("KEY2 motion idle");
            }
        }
#else
        vTaskDelete(NULL);
# endif
    }
}


void phy_8211(ether_phy_instance_ctrl_t *p_instance_ctrl) {
#define RTL_8211F_PAGE_SELECT 0x1F
#define RTL_8211F_EEELCR_ADDR 0x11
#define RTL_8211F_LED_PAGE 0xD04
#define RTL_8211F_LCR_ADDR 0x10

    uint32_t val1, val2 = 0;
    vTaskDelay(pdMS_TO_TICKS(100));
    /* switch to led page */
    R_ETHER_PHY_Write(p_instance_ctrl, RTL_8211F_PAGE_SELECT, RTL_8211F_LED_PAGE);

    /* set led1(green) Link 10/100/1000M, and set led2(yellow) Link 10/100/1000M+Active */
    R_ETHER_PHY_Read(p_instance_ctrl, RTL_8211F_LCR_ADDR, &val1);
    val1 |= (1 << 5);
    val1 |= (1 << 8);
    val1 &= (~(1 << 9));
    val1 |= (1 << 10);
    val1 |= (1 << 11);
    R_ETHER_PHY_Write(p_instance_ctrl, RTL_8211F_LCR_ADDR, val1);

    /* set led1(green) EEE LED function disabled so it can keep on when linked */
    R_ETHER_PHY_Read(p_instance_ctrl, RTL_8211F_EEELCR_ADDR, &val2);
    val2 &= (~(1 << 2));
    R_ETHER_PHY_Write(p_instance_ctrl, RTL_8211F_EEELCR_ADDR, val2);

    /* switch back to page0 */
    R_ETHER_PHY_Write(p_instance_ctrl, RTL_8211F_PAGE_SELECT, 0xa42);
}

static bool key_pressed(bsp_io_port_pin_t pin) {
    bsp_io_level_t level;

    if (R_IOPORT_PinRead(&g_ioport_ctrl,
                         pin,
                         &level) != FSP_SUCCESS) {
        return false;
    }

    /* 原理图外部上拉，按下时为低电平。 */
    return level == BSP_IO_LEVEL_LOW;
}
