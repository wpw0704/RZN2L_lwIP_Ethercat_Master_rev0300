#include "ethercat_master.h"

#include "ethercat_app_common.h"
#include "ethercat_port_cfg.h"
#include "FreeRTOS.h"
#include "ethercat.h"
#include "task.h"
#include <stdint.h>
#include "ethercatprint.h"
#include "gpt.h"

#define DEBUG 1
#define ETHERCAT_MASTER_TASK_NAME       "SOEM master"
#define ETHERCAT_MASTER_TASK_STACK_SIZE (8192U)
#if (configMAX_PRIORITIES < 4)
#error "configMAX_PRIORITIES must be at least 4"
#endif

/*
 * 扫描、SDO和状态配置阶段使用的优先级。
 */
#define ETHERCAT_MASTER_TASK_PRIORITY \
(tskIDLE_PRIORITY + 3U)
/*
 * OP后的4 ms周期通信优先级。
 * FreeRTOS任务优先级数值越大，优先级越高。
 */
#define ETHERCAT_CYCLE_TASK_PRIORITY \
(configMAX_PRIORITIES - 2U)
/*
 * 低优先级监控日志任务。
 */
#define ETHERCAT_MONITOR_TASK_PRIORITY \
(tskIDLE_PRIORITY + 1U)
#define ETHERCAT_MONITOR_STACK_BYTES (2048U)
#define ETHERCAT_STABLE_TEST_CYCLES (7500U) /* 7500 × 4 ms = 30秒 */
#define ETHERCAT_NOTIFY_TIMEOUT_MS  (8U)
#define ETHERCAT_DC_SYNC0_CYCLE_NS      (4000000U)
#define ETHERCAT_DC_WARMUP_MS           (1000U)
#define ETHERCAT_OP_TIMEOUT_MS          (5000U)

/* CiA402 控制字常用定义 */
#define CIA402_CTRL_SHUTDOWN        0x0006U  /* Shutdown: Fault Reset + 切换为 Ready */
#define CIA402_CTRL_SWITCH_ON       0x0007U  /* Switch On: 激磁但不使能输出 */
#define CIA402_CTRL_ENABLE_OP       0x000FU  /* Enable Operation: 允许运动 */
#define CIA402_CTRL_QUICK_STOP      0x000BU  /* Quick Stop: 快速停止 */
#define CIA402_CTRL_FAULT_RESET     0x0080U  /* Fault Reset: 清除驱动器故障 */

/* CiA402 StatusWord 掩码与期望值 */
#define CIA402_SW_MASK_FAULT        0x004FU
#define CIA402_SW_VAL_SWITCH_ON_DIS 0x0040U  /* Switch On Disabled (Fault 清除后) */
#define CIA402_SW_MASK_RDY          0x006FU
#define CIA402_SW_VAL_READY         0x0021U  /* Ready to Switch On */
#define CIA402_SW_VAL_SWITCHED_ON   0x0023U  /* Switched On */
#define CIA402_SW_VAL_OP_ENABLED    0x0027U  /* Operation Enabled */
#define CIA402_SW_BIT_FAULT         0x0008U  /* Bit3: Fault active */

/* CiA402 非阻塞状态机 — 每个 PDO 周期自动推进 */
typedef enum {
    CIA402_SEQ_IDLE = 0, /* 不进行状态切换 */
    CIA402_SEQ_FAULT_RESET_LO, /* 上升沿前奏: 确保 bit7=0 */
    CIA402_SEQ_FAULT_RESET_HI, /* 上升沿: 发 0x0080 等 Fault 清除 */
    CIA402_SEQ_SHUTDOWN, /* 发 0x0006 等 Ready to Switch On */
    CIA402_SEQ_SWITCH_ON, /* 发 0x0007 等 Switched On */
    CIA402_SEQ_ENABLE_OP, /* 发 0x000F 等 Operation Enabled */
    CIA402_SEQ_ENABLED, /* 已使能, 维持 0x000F */
    CIA402_SEQ_DISABLING, /* 正在禁用, 发 0x0006 */
    CIA402_SEQ_ERROR, /* 超时/失败 */
} CiA402_SeqState_t;

static volatile CiA402_SeqState_t s_cia402_seq = CIA402_SEQ_IDLE;
static uint32_t s_cia402_stepTick = 0; /* 当前步起始 tick (1ms 计数) */
#define CIA402_STEP_TIMEOUT_MS  2000U   /* 单步超时保护 */
/* IOmap buffer for EtherCAT process data */
static char IOmap[4096];
/* SV630 CSP 模式下使用的 PDO 映射。
 * RxPDO 0x1600: 主站 -> 驱动器，ControlWord + TargetPosition + TargetVelocity。
 * TxPDO 0x1A00: 驱动器 -> 主站，StatusWord + ActualPosition + ActualVelocity + ActualTorque。
 * ModeOfOperation 不放进 PDO，避免 8 bit 对象造成同步管理器长度奇数字节问题。 */
#pragma pack(push, 1)

PACKED_BEGIN
typedef struct PACKED {
    uint16 ControlWord;
    int32 TargetPos;
    int32 TargetVelocity;
    int8 OpModeSet;
    uint16 TouchProbe;
    //		int8 temp;
} SOEM_PDO_Out_t;

PACKED_END

PACKED_BEGIN
typedef struct PACKED {
    uint16 StatusWord;
    int32 CurrentPosition;
    int8 OpModeNow;
    uint16 TouchProbeStatus;
    int32 TouchProbePos1;
    uint32 Digitalinputs;
    //		int8 temp;
} SOEM_PDO_In_t;

PACKED_END
#pragma pack(pop)

static SOEM_PDO_Out_t s_out_shadow;
static SOEM_PDO_In_t s_in_shadow;

/* 指向 SOEM 内部 PDO 缓冲的快捷指针 */
static SOEM_PDO_Out_t *sv_out = NULL;
static SOEM_PDO_In_t *sv_in = NULL;

static TaskHandle_t s_ecat_monitor_task = NULL;

static volatile uint8_t s_ecat_cycle_running = 0U;
static volatile uint8_t s_ecat_stable_done = 0U;
static volatile uint8_t s_ecat_stable_pass = 0U;

static volatile uint32_t s_ecat_cycle_count = 0U;
static volatile uint32_t s_ecat_notify_timeout_count = 0U;
static volatile uint32_t s_ecat_missed_cycle_count = 0U;
static volatile uint32_t s_ecat_bad_wkc_count = 0U;

static volatile int s_ecat_last_wkc = 0;
static volatile int s_ecat_expected_wkc = 0;

static volatile uint16_t s_ecat_monitor_status_word = 0U;
static volatile int32_t s_ecat_monitor_position = 0;
static volatile int8_t s_ecat_monitor_mode = 0;

static volatile uint8_t s_servo_setup_ok = 0U;

static void ethercat_monitor_task(void *pvParameters);

static void ethercat_safe_op_cycle_forever(
    int expected_wkc);

/* SOEM 主站任务入口：完成从站扫描、SV630 PDO/DC 配置，并请求进入 OP。 */
static void ethercat_master_scan_task(void *pvParameters);

int write8(uint16 slave, uint16 index, uint8 subindex, int value) {
    uint8 temp = value;

    int rtn = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(temp), &temp, EC_TIMEOUTRXM);

    if (rtn == 0) {
        printf("SDOwrite to %#x failed !!! \r\n", index);
    } else if (DEBUG) {
        printf("SDOwrite to slave%d  index:%#x value:%x Successed !!! \r\n", slave, index, temp);
    }
    return rtn;
}

int write16(uint16 slave, uint16 index, uint8 subindex, int value) {
    uint16 temp = value;

    int rtn = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(temp), &temp, EC_TIMEOUTRXM * 20);

    if (rtn == 0) {
        printf("SDOwrite to %#x failed !!! \r\n", index);
    } else if (DEBUG) {
        printf("SDOwrite to slave%d  index:%#x value:%x Successed !!! \r\n", slave, index, temp);
    }
    return rtn;
}

int write32(uint16 slave, uint16 index, uint8 subindex, int value) {
    uint32 temp = value;

    int rtn = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(temp), &temp, EC_TIMEOUTRXM * 20);
    if (rtn == 0) {
        printf("SDOwrite to %#x failed !!! \r\n", index);
    } else if (DEBUG) {
        printf("SDOwrite to slave%d  index:%#x value:%x Successed !!! \r\n", slave, index, temp);
    }
    return rtn;
}

static int read16(uint16 slave,
                  uint16 index,
                  uint8 subindex,
                  uint16 *value) {
    int size;
    int ret;

    if (value == NULL) {
        return 0;
    }

    *value = 0U;
    size = sizeof(*value);
    ret = ec_SDOread(slave, index, subindex,FALSE, &size, value,EC_TIMEOUTRXM);

    if (ret <= 0) {
        USR_LOG_ERROR("SDO read failed: slave=%u index=0x%04X:%02X", slave, index, subindex);
        return 0;
    }

    return ret;
}

static int Servosetup(uint16 slave) {
    s_servo_setup_ok = 0U;
    uint16 sync_type_out = 0U;
    uint16 sync_type_in = 0U;
    if (!write8(slave, 0x1C12, 0x00, 0)) return 0;

    if (!write8(slave, 0x1600, 0x00, 0)) return 0;
    if (!write32(slave, 0x1600, 0x01, 0x60400010)) return 0;
    if (!write32(slave, 0x1600, 0x02, 0x607A0020)) return 0;
    if (!write32(slave, 0x1600, 0x03, 0x60FF0020)) return 0;
    if (!write32(slave, 0x1600, 0x04, 0x60600008)) return 0;
    if (!write32(slave, 0x1600, 0x05, 0x60B80010)) return 0;
    if (!write8(slave, 0x1600, 0x00, 5)) return 0;

    if (!write16(slave, 0x1C12, 0x01, 0x1600)) return 0;
    if (!write8(slave, 0x1C12, 0x00, 1)) return 0;

    if (!write8(slave, 0x1C13, 0x00, 0)) return 0;

    if (!write8(slave, 0x1A00, 0x00, 0)) return 0;
    if (!write32(slave, 0x1A00, 0x01, 0x60410010)) return 0;
    if (!write32(slave, 0x1A00, 0x02, 0x60640020)) return 0;
    if (!write32(slave, 0x1A00, 0x03, 0x60610008)) return 0;
    if (!write32(slave, 0x1A00, 0x04, 0x60B90010)) return 0;
    if (!write32(slave, 0x1A00, 0x05, 0x60BA0020)) return 0;
    if (!write32(slave, 0x1A00, 0x06, 0x60FD0020)) return 0;
    if (!write8(slave, 0x1A00, 0x00, 6)) return 0;

    if (!write16(slave, 0x1C13, 0x01, 0x1A00)) return 0;
    if (!write8(slave, 0x1C13, 0x00, 1)) return 0;

    //    /*
    // * CSP插补周期：
    // * 4 × 10^-3秒 = 4 ms
    // */
    //    if (!write8(slave, 0x60C2, 0x01, 4)) return 0;
    //    if (!write8(slave, 0x60C2, 0x02, -3))return 0;

    /*
     * 关键修复：
     * 0 = FreeRun
     * 2 = DC SYNC0
     */
    if (!write16(slave, 0x1C32, 0x01, 2)) return 0;
    if (!write16(slave, 0x1C33, 0x01, 2)) return 0;
    if (!read16(slave, 0x1C32, 0x01, &sync_type_out)) {
        return 0;
    }

    if (!read16(slave, 0x1C33, 0x01, &sync_type_in)) {
        return 0;
    }

    USR_LOG_INFO("Sync type: SM2=%u SM3=%u",
                 sync_type_out,
                 sync_type_in);

    if ((sync_type_out != 2U) ||
        (sync_type_in != 2U)) {
        return 0;
    }
    s_servo_setup_ok = 1U;
    return 1;
}

static void ethercat_monitor_task(void *pvParameters) {
    uint8_t final_result_reported = 0U;

    (void) pvParameters;

    for (;;) {
        uint32_t cycle_count;
        uint32_t timeout_count;
        uint32_t missed_count;
        uint32_t bad_wkc_count;
        int last_wkc;
        int expected_wkc;
        uint16_t status_word;
        int32_t position;
        int8_t mode;
        uint8_t stable_done;
        uint8_t stable_pass;

        /*
         * 监控任务每5秒打印一次。
         * 它的优先级低于EtherCAT周期任务，
         * 因此周期任务可以随时抢占它。
         */
        vTaskDelay(pdMS_TO_TICKS(5000U));

        if (!s_ecat_cycle_running) {
            continue;
        }

        cycle_count = s_ecat_cycle_count;
        timeout_count =
                s_ecat_notify_timeout_count;
        missed_count =
                s_ecat_missed_cycle_count;
        bad_wkc_count =
                s_ecat_bad_wkc_count;

        last_wkc = s_ecat_last_wkc;
        expected_wkc = s_ecat_expected_wkc;

        status_word =
                s_ecat_monitor_status_word;
        position =
                s_ecat_monitor_position;
        mode =
                s_ecat_monitor_mode;

        stable_done =
                s_ecat_stable_done;
        stable_pass =
                s_ecat_stable_pass;

        USR_LOG_INFO(
            "OP monitor: cycles=%lu "
            "WKC=%d/%d timeout=%lu "
            "missed=%lu badWKC=%lu "
            "SW=0x%04X Pos=%ld Mode=%d",
            (unsigned long)cycle_count,
            last_wkc,
            expected_wkc,
            (unsigned long)timeout_count,
            (unsigned long)missed_count,
            (unsigned long)bad_wkc_count,
            status_word,
            (long)position,
            (int)mode);

        if (stable_done &&
            !final_result_reported) {
            if (stable_pass) {
                USR_LOG_INFO(
                    "30-second stable OP test PASSED");
            } else {
                USR_LOG_ERROR(
                    "30-second stable OP test FAILED");
            }

            final_result_reported = 1U;
        }
    }
}

static void ethercat_safe_op_cycle_forever(
    int expected_wkc) {
    TaskHandle_t current_task;
    uint32_t notify_count;
    int wkc;

    current_task =
            xTaskGetCurrentTaskHandle();

    s_ecat_expected_wkc =
            expected_wkc;

    s_ecat_cycle_count = 0U;
    s_ecat_notify_timeout_count = 0U;
    s_ecat_missed_cycle_count = 0U;
    s_ecat_bad_wkc_count = 0U;

    s_ecat_last_wkc = 0;
    s_ecat_stable_done = 0U;
    s_ecat_stable_pass = 0U;

    /*
     * 清除该任务之前可能残留的通知。
     */
    while (ulTaskNotifyTake(
        pdTRUE,
        0U) != 0U) {
        /* 清空通知计数 */
    }

    /*
     * 保持安全输出：
     * EtherCAT处于OP，但伺服不使能。
     */
    sv_out->ControlWord = 0x0000U;
    sv_out->OpModeSet = 8;
    sv_out->TargetVelocity = 0;
    sv_out->TargetPos =
            sv_in->CurrentPosition;
    sv_out->TouchProbe = 0U;

    /*
     * GPT启动前先立即发送一帧，
     * 避免OP切换后产生额外空闲间隔。
     */
    (void) ec_send_processdata();

    wkc = ec_receive_processdata(
        EC_TIMEOUTRET);

    if (wkc < expected_wkc) {
        USR_LOG_ERROR(
            "Initial OP PDO failed: WKC=%d/%d",
            wkc,
            expected_wkc);
    }

    /*
     * 设置GPT唤醒目标。
     */
    gpt_set_notify_task(current_task);

    /*
     * 在启动GPT之前提升为周期通信优先级。
     */
    vTaskPrioritySet(
        current_task,
        ETHERCAT_CYCLE_TASK_PRIORITY);

    /*
     * GPT启动后，本任务不能再执行阻塞日志。
     */
    if (gpt_init() != FSP_SUCCESS) {
        /*
         * 启动失败时仍不能让任务自然返回。
         * 使用FreeRTOS Tick维持安全PDO通信。
         */
        TickType_t fallback_wake =
                xTaskGetTickCount();

        s_ecat_stable_done = 1U;
        s_ecat_stable_pass = 0U;

        for (;;) {
            sv_out->ControlWord = 0x0000U;
            sv_out->OpModeSet = 8;
            sv_out->TargetVelocity = 0;
            sv_out->TargetPos =
                    sv_in->CurrentPosition;
            sv_out->TouchProbe = 0U;

            (void) ec_send_processdata();

            s_ecat_last_wkc =
                    ec_receive_processdata(
                        EC_TIMEOUTRET);

            vTaskDelayUntil(
                &fallback_wake,
                pdMS_TO_TICKS(4U));
        }
    }

    s_ecat_cycle_running = 1U;

    for (;;) {
        /*
         * 正常情况下每4 ms收到一次通知。
         * 8 ms超时仅用于识别GPT或调度异常。
         */
        notify_count =
                ulTaskNotifyTake(
                    pdTRUE,
                    pdMS_TO_TICKS(
                        ETHERCAT_NOTIFY_TIMEOUT_MS));

        if (notify_count == 0U) {
            s_ecat_notify_timeout_count++;
        } else if (notify_count > 1U) {
            /*
             * 通知计数大于1，表示任务被延迟，
             * 至少错过了一个4 ms周期点。
             */
            s_ecat_missed_cycle_count +=
                    notify_count - 1U;
        }

        /*
         * 当前阶段只验证EtherCAT OP稳定。
         * 不发送伺服使能控制字。
         */
        sv_out->ControlWord = 0x0000U;
        sv_out->OpModeSet = 8;
        sv_out->TargetVelocity = 0;
        sv_out->TargetPos =
                sv_in->CurrentPosition;
        sv_out->TouchProbe = 0U;

        (void) ec_send_processdata();

        wkc = ec_receive_processdata(
            EC_TIMEOUTRET);

        s_ecat_last_wkc = wkc;

        if (wkc < expected_wkc) {
            s_ecat_bad_wkc_count++;
        }

        /*
         * 更新供低优先级日志任务读取的快照。
         */
        s_ecat_monitor_status_word =
                sv_in->StatusWord;

        s_ecat_monitor_position =
                sv_in->CurrentPosition;

        s_ecat_monitor_mode =
                sv_in->OpModeNow;

        s_ecat_cycle_count++;

        /*
         * 满7500周期后只设置一次最终结果，
         * 之后仍然永久进行PDO通信。
         */
        if ((!s_ecat_stable_done) &&
            (s_ecat_cycle_count >=
             ETHERCAT_STABLE_TEST_CYCLES)) {
            if ((s_ecat_notify_timeout_count == 0U) &&
                (s_ecat_missed_cycle_count == 0U) &&
                (s_ecat_bad_wkc_count == 0U) &&
                (s_ecat_last_wkc >= expected_wkc)) {
                s_ecat_stable_pass = 1U;
            } else {
                s_ecat_stable_pass = 0U;
            }

            s_ecat_stable_done = 1U;
        }
    }
}

/* 创建 SOEM 主站任务。任务已经存在时直接返回成功，防止链路抖动导致重复创建。 */
usr_err_t ethercat_master_scan_start(void) {
    ethercat_app_notify_t *p_notify;
    usr_err_t usr_err = ethercat_app_common_open();

    if (USR_SUCCESS != usr_err) {
        return usr_err;
    }

    p_notify = ethercat_app_notify_get();
    if (NULL != p_notify->master_scan_task) {
        return USR_SUCCESS;
    }

    ethercat_app_master_scan_set_state(ETHERCAT_MASTER_SCAN_STATE_RUNNING, 0);

    if (pdPASS != xTaskCreate(ethercat_master_scan_task,
                              ETHERCAT_MASTER_TASK_NAME,
                              ETHERCAT_MASTER_TASK_STACK_SIZE / sizeof(StackType_t),
                              NULL,
                              ETHERCAT_MASTER_TASK_PRIORITY,
                              &p_notify->master_scan_task)) {
        USR_LOG_ERROR("SOEM master task create failed.");
        ethercat_app_master_scan_set_state(ETHERCAT_MASTER_SCAN_STATE_FAILED, 0);
        return USR_ERR_NOT_INITIALIZED;
    }

    return USR_SUCCESS;
}

static void ethercat_verify_dc(uint16 slave) {
    uint16 config_address = ec_slave[slave].configadr;
    uint8 dcsyncact = 0U;
    uint32 cycle_time = 0U;
    uint64 start_time = 0U;
    uint64 system_time = 0U;

    (void) ec_FPRD(config_address,
                   0x0981,
                   sizeof(dcsyncact),
                   &dcsyncact,
                   EC_TIMEOUTRET);

    (void) ec_FPRD(config_address,
                   0x09A0,
                   sizeof(cycle_time),
                   &cycle_time,
                   EC_TIMEOUTRET);

    (void) ec_FPRD(config_address,
                   0x0990,
                   sizeof(start_time),
                   &start_time,
                   EC_TIMEOUTRET);

    (void) ec_FPRD(config_address,
                   0x0910,
                   sizeof(system_time),
                   &system_time,
                   EC_TIMEOUTRET);

    USR_LOG_INFO("DC: SyncAct=0x%02X Cycle=%lu ns",
                 dcsyncact,
                 (unsigned long)cycle_time);

    USR_LOG_INFO("DC: Start hi=%08lX lo=%08lX",
                 (unsigned long)(start_time >> 32),
                 (unsigned long)start_time);

    USR_LOG_INFO("DC: Time  hi=%08lX lo=%08lX",
                 (unsigned long)(system_time >> 32),
                 (unsigned long)system_time);
}

static void ethercat_master_scan_task(void *pvParameters) {
    int slave_count;
    int slave;

    (void) pvParameters;

    USR_LOG_INFO("SOEM master start on %s.", ETHERCAT_MASTER_IFNAME);

    /* 初始化 SOEM，oshw_mac 适配层会把 EtherCAT 帧绑定到 RZ/N2L port1。 */
    if (ecx_init(&ecx_context, ETHERCAT_MASTER_IFNAME) <= 0) {
        USR_LOG_ERROR("SOEM ecx_init failed.");
        ethercat_app_notify_get()->master_scan_task = NULL;
        ethercat_app_master_scan_set_state(ETHERCAT_MASTER_SCAN_STATE_FAILED, 0);
        vTaskDelete(NULL);
        return;
    }

    /* 扫描从站并进入 PRE-OP。 */
    slave_count = ecx_config_init(&ecx_context, FALSE);
    if (slave_count <= 0) {
        USR_LOG_WARN("SOEM scan finished, no EtherCAT slaves found.");
        ecx_close(&ecx_context);
        ethercat_app_notify_get()->master_scan_task = NULL;
        ethercat_app_master_scan_set_state(ETHERCAT_MASTER_SCAN_STATE_FAILED, 0);
        vTaskDelete(NULL);
        return;
    }

    ethercat_app_master_scan_set_state(ETHERCAT_MASTER_SCAN_STATE_DONE, slave_count);
    USR_LOG_INFO("SOEM scan found %d EtherCAT slave(s).", slave_count);

    for (slave = 1; slave <= ec_slavecount; slave++) {
        USR_LOG_INFO("Slave %d: name=%s, config=0x%04x, state=0x%04x, vendor=0x%08lx, product=0x%08lx.",
                     slave,
                     ec_slave[slave].name,
                     ec_slave[slave].configadr,
                     ec_slave[slave].state,
                     ec_slave[slave].eep_man,
                     ec_slave[slave].eep_id);
    }

    ec_slave[1].PO2SOconfig = Servosetup;
    /*
    * 禁止ec_config_map()自动请求SAFE-OP。
    * 因为SV630已经设置为DC Sync0模式，
    * 必须先完成DC/SYNC0配置，再进入SAFE-OP。
    */
    ecx_context.manualstatechange = 1; //开启手动状态切换
    /* 此函数内部会调用Servosetup */
    int iomap_size = ec_config_map(IOmap);

    if ((iomap_size <= 0) || (!s_servo_setup_ok)) {
        USR_LOG_ERROR("PDO map failed");
        vTaskDelete(NULL);
    }

    USR_LOG_INFO("Obytes=%lu Ibytes=%lu",
                 (unsigned long)ec_slave[1].Obytes,
                 (unsigned long)ec_slave[1].Ibytes);


    if (ec_slave[1].Obytes != sizeof(SOEM_PDO_Out_t)) {
        USR_LOG_ERROR("RxPDO size mismatch");
        vTaskDelete(NULL);
    }

    if (ec_slave[1].Ibytes != sizeof(SOEM_PDO_In_t)) {
        USR_LOG_ERROR("TxPDO size mismatch");
        vTaskDelete(NULL);
    }

    ec_readstate();

    USR_LOG_INFO("After PDO map: state=0x%04X AL=0x%04X",
                 ec_slave[1].state,
                 ec_slave[1].ALstatuscode);


    if (!ec_slave[1].hasdc) {
        USR_LOG_ERROR("Slave 1 does not support DC");
        vTaskDelete(NULL);
    }

    if (!ec_configdc()) {
        USR_LOG_ERROR("ec_configdc failed");
        vTaskDelete(NULL);
    }

    USR_LOG_INFO("ec_configdc success");

    /* 4 ms SYNC0 */
    ec_dcsync0(1,
               TRUE,
               ETHERCAT_DC_SYNC0_CYCLE_NS,
               0);
    ethercat_verify_dc(1);

    /*
    * DC和SYNC0都配置完成后，再手动请求SAFE-OP。
    */
    ec_slave[0].state = EC_STATE_SAFE_OP;

    int safeop_wkc = ec_writestate(0);

    USR_LOG_INFO("SAFE-OP request sent: WKC=%d",
                 safeop_wkc);

    if (ec_statecheck(0,
                      EC_STATE_SAFE_OP,
                      EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP) {
        ec_readstate();

        USR_LOG_ERROR(
            "SAFE-OP failed: slave=1 state=0x%04X AL=0x%04X %s",
            ec_slave[1].state,
            ec_slave[1].ALstatuscode,
            ec_ALstatuscode2string(
                ec_slave[1].ALstatuscode));

        vTaskDelete(NULL);
    }

    USR_LOG_INFO("EtherCAT SAFE-OP reached");

    int expected_wkc;
    int valid_count = 0;
    int invalid_count = 0;
    int wkc;
    TickType_t last_wake;

    expected_wkc =
            (ec_group[0].outputsWKC * 2) +
            ec_group[0].inputsWKC;

    USR_LOG_INFO("PDO WKC: outputs=%d inputs=%d expected=%d",
                 ec_group[0].outputsWKC,
                 ec_group[0].inputsWKC,
                 expected_wkc);

    sv_out = (SOEM_PDO_Out_t *) ec_slave[1].outputs;
    sv_in = (SOEM_PDO_In_t *) ec_slave[1].inputs;

    if ((sv_out == NULL) || (sv_in == NULL)) {
        USR_LOG_ERROR("PDO pointer is NULL");
        vTaskDelete(NULL);
    }

    memset(sv_out, 0, sizeof(*sv_out));

    sv_out->ControlWord = 0x0000U;
    sv_out->TargetPos = 0;
    sv_out->TargetVelocity = 0;
    sv_out->OpModeSet = 8;
    sv_out->TouchProbe = 0;

    last_wake = xTaskGetTickCount();

    for (int cycle = 0; cycle < 250; cycle++) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(4));

        (void) ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        if (wkc >= expected_wkc) {
            valid_count++;

            /*
             * 收到实际位置后，让目标位置跟随当前位置。
             * 避免后续使能时突然跳到0位置。
             */
            sv_out->TargetPos = sv_in->CurrentPosition;
        } else {
            invalid_count++;
        }

        if ((cycle % 25) == 0) {
            USR_LOG_INFO(
                "SAFEOP PDO: WKC=%d/%d SW=0x%04X Pos=%ld Mode=%d DC=%08lX",
                wkc,
                expected_wkc,
                sv_in->StatusWord,
                (long)sv_in->CurrentPosition,
                (int)sv_in->OpModeNow,
                (unsigned long)ec_DCtime);
        }
    }

    USR_LOG_INFO("SAFEOP PDO result: valid=%d invalid=%d",
                 valid_count,
                 invalid_count);
    int op_reached = 0;

    /* 保持安全输出 */
    sv_out->ControlWord = 0x0000U;
    sv_out->TargetPos = sv_in->CurrentPosition;
    sv_out->TargetVelocity = 0;
    sv_out->OpModeSet = 8;
    sv_out->TouchProbe = 0;

    /* 请求所有从站进入 EtherCAT OP */
    ec_slave[0].state = EC_STATE_OPERATIONAL;

    int state_wkc = ec_writestate(0);

    USR_LOG_INFO("OP request sent: stateWKC=%d", state_wkc);

    /*
     * 请求OP期间必须继续进行周期PDO通信。
     * 1250 × 4 ms约为5秒。
     */
    for (int cycle = 0; cycle < 1250; cycle++) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(4));

        /* 进入驱动使能前始终保持当前位置 */
        sv_out->ControlWord = 0x0000U;
        sv_out->TargetPos = sv_in->CurrentPosition;
        sv_out->OpModeSet = 8;

        (void) ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        /* 每100ms读取一次EtherCAT状态 */
        if ((cycle % 25) == 0) {
            ec_readstate();

            USR_LOG_INFO(
                "OP wait: state=0x%04X AL=0x%04X "
                "WKC=%d/%d SW=0x%04X Mode=%d",
                ec_slave[1].state,
                ec_slave[1].ALstatuscode,
                wkc,
                expected_wkc,
                sv_in->StatusWord,
                (int)sv_in->OpModeNow);

            if ((ec_slave[1].state & 0x000FU) ==
                EC_STATE_OPERATIONAL) {
                op_reached = 1;
                break;
            }
        }
    }

    if (!op_reached) {
        ec_readstate();

        USR_LOG_ERROR(
            "EtherCAT OP failed: state=0x%04X AL=0x%04X",
            ec_slave[1].state,
            ec_slave[1].ALstatuscode);

        vTaskDelete(NULL);
    }

    USR_LOG_INFO(
        "EtherCAT OP reached: state=0x%04X WKC=%d/%d",
        ec_slave[1].state,
        wkc,
        expected_wkc);

    /* 获取我们第 1 个从站（SV630N）的指针 */
    sv_out = (SOEM_PDO_Out_t *) ec_slave[1].outputs;
    sv_in = (SOEM_PDO_In_t *) ec_slave[1].inputs;

    USR_LOG_INFO(
        "EtherCAT OP reached: "
        "state=0x%04X WKC=%d/%d",
        ec_slave[1].state,
        wkc,
        expected_wkc);

    /*
     * 在启动4 ms GPT之前创建低优先级监控任务。
     */
    if (s_ecat_monitor_task == NULL) {
        if (xTaskCreate(
                ethercat_monitor_task,
                "ECAT monitor",
                ETHERCAT_MONITOR_STACK_BYTES /
                sizeof(StackType_t),
                NULL,
                ETHERCAT_MONITOR_TASK_PRIORITY,
                &s_ecat_monitor_task) != pdPASS) {
            /*
             * 监控任务失败不会阻止周期通信，
             * 但先记录错误。
             */
            USR_LOG_ERROR(
                "EtherCAT monitor task create failed");
        }
    }

    USR_LOG_INFO(
        "Starting 30-second stable OP test");

    /*
     * 进入后不再返回。
     */
    ethercat_safe_op_cycle_forever(
        expected_wkc);
}
