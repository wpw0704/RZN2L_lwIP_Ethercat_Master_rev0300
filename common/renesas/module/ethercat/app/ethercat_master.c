#include "ethercat_master.h"

#include "ethercat_app_common.h"
#include "ethercat_port_cfg.h"
#include "FreeRTOS.h"
#include "ethercat.h"
#include "task.h"
#include <stdint.h>

#define DEBUG 1
#define ETHERCAT_MASTER_TASK_NAME       "SOEM master"
#define ETHERCAT_MASTER_TASK_STACK_SIZE (8192U)
#define ETHERCAT_MASTER_TASK_PRIORITY   (tskIDLE_PRIORITY + 3U)
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

static int Servosetup(uint16 slvcnt) {
    USR_LOG_INFO(" slvcnt = %d\r\n", slvcnt);
    write8(slvcnt, 0x1C12, 00, 0);
    write8(slvcnt, 0x1600, 00, 0);
    write32(slvcnt, 0x1600, 01, 0x60400010);
    write32(slvcnt, 0x1600, 02, 0x607A0020);
    write32(slvcnt, 0x1600, 03, 0x60FF0020);
    write32(slvcnt, 0x1600, 04, 0x60600008);
    write32(slvcnt, 0x1600, 05, 0x60B80010);
    write8(slvcnt, 0x1600, 00, 5);

    write16(slvcnt, 0x1C12, 01, 0x1600);
    write8(slvcnt, 0x1C12, 00, 1);

    write8(slvcnt, 0x1C13, 00, 00);
    write8(slvcnt, 0x1A00, 00, 00);
    write32(slvcnt, 0x1A00, 01, 0x60410010);
    write32(slvcnt, 0x1A00, 02, 0x60640020);
    write32(slvcnt, 0x1A00, 03, 0x60610008);
    write32(slvcnt, 0x1A00, 04, 0x60B90010);
    write32(slvcnt, 0x1A00, 05, 0x60BA0020);
    write32(slvcnt, 0x1A00, 06, 0x60FD0020);
    write8(slvcnt, 0x1A00, 00, 06);

    write16(slvcnt, 0x1C13, 01, 0x1A00);
    write8(slvcnt, 0x1C13, 00, 01);

    return 0;
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


    /* 2.5 注册 PO2SO 回调：在 ec_config_map() 内从 PRE-OP→SAFE-OP 时，
 *     自动调用 sv630n_pdo_setup() 通过 SDO 显式配置 PDO 映射 */
    ec_slave[1].PO2SOconfig = Servosetup;
    USR_LOG_INFO("[ECAT] PO2SO callback registered for slave 1\r\n");

    ec_slave[1].hasdc = TRUE;
    ec_configdc();
    // ecx_context.manualstatechange = 1;
    ec_config_map(&IOmap);

    ec_dcsync0(1, TRUE, ETHERCAT_DC_SYNC0_CYCLE_NS, 0); /* SYNC0 = 4 ms */
    ec_readstate();

    /* 3. 分配 PDO 内存（内部会调用 PO2SO 回调） */

    //
    // /* ============================================================
    // //                * DC 寄存器回读验证 — 确认 ec_dcsync0 的 FPWR 确实写入成功
    //                * ============================================================ */
    {
        uint16_t slaveh = ec_slave[1].configadr;
        uint8_t dccuc = 0, dcsyncact = 0;
        int32_t dc_cycle0 = 0;
        int64_t dc_start0 = 0, dc_systime = 0;
        int sz;

        sz = sizeof(dccuc);
        ecx_FPRD(ecx_context.port, slaveh, 0x0980, sz, &dccuc, EC_TIMEOUTRET);
        sz = sizeof(dcsyncact);
        ecx_FPRD(ecx_context.port, slaveh, 0x0981, sz, &dcsyncact, EC_TIMEOUTRET);
        sz = sizeof(dc_cycle0);
        ecx_FPRD(ecx_context.port, slaveh, 0x09A0, sz, &dc_cycle0, EC_TIMEOUTRET);
        sz = sizeof(dc_start0);
        ecx_FPRD(ecx_context.port, slaveh, 0x0990, sz, &dc_start0, EC_TIMEOUTRET);
        sz = sizeof(dc_systime);
        ecx_FPRD(ecx_context.port, slaveh, 0x0910, sz, &dc_systime, EC_TIMEOUTRET);

        USR_LOG_INFO("[ECAT] DC verify: CUC=0x%02X SyncAct=0x%02X\r\n", dccuc, dcsyncact);
        USR_LOG_INFO("[ECAT] DC verify: SYNC0 CycleTime=%d ns\r\n", (int) dc_cycle0);
        USR_LOG_INFO("[ECAT] DC verify: SYNC0 StartTime hi=%08X lo=%08X\r\n",
                     (uint32_t) (dc_start0 >> 32), (uint32_t) (dc_start0 & 0xFFFFFFFF));
        USR_LOG_INFO("[ECAT] DC verify: SysTime     hi=%08X lo=%08X\r\n",
                     (uint32_t) (dc_systime >> 32), (uint32_t) (dc_systime & 0xFFFFFFFF));
        USR_LOG_INFO("[ECAT] DC verify: StartTime-SysTime = %d ns\r\n",
                     (int) (int32_t) (dc_start0 - dc_systime));
    }
    // /* Check current state after config */
    ec_readstate();
    USR_LOG_INFO("[ECAT] After config: slave1 state=0x%02x\r\n", ec_slave[1].state);

    /* 获取我们第 1 个从站（SV630N）的指针 */
    sv_out = (SOEM_PDO_Out_t *) ec_slave[1].outputs;
    sv_in = (SOEM_PDO_In_t *) ec_slave[1].inputs;
}




