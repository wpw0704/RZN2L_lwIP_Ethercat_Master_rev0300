#include "ethercat_master.h"
#include "ethercat_app_common.h"

#define DEBUG 1
#define ETHERCAT_MASTER_TASK_NAME       "SOEM master"
#define ETHERCAT_MASTER_TASK_STACK_SIZE (8192U)

#define ETHERCAT_PDO_MONITOR_LOG_PERIOD (500U)
#define ETHERCAT_PDO_STALE_WARN_CYCLES  (1000U)

#define CSP_COUNTS_PER_REV              (262144L)
#define CSP_TEST_STEP_COUNTS            (64L)

#define CSP_LOCAL_TEST_ENABLE          (1U)
#define CSP_LOCAL_TEST_WAIT_CYCLES     (500U)  /* 2ms × 500 = 1秒 */
#define CSP_LOCAL_TEST_MAX_POS_ERROR   (500L)  /* 约0.095mm */

/*
 * 扫描、SDO和状态配置阶段使用的优先级。
 */
#if (configMAX_PRIORITIES < 9)
#error "configMAX_PRIORITIES must be at least 9 for the EtherCAT task priority plan"
#endif

#define ETHERCAT_MASTER_TASK_PRIORITY (7U)

#define ETHERCAT_DC_SYNC0_CYCLE_NS      (2000000U)


typedef enum {
    CSP_TEST_IDLE = 0,
    CSP_TEST_FORWARD,
    CSP_TEST_BACKWARD,
    CSP_TEST_DONE,
} csp_test_state_t;

static csp_test_state_t s_csp_test_state = CSP_TEST_IDLE;
static int32 s_csp_start_pos = 0;
static int32 s_csp_target_pos = 0;
static int32 s_servo_enable_hold_pos = 0;

static int ethercat_csp_one_rev_test_process(void);

/* SOEM 过程数据映射区，ec_config_map() 会把 PDO 输入输出映射到这里 */
static char IOmap[4096];

/* 指向 1 号从站 PDO 输出/输入区，进入 OP 后由 ec_slave[slc].outputs/inputs 赋值 */
PDO_Output *output1s;
PDO_Input *input1s;

/* PDO 监控缓存，由 2ms PDO 周期任务更新，由低优先级日志任务读取 */
static ethercat_pdo_monitor_t s_pdo_monitor;

/* WKC 统计：expected 是理论期望值，last 是最近一次 PDO 返回值 */
static int s_expected_wkc;
static int s_last_wkc;

/* CiA402 使能状态机当前阶段和等待计数 */
static servo_enable_state_t s_enable_state = SERVO_ENABLE_IDLE;
static uint32_t s_enable_wait_count = 0U;

/* SOEM 主站任务入口：完成从站扫描、SV630 PDO/DC 配置，并请求进入 OP。 */
static void ethercat_master_scan_task(void *pvParameters);

// 周期打印任务
static void ethercat_pdo_monitor_log_task(void *pvParameters);

static int ethercat_master_pdo_process_check(int wkc);

// 伺服使能
static int ethercat_servo_enable_process(int8 op_mode);

/******************************测试************************************/
static void ethercat_csp_local_test_process(void);

/*
 * SDO 写封装函数。
 * 用于在 PRE-OP 到 SAFE-OP 阶段配置 PDO 映射对象，例如 0x1600、0x1A00、0x1C12、0x1C13。
 * 注意：SDO 是邮箱通信，不能放在 2ms PDO 周期任务中频繁调用。
 */
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

// PDO配置
static int Servosetup(uint16 slvcnt) {
    printf(" slvcnt = %d\r\n", slvcnt);
    write8(slvcnt, 0x1C12, 00, 0); // 清空0x1c12
    write8(slvcnt, 0x1600, 00, 0); // 清空0x1600
    write32(slvcnt, 0x1600, 01, 0x60400010); // 写入0x1600
    write32(slvcnt, 0x1600, 02, 0x607A0020); // 写入0x1600
    write32(slvcnt, 0x1600, 03, 0x60FF0020); // 写入0x1600
    write32(slvcnt, 0x1600, 04, 0x60600008); // 写入0x1600
    write32(slvcnt, 0x1600, 05, 0x60B80010); // 写入0x1600
    write8(slvcnt, 0x1600, 00, 5);

    write16(slvcnt, 0x1C12, 01, 0x1600); // 设定RxPDO映射
    write8(slvcnt, 0x1C12, 00, 1);

    write8(slvcnt, 0x1C13, 00, 00); // 清空0x1C12计数
    write8(slvcnt, 0x1A00, 00, 00); // 清空0x1600计数
    write32(slvcnt, 0x1A00, 01, 0x60410010); // 写入0x1A00
    write32(slvcnt, 0x1A00, 02, 0x60640020); // 写入0x1A00
    write32(slvcnt, 0x1A00, 03, 0x60610008); // 写入0x1A00
    write32(slvcnt, 0x1A00, 04, 0x60B90010); // 写入0x1A00
    write32(slvcnt, 0x1A00, 05, 0x60BA0020); // 写入0x1A00
    write32(slvcnt, 0x1A00, 06, 0x60FD0020); // 写入0x1A00
    write8(slvcnt, 0x1A00, 00, 06);

    write16(slvcnt, 0x1C13, 01, 0x1A00);
    write8(slvcnt, 0x1C13, 00, 01);

    return 0;
}

void ecat_init(void) {
    int slc;
    /*
     * 第 1 步：初始化 SOEM 网卡接口。
     * ec_init() 会调用本工程的 nicdrv/oshw 适配层，把 SOEM 主站绑定到 EtherCAT 网口。
     */
    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ETHERCAT_MASTER_IFNAME)) {
        USR_LOG_INFO("ec_init succeeded.");
        /*
         * 第 2 步：扫描并初始化 EtherCAT 从站。
         * ec_config_init(TRUE) 会枚举总线从站；返回值大于 0 表示至少发现一个从站。
         */
        if (ec_config_init(TRUE) > 0) {
            if (ec_slavecount >= 1) {
                for (slc = 1; slc <= ec_slavecount; slc++) {
                    /*
                     * 第 3 步：打印从站信息，并挂接 PDO 配置回调。
                     * 后续 PRE-OP 到 SAFE-OP 阶段会调用 Servosetup() 配置 0x1600/0x1A00 等 PDO 映射。
                     */
                    printf("%ld slaves found and configured. %ld \r\n", ec_slave[slc].eep_man, ec_slave[slc].eep_id);
                    printf("Found name=%s at position %d\r\n", ec_slave[slc].name, slc);
                    printf("Found configadr=%d at position %d\r\n", ec_slave[slc].configadr, slc);
                    //					if ((ec_slave[slc].eep_man == 0x100000) && (ec_slave[slc].eep_id == 0xc0112))
                    ec_slave[slc].PO2SOconfig = &Servosetup;
                    //					else
                    //						USR_LOG_INFO("NULL");
                }
            }

            /*
             * 第 4 步：配置分布式时钟 DC 与 SYNC0。
             * 当前代码对 1 号从站开启 SYNC0，周期为 ETHERCAT_DC_SYNC0_CYCLE_NS，即 2 ms。
             */
            ec_configdc();
            // for (slc = 1; slc <= ec_slavecount; slc++)
            ec_dcsync0(1,TRUE,ETHERCAT_DC_SYNC0_CYCLE_NS, 0);
            /*
             * 第 5 步：建立 PDO 过程数据映射。
             * ec_config_map() 会生成 IOmap，并把 ec_slave[x].outputs / inputs 指向对应 PDO 区域。
             */
            ec_config_map(&IOmap);
            printf("Slaves mapped, state to SAFE_OP.\n \r");
            /*
             * 第 6 步：等待所有从站进入 SAFE_OP。
             * SAFE_OP 表示 PDO 映射已经生效，输入可读，但还未进入正式输出运行状态。
             */
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
            ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_SAFE_OP);

            /*
             * 第 7 步：刷新并打印主站/从站状态。
             * ec_readstate() 会读取各从站 AL 状态并更新 ec_slave[]。
             */
            /* read indevidual slave state and store in ec_slave[] */
            ec_readstate();
            for (slc = 0; slc <= ec_slavecount; slc++)
                printf("Slave %d State=0x%04x\r\n", slc, ec_slave[slc].state);
            printf("segments : %d : %ld %ld %ld %ld\n", ec_group[0].nsegments, ec_group[0].IOsegment[0],
                   ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

            /*
            * 第 8 步：计算期望 WKC，并先交换一帧有效过程数据。
            * expectedWKC 用于判断 PDO 通信是否完整；进入 OP 前先发一帧可让从站输出侧准备好。
            */
            printf("Request operational state for all slaves\n");
            ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_OP_REQUESTING);
            int expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /*
             * 第 9 步：请求从站进入 OPERATIONAL。
             * OP 状态表示 EtherCAT 总线进入正式过程数据交互阶段。
             */
            ec_writestate(0);
            R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
            /*
            * 请求进入 OP 后，持续交换 PDO，并用 ec_readstate() 读取真实状态。
            * 不使用长时间阻塞的 ec_statecheck(..., 5000)，避免 SV630 因 PDO/SYNC 更新超时触发 E08.6。
            */
#if 1
            int chk = 200;
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* 先发一帧有效 PDO */
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            /* 请求所有从站进入 OP */
            ec_writestate(0);

            /* 等待 OP，但循环里持续 PDO 交换，不做长时间阻塞 */
            do {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);

                ec_readstate();

                R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
            } while ((chk-- > 0) && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            for (slc = 1; slc <= ec_slavecount; slc++) {
                printf("Slave %d State=0x%04x ALstatuscode=0x%04x\r\n",
                       slc,
                       ec_slave[slc].state,
                       ec_slave[slc].ALstatuscode);
            }
#else
            // 强制进入OP
            /* wait for all slaves to reach OP state */
            do {
                for (slc = 0; slc <= ec_slavecount; slc++) {
                    ec_slave[slc].state = EC_STATE_OPERATIONAL;
                    ec_writestate(slc);
                    printf("Slave %d State=0x%04x\r\n", slc, ec_slave[slc].state);
                }
            } while ((ec_slave[0].state != EC_STATE_OPERATIONAL) || (ec_slave[1].state != EC_STATE_OPERATIONAL));
            R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
#endif
            ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_OPERATIONAL);
            if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
                /*
                 * 第 10 步：保存 PDO 输入/输出结构体指针。
                 * output1s 指向主站写给驱动器的 RxPDO；input1s 指向驱动器返回给主站的 TxPDO。
                 */
                for (slc = 1; slc <= ec_slavecount; slc++) {
                    output1s = (PDO_Output *) ec_slave[slc].outputs;
                    input1s = (PDO_Input *) ec_slave[slc].inputs;
                }
                /*
                * GPT启动后，本任务不能再执行阻塞日志。*/
                /*
                 * 第 11 步：启动 GPT 周期定时器。
                 * GPT 回调每 4 ms 释放一次信号量，SOEM master 任务后续按该节拍执行 PDO 收发。
                 */
                if (gpt_init() != FSP_SUCCESS) {
                    USR_LOG_INFO("GPT FARIL");
                }
                ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_PDO_RUNNING);
                xTaskCreate(ethercat_pdo_monitor_log_task,
                            "PDO monitor",
                            1024U / sizeof(StackType_t),
                            NULL,
                            tskIDLE_PRIORITY + 1U,
                            NULL);
                /* 设置机械参数：电机一圈 262144 counts，导程 50mm，直连 */
                ethercat_csp_mech_param_set(262144.0f, 50.0f, 1.0f);

                USR_LOG_INFO("all slaves reached operational state.");
            } else {
                printf("E/BOX not found in slave configuration.\r\n");
                ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_FAILED);
            }
        } else {
            printf("No slaves found!\r\n");
            ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_FAILED);
        }
    } else {
        printf("No socket connection Excecute as root\r\n");
        ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_FAILED);
    }
}


/* 创建 SOEM 主站任务。任务已经存在时直接返回成功，防止链路抖动导致重复创建。 */
usr_err_t ethercat_master_scan_start(void) {
    usr_err_t usr_err = ethercat_app_common_open();

    if (USR_SUCCESS != usr_err) {
        return usr_err;
    }
    ethercat_app_notify_t *p_notify = ethercat_app_notify_get();
    if (NULL != p_notify->master_scan_task) {
        return USR_SUCCESS;
    }
    ethercat_app_master_scan_set_state(ETHERCAT_MASTER_SCAN_STATE_RUNNING, 0);
    ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_SCANNING);

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
    (void) pvParameters;
    USR_LOG_INFO("SOEM master start on %s.", ETHERCAT_MASTER_IFNAME);
    ecat_init();

    for (;;) {
        xSemaphoreTake(s_gpt_cycle_semaphore, portMAX_DELAY);

        /* 1. 发送上一周期已经准备好的目标值 */
        (void) ec_send_processdata();

        /* 2. 接收本周期反馈 */
        int wkc = ec_receive_processdata(EC_TIMEOUTRET);

        /* 3. 检查 PDO 状态，只做简单统计 */
        (void) ethercat_master_pdo_process_check(wkc);

        /* 4. 伺服使能状态机，只写 ControlWord */
        if (ethercat_servo_enable_process(0) == 1) {
            /*
             * 5. 计算下一周期 TargetPos
             * 注意：这里算出的 TargetPos 会在下一次 ec_send_processdata() 发出去。
             */
            ethercat_csp_local_test_process();
            ethercat_csp_motion_process();
            // ethercat_csp_one_rev_test_process();
        }
    }
}

/************************** 伺服使能 **************************/
/*
 * 返回值：
 *  1  = 已进入 Operation Enabled
 *  0  = 正在使能过程中
 * -1  = 使能失败或 PDO 未准备好
 */
static int ethercat_servo_enable_process(int8 op_mode) {
    uint16 status_word;
    uint16 status_state;

    if ((input1s == NULL) || (output1s == NULL)) {
        return -1;
    }

    status_word = input1s->StatusWord;
    status_state = status_word & CIA402_SW_MASK;


    if (op_mode != 0) {
        output1s->OpModeSet = op_mode;
    }

    if (status_state == CIA402_SW_OPERATION_ENABLED) {
        output1s->ControlWord = CIA402_CW_ENABLE_OPERATION;
        s_enable_state = SERVO_ENABLE_DONE;
        s_enable_wait_count = 0U;
        return 1;
    }

    switch (s_enable_state) {
        case SERVO_ENABLE_IDLE:
            s_enable_wait_count = 0U;

            if ((status_word & CIA402_SW_FAULT_MASK) == CIA402_SW_FAULT) {
                s_enable_state = SERVO_ENABLE_FAULT_RESET_PULSE;
            } else {
                s_enable_state = SERVO_ENABLE_SHUTDOWN;
            }
            break;

        case SERVO_ENABLE_FAULT_RESET_PULSE:
            /*
             * Fault Reset 只给一个短脉冲，不要一直保持 0x0080。
             */
            output1s->ControlWord = CIA402_CW_FAULT_RESET;
            s_enable_state = SERVO_ENABLE_WAIT_FAULT_CLEAR;
            s_enable_wait_count = 0U;
            break;

        case SERVO_ENABLE_WAIT_FAULT_CLEAR:
            /*
             * 复位后先回 0，等待故障位清除。
             */
            output1s->ControlWord = CIA402_CW_DISABLE_VOLTAGE;

            if ((status_word & CIA402_SW_FAULT_MASK) != CIA402_SW_FAULT) {
                s_enable_state = SERVO_ENABLE_SHUTDOWN;
                s_enable_wait_count = 0U;
            } else if (++s_enable_wait_count > 500U) {
                s_enable_state = SERVO_ENABLE_FAILED;
                return -1;
            }
            break;

        case SERVO_ENABLE_SHUTDOWN:
            output1s->ControlWord = CIA402_CW_SHUTDOWN;

            if (status_state == CIA402_SW_READY_TO_SWITCH_ON) {
                s_enable_state = SERVO_ENABLE_SWITCH_ON;
                s_enable_wait_count = 0U;
            } else if (++s_enable_wait_count > 1000U) {
                s_enable_state = SERVO_ENABLE_FAILED;
                return -1;
            }
            break;

        case SERVO_ENABLE_SWITCH_ON:
            output1s->ControlWord = CIA402_CW_SWITCH_ON; /* 保持0x0007 */

            if (status_state == CIA402_SW_SWITCHED_ON) {
                /* 在0x000F使能之前，先锁定当前位置 */
                s_servo_enable_hold_pos = input1s->CurrentPosition;
                output1s->TargetPos = s_servo_enable_hold_pos;
                output1s->TargetVelocity = 0;
                output1s->OpModeSet = 8;

                s_enable_wait_count = 0U;
                s_enable_state = SERVO_ENABLE_CSP_PREPARE;
            }
            break;

        case SERVO_ENABLE_CSP_PREPARE:
            /*
             * 仍保持0x0007，不输出转矩。
             * 连续发送几帧当前位置，确保驱动器已收到安全目标。
             */
            output1s->ControlWord = CIA402_CW_SWITCH_ON;
            output1s->OpModeSet = 8;

            s_servo_enable_hold_pos = input1s->CurrentPosition;
            output1s->TargetPos = s_servo_enable_hold_pos;
            output1s->TargetVelocity = 0;

            if ((input1s->OpModeNow == 8) &&
                (++s_enable_wait_count >= 5U)) {
                s_enable_wait_count = 0U;
                s_enable_state = SERVO_ENABLE_ENABLE_OPERATION;
            }
            break;

        case SERVO_ENABLE_ENABLE_OPERATION:
            /*
             * 第一次发送0x000F时，目标位置已经提前发送了至少5个周期。
             */
            output1s->TargetPos = s_servo_enable_hold_pos;
            output1s->TargetVelocity = 0;
            output1s->ControlWord = CIA402_CW_ENABLE_OPERATION;

            if (status_state == CIA402_SW_OPERATION_ENABLED) {
                s_enable_state = SERVO_ENABLE_DONE;
                s_enable_wait_count = 0U;
                return 1;
            }
            break;

        case SERVO_ENABLE_DONE:
            output1s->ControlWord = CIA402_CW_ENABLE_OPERATION;
            return 1;

        case SERVO_ENABLE_FAILED:
        default:
            output1s->ControlWord = CIA402_CW_DISABLE_VOLTAGE;
            return -1;
    }

    return 0;
}


/************************** 任务日志 **************************/
/*
 * 低优先级 PDO 监控日志任务。
 * 该任务每 1 秒打印一次缓存数据，不直接调用 SOEM 收发函数，避免影响 PDO 实时周期。
 */
static void ethercat_pdo_monitor_log_task(void *pvParameters) {
    ethercat_pdo_monitor_t mon;

    (void) pvParameters;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        taskENTER_CRITICAL();
        mon = s_pdo_monitor;
        taskEXIT_CRITICAL();

        USR_LOG_INFO("PDO monitor: ok=%d cyc=%lu good=%lu bad=%lu wkc=%d/%d state=0x%04x "
                     "status=0x%04x pos=%ld dpos=%ld mode=%d ctrl=0x%04x target=%ld.",
                     mon.pdo_ok,
                     (unsigned long) mon.cycle_count,
                     (unsigned long) mon.good_count,
                     (unsigned long) mon.bad_count,
                     mon.wkc,
                     mon.expected_wkc,
                     mon.state,
                     mon.status_word,
                     (long) mon.position,
                     (long) mon.position_delta,
                     mon.mode,
                     mon.control_word,
                     (long) mon.target_pos);
    }
}

int ethercat_master_pdo_process_check(int wkc) {
    static int32 last_position; // 保存上一周期的位置
    static uint16 last_status_word; // 保存上一周期的状态字
    static int8 last_mode; // 保存上一周期的模式
    static uint8_t has_last_sample; // 是否已经有上一周期样本

    int pdo_ok = 1;
    int32 position_delta = 0;

    /* 保存最近一次 WKC，供其他任务读取 */
    s_last_wkc = wkc;

    /* 如果期望 WKC 还没有初始化，则根据 SOEM 的 group 信息计算 */
    if (s_expected_wkc <= 0) {
        s_expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    }

    /*
     * PDO 指针未准备好，说明还没有完成 OP 后的 inputs/outputs 指针绑定。
     * 这种情况下不能访问 input1s/output1s，否则可能 HardFault。
     */
    if ((input1s == NULL) || (output1s == NULL)) {
        pdo_ok = 0;

        taskENTER_CRITICAL();
        s_pdo_monitor.cycle_count++;
        s_pdo_monitor.bad_count++;
        s_pdo_monitor.wkc = wkc;
        s_pdo_monitor.expected_wkc = s_expected_wkc;
        s_pdo_monitor.pdo_ok = pdo_ok;
        taskEXIT_CRITICAL();

        return 0;
    }

    /*
     * 判断 WKC 是否达标。
     * wkc < expected_wkc 表示本周期 PDO 收发不完整，输入数据不一定可信。
     */
    if ((s_expected_wkc > 0) && (wkc < s_expected_wkc)) {
        pdo_ok = 0;
    }

    /*
     * 判断 EtherCAT 状态是否仍然是 OP。
     * 0x0008 = EC_STATE_OPERATIONAL。
     */
    if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
        pdo_ok = 0;
    }

    /*
     * 计算当前位置变化量。
     * 如果手动转动电机头，position_delta 应该会出现明显变化。
     */
    if (has_last_sample) {
        position_delta = input1s->CurrentPosition - last_position;

        /*
         * 如果位置、状态字、模式都没有变化，认为输入数据连续未变化。
         * 注意：电机静止时 unchanged 增加是正常现象，不一定是错误。
         */
        if ((input1s->CurrentPosition == last_position) &&
            (input1s->StatusWord == last_status_word) &&
            (input1s->OpModeNow == last_mode)) {
            s_pdo_monitor.unchanged_count++;
        } else {
            s_pdo_monitor.unchanged_count = 0U;
        }
    } else {
        has_last_sample = 1U;
    }

    /* 更新上一周期样本 */
    last_position = input1s->CurrentPosition;
    last_status_word = input1s->StatusWord;
    last_mode = input1s->OpModeNow;

    /*
     * 进入临界区，防止 PDO 周期任务正在更新时，
     * 另一个打印任务同时读取 s_pdo_monitor，导致数据读到一半。
     */
    taskENTER_CRITICAL();

    s_pdo_monitor.cycle_count++;
    s_pdo_monitor.wkc = wkc;
    s_pdo_monitor.expected_wkc = s_expected_wkc;
    s_pdo_monitor.pdo_ok = pdo_ok;

    if (pdo_ok) {
        s_pdo_monitor.good_count++;
    } else {
        s_pdo_monitor.bad_count++;
    }

    /*
     * 保存本周期关键 PDO 数据。
     * 打印任务只读取这些缓存值，不直接访问 SOEM 收发函数。
     */
    s_pdo_monitor.state = ec_slave[0].state;
    s_pdo_monitor.status_word = input1s->StatusWord;
    s_pdo_monitor.position = input1s->CurrentPosition;
    s_pdo_monitor.position_delta = position_delta;
    s_pdo_monitor.mode = input1s->OpModeNow;
    s_pdo_monitor.control_word = output1s->ControlWord;
    s_pdo_monitor.target_pos = output1s->TargetPos;

    taskEXIT_CRITICAL();
    ethercat_app_master_status_update(wkc,
                                      s_expected_wkc,
                                      ec_slave[0].state,
                                      input1s->StatusWord,
                                      output1s->ControlWord);
    // if (!pdo_ok) {
    //     ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_FAULT);
    // }

    return pdo_ok;
}

/*
 * CSP 正反一圈测试：
 * - 第一次进入时记录当前位置
 * - 正转 1 圈
 * - 再反转回起点
 * - 全程只更新 TargetPos，不阻塞、不打印
 *
 * 返回值：
 *  0 = 正在运行
 *  1 = 测试完成
 * -1 = PDO 未准备好或未使能
 */
static int ethercat_csp_one_rev_test_process(void) {
    int32 forward_target;

    if ((input1s == NULL) || (output1s == NULL)) {
        return -1;
    }

    /* 必须已经 Operation Enabled */
    if ((input1s->StatusWord & CIA402_SW_MASK) != CIA402_SW_OPERATION_ENABLED) {
        return -1;
    }

    /* 必须处于 CSP 模式，0x6061 = 8 */
    if (input1s->OpModeNow != 8) {
        output1s->OpModeSet = 8;
        output1s->TargetPos = input1s->CurrentPosition;
        return -1;
    }

    switch (s_csp_test_state) {
        case CSP_TEST_IDLE:
            s_csp_start_pos = input1s->CurrentPosition;
            s_csp_target_pos = s_csp_start_pos;

            output1s->OpModeSet = 8;
            output1s->TargetVelocity = 0;
            output1s->TargetPos = s_csp_target_pos;

            s_csp_test_state = CSP_TEST_FORWARD;
            break;

        case CSP_TEST_FORWARD:
            forward_target = s_csp_start_pos + CSP_COUNTS_PER_REV;

            if (s_csp_target_pos < forward_target) {
                s_csp_target_pos += CSP_TEST_STEP_COUNTS;

                if (s_csp_target_pos > forward_target) {
                    s_csp_target_pos = forward_target;
                }
            } else {
                s_csp_test_state = CSP_TEST_BACKWARD;
            }

            output1s->TargetPos = s_csp_target_pos;
            break;

        case CSP_TEST_BACKWARD:
            if (s_csp_target_pos > s_csp_start_pos) {
                s_csp_target_pos -= CSP_TEST_STEP_COUNTS;

                if (s_csp_target_pos < s_csp_start_pos) {
                    s_csp_target_pos = s_csp_start_pos;
                }
            } else {
                s_csp_test_state = CSP_TEST_DONE;
            }

            output1s->TargetPos = s_csp_target_pos;
            break;

        case CSP_TEST_DONE:
            output1s->TargetPos = s_csp_start_pos;
            return 1;

        default:
            s_csp_test_state = CSP_TEST_IDLE;
            break;
    }

    return 0;
}


/*
 * 本地单次运动测试。
 * 驱动器稳定使能1秒后，自动执行一次1mm绝对位置运动。
 */
static void ethercat_csp_local_test_process(void)
{
#if CSP_LOCAL_TEST_ENABLE
    static uint32_t stable_cycles = 0U;
    static uint8_t command_sent = 0U;
    int32 position_error;

    /* 每次上电只发送一次测试命令 */
    if (command_sent != 0U) {
        return;
    }

    /* 必须已经进入Operation Enabled和CSP模式 */
    if (((input1s->StatusWord & CIA402_SW_MASK) !=
         CIA402_SW_OPERATION_ENABLED) ||
        (input1s->OpModeNow != 8)) {
        stable_cycles = 0U;
        return;
        }

    /* 检查目标位置与实际位置是否已经基本一致 */
    position_error = output1s->TargetPos - input1s->CurrentPosition;

    if (position_error < 0) {
        position_error = -position_error;
    }

    if (position_error > CSP_LOCAL_TEST_MAX_POS_ERROR) {
        stable_cycles = 0U;
        return;
    }

    /* 连续稳定1秒后才下发测试运动 */
    if (++stable_cycles >= CSP_LOCAL_TEST_WAIT_CYCLES) {

        /*
         * 向正方向移动1mm：
         * 位置：1mm
         * 速度：0.002m/s，即2mm/s
         * 加速度：0.02m/s²
         */
        // ethercat_csp_move_abs_start_mm(250.0f, 1.0f, 0.1f);
        ethercat_csp_move_rel_start_mm(-250.0f, 1.0f, 0.1f);

        command_sent = 1U;
    }
#endif
}
