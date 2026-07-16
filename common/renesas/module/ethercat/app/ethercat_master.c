#include "ethercat_master.h"

#define DEBUG 1
#define ETHERCAT_MASTER_TASK_NAME       "SOEM master"
#define ETHERCAT_MASTER_TASK_STACK_SIZE (8192U)

#define ETHERCAT_PDO_MONITOR_LOG_PERIOD (500U)
#define ETHERCAT_PDO_STALE_WARN_CYCLES  (1000U)

/*
 * 扫描、SDO和状态配置阶段使用的优先级。
 */
#if (configMAX_PRIORITIES < 9)
#error "configMAX_PRIORITIES must be at least 9 for the EtherCAT task priority plan"
#endif

#define ETHERCAT_MASTER_TASK_PRIORITY (7U)

#define ETHERCAT_DC_SYNC0_CYCLE_NS      (3000000U)
/* IOmap buffer for EtherCAT process data */
static char IOmap[4096];
#pragma pack(push, 1)

#pragma pack(pop)
PDO_Output *output1s;
PDO_Input *input1s;

static ethercat_pdo_monitor_t s_pdo_monitor;

static int s_expected_wkc;
static int s_last_wkc;

static servo_enable_state_t s_enable_state = SERVO_ENABLE_IDLE;
static uint32_t s_enable_wait_count = 0U;

/* SOEM 主站任务入口：完成从站扫描、SV630 PDO/DC 配置，并请求进入 OP。 */
static void ethercat_master_scan_task(void *pvParameters);

// 周期打印任务
static void ethercat_pdo_monitor_log_task(void *pvParameters);

static int ethercat_master_pdo_process_check(int wkc);

// 伺服使能
static int ethercat_servo_enable_process(int8 op_mode);

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
#if 0
            int chk = 200;
            /* request OP state for all slaves */
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data before requesting OP */
            ec_send_processdata(); // 先发一帧有效 PDO 输出
            ec_receive_processdata(EC_TIMEOUTRET); // 收一帧 PDO 输入，更新输入数据
            ec_writestate(0); // 请求所有从站切换到目标状态
            /* wait for all slaves to actually reach OP */
            do {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 5000);
            } while ((chk-- > 0) && (ec_slave[0].state != EC_STATE_OPERATIONAL));
#endif
            /*
            * 第 8 步：计算期望 WKC，并先交换一帧有效过程数据。
            * expectedWKC 用于判断 PDO 通信是否完整；进入 OP 前先发一帧可让从站输出侧准备好。
            */
            printf("Request operational state for all slaves\n");
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
            /* wait for all slaves to reach OP state */
            do {
                for (slc = 0; slc <= ec_slavecount; slc++) {
                    ec_slave[slc].state = EC_STATE_OPERATIONAL;
                    ec_writestate(slc);
                    printf("Slave %d State=0x%04x\r\n", slc, ec_slave[slc].state);
                }
            } while ((ec_slave[0].state != EC_STATE_OPERATIONAL) || (ec_slave[1].state != EC_STATE_OPERATIONAL));
            R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
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
                xTaskCreate(ethercat_pdo_monitor_log_task,
                            "PDO monitor",
                            1024U / sizeof(StackType_t),
                            NULL,
                            tskIDLE_PRIORITY + 1U,
                            NULL);
                USR_LOG_INFO("all slaves reached operational state.");
            } else {
                printf("E/BOX not found in slave configuration.\r\n");
            }
        } else {
            printf("No slaves found!\r\n");
        }
    } else {
        printf("No socket connection Excecute as root\r\n");
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
        /*
         * 正常情况下每4 ms收到一次通知。
         */
        xSemaphoreTake(s_gpt_cycle_semaphore, portMAX_DELAY);

        (void) ec_send_processdata();
        int wkc = ec_receive_processdata(EC_TIMEOUTRET);
        // ec_receive_processdata(EC_TIMEOUTRET);
        (void) ethercat_master_pdo_process_check(wkc);

        /* CSP 模式传 8。如果你暂时不想改运行模式，可以传 0。 */
        (void) ethercat_servo_enable_process(0);
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

    /*
     * CSP 模式下，使能前先把目标位置对齐到当前位置，
     * 避免 target=0 和当前位置差太大导致驱动器报警。
     */
    if (op_mode == 8) {
        output1s->TargetPos = input1s->CurrentPosition;
    }

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
            output1s->ControlWord = CIA402_CW_SWITCH_ON;

            if (status_state == CIA402_SW_SWITCHED_ON) {
                s_enable_state = SERVO_ENABLE_ENABLE_OPERATION;
                s_enable_wait_count = 0U;
            } else if (++s_enable_wait_count > 1000U) {
                s_enable_state = SERVO_ENABLE_FAILED;
                return -1;
            }
            break;

        case SERVO_ENABLE_ENABLE_OPERATION:
            output1s->ControlWord = CIA402_CW_ENABLE_OPERATION;

            if (status_state == CIA402_SW_OPERATION_ENABLED) {
                s_enable_state = SERVO_ENABLE_DONE;
                s_enable_wait_count = 0U;
                return 1;
            } else if (++s_enable_wait_count > 1000U) {
                s_enable_state = SERVO_ENABLE_FAILED;
                return -1;
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

    return pdo_ok;
}
