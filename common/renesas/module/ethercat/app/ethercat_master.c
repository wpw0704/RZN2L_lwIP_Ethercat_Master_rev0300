#include "ethercat_master.h"
#include "ethercat_app_common.h"

#define DEBUG 1
#define ETHERCAT_MASTER_TASK_NAME       "SOEM master"
#define ETHERCAT_MASTER_TASK_STACK_SIZE (8192U)
/*
 * 扫描、SDO和状态配置阶段使用的优先级。
 */
#if (configMAX_PRIORITIES < 9)
#error "configMAX_PRIORITIES must be at least 9 for the EtherCAT task priority plan"
#endif

#define ETHERCAT_MASTER_TASK_PRIORITY (7U)
#define ETHERCAT_DC_SYNC0_CYCLE_NS      (2000000U)
#define ETHERCAT_CIA402_ENABLE_ACTIVE   (0U)
#define ETHERCAT_SAFE_OP_WARMUP_CYCLES  (50U)
#define ETHERCAT_AXIS_OP_TIMEOUT_CYCLES (1000U)
#define ETHERCAT_AXIS_OP_STABLE_CYCLES  (50U)
#define ETHERCAT_STATE_CHECK_INTERVAL   (25U)
#define ETHERCAT_ZERO_WKC_LIMIT         (5U)
#define ETHERCAT_GPT_WAIT_MS            (20U)

#define ETHERCAT_MASTER_NOTIFY_LINK_DOWN (1UL << 0)


/*
 * SOEM过程数据映射区。
 * 当前所有从站均使用本文件定义的固定RxPDO/TxPDO结构，按SOEM最大从站数预留。
 */
static char IOmap[EC_MAXSLAVE *
                  (sizeof(PDO_Output) + sizeof(PDO_Input))];
uint8_t s_servo_enable_request = 0U;
control_state_t current_state;
/*
 * 现有运动模块使用1号从站的PDO。
 * 多轴扫描、配置和EtherCAT OP切换覆盖全部从站，但本次不扩展多轴运动控制。
 */
PDO_Output *output1s;
PDO_Input *input1s;

/*
 * 多轴PDO指针表，数组下标与SOEM从站号一致。
 * 0号保留，1..s_axis_count对应扫描到的各个驱动器。
 */
static PDO_Output *s_axis_outputs[EC_MAXSLAVE];
static PDO_Input *s_axis_inputs[EC_MAXSLAVE];
static uint16_t s_axis_count;

static ethercat_pdo_monitor_t s_pdo_monitor[EC_MAXSLAVE];
static int32 s_last_position[EC_MAXSLAVE];
static uint16 s_last_status_word[EC_MAXSLAVE];
static int8 s_last_mode[EC_MAXSLAVE];
static uint8_t s_has_last_sample[EC_MAXSLAVE];

extern motion_control_t s_control;
static TaskHandle_t monitor_log = NULL;

/* WKC 统计：expected 是理论期望值，last 是最近一次 PDO 返回值 */
static int s_expected_wkc;
static int s_last_wkc;

/*
 * 每个轴独立的CiA402使能运行数据，数组下标与SOEM从站号一致。
 * 不使用多维void指针数组，避免丢失PDO输入/输出的类型检查。
 */
static servo_enable_state_t s_enable_state[EC_MAXSLAVE];
static uint32_t s_enable_wait_count[EC_MAXSLAVE];
static int32 s_servo_enable_hold_pos[EC_MAXSLAVE];
static int8_t s_enable_result[EC_MAXSLAVE];

/* SOEM 主站任务入口：完成从站扫描、SV630 PDO/DC 配置，并请求进入 OP。 */
static void ethercat_master_scan_task(void *pvParameters);

/* 周期打印1号轴PDO状态。 */
static void ethercat_pdo_monitor_log_task(void *pvParameters);

static int ethercat_master_pdo_process_check(int wkc);

/* 保存全部从站的PDO映射指针；全部有效返回1，否则返回0。 */
static int ethercat_master_axis_pdo_bind(void);

/* 判断全部扫描到的从站是否处于指定EtherCAT状态。 */
static uint8_t ethercat_master_all_slaves_in_state(uint16 state);

/* 使用GPT周期完成一次启动阶段PDO交换，返回实际WKC，失败返回-1。 */
static int ethercat_master_startup_pdo_exchange(void);

/* 在全部从站处于SAFE_OP时预热周期PDO，成功返回1，失败返回0。 */
static int ethercat_master_safe_op_warmup(void);

/* 请求指定从站进入OP并等待稳定，成功返回1，失败返回0。 */
static int ethercat_master_slave_request_op(uint16_t slave);

/* 按从站号逐轴请求OP，全部成功返回1，任一失败返回0。 */
static int ethercat_master_request_op_sequential(void);

/* 对指定轴执行一次CiA402使能状态机。 */
static int ethercat_servo_enable_process(uint16_t slave, int8 op_mode);

/* 对全部已绑定轴执行一次CiA402使能状态机。 */
static int ethercat_servo_all_axes_enable_process(int8 op_mode);

/*
 * SDO 写封装函数。
 * 用于在 PRE-OP 到 SAFE-OP 阶段配置 PDO 映射对象，例如 0x1600、0x1A00、0x1C12、0x1C13。
 * 注意：SDO 是邮箱通信，不能放在 2ms PDO 周期任务中频繁调用。
 */
int write8(uint16 slave, uint16 index, uint8 subindex, int value) {
    uint8 temp = value;

    int rtn = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(temp), &temp, EC_TIMEOUTRXM);

    if (rtn == 0) {
        printf("[Axis %u][SDO8] write failed: index=0x%04x sub=0x%02x\r\n",
               (unsigned int) slave,
               (unsigned int) index,
               (unsigned int) subindex);
    } else if (DEBUG) {
        printf("[Axis %u][SDO8] index=0x%04x sub=0x%02x value=0x%02x OK\r\n",
               (unsigned int) slave,
               (unsigned int) index,
               (unsigned int) subindex,
               (unsigned int) temp);
    }
    return rtn;
}

int write16(uint16 slave, uint16 index, uint8 subindex, int value) {
    uint16 temp = value;

    int rtn = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(temp), &temp, EC_TIMEOUTRXM * 20);
    // int rtn = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(temp), &temp, EC_TIMEOUTRXM);

    if (rtn == 0) {
        printf("[Axis %u][SDO16] write failed: index=0x%04x sub=0x%02x\r\n",
               (unsigned int) slave,
               (unsigned int) index,
               (unsigned int) subindex);
    } else if (DEBUG) {
        printf("[Axis %u][SDO16] index=0x%04x sub=0x%02x value=0x%04x OK\r\n",
               (unsigned int) slave,
               (unsigned int) index,
               (unsigned int) subindex,
               (unsigned int) temp);
    }
    return rtn;
}

int write32(uint16 slave, uint16 index, uint8 subindex, int value) {
    uint32 temp = value;

    // int rtn = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(temp), &temp, EC_TIMEOUTRXM * 20);
    int rtn = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(temp), &temp, EC_TIMEOUTRXM);
    if (rtn == 0) {
        printf("[Axis %u][SDO32] write failed: index=0x%04x sub=0x%02x\r\n",
               (unsigned int) slave,
               (unsigned int) index,
               (unsigned int) subindex);
    } else if (DEBUG) {
        printf("[Axis %u][SDO32] index=0x%04x sub=0x%02x value=0x%08lx OK\r\n",
               (unsigned int) slave,
               (unsigned int) index,
               (unsigned int) subindex,
               (unsigned long) temp);
    }
    return rtn;
}

int write16_test(uint16 slave, uint16 index, uint8 subindex, int value) {
    uint16 temp = (uint16) value;
    TickType_t start_tick;
    TickType_t end_tick;
    uint32_t elapsed_ms;
    int rtn;

    printf("[SDO16] enter slave=%u index=0x%04x sub=0x%02x value=0x%04x\r\n",
           (unsigned int) slave,
           (unsigned int) index,
           (unsigned int) subindex,
           (unsigned int) temp);

    start_tick = xTaskGetTickCount();

    rtn = ec_SDOwrite(slave,
                      index,
                      subindex,
                      FALSE,
                      sizeof(temp),
                      &temp,
                      EC_TIMEOUTRXM);

    end_tick = xTaskGetTickCount();
    elapsed_ms =
            ((uint32_t) (end_tick - start_tick) * 1000U) /
            (uint32_t) configTICK_RATE_HZ;

    printf("[SDO16] return rtn=%d elapsed=%lu ms\r\n",
           rtn,
           (unsigned long) elapsed_ms);

    if (rtn <= 0) {
        printf("[SDO16] FAILED\r\n");

        while (ec_iserror()) {
            printf("[SDO16] SOEM error: %s\r\n", ec_elist2string());
        }
    } else {
        printf("[SDO16] OK wkc=%d\r\n", rtn);
    }

    return rtn;
}

// PDO配置
static int Servosetup(uint16 slvcnt) {
    printf("[Axis %u][PDO] configuring RxPDO/TxPDO mapping\r\n",
           (unsigned int) slvcnt);
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

/*
 * 保存全部从站的PDO映射指针。
 * 参数：无，使用ec_slavecount和ec_slave[]。
 * 返回值：全部绑定成功返回1；数量或任一PDO指针无效返回0。
 */
static int ethercat_master_axis_pdo_bind(void) {
    uint16_t slave;

    s_axis_count = 0U;
    output1s = NULL;
    input1s = NULL;

    for (slave = 0U; slave < (uint16_t) EC_MAXSLAVE; slave++) {
        s_axis_outputs[slave] = NULL;
        s_axis_inputs[slave] = NULL;
        s_enable_state[slave] = SERVO_ENABLE_IDLE;
        s_enable_wait_count[slave] = 0U;
        s_servo_enable_hold_pos[slave] = 0;
        s_enable_result[slave] = 0;
        s_last_position[slave] = 0;
        s_last_status_word[slave] = 0U;
        s_last_mode[slave] = 0;
        s_has_last_sample[slave] = 0U;
        s_pdo_monitor[slave] = (ethercat_pdo_monitor_t){0};
    }

    if ((ec_slavecount <= 0) || (ec_slavecount >= EC_MAXSLAVE)) {
        return 0;
    }

    for (slave = 1U; slave <= (uint16_t) ec_slavecount; slave++) {
        s_axis_outputs[slave] = (PDO_Output *) ec_slave[slave].outputs;
        s_axis_inputs[slave] = (PDO_Input *) ec_slave[slave].inputs;

        if ((s_axis_outputs[slave] == NULL) ||
            (s_axis_inputs[slave] == NULL)) {
            printf("[Axis %u][PDO] invalid mapping: outputs=%p inputs=%p\r\n",
                   (unsigned int) slave,
                   (void *) s_axis_outputs[slave],
                   (void *) s_axis_inputs[slave]);
            return 0;
        }

        s_axis_outputs[slave]->ControlWord = CIA402_CW_DISABLE_VOLTAGE;
        s_axis_outputs[slave]->TargetVelocity = 0;
        s_axis_outputs[slave]->OpModeSet = 8;
        s_axis_outputs[slave]->TouchProbe = 0U;
    }

    s_axis_count = (uint16_t) ec_slavecount;
    output1s = s_axis_outputs[1];
    input1s = s_axis_inputs[1];
    return 1;
}

/*
 * 获取已经绑定的轴数量。
 * 参数：无。
 * 返回值：成功绑定的轴数量；尚未绑定时返回0。
 */
uint16_t ethercat_master_axis_count_get(void) {
    return s_axis_count;
}

/*
 * 获取指定轴的RxPDO输出区。
 * 参数slave：SOEM从站号，合法范围为1..ethercat_master_axis_count_get()。
 * 返回值：对应PDO输出指针；编号越界或尚未绑定时返回NULL。
 */
PDO_Output *ethercat_master_axis_output_get(uint16_t slave) {
    if ((slave == 0U) || (slave > s_axis_count)) {
        return NULL;
    }

    return s_axis_outputs[slave];
}

/*
 * 获取指定轴的TxPDO输入区。
 * 参数slave：SOEM从站号，合法范围为1..ethercat_master_axis_count_get()。
 * 返回值：对应PDO输入指针；编号越界或尚未绑定时返回NULL。
 */
PDO_Input *ethercat_master_axis_input_get(uint16_t slave) {
    if ((slave == 0U) || (slave > s_axis_count)) {
        return NULL;
    }

    return s_axis_inputs[slave];
}

/*
 * 查询指定轴是否已经进入CiA402 Operation Enabled。
 * 参数slave：SOEM从站号，范围为1..s_axis_count。
 * 返回值：1表示已使能；0表示未使能；-1表示编号或PDO指针无效。
 */
int ethercat_master_axis_operation_enabled_get(uint16_t slave) {
    PDO_Input *input = ethercat_master_axis_input_get(slave);

    if (input == NULL) {
        return -1;
    }

    return (((input->StatusWord & CIA402_SW_MASK) ==
             CIA402_SW_OPERATION_ENABLED)
                ? 1
                : 0);
}

/*
 * 查询全部轴是否已经进入CiA402 Operation Enabled。
 * 参数：无。
 * 返回值：1表示全部使能；0表示至少一轴未使能；-1表示尚未绑定轴。
 */
int ethercat_master_all_axes_operation_enabled_get(void) {
    uint16_t slave;

    if (s_axis_count == 0U) {
        return -1;
    }

    for (slave = 1U; slave <= s_axis_count; slave++) {
        if (ethercat_master_axis_operation_enabled_get(slave) != 1) {
            return 0;
        }
    }

    return 1;
}

/* 全部从站均处于指定EtherCAT状态时返回1，否则返回0。 */
static uint8_t ethercat_master_all_slaves_in_state(uint16 state) {
    uint16 slave;

    if ((ec_slavecount <= 0) ||
        (ec_slavecount >= EC_MAXSLAVE)) {
        return 0U;
    }

    for (slave = 1U;
         slave <= (uint16_t) ec_slavecount;
         slave++) {
        if (ec_slave[slave].state != state) {
            return 0U;
        }
    }

    return 1U;
}

/*
 * 使用GPT信号量完成一次启动阶段PDO交换。
 * 参数：无。
 * 返回值：ec_receive_processdata()返回的实际WKC；GPT未启动或等待超时返回-1。
 */
static int ethercat_master_startup_pdo_exchange(void) {
    int wkc;

    if (s_gpt_cycle_semaphore == NULL) {
        return -1;
    }

    if (xSemaphoreTake(s_gpt_cycle_semaphore,
                       pdMS_TO_TICKS(ETHERCAT_GPT_WAIT_MS)) != pdTRUE) {
        return -1;
    }

    (void) ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    s_last_wkc = wkc;

    if (wkc > 0) {
        gpt_dc_sync_adjust(ec_DCtime);
    }

    return wkc;
}

/*
 * 在SAFE_OP阶段按2 ms周期预热PDO。
 * 参数：无。
 * 返回值：连续完成预热返回1；GPT等待失败或WKC连续为0返回0。
 */
static int ethercat_master_safe_op_warmup(void) {
    uint32_t cycle;
    uint32_t zero_wkc_count = 0U;
    uint16_t slave;
    int wkc;

    for (cycle = 0U; cycle < ETHERCAT_SAFE_OP_WARMUP_CYCLES; cycle++) {
        wkc = ethercat_master_startup_pdo_exchange();
        if (wkc < 0) {
            return 0;
        }

        if (wkc == 0) {
            zero_wkc_count++;
            if (zero_wkc_count > ETHERCAT_ZERO_WKC_LIMIT) {
                return 0;
            }
        } else {
            zero_wkc_count = 0U;
        }
    }

    /*
     * SAFE_OP下已经取得有效输入，把每个轴的当前位置作为后续使能前目标值，
     * 避免目标位置保持为IOmap初始化时的0。
     */
    for (slave = 1U; slave <= s_axis_count; slave++) {
        s_servo_enable_hold_pos[slave] =
                s_axis_inputs[slave]->CurrentPosition;
        s_axis_outputs[slave]->TargetPos =
                s_servo_enable_hold_pos[slave];
    }

    return 1;
}

/*
 * 请求指定从站进入EtherCAT OP，并在GPT周期PDO不中断的条件下等待稳定。
 * 参数slave：SOEM从站号，范围为1..s_axis_count。
 * 返回值：该轴进入OP并稳定返回1；参数错误、超时、AL错误或PDO异常返回0。
 */
static int ethercat_master_slave_request_op(uint16_t slave) {
    uint32_t cycle;
    uint32_t stable_count = 0U;
    uint32_t zero_wkc_count = 0U;
    uint16 actual_state = 0U;
    uint8_t state_confirmed = 0U;
    int wkc;
    int wkc_ready;

    if ((slave == 0U) || (slave > s_axis_count)) {
        return 0;
    }

    ec_slave[slave].state = EC_STATE_OPERATIONAL;
    (void) ec_writestate(slave);

    for (cycle = 0U; cycle < ETHERCAT_AXIS_OP_TIMEOUT_CYCLES; cycle++) {
        wkc = ethercat_master_startup_pdo_exchange();
        if (wkc < 0) {
            return 0;
        }

        if (wkc == 0) {
            zero_wkc_count++;
            stable_count = 0U;
            if (zero_wkc_count > ETHERCAT_ZERO_WKC_LIMIT) {
                return 0;
            }
        } else {
            zero_wkc_count = 0U;
        }

        /*
         * 状态寄存器低频读取，避免在每个2 ms周期执行阻塞FPRD。
         * ec_statecheck(..., 0)仍会执行一次实际状态读取，并刷新AL状态码。
         */
        if ((cycle % ETHERCAT_STATE_CHECK_INTERVAL) == 0U) {
            actual_state = ec_statecheck(slave,
                                         EC_STATE_OPERATIONAL,
                                         0);
            state_confirmed =
            ((actual_state == EC_STATE_OPERATIONAL) &&
             (ec_slave[slave].state == EC_STATE_OPERATIONAL))
                ? 1U
                : 0U;

            if ((ec_slave[slave].state & EC_STATE_ERROR) != 0U) {
                return 0;
            }
        }

        /*
         * 中间轴尚有从站处于SAFE_OP，实际WKC不等于最终WKC。
         * 最后一轴进入时才要求完整WKC，之前只要求过程数据帧有效。
         */
        if (slave == s_axis_count) {
            wkc_ready = ((s_expected_wkc > 0) &&
                         (wkc >= s_expected_wkc));
        } else {
            wkc_ready = (wkc > 0);
        }

        if ((state_confirmed != 0U) && (wkc_ready != 0)) {
            stable_count++;
        } else {
            stable_count = 0U;
        }

        if (stable_count >= ETHERCAT_AXIS_OP_STABLE_CYCLES) {
            actual_state = ec_statecheck(slave,
                                         EC_STATE_OPERATIONAL,
                                         0);
            if ((actual_state == EC_STATE_OPERATIONAL) &&
                (ec_slave[slave].state == EC_STATE_OPERATIONAL)) {
                return 1;
            }

            stable_count = 0U;
            state_confirmed = 0U;
        }
    }

    return 0;
}

/*
 * 按SOEM从站号顺序逐轴请求EtherCAT OP。
 * 参数：无。
 * 返回值：全部轴进入OP并通过最终状态检查返回1；任一轴失败返回0。
 */
static int ethercat_master_request_op_sequential(void) {
    uint16_t slave;

    for (slave = 1U; slave <= s_axis_count; slave++) {
        if (ethercat_master_slave_request_op(slave) == 0) {
            return 0;
        }
    }

    ec_readstate();
    return (int) ethercat_master_all_slaves_in_state(
        EC_STATE_OPERATIONAL);
}

void ecat_init(void) {
    uint16 slc;
    /*
     * 第 1 步：初始化 SOEM 网卡接口。
     * ec_init() 会调用本工程的 nicdrv/oshw 适配层，把 SOEM 主站绑定到 EtherCAT 网口。
     */
    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ETHERCAT_MASTER_IFNAME)) {
        USR_LOG_INFO("[Master] ec_init succeeded.");
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
                    printf("[Axis %u][Scan] name=%s vendor=0x%08lx product=0x%08lx configadr=%u\r\n",
                           (unsigned int) slc,
                           ec_slave[slc].name,
                           (unsigned long) ec_slave[slc].eep_man,
                           (unsigned long) ec_slave[slc].eep_id,
                           (unsigned int) ec_slave[slc].configadr);
                    if ((ec_slave[slc].eep_man == 0x100000) && (ec_slave[slc].eep_id == 0xc0112))
                        ec_slave[slc].PO2SOconfig = &Servosetup;
                    //					else
                    //						USR_LOG_INFO("NULL");
                }
            }

            /*
             * 第 4 步：配置分布式时钟 DC 与 SYNC0。
             * 对扫描到的每个从站开启SYNC0，周期为ETHERCAT_DC_SYNC0_CYCLE_NS，即2 ms。
             */
            ec_configdc();
            for (slc = 1; slc <= ec_slavecount; slc++)
                ec_dcsync0(slc,TRUE,ETHERCAT_DC_SYNC0_CYCLE_NS, 0);
            /*
             * 第 5 步：建立 PDO 过程数据映射。
             * ec_config_map() 会生成 IOmap，并把 ec_slave[x].outputs / inputs 指向对应 PDO 区域。
             */
            ec_config_map(&IOmap);
            printf("[Master] PDO mapping completed; requesting SAFE_OP\r\n");
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
            printf("[Master] aggregate_state=0x%04x\r\n",
                   (unsigned int) ec_slave[0].state);
            for (slc = 1U; slc <= (uint16) ec_slavecount; slc++) {
                printf("[Axis %u][EtherCAT] state=0x%04x\r\n",
                       (unsigned int) slc,
                       (unsigned int) ec_slave[slc].state);
            }
            printf("[Master] segments=%u sizes=%lu/%lu/%lu/%lu\r\n",
                   (unsigned int) ec_group[0].nsegments,
                   (unsigned long) ec_group[0].IOsegment[0],
                   (unsigned long) ec_group[0].IOsegment[1],
                   (unsigned long) ec_group[0].IOsegment[2],
                   (unsigned long) ec_group[0].IOsegment[3]);
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

            /*
             * 第 8 步：计算整个从站组的期望WKC。
             * 多轴时不能再使用单轴固定值3。
             */
            printf("[Master] requesting EtherCAT OP sequentially\r\n");
            ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_OP_REQUESTING);
            s_expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("[Master] expected WKC=%d for %d axes\r\n",
                   s_expected_wkc,
                   ec_slavecount);

            /* 保存每个轴的inputs/outputs，数组下标与SOEM从站号一致。 */
            if (0 == ethercat_master_axis_pdo_bind()) {
                printf("[Master] one or more axis PDO pointers are invalid\r\n");
                ethercat_app_master_scan_set_state(ETHERCAT_MASTER_SCAN_STATE_FAILED, 0);
                ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_FAILED);
                return;
            }

            /*
             * 第9步：在SAFE_OP阶段先启动2 ms GPT并预热PDO，然后按轴号逐个请求OP。
             * 任一轴进入OP后，等待下一轴期间仍由同一GPT节拍持续交换全部PDO。
             */
            if (gpt_init() != FSP_SUCCESS) {
                USR_LOG_ERROR("[Master] GPT init failed.");
                ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_FAILED);
                return;
            }

            if ((ethercat_master_safe_op_warmup() != 0) &&
                (ethercat_master_request_op_sequential() != 0)) {
                /*
                 * 第10步：全部从站均确认OP后切换到正常PDO处理。
                 * 同一个GPT周期继续运行，随后逐轴执行CiA402使能，但不启动运动轨迹。
                 */
                ethercat_app_master_scan_set_state(ETHERCAT_MASTER_SCAN_STATE_DONE, ec_slavecount);
                ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_OPERATIONAL);

                ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_PDO_RUNNING);
                // xTaskCreate(ethercat_pdo_monitor_log_task,
                //             "PDO monitor",
                //             1024U / sizeof(StackType_t),
                //             NULL,
                //             tskIDLE_PRIORITY + 1U,
                //             NULL);
            } else {
                (void) gpt_stop();
                ec_readstate();
                for (slc = 1; slc <= ec_slavecount; slc++) {
                    if (ec_slave[slc].state != EC_STATE_OPERATIONAL) {
                        printf("[Axis %u][EtherCAT] failed to reach OP: state=0x%04x "
                               "ALstatus=0x%04x (%s)\r\n",
                               (unsigned int) slc,
                               (unsigned int) ec_slave[slc].state,
                               (unsigned int) ec_slave[slc].ALstatuscode,
                               ec_ALstatuscode2string(ec_slave[slc].ALstatuscode));
                    }
                }
                ethercat_app_master_scan_set_state(ETHERCAT_MASTER_SCAN_STATE_DONE, ec_slavecount);
                ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_FAILED);
                return;
            }
        } else {
            printf("[Master] no EtherCAT axes found\r\n");
            ethercat_app_master_scan_set_state(ETHERCAT_MASTER_SCAN_STATE_FAILED, 0);
            ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_FAILED);
            return;
        }
    } else {
        printf("[Master] EtherCAT network interface initialization failed\r\n");
        ethercat_app_master_scan_set_state(ETHERCAT_MASTER_SCAN_STATE_FAILED, 0);
        ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_FAILED);
        return;
    }
}

/**
 * @brief 通知SOEM任务物理链路已经断开。
 * @param 无。
 * @return 无。
 */
void ethercat_master_link_down_notify(void) {
    ethercat_app_notify_t *p_notify;
    TaskHandle_t master_task;

    p_notify = ethercat_app_notify_get();

    taskENTER_CRITICAL();

    master_task = p_notify->master_scan_task;
    if (master_task != NULL) {
        (void) xTaskNotify(master_task,
                           ETHERCAT_MASTER_NOTIFY_LINK_DOWN,
                           eSetBits);
    }

    taskEXIT_CRITICAL();
}


/* 创建 SOEM 主站任务。任务已经存在时直接返回成功，防止链路抖动导致重复创建。 */
usr_err_t ethercat_master_scan_start(void) {
    usr_err_t usr_err = ethercat_app_common_open();

    if (USR_SUCCESS != usr_err) {
        return usr_err;
    }
    ethercat_app_notify_t *p_notify = ethercat_app_notify_get();
    if (NULL != p_notify->master_scan_task) {
        return USR_ERR_ALREADY_RUNNING;
    }
    ethercat_app_master_scan_set_state(ETHERCAT_MASTER_SCAN_STATE_RUNNING, 0);
    ethercat_app_master_run_set_state(ETHERCAT_MASTER_RUN_STATE_SCANNING);

    if (pdPASS != xTaskCreate(ethercat_master_scan_task,
                              ETHERCAT_MASTER_TASK_NAME,
                              ETHERCAT_MASTER_TASK_STACK_SIZE / sizeof(StackType_t),
                              NULL,
                              ETHERCAT_MASTER_TASK_PRIORITY,
                              &p_notify->master_scan_task)) {
        USR_LOG_ERROR("[Master] SOEM task creation failed.");
        ethercat_app_master_scan_set_state(ETHERCAT_MASTER_SCAN_STATE_FAILED, 0);
        return USR_ERR_NOT_INITIALIZED;
    }

    return USR_SUCCESS;
}


static void ethercat_master_scan_task(void *pvParameters) {
    (void) pvParameters;
    USR_LOG_INFO("[Master] SOEM start on %s.", ETHERCAT_MASTER_IFNAME);
    ecat_init();

    if (ethercat_app_master_run_get_state() != ETHERCAT_MASTER_RUN_STATE_PDO_RUNNING) {
        USR_LOG_ERROR("[Master] stopped before PDO cycle because not all axes reached OP.");
        vTaskDelete(NULL);
        return;
    }
    ethercat_motion_motor_params_set(262144, 5, 1, 1, 3000);

    for (;;) {
        // xSemaphoreTake(s_gpt_cycle_semaphore, portMAX_DELAY);

        uint32_t notify_value = 0U;

        if ((xTaskNotifyWait(0U,
                             UINT32_MAX,
                             &notify_value,
                             0U) == pdTRUE) &&
            ((notify_value &
              ETHERCAT_MASTER_NOTIFY_LINK_DOWN) != 0U)) {
            USR_LOG_WARN("[Master] Link Down, stop SOEM.");
            break;
        }

        /*
         * 使用有限等待，防止GPT意外停止后任务永远无法退出。
         */
        if (xSemaphoreTake(s_gpt_cycle_semaphore,
                           pdMS_TO_TICKS(ETHERCAT_GPT_WAIT_MS)) != pdTRUE) {
            continue;
        }

        /* 1. 发送上一周期已经准备好的目标值 */
        (void) ec_send_processdata();

        /* 2. 接收本周期反馈 */
        int wkc = ec_receive_processdata(EC_TIMEOUTRET);

        /* 3. 检查整个从站组的PDO状态。 */
        int pdo_ok = ethercat_master_pdo_process_check(wkc);
        /* 仅在PDO通信正常时调整GPT周期 */
        if (pdo_ok != 0) {
            gpt_dc_sync_adjust(ec_DCtime);
        }

        if (current_state == SET_0RIGIN) {
            continue;
        }
        if (pdo_ok != 0) {
            if (s_servo_enable_request != 0U) {
                /*
                 * 请求进入或保持Operation Enabled。
                 */
                /*
                 * 先逐轴推进CiA402使能状态机，使能期间由状态机保持
                * 各轴当前位置，避免目标位置从0或旧轨迹值突跳。
                */
                (void) ethercat_servo_all_axes_enable_process(8);
                /*
                 * 当前运动模块只绑定1号轴。确认1号轴已进入
                 * Operation Enabled后再推进轨迹，生成下一PDO周期的0x607A。
                 */
                if (ethercat_master_axis_operation_enabled_get(1U) == 1) {
                    ethercat_motion_process();
                }
            } else {
                uint16_t status_state = input1s->StatusWord & CIA402_SW_MASK;
                /*
                 * 调用该模式前必须保证轨迹已经停止。
                 * 保持目标位置与实际位置一致，防止残留目标。
                 */
                output1s->TargetPos = input1s->CurrentPosition;
                output1s->TargetVelocity = 0;
                /*
                 * 请求Ready To Switch On。
                 */
                output1s->ControlWord = CIA402_CW_SHUTDOWN; /* 0x0006 */

                if (status_state == CIA402_SW_READY_TO_SWITCH_ON) {
                    /*
                     * 为下一次重新进入使能准备完整状态机。
                     */
                    s_enable_state[1U] = SERVO_ENABLE_IDLE;
                    s_enable_wait_count[1U] = 0U;
                }
            }
        }
    }
    ethercat_app_notify_t *p_notify;

    ethercat_app_master_run_set_state(
        ETHERCAT_MASTER_RUN_STATE_STOPPING);

    s_servo_enable_request = 0U;
    current_state = IDLE_STATE;

    (void) gpt_stop();
    ec_close();

    taskENTER_CRITICAL();

    s_axis_count = 0U;
    input1s = NULL;
    output1s = NULL;
    s_expected_wkc = 0;
    s_last_wkc = 0;

    p_notify = ethercat_app_notify_get();
    p_notify->master_scan_task = NULL;

    taskEXIT_CRITICAL();

    ethercat_app_master_status_update(0, 0, 0U, 0U, 0U);
    ethercat_app_master_scan_set_state(
        ETHERCAT_MASTER_SCAN_STATE_IDLE,
        0);
    ethercat_app_master_run_set_state(
        ETHERCAT_MASTER_RUN_STATE_LINK_DOWN);

    vTaskDelete(NULL);
}

/************************** 伺服使能 **************************/
/*
 * 对指定轴执行一次CiA402使能状态机。
 * 参数slave：SOEM从站号，范围为1..s_axis_count。
 * 参数op_mode：运行模式，8表示CSP；0表示不修改模式。
 * 返回值：1表示Operation Enabled；0表示正在使能；-1表示失败或PDO无效。
 */
static int ethercat_servo_enable_process(uint16_t slave, int8 op_mode) {
    PDO_Input *input;
    PDO_Output *output;
    uint16 status_word;
    uint16 status_state;

    if ((slave == 0U) || (slave > s_axis_count)) {
        return -1;
    }

    input = s_axis_inputs[slave];
    output = s_axis_outputs[slave];
    if ((input == NULL) || (output == NULL)) {
        return -1;
    }

    status_word = input->StatusWord;
    status_state = status_word & CIA402_SW_MASK;

    if (op_mode != 0) {
        output->OpModeSet = op_mode;
    }

    /*
     * 驱动器已经使能时继续保持该轴锁定位置。
     * 首次发现已使能状态时先采用当前位置，避免目标值仍为0导致突跳。
     */
    if (status_state == CIA402_SW_OPERATION_ENABLED) {
        if (s_enable_state[slave] != SERVO_ENABLE_DONE) {
            s_servo_enable_hold_pos[slave] = input->CurrentPosition;
        }

        output->TargetPos = s_servo_enable_hold_pos[slave];
        output->TargetVelocity = 0;
        output->ControlWord = CIA402_CW_ENABLE_OPERATION;

        if ((op_mode != 0) && (input->OpModeNow != op_mode)) {
            return 0;
        }

        s_enable_state[slave] = SERVO_ENABLE_DONE;
        s_enable_wait_count[slave] = 0U;
        return 1;
    }

    switch (s_enable_state[slave]) {
        case SERVO_ENABLE_IDLE:
            s_enable_wait_count[slave] = 0U;
            output->ControlWord = CIA402_CW_DISABLE_VOLTAGE;

            if ((status_word & CIA402_SW_FAULT_MASK) == CIA402_SW_FAULT) {
                s_enable_state[slave] = SERVO_ENABLE_FAULT_RESET_PULSE;
            } else {
                s_enable_state[slave] = SERVO_ENABLE_SHUTDOWN;
            }
            break;

        case SERVO_ENABLE_FAULT_RESET_PULSE:
            /* Fault Reset只保持一个PDO周期。 */
            output->ControlWord = CIA402_CW_FAULT_RESET;
            s_enable_state[slave] = SERVO_ENABLE_WAIT_FAULT_CLEAR;
            s_enable_wait_count[slave] = 0U;
            break;

        case SERVO_ENABLE_WAIT_FAULT_CLEAR:
            output->ControlWord = CIA402_CW_DISABLE_VOLTAGE;

            if ((status_word & CIA402_SW_FAULT_MASK) != CIA402_SW_FAULT) {
                s_enable_state[slave] = SERVO_ENABLE_SHUTDOWN;
                s_enable_wait_count[slave] = 0U;
            } else if (++s_enable_wait_count[slave] > 500U) {
                s_enable_state[slave] = SERVO_ENABLE_FAILED;
                return -1;
            }
            break;

        case SERVO_ENABLE_SHUTDOWN:
            output->ControlWord = CIA402_CW_SHUTDOWN;

            if (status_state == CIA402_SW_READY_TO_SWITCH_ON) {
                s_enable_state[slave] = SERVO_ENABLE_SWITCH_ON;
                s_enable_wait_count[slave] = 0U;
            } else if (++s_enable_wait_count[slave] > 1000U) {
                s_enable_state[slave] = SERVO_ENABLE_FAILED;
                return -1;
            }
            break;

        case SERVO_ENABLE_SWITCH_ON:
            output->ControlWord = CIA402_CW_SWITCH_ON;

            if (status_state == CIA402_SW_SWITCHED_ON) {
                s_servo_enable_hold_pos[slave] = input->CurrentPosition;
                output->TargetPos = s_servo_enable_hold_pos[slave];
                output->TargetVelocity = 0;
                if (op_mode != 0) {
                    output->OpModeSet = op_mode;
                }

                s_enable_wait_count[slave] = 0U;
                s_enable_state[slave] = SERVO_ENABLE_CSP_PREPARE;
            } else if (++s_enable_wait_count[slave] > 1000U) {
                s_enable_state[slave] = SERVO_ENABLE_FAILED;
                return -1;
            }
            break;

        case SERVO_ENABLE_CSP_PREPARE:
            /*
             * 保持0x0007并连续发送当前位置，确认CSP模式后再输出0x000F。
             */
            output->ControlWord = CIA402_CW_SWITCH_ON;
            if (op_mode != 0) {
                output->OpModeSet = op_mode;
            }
            s_servo_enable_hold_pos[slave] = input->CurrentPosition;
            output->TargetPos = s_servo_enable_hold_pos[slave];
            output->TargetVelocity = 0;
            s_enable_wait_count[slave]++;

            if (((op_mode == 0) || (input->OpModeNow == op_mode)) &&
                (s_enable_wait_count[slave] >= 5U)) {
                s_enable_wait_count[slave] = 0U;
                s_enable_state[slave] = SERVO_ENABLE_ENABLE_OPERATION;
            } else if (s_enable_wait_count[slave] > 1000U) {
                s_enable_state[slave] = SERVO_ENABLE_FAILED;
                return -1;
            }
            break;

        case SERVO_ENABLE_ENABLE_OPERATION:
            output->TargetPos = s_servo_enable_hold_pos[slave];
            output->TargetVelocity = 0;
            output->ControlWord = CIA402_CW_ENABLE_OPERATION;

            if (++s_enable_wait_count[slave] > 1000U) {
                s_enable_state[slave] = SERVO_ENABLE_FAILED;
                return -1;
            }
            break;

        case SERVO_ENABLE_DONE:
            /*
             * 状态字已离开Operation Enabled，先撤销控制字并重新进入状态机。
             */
            output->ControlWord = CIA402_CW_DISABLE_VOLTAGE;
            s_enable_state[slave] = SERVO_ENABLE_IDLE;
            s_enable_wait_count[slave] = 0U;
            break;

        case SERVO_ENABLE_FAILED:
        default:
            output->ControlWord = CIA402_CW_DISABLE_VOLTAGE;
            return -1;
    }

    return 0;
}

/*
 * 推进全部轴的CiA402状态机。
 * 参数op_mode：写入所有轴的运行模式，8表示CSP。
 * 返回值：全部轴使能返回1；仍有轴使能中返回0；任一轴失败返回-1。
 */
static int ethercat_servo_all_axes_enable_process(int8 op_mode) {
    uint16_t slave;
    int result;
    int all_enabled = 1;
    int any_failed = 0;

    if (s_axis_count == 0U) {
        return -1;
    }

    for (slave = 1U; slave <= s_axis_count; slave++) {
        result = ethercat_servo_enable_process(slave, op_mode);
        s_enable_result[slave] = (int8_t) result;

        if (result < 0) {
            any_failed = 1;
        } else if (result == 0) {
            all_enabled = 0;
        }
    }

    if (any_failed != 0) {
        return -1;
    }

    return all_enabled;
}


/************************** 任务日志 **************************/
/*
 * 低优先级 PDO 监控日志任务。
 * 该任务每 1 秒打印一次缓存数据，不直接调用 SOEM 收发函数，避免影响 PDO 实时周期。
 */
static void ethercat_pdo_monitor_log_task(void *pvParameters) {
    ethercat_pdo_monitor_t mon;
    uint16_t slave;
    int8_t enable_result;
    servo_enable_state_t enable_state;

    (void) pvParameters;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        for (slave = 1U; slave <= s_axis_count; slave++) {
            taskENTER_CRITICAL();
            mon = s_pdo_monitor[slave];
            enable_state = s_enable_state[slave];
            enable_result = s_enable_result[slave];
            taskEXIT_CRITICAL();

            USR_LOG_INFO("[Axis %u][PDO] enable=%d step=%d ok=%d cyc=%lu good=%lu bad=%lu "
                         "wkc=%d/%d ec=0x%04x status=0x%04x ctrl=0x%04x "
                         "mode=%d pos=%ld dpos=%ld target=%ld.",
                         (unsigned int) slave,
                         (int) enable_result,
                         (int) enable_state,
                         mon.pdo_ok,
                         (unsigned long) mon.cycle_count,
                         (unsigned long) mon.good_count,
                         (unsigned long) mon.bad_count,
                         mon.wkc,
                         mon.expected_wkc,
                         mon.state,
                         mon.status_word,
                         mon.control_word,
                         mon.mode,
                         (long) mon.position,
                         (long) mon.position_delta,
                         (long) mon.target_pos);
        }
    }
}

int ethercat_master_pdo_process_check(int wkc) {
    int pdo_ok = 1;
    uint16_t slave;
    PDO_Input *input;
    PDO_Output *output;
    int32 position_delta;
    uint8_t had_last_sample;

    /* 保存最近一次 WKC，供其他任务读取 */
    s_last_wkc = wkc;

    /* 如果期望 WKC 还没有初始化，则根据 SOEM 的 group 信息计算 */
    if (s_expected_wkc <= 0) {
        s_expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    }

    if (s_axis_count == 0U) {
        return 0;
    }

    for (slave = 1U; slave <= s_axis_count; slave++) {
        if ((s_axis_inputs[slave] == NULL) ||
            (s_axis_outputs[slave] == NULL)) {
            pdo_ok = 0;
            break;
        }
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
    if (0U == ethercat_master_all_slaves_in_state(EC_STATE_OPERATIONAL)) {
        pdo_ok = 0;
    }

    for (slave = 1U; slave <= s_axis_count; slave++) {
        input = s_axis_inputs[slave];
        output = s_axis_outputs[slave];
        if ((input == NULL) || (output == NULL)) {
            continue;
        }

        position_delta = 0;
        had_last_sample = s_has_last_sample[slave];
        if (had_last_sample != 0U) {
            position_delta =
                    input->CurrentPosition - s_last_position[slave];
        } else {
            s_has_last_sample[slave] = 1U;
        }

        taskENTER_CRITICAL();
        s_pdo_monitor[slave].cycle_count++;
        s_pdo_monitor[slave].wkc = wkc;
        s_pdo_monitor[slave].expected_wkc = s_expected_wkc;
        s_pdo_monitor[slave].pdo_ok = pdo_ok;

        if (pdo_ok != 0) {
            s_pdo_monitor[slave].good_count++;
        } else {
            s_pdo_monitor[slave].bad_count++;
        }

        if ((had_last_sample != 0U) &&
            (input->CurrentPosition == s_last_position[slave]) &&
            (input->StatusWord == s_last_status_word[slave]) &&
            (input->OpModeNow == s_last_mode[slave])) {
            s_pdo_monitor[slave].unchanged_count++;
        } else {
            s_pdo_monitor[slave].unchanged_count = 0U;
        }

        s_pdo_monitor[slave].state = ec_slave[slave].state;
        s_pdo_monitor[slave].status_word = input->StatusWord;
        s_pdo_monitor[slave].position = input->CurrentPosition;
        s_pdo_monitor[slave].position_delta = position_delta;
        s_pdo_monitor[slave].mode = input->OpModeNow;
        s_pdo_monitor[slave].control_word = output->ControlWord;
        s_pdo_monitor[slave].target_pos = output->TargetPos;
        taskEXIT_CRITICAL();

        s_last_position[slave] = input->CurrentPosition;
        s_last_status_word[slave] = input->StatusWord;
        s_last_mode[slave] = input->OpModeNow;
    }

    /* 公共状态接口继续使用1号轴，保持现有上位机接口兼容。 */
    if ((s_axis_inputs[1] != NULL) && (s_axis_outputs[1] != NULL)) {
        ethercat_app_master_status_update(wkc,
                                          s_expected_wkc,
                                          ec_slave[1].state,
                                          s_axis_inputs[1]->StatusWord,
                                          s_axis_outputs[1]->ControlWord);
    }

    return pdo_ok;
}


uint8_t servo_enable_allowed(void) {
    ethercat_motion_status_t status;
    ethercat_motion_status_get(&status);
    if (ethercat_app_master_run_get_state() !=
        ETHERCAT_MASTER_RUN_STATE_PDO_RUNNING) {
        return 2;
    }
    if ((input1s->StatusWord & CIA402_SW_MASK) == CIA402_SW_READY_TO_SWITCH_ON && s_servo_enable_request == 0) {
        ethercat_motion_position_sync();
        s_servo_enable_request = 1;
    } else if ((input1s->StatusWord & CIA402_SW_MASK) == CIA402_SW_OPERATION_ENABLED && s_servo_enable_request == 1 &&
               status.busy == 0 && status.done == 1) {
        s_servo_enable_request = 0;
    }
    return s_servo_enable_request;
}
