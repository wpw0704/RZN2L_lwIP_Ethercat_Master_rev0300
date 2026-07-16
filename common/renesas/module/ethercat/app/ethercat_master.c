#include "ethercat_master.h"

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
/* IOmap buffer for EtherCAT process data */
static char IOmap[4096];
#pragma pack(push, 1)

#pragma pack(pop)
PDO_Output *output1s;
PDO_Input *input1s;

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
#if 0
            /* wait for all slaves to reach OP state */
            // do {
            //     for (slc = 0; slc <= ec_slavecount; slc++) {
            //         ec_slave[slc].state = EC_STATE_OPERATIONAL;
            //         ec_writestate(slc);
            //         printf("Slave %d State=0x%04x\r\n", slc, ec_slave[slc].state);
            //     }
            // } while ((ec_slave[0].state != EC_STATE_OPERATIONAL) || (ec_slave[1].state != EC_STATE_OPERATIONAL));
#endif
            int chk = 200;
            /* request OP state for all slaves */
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data before requesting OP */
            ec_send_processdata();                 // 先发一帧有效 PDO 输出
            ec_receive_processdata(EC_TIMEOUTRET); // 收一帧 PDO 输入，更新输入数据
            ec_writestate(0);                      // 请求所有从站切换到目标状态
            /* wait for all slaves to actually reach OP */
            do {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 2000);
            } while ((chk-- > 0) && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            ec_readstate();
            for (slc = 1; slc <= ec_slavecount; slc++) {
                printf("Slave %d State=0x%04x ALstatuscode=0x%04x\r\n",
                       slc,
                       ec_slave[slc].state,
                       ec_slave[slc].ALstatuscode);
            }

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
        ec_receive_processdata(EC_TIMEOUTRET);
    }
}
