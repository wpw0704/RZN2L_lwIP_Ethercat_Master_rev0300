#ifndef ETHERCAT_MASTER_H
#define ETHERCAT_MASTER_H

#include "um_common.h"
#include "ethercat_app_common.h"
#include "ethercat_port_cfg.h"
#include "FreeRTOS.h"
#include "ethercat.h"
#include "semphr.h"
#include "task.h"
#include <stdint.h>
#include "ethercatprint.h"
#include "gpt.h"

/* CiA402 控制字 0x6040，主站通过 RxPDO 写给驱动器 */
#define CIA402_CW_DISABLE_VOLTAGE       (0x0000U)  // 关闭电压/释放使能
#define CIA402_CW_SHUTDOWN              (0x0006U)  // Shutdown，进入 Ready to switch on
#define CIA402_CW_SWITCH_ON             (0x0007U)  // Switch on，进入 Switched on
#define CIA402_CW_ENABLE_OPERATION      (0x000FU)  // Enable operation，进入运行使能
#define CIA402_CW_FAULT_RESET           (0x0080U)  // 故障复位脉冲

/* CiA402 状态字 0x6041 判断掩码，用于识别驱动器状态机阶段 */
#define CIA402_SW_MASK                  (0x006FU)  // 提取 CiA402 状态机低位状态
#define CIA402_SW_FAULT_MASK            (0x004FU)  // 判断 Fault 状态使用的掩码
#define CIA402_SW_FAULT                 (0x0008U)  // Fault 故障状态
#define CIA402_SW_SWITCH_ON_DISABLED    (0x0040U)  // Switch on disabled
#define CIA402_SW_READY_TO_SWITCH_ON    (0x0021U)  // Ready to switch on
#define CIA402_SW_SWITCHED_ON           (0x0023U)  // Switched on
#define CIA402_SW_OPERATION_ENABLED     (0x0027U)  // Operation enabled

typedef enum {
    SERVO_ENABLE_IDLE = 0,              // 初始状态，根据状态字决定走故障复位还是正常使能
    SERVO_ENABLE_FAULT_RESET_PULSE,     // 输出一次 0x0080 故障复位脉冲
    SERVO_ENABLE_WAIT_FAULT_CLEAR,      // 释放 0x0080，等待驱动器故障位清除
    SERVO_ENABLE_SHUTDOWN,              // 输出 0x0006，等待 Ready to switch on
    SERVO_ENABLE_SWITCH_ON,             // 输出 0x0007，等待 Switched on
    SERVO_ENABLE_ENABLE_OPERATION,      // 输出 0x000F，等待 Operation enabled
    SERVO_ENABLE_DONE,                  // 已使能，持续保持 0x000F
    SERVO_ENABLE_FAILED,                // 使能超时或失败
} servo_enable_state_t;

PACKED_BEGIN
typedef struct PACKED {
    uint16 ControlWord;      // 0x6040 控制字
    int32 TargetPos;         // 0x607A 目标位置
    int32 TargetVelocity;    // 0x60FF 目标速度
    int8 OpModeSet;          // 0x6060 模式设置，8 表示 CSP
    uint16 TouchProbe;       // 0x60B8 探针控制
} PDO_Output;

PACKED_END

PACKED_BEGIN
typedef struct PACKED {
    uint16 StatusWord;       // 0x6041 状态字
    int32 CurrentPosition;   // 0x6064 当前位置
    int8 OpModeNow;          // 0x6061 当前实际模式显示
    uint16 TouchProbeStatus; // 0x60B9 探针状态
    int32 TouchProbePos1;    // 0x60BA 探针位置 1
    uint32 Digitalinputs;    // 0x60FD 数字输入
} PDO_Input;

PACKED_END

typedef struct {
    uint32_t cycle_count;      // PDO 检测总周期计数，每调用一次检测函数加 1
    uint32_t good_count;       // PDO 正常周期累计次数
    uint32_t bad_count;        // PDO 异常周期累计次数
    uint32_t unchanged_count;  // 输入数据连续未变化的周期数，用于观察位置/状态是否一直不刷新

    int wkc;                  // 本次 ec_receive_processdata() 返回的实际 WKC
    int expected_wkc;         // 期望 WKC，通常为 outputsWKC * 2 + inputsWKC

    uint16 state;             // EtherCAT 主站记录的从站状态，0x0008 表示 OP
    uint16 status_word;       // 驱动器状态字 0x6041
    int32 position;           // 驱动器当前位置 0x6064
    int32 position_delta;     // 本次位置与上次位置的差值，用于判断位置是否变化
    int8 mode;                // 当前运行模式 0x6061

    uint16 control_word;      // 主站输出的控制字 0x6040
    int32 target_pos;         // 主站输出的目标位置 0x607A

    int pdo_ok;               // 本周期 PDO 检测结果，1 表示正常，0 表示异常
} ethercat_pdo_monitor_t;

extern PDO_Input *input1s;
extern PDO_Output *output1s;

usr_err_t ethercat_master_scan_start(void);
int ethercat_master_expected_wkc_get(void);
int ethercat_master_last_wkc_get(void);

#endif
