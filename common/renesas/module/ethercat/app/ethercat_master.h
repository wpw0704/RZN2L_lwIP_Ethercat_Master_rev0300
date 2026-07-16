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

#define CIA402_CW_DISABLE_VOLTAGE       (0x0000U)
#define CIA402_CW_SHUTDOWN              (0x0006U)
#define CIA402_CW_SWITCH_ON             (0x0007U)
#define CIA402_CW_ENABLE_OPERATION      (0x000FU)
#define CIA402_CW_FAULT_RESET           (0x0080U)

#define CIA402_SW_MASK                  (0x006FU)
#define CIA402_SW_FAULT_MASK            (0x004FU)
#define CIA402_SW_FAULT                 (0x0008U)
#define CIA402_SW_SWITCH_ON_DISABLED    (0x0040U)
#define CIA402_SW_READY_TO_SWITCH_ON    (0x0021U)
#define CIA402_SW_SWITCHED_ON           (0x0023U)
#define CIA402_SW_OPERATION_ENABLED     (0x0027U)

typedef enum {
    SERVO_ENABLE_IDLE = 0,
    SERVO_ENABLE_FAULT_RESET_PULSE,
    SERVO_ENABLE_WAIT_FAULT_CLEAR,
    SERVO_ENABLE_SHUTDOWN,
    SERVO_ENABLE_SWITCH_ON,
    SERVO_ENABLE_ENABLE_OPERATION,
    SERVO_ENABLE_DONE,
    SERVO_ENABLE_FAILED,
} servo_enable_state_t;

PACKED_BEGIN
typedef struct PACKED {
    uint16 ControlWord;
    int32 TargetPos;
    int32 TargetVelocity;
    int8 OpModeSet;
    uint16 TouchProbe;
    //		int8 temp;
} PDO_Output;

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
