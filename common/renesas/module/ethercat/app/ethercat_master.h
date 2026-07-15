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

extern PDO_Input *input1s;
extern PDO_Output *output1s;

usr_err_t ethercat_master_scan_start(void);
int ethercat_master_expected_wkc_get(void);
int ethercat_master_last_wkc_get(void);

#endif
