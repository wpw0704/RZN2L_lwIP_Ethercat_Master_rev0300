#ifndef RENESAS_MODULE_MIO_IO_IO_H
#define RENESAS_MODULE_MIO_IO_IO_H

#include <um_common.h>
#include <hal_data.h>

#define HC165_PRINT_PERIOD_MS    (1000U)

#define SN595_DATA_COUNT        (24U)
#define SN165_DATA_COUNT        (24U)
#define SN595_START_INDEX       (2U)
#define SN595_CHANGE_COUNT      (5U)

void sn595_outdata(void);

// test
void sn595_data_update(void);

uint8_t sn595_data_set(const uint8_t index, const uint8_t data);

#endif
