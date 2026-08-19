#include <io.h>

/*
 *
 * 左侧为 24  右侧为 GND
 * 输出 1 有效为 NPN
 * 输出 0 有效为 PNP
sn595_data[2]	YOUT11	D36		OUT22---U15/Q5
sn595_data[3]	YOUT22	D22		OUT21---U15/Q4
sn595_data[4]	YOUT10	D28		OUT20---U15/Q3
sn595_data[5]	YOUT21	D14		OUT19---U15/Q2
sn595_data[6]	YOUT09	D20		OUT18---U15/Q1
sn595_data[7]	YOUT20	D50		OUT17---U15/Q0
sn595_data[8]	YOUT04	D35		OUT16---U14/Q7
sn595_data[9]	YOUT15	D13		OUT15---U14/Q6
sn595_data[10]	YOUT03	D27		OUT14---U14/Q5
sn595_data[11]	YOUT14	D54		OUT13---U14/Q4
sn595_data[12]	YOUT02	D19		OUT12---U14/Q3
sn595_data[13]	YOUT13	D49		OUT11---U14/Q2
sn595_data[14]	YOUT?	D?
sn595_data[15]	YOUT?	D?
sn595_data[16]	YOUT08	D12		OUT08---U13/Q7
sn595_data[17]	YOUT19	D44		OUT07---U13/Q6
sn595_data[18]	YOUT07	D53		OUT06---U13/Q5
sn595_data[19]	YOUT18	D37		OUT05---U13/Q4
sn595_data[20]	YOUT06	D48		OUT04---U13/Q3
sn595_data[21]	YOUT17	D29		OUT03---U13/Q2
sn595_data[22]	YOUT05	D42		OUT02---U13/Q1
sn595_data[23]	YOUT16	D21		OUT01---U13/Q0
 */
/* 初始状态全部为0 */
uint8_t sn595_data[SN595_DATA_COUNT] = {0};

void sn595_outdata(void) {
    R_IOPORT_PinWrite(&g_ioport_ctrl, SN595_CLK, BSP_IO_LEVEL_LOW);
    for (uint32_t i = 0; i < 24; i++) {
        R_IOPORT_PinWrite(&g_ioport_ctrl, SN595_DATA, (bsp_io_level_t) sn595_data[i]);
        R_IOPORT_PinWrite(&g_ioport_ctrl, SN595_CLK, BSP_IO_LEVEL_HIGH);
        R_IOPORT_PinWrite(&g_ioport_ctrl, SN595_CLK, BSP_IO_LEVEL_LOW);
    }
    R_IOPORT_PinWrite(&g_ioport_ctrl, SN595_CS, BSP_IO_LEVEL_HIGH);
    R_IOPORT_PinWrite(&g_ioport_ctrl, SN595_CS, BSP_IO_LEVEL_LOW);
}

/**
 * @brief  每累计5次调用，将有效位向后移动一个位置。
 *
 * 从sn595_data[2]开始移动到sn595_data[23]，
 * 到达最后一个元素后重新返回sn595_data[2]。
 *
 * @param  无
 * @return 无
 */
void sn595_data_update(void) {
    static uint32_t count = 0U;
    static uint32_t active_index = SN595_START_INDEX;

    count++;

    if (count >= SN595_CHANGE_COUNT) {
        count = 0U;

        /* 清除所有输出，保证同一时刻只有一个元素为1 */
        for (uint32_t index = 0U; index < SN595_DATA_COUNT; index++) {
            sn595_data[index] = 0U;
        }

        /* 设置当前有效位置 */
        sn595_data[active_index] = 1U;

        /* 移动到下一个元素 */
        active_index++;

        if (active_index >= SN595_DATA_COUNT) {
            active_index = SN595_START_INDEX;
        }
    }
}

uint8_t sn595_data_set(const uint8_t index, const uint8_t data) {
    if (index > 22) {
        return 2;
    }
    if (data > 1) {
        return 2;
    }
    switch (index) {
        case 0x01:
            sn595_data[14] = data;
            break;
        case 0x02:
            sn595_data[12] = data;
            break;
        case 0x03:
            sn595_data[10] = data;
            break;
        case 0x04:
            sn595_data[8] = data;
            break;
        case 0x05:
            sn595_data[22] = data;
            break;
        case 0x06:
            sn595_data[20] = data;
            break;
        case 0x07:
            sn595_data[18] = data;
            break;
        case 0x08:
            sn595_data[16] = data;
            break;
        case 0x09:
            sn595_data[17] = data;
            break;
        case 0x0A:
            sn595_data[4] = data;
            break;
        case 0x0B:
            sn595_data[2] = data;
            break;
        case 0x0C:
            sn595_data[15] = data;
            break;
        case 0x0D:
            sn595_data[13] = data;
            break;
        case 0x0E:
            sn595_data[11] = data;
            break;
        case 0x0F:
            sn595_data[9] = data;
            break;
        case 0x10:
            sn595_data[23] = data;
            break;
        case 0x11:
            sn595_data[21] = data;
            break;
        case 0x12:
            sn595_data[19] = data;
            break;
        case 0x13:
            sn595_data[17] = data;
            break;
        case 0x14:
            sn595_data[7] = data;
            break;
        case 0x15:
            sn595_data[5] = data;
            break;
        case 0x16:
            sn595_data[3] = data;
        default:
            break;
    }
    return data;
}
