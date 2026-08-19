
#include "crc_L16.h"

/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/
static uint32_t crc_length;
static uint32_t crc_result;
static crc_input_t crc_input;

static fsp_err_t err;

/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/

/**
 * @brief  CRC初始化
 */
void crc_init(void)
{
    err = R_CRC_Open(&g_crc0_ctrl, &g_crc0_cfg);
    if (err != FSP_SUCCESS)
    {
        USR_LOG_ERROR("CRC_Open error : %d\r\n", err);
    }
}

/**
 * @brief  CRC计算
 * @param  p_data: 数据指针
 * @param  length: 数据长度
 * @return CRC值
 */
uint32_t crc_calc(uint8_t *p_data, uint32_t length)
{
    crc_input =
        (crc_input_t){
            .p_input_buffer = p_data,
            .num_bytes = length,
            .crc_seed = 0xffff,
        };
    R_CRC_Calculate(&g_crc0_ctrl, &crc_input, &crc_result);

    return crc_result;
}