#include "host_command.h"

#include "ethercat_motion.h"

/* 把运动模块返回值转换为上位机协议状态码。 */
static host_protocol_status_t host_command_motion_result_map(int result) {
    switch (result) {
        case ETHERCAT_MOTION_OK:
            return HOST_PROTOCOL_STATUS_OK;

        case ETHERCAT_MOTION_ERR_PARAM:
            return HOST_PROTOCOL_STATUS_PARAM_ERROR;

        case ETHERCAT_MOTION_ERR_BUSY:
            return HOST_PROTOCOL_STATUS_BUSY;

        case ETHERCAT_MOTION_ERR_NOT_READY:
            return HOST_PROTOCOL_STATUS_NOT_READY;

        case ETHERCAT_MOTION_ERR_LIMIT:
            return HOST_PROTOCOL_STATUS_LIMIT_ERROR;

        case ETHERCAT_MOTION_ERR_STATE:
            return HOST_PROTOCOL_STATUS_STATE_ERROR;

        default:
            return HOST_PROTOCOL_STATUS_INTERNAL_ERROR;
    }
}

/*
 * 根据1字节命令调用对应的运动接口。
 * 未定义命令不触发任何运动，直接返回UNKNOWN_COMMAND。
 */
host_protocol_status_t host_command_execute(uint8_t command) {
    int result;

    switch ((host_command_id_t) command) {
        case HOST_COMMAND_SET_SOFTWARE_ZERO:
            result = ethercat_motion_software_zero_set();
            break;

        case HOST_COMMAND_RETURN_MECHANICAL_ZERO:
            result = ethercat_motion_mechanical_zero_return();
            break;

        case HOST_COMMAND_RETURN_SOFTWARE_ZERO:
            result = ethercat_motion_software_zero_return();
            break;

        case HOST_COMMAND_PAUSE:
            result = ethercat_motion_stop();
            break;

        case HOST_COMMAND_CONTINUE:
            result = ethercat_motion_continue();
            break;

        case HOST_COMMAND_ABORT:
            result = ethercat_motion_abort();
            break;

        default:
            return HOST_PROTOCOL_STATUS_UNKNOWN_COMMAND;
    }

    return host_command_motion_result_map(result);
}
