#ifndef HOST_COMMAND_H
#define HOST_COMMAND_H

#include "host_protocol.h"

#include <stdint.h>

/* 上位机下发帧DATA字节的命令定义。 */
typedef enum {
    HOST_COMMAND_SET_SOFTWARE_ZERO = 0x01,
    HOST_COMMAND_RETURN_MECHANICAL_ZERO = 0x02,
    HOST_COMMAND_RETURN_SOFTWARE_ZERO = 0x03,
    HOST_COMMAND_PAUSE = 0x04,
    HOST_COMMAND_CONTINUE = 0x05,
    HOST_COMMAND_ABORT = 0x06,
} host_command_id_t;

/**
 * @brief 判断并执行一条上位机控制命令。
 *
 * 所有运动接口均为非阻塞接口；返回成功只表示命令已经接受，
 * 最终运动状态仍需通过运动状态接口读取。
 *
 * @param command 上位机帧中的1字节命令。
 * @return host_protocol_status_t，可直接放入状态应答帧。
 */
host_protocol_status_t host_command_execute(uint8_t command);

#endif /* HOST_COMMAND_H */
