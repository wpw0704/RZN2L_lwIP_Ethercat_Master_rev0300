#ifndef HOST_PROTOCOL_H
#define HOST_PROTOCOL_H

#include <stdint.h>

/* 固定协议帧：77 44 DATA 44 77。 */
#define HOST_PROTOCOL_FRAME_SIZE     (5U)
#define HOST_PROTOCOL_HEADER_FIRST   (0x77U)
#define HOST_PROTOCOL_HEADER_SECOND  (0x44U)
#define HOST_PROTOCOL_TAIL_FIRST     (0x44U)
#define HOST_PROTOCOL_TAIL_SECOND    (0x77U)

/* 设备应答帧DATA字节的状态定义。 */
typedef enum {
    HOST_PROTOCOL_STATUS_OK = 0x00,              /* 命令已经成功接受。 */
    HOST_PROTOCOL_STATUS_FRAME_ERROR = 0x01,     /* 帧尾错误。 */
    HOST_PROTOCOL_STATUS_UNKNOWN_COMMAND = 0x02, /* 命令字未定义。 */
    HOST_PROTOCOL_STATUS_PARAM_ERROR = 0x03,     /* 参数或模式错误。 */
    HOST_PROTOCOL_STATUS_BUSY = 0x04,            /* 运动或请求槽忙。 */
    HOST_PROTOCOL_STATUS_NOT_READY = 0x05,       /* PDO或电机参数未就绪。 */
    HOST_PROTOCOL_STATUS_LIMIT_ERROR = 0x06,     /* 位置或速度超限。 */
    HOST_PROTOCOL_STATUS_STATE_ERROR = 0x07,     /* 当前状态不允许执行。 */
    HOST_PROTOCOL_STATUS_INTERNAL_ERROR = 0x7F,  /* 未映射的内部错误。 */
} host_protocol_status_t;

/* 每输入一个字节后解析器产生的事件。 */
typedef enum {
    HOST_PROTOCOL_EVENT_NONE = 0,    /* 尚未得到完整帧。 */
    HOST_PROTOCOL_EVENT_COMMAND,     /* 已得到一个合法命令帧。 */
    HOST_PROTOCOL_EVENT_FRAME_ERROR, /* 已识别帧头，但帧尾错误。 */
} host_protocol_event_t;

/* 单个TCP连接的协议解析状态。 */
typedef struct {
    uint8_t state;   /* 当前固定帧解析阶段。 */
    uint8_t command; /* 已暂存的命令字。 */
} host_protocol_parser_t;

/**
 * @brief 重置一个TCP连接的协议解析状态。
 *
 * @param parser 要重置的解析器；传入NULL时不执行操作。
 */
void host_protocol_parser_reset(host_protocol_parser_t *parser);

/**
 * @brief 向固定帧解析器输入一个字节。
 *
 * 函数可跨多次TCP接收保存状态，并支持连续帧和错误帧后的重新同步。
 *
 * @param parser  当前TCP连接对应的解析器。
 * @param byte    本次输入的一个字节。
 * @param command 收到合法帧时用于返回1字节命令；不得为NULL。
 * @return host_protocol_event_t，表示无事件、合法命令或错误帧。
 */
host_protocol_event_t host_protocol_byte_push(
    host_protocol_parser_t *parser,
    uint8_t byte,
    uint8_t *command);

/**
 * @brief 生成固定5字节状态应答帧。
 *
 * 输出格式为77 44 STATUS 44 77。
 *
 * @param status         要返回给上位机的状态码。
 * @param response_frame 长度至少为HOST_PROTOCOL_FRAME_SIZE的输出缓冲区。
 */
void host_protocol_response_build(
    host_protocol_status_t status,
    uint8_t response_frame[HOST_PROTOCOL_FRAME_SIZE]);

#endif /* HOST_PROTOCOL_H */
