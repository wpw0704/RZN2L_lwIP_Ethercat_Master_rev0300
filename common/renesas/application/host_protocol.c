#include "host_protocol.h"

#include <stddef.h>

/* 固定5字节帧的内部解析阶段。 */
typedef enum {
    HOST_PROTOCOL_PARSE_WAIT_HEADER_FIRST = 0,
    HOST_PROTOCOL_PARSE_WAIT_HEADER_SECOND,
    HOST_PROTOCOL_PARSE_WAIT_COMMAND,
    HOST_PROTOCOL_PARSE_WAIT_TAIL_FIRST,
    HOST_PROTOCOL_PARSE_WAIT_TAIL_SECOND,
} host_protocol_parse_state_t;

/*
 * 根据当前字节恢复到等待帧头状态。
 * 如果当前字节本身是0x77，则保留为下一帧的第一个帧头字节。
 */
static void host_protocol_resync(
    host_protocol_parser_t *parser,
    uint8_t byte) {
    if (byte == HOST_PROTOCOL_HEADER_FIRST) {
        parser->state =
            (uint8_t) HOST_PROTOCOL_PARSE_WAIT_HEADER_SECOND;
    } else {
        parser->state =
            (uint8_t) HOST_PROTOCOL_PARSE_WAIT_HEADER_FIRST;
    }
}

/* 重置解析器，丢弃尚未完成的帧。 */
void host_protocol_parser_reset(host_protocol_parser_t *parser) {
    if (parser == NULL) {
        return;
    }

    parser->state =
        (uint8_t) HOST_PROTOCOL_PARSE_WAIT_HEADER_FIRST;
    parser->command = 0U;
}

/*
 * 固定帧流式解析器。
 * 只有已经识别帧头和命令后发现帧尾错误，才报告FRAME_ERROR；
 * 帧头之前的无关字节直接丢弃。
 */
host_protocol_event_t host_protocol_byte_push(
    host_protocol_parser_t *parser,
    uint8_t byte,
    uint8_t *command) {
    host_protocol_event_t event = HOST_PROTOCOL_EVENT_NONE;

    if ((parser == NULL) || (command == NULL)) {
        return HOST_PROTOCOL_EVENT_FRAME_ERROR;
    }

    switch ((host_protocol_parse_state_t) parser->state) {
        case HOST_PROTOCOL_PARSE_WAIT_HEADER_FIRST:
            if (byte == HOST_PROTOCOL_HEADER_FIRST) {
                parser->state =
                    (uint8_t) HOST_PROTOCOL_PARSE_WAIT_HEADER_SECOND;
            }
            break;

        case HOST_PROTOCOL_PARSE_WAIT_HEADER_SECOND:
            if (byte == HOST_PROTOCOL_HEADER_SECOND) {
                parser->state =
                    (uint8_t) HOST_PROTOCOL_PARSE_WAIT_COMMAND;
            } else {
                host_protocol_resync(parser, byte);
            }
            break;

        case HOST_PROTOCOL_PARSE_WAIT_COMMAND:
            parser->command = byte;
            parser->state =
                (uint8_t) HOST_PROTOCOL_PARSE_WAIT_TAIL_FIRST;
            break;

        case HOST_PROTOCOL_PARSE_WAIT_TAIL_FIRST:
            if (byte == HOST_PROTOCOL_TAIL_FIRST) {
                parser->state =
                    (uint8_t) HOST_PROTOCOL_PARSE_WAIT_TAIL_SECOND;
            } else {
                event = HOST_PROTOCOL_EVENT_FRAME_ERROR;
                host_protocol_resync(parser, byte);
            }
            break;

        case HOST_PROTOCOL_PARSE_WAIT_TAIL_SECOND:
            if (byte == HOST_PROTOCOL_TAIL_SECOND) {
                *command = parser->command;
                event = HOST_PROTOCOL_EVENT_COMMAND;
                host_protocol_parser_reset(parser);
            } else {
                event = HOST_PROTOCOL_EVENT_FRAME_ERROR;
                host_protocol_resync(parser, byte);
            }
            break;

        default:
            host_protocol_parser_reset(parser);
            break;
    }

    return event;
}

/* 生成77 44 STATUS 44 77状态应答帧。 */
void host_protocol_response_build(
    host_protocol_status_t status,
    uint8_t response_frame[HOST_PROTOCOL_FRAME_SIZE]) {
    if (response_frame == NULL) {
        return;
    }

    response_frame[0] = HOST_PROTOCOL_HEADER_FIRST;
    response_frame[1] = HOST_PROTOCOL_HEADER_SECOND;
    response_frame[2] = (uint8_t) status;
    response_frame[3] = HOST_PROTOCOL_TAIL_FIRST;
    response_frame[4] = HOST_PROTOCOL_TAIL_SECOND;
}
