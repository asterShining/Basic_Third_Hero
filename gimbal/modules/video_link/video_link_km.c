#include "video_link_km.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "memory.h"
#include "stdbool.h"
#include "stdlib.h"

#define VIDEO_LINK_KM_RX_BUFFER_SIZE 64u
#define VIDEO_LINK_KM_LOG_INTERVAL 50u

typedef struct
{
    int32_t mouse_x;
    int32_t mouse_y;
    int32_t mouse_z;
    uint32_t keyboard_value;
    uint8_t left_button_down;
    uint8_t right_button_down;
    uint8_t mid_button_down;
} VideoLinkKM_Decode_s;

static RC_ctrl_t video_link_ctrl[2];
static USARTInstance *video_link_usart_instance;
static DaemonInstance *video_link_daemon_instance;
static VideoLinkKM_Diag_s video_link_diag;
static uint8_t video_link_init_flag = 0;

/**
 * @brief 将 32 位整型限制到 RC 鼠标字段可承载的范围
 *
 * @param raw_value 原始图传鼠标增量
 * @return int16_t 限幅后的鼠标增量
 */
static int16_t VideoLinkClampToInt16(int32_t raw_value)
{
    // 这里统一把图传的 int32 鼠标增量压到 int16，作用是复用现有 RC_ctrl_t 结构；
    // 原因是 robot_cmd 已经完全围绕 RC_ctrl_t 写好，直接复用能避免重写下游控制分支。
    if (raw_value > 32767) {
        return 32767;
    }
    if (raw_value < -32768) {
        return -32768;
    }
    return (int16_t)raw_value;
}

/**
 * @brief 清空本拍的图传键鼠控制量
 *
 * @note 只清控制态,不清 key_count 统计
 */
static void VideoLinkClearCurrentControl(void)
{
    // 这里每拍先清空当前控制态，作用是避免丢字段或解析失败时沿用上一拍残留命令；
    // 原因是图传链路失真时最重要的是安全回零，而不是继续执行旧输入。
    memset(&video_link_ctrl[TEMP].rc, 0, sizeof(video_link_ctrl[TEMP].rc));
    memset(&video_link_ctrl[TEMP].mouse, 0, sizeof(video_link_ctrl[TEMP].mouse));
    memset(video_link_ctrl[TEMP].key, 0, sizeof(video_link_ctrl[TEMP].key));
}

/**
 * @brief 解析 protobuf varint
 *
 * @param buff 输入缓冲区
 * @param buff_len 输入缓冲区长度
 * @param value 输出值
 * @return uint8_t 已消费字节数,0 表示失败
 */
static uint8_t VideoLinkParseVarint(const uint8_t *buff, uint16_t buff_len, uint64_t *value)
{
    uint64_t result = 0u;

    if (buff == NULL || value == NULL) {
        return 0u;
    }

    for (uint8_t i = 0; i < 10u && i < buff_len; ++i) {
        result |= ((uint64_t)(buff[i] & 0x7Fu)) << (7u * i);
        if ((buff[i] & 0x80u) == 0u) {
            *value = result;
            return (uint8_t)(i + 1u);
        }
    }
    return 0u;
}

/**
 * @brief 直接按 KeyboardMouseControl protobuf 语义解析负载
 *
 * @param payload protobuf 负载
 * @param payload_len 负载长度
 * @param decode_out 解析结果
 * @return uint8_t 1:成功 0:失败
 */
static uint8_t VideoLinkTryDecodePayload(const uint8_t *payload, uint16_t payload_len, VideoLinkKM_Decode_s *decode_out)
{
    uint16_t offset = 0u;
    uint8_t recognized_mask = 0u;

    if (payload == NULL || decode_out == NULL || payload_len == 0u) {
        return 0u;
    }

    memset(decode_out, 0, sizeof(*decode_out));

    while (offset < payload_len) {
        uint64_t tag = 0u;
        uint64_t raw_value = 0u;
        uint8_t tag_len = VideoLinkParseVarint(payload + offset, (uint16_t)(payload_len - offset), &tag);
        uint8_t value_len;
        uint32_t field_number;
        uint8_t wire_type;

        if (tag_len == 0u || tag == 0u) {
            return 0u;
        }
        offset = (uint16_t)(offset + tag_len);

        field_number = (uint32_t)(tag >> 3);
        wire_type = (uint8_t)(tag & 0x07u);
        if (wire_type != 0u || field_number == 0u || field_number > 7u) {
            return 0u;
        }

        value_len = VideoLinkParseVarint(payload + offset, (uint16_t)(payload_len - offset), &raw_value);
        if (value_len == 0u) {
            return 0u;
        }
        offset = (uint16_t)(offset + value_len);

        switch (field_number) {
        case 1u:
            decode_out->mouse_x = (int32_t)raw_value;
            recognized_mask |= (1u << 0);
            break;
        case 2u:
            decode_out->mouse_y = (int32_t)raw_value;
            recognized_mask |= (1u << 1);
            break;
        case 3u:
            decode_out->mouse_z = (int32_t)raw_value;
            recognized_mask |= (1u << 2);
            break;
        case 4u:
            if (raw_value > 1u) {
                return 0u;
            }
            decode_out->left_button_down = (uint8_t)raw_value;
            recognized_mask |= (1u << 3);
            break;
        case 5u:
            if (raw_value > 1u) {
                return 0u;
            }
            decode_out->right_button_down = (uint8_t)raw_value;
            recognized_mask |= (1u << 4);
            break;
        case 6u:
            if (raw_value > 0xFFFFu) {
                return 0u;
            }
            decode_out->keyboard_value = (uint32_t)raw_value;
            recognized_mask |= (1u << 5);
            break;
        case 7u:
            if (raw_value > 1u) {
                return 0u;
            }
            decode_out->mid_button_down = (uint8_t)raw_value;
            recognized_mask |= (1u << 6);
            break;
        default:
            return 0u;
        }
    }

    return recognized_mask != 0u;
}

/**
 * @brief 尝试处理带长度前缀的 protobuf 帧
 *
 * @param frame 原始帧
 * @param frame_len 原始帧长
 * @param decode_out 解析结果
 * @return uint8_t 1:成功 0:失败
 */
static uint8_t VideoLinkTryDecodeWithLengthPrefix(const uint8_t *frame, uint16_t frame_len, VideoLinkKM_Decode_s *decode_out)
{
    uint64_t declared_len = 0u;
    uint8_t varint_len = 0u;
    uint16_t prefix_len_16 = 0u;

    if (frame == NULL || decode_out == NULL || frame_len <= 1u) {
        return 0u;
    }

    // 这里先尝试 varint 长度前缀，作用是兼容“长度 + protobuf”的常见串口封包方式；
    // 原因是 PDF 只定义了消息语义，没有给机器人侧 UART 封边格式，必须做一层兼容兜底。
    varint_len = VideoLinkParseVarint(frame, frame_len, &declared_len);
    if (varint_len != 0u && declared_len != 0u && (declared_len + varint_len) == frame_len) {
        if (VideoLinkTryDecodePayload(frame + varint_len, (uint16_t)declared_len, decode_out)) {
            return 1u;
        }
    }

    // 这里补 1 字节长度前缀，作用是兼容简化封装；
    // 原因是一些上位机桥接层会直接把 payload_len 放在首字节后透传到串口。
    if (frame[0] == (uint8_t)(frame_len - 1u)) {
        if (VideoLinkTryDecodePayload(frame + 1u, (uint16_t)(frame_len - 1u), decode_out)) {
            return 1u;
        }
    }

    if (frame_len > 2u) {
        prefix_len_16 = (uint16_t)(frame[0] | (frame[1] << 8));
        if ((uint16_t)(prefix_len_16 + 2u) == frame_len) {
            if (VideoLinkTryDecodePayload(frame + 2u, prefix_len_16, decode_out)) {
                return 1u;
            }
        }

        prefix_len_16 = (uint16_t)((frame[0] << 8) | frame[1]);
        if ((uint16_t)(prefix_len_16 + 2u) == frame_len) {
            if (VideoLinkTryDecodePayload(frame + 2u, prefix_len_16, decode_out)) {
                return 1u;
            }
        }
    }

    return 0u;
}

/**
 * @brief 综合尝试解析图传键鼠帧
 *
 * @param frame 原始帧
 * @param frame_len 原始帧长
 * @param decode_out 解析结果
 * @return uint8_t 1:成功 0:失败
 */
static uint8_t VideoLinkTryDecodeFrame(const uint8_t *frame, uint16_t frame_len, VideoLinkKM_Decode_s *decode_out)
{
    if (frame == NULL || decode_out == NULL || frame_len == 0u) {
        return 0u;
    }

    // 这里优先按“纯 protobuf 负载”解析，作用是直接匹配官方 KeyboardMouseControl 消息体；
    // 原因是比赛时官方选手端与图传链路最终到机器人侧的有效语义就是该消息的字段集合。
    if (VideoLinkTryDecodePayload(frame, frame_len, decode_out)) {
        return 1u;
    }

    if (VideoLinkTryDecodeWithLengthPrefix(frame, frame_len, decode_out)) {
        return 1u;
    }

    // 这里补 1~2 字节头尾裁剪尝试，作用是给可能存在的轻量封边留出兼容空间；
    // 原因是当前仓库没有官方机器人侧 UART 封装样例，真机链路上可能多一层短头短尾。
    for (uint8_t head_skip = 0u; head_skip <= 2u && head_skip < frame_len; ++head_skip) {
        for (uint8_t tail_trim = 0u; tail_trim <= 2u && (uint16_t)(head_skip + tail_trim) < frame_len; ++tail_trim) {
            uint16_t payload_len = (uint16_t)(frame_len - head_skip - tail_trim);
            if (head_skip == 0u && tail_trim == 0u) {
                continue;
            }
            if (VideoLinkTryDecodePayload(frame + head_skip, payload_len, decode_out)) {
                return 1u;
            }
        }
    }

    return 0u;
}

/**
 * @brief 根据解析结果更新 RC 风格的键鼠状态
 *
 * @param decode 解析结果
 */
static void VideoLinkApplyDecodedState(const VideoLinkKM_Decode_s *decode)
{
    uint16_t key_now;
    uint16_t key_last;
    uint16_t key_with_ctrl;
    uint16_t key_with_shift;
    uint16_t key_last_with_ctrl;
    uint16_t key_last_with_shift;

    if (decode == NULL) {
        return;
    }

    VideoLinkClearCurrentControl();

    video_link_ctrl[TEMP].mouse.x = VideoLinkClampToInt16(decode->mouse_x);
    video_link_ctrl[TEMP].mouse.y = VideoLinkClampToInt16(decode->mouse_y);
    video_link_ctrl[TEMP].mouse.press_l = decode->left_button_down;
    video_link_ctrl[TEMP].mouse.press_r = decode->right_button_down;
    *(uint16_t *)&video_link_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(decode->keyboard_value & 0xFFFFu);

    if (video_link_ctrl[TEMP].key[KEY_PRESS].ctrl) {
        video_link_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = video_link_ctrl[TEMP].key[KEY_PRESS];
    } else {
        memset(&video_link_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    }

    if (video_link_ctrl[TEMP].key[KEY_PRESS].shift) {
        video_link_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = video_link_ctrl[TEMP].key[KEY_PRESS];
    } else {
        memset(&video_link_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));
    }

    key_now = video_link_ctrl[TEMP].key[KEY_PRESS].keys;
    key_last = video_link_ctrl[LAST].key[KEY_PRESS].keys;
    key_with_ctrl = video_link_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys;
    key_with_shift = video_link_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys;
    key_last_with_ctrl = video_link_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys;
    key_last_with_shift = video_link_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys;

    for (uint16_t i = 0u, j = 0x1u; i < 16u; j <<= 1u, ++i) {
        if (i == 4u || i == 5u) {
            continue;
        }
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j)) {
            video_link_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        }
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j)) {
            video_link_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        }
        if ((key_with_shift & j) && !(key_last_with_shift & j)) {
            video_link_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
        }
    }

    video_link_diag.last_mouse_z = decode->mouse_z;
    video_link_diag.last_mid_button = decode->mid_button_down;
    memcpy(&video_link_ctrl[LAST], &video_link_ctrl[TEMP], sizeof(RC_ctrl_t));
}

/**
 * @brief 图传键鼠串口接收回调
 *
 */
static void VideoLinkKMRxCallback(void)
{
    VideoLinkKM_Decode_s decode;
    uint16_t recv_len = 0u;

    if (video_link_usart_instance == NULL) {
        return;
    }

    recv_len = video_link_usart_instance->recv_len;
    video_link_diag.rx_frame_count++;
    video_link_diag.last_frame_len = recv_len;
    DaemonReload(video_link_daemon_instance);

    VideoLinkClearCurrentControl();

    if (VideoLinkTryDecodeFrame(video_link_usart_instance->recv_buff, recv_len, &decode)) {
        // 这里解析成功后立即覆写当前控制态，作用是把图传键鼠无缝映射到现有 robot_cmd；
        // 原因是下游已经以 RC_ctrl_t 为统一输入层，复用该结构能把修改面控制在最小范围。
        VideoLinkApplyDecodedState(&decode);
        video_link_diag.decode_success_count++;
    } else {
        video_link_diag.decode_fail_count++;
        memcpy(&video_link_ctrl[LAST], &video_link_ctrl[TEMP], sizeof(RC_ctrl_t));
        if ((video_link_diag.decode_fail_count % VIDEO_LINK_KM_LOG_INTERVAL) == 1u) {
            LOGWARNING("[video_link] decode failed, rx_len=%d fail_cnt=%d",
                       recv_len,
                       video_link_diag.decode_fail_count);
        }
    }
}

/**
 * @brief 图传键鼠掉线回调
 *
 * @param id 未使用
 */
static void VideoLinkKMOfflineCallback(void *id)
{
    (void)id;

    // 这里掉线时直接清空图传键鼠状态，作用是让 robot_cmd 在下一拍拿到全零输入；
    // 原因是图传控车最怕断链后残留旧命令继续执行，必须在模块层先做安全兜底。
    memset(video_link_ctrl, 0, sizeof(video_link_ctrl));
    video_link_diag.last_mouse_z = 0;
    video_link_diag.last_mid_button = 0u;
    USARTServiceInit(video_link_usart_instance);
    LOGWARNING("[video_link] keyboard-mouse link lost");
}

RC_ctrl_t *VideoLinkKMInit(UART_HandleTypeDef *video_link_usart_handle)
{
    USART_Init_Config_s usart_conf;
    Daemon_Init_Config_s daemon_conf;

    memset(&video_link_diag, 0, sizeof(video_link_diag));
    memset(video_link_ctrl, 0, sizeof(video_link_ctrl));

    usart_conf.recv_buff_size = VIDEO_LINK_KM_RX_BUFFER_SIZE;
    usart_conf.usart_handle = video_link_usart_handle;
    usart_conf.module_callback = VideoLinkKMRxCallback;
    video_link_usart_instance = USARTRegister(&usart_conf);

    daemon_conf.reload_count = 10u;
    daemon_conf.init_count = 20u;
    daemon_conf.callback = VideoLinkKMOfflineCallback;
    daemon_conf.owner_id = NULL;
    video_link_daemon_instance = DaemonRegister(&daemon_conf);

    video_link_init_flag = 1u;
    LOGINFO("[video_link] init ok, uart=%p rx_buff=%d",
            (void *)video_link_usart_handle,
            VIDEO_LINK_KM_RX_BUFFER_SIZE);
    return video_link_ctrl;
}

uint8_t VideoLinkKMIsOnline(void)
{
    if (!video_link_init_flag) {
        return 0u;
    }
    return DaemonIsOnline(video_link_daemon_instance);
}

const VideoLinkKM_Diag_s *VideoLinkKMGetDiag(void)
{
    return &video_link_diag;
}
