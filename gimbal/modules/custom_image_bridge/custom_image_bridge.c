#include "custom_image_bridge.h"

#include "bsp_log.h"
#include "bsp_usb.h"
#include "crc_ref.h"
#include "main.h"
#include "referee_protocol.h"
#include "string.h"
#include "usart.h"

#define CUSTOM_IMAGE_BRIDGE_USB_PACKET_SLOT_COUNT 12u
#define CUSTOM_IMAGE_BRIDGE_USB_PACKET_MAX_BYTES APP_RX_DATA_SIZE
#define CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES 6144u
#define CUSTOM_IMAGE_BRIDGE_PACKET_QUEUE_SLOT_COUNT 16u
#define CUSTOM_IMAGE_BRIDGE_PACKET_QUEUE_HIGH_WATERMARK 12u
#define CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES LEN_robot_custom_data_2
#define CUSTOM_IMAGE_BRIDGE_PROTOCOL_PAYLOAD_BYTES 300u
#define CUSTOM_IMAGE_BRIDGE_FRAME_BYTES (LEN_HEADER + LEN_CMDID + LEN_robot_custom_data_2 + LEN_TAIL)
#define CUSTOM_IMAGE_BRIDGE_LOG_INTERVAL_MS 1000u
#define CUSTOM_IMAGE_BRIDGE_UART_BUSY_RECOVER_MS 120u
#define CUSTOM_IMAGE_BRIDGE_USB_SILENCE_RESET_MS 1000u
#define CUSTOM_IMAGE_BRIDGE_H264_TAIL_KEEP_BYTES 5u
#define CUSTOM_IMAGE_BRIDGE_H264_NAL_IDR 5u
#define CUSTOM_IMAGE_BRIDGE_H264_NAL_SPS 7u

// 这里用编译期断言把 0x0310 的 payload 大小钉死为协议要求的 300B；
// 原因是这条桥就是靠“上位机原始 H.264 字节流 -> 300B 固定块 -> 0x0310”工作的，
// 一旦裁判协议常量或本地宏被人顺手改掉，继续编译运行只会把整条图传链 silently 打坏。
_Static_assert(CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES == CUSTOM_IMAGE_BRIDGE_PROTOCOL_PAYLOAD_BYTES,
               "custom image bridge payload must stay at 300 bytes for 0x0310");

// 这里将图像桥接缓存收敛到“中间档”，目的是在视频稳定性和板上 RAM 余量之间取平衡。
// 满配缓存会把当前云台板 RAM 顶到接近 100%，而最小缓存又已经在实机上暴露出恢复慢、画面断续的问题。
// 现有上位机稳态码率约 8~10kB/s，这组尺寸能比最小档多覆盖几拍 USB CDC / VT03 串口抖动，
// 同时显著低于满配缓存的占用；后续继续调时，应直接看 `pending_drop`、`usb_queue_drop_count` 和客户端 AU 恢复速率。

typedef struct
{
    uint8_t bytes[CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES];
} CustomImageBridgePacketSlot_s;

typedef struct
{
    uint32_t usb_rx_packets; // 记录 USB RX 包数，便于确认上位机是否持续送协议包
    uint32_t usb_rx_bytes; // 记录 USB RX 字节数，便于核对链路吞吐
    uint16_t last_usb_rx_len; // 记录最近一次 USB RX 包长，便于确认上位机当前是否真的在送有效负载
    uint32_t last_usb_rx_tick_ms; // 记录最近一次 USB RX 时刻，便于区分“刚启动还没来包”和“已经断流”
    uint32_t usb_queue_drop_count; // 记录 USB 回调槽位挤爆时丢弃旧包的次数
    uint32_t pending_trim_count; // 记录原始缓存因拥塞被裁剪的次数
    uint32_t pending_drop_bytes; // 记录为了保留最新视频流而丢掉的历史原始字节数
    uint32_t packet_queue_drop_count; // 记录 packet 队列满时丢弃旧 packet 的次数
    uint32_t uart_tx_packet_count; // 记录成功下发的 0x0310 包数
    uint32_t uart_tx_bytes; // 记录成功发出的原始视频字节数，便于和上位机限速配置核对
    uint32_t last_uart_tx_tick_ms; // 记录最近一次 0x0310 成功下发时刻，便于判断桥接是否只收不发
    uint32_t uart_busy_skip_count; // 记录因 USART6 TX 忙而本周期跳过发送的次数
    uint32_t uart_tx_error_count; // 记录 HAL_UART_Transmit_DMA 返回失败的次数
    uint32_t uart_tx_abort_count; // 记录 TX 长时间 busy 或异常后主动 abort 的次数，用于判断是否存在 DMA 状态卡死
    uint32_t resync_count; // 记录桥接层主动丢弃旧流并等待 H.264 关键边界的次数
    uint32_t resync_drop_bytes; // 记录重同步期间丢掉的原始视频字节数，便于和客户端坏块窗口对齐
    uint32_t last_reset_tick_ms; // 记录最近一次自愈复位时刻，用于判断问题是否随运行时间累积
    uint8_t last_reset_reason; // 记录最近一次自愈原因，日志中会转成可读文本
    uint32_t last_logged_usb_rx_packets; // 记录上一次打印日志时的累计 USB 包数
    uint32_t last_logged_uart_tx_packet_count; // 记录上一次打印日志时的累计 0x0310 包数
    uint32_t last_log_tick_ms; // 记录上次打印诊断的时间，用于日志限频
} CustomImageBridgeDiag_s;

typedef enum
{
    CUSTOM_IMAGE_BRIDGE_RESET_NONE = 0u,
    CUSTOM_IMAGE_BRIDGE_RESET_USB_QUEUE_FULL,
    CUSTOM_IMAGE_BRIDGE_RESET_PENDING_OVERFLOW,
    CUSTOM_IMAGE_BRIDGE_RESET_PACKET_BACKLOG,
    CUSTOM_IMAGE_BRIDGE_RESET_UART_BUSY_TIMEOUT,
    CUSTOM_IMAGE_BRIDGE_RESET_UART_TX_ERROR,
    CUSTOM_IMAGE_BRIDGE_RESET_USB_SILENCE,
} CustomImageBridgeResetReason_e;

static uint8_t *custom_image_bridge_usb_rx_buffer;
static uint8_t custom_image_bridge_usb_packet_slots[CUSTOM_IMAGE_BRIDGE_USB_PACKET_SLOT_COUNT][CUSTOM_IMAGE_BRIDGE_USB_PACKET_MAX_BYTES];
static uint16_t custom_image_bridge_usb_packet_lens[CUSTOM_IMAGE_BRIDGE_USB_PACKET_SLOT_COUNT];
static volatile uint8_t custom_image_bridge_usb_queue_read_index;
static volatile uint8_t custom_image_bridge_usb_queue_write_index;
static volatile uint8_t custom_image_bridge_usb_queue_count;
static uint8_t custom_image_bridge_task_chunk[CUSTOM_IMAGE_BRIDGE_USB_PACKET_MAX_BYTES];
static uint8_t custom_image_bridge_pending_bytes[CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES];
static size_t custom_image_bridge_pending_len;
static CustomImageBridgePacketSlot_s custom_image_bridge_packet_queue[CUSTOM_IMAGE_BRIDGE_PACKET_QUEUE_SLOT_COUNT];
static uint8_t custom_image_bridge_packet_queue_read_index;
static uint8_t custom_image_bridge_packet_queue_write_index;
static uint8_t custom_image_bridge_packet_queue_count;
static uint8_t custom_image_bridge_tx_frame[CUSTOM_IMAGE_BRIDGE_FRAME_BYTES];
static uint8_t custom_image_bridge_tx_seq;
static CustomImageBridgeDiag_s custom_image_bridge_diag;
static volatile uint8_t custom_image_bridge_reset_requested;
static volatile uint8_t custom_image_bridge_requested_reset_reason;
static uint8_t custom_image_bridge_wait_key_boundary;
static uint32_t custom_image_bridge_uart_busy_start_tick_ms;
static uint32_t custom_image_bridge_last_silence_reset_rx_tick_ms;

static uint32_t CustomImageBridgeEnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void CustomImageBridgeExitCritical(uint32_t primask)
{
    if (primask == 0u) {
        __enable_irq();
    }
}

static const char *CustomImageBridgeResetReasonName(uint8_t reason)
{
    switch ((CustomImageBridgeResetReason_e)reason) {
    case CUSTOM_IMAGE_BRIDGE_RESET_USB_QUEUE_FULL:
        return "usb_queue_full";
    case CUSTOM_IMAGE_BRIDGE_RESET_PENDING_OVERFLOW:
        return "pending_overflow";
    case CUSTOM_IMAGE_BRIDGE_RESET_PACKET_BACKLOG:
        return "packet_backlog";
    case CUSTOM_IMAGE_BRIDGE_RESET_UART_BUSY_TIMEOUT:
        return "uart_busy_timeout";
    case CUSTOM_IMAGE_BRIDGE_RESET_UART_TX_ERROR:
        return "uart_tx_error";
    case CUSTOM_IMAGE_BRIDGE_RESET_USB_SILENCE:
        return "usb_silence";
    default:
        return "none";
    }
}

static void CustomImageBridgeRequestResetFromCallback(CustomImageBridgeResetReason_e reason)
{
    // USB CDC 回调运行在中断相关路径，不能在这里搬移大块内存或重置整条桥接状态；
    // 因此只留下一个复位请求，由 20ms 图像桥接任务在普通任务上下文里执行真正的清队列和重同步。
    custom_image_bridge_requested_reset_reason = (uint8_t)reason;
    custom_image_bridge_reset_requested = 1u;
}

static void CustomImageBridgeResetStreamState(CustomImageBridgeResetReason_e reason)
{
    uint32_t primask = 0u;
    uint8_t usb_queue_dropped = 0u;
    uint8_t packet_queue_dropped = custom_image_bridge_packet_queue_count;
    uint32_t raw_drop_bytes =
        (uint32_t)custom_image_bridge_pending_len +
        (uint32_t)packet_queue_dropped * (uint32_t)CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES;

    // 这里同时清掉 USB 暂存队列、原始 pending 和 300B packet 队列，作用是切断已经错位或过期的 H.264 字节流；
    // 原因是自定义客户端解码器对字节连续性敏感，任意丢旧 300B 块后继续发送只会把坏块和高延迟维持更久。
    primask = CustomImageBridgeEnterCritical();
    usb_queue_dropped = custom_image_bridge_usb_queue_count;
    custom_image_bridge_usb_queue_read_index = custom_image_bridge_usb_queue_write_index;
    custom_image_bridge_usb_queue_count = 0u;
    CustomImageBridgeExitCritical(primask);

    custom_image_bridge_pending_len = 0u;
    custom_image_bridge_packet_queue_read_index = 0u;
    custom_image_bridge_packet_queue_write_index = 0u;
    custom_image_bridge_packet_queue_count = 0u;
    custom_image_bridge_wait_key_boundary = 1u;

    custom_image_bridge_diag.usb_queue_drop_count += usb_queue_dropped;
    custom_image_bridge_diag.packet_queue_drop_count += packet_queue_dropped;
    custom_image_bridge_diag.pending_drop_bytes += raw_drop_bytes;
    custom_image_bridge_diag.resync_drop_bytes += raw_drop_bytes;
    custom_image_bridge_diag.resync_count++;
    custom_image_bridge_diag.last_reset_reason = (uint8_t)reason;
    custom_image_bridge_diag.last_reset_tick_ms = HAL_GetTick();
}

static void CustomImageBridgeApplyRequestedReset(void)
{
    uint32_t primask = 0u;
    uint8_t reset_requested = 0u;
    uint8_t reset_reason = CUSTOM_IMAGE_BRIDGE_RESET_NONE;

    // 这里把中断侧复位请求一次性取走，作用是让 USB 溢出这类异步事件不会和任务侧清队列互相踩状态；
    // 原因是 USB 回调可能在任意时刻到来，任务侧必须以一个稳定快照决定是否执行完整自愈流程。
    primask = CustomImageBridgeEnterCritical();
    reset_requested = custom_image_bridge_reset_requested;
    reset_reason = custom_image_bridge_requested_reset_reason;
    custom_image_bridge_reset_requested = 0u;
    CustomImageBridgeExitCritical(primask);

    if (reset_requested != 0u) {
        CustomImageBridgeResetStreamState((CustomImageBridgeResetReason_e)reset_reason);
    }
}

static uint8_t CustomImageBridgeEnqueuePacket(const uint8_t *packet)
{
    uint8_t write_index = custom_image_bridge_packet_queue_write_index;

    if (packet == NULL) {
        return 0u;
    }

    if (custom_image_bridge_packet_queue_count >= CUSTOM_IMAGE_BRIDGE_PACKET_QUEUE_SLOT_COUNT) {
        CustomImageBridgeResetStreamState(CUSTOM_IMAGE_BRIDGE_RESET_PACKET_BACKLOG);
        return 0u;
    }

    write_index = custom_image_bridge_packet_queue_write_index;
    memcpy(custom_image_bridge_packet_queue[write_index].bytes, packet, CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES);
    custom_image_bridge_packet_queue_write_index =
        (uint8_t)((write_index + 1u) % CUSTOM_IMAGE_BRIDGE_PACKET_QUEUE_SLOT_COUNT);
    custom_image_bridge_packet_queue_count++;
    return 1u;
}

static void CustomImageBridgeUSBRxCallback(uint16_t recv_len)
{
    uint16_t valid_len = recv_len;
    uint8_t write_index = 0u;
    uint32_t primask = 0u;
    uint32_t now_tick = HAL_GetTick();

    if (custom_image_bridge_usb_rx_buffer == NULL || recv_len == 0u) {
        return;
    }

    if (valid_len > CUSTOM_IMAGE_BRIDGE_USB_PACKET_MAX_BYTES) {
        valid_len = CUSTOM_IMAGE_BRIDGE_USB_PACKET_MAX_BYTES;
    }

    primask = CustomImageBridgeEnterCritical();
    write_index = custom_image_bridge_usb_queue_write_index;
    if (custom_image_bridge_usb_queue_count >= CUSTOM_IMAGE_BRIDGE_USB_PACKET_SLOT_COUNT) {
        custom_image_bridge_usb_queue_read_index =
            (uint8_t)((custom_image_bridge_usb_queue_read_index + 1u) % CUSTOM_IMAGE_BRIDGE_USB_PACKET_SLOT_COUNT);
        custom_image_bridge_usb_queue_count--;
        custom_image_bridge_diag.usb_queue_drop_count++;
        CustomImageBridgeRequestResetFromCallback(CUSTOM_IMAGE_BRIDGE_RESET_USB_QUEUE_FULL);
    }
    CustomImageBridgeExitCritical(primask);

    memcpy(custom_image_bridge_usb_packet_slots[write_index], custom_image_bridge_usb_rx_buffer, valid_len);
    custom_image_bridge_usb_packet_lens[write_index] = valid_len;

    primask = CustomImageBridgeEnterCritical();
    custom_image_bridge_usb_queue_write_index =
        (uint8_t)((write_index + 1u) % CUSTOM_IMAGE_BRIDGE_USB_PACKET_SLOT_COUNT);
    custom_image_bridge_usb_queue_count++;
    custom_image_bridge_diag.usb_rx_packets++;
    custom_image_bridge_diag.usb_rx_bytes += valid_len;
    custom_image_bridge_diag.last_usb_rx_len = valid_len;
    custom_image_bridge_diag.last_usb_rx_tick_ms = now_tick;
    CustomImageBridgeExitCritical(primask);
}

static void CustomImageBridgeAppendPendingBytes(const uint8_t *data, uint16_t data_len)
{
    uint16_t valid_len = data_len;

    if (data == NULL || data_len == 0u) {
        return;
    }

    if (valid_len > CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES) {
        custom_image_bridge_diag.pending_trim_count++;
        custom_image_bridge_diag.pending_drop_bytes +=
            (uint32_t)(valid_len - CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES);
        custom_image_bridge_diag.resync_drop_bytes +=
            (uint32_t)(valid_len - CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES);
        data += (valid_len - CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES);
        valid_len = CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES;
        custom_image_bridge_wait_key_boundary = 1u;
    }

    if (custom_image_bridge_pending_len + valid_len > CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES) {
        // pending 溢出说明上位机输入已经超过 0x0310 输出消化能力，继续裁掉任意前缀会破坏 H.264 NAL 边界；
        // 这里直接清空旧状态并等待下一个 SPS/IDR，牺牲一小段画面换取解码器快速恢复稳定。
        custom_image_bridge_diag.pending_trim_count++;
        CustomImageBridgeResetStreamState(CUSTOM_IMAGE_BRIDGE_RESET_PENDING_OVERFLOW);
    }

    memcpy(custom_image_bridge_pending_bytes + custom_image_bridge_pending_len, data, valid_len);
    custom_image_bridge_pending_len += valid_len;
}

static void CustomImageBridgeConsumePendingPrefix(size_t bytes_to_drop)
{
    if (bytes_to_drop == 0u || custom_image_bridge_pending_len == 0u) {
        return;
    }

    if (bytes_to_drop >= custom_image_bridge_pending_len) {
        custom_image_bridge_pending_len = 0u;
        return;
    }

    memmove(custom_image_bridge_pending_bytes,
            custom_image_bridge_pending_bytes + bytes_to_drop,
            custom_image_bridge_pending_len - bytes_to_drop);
    custom_image_bridge_pending_len -= bytes_to_drop;
}

static uint8_t CustomImageBridgeFindRecoverOffset(size_t *recover_offset)
{
    size_t i = 0u;

    if (recover_offset == NULL || custom_image_bridge_pending_len < 4u) {
        return 0u;
    }

    // 这里只接受 Annex-B start code 后的 SPS 或 IDR 作为恢复点，作用是让客户端从可解码边界重新进入；
    // 原因是随机从 P/B slice 中间恢复会造成颜色崩塌和长时间坏块，必须等编码器重复头或关键帧把状态补齐。
    for (i = 0u; i + 4u <= custom_image_bridge_pending_len; i++) {
        size_t nal_offset = 0u;

        if (custom_image_bridge_pending_bytes[i] == 0u &&
            custom_image_bridge_pending_bytes[i + 1u] == 0u &&
            custom_image_bridge_pending_bytes[i + 2u] == 1u) {
            nal_offset = i + 3u;
        } else if (i + 5u <= custom_image_bridge_pending_len &&
                   custom_image_bridge_pending_bytes[i] == 0u &&
                   custom_image_bridge_pending_bytes[i + 1u] == 0u &&
                   custom_image_bridge_pending_bytes[i + 2u] == 0u &&
                   custom_image_bridge_pending_bytes[i + 3u] == 1u) {
            nal_offset = i + 4u;
        } else {
            continue;
        }

        if (nal_offset < custom_image_bridge_pending_len) {
            uint8_t nal_type = (uint8_t)(custom_image_bridge_pending_bytes[nal_offset] & 0x1Fu);
            if (nal_type == CUSTOM_IMAGE_BRIDGE_H264_NAL_SPS ||
                nal_type == CUSTOM_IMAGE_BRIDGE_H264_NAL_IDR) {
                *recover_offset = i;
                return 1u;
            }
        }
    }

    return 0u;
}

static void CustomImageBridgeDropUntilRecoverPoint(void)
{
    size_t recover_offset = 0u;
    size_t bytes_to_drop = 0u;

    if (custom_image_bridge_wait_key_boundary == 0u || custom_image_bridge_pending_len == 0u) {
        return;
    }

    if (CustomImageBridgeFindRecoverOffset(&recover_offset) != 0u) {
        if (recover_offset > 0u) {
            custom_image_bridge_diag.pending_drop_bytes += (uint32_t)recover_offset;
            custom_image_bridge_diag.resync_drop_bytes += (uint32_t)recover_offset;
            CustomImageBridgeConsumePendingPrefix(recover_offset);
        }
        custom_image_bridge_wait_key_boundary = 0u;
        return;
    }

    if (custom_image_bridge_pending_len <= CUSTOM_IMAGE_BRIDGE_H264_TAIL_KEEP_BYTES) {
        return;
    }

    // 没找到关键边界时只保留最后几个字节，作用是允许 4 字节 start code 加 1 字节 NAL header 横跨两次 USB 回调；
    // 原因是 USB CDC 分包边界和 H.264 NAL 边界无关，尾部 00 00 00 01 或 NAL header 被切掉都会错过恢复点。
    bytes_to_drop = custom_image_bridge_pending_len - CUSTOM_IMAGE_BRIDGE_H264_TAIL_KEEP_BYTES;
    custom_image_bridge_diag.pending_drop_bytes += (uint32_t)bytes_to_drop;
    custom_image_bridge_diag.resync_drop_bytes += (uint32_t)bytes_to_drop;
    CustomImageBridgeConsumePendingPrefix(bytes_to_drop);
}

static void CustomImageBridgeMaybeResetOnUSBSilence(void)
{
    uint32_t now_tick = HAL_GetTick();

    if (custom_image_bridge_diag.last_usb_rx_tick_ms == 0u) {
        return;
    }

    if ((now_tick - custom_image_bridge_diag.last_usb_rx_tick_ms) < CUSTOM_IMAGE_BRIDGE_USB_SILENCE_RESET_MS) {
        return;
    }

    if (custom_image_bridge_last_silence_reset_rx_tick_ms == custom_image_bridge_diag.last_usb_rx_tick_ms) {
        return;
    }

    // USB 输入长时间静默后即使队列已经发空，也不能假设下一批字节仍接在同一个 H.264 上下文后面；
    // 这里把恢复点重新钉到 SPS/IDR，避免上位机重连或重启编码器后从半帧位置继续透传。
    custom_image_bridge_last_silence_reset_rx_tick_ms = custom_image_bridge_diag.last_usb_rx_tick_ms;
    CustomImageBridgeResetStreamState(CUSTOM_IMAGE_BRIDGE_RESET_USB_SILENCE);
}

static void CustomImageBridgeDrainUSBQueue(void)
{
    uint16_t recv_len = 0u;
    uint8_t read_index = 0u;
    uint32_t primask = 0u;

    while (1) {
        primask = CustomImageBridgeEnterCritical();
        if (custom_image_bridge_usb_queue_count == 0u) {
            CustomImageBridgeExitCritical(primask);
            break;
        }

        read_index = custom_image_bridge_usb_queue_read_index;
        recv_len = custom_image_bridge_usb_packet_lens[read_index];
        if (recv_len > CUSTOM_IMAGE_BRIDGE_USB_PACKET_MAX_BYTES) {
            recv_len = CUSTOM_IMAGE_BRIDGE_USB_PACKET_MAX_BYTES;
        }

        memcpy(custom_image_bridge_task_chunk, custom_image_bridge_usb_packet_slots[read_index], recv_len);
        custom_image_bridge_usb_queue_read_index =
            (uint8_t)((custom_image_bridge_usb_queue_read_index + 1u) % CUSTOM_IMAGE_BRIDGE_USB_PACKET_SLOT_COUNT);
        custom_image_bridge_usb_queue_count--;
        CustomImageBridgeExitCritical(primask);

        CustomImageBridgeAppendPendingBytes(custom_image_bridge_task_chunk, recv_len);
        CustomImageBridgeDropUntilRecoverPoint();
    }

    while (custom_image_bridge_wait_key_boundary == 0u &&
           custom_image_bridge_pending_len >= CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES) {
        if (custom_image_bridge_packet_queue_count >= CUSTOM_IMAGE_BRIDGE_PACKET_QUEUE_HIGH_WATERMARK) {
            CustomImageBridgeResetStreamState(CUSTOM_IMAGE_BRIDGE_RESET_PACKET_BACKLOG);
            break;
        }
        if (CustomImageBridgeEnqueuePacket(custom_image_bridge_pending_bytes) == 0u) {
            break;
        }
        CustomImageBridgeConsumePendingPrefix(CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES);
    }
}

static void CustomImageBridgePack0310Frame(const uint8_t *packet)
{
    custom_image_bridge_tx_frame[0] = REFEREE_SOF;
    custom_image_bridge_tx_frame[1] = (uint8_t)(CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES & 0xFFu);
    custom_image_bridge_tx_frame[2] = (uint8_t)((CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES >> 8) & 0xFFu);
    custom_image_bridge_tx_frame[3] = custom_image_bridge_tx_seq++;
    custom_image_bridge_tx_frame[4] =
        Get_CRC8_Check_Sum(custom_image_bridge_tx_frame, LEN_CRC8, 0xFFu);
    custom_image_bridge_tx_frame[5] = (uint8_t)(ID_robot_custom_data_2 & 0xFFu);
    custom_image_bridge_tx_frame[6] = (uint8_t)((ID_robot_custom_data_2 >> 8) & 0xFFu);
    memcpy(custom_image_bridge_tx_frame + DATA_Offset, packet, CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES);
    Append_CRC16_Check_Sum(custom_image_bridge_tx_frame, CUSTOM_IMAGE_BRIDGE_FRAME_BYTES);
}

static void CustomImageBridgeTrySend0310(void)
{
    HAL_StatusTypeDef tx_status;
    CustomImageBridgePacketSlot_s *packet_slot = NULL;
    uint32_t now_tick = HAL_GetTick();

    if (custom_image_bridge_packet_queue_count == 0u) {
        return;
    }

    if (huart6.gState != HAL_UART_STATE_READY) {
        if (custom_image_bridge_uart_busy_start_tick_ms == 0u) {
            custom_image_bridge_uart_busy_start_tick_ms = now_tick;
        }
        custom_image_bridge_diag.uart_busy_skip_count++;
        if ((now_tick - custom_image_bridge_uart_busy_start_tick_ms) >= CUSTOM_IMAGE_BRIDGE_UART_BUSY_RECOVER_MS) {
            // 只 abort USART6 的 TX 半边，作用是解开偶发 DMA 发送状态卡死，同时尽量不碰 USART6 RX 上的 VT03/键鼠输入；
            // 原因是用户观察到重启下位机会恢复，说明需要在板上给 TX busy 长尾状态一个轻量自愈出口。
            HAL_UART_AbortTransmit(&huart6);
            custom_image_bridge_diag.uart_tx_abort_count++;
            custom_image_bridge_uart_busy_start_tick_ms = 0u;
            CustomImageBridgeResetStreamState(CUSTOM_IMAGE_BRIDGE_RESET_UART_BUSY_TIMEOUT);
        }
        return;
    }
    custom_image_bridge_uart_busy_start_tick_ms = 0u;

    packet_slot = &custom_image_bridge_packet_queue[custom_image_bridge_packet_queue_read_index];
    CustomImageBridgePack0310Frame(packet_slot->bytes);
    tx_status = HAL_UART_Transmit_DMA(&huart6, custom_image_bridge_tx_frame, CUSTOM_IMAGE_BRIDGE_FRAME_BYTES);
    if (tx_status != HAL_OK) {
        custom_image_bridge_diag.uart_tx_error_count++;
        HAL_UART_AbortTransmit(&huart6);
        custom_image_bridge_diag.uart_tx_abort_count++;
        CustomImageBridgeResetStreamState(CUSTOM_IMAGE_BRIDGE_RESET_UART_TX_ERROR);
        return;
    }

    custom_image_bridge_diag.last_uart_tx_tick_ms = now_tick;
    custom_image_bridge_diag.uart_tx_packet_count++;
    custom_image_bridge_diag.uart_tx_bytes += CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES;

    custom_image_bridge_packet_queue_read_index =
        (uint8_t)((custom_image_bridge_packet_queue_read_index + 1u) % CUSTOM_IMAGE_BRIDGE_PACKET_QUEUE_SLOT_COUNT);
    custom_image_bridge_packet_queue_count--;
}

static void CustomImageBridgeMaybeLog(void)
{
    uint32_t now_tick = HAL_GetTick();
    uint32_t usb_rx_packets_delta = 0u;
    uint32_t uart_tx_packets_delta = 0u;
    uint32_t usb_silence_ms = 0u;
    uint32_t uart_tx_silence_ms = 0u;

    if ((now_tick - custom_image_bridge_diag.last_log_tick_ms) < CUSTOM_IMAGE_BRIDGE_LOG_INTERVAL_MS) {
        return;
    }

    usb_rx_packets_delta =
        custom_image_bridge_diag.usb_rx_packets - custom_image_bridge_diag.last_logged_usb_rx_packets;
    uart_tx_packets_delta =
        custom_image_bridge_diag.uart_tx_packet_count - custom_image_bridge_diag.last_logged_uart_tx_packet_count;

    if (custom_image_bridge_diag.last_usb_rx_tick_ms != 0u) {
        usb_silence_ms = now_tick - custom_image_bridge_diag.last_usb_rx_tick_ms;
    }

    if (custom_image_bridge_diag.last_uart_tx_tick_ms != 0u) {
        uart_tx_silence_ms = now_tick - custom_image_bridge_diag.last_uart_tx_tick_ms;
    }

    custom_image_bridge_diag.last_log_tick_ms = now_tick;
    custom_image_bridge_diag.last_logged_usb_rx_packets = custom_image_bridge_diag.usb_rx_packets;
    custom_image_bridge_diag.last_logged_uart_tx_packet_count = custom_image_bridge_diag.uart_tx_packet_count;

    if (usb_rx_packets_delta == 0u) {
        LOGWARNING("[img_bridge] 上位机视频流未接收 usb_pkt_total:%lu usb_bytes_total:%lu pending_raw:%u pkt_queue:%u last_rx_len:%u silence_ms:%lu tx0310_total:%lu tx_silence_ms:%lu pending_drop:%lu usb_drop:%lu resync:%lu resync_drop:%lu tx_abort:%lu reason:%s",
                   (unsigned long)custom_image_bridge_diag.usb_rx_packets,
                   (unsigned long)custom_image_bridge_diag.usb_rx_bytes,
                   (unsigned int)custom_image_bridge_pending_len,
                   (unsigned int)custom_image_bridge_packet_queue_count,
                   (unsigned int)custom_image_bridge_diag.last_usb_rx_len,
                   (unsigned long)usb_silence_ms,
                   (unsigned long)custom_image_bridge_diag.uart_tx_packet_count,
                   (unsigned long)uart_tx_silence_ms,
                   (unsigned long)custom_image_bridge_diag.pending_drop_bytes,
                   (unsigned long)custom_image_bridge_diag.usb_queue_drop_count,
                   (unsigned long)custom_image_bridge_diag.resync_count,
                   (unsigned long)custom_image_bridge_diag.resync_drop_bytes,
                   (unsigned long)custom_image_bridge_diag.uart_tx_abort_count,
                   CustomImageBridgeResetReasonName(custom_image_bridge_diag.last_reset_reason));
        return;
    }

    if (uart_tx_packets_delta == 0u) {
        LOGINFO("[img_bridge] 已收视频流 但本周期未转发0310 usb_pkt_delta:%lu usb_pkt_total:%lu pending_raw:%u pkt_queue:%u wait_key:%u busy_skip:%lu tx_err:%lu tx_abort:%lu pkt_drop:%lu pending_trim:%lu pending_drop:%lu resync:%lu reason:%s",
                (unsigned long)usb_rx_packets_delta,
                (unsigned long)custom_image_bridge_diag.usb_rx_packets,
                (unsigned int)custom_image_bridge_pending_len,
                (unsigned int)custom_image_bridge_packet_queue_count,
                (unsigned int)custom_image_bridge_wait_key_boundary,
                (unsigned long)custom_image_bridge_diag.uart_busy_skip_count,
                (unsigned long)custom_image_bridge_diag.uart_tx_error_count,
                (unsigned long)custom_image_bridge_diag.uart_tx_abort_count,
                (unsigned long)custom_image_bridge_diag.packet_queue_drop_count,
                (unsigned long)custom_image_bridge_diag.pending_trim_count,
                (unsigned long)custom_image_bridge_diag.pending_drop_bytes,
                (unsigned long)custom_image_bridge_diag.resync_count,
                CustomImageBridgeResetReasonName(custom_image_bridge_diag.last_reset_reason));
        return;
    }

    LOGINFO("[img_bridge] 通信正常 usb_pkt_delta:%lu usb_pkt_total:%lu usb_bytes_total:%lu pending_raw:%u pkt_queue:%u tx0310_delta:%lu tx0310_total:%lu tx_bytes_total:%lu pkt_drop:%lu pending_drop:%lu resync:%lu resync_drop:%lu tx_abort:%lu reason:%s",
            (unsigned long)usb_rx_packets_delta,
            (unsigned long)custom_image_bridge_diag.usb_rx_packets,
            (unsigned long)custom_image_bridge_diag.usb_rx_bytes,
            (unsigned int)custom_image_bridge_pending_len,
            (unsigned int)custom_image_bridge_packet_queue_count,
            (unsigned long)uart_tx_packets_delta,
            (unsigned long)custom_image_bridge_diag.uart_tx_packet_count,
            (unsigned long)custom_image_bridge_diag.uart_tx_bytes,
            (unsigned long)custom_image_bridge_diag.packet_queue_drop_count,
            (unsigned long)custom_image_bridge_diag.pending_drop_bytes,
            (unsigned long)custom_image_bridge_diag.resync_count,
            (unsigned long)custom_image_bridge_diag.resync_drop_bytes,
            (unsigned long)custom_image_bridge_diag.uart_tx_abort_count,
            CustomImageBridgeResetReasonName(custom_image_bridge_diag.last_reset_reason));
}

void CustomImageBridgeInit(void)
{
    USB_Init_Config_s usb_conf = { 0 };

    memset(&custom_image_bridge_diag, 0, sizeof(custom_image_bridge_diag));
    memset(custom_image_bridge_usb_packet_lens, 0, sizeof(custom_image_bridge_usb_packet_lens));
    memset(custom_image_bridge_packet_queue, 0, sizeof(custom_image_bridge_packet_queue));
    custom_image_bridge_usb_queue_read_index = 0u;
    custom_image_bridge_usb_queue_write_index = 0u;
    custom_image_bridge_usb_queue_count = 0u;
    custom_image_bridge_packet_queue_read_index = 0u;
    custom_image_bridge_packet_queue_write_index = 0u;
    custom_image_bridge_packet_queue_count = 0u;
    custom_image_bridge_pending_len = 0u;
    custom_image_bridge_tx_seq = 0u;
    custom_image_bridge_reset_requested = 0u;
    custom_image_bridge_requested_reset_reason = CUSTOM_IMAGE_BRIDGE_RESET_NONE;
    custom_image_bridge_wait_key_boundary = 1u;
    custom_image_bridge_uart_busy_start_tick_ms = 0u;
    custom_image_bridge_last_silence_reset_rx_tick_ms = 0u;

    usb_conf.rx_cbk = CustomImageBridgeUSBRxCallback;
    custom_image_bridge_usb_rx_buffer = USBInit(usb_conf);

    LOGINFO("[img_bridge] init ok, usb_rx_buf=%p, raw_h264_chunk=%uB, referee_payload=%uB",
            custom_image_bridge_usb_rx_buffer,
            (unsigned int)CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES,
            (unsigned int)CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES);
}

void CustomImageBridgeTask(void)
{
    CustomImageBridgeApplyRequestedReset();
    CustomImageBridgeMaybeResetOnUSBSilence();
    CustomImageBridgeDrainUSBQueue();
    CustomImageBridgeTrySend0310();
    CustomImageBridgeMaybeLog();
}
