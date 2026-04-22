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
#define CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES LEN_robot_custom_data_2
#define CUSTOM_IMAGE_BRIDGE_PROTOCOL_PAYLOAD_BYTES 300u
#define CUSTOM_IMAGE_BRIDGE_FRAME_BYTES (LEN_HEADER + LEN_CMDID + LEN_robot_custom_data_2 + LEN_TAIL)
#define CUSTOM_IMAGE_BRIDGE_LOG_INTERVAL_MS 1000u

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
    uint32_t last_logged_usb_rx_packets; // 记录上一次打印日志时的累计 USB 包数
    uint32_t last_logged_uart_tx_packet_count; // 记录上一次打印日志时的累计 0x0310 包数
    uint32_t last_log_tick_ms; // 记录上次打印诊断的时间，用于日志限频
} CustomImageBridgeDiag_s;

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

static void CustomImageBridgeDropOldestPacket(void)
{
    if (custom_image_bridge_packet_queue_count == 0u) {
        return;
    }

    custom_image_bridge_packet_queue_read_index =
        (uint8_t)((custom_image_bridge_packet_queue_read_index + 1u) % CUSTOM_IMAGE_BRIDGE_PACKET_QUEUE_SLOT_COUNT);
    custom_image_bridge_packet_queue_count--;
    custom_image_bridge_diag.packet_queue_drop_count++;
}

static void CustomImageBridgeEnqueuePacket(const uint8_t *packet)
{
    uint8_t write_index = custom_image_bridge_packet_queue_write_index;

    if (packet == NULL) {
        return;
    }

    if (custom_image_bridge_packet_queue_count >= CUSTOM_IMAGE_BRIDGE_PACKET_QUEUE_SLOT_COUNT) {
        CustomImageBridgeDropOldestPacket();
    }

    write_index = custom_image_bridge_packet_queue_write_index;
    memcpy(custom_image_bridge_packet_queue[write_index].bytes, packet, CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES);
    custom_image_bridge_packet_queue_write_index =
        (uint8_t)((write_index + 1u) % CUSTOM_IMAGE_BRIDGE_PACKET_QUEUE_SLOT_COUNT);
    custom_image_bridge_packet_queue_count++;
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
        data += (valid_len - CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES);
        valid_len = CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES;
    }

    if (custom_image_bridge_pending_len + valid_len > CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES) {
        size_t bytes_to_drop =
            custom_image_bridge_pending_len + valid_len - CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES;

        if (bytes_to_drop >= custom_image_bridge_pending_len) {
            custom_image_bridge_pending_len = 0u;
        } else {
            memmove(custom_image_bridge_pending_bytes,
                    custom_image_bridge_pending_bytes + bytes_to_drop,
                    custom_image_bridge_pending_len - bytes_to_drop);
            custom_image_bridge_pending_len -= bytes_to_drop;
        }

        custom_image_bridge_diag.pending_trim_count++;
        custom_image_bridge_diag.pending_drop_bytes += (uint32_t)bytes_to_drop;
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
    }

    while (custom_image_bridge_pending_len >= CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES) {
        CustomImageBridgeEnqueuePacket(custom_image_bridge_pending_bytes);
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

    if (custom_image_bridge_packet_queue_count == 0u) {
        return;
    }

    if (huart6.gState != HAL_UART_STATE_READY) {
        custom_image_bridge_diag.uart_busy_skip_count++;
        return;
    }

    packet_slot = &custom_image_bridge_packet_queue[custom_image_bridge_packet_queue_read_index];
    CustomImageBridgePack0310Frame(packet_slot->bytes);
    tx_status = HAL_UART_Transmit_DMA(&huart6, custom_image_bridge_tx_frame, CUSTOM_IMAGE_BRIDGE_FRAME_BYTES);
    if (tx_status != HAL_OK) {
        custom_image_bridge_diag.uart_tx_error_count++;
        return;
    }

    custom_image_bridge_diag.last_uart_tx_tick_ms = HAL_GetTick();
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
        LOGWARNING("[img_bridge] 上位机视频流未接收 usb_pkt_total:%lu usb_bytes_total:%lu pending_raw:%u pkt_queue:%u last_rx_len:%u silence_ms:%lu tx0310_total:%lu tx_silence_ms:%lu pending_drop:%lu usb_drop:%lu",
                   (unsigned long)custom_image_bridge_diag.usb_rx_packets,
                   (unsigned long)custom_image_bridge_diag.usb_rx_bytes,
                   (unsigned int)custom_image_bridge_pending_len,
                   (unsigned int)custom_image_bridge_packet_queue_count,
                   (unsigned int)custom_image_bridge_diag.last_usb_rx_len,
                   (unsigned long)usb_silence_ms,
                   (unsigned long)custom_image_bridge_diag.uart_tx_packet_count,
                   (unsigned long)uart_tx_silence_ms,
                   (unsigned long)custom_image_bridge_diag.pending_drop_bytes,
                   (unsigned long)custom_image_bridge_diag.usb_queue_drop_count);
        return;
    }

    if (uart_tx_packets_delta == 0u) {
        LOGINFO("[img_bridge] 已收视频流 但本周期未转发0310 usb_pkt_delta:%lu usb_pkt_total:%lu pending_raw:%u pkt_queue:%u busy_skip:%lu tx_err:%lu pkt_drop:%lu pending_trim:%lu pending_drop:%lu",
                (unsigned long)usb_rx_packets_delta,
                (unsigned long)custom_image_bridge_diag.usb_rx_packets,
                (unsigned int)custom_image_bridge_pending_len,
                (unsigned int)custom_image_bridge_packet_queue_count,
                (unsigned long)custom_image_bridge_diag.uart_busy_skip_count,
                (unsigned long)custom_image_bridge_diag.uart_tx_error_count,
                (unsigned long)custom_image_bridge_diag.packet_queue_drop_count,
                (unsigned long)custom_image_bridge_diag.pending_trim_count,
                (unsigned long)custom_image_bridge_diag.pending_drop_bytes);
        return;
    }

    LOGINFO("[img_bridge] 通信正常 usb_pkt_delta:%lu usb_pkt_total:%lu usb_bytes_total:%lu pending_raw:%u pkt_queue:%u tx0310_delta:%lu tx0310_total:%lu tx_bytes_total:%lu pkt_drop:%lu pending_drop:%lu",
            (unsigned long)usb_rx_packets_delta,
            (unsigned long)custom_image_bridge_diag.usb_rx_packets,
            (unsigned long)custom_image_bridge_diag.usb_rx_bytes,
            (unsigned int)custom_image_bridge_pending_len,
            (unsigned int)custom_image_bridge_packet_queue_count,
            (unsigned long)uart_tx_packets_delta,
            (unsigned long)custom_image_bridge_diag.uart_tx_packet_count,
            (unsigned long)custom_image_bridge_diag.uart_tx_bytes,
            (unsigned long)custom_image_bridge_diag.packet_queue_drop_count,
            (unsigned long)custom_image_bridge_diag.pending_drop_bytes);
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

    usb_conf.rx_cbk = CustomImageBridgeUSBRxCallback;
    custom_image_bridge_usb_rx_buffer = USBInit(usb_conf);

    LOGINFO("[img_bridge] init ok, usb_rx_buf=%p, raw_h264_chunk=%uB, referee_payload=%uB",
            custom_image_bridge_usb_rx_buffer,
            (unsigned int)CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES,
            (unsigned int)CUSTOM_IMAGE_BRIDGE_CHUNK_BYTES);
}

void CustomImageBridgeTask(void)
{
    CustomImageBridgeDrainUSBQueue();
    CustomImageBridgeTrySend0310();
    CustomImageBridgeMaybeLog();
}
