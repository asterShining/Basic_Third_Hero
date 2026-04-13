#include "custom_image_bridge.h"

#include "bsp_log.h"
#include "bsp_usb.h"
#include "crc_ref.h"
#include "main.h"
#include "referee_protocol.h"
#include "string.h"
#include "usart.h"

#define CUSTOM_IMAGE_BRIDGE_USB_PACKET_SLOT_COUNT 4u
#define CUSTOM_IMAGE_BRIDGE_USB_PACKET_MAX_BYTES APP_RX_DATA_SIZE
#define CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES 8192u
#define CUSTOM_IMAGE_BRIDGE_PAYLOAD_BYTES LEN_robot_custom_data_2
#define CUSTOM_IMAGE_BRIDGE_FRAME_BYTES (LEN_HEADER + LEN_CMDID + LEN_robot_custom_data_2 + LEN_TAIL)
#define CUSTOM_IMAGE_BRIDGE_LOG_INTERVAL_MS 1000u

typedef struct
{
    uint32_t usb_rx_packets;         // 记录 USB RX 包数，便于确认上位机是否持续送流
    uint32_t usb_rx_bytes;           // 记录 USB RX 字节数，便于核对链路吞吐
    uint16_t last_usb_rx_len;        // 记录最近一次 USB RX 包长，便于确认上位机当前是否真的在送有效负载
    uint32_t last_usb_rx_tick_ms;    // 记录最近一次 USB RX 时刻，便于区分“刚启动还没来包”和“已经断流”
    uint32_t usb_queue_drop_count;   // 记录 USB 队列挤爆时丢弃旧包的次数
    uint32_t pending_reset_count;    // 记录任务侧原始字节积压过多后清空旧数据的次数
    uint32_t uart_tx_frame_count;    // 记录成功下发的 0x0310 帧数
    uint32_t last_uart_tx_tick_ms;   // 记录最近一次 0x0310 成功下发时刻，便于判断桥接是否只收不发
    uint32_t uart_busy_skip_count;   // 记录因 USART6 TX 忙而本周期跳过发送的次数
    uint32_t uart_tx_error_count;    // 记录 HAL_UART_Transmit_DMA 返回失败的次数
    uint32_t last_logged_usb_rx_packets; // 记录上一次打印日志时的累计 USB 包数，便于统计“本周期有没有新数据”
    uint32_t last_logged_uart_tx_frame_count; // 记录上一次打印日志时的累计 0x0310 帧数，便于统计“本周期有没有成功转发”
    uint32_t last_log_tick_ms;       // 记录上次打印诊断的时间，用于日志限频
} CustomImageBridgeDiag_s;

static uint8_t *custom_image_bridge_usb_rx_buffer; // USB CDC 底层接收缓冲区首地址，由 USBInit 返回并长期复用
static uint8_t custom_image_bridge_usb_packet_slots[CUSTOM_IMAGE_BRIDGE_USB_PACKET_SLOT_COUNT][CUSTOM_IMAGE_BRIDGE_USB_PACKET_MAX_BYTES];
static uint16_t custom_image_bridge_usb_packet_lens[CUSTOM_IMAGE_BRIDGE_USB_PACKET_SLOT_COUNT];
static volatile uint8_t custom_image_bridge_usb_queue_read_index;
static volatile uint8_t custom_image_bridge_usb_queue_write_index;
static volatile uint8_t custom_image_bridge_usb_queue_count;
static uint8_t custom_image_bridge_task_chunk[CUSTOM_IMAGE_BRIDGE_USB_PACKET_MAX_BYTES]; // 任务侧暂存一整包 USB 数据，避免直接操作共享槽位
static uint8_t custom_image_bridge_pending_bytes[CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES]; // 任务侧连续原始码流缓存，只由任务上下文访问
static size_t custom_image_bridge_pending_len;
static uint8_t custom_image_bridge_tx_frame[CUSTOM_IMAGE_BRIDGE_FRAME_BYTES]; // USART6 DMA 发送缓冲，发送完成前不得改写
static uint8_t custom_image_bridge_tx_seq;
static CustomImageBridgeDiag_s custom_image_bridge_diag;

static uint32_t CustomImageBridgeEnterCritical(void)
{
    // 这里手动保存并关闭全局中断，作用是把 USB 回调和周期任务之间共享的队列索引保护起来；
    // 原因是该模块的生产者来自 USB 中断，消费者来自普通任务，若不做最小粒度保护就会出现索引撕裂和重复消费。
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void CustomImageBridgeExitCritical(uint32_t primask)
{
    // 这里按进入前状态恢复中断，作用是避免把外层已经关闭中断的调用链意外重新打开；
    // 原因是该工具函数可能同时被任务上下文和中断上下文复用，必须保留原有中断屏蔽语义。
    if (primask == 0u) {
        __enable_irq();
    }
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
        // 这里把异常长度裁剪到静态槽位上限，作用是避免底层长度异常时 memcpy 越界；
        // 原因是 USB CDC 缓冲区大小由 Cube 配置控制，桥接模块必须用自己的边界兜住。
        valid_len = CUSTOM_IMAGE_BRIDGE_USB_PACKET_MAX_BYTES;
    }

    primask = CustomImageBridgeEnterCritical();
    write_index = custom_image_bridge_usb_queue_write_index;
    if (custom_image_bridge_usb_queue_count >= CUSTOM_IMAGE_BRIDGE_USB_PACKET_SLOT_COUNT) {
        // 这里在队列满时直接丢掉最旧的一包，作用是把系统行为固定成“保最新，不反压”；
        // 原因是图像桥接只需要低时延，不允许因为旧数据堆积去影响控制链路和 USB 中断处理。
        custom_image_bridge_usb_queue_read_index =
            (uint8_t)((custom_image_bridge_usb_queue_read_index + 1u) % CUSTOM_IMAGE_BRIDGE_USB_PACKET_SLOT_COUNT);
        custom_image_bridge_usb_queue_count--;
        custom_image_bridge_diag.usb_queue_drop_count++;
    }
    CustomImageBridgeExitCritical(primask);

    // 这里先把本次 USB 包拷进尚未发布的静态槽位，作用是确保任务侧永远只会看到已经完整拷贝好的数据；
    // 原因是如果先发布索引再拷数据，任务线程可能读到一半旧数据一半新数据的坏包。
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
        // 这里当单包本身就超过任务缓存上限时，只保留尾部最新字节，作用是避免超大包直接把缓存和时延一起拖垮；
        // 原因是 H.264 码流是时序流，旧字节价值最低，保留最近的数据更符合“低延迟优先”的策略。
        data += (valid_len - CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES);
        valid_len = CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES;
    }

    if (custom_image_bridge_pending_len + valid_len > CUSTOM_IMAGE_BRIDGE_PENDING_BUFFER_BYTES) {
        // 这里任务侧积压过多时直接清空旧字节，作用是把系统恢复到“从最新输入重新对齐”的状态；
        // 原因是继续堆着旧图像只会把延迟越拖越大，而客户端和解码器都更接受短时掉帧而不是秒级滞后。
        custom_image_bridge_pending_len = 0u;
        custom_image_bridge_diag.pending_reset_count++;
    }

    memcpy(custom_image_bridge_pending_bytes + custom_image_bridge_pending_len, data, valid_len);
    custom_image_bridge_pending_len += valid_len;
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
            return;
        }

        read_index = custom_image_bridge_usb_queue_read_index;
        recv_len = custom_image_bridge_usb_packet_lens[read_index];
        if (recv_len > CUSTOM_IMAGE_BRIDGE_USB_PACKET_MAX_BYTES) {
            recv_len = CUSTOM_IMAGE_BRIDGE_USB_PACKET_MAX_BYTES;
        }

        // 这里在临界区内把共享槽位复制到任务侧私有缓冲，作用是避免生产者在任务还没处理完时重用同一个槽位；
        // 原因是 USB 回调和任务是并发关系，而静态槽位数量有限，必须在释放槽位前先把内容搬走。
        memcpy(custom_image_bridge_task_chunk, custom_image_bridge_usb_packet_slots[read_index], recv_len);
        custom_image_bridge_usb_queue_read_index =
            (uint8_t)((custom_image_bridge_usb_queue_read_index + 1u) % CUSTOM_IMAGE_BRIDGE_USB_PACKET_SLOT_COUNT);
        custom_image_bridge_usb_queue_count--;
        CustomImageBridgeExitCritical(primask);

        CustomImageBridgeAppendPendingBytes(custom_image_bridge_task_chunk, recv_len);
    }
}

static void CustomImageBridgePack0310Frame(void)
{
    custom_image_bridge_tx_frame[0] = REFEREE_SOF;
    custom_image_bridge_tx_frame[1] = (uint8_t)(CUSTOM_IMAGE_BRIDGE_PAYLOAD_BYTES & 0xFFu);
    custom_image_bridge_tx_frame[2] = (uint8_t)((CUSTOM_IMAGE_BRIDGE_PAYLOAD_BYTES >> 8) & 0xFFu);
    custom_image_bridge_tx_frame[3] = custom_image_bridge_tx_seq++;
    custom_image_bridge_tx_frame[4] =
        Get_CRC8_Check_Sum(custom_image_bridge_tx_frame, LEN_CRC8, 0xFFu);

    // 这里按小端写入官方 0x0310 命令字，作用是让 USART6 发出的整帧严格符合最新协议；
    // 原因是上位机现在只负责送原始 H.264，官方交互层封包必须在云台板上完成，不能再沿用 PC 侧伪造的旧实现。
    custom_image_bridge_tx_frame[5] = (uint8_t)(ID_robot_custom_data_2 & 0xFFu);
    custom_image_bridge_tx_frame[6] = (uint8_t)((ID_robot_custom_data_2 >> 8) & 0xFFu);

    memcpy(custom_image_bridge_tx_frame + DATA_Offset, custom_image_bridge_pending_bytes,
           CUSTOM_IMAGE_BRIDGE_PAYLOAD_BYTES);
    Append_CRC16_Check_Sum(custom_image_bridge_tx_frame, CUSTOM_IMAGE_BRIDGE_FRAME_BYTES);
}

static void CustomImageBridgeTrySend0310(void)
{
    HAL_StatusTypeDef tx_status;

    if (custom_image_bridge_pending_len < CUSTOM_IMAGE_BRIDGE_PAYLOAD_BYTES) {
        return;
    }

    if (huart6.gState != HAL_UART_STATE_READY) {
        // 这里在 USART6 TX 忙时直接跳过本周期发送，作用是保证图传键鼠 RX 和已有 TX DMA 流程完全优先于图像桥接；
        // 原因是这条链路的设计目标就是“宁可丢旧图像，也绝不反压控制口”。
        custom_image_bridge_diag.uart_busy_skip_count++;
        return;
    }

    CustomImageBridgePack0310Frame();
    tx_status = HAL_UART_Transmit_DMA(&huart6, custom_image_bridge_tx_frame, CUSTOM_IMAGE_BRIDGE_FRAME_BYTES);
    if (tx_status != HAL_OK) {
        // 这里把 HAL 返回失败单独记账，作用是区分“串口只是忙”与“底层 DMA/状态机真的发起失败”；
        // 原因是这两类问题对应的排障方向完全不同，必须拆开看。
        custom_image_bridge_diag.uart_tx_error_count++;
        return;
    }

    memmove(custom_image_bridge_pending_bytes,
            custom_image_bridge_pending_bytes + CUSTOM_IMAGE_BRIDGE_PAYLOAD_BYTES,
            custom_image_bridge_pending_len - CUSTOM_IMAGE_BRIDGE_PAYLOAD_BYTES);
    custom_image_bridge_pending_len -= CUSTOM_IMAGE_BRIDGE_PAYLOAD_BYTES;
    custom_image_bridge_diag.uart_tx_frame_count++;
    custom_image_bridge_diag.last_uart_tx_tick_ms = HAL_GetTick();
}

static void CustomImageBridgeMaybeLog(void)
{
    uint32_t now_tick = HAL_GetTick();
    uint32_t usb_rx_packets_delta = 0u;
    uint32_t uart_tx_frames_delta = 0u;
    uint32_t usb_silence_ms = 0u;
    uint32_t uart_tx_silence_ms = 0u;

    if ((now_tick - custom_image_bridge_diag.last_log_tick_ms) < CUSTOM_IMAGE_BRIDGE_LOG_INTERVAL_MS) {
        return;
    }

    usb_rx_packets_delta =
        custom_image_bridge_diag.usb_rx_packets - custom_image_bridge_diag.last_logged_usb_rx_packets;
    uart_tx_frames_delta =
        custom_image_bridge_diag.uart_tx_frame_count - custom_image_bridge_diag.last_logged_uart_tx_frame_count;

    if (custom_image_bridge_diag.last_usb_rx_tick_ms != 0u) {
        usb_silence_ms = now_tick - custom_image_bridge_diag.last_usb_rx_tick_ms;
    }

    if (custom_image_bridge_diag.last_uart_tx_tick_ms != 0u) {
        uart_tx_silence_ms = now_tick - custom_image_bridge_diag.last_uart_tx_tick_ms;
    }

    // 这里把桥接关键计数按秒级打印，作用是现场联调时一眼看出“USB 有没有进来、0x0310 有没有发出去、是否在丢数据”；
    // 原因是这条链路跨三端，若没有中间态计数，任何一端黑屏都会变成纯猜测。
    custom_image_bridge_diag.last_log_tick_ms = now_tick;
    custom_image_bridge_diag.last_logged_usb_rx_packets = custom_image_bridge_diag.usb_rx_packets;
    custom_image_bridge_diag.last_logged_uart_tx_frame_count = custom_image_bridge_diag.uart_tx_frame_count;

    if (usb_rx_packets_delta == 0u) {
        // 这里单独把“本周期完全没收到上位机数据”打成告警，作用是让联调时第一眼就能确认 USB CDC 是否真的接通；
        // 原因是若仍沿用总计数日志，启动后长期静止会和“只是计数没变化但链路本来通过”混在一起，不利于现场判断。
        LOGWARNING("[img_bridge] 上位机数据未接收 usb_pkt_total:%lu usb_bytes_total:%lu queued:%u pending:%u last_rx_len:%u silence_ms:%lu tx0310_total:%lu tx_silence_ms:%lu",
                   (unsigned long)custom_image_bridge_diag.usb_rx_packets,
                   (unsigned long)custom_image_bridge_diag.usb_rx_bytes,
                   (unsigned int)custom_image_bridge_usb_queue_count,
                   (unsigned int)custom_image_bridge_pending_len,
                   (unsigned int)custom_image_bridge_diag.last_usb_rx_len,
                   (unsigned long)usb_silence_ms,
                   (unsigned long)custom_image_bridge_diag.uart_tx_frame_count,
                   (unsigned long)uart_tx_silence_ms);
        return;
    }

    if (uart_tx_frames_delta == 0u) {
        // 这里把“已经收到上位机数据但本周期没成功发出 0x0310”单独打印，作用是区分问题卡在 USB 入口还是卡在 USART6/300B 聚合出口；
        // 原因是这两类故障的排查方向完全不同，必须在日志层面直接分叉。
        LOGINFO("[img_bridge] 上位机数据已接收 但本周期未转发0310 usb_pkt_delta:%lu usb_pkt_total:%lu last_rx_len:%u queued:%u pending:%u busy_skip:%lu tx_err:%lu queue_drop:%lu pending_reset:%lu",
                (unsigned long)usb_rx_packets_delta,
                (unsigned long)custom_image_bridge_diag.usb_rx_packets,
                (unsigned int)custom_image_bridge_diag.last_usb_rx_len,
                (unsigned int)custom_image_bridge_usb_queue_count,
                (unsigned int)custom_image_bridge_pending_len,
                (unsigned long)custom_image_bridge_diag.uart_busy_skip_count,
                (unsigned long)custom_image_bridge_diag.uart_tx_error_count,
                (unsigned long)custom_image_bridge_diag.usb_queue_drop_count,
                (unsigned long)custom_image_bridge_diag.pending_reset_count);
        return;
    }

    // 这里把“本周期既收到了上位机数据，也成功发出了 0x0310”作为正常通信态打印，作用是让联调时快速确认双向桥接已经贯通；
    // 原因是用户当前最关心的就是两端是否接通，因此正常态也必须明确可见，而不能只在异常时打印。
    LOGINFO("[img_bridge] 通信正常 usb_pkt_delta:%lu usb_pkt_total:%lu usb_bytes_total:%lu last_rx_len:%u queued:%u pending:%u tx0310_delta:%lu tx0310_total:%lu busy_skip:%lu tx_err:%lu",
            (unsigned long)usb_rx_packets_delta,
            (unsigned long)custom_image_bridge_diag.usb_rx_packets,
            (unsigned long)custom_image_bridge_diag.usb_rx_bytes,
            (unsigned int)custom_image_bridge_diag.last_usb_rx_len,
            (unsigned int)custom_image_bridge_usb_queue_count,
            (unsigned int)custom_image_bridge_pending_len,
            (unsigned long)uart_tx_frames_delta,
            (unsigned long)custom_image_bridge_diag.uart_tx_frame_count,
            (unsigned long)custom_image_bridge_diag.uart_busy_skip_count,
            (unsigned long)custom_image_bridge_diag.uart_tx_error_count);
}

void CustomImageBridgeInit(void)
{
    USB_Init_Config_s usb_conf = { 0 };

    memset(&custom_image_bridge_diag, 0, sizeof(custom_image_bridge_diag));
    memset(custom_image_bridge_usb_packet_lens, 0, sizeof(custom_image_bridge_usb_packet_lens));
    custom_image_bridge_usb_queue_read_index = 0u;
    custom_image_bridge_usb_queue_write_index = 0u;
    custom_image_bridge_usb_queue_count = 0u;
    custom_image_bridge_pending_len = 0u;
    custom_image_bridge_tx_seq = 0u;

    // 这里把 USB CDC 的接收回调独立注册给图像桥接模块，作用是让上位机送来的原始 H.264 字节直接进入桥接缓存；
    // 原因是当前 VisionInit(VCP) 默认未启用，而图像桥接必须独立占用 USB CDC，不能混到现有控制协议里。
    usb_conf.rx_cbk = CustomImageBridgeUSBRxCallback;
    custom_image_bridge_usb_rx_buffer = USBInit(usb_conf);

    LOGINFO("[img_bridge] init ok, usb_rx_buf=%p, payload=%u, frame=%u",
            custom_image_bridge_usb_rx_buffer,
            (unsigned int)CUSTOM_IMAGE_BRIDGE_PAYLOAD_BYTES,
            (unsigned int)CUSTOM_IMAGE_BRIDGE_FRAME_BYTES);
}

void CustomImageBridgeTask(void)
{
    // 这里先把 USB 回调堆积的包全部转成任务侧连续字节流，作用是把中断上下文里的工作压缩到最小；
    // 原因是 USB 回调只负责“快收”，而真正的聚合、丢旧和 0x0310 封包必须放到低优先级任务里做。
    CustomImageBridgeDrainUSBQueue();

    // 这里每个周期最多尝试发送一帧 300B 的 0x0310，作用是把发送频率锁死在任务周期 20ms，也就是协议允许的 50Hz 上限；
    // 原因是这条链路不需要追求满带宽突发，只需要稳定、低时延、且不抢控制资源。
    CustomImageBridgeTrySend0310();
    CustomImageBridgeMaybeLog();
}
