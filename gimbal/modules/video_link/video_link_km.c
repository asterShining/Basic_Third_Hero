#include "video_link_km.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "crc_ref.h"
#include "daemon.h"
#include "memory.h"
#include <stdbool.h>

#define VIDEO_LINK_KM_RX_BUFFER_SIZE 64u
#define VIDEO_LINK_KM_LOG_INTERVAL 50u
#define VIDEO_LINK_KM_FRAME_SIZE 21u
#define VIDEO_LINK_KM_FRAME_HEADER_0 0xA9u
#define VIDEO_LINK_KM_FRAME_HEADER_1 0x53u
// What: 定义 VT03 在线判定重载拍数；Why: DaemonTask 以 100Hz 运行，原来的 10 拍只有约 100ms 容错，串口偶发抖动就会把主控误判成离线。
#define VIDEO_LINK_KM_DAEMON_RELOAD_COUNT 40u
// What: 定义 VT03 上电初始在线宽限拍数；Why: 让初始化阶段和运行阶段使用同一套约 400ms 时间窗，避免刚上线时在线判定更苛刻。
#define VIDEO_LINK_KM_DAEMON_INIT_COUNT 40u

#define VIDEO_LINK_KM_CH0_BIT_OFFSET 16u
#define VIDEO_LINK_KM_CH1_BIT_OFFSET 27u
#define VIDEO_LINK_KM_CH2_BIT_OFFSET 38u
#define VIDEO_LINK_KM_CH3_BIT_OFFSET 49u
#define VIDEO_LINK_KM_MODE_SW_BIT_OFFSET 60u
#define VIDEO_LINK_KM_PAUSE_BIT_OFFSET 62u
#define VIDEO_LINK_KM_FN_LEFT_BIT_OFFSET 63u
#define VIDEO_LINK_KM_FN_RIGHT_BIT_OFFSET 64u
#define VIDEO_LINK_KM_WHEEL_BIT_OFFSET 65u
#define VIDEO_LINK_KM_TRIGGER_BIT_OFFSET 76u
#define VIDEO_LINK_KM_MOUSE_X_BIT_OFFSET 80u
#define VIDEO_LINK_KM_MOUSE_Y_BIT_OFFSET 96u
#define VIDEO_LINK_KM_MOUSE_Z_BIT_OFFSET 112u
#define VIDEO_LINK_KM_MOUSE_LEFT_BIT_OFFSET 128u
#define VIDEO_LINK_KM_MOUSE_RIGHT_BIT_OFFSET 130u
#define VIDEO_LINK_KM_MOUSE_MID_BIT_OFFSET 132u
#define VIDEO_LINK_KM_KEYBOARD_BIT_OFFSET 136u

typedef struct
{
    int16_t rocker_l_;
    int16_t rocker_l1;
    int16_t rocker_r_;
    int16_t rocker_r1;
    int16_t dial;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint16_t keyboard_value;
    uint8_t left_button_down;
    uint8_t right_button_down;
    uint8_t mid_button_down;
    uint8_t mode_sw;
    uint8_t pause_button_down;
    uint8_t fn_left_button_down;
    uint8_t fn_right_button_down;
    uint8_t trigger_button_down;
} VideoLinkKM_Decode_s;

static RC_ctrl_t video_link_ctrl[2];
static USARTInstance *video_link_usart_instance;
static DaemonInstance *video_link_daemon_instance;
static VideoLinkKM_Diag_s video_link_diag;
static VideoLinkKM_RemoteState_s video_link_remote_state;
static uint8_t video_link_init_flag = 0u;
static uint8_t video_link_has_valid_frame = 0u;

/**
 * @brief 清空本拍的图传键鼠控制量
 *
 * @note 只清控制态,不清 key_count 统计
 */
static void VideoLinkClearCurrentControl(void)
{
    // 这里每拍先清空当前控制态，作用是让坏帧和断链都立即回零；
    // 原因是图传键鼠是高优先级输入，若沿用上一拍残留会直接把旧运动命令带进 `robot_cmd`。
    memset(&video_link_ctrl[TEMP].rc, 0, sizeof(video_link_ctrl[TEMP].rc));
    memset(&video_link_ctrl[TEMP].mouse, 0, sizeof(video_link_ctrl[TEMP].mouse));
    memset(video_link_ctrl[TEMP].key, 0, sizeof(video_link_ctrl[TEMP].key));
}

/**
 * @brief 按小端位序提取无符号字段
 *
 * @param frame 原始 21 字节图传帧
 * @param bit_offset 起始 bit 偏移
 * @param bit_length 字段 bit 长度
 * @return uint32_t 提取出的字段值
 */
static uint32_t VideoLinkExtractBitsLE(const uint8_t *frame, uint16_t bit_offset, uint8_t bit_length)
{
    uint32_t value = 0u;

    // 这里显式按 bit 偏移提取字段，作用是摆脱编译器位域布局的不确定性；
    // 原因是 VT03 官方样例虽然用了位域结构体，但机器人侧更适合用协议表的 bit 偏移做稳定解包。
    for (uint8_t bit_index = 0u; bit_index < bit_length; ++bit_index) {
        uint16_t current_bit = (uint16_t)(bit_offset + bit_index);
        uint8_t byte_index = (uint8_t)(current_bit >> 3);
        uint8_t bit_in_byte = (uint8_t)(current_bit & 0x07u);

        value |= (uint32_t)(((frame[byte_index] >> bit_in_byte) & 0x01u) << bit_index);
    }

    return value;
}

/**
 * @brief 读取小端有符号 16 bit 字段
 *
 * @param frame 原始 21 字节图传帧
 * @param bit_offset 起始 bit 偏移
 * @return int16_t 解出的有符号值
 */
static int16_t VideoLinkExtractSigned16LE(const uint8_t *frame, uint16_t bit_offset)
{
    uint16_t raw_value = (uint16_t)VideoLinkExtractBitsLE(frame, bit_offset, 16u);

    // 这里统一把鼠标轴按 int16 小端解释，作用是与 VT03 官方 21 字节定义严格一致；
    // 原因是三个鼠标轴都占整 16 bit，直接转成 `int16_t` 比再做额外符号扩展更直接也更稳。
    return (int16_t)raw_value;
}

/**
 * @brief 校验 VT03 固定长度图传帧
 *
 * @param frame 指向 21 字节完整帧
 * @return uint8_t 1:校验通过 0:校验失败
 */
static uint8_t VideoLinkVerifyFrame(const uint8_t *frame)
{
    // 这里先校验固定帧头，再走 CRC16，作用是快速排除噪声和错位数据；
    // 原因是 `USART1` 可能一次收到多帧或杂散字节，必须先用 `0xA9 0x53` 锁住 VT03 真实帧边界。
    if (frame == NULL || frame[0] != VIDEO_LINK_KM_FRAME_HEADER_0 || frame[1] != VIDEO_LINK_KM_FRAME_HEADER_1) {
        return 0u;
    }

    // 这里沿用 DJI 官方 VT03 示例代码同款 CRC16 校验，作用是让机器人侧实现与真链路样例保持一致；
    // 原因是手册文字描述和样例命名存在差异，而官方给出的数据帧示例代码更贴近实际出帧格式。
    return (uint8_t)Verify_CRC16_Check_Sum((uint8_t *)frame, VIDEO_LINK_KM_FRAME_SIZE);
}

/**
 * @brief 解包一帧 VT03 图传键鼠数据
 *
 * @param frame 指向 21 字节完整帧
 * @param decode_out 输出解包结果
 * @return uint8_t 1:解包成功 0:字段非法
 */
static uint8_t VideoLinkDecodeFrame(const uint8_t *frame, VideoLinkKM_Decode_s *decode_out)
{
    uint32_t ch0;
    uint32_t ch1;
    uint32_t ch2;
    uint32_t ch3;
    uint32_t wheel;
    uint32_t mode_sw;
    uint32_t pause_button;
    uint32_t fn_left_button;
    uint32_t fn_right_button;
    uint32_t trigger_button;
    uint32_t mouse_left_button;
    uint32_t mouse_right_button;
    uint32_t mouse_mid_button;

    if (frame == NULL || decode_out == NULL) {
        return 0u;
    }

    memset(decode_out, 0, sizeof(*decode_out));

    ch0 = VideoLinkExtractBitsLE(frame, VIDEO_LINK_KM_CH0_BIT_OFFSET, 11u);
    ch1 = VideoLinkExtractBitsLE(frame, VIDEO_LINK_KM_CH1_BIT_OFFSET, 11u);
    ch2 = VideoLinkExtractBitsLE(frame, VIDEO_LINK_KM_CH2_BIT_OFFSET, 11u);
    ch3 = VideoLinkExtractBitsLE(frame, VIDEO_LINK_KM_CH3_BIT_OFFSET, 11u);
    wheel = VideoLinkExtractBitsLE(frame, VIDEO_LINK_KM_WHEEL_BIT_OFFSET, 11u);
    mode_sw = VideoLinkExtractBitsLE(frame, VIDEO_LINK_KM_MODE_SW_BIT_OFFSET, 2u);
    pause_button = VideoLinkExtractBitsLE(frame, VIDEO_LINK_KM_PAUSE_BIT_OFFSET, 1u);
    fn_left_button = VideoLinkExtractBitsLE(frame, VIDEO_LINK_KM_FN_LEFT_BIT_OFFSET, 1u);
    fn_right_button = VideoLinkExtractBitsLE(frame, VIDEO_LINK_KM_FN_RIGHT_BIT_OFFSET, 1u);
    trigger_button = VideoLinkExtractBitsLE(frame, VIDEO_LINK_KM_TRIGGER_BIT_OFFSET, 1u);
    mouse_left_button = VideoLinkExtractBitsLE(frame, VIDEO_LINK_KM_MOUSE_LEFT_BIT_OFFSET, 2u);
    mouse_right_button = VideoLinkExtractBitsLE(frame, VIDEO_LINK_KM_MOUSE_RIGHT_BIT_OFFSET, 2u);
    mouse_mid_button = VideoLinkExtractBitsLE(frame, VIDEO_LINK_KM_MOUSE_MID_BIT_OFFSET, 2u);

    // 这里按手册给出的物理量范围做一次字段合法性检查，作用是在 CRC 通过后继续挡掉异常组合帧；
    // 原因是真机串口偶发错位时仍可能碰巧撞上 CRC，通过范围过滤能降低把坏帧映射成运动命令的概率。
    if (ch0 < RC_CH_VALUE_MIN || ch0 > RC_CH_VALUE_MAX ||
        ch1 < RC_CH_VALUE_MIN || ch1 > RC_CH_VALUE_MAX ||
        ch2 < RC_CH_VALUE_MIN || ch2 > RC_CH_VALUE_MAX ||
        ch3 < RC_CH_VALUE_MIN || ch3 > RC_CH_VALUE_MAX ||
        wheel < RC_CH_VALUE_MIN || wheel > RC_CH_VALUE_MAX ||
        mode_sw > 2u ||
        mouse_left_button > 1u ||
        mouse_right_button > 1u ||
        mouse_mid_button > 1u) {
        return 0u;
    }

    // 这里把 VT03 的 4 路摇杆重新映射到现有 `RC_ctrl_t` 语义，作用是让下游仍按 DBUS 坐标系读摇杆；
    // 原因是 VT03 手册里的 `channel2/3` 定义顺序与仓库中 `rocker_l_/rocker_l1` 的排列不同，不能直接照抄编号。
    decode_out->rocker_r_ = (int16_t)ch0 - RC_CH_VALUE_OFFSET;
    decode_out->rocker_r1 = (int16_t)ch1 - RC_CH_VALUE_OFFSET;
    decode_out->rocker_l1 = (int16_t)ch2 - RC_CH_VALUE_OFFSET;
    decode_out->rocker_l_ = (int16_t)ch3 - RC_CH_VALUE_OFFSET;
    decode_out->dial = (int16_t)wheel - RC_CH_VALUE_OFFSET;
    decode_out->mouse_x = VideoLinkExtractSigned16LE(frame, VIDEO_LINK_KM_MOUSE_X_BIT_OFFSET);
    decode_out->mouse_y = VideoLinkExtractSigned16LE(frame, VIDEO_LINK_KM_MOUSE_Y_BIT_OFFSET);
    decode_out->mouse_z = VideoLinkExtractSigned16LE(frame, VIDEO_LINK_KM_MOUSE_Z_BIT_OFFSET);
    decode_out->keyboard_value = (uint16_t)VideoLinkExtractBitsLE(frame, VIDEO_LINK_KM_KEYBOARD_BIT_OFFSET, 16u);
    decode_out->left_button_down = (uint8_t)mouse_left_button;
    decode_out->right_button_down = (uint8_t)mouse_right_button;
    decode_out->mid_button_down = (uint8_t)mouse_mid_button;
    decode_out->mode_sw = (uint8_t)mode_sw;
    decode_out->pause_button_down = (uint8_t)pause_button;
    decode_out->fn_left_button_down = (uint8_t)fn_left_button;
    decode_out->fn_right_button_down = (uint8_t)fn_right_button;
    decode_out->trigger_button_down = (uint8_t)trigger_button;

    return 1u;
}

/**
 * @brief 根据解包结果更新 RC 风格的键鼠状态
 *
 * @param decode 解包结果
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

    // 这里把 VT03 的摇杆/键鼠等价字段映射进 `RC_ctrl_t`，作用是让 `robot_cmd` 的键鼠和摇杆路径继续复用旧接口；
    // 原因是本轮虽然新增了 VT03 遥控主状态机，但底层输入容器仍以 `RC_ctrl_t` 为统一消费格式。
    video_link_ctrl[TEMP].rc.rocker_l_ = decode->rocker_l_;
    video_link_ctrl[TEMP].rc.rocker_l1 = decode->rocker_l1;
    video_link_ctrl[TEMP].rc.rocker_r_ = decode->rocker_r_;
    video_link_ctrl[TEMP].rc.rocker_r1 = decode->rocker_r1;
    video_link_ctrl[TEMP].rc.dial = decode->dial;
    video_link_ctrl[TEMP].mouse.x = decode->mouse_x;
    video_link_ctrl[TEMP].mouse.y = decode->mouse_y;
    video_link_ctrl[TEMP].mouse.press_l = decode->left_button_down;
    video_link_ctrl[TEMP].mouse.press_r = decode->right_button_down;
    *(uint16_t *)&video_link_ctrl[TEMP].key[KEY_PRESS] = decode->keyboard_value;
    key_now = video_link_ctrl[TEMP].key[KEY_PRESS].keys;

    // 这里继续按显式位掩码生成 Ctrl/Shift 组合键态，作用是让图传与 DBUS 共用同一套稳定键盘语义；
    // 原因是位域布局依赖编译器实现，而 `robot_cmd` 的按键逻辑已经统一改成了按协议 bit 位判断。
    if ((key_now & (1u << Key_Ctrl)) != 0u) {
        video_link_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = video_link_ctrl[TEMP].key[KEY_PRESS];
    } else {
        memset(&video_link_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    }

    if ((key_now & (1u << Key_Shift)) != 0u) {
        video_link_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = video_link_ctrl[TEMP].key[KEY_PRESS];
    } else {
        memset(&video_link_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));
    }

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
    video_link_diag.last_switch_position = decode->mode_sw;
    video_link_diag.last_trigger_state = decode->trigger_button_down;
    // 这里同步保存 VT03 遥控专用状态，作用是让 `robot_cmd` 直接读取 Pause/CNS/自定义键/扳机；
    // 原因是这些量不属于 `RC_ctrl_t` 标准字段，若只塞进诊断结构会让主状态机读取语义不清晰。
    video_link_remote_state.mode_sw = decode->mode_sw;
    video_link_remote_state.pause_button_down = decode->pause_button_down;
    video_link_remote_state.fn_left_button_down = decode->fn_left_button_down;
    video_link_remote_state.fn_right_button_down = decode->fn_right_button_down;
    video_link_remote_state.trigger_button_down = decode->trigger_button_down;
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
    uint16_t index = 0u;
    uint8_t found_candidate = 0u;
    uint8_t found_valid_frame = 0u;
    uint32_t prev_header_fail_count;
    uint32_t prev_crc_fail_count;

    if (video_link_usart_instance == NULL) {
        return;
    }

    recv_len = video_link_usart_instance->recv_len;
    prev_header_fail_count = video_link_diag.header_fail_count;
    prev_crc_fail_count = video_link_diag.crc_fail_count;
    video_link_diag.last_frame_len = recv_len;
    VideoLinkClearCurrentControl();

    // 这里在 DMA 收到的缓冲区里扫描多帧，作用是兼容一次空闲中断带回多包数据的情况；
    // 原因是 VT03 固定 14ms 连续出帧，任务负载变化时同一回调里可能粘连两帧以上数据。
    while ((uint16_t)(index + 1u) < recv_len) {
        if (video_link_usart_instance->recv_buff[index] != VIDEO_LINK_KM_FRAME_HEADER_0 ||
            video_link_usart_instance->recv_buff[index + 1u] != VIDEO_LINK_KM_FRAME_HEADER_1) {
            index++;
            continue;
        }

        found_candidate = 1u;
        if ((uint16_t)(recv_len - index) < VIDEO_LINK_KM_FRAME_SIZE) {
            video_link_diag.header_fail_count++;
            break;
        }

        video_link_diag.rx_frame_count++;
        if (!VideoLinkVerifyFrame(video_link_usart_instance->recv_buff + index)) {
            video_link_diag.crc_fail_count++;
            index++;
            continue;
        }

        if (!VideoLinkDecodeFrame(video_link_usart_instance->recv_buff + index, &decode)) {
            video_link_diag.header_fail_count++;
            index += VIDEO_LINK_KM_FRAME_SIZE;
            continue;
        }

        // 这里始终保留本次缓冲区中的最后一帧有效数据，作用是让控制层拿到最新输入；
        // 原因是多帧粘连时旧帧已经过时，继续消费只会给底盘和云台增加额外延迟。
        found_valid_frame = 1u;
        video_link_diag.valid_frame_count++;
        index += VIDEO_LINK_KM_FRAME_SIZE;
    }

    if (!found_candidate && recv_len != 0u) {
        video_link_diag.header_fail_count++;
    }

    if (found_valid_frame) {
        VideoLinkApplyDecodedState(&decode);
        video_link_has_valid_frame = 1u;
        DaemonReload(video_link_daemon_instance);
        if (video_link_diag.valid_frame_count == 1u) {
            LOGINFO("[video_link] VT03 frame online, rx_len=%u", recv_len);
        }
        return;
    }

    memcpy(&video_link_ctrl[LAST], &video_link_ctrl[TEMP], sizeof(RC_ctrl_t));

    if (video_link_diag.header_fail_count != prev_header_fail_count &&
        (video_link_diag.header_fail_count % VIDEO_LINK_KM_LOG_INTERVAL) == 1u) {
        LOGWARNING("[video_link] VT03 header failed, rx_len=%u fail=%lu",
                   recv_len,
                   (unsigned long)video_link_diag.header_fail_count);
    }

    if (video_link_diag.crc_fail_count != prev_crc_fail_count &&
        (video_link_diag.crc_fail_count % VIDEO_LINK_KM_LOG_INTERVAL) == 1u) {
        LOGWARNING("[video_link] VT03 crc failed, rx_len=%u fail=%lu",
                   recv_len,
                   (unsigned long)video_link_diag.crc_fail_count);
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

    // What: 离线时先打印关键诊断快照；Why: 用户现场看到“突然失能又恢复”时，需要第一时间区分是根本没收到帧，还是收到后被头/CRC过滤掉了。
    LOGWARNING("[video_link] VT03 keyboard-mouse link lost valid=%lu header=%lu crc=%lu last_len=%u",
               (unsigned long)video_link_diag.valid_frame_count,
               (unsigned long)video_link_diag.header_fail_count,
               (unsigned long)video_link_diag.crc_fail_count,
               video_link_diag.last_frame_len);

    // 这里掉线时直接清空图传键鼠状态，作用是让 `robot_cmd` 下一拍立刻看到全零输入；
    // 原因是图传在线时它优先于 DBUS 键鼠，若断链不清状态就会把旧按键和鼠标增量继续带下去。
    memset(video_link_ctrl, 0, sizeof(video_link_ctrl));
    video_link_has_valid_frame = 0u;
    video_link_diag.last_mouse_z = 0;
    video_link_diag.last_mid_button = 0u;
    video_link_diag.last_switch_position = 0u;
    video_link_diag.last_trigger_state = 0u;
    // 这里离线时同步清空遥控状态，作用是让上层不会把旧的 VT03 拨挡和扳机电平当成当前命令；
    // 原因是图传断链后 `robot_cmd` 需要立即回退到 DT7 或 NONE，不能残留上一次有效帧的遥控语义。
    memset(&video_link_remote_state, 0, sizeof(video_link_remote_state));
    USARTServiceInit(video_link_usart_instance);
    LOGWARNING("[video_link] VT03 keyboard-mouse link lost");
}

RC_ctrl_t *VideoLinkKMInit(UART_HandleTypeDef *video_link_usart_handle)
{
    USART_Init_Config_s usart_conf;
    Daemon_Init_Config_s daemon_conf;

    memset(&video_link_diag, 0, sizeof(video_link_diag));
    memset(video_link_ctrl, 0, sizeof(video_link_ctrl));
    memset(&video_link_remote_state, 0, sizeof(video_link_remote_state));
    video_link_has_valid_frame = 0u;

    usart_conf.recv_buff_size = VIDEO_LINK_KM_RX_BUFFER_SIZE;
    usart_conf.usart_handle = video_link_usart_handle;
    usart_conf.module_callback = VideoLinkKMRxCallback;
    video_link_usart_instance = USARTRegister(&usart_conf);

    // What: 放宽 VT03 在线宽限到约 400ms；Why: 图传偶发错帧、DMA 重启或短暂无线抖动不应立刻把整车打到失能或主控回退。
    daemon_conf.reload_count = VIDEO_LINK_KM_DAEMON_RELOAD_COUNT;
    daemon_conf.init_count = VIDEO_LINK_KM_DAEMON_INIT_COUNT;
    daemon_conf.callback = VideoLinkKMOfflineCallback;
    daemon_conf.owner_id = NULL;
    video_link_daemon_instance = DaemonRegister(&daemon_conf);

    video_link_init_flag = 1u;
    LOGINFO("[video_link] VT03 init ok, uart=%p frame_size=%u",
            (void *)video_link_usart_handle,
            VIDEO_LINK_KM_FRAME_SIZE);
    return video_link_ctrl;
}

uint8_t VideoLinkKMIsOnline(void)
{
    if (!video_link_init_flag) {
        return 0u;
    }
    return DaemonIsOnline(video_link_daemon_instance);
}

uint8_t VideoLinkKMHasValidFrame(void)
{
    return video_link_has_valid_frame;
}

const VideoLinkKM_Diag_s *VideoLinkKMGetDiag(void)
{
    return &video_link_diag;
}

const VideoLinkKM_RemoteState_s *VideoLinkKMGetRemoteState(void)
{
    return &video_link_remote_state;
}
