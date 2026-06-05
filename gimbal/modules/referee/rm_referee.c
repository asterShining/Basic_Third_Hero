/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小
#define REF_PARSER_WARN_INTERVAL_MS 500u // 解析异常日志限频，避免中断内高频打印造成二次干扰

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon; // 裁判系统守护进程
static referee_info_t referee_info; // 裁判系统数据
static RefereeRxDiag_s referee_rx_diag; // 裁判链路诊断统计
static uint32_t referee_parser_warn_tick_ms; // 解析告警限频时间戳
static uint8_t referee_ui_tx_log_count; // 记录已打印的 UI 发包日志数量，便于现场确认旧 UI 头字段是否仍然正确

// 这里把“命令码已识别但长度不匹配”的情况单独记下来，作用是把协议版本漂移和纯链路损坏分开看；
// 原因是升级到 2026 新协议后，最容易遇到的问题不是完全收不到，而是命令能对上但长度还停留在旧版本。
static uint8_t RefereeRecordLengthMismatch(uint16_t cmd_id, uint16_t data_len)
{
    referee_rx_diag.cmd_len_mismatch_count++;
    referee_rx_diag.last_len_mismatch_cmd_id = cmd_id;
    referee_rx_diag.last_data_len = data_len;
    return 0u;
}

// 这里按“命令码 + DataLength”双重条件落盘，作用是避免同一命令在跨版本长度变化时误把错误字节拷进状态结构；
// 原因是本轮升级的核心就是把云台板从旧协议同步到新协议，长度校验必须成为解析层第一道硬门槛。
static uint8_t RefereeDecodeFrameByCmdID(uint16_t cmd_id, const uint8_t *data_ptr, uint16_t data_len)
{
    uint16_t user_data_len = 0u;

    switch (cmd_id) {
    case ID_game_state: // 0x0001
        if (data_len == LEN_game_state) {
            memcpy(&referee_info.GameState, data_ptr, LEN_game_state);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_game_result: // 0x0002
        if (data_len == LEN_game_result) {
            memcpy(&referee_info.GameResult, data_ptr, LEN_game_result);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_game_robot_survivors: // 0x0003
        if (data_len == LEN_game_robot_HP) {
            memcpy(&referee_info.GameRobotHP, data_ptr, LEN_game_robot_HP);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_event_data: // 0x0101
        if (data_len == LEN_event_data) {
            memcpy(&referee_info.EventData, data_ptr, LEN_event_data);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_referee_warning: // 0x0104
        if (data_len == LEN_referee_warning) {
            memcpy(&referee_info.RefereeWarning, data_ptr, LEN_referee_warning);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_dart_info: // 0x0105
        if (data_len == LEN_dart_info) {
            memcpy(&referee_info.DartInfo, data_ptr, LEN_dart_info);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_game_robot_state: // 0x0201
        if (data_len == LEN_game_robot_state) {
            memcpy(&referee_info.GameRobotState, data_ptr, LEN_game_robot_state);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_power_heat_data: // 0x0202
        if (data_len == LEN_power_heat_data) {
            memcpy(&referee_info.PowerHeatData, data_ptr, LEN_power_heat_data);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_game_robot_pos: // 0x0203
        if (data_len == LEN_game_robot_pos) {
            memcpy(&referee_info.GameRobotPos, data_ptr, LEN_game_robot_pos);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_buff_musk: // 0x0204
        if (data_len == LEN_buff_musk) {
            memcpy(&referee_info.BuffMusk, data_ptr, LEN_buff_musk);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_robot_hurt: // 0x0206
        if (data_len == LEN_robot_hurt) {
            memcpy(&referee_info.RobotHurt, data_ptr, LEN_robot_hurt);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_shoot_data: // 0x0207
        if (data_len == LEN_shoot_data) {
            memcpy(&referee_info.ShootData, data_ptr, LEN_shoot_data);
            // 这里在成功解析射击数据后记录到达时刻，作用是给旧 UI 或调试逻辑提供“弹速是否新鲜”的依据；
            // 原因是只看数值本身是否非零，无法区分“刚开火”和“很久以前残留的一次开火”。
            referee_info.last_shoot_data_tick_ms = HAL_GetTick();
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_projectile_allowance: // 0x0208
        if (data_len == LEN_projectile_allowance) {
            memcpy(&referee_info.ProjectileAllowance, data_ptr, LEN_projectile_allowance);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_rfid_status: // 0x0209
        if (data_len == LEN_rfid_status) {
            memcpy(&referee_info.RFIDStatus, data_ptr, LEN_rfid_status);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_dart_client_cmd: // 0x020A
        if (data_len == LEN_dart_client_cmd) {
            memcpy(&referee_info.DartClientCmd, data_ptr, LEN_dart_client_cmd);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_ground_robot_position: // 0x020B
        if (data_len == LEN_ground_robot_position) {
            memcpy(&referee_info.GroundRobotPosition, data_ptr, LEN_ground_robot_position);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_radar_mark_data: // 0x020C
        if (data_len == LEN_radar_mark_data) {
            memcpy(&referee_info.RadarMarkData, data_ptr, LEN_radar_mark_data);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_sentry_info: // 0x020D
        if (data_len == LEN_sentry_info) {
            memcpy(&referee_info.SentryInfo, data_ptr, LEN_sentry_info);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_radar_info: // 0x020E
        if (data_len == LEN_radar_info) {
            memcpy(&referee_info.RadarInfo, data_ptr, LEN_radar_info);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_student_interactive: // 0x0301
        // 这里按协议处理 0x0301 的可变用户段，作用是同时支持官方最大 112 字节负载和本工程历史上的 5 字节裁剪缓存；
        // 原因是用户要求协议层同步到最新，但旧 UI/多机通信的读取路径仍不能被破坏。
        if ((data_len >= Interactive_Data_LEN_Head) && (data_len <= LEN_robot_interaction_data)) {
            memset(&referee_info.RobotInteractionData, 0, sizeof(referee_info.RobotInteractionData));
            memcpy(&referee_info.RobotInteractionData, data_ptr, Interactive_Data_LEN_Head);
            user_data_len = (uint16_t)(data_len - Interactive_Data_LEN_Head);
            memcpy(referee_info.RobotInteractionData.user_data, data_ptr + Interactive_Data_LEN_Head, user_data_len);

            if (data_len >= (uint16_t)(Interactive_Data_LEN_Head + Communicate_Data_LEN)) {
                memcpy(&referee_info.ReceiveData.datahead, data_ptr, Interactive_Data_LEN_Head);
                memcpy(referee_info.ReceiveData.Data.data, data_ptr + Interactive_Data_LEN_Head, Communicate_Data_LEN);
            }
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_custom_robot_data: // 0x0302
        if (data_len == LEN_custom_robot_data) {
            memcpy(&referee_info.CustomRobotData, data_ptr, LEN_custom_robot_data);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_map_command: // 0x0303
        if (data_len == LEN_map_command) {
            memcpy(&referee_info.MapCommand, data_ptr, LEN_map_command);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_map_robot_data: // 0x0305
        if (data_len == LEN_map_robot_data) {
            memcpy(&referee_info.MapRobotData, data_ptr, LEN_map_robot_data);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_custom_client_data: // 0x0306
        if (data_len == LEN_custom_client_data) {
            memcpy(&referee_info.CustomClientData, data_ptr, LEN_custom_client_data);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_map_data: // 0x0307
        if (data_len == LEN_map_data) {
            memcpy(&referee_info.MapData, data_ptr, LEN_map_data);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_custom_info: // 0x0308
        if (data_len == LEN_custom_info) {
            memcpy(&referee_info.CustomInfo, data_ptr, LEN_custom_info);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_robot_custom_data: // 0x0309
        if (data_len == LEN_robot_custom_data) {
            memcpy(&referee_info.RobotCustomData, data_ptr, LEN_robot_custom_data);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_robot_custom_data_2: // 0x0310
        if (data_len == LEN_robot_custom_data_2) {
            memcpy(&referee_info.RobotCustomData2, data_ptr, LEN_robot_custom_data_2);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    case ID_robot_custom_data_3: // 0x0311
        if (data_len == LEN_robot_custom_data_3) {
            memcpy(&referee_info.RobotCustomData3, data_ptr, LEN_robot_custom_data_3);
            return 1u;
        }
        return RefereeRecordLengthMismatch(cmd_id, data_len);
    default:
        // 这里对未知命令单独计数，作用是让排障时能区分“串口坏了”和“串口通了但协议版本没对齐”；
        // 原因是升级协议后，未识别命令往往比 CRC 错误更能直接指向版本漂移问题。
        referee_rx_diag.cmd_unknown_count++;
        return 0u;
    }
}

/**
 * @brief 读取并解析裁判原始数据
 * @param buff DMA接收缓冲区首地址
 * @param recv_len 本次DMA+IDLE接收长度
 * @retval 本次解析成功帧数
 * @attention 使用边界检查 + CRC分层校验，避免脏数据导致越界递归
 */
static uint16_t JudgeReadData(uint8_t *buff, uint16_t recv_len)
{
    uint16_t valid_frame_count = 0u;
    uint16_t offset = 0u;
    uint16_t frame_len = 0u;
    uint16_t data_len = 0u;
    uint16_t cmd_id = 0u;

    if (buff == NULL || recv_len == 0u) {
        return 0u;
    }

    while ((uint16_t)(offset + LEN_HEADER) <= recv_len) {
        if (buff[offset + SOF] != REFEREE_SOF) {
            // 这里按字节推进重同步，作用是在链路错位后尽快重新咬住下一帧，而不是递归走飞；
            // 原因是旧版递归解析在坏包场景里很容易越界，最新版必须先把边界稳住。
            referee_rx_diag.sof_miss_count++;
            offset++;
            continue;
        }

        if (Verify_CRC8_Check_Sum(buff + offset, LEN_HEADER) != TRUE) {
            referee_rx_diag.crc8_fail_count++;
            offset++;
            continue;
        }

        data_len = (uint16_t)buff[offset + DATA_LENGTH] | ((uint16_t)buff[offset + DATA_LENGTH + 1u] << 8);
        frame_len = (uint16_t)(LEN_HEADER + LEN_CMDID + LEN_TAIL + data_len);

        if (frame_len < (uint16_t)(LEN_HEADER + LEN_CMDID + LEN_TAIL) || frame_len > RE_RX_BUFFER_SIZE) {
            referee_rx_diag.frame_len_error_count++;
            offset++;
            continue;
        }

        if ((uint16_t)(offset + frame_len) > recv_len) {
            referee_rx_diag.short_frame_count++;
            break;
        }

        if (Verify_CRC16_Check_Sum(buff + offset, frame_len) != TRUE) {
            referee_rx_diag.crc16_fail_count++;
            offset++;
            continue;
        }

        memcpy(&referee_info.FrameHeader, buff + offset, LEN_HEADER);
        cmd_id = (uint16_t)((buff[offset + CMD_ID_Offset + 1u] << 8) | buff[offset + CMD_ID_Offset]);
        referee_info.CmdID = cmd_id;
        referee_rx_diag.last_cmd_id = cmd_id;
        referee_rx_diag.last_data_len = data_len;
        referee_rx_diag.last_frame_len = frame_len;

        // 这里把“CRC 通过的完整帧”视为有效链路证据，即使命令长度不匹配也不立即判离线；
        // 原因是协议版本漂移不等于物理链路断开，喂狗逻辑必须建立在“是否真的有完整帧”而不是“业务是否完全匹配”上。
        RefereeDecodeFrameByCmdID(cmd_id, buff + offset + DATA_Offset, data_len);
        valid_frame_count++;
        offset = (uint16_t)(offset + frame_len);
    }

    return valid_frame_count;
}

/* 裁判系统串口接收回调函数 */
static void RefereeRxCallback()
{
    uint16_t valid_frame = 0u;
    uint16_t rx_size = 0u;
    uint32_t now_tick = HAL_GetTick();
    const USARTDiagInfo *uart_diag = USARTGetDiagInfo(referee_usart_instance);

    if (uart_diag != NULL) {
        rx_size = uart_diag->last_rx_size;
    }

    referee_rx_diag.rx_callback_count++;
    referee_rx_diag.rx_byte_count += rx_size;
    referee_rx_diag.last_rx_size = rx_size;

    valid_frame = JudgeReadData(referee_usart_instance->recv_buff, rx_size);
    referee_rx_diag.valid_frame_count += valid_frame;

    if (valid_frame > 0u) {
        // 这里仅在成功解析出完整帧后喂狗，作用是防止坏包风暴把真正的离线状态掩盖掉；
        // 原因是云台板升级协议后，必须把“有中断”与“真有可用数据”这两件事分开看。
        DaemonReload(referee_daemon);
        referee_rx_diag.daemon_reload_count++;
        referee_rx_diag.last_valid_tick_ms = now_tick;
        return;
    }

    if (rx_size > 0u && (now_tick - referee_parser_warn_tick_ms >= REF_PARSER_WARN_INTERVAL_MS)) {
        referee_parser_warn_tick_ms = now_tick;
        LOGWARNING("[rm_ref][stage:parser] rx_len:%u valid:0 sof_miss:%lu crc8:%lu crc16:%lu short:%lu len_err:%lu len_mismatch:%lu last_mismatch_cmd:0x%04X last_data_len:%u uart_err:%lu",
                   (unsigned int)rx_size,
                   (unsigned long)referee_rx_diag.sof_miss_count,
                   (unsigned long)referee_rx_diag.crc8_fail_count,
                   (unsigned long)referee_rx_diag.crc16_fail_count,
                   (unsigned long)referee_rx_diag.short_frame_count,
                   (unsigned long)referee_rx_diag.frame_len_error_count,
                   (unsigned long)referee_rx_diag.cmd_len_mismatch_count,
                   (unsigned int)referee_rx_diag.last_len_mismatch_cmd_id,
                   (unsigned int)referee_rx_diag.last_data_len,
                   (unsigned long)((uart_diag == NULL) ? 0u : uart_diag->error_callback_count));
    }
}

// 裁判系统丢失回调函数，重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
    uint32_t now_tick = HAL_GetTick();
    uint32_t stale_ms = 0u;
    const USARTDiagInfo *uart_diag = USARTGetDiagInfo(referee_usart_instance);
    (void)arg;

    referee_rx_diag.lost_count++;
    referee_rx_diag.last_lost_tick_ms = now_tick;
    if (referee_rx_diag.last_valid_tick_ms != 0u) {
        stale_ms = now_tick - referee_rx_diag.last_valid_tick_ms;
    }

    USARTServiceInit(referee_usart_instance);
    // 这里在离线回调后立即重载 daemon，作用是避免进入“每 10ms 触发一次离线日志和重启”的风暴状态；
    // 原因是现场排障更需要一次有信息量的日志，而不是刷屏。
    DaemonReload(referee_daemon);

    LOGWARNING("[rm_ref][stage:lost] lost:%lu stale_ms:%lu valid:%lu rx_cb:%lu crc8:%lu crc16:%lu len_mismatch:%lu last_mismatch_cmd:0x%04X last_data_len:%u uart_ne:%lu uart_fe:%lu uart_ore:%lu last_ec:0x%08lX",
               (unsigned long)referee_rx_diag.lost_count,
               (unsigned long)stale_ms,
               (unsigned long)referee_rx_diag.valid_frame_count,
               (unsigned long)referee_rx_diag.rx_callback_count,
               (unsigned long)referee_rx_diag.crc8_fail_count,
               (unsigned long)referee_rx_diag.crc16_fail_count,
               (unsigned long)referee_rx_diag.cmd_len_mismatch_count,
               (unsigned int)referee_rx_diag.last_len_mismatch_cmd_id,
               (unsigned int)referee_rx_diag.last_data_len,
               (unsigned long)((uart_diag == NULL) ? 0u : uart_diag->error_ne_count),
               (unsigned long)((uart_diag == NULL) ? 0u : uart_diag->error_fe_count),
               (unsigned long)((uart_diag == NULL) ? 0u : uart_diag->error_ore_count),
               (unsigned long)((uart_diag == NULL) ? 0u : uart_diag->last_error_code));
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
    USART_Init_Config_s conf;
    Daemon_Init_Config_s daemon_conf;

    memset(&referee_info, 0, sizeof(referee_info));
    memset(&referee_rx_diag, 0, sizeof(referee_rx_diag));
    referee_parser_warn_tick_ms = 0u;

    conf.module_callback = RefereeRxCallback;
    conf.usart_handle = referee_usart_handle;
    conf.recv_buff_size = RE_RX_BUFFER_SIZE; // max 255(u8)
    referee_usart_instance = USARTRegister(&conf);

    daemon_conf.callback = RefereeLostCallback;
    daemon_conf.owner_id = referee_usart_instance;
    daemon_conf.reload_count = 200; // DaemonTask按100Hz运行，200计数约2s，过滤短时抖动
    daemon_conf.init_count = 200; // 上电后同样给2s启动窗口，避免初始化阶段误报离线
    referee_daemon = DaemonRegister(&daemon_conf);

    LOGINFO("[rm_ref][stage:init] referee usart ready, timeout_ms:%u",
            (unsigned int)(daemon_conf.reload_count * 10u));

    return &referee_info;
}

/**
 * @brief 裁判系统数据发送函数
 * @param send 发送首地址
 * @param tx_len 发送长度
 */
void RefereeSendRaw(uint8_t *send, uint16_t tx_len)
{
    uint16_t cmd_id = 0u;
    uint16_t sub_id = 0u;
    uint16_t sender_id = 0u;
    uint16_t receiver_id = 0u;

    if (send == NULL || tx_len == 0u) {
        return;
    }

    if (tx_len >= 11u) {
        cmd_id = (uint16_t)((uint16_t)send[5] | ((uint16_t)send[6] << 8));
        sub_id = (uint16_t)((uint16_t)send[7] | ((uint16_t)send[8] << 8));
        sender_id = (uint16_t)((uint16_t)send[9] | ((uint16_t)send[10] << 8));
        if (tx_len >= 13u) {
            receiver_id = (uint16_t)((uint16_t)send[11] | ((uint16_t)send[12] << 8));
        }
    }

    if (cmd_id == ID_student_interactive && referee_ui_tx_log_count < 24u) {
        LOGINFO("[ui_tx] len:%u seq:%u sub:0x%04X sender:%u recv:0x%04X",
                (unsigned int)tx_len,
                (unsigned int)send[3],
                (unsigned int)sub_id,
                (unsigned int)sender_id,
                (unsigned int)receiver_id);
        referee_ui_tx_log_count++;
    }

    // 这里优先沿用 DMA 发包，作用是在旧 UI 调用链下尽量降低 CPU 占用并保持和原 BSP 一致的发送方式；
    // 原因是协议层升级不应该顺手把发送模式改掉，能用现有 DMA 就不新增新的调度复杂度。
    if (USARTIsReady(referee_usart_instance) != 0u) {
        USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
        return;
    }

    // 这里在 DMA 正忙时退回阻塞发送，作用是避免初始化建图阶段直接丢掉关键 UI 帧；
    // 原因是旧工程没有发送队列，若一律放弃，用户会遇到“偶发少图层”的隐蔽问题。
    USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_BLOCKING);
}

void RefereeSend(uint8_t *send, uint16_t tx_len)
{
    RefereeSendRaw(send, tx_len);
    // 这里保留旧接口自带的 115ms 限速语义，作用是保证还在走旧 UI 路径的代码不需要同步重构；
    // 原因是本轮要求仅同步协议层，不整体迁移到底盘新版 UI task。
    osDelay(115);
}

const RefereeRxDiag_s *RefereeGetRxDiag(void)
{
    return &referee_rx_diag;
}
