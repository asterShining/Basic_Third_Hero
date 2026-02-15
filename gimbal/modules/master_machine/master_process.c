/**
 * @file    master_process.c
 * @brief   视觉上位机 (sp_vision_25) 通信模块 — 实现
 * @details 适配同济大学 sp_vision 上位机通信协议。
 *          根据 robot_def.h 中的宏定义, 选择以下通信方式之一:
 *          - VISION_USE_VCP:  USB 虚拟串口 (默认)
 *          - VISION_USE_UART: 硬件串口
 *          - VISION_USE_CAN:  CAN 总线
 *
 *          CAN 协议: 使用 int16 缩放编码, 与 io/cboard.cpp 对应
 *          串口协议: 使用 SP 帧头 + CRC16-CCITT, 与 io/gimbal 对应
 * @version 2.0
 * @date    2026-02-15
 */

#include "master_process.h"
#include "sp_vision_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "memory.h"

/* ======================== 模块私有变量 ======================== */
static Vision_Recv_s recv_data; // 接收数据 (解析后)
static Vision_Send_s send_data; // 发送数据 (由应用层填充)
static DaemonInstance *vision_daemon_instance; // 通信看门狗
static uint32_t vision_log_cnt = 0; // 日志降频计数器 (避免高频刷屏)
#define VISION_LOG_INTERVAL 200 // 每200次打印一次日志 (200Hz调用时≈1Hz输出)

/* ======================== 公共接口实现 ======================== */

/**
 * @brief 设置发送数据的 IMU 四元数
 * @note  四元数顺序为 [w, x, y, z], 与上位机 GimbalToVision.q 一致
 */
void VisionSetQuaternion(const float *q)
{
    memcpy(send_data.q, q, sizeof(float) * 4);
}

/**
 * @brief 设置发送数据的云台姿态
 */
void VisionSetAltitude(float yaw, float pitch, float yaw_vel, float pitch_vel)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.yaw_vel = yaw_vel;
    send_data.pitch_vel = pitch_vel;
}

/**
 * @brief 设置发送数据的机器人状态标志位
 */
void VisionSetStatus(uint8_t mode, float bullet_speed, uint16_t bullet_count)
{
    send_data.mode = mode;
    send_data.bullet_speed = bullet_speed;
    send_data.bullet_count = bullet_count;
}

/**
 * @brief 通信离线回调函数, 由 daemon 模块在检测到超时时调用
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    extern USARTInstance *vision_usart_instance;
    USARTServiceInit(vision_usart_instance);
#endif
    LOGWARNING("[vision] offline, restarting communication.");
}

/**
 * @brief 检查视觉通信是否在线
 */
uint8_t VisionIsOnline(void)
{
    return DaemonIsOnline(vision_daemon_instance);
}

/* ======================== CAN 模式实现 ======================== */
#ifdef VISION_USE_CAN

#include "bsp_can.h"

static CANInstance *vision_can_recv_instance; // CAN 接收实例 (接收视觉命令)
static CANInstance *vision_can_send_quat; // CAN 发送实例 (发送四元数)
static CANInstance *vision_can_send_status; // CAN 发送实例 (发送弹速/模式)

/**
 * @brief CAN 接收回调: 解析上位机发来的控制命令
 * @note  数据格式与 io/cboard.cpp 中的 send() 函数一致:
 *        data[0] = control, data[1] = shoot
 *        data[2..3] = yaw (int16×1e4), data[4..5] = pitch (int16×1e4)
 *        data[6..7] = horizon_distance (int16×1e4)
 */
static void VisionCANCallback(CANInstance *instance)
{
    DaemonReload(vision_daemon_instance); // 喂狗

    uint8_t *data = instance->rx_buff;
    recv_data.control = data[0];
    recv_data.shoot = data[1];
    recv_data.yaw = SP_CAN_DecodeInt16(&data[2], SP_CAN_ANGLE_SCALE);
    recv_data.pitch = SP_CAN_DecodeInt16(&data[4], SP_CAN_ANGLE_SCALE);
    recv_data.horizon_distance = SP_CAN_DecodeInt16(&data[6], SP_CAN_ANGLE_SCALE);

    // 日志: CAN 接收成功, 输出控制指令
    char y_s[16], p_s[16], d_s[16];
    Float2Str(y_s, recv_data.yaw);
    Float2Str(p_s, recv_data.pitch);
    Float2Str(d_s, recv_data.horizon_distance);
    LOGINFO("[vision] CAN_RX ctrl=%d shoot=%d yaw=%s pitch=%s dist=%s",
            recv_data.control, recv_data.shoot, y_s, p_s, d_s);
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    (void)_handle; // CAN模式不使用串口

    // 注册 CAN 接收实例 (接收上位机命令, CAN ID = 0xFF)
    CAN_Init_Config_s recv_conf = {
        .can_handle = &hcan1,
        .tx_id = SP_CAN_ID_QUATERNION,
        .rx_id = SP_CAN_ID_VISION_CMD,
        .can_module_callback = VisionCANCallback,
    };
    vision_can_recv_instance = CANRegister(&recv_conf);

    // 注册 CAN 发送实例 (发送四元数, CAN ID = 0x100)
    CAN_Init_Config_s quat_conf = {
        .can_handle = &hcan1,
        .tx_id = SP_CAN_ID_QUATERNION,
        .rx_id = 0, // 只发送, 不接收
    };
    vision_can_send_quat = CANRegister(&quat_conf);

    // 注册 CAN 发送实例 (发送弹速/模式, CAN ID = 0x101)
    CAN_Init_Config_s status_conf = {
        .can_handle = &hcan1,
        .tx_id = SP_CAN_ID_BULLET_SPEED,
        .rx_id = 0, // 只发送, 不接收
    };
    vision_can_send_status = CANRegister(&status_conf);

    // 注册 daemon (通信看门狗)
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = vision_can_recv_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    // 日志: CAN 模式初始化完成
    LOGINFO("[vision] Init OK (CAN mode) tx_quat=0x%X tx_status=0x%X rx_cmd=0x%X",
            SP_CAN_ID_QUATERNION, SP_CAN_ID_BULLET_SPEED, SP_CAN_ID_VISION_CMD);

    return &recv_data;
}

/**
 * @brief CAN模式发送: 发送四元数和机器人状态给上位机
 * @note  四元数使用 int16×1e4 编码, 与 io/cboard.cpp callback() 中的解码一致
 *        弹速使用 int16×1e2 编码
 */
void VisionSend(void)
{
    // 发送四元数 (CAN ID = 0x100)
    // 编码顺序: x, y, z, w (与上位机 callback 中的解码顺序一致)
    SP_CAN_EncodeInt16(&vision_can_send_quat->tx_buff[0], send_data.q[1], SP_CAN_QUAT_SCALE); // x
    SP_CAN_EncodeInt16(&vision_can_send_quat->tx_buff[2], send_data.q[2], SP_CAN_QUAT_SCALE); // y
    SP_CAN_EncodeInt16(&vision_can_send_quat->tx_buff[4], send_data.q[3], SP_CAN_QUAT_SCALE); // z
    SP_CAN_EncodeInt16(&vision_can_send_quat->tx_buff[6], send_data.q[0], SP_CAN_QUAT_SCALE); // w
    CANTransmit(vision_can_send_quat, 1);

    // 发送弹速/模式 (CAN ID = 0x101)
    SP_CAN_EncodeInt16(&vision_can_send_status->tx_buff[0], send_data.bullet_speed, SP_CAN_BULLET_SCALE);
    vision_can_send_status->tx_buff[2] = send_data.mode;
    vision_can_send_status->tx_buff[3] = send_data.shoot_mode;
    // 预留 data[4..7] (可扩展, 例如 ft_angle)
    vision_can_send_status->tx_buff[4] = 0;
    vision_can_send_status->tx_buff[5] = 0;
    vision_can_send_status->tx_buff[6] = 0;
    vision_can_send_status->tx_buff[7] = 0;
    CANTransmit(vision_can_send_status, 1);

    // 日志: CAN 发送完成
    LOGINFO("[vision] CAN_TX sent quat+status, mode=%d", send_data.mode);
}

#endif // VISION_USE_CAN

/* ======================== 串口 (UART) 模式实现 ======================== */
#ifdef VISION_USE_UART

static USARTInstance *vision_usart_instance;

/**
 * @brief 串口接收回调: 解析上位机发来的 VisionToGimbal 数据帧
 * @note  帧格式: ['S', 'P', mode, yaw(f), yaw_vel(f), yaw_acc(f),
 *                 pitch(f), pitch_vel(f), pitch_acc(f), crc16(u16)]
 */
static void DecodeVision(void)
{
    SP_VisionToGimbal_t rx_frame;

    // 使用协议层解包 (验证帧头 + CRC16)
    if (SP_Serial_Unpack(vision_usart_instance->recv_buff, &rx_frame)) {
        DaemonReload(vision_daemon_instance); // 喂狗

        // 解析模式: 0=不控制, 1=控制不开火, 2=控制且开火
        recv_data.control = (rx_frame.mode >= 1) ? 1 : 0;
        recv_data.shoot = (rx_frame.mode == 2) ? 1 : 0;

        // 解析控制数据
        recv_data.yaw = rx_frame.yaw;
        recv_data.yaw_vel = rx_frame.yaw_vel;
        recv_data.yaw_acc = rx_frame.yaw_acc;
        recv_data.pitch = rx_frame.pitch;
        recv_data.pitch_vel = rx_frame.pitch_vel;
        recv_data.pitch_acc = rx_frame.pitch_acc;

        // 日志: UART 接收解析成功
        char y_s[16], p_s[16];
        Float2Str(y_s, recv_data.yaw);
        Float2Str(p_s, recv_data.pitch);
        LOGINFO("[vision] UART_RX OK ctrl=%d shoot=%d yaw=%s pitch=%s",
                recv_data.control, recv_data.shoot, y_s, p_s);
    } else {
        // 日志: UART 接收解析失败 (帧头或CRC错误, 具体原因由SP_Serial_Unpack输出)
        LOGWARNING("[vision] UART_RX decode failed");
    }
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 注册 daemon (通信看门狗)
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    // 日志: UART 模式初始化完成
    LOGINFO("[vision] Init OK (UART mode) recv_size=%d", VISION_RECV_SIZE);

    return &recv_data;
}

/**
 * @brief 串口模式发送: 打包 GimbalToVision 帧并通过 UART DMA 发送
 */
void VisionSend(void)
{
    static SP_GimbalToVision_t tx_frame;

    // 填充数据
    tx_frame.mode = send_data.mode;
    memcpy(tx_frame.q, send_data.q, sizeof(float) * 4);
    tx_frame.yaw = send_data.yaw;
    tx_frame.yaw_vel = send_data.yaw_vel;
    tx_frame.pitch = send_data.pitch;
    tx_frame.pitch_vel = send_data.pitch_vel;
    tx_frame.bullet_speed = send_data.bullet_speed;
    tx_frame.bullet_count = send_data.bullet_count;

    // 打包 (自动填充帧头和 CRC16)
    SP_Serial_Pack(&tx_frame);

    // 发送
    USARTSend(vision_usart_instance, (uint8_t *)&tx_frame, sizeof(tx_frame), USART_TRANSFER_DMA);

    // 日志: UART 发送完成
    char y_s[16], p_s[16];
    Float2Str(y_s, send_data.yaw);
    Float2Str(p_s, send_data.pitch);
    LOGINFO("[vision] UART_TX mode=%d yaw=%s pitch=%s crc=0x%04X",
            tx_frame.mode, y_s, p_s, tx_frame.crc16);
}

#endif // VISION_USE_UART

/* ======================== USB 虚拟串口 (VCP) 模式实现 ======================== */
#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff; // USB 接收缓冲区指针

/**
 * @brief VCP 接收回调: 解析上位机发来的 VisionToGimbal 数据帧
 * @param recv_len 接收到的数据长度
 */
static void DecodeVision(uint16_t recv_len)
{
    // 长度校验 (必须等于 VisionToGimbal 结构体大小)
    // if (recv_len != sizeof(SP_VisionToGimbal_t)) {
    //     // 日志: 长度不匹配
    //     LOGWARNING("[vision] VCP_RX len mismatch: got %d, expect %d",
    //                recv_len, sizeof(SP_VisionToGimbal_t));
    //     return;
    // }

    SP_VisionToGimbal_t rx_frame;

    // 使用协议层解包 (验证帧头 + CRC16)
    if (SP_Serial_Unpack(vis_recv_buff, &rx_frame)) {
        DaemonReload(vision_daemon_instance); // 喂狗

        // 解析模式: 0=不控制, 1=控制不开火, 2=控制且开火
        recv_data.control = (rx_frame.mode >= 1) ? 1 : 0;
        recv_data.shoot = (rx_frame.mode == 2) ? 1 : 0;

        // 解析控制数据
        recv_data.yaw = rx_frame.yaw;
        recv_data.yaw_vel = rx_frame.yaw_vel;
        recv_data.yaw_acc = rx_frame.yaw_acc;
        recv_data.pitch = rx_frame.pitch;
        recv_data.pitch_vel = rx_frame.pitch_vel;
        recv_data.pitch_acc = rx_frame.pitch_acc;

        // 日志: 当识别到目标(control=1)时打印关键跟踪数据
        if (recv_data.control > 0) {
            char y_s[16], p_s[16];
            // 打印弧度值 (上位机发送的是绝对弧度)
            Float2Str(y_s, recv_data.yaw);
            Float2Str(p_s, recv_data.pitch);

            if (recv_data.shoot) {
                LOGINFO("[vision] FIRE! yaw=%s pitch=%s", y_s, p_s);
            } else {
                LOGINFO("[vision] TRACK yaw=%s pitch=%s", y_s, p_s);
            }
        }
    } else {
        // 日志: VCP 解析失败 (具体原因由SP_Serial_Unpack输出)
        LOGWARNING("[vision] VCP_RX decode failed (len=%d)", recv_len);
    }
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    (void)_handle; // VCP模式不使用串口句柄
    USB_Init_Config_s conf = { .rx_cbk = DecodeVision };
    vis_recv_buff = USBInit(conf);

    // 注册 daemon (通信看门狗)
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = NULL,
        .reload_count = 5, // 50ms (VCP通信频率较高)
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    // // 日志: VCP 模式初始化完成
    // LOGINFO("[vision] Init OK (VCP mode) recv_size=%d, expect_struct_size=%d",
    //         VISION_RECV_SIZE, sizeof(SP_VisionToGimbal_t));

    return &recv_data;
}

/**
 * @brief VCP模式发送: 打包 GimbalToVision 帧并通过 USB 发送
 */
void VisionSend(void)
{
    static SP_GimbalToVision_t tx_frame;

    // 填充数据
    tx_frame.mode = send_data.mode;
    memcpy(tx_frame.q, send_data.q, sizeof(float) * 4);
    tx_frame.yaw = send_data.yaw;
    tx_frame.yaw_vel = send_data.yaw_vel;
    tx_frame.pitch = send_data.pitch;
    tx_frame.pitch_vel = send_data.pitch_vel;
    tx_frame.bullet_speed = send_data.bullet_speed;
    tx_frame.bullet_count = send_data.bullet_count;

    // 打包 (自动填充帧头和 CRC16)
    SP_Serial_Pack(&tx_frame);

    // 通过 USB 虚拟串口发送
    USBTransmit((uint8_t *)&tx_frame, sizeof(tx_frame));

    // // 日志: 降频输出 (每 VISION_LOG_INTERVAL 次打印一次, ≈1Hz)
    // if ((vision_log_cnt++ % VISION_LOG_INTERVAL) == 0) {
    //     char y_s[16], p_s[16], w_s[16];
    //     Float2Str(y_s, send_data.yaw);
    //     Float2Str(p_s, send_data.pitch);
    //     Float2Str(w_s, send_data.q[0]);
    //     LOGINFO("[vision] TX mode=%d yaw=%s pitch=%s qw=%s crc=0x%04X",
    //             tx_frame.mode, y_s, p_s, w_s, tx_frame.crc16);
    //     LOGINFO("[vision] online=%d", VisionIsOnline());
    // }
}

#endif // VISION_USE_VCP
