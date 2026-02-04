/**
 * @file master_process.c
 * @brief 视觉通信模块实现 (SP协议版本)
 *
 * 实现与 sp_vision_25 上位机的通信
 * 支持 VCP (USB虚拟串口) 模式
 */

#include "master_process.h"
#include "sp_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"

/* ==================== 静态变量 ==================== */
static Vision_Recv_s recv_data; // 接收数据缓存
static Vision_Send_s send_data; // 发送数据缓存
static SP_GimbalToVision_t tx_frame; // SP协议发送帧
static SP_VisionToGimbal_t rx_frame; // SP协议接收帧
static DaemonInstance *vision_daemon; // 离线检测守护进程

/* ==================== 测试模式变量 ==================== */
static uint8_t test_mode_enabled = 0; // 测试模式标志
static uint32_t test_counter = 0; // 测试计数器
static uint32_t rx_frame_count = 0; // 接收帧计数

/* ==================== 离线回调函数 ==================== */
/**
 * @brief 视觉离线回调
 *
 * 功能: 当视觉通信超时时被 daemon 调用
 * 原因: 检测通信异常并尝试恢复
 */
static void VisionOfflineCallback(void *id)
{
    LOGWARNING("[Vision] SP通信离线, 尝试恢复...");
    // VCP模式下无需重启, 等待上位机重新发送即可
}

/* ==================== VCP 模式实现 ==================== */
#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff; // USB接收缓冲区指针

/**
 * @brief USB接收回调 (解析上位机数据)
 *
 * 功能: 解析上位机发来的 VisionToGimbal 帧
 * 原因: 提取控制指令供云台应用使用
 */
static void DecodeVision(uint16_t recv_len)
{
    // 尝试解析SP协议帧
    if (SP_ParseRxFrame(vis_recv_buff, recv_len, &rx_frame)) {
        // 解析成功, 更新接收数据
        recv_data.mode = rx_frame.mode;
        recv_data.fire_command = (rx_frame.mode == SP_CTRL_FIRE) ? 1 : 0;
        recv_data.yaw = rx_frame.yaw;
        recv_data.yaw_vel = rx_frame.yaw_vel;
        recv_data.yaw_acc = rx_frame.yaw_acc;
        recv_data.pitch = rx_frame.pitch;
        recv_data.pitch_vel = rx_frame.pitch_vel;
        recv_data.pitch_acc = rx_frame.pitch_acc;

        // 喂狗
        DaemonReload(vision_daemon);
        rx_frame_count++;

        // 测试模式下输出日志
        if (test_mode_enabled) {
            LOGINFO("[Vision Test] Recv mode=%d, yaw=%.2f, pitch=%.2f",
                    rx_frame.mode, rx_frame.yaw, rx_frame.pitch);
        }
    }
}

/**
 * @brief 初始化视觉通信 (VCP模式)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // VCP模式不使用UART句柄

    // 初始化USB VCP
    USB_Init_Config_s conf = { .rx_cbk = DecodeVision };
    vis_recv_buff = USBInit(conf);

    // 初始化发送帧
    SP_InitTxFrame(&tx_frame);

    // 注册离线检测守护进程
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = NULL,
        .reload_count = 10, // 100ms超时
    };
    vision_daemon = DaemonRegister(&daemon_conf);

    LOGINFO("[Vision] SP协议初始化完成 (VCP模式)");
    return &recv_data;
}

/**
 * @brief 发送视觉数据帧 (VCP模式)
 */
void VisionSend(void)
{
    // 测试模式: 发送规律数据
    if (test_mode_enabled) {
        tx_frame.mode = SP_MODE_AUTO_AIM;
        tx_frame.q[0] = 1.0f;
        tx_frame.q[1] = 0.0f;
        tx_frame.q[2] = 0.0f;
        tx_frame.q[3] = 0.0f;
        tx_frame.yaw = (float)(test_counter % 360);
        tx_frame.yaw_vel = 10.0f;
        tx_frame.pitch = 0.0f;
        tx_frame.pitch_vel = 0.0f;
        tx_frame.bullet_speed = 16.0f;
        tx_frame.bullet_count = (uint16_t)(test_counter & 0xFFFF);
        test_counter++;
    } else {
        // 正常模式: 使用设置的发送数据
        tx_frame.mode = send_data.mode;
        SP_SetQuaternion(&tx_frame,
                         send_data.quaternion[0], send_data.quaternion[1],
                         send_data.quaternion[2], send_data.quaternion[3]);
        SP_SetGimbalState(&tx_frame,
                          send_data.yaw, send_data.yaw_vel,
                          send_data.pitch, send_data.pitch_vel);
        SP_SetBulletInfo(&tx_frame, send_data.bullet_speed, send_data.bullet_count);
    }

    // 计算CRC并发送
    SP_CalcTxCRC(&tx_frame);
    USBTransmit((uint8_t *)&tx_frame, sizeof(tx_frame));
}

#endif // VISION_USE_VCP

/* ==================== UART 模式实现 ==================== */
#ifdef VISION_USE_UART

#include "bsp_usart.h"
static USARTInstance *vision_usart_instance;

/**
 * @brief UART接收回调 (解析上位机数据)
 */
static void DecodeVision(void)
{
    DaemonReload(vision_daemon);

    if (SP_ParseRxFrame(vision_usart_instance->recv_buff, VISION_RECV_SIZE, &rx_frame)) {
        recv_data.mode = rx_frame.mode;
        recv_data.fire_command = (rx_frame.mode == SP_CTRL_FIRE) ? 1 : 0;
        recv_data.yaw = rx_frame.yaw;
        recv_data.yaw_vel = rx_frame.yaw_vel;
        recv_data.yaw_acc = rx_frame.yaw_acc;
        recv_data.pitch = rx_frame.pitch;
        recv_data.pitch_vel = rx_frame.pitch_vel;
        recv_data.pitch_acc = rx_frame.pitch_acc;
        rx_frame_count++;

        if (test_mode_enabled) {
            LOGINFO("[Vision Test] Recv mode=%d, yaw=%.2f, pitch=%.2f",
                    rx_frame.mode, rx_frame.yaw, rx_frame.pitch);
        }
    }
}

/**
 * @brief 初始化视觉通信 (UART模式)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    SP_InitTxFrame(&tx_frame);

    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon = DaemonRegister(&daemon_conf);

    LOGINFO("[Vision] SP协议初始化完成 (UART模式)");
    return &recv_data;
}

/**
 * @brief 发送视觉数据帧 (UART模式)
 */
void VisionSend(void)
{
    if (test_mode_enabled) {
        tx_frame.mode = SP_MODE_AUTO_AIM;
        tx_frame.q[0] = 1.0f;
        tx_frame.q[1] = 0.0f;
        tx_frame.q[2] = 0.0f;
        tx_frame.q[3] = 0.0f;
        tx_frame.yaw = (float)(test_counter % 360);
        tx_frame.yaw_vel = 10.0f;
        tx_frame.pitch = 0.0f;
        tx_frame.pitch_vel = 0.0f;
        tx_frame.bullet_speed = 16.0f;
        tx_frame.bullet_count = (uint16_t)(test_counter & 0xFFFF);
        test_counter++;
    } else {
        tx_frame.mode = send_data.mode;
        SP_SetQuaternion(&tx_frame,
                         send_data.quaternion[0], send_data.quaternion[1],
                         send_data.quaternion[2], send_data.quaternion[3]);
        SP_SetGimbalState(&tx_frame,
                          send_data.yaw, send_data.yaw_vel,
                          send_data.pitch, send_data.pitch_vel);
        SP_SetBulletInfo(&tx_frame, send_data.bullet_speed, send_data.bullet_count);
    }

    SP_CalcTxCRC(&tx_frame);
    USARTSend(vision_usart_instance, (uint8_t *)&tx_frame, sizeof(tx_frame), USART_TRANSFER_DMA);
}

#endif // VISION_USE_UART

/* ==================== 公共接口实现 ==================== */

/**
 * @brief 设置工作模式
 */
void VisionSetMode(Vision_Work_Mode_e mode)
{
    send_data.mode = (uint8_t)mode;
}

/**
 * @brief 设置姿态四元数
 */
void VisionSetQuaternion(float w, float x, float y, float z)
{
    send_data.quaternion[0] = w;
    send_data.quaternion[1] = x;
    send_data.quaternion[2] = y;
    send_data.quaternion[3] = z;
}

/**
 * @brief 设置云台状态
 */
void VisionSetGimbalState(float yaw, float yaw_vel, float pitch, float pitch_vel)
{
    send_data.yaw = yaw;
    send_data.yaw_vel = yaw_vel;
    send_data.pitch = pitch;
    send_data.pitch_vel = pitch_vel;
}

/**
 * @brief 设置弹道信息
 */
void VisionSetBulletInfo(float bullet_speed, uint16_t bullet_count)
{
    send_data.bullet_speed = bullet_speed;
    send_data.bullet_count = bullet_count;
}

/**
 * @brief 获取上位机控制模式
 */
uint8_t VisionGetMode(void)
{
    return recv_data.mode;
}

/**
 * @brief 检查视觉是否在线
 */
uint8_t VisionIsReady(void)
{
    return DaemonIsOnline(vision_daemon);
}

/**
 * @brief 启用测试模式
 */
void VisionTestMode(void)
{
    test_mode_enabled = 1;
    test_counter = 0;
    LOGINFO("[Vision] 测试模式已启用");
}

/**
 * @brief 获取测试统计
 */
uint32_t VisionGetTestStats(void)
{
    return rx_frame_count;
}

/**
 * @brief 设置云台姿态 (兼容旧接口)
 *
 * 功能: 通过欧拉角设置云台姿态 (简化版)
 * 原因: 兼容ins_task.c中的调用, 暂时注释掉不使用
 */
void VisionSetAltitude(float yaw, float pitch, float roll)
{
    // 暂时仅更新角度, 不计算四元数
    // 实际使用时应调用 VisionSetGimbalState 和 VisionSetQuaternion
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    // roll 暂不使用
    (void)roll;
}
