/**
 * @file master_process.h
 * @brief 视觉通信模块接口定义 (SP协议版本)
 *
 * 提供与 sp_vision_25 上位机通信的接口
 * 支持 VCP (USB虚拟串口) 模式
 */

#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "sp_protocol.h"

/* ==================== 接收数据缓冲区大小 ==================== */
#define VISION_RECV_SIZE SP_RX_FRAME_SIZE // 30字节
#define VISION_SEND_SIZE SP_TX_FRAME_SIZE // 43字节

/* ==================== 接收数据结构 (从上位机) ==================== */
/**
 * @brief 视觉接收数据结构
 *
 * 从上位机接收的控制指令, 用于云台自瞄控制
 */
typedef struct {
    uint8_t mode; // 控制模式: 0-不控制, 1-控制不开火, 2-控制并开火
    uint8_t fire_command; // 开火指令 (mode==2时为1)

    float yaw; // 目标Yaw角度 (度)
    float yaw_vel; // Yaw角速度前馈 (度/秒)
    float yaw_acc; // Yaw角加速度前馈 (度/秒²)

    float pitch; // 目标Pitch角度 (度)
    float pitch_vel; // Pitch角速度前馈 (度/秒)
    float pitch_acc; // Pitch角加速度前馈 (度/秒²)
} Vision_Recv_s;

/* ==================== 发送数据结构 (发送到上位机) ==================== */
/**
 * @brief 视觉发送数据结构
 *
 * 发送给上位机的云台状态, 用于视觉解算
 */
typedef struct {
    uint8_t mode; // 工作模式: 0-空闲, 1-自瞄

    float quaternion[4]; // 姿态四元数 (wxyz)
    float yaw; // 当前Yaw角度 (度)
    float yaw_vel; // 当前Yaw角速度 (度/秒)
    float pitch; // 当前Pitch角度 (度)
    float pitch_vel; // 当前Pitch角速度 (度/秒)

    float bullet_speed; // 弹速 (m/s)
    uint16_t bullet_count; // 累计发弹数
} Vision_Send_s;

/* ==================== 工作模式枚举 ==================== */
typedef enum {
    VISION_MODE_IDLE = 0, // 空闲模式
    VISION_MODE_AUTO_AIM = 1, // 自动瞄准模式
} Vision_Work_Mode_e;

/* ==================== 兼容旧代码的类型定义 ==================== */

/**
 * @brief 敌方颜色枚举 (兼容旧接口)
 */
typedef enum {
    COLOR_NONE = 0,
    COLOR_BLUE = 1,
    COLOR_RED = 2,
} Enemy_Color_e;

/**
 * @brief 弹速枚举 (兼容旧接口)
 */
typedef enum {
    BULLET_SPEED_NONE = 0,
    BIG_AMU_12 = 12, // 42mm 12m/s
    SMALL_AMU_15 = 15, // 17mm 15m/s
    BIG_AMU_16 = 16, // 42mm 16m/s
    SMALL_AMU_20 = 20, // 17mm 20m/s (默认)
    SMALL_AMU_30 = 30, // 17mm 30m/s
} Bullet_Speed_e;

/* ==================== 公开接口函数 ==================== */

/**
 * @brief 初始化视觉通信模块
 *
 * @param _handle 用于通信的串口handle (VCP模式传NULL)
 * @return 接收数据结构体指针
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据帧
 *
 * 将当前设置的云台状态打包并发送给上位机
 */
void VisionSend(void);

/**
 * @brief 设置工作模式
 *
 * @param mode 工作模式 (空闲/自瞄)
 */
void VisionSetMode(Vision_Work_Mode_e mode);

/**
 * @brief 设置姿态四元数
 *
 * @param w 标量部分
 * @param x 向量x
 * @param y 向量y
 * @param z 向量z
 */
void VisionSetQuaternion(float w, float x, float y, float z);

/**
 * @brief 设置云台状态
 *
 * @param yaw       Yaw角度 (度)
 * @param yaw_vel   Yaw角速度 (度/秒)
 * @param pitch     Pitch角度 (度)
 * @param pitch_vel Pitch角速度 (度/秒)
 */
void VisionSetGimbalState(float yaw, float yaw_vel, float pitch, float pitch_vel);

/**
 * @brief 设置弹道信息
 *
 * @param bullet_speed 弹速 (m/s)
 * @param bullet_count 累计发弹数
 */
void VisionSetBulletInfo(float bullet_speed, uint16_t bullet_count);

/**
 * @brief 获取上位机控制模式
 *
 * @return 控制模式: 0-不控制, 1-控制不开火, 2-控制并开火
 */
uint8_t VisionGetMode(void);

/**
 * @brief 检查视觉通信是否在线
 *
 * @return 1-在线, 0-离线
 */
uint8_t VisionIsReady(void);

/**
 * @brief 启用通信测试模式
 *
 * 测试模式下:
 * - 下位机发送规律递增的测试数据
 * - 下位机将上位机发来的数据通过日志输出
 */
void VisionTestMode(void);

/**
 * @brief 获取测试统计信息
 *
 * @return 接收到的有效帧数量
 */
uint32_t VisionGetTestStats(void);

/**
 * @brief 设置云台姿态 (兼容旧接口)
 *
 * @param yaw   Yaw角度 (度)
 * @param pitch Pitch角度 (度)
 * @param roll  Roll角度 (度)
 */
void VisionSetAltitude(float yaw, float pitch, float roll);

#endif // MASTER_PROCESS_H