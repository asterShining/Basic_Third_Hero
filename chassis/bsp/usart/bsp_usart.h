#ifndef BSP_RC_H
#define BSP_RC_H

#include <stdint.h>
#include "main.h"

#define DEVICE_USART_CNT 3     // C板至多分配3个串口
#define USART_RXBUFF_LIMIT 256 // 如果协议需要更大的buff,请修改这里

// 模块回调函数,用于解析协议
typedef void (*usart_module_callback)();

/* 发送模式枚举 */
typedef enum
{
    USART_TRANSFER_NONE=0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

// 串口诊断信息，用于分阶段定位问题（接收是否触发、错误类型、重启次数），避免只看到“离线”却无法定位首个异常点
typedef struct
{
    uint32_t service_restart_count; // 记录接收服务重启次数，用于判断是否频繁被错误/离线回调拉起
    uint32_t rx_event_count;        // 记录IDLE/DMA完成触发次数，用于确认接收中断路径是否正常运行
    uint32_t rx_bytes_total;        // 记录累计接收字节数，用于评估链路吞吐与是否存在长期无数据
    uint16_t last_rx_size;          // 记录最近一次接收长度，便于定位是空包还是短包导致解析失败

    uint32_t error_callback_count; // 记录错误回调总次数，用于衡量链路稳定性
    uint32_t error_pe_count;       // 奇偶校验错误次数，常见于校验参数不一致
    uint32_t error_ne_count;       // 噪声错误次数，常见于电气噪声或地线问题
    uint32_t error_fe_count;       // 帧错误次数，常见于波特率/停止位不匹配或波形畸变
    uint32_t error_ore_count;      // 过载错误次数，常见于中断/ DMA 服务不及时
    uint32_t error_dma_count;      // DMA错误次数，用于排查DMA链路异常
    uint32_t error_unknown_count;  // 未知错误次数，保留给非预期错误位

    uint32_t last_error_code;     // 最近一次ErrorCode原值，便于离线后复盘
    uint32_t last_error_tick_ms;  // 最近一次错误时间戳，便于关联其他任务日志
    uint32_t last_error_log_tick; // 错误日志限频时间戳，避免高频中断刷屏影响可读性
} USARTDiagInfo;

// 串口实例结构体,每个module都要包含一个实例.
// 由于串口是独占的点对点通信,所以不需要考虑多个module同时使用一个串口的情况,因此不用加入id;当然也可以选择加入,这样在bsp层可以访问到module的其他信息
typedef struct
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; // 预先定义的最大buff大小,如果太小请修改USART_RXBUFF_LIMIT
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
    USARTDiagInfo diag;                    // 串口分阶段诊断信息，复用给所有基于USART的模块
} USARTInstance;

/* usart 初始化配置结构体 */
typedef struct
{
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} USART_Init_Config_s;

/**
 * @brief 注册一个串口实例,返回一个串口实例指针
 *
 * @param init_config 传入串口初始化结构体
 */
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);

/**
 * @brief 启动串口服务,需要传入一个usart实例.一般用于lost callback的情况(使用串口的模块daemon)
 *
 * @param _instance
 */
void USARTServiceInit(USARTInstance *_instance);


/**
 * @brief 通过调用该函数可以发送一帧数据,需要传入一个usart实例,发送buff以及这一帧的长度
 * @note 在短时间内连续调用此接口,若采用IT/DMA会导致上一次的发送未完成而新的发送取消.
 * @note 若希望连续使用DMA/IT进行发送,请配合USARTIsReady()使用,或自行为你的module实现一个发送队列和任务.
 * @todo 是否考虑为USARTInstance增加发送队列以进行连续发送?
 * 
 * @param _instance 串口实例
 * @param send_buf 待发送数据的buffer
 * @param send_size how many bytes to send
 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size,USART_TRANSFER_MODE mode);

/**
 * @brief 判断串口是否准备好,用于连续或异步的IT/DMA发送
 *
 * @param _instance 要判断的串口实例
 * @return uint8_t ready 1, busy 0
 */
uint8_t USARTIsReady(USARTInstance *_instance);

/**
 * @brief 获取串口实例的诊断信息快照入口
 *
 * @param _instance 串口实例
 * @return const USARTDiagInfo* 诊断信息指针，若入参为空则返回NULL
 */
const USARTDiagInfo *USARTGetDiagInfo(USARTInstance *_instance);

#endif
