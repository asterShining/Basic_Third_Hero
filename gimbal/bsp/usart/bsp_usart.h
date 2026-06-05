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

// 串口诊断信息统一记录“服务是否重启、最近一次收到多长、错误具体是什么类型”，作用是让上层能快速分辨问题在物理层还是解析层；
// 原因是图传键鼠、裁判系统和后续自定义图像桥接都会复用这套 USART BSP，若仍只知道“串口错了”会很难定位到首个失效环节。
typedef struct
{
    uint32_t service_restart_count; // 记录接收服务重启次数，用于判断链路是否频繁被错误回调拉起
    uint32_t rx_event_count;        // 记录 RX 事件触发次数，用于确认 DMA+IDLE 中断路径是否在持续工作
    uint32_t rx_bytes_total;        // 记录累计接收字节数，用于观察链路吞吐和长期空闲状态
    uint16_t last_rx_size;          // 记录最近一次接收长度，便于上层把短包、空包和正常包区分开

    uint32_t error_callback_count; // 记录错误回调总次数，快速判断链路是否稳定
    uint32_t error_pe_count;       // 记录奇偶校验错误次数，常用于排查参数不一致
    uint32_t error_ne_count;       // 记录噪声错误次数，常用于排查电气噪声
    uint32_t error_fe_count;       // 记录帧错误次数，常用于排查波特率或停止位问题
    uint32_t error_ore_count;      // 记录过载错误次数，常用于排查服务不及时
    uint32_t error_dma_count;      // 记录 DMA 错误次数，用于排查 DMA 链路异常
    uint32_t error_unknown_count;  // 记录未归类错误次数，保留给 HAL 版本差异或异常状态位

    uint32_t last_error_code;     // 保存最近一次 HAL ErrorCode 原值，便于和日志对照
    uint32_t last_error_tick_ms;  // 保存最近一次错误时间戳，便于关联其它任务的异常窗口
    uint32_t last_error_log_tick; // 保存错误日志限频时间戳，避免高频异常刷屏影响排障
} USARTDiagInfo;

// 串口实例结构体,每个module都要包含一个实例.
// 由于串口是独占的点对点通信,所以不需要考虑多个module同时使用一个串口的情况,因此不用加入id;当然也可以选择加入,这样在bsp层可以访问到module的其他信息
typedef struct
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; // 预先定义的最大buff大小,如果太小请修改USART_RXBUFF_LIMIT
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    uint16_t recv_len;                     // 本次实际收到的数据长度,供变长协议读取
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
    USARTDiagInfo diag;                    // 统一挂载串口诊断快照，供裁判系统和图传桥接共用
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
 * @brief 获取串口实例当前的诊断信息快照入口
 *
 * @param _instance 串口实例
 * @return const USARTDiagInfo* 诊断信息指针；若实例无效则返回NULL
 */
const USARTDiagInfo *USARTGetDiagInfo(USARTInstance *_instance);

#endif
