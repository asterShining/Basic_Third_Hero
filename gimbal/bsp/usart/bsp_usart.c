/**
 * @file bsp_usart.c
 * @author neozng
 * @brief  串口bsp层的实现
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "bsp_usart.h"
#include "bsp_log.h"
#include "stdlib.h"
#include "memory.h"

/* usart service instance, modules' info would be recoreded here using USARTRegister() */
/* usart服务实例,所有注册了usart的模块信息会被保存在这里 */
static uint8_t idx;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

// 这里把 DMA 半传输中断关闭逻辑抽出来复用，作用是保证每次重启接收后都只保留“整包完成/IDLE”这两个真正有效的事件；
// 原因是 HAL 默认会在半传输时再次回调，图传键鼠和裁判协议都会把同一帧解析两次。
static void USARTDisableDMAHalfTransferIT(UART_HandleTypeDef *huart)
{
    if (huart != NULL && huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

// 这里把 HAL 错误码拆分成可读的统计项，作用是让上层能从“USART error”直接定位到噪声、帧错还是 ORE；
// 原因是后续新版裁判接收诊断和图像桥接都需要拿到比单一错误计数更细的证据链。
static void USARTRecordErrorDiag(USARTInstance *instance, uint32_t error_code)
{
    instance->diag.error_callback_count++;
    instance->diag.last_error_code = error_code;
    instance->diag.last_error_tick_ms = HAL_GetTick();

    if ((error_code & HAL_UART_ERROR_PE) != 0u)
        instance->diag.error_pe_count++;
    if ((error_code & HAL_UART_ERROR_NE) != 0u)
        instance->diag.error_ne_count++;
    if ((error_code & HAL_UART_ERROR_FE) != 0u)
        instance->diag.error_fe_count++;
    if ((error_code & HAL_UART_ERROR_ORE) != 0u)
        instance->diag.error_ore_count++;
    if ((error_code & HAL_UART_ERROR_DMA) != 0u)
        instance->diag.error_dma_count++;

    if ((error_code != 0u) &&
        ((error_code & ~(HAL_UART_ERROR_PE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE |
                         HAL_UART_ERROR_ORE | HAL_UART_ERROR_DMA)) != 0u)) {
        instance->diag.error_unknown_count++;
    }
}

/**
 * @brief 启动串口服务,会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *
 * @todo 串口服务会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *       可能还要将此函数修改为extern,使得module可以控制串口的启停
 *
 * @param _instance instance owned by module,模块拥有的串口实例
 */
void USARTServiceInit(USARTInstance *_instance)
{
    // 这里记录服务重启次数，作用是把“只初始化过一次”和“运行中反复被拉起”区分开；
    // 原因是链路抖动问题经常不是彻底离线，而是反复进错误回调后又被拉起，这类问题必须靠计数看出来。
    _instance->diag.service_restart_count++;

    // 这里在重启 DMA 接收前先清零本次长度，作用是避免变长协议误读上一帧长度；
    // 原因是图传键鼠链路需要读取实际帧长，若长度不清零会把历史值带进新一轮解析。
    _instance->recv_len = 0u;
    HAL_UART_AbortReceive(_instance->usart_handle);
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size);
    // 关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
    // 这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
    // 我们只希望处理第一种和第三种情况,因此直接关闭DMA半传输中断
    USARTDisableDMAHalfTransferIT(_instance->usart_handle);
}

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    if (idx >= DEVICE_USART_CNT) // 超过最大实例数
        while (1)
            LOGERROR("[bsp_usart] USART exceed max instance count!");

    for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
            while (1)
                LOGERROR("[bsp_usart] USART instance already registered!");

    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->recv_len = 0u;
    instance->module_callback = init_config->module_callback;

    usart_instance[idx++] = instance;
    USARTServiceInit(instance);
    return instance;
}

/* @todo 当前仅进行了形式上的封装,后续要进一步考虑是否将module的行为与bsp完全分离 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        while (1)
            ; // illegal mode! check your code context! 检查定义instance的代码上下文,可能出现指针越界
        break;
    }
}

/* 串口发送时,gstate会被设为BUSY_TX */
uint8_t USARTIsReady(USARTInstance *_instance)
{
    // 这里严格判断 HAL 发送状态是否回到 READY，作用是避免 DMA/UI/图像桥接把仍在忙的串口误判成可继续发送；
    // 原因是旧实现误用了按位或运算，导致条件几乎恒成立，任何依赖该接口的发送就绪检测都会失效。
    if (_instance == NULL || _instance->usart_handle == NULL) {
        return 0u;
    }
    return (_instance->usart_handle->gState == HAL_UART_STATE_READY) ? 1u : 0u;
}

/**
 * @brief 每次dma/idle中断发生时，都会调用此函数.对于每个uart实例会调用对应的回调进行进一步的处理
 *        例如:视觉协议解析/遥控器解析/裁判系统解析
 *
 * @note  通过__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
 *        这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
 *        我们只希望处理，因此直接关闭DMA半传输中断第一种和第三种情况
 *
 * @param huart 发生中断的串口
 * @param Size 此次接收到的总数居量,暂时没用
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i)
    { // find the instance which is being handled
        if (huart == usart_instance[i]->usart_handle)
        { // call the callback function if it is not NULL
            if (usart_instance[i]->module_callback != NULL)
            {
                // 这里把 HAL 回调给出的真实收包长度保存下来，作用是支持 protobuf 这类变长协议；
                // 原因是仅靠固定 buffer 大小无法判断本次消息边界，图传键鼠解析必须依赖真实长度。
                usart_instance[i]->recv_len = Size;
                usart_instance[i]->diag.rx_event_count++;
                usart_instance[i]->diag.rx_bytes_total += Size;
                usart_instance[i]->diag.last_rx_size = Size;
                usart_instance[i]->module_callback();
                memset(usart_instance[i]->recv_buff, 0, Size); // 接收结束后清空buffer,对于变长数据是必要的
            }
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            USARTDisableDMAHalfTransferIT(usart_instance[i]->usart_handle);
            return; // break the loop
        }
    }
}

/**
 * @brief 当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
 *
 * @note  最常见的错误:奇偶校验/溢出/帧错误
 *
 * @param huart 发生错误的串口
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t now_tick = HAL_GetTick();

    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            USARTInstance *instance = usart_instance[i];
            uint32_t error_code = huart->ErrorCode;

            // 这里遇到串口错误时把长度清零，作用是阻断错误帧长度影响后续解析；
            // 原因是图传链路和裁判链路共用同一套 USART BSP，错误回调必须把变长状态一起复位。
            instance->recv_len = 0u;
            USARTRecordErrorDiag(instance, error_code);

            __HAL_UART_CLEAR_PEFLAG(huart);
            __HAL_UART_CLEAR_FEFLAG(huart);
            __HAL_UART_CLEAR_NEFLAG(huart);
            __HAL_UART_CLEAR_OREFLAG(huart);
            HAL_UART_AbortReceive(huart);
            HAL_UARTEx_ReceiveToIdle_DMA(instance->usart_handle, instance->recv_buff, instance->recv_buff_size);
            USARTDisableDMAHalfTransferIT(instance->usart_handle);

            if ((now_tick - instance->diag.last_error_log_tick >= 100u) || (instance->diag.error_callback_count <= 3u)) {
                instance->diag.last_error_log_tick = now_tick;
                LOGWARNING("[bsp_usart] USART error callback triggered, idx [%d], ec [0x%08lX]", i,
                           (unsigned long)error_code);
            }
            return;
        }
    }
}

const USARTDiagInfo *USARTGetDiagInfo(USARTInstance *_instance)
{
    // 这里统一返回只读诊断指针，作用是让上层按需采样而不复制整份结构；
    // 原因是诊断信息只用于观测，不应该让上层直接改写 BSP 内部状态。
    if (_instance == NULL) {
        return NULL;
    }
    return &_instance->diag;
}
