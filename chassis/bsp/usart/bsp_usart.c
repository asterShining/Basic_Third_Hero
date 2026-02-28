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
static USARTInstance *usart_instance[DEVICE_USART_CNT] = { NULL };

// 统一关闭DMA半传输中断，防止HAL在半传输时重复触发RxEvent回调导致模块重复解析同一批数据
static void USARTDisableDMAHalfTransferIT(UART_HandleTypeDef *huart)
{
    if (huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

// 记录UART错误位的分项统计，目的是把“串口出错”细化为具体类型，直接定位到参数/电气/实时性问题
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

    // 记录非标准错误位，目的是在HAL版本差异或底层异常时保留线索，避免误判为“无错误”
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
    // 记录服务重启次数，目的是区分“初始化启动一次”与“运行期频繁重启”两种问题形态
    _instance->diag.service_restart_count++;

    /* 终止所有可能已经卡死的接收过程并重置接收状态 (HAL_BUSY), 以防止后续重启DMA失败 */
    HAL_UART_AbortReceive(_instance->usart_handle);

    if (HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size) != HAL_OK) {
        LOGERROR("[bsp_usart] ReceiveToIdle start failed, instance [%p], restart_cnt [%lu]",
                 _instance,
                 (unsigned long)_instance->diag.service_restart_count);
        return;
    }
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

    if (init_config->recv_buff_size == 0u || init_config->recv_buff_size > USART_RXBUFF_LIMIT)
        while (1)
            LOGERROR("[bsp_usart] invalid recv buff size [%u], limit [%u]",
                     (unsigned int)init_config->recv_buff_size,
                     (unsigned int)USART_RXBUFF_LIMIT);

    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;

    usart_instance[idx++] = instance;
    USARTServiceInit(instance);
    return instance;
}

/* @todo 当前仅进行了形式上的封装,后续要进一步考虑是否将module的行为与bsp完全分离 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode) {
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
    if (_instance->usart_handle->gState | HAL_UART_STATE_BUSY_TX)
        return 0;
    else
        return 1;
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
    for (uint8_t i = 0; i < idx; ++i) { // find the instance which is being handled
        if (huart == usart_instance[i]->usart_handle) { // call the callback function if it is not NULL
            uint16_t valid_size = Size;
            if (valid_size > usart_instance[i]->recv_buff_size) {
                // 对Size做上限裁剪，目的是防止异常长度导致后续memset越界
                valid_size = usart_instance[i]->recv_buff_size;
            }

            usart_instance[i]->diag.rx_event_count++;
            usart_instance[i]->diag.rx_bytes_total += valid_size;
            usart_instance[i]->diag.last_rx_size = valid_size;

            if (usart_instance[i]->module_callback != NULL) {
                usart_instance[i]->module_callback();
                memset(usart_instance[i]->recv_buff, 0, valid_size); // 接收结束后清空buffer,对于变长数据是必要的
            }

            if (HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff,
                                             usart_instance[i]->recv_buff_size) != HAL_OK) {
                LOGERROR("[bsp_usart] ReceiveToIdle restart failed in rx event, idx [%u]",
                         (unsigned int)i);
                return;
            }
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
    for (uint8_t i = 0; i < idx; ++i) {
        if (huart == usart_instance[i]->usart_handle) {
            USARTInstance *instance = usart_instance[i];
            /* 提前保存 ErrorCode, 因为 HAL_UART_AbortReceive 或 ReceiveToIdle 可能会将其清零，导致无法准确定位错误类型 */
            uint32_t error_code = huart->ErrorCode;
            uint32_t now_tick = HAL_GetTick();

            USARTRecordErrorDiag(instance, error_code);

            // 主动清除常见错误标志位，目的是避免硬件错误标志残留导致DMA重启后立即再次报错
            __HAL_UART_CLEAR_PEFLAG(huart);
            __HAL_UART_CLEAR_FEFLAG(huart);
            __HAL_UART_CLEAR_NEFLAG(huart);
            __HAL_UART_CLEAR_OREFLAG(huart);

            /* 终止当前异常的接收操作并重置状态寄存器，以确保因为 ORE 等错误发生后，串口不会一直卡在 BUSY_RX 状态 */
            HAL_UART_AbortReceive(huart);

            if (HAL_UARTEx_ReceiveToIdle_DMA(instance->usart_handle, instance->recv_buff, instance->recv_buff_size) != HAL_OK) {
                LOGERROR("[bsp_usart] ReceiveToIdle restart failed in error callback, idx [%u], EC [0x%08lX]",
                         (unsigned int)i,
                         (unsigned long)error_code);
                return;
            }
            USARTDisableDMAHalfTransferIT(instance->usart_handle);

            // 错误日志限频输出，目的是保留关键错误信息同时避免高频日志淹没真正根因
            if ((now_tick - instance->diag.last_error_log_tick >= 100u) || (instance->diag.error_callback_count <= 3u)) {
                instance->diag.last_error_log_tick = now_tick;
                LOGWARNING("[bsp_usart][stage:uart_error] idx[%u] EC[0x%08lX] PE:%u NE:%u FE:%u ORE:%u DMA:%u total:%lu last_rx:%u",
                           (unsigned int)i,
                           (unsigned long)error_code,
                           (unsigned int)((error_code & HAL_UART_ERROR_PE) != 0u),
                           (unsigned int)((error_code & HAL_UART_ERROR_NE) != 0u),
                           (unsigned int)((error_code & HAL_UART_ERROR_FE) != 0u),
                           (unsigned int)((error_code & HAL_UART_ERROR_ORE) != 0u),
                           (unsigned int)((error_code & HAL_UART_ERROR_DMA) != 0u),
                           (unsigned long)instance->diag.error_callback_count,
                           (unsigned int)instance->diag.last_rx_size);
            }
            return;
        }
    }
}

const USARTDiagInfo *USARTGetDiagInfo(USARTInstance *_instance)
{
    if (_instance == NULL) {
        return NULL;
    }
    return &_instance->diag;
}
