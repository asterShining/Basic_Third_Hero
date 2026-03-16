#ifndef VIDEO_LINK_KM_H
#define VIDEO_LINK_KM_H

#include "remote_control.h"

/* 图传键鼠链路诊断信息 */
typedef struct
{
    uint32_t rx_frame_count;
    uint32_t decode_success_count;
    uint32_t decode_fail_count;
    uint16_t last_frame_len;
    int32_t last_mouse_z;
    uint8_t last_mid_button;
} VideoLinkKM_Diag_s;

/**
 * @brief 初始化图传键鼠模块
 *
 * @param video_link_usart_handle 图传串口句柄
 * @return RC_ctrl_t* 返回与遥控器模块一致的键鼠数据视图
 */
RC_ctrl_t *VideoLinkKMInit(UART_HandleTypeDef *video_link_usart_handle);

/**
 * @brief 检查图传键鼠链路是否在线
 *
 * @return uint8_t 1:在线 0:离线
 */
uint8_t VideoLinkKMIsOnline(void);

/**
 * @brief 检查图传键鼠是否已经成功解析过有效帧
 *
 * @return uint8_t 1:已有有效帧 0:尚无有效帧
 */
uint8_t VideoLinkKMHasValidFrame(void);

/**
 * @brief 获取图传键鼠链路诊断信息
 *
 * @return const VideoLinkKM_Diag_s* 诊断信息指针
 */
const VideoLinkKM_Diag_s *VideoLinkKMGetDiag(void);

#endif
