#ifndef VIDEO_LINK_KM_H
#define VIDEO_LINK_KM_H

#include "remote_control.h"

/* VT03 图传键鼠链路诊断信息 */
typedef struct
{
    uint32_t rx_frame_count;
    uint32_t valid_frame_count;
    uint32_t header_fail_count;
    uint32_t crc_fail_count;
    uint16_t last_frame_len;
    int16_t last_mouse_z;
    uint8_t last_mid_button;
    uint8_t last_switch_position;
    uint8_t last_trigger_state;
} VideoLinkKM_Diag_s;

/* VT03 遥控器主控状态 */
typedef struct
{
    uint8_t mode_sw;
    uint8_t pause_button_down;
    uint8_t fn_left_button_down;
    uint8_t fn_right_button_down;
    uint8_t trigger_button_down;
} VideoLinkKM_RemoteState_s;

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

/**
 * @brief 获取最近一次有效 VT03 遥控器状态
 *
 * @return const VideoLinkKM_RemoteState_s* 遥控器状态指针
 */
const VideoLinkKM_RemoteState_s *VideoLinkKMGetRemoteState(void);

#endif
