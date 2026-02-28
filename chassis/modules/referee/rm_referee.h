#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include "usart.h"
#include "referee_protocol.h"
#include "robot_def.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"

extern uint8_t UI_Seq;

#pragma pack(1)
typedef struct
{
	uint8_t Robot_Color;		// 机器人颜色
	uint16_t Robot_ID;			// 本机器人ID
	uint16_t Cilent_ID;			// 本机器人对应的客户端ID
	uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
typedef struct
{
	referee_id_t referee_id;

	xFrameHeader FrameHeader; // 接收到的帧头信息
	uint16_t CmdID;
	ext_game_state_t GameState;							   // 0x0001
	ext_game_result_t GameResult;						   // 0x0002
	ext_game_robot_HP_t GameRobotHP;					   // 0x0003
	ext_event_data_t EventData;							   // 0x0101
	ext_referee_warning_t RefereeWarning;				   // 0x0104
	ext_dart_info_t DartInfo;							   // 0x0105
	ext_game_robot_state_t GameRobotState;				   // 0x0201
	ext_power_heat_data_t PowerHeatData;				   // 0x0202
	ext_game_robot_pos_t GameRobotPos;					   // 0x0203
	ext_buff_musk_t BuffMusk;							   // 0x0204
	ext_robot_hurt_t RobotHurt;							   // 0x0206
	ext_shoot_data_t ShootData;							   // 0x0207
	ext_projectile_allowance_t ProjectileAllowance;		   // 0x0208
	ext_rfid_status_t RFIDStatus;						   // 0x0209
	ext_dart_client_cmd_t DartClientCmd;				   // 0x020A
	ext_ground_robot_position_t GroundRobotPosition;	   // 0x020B
	ext_radar_mark_data_t RadarMarkData;				   // 0x020C
	ext_sentry_info_t SentryInfo;						   // 0x020D
	ext_radar_info_t RadarInfo;							   // 0x020E

	// What: 保存0x0301完整协议包(最大112字节用户段)；Why: 对齐通信协议.md并避免上层只能读取裁剪数据
	ext_robot_interaction_data_t RobotInteractionData;	   // 0x0301(最大长度)

	// What: 保留本工程5字节子内容缓存；Why: 不破坏现有UI/多机通信逻辑读取路径
	Communicate_ReceiveData_t ReceiveData;
	ext_custom_robot_data_t CustomRobotData;	 // 0x0302
	ext_map_command_t MapCommand;				 // 0x0303
	ext_map_robot_data_t MapRobotData;			 // 0x0305
	ext_custom_client_data_t CustomClientData;	 // 0x0306
	ext_map_data_t MapData;						 // 0x0307
	ext_custom_info_t CustomInfo;				 // 0x0308
	ext_robot_custom_data_t RobotCustomData;	 // 0x0309
	ext_robot_custom_data_2_t RobotCustomData2; // 0x0310
	ext_robot_custom_data_3_t RobotCustomData3; // 0x0311

	uint8_t init_flag;

} referee_info_t;

// 模式是否切换标志位，0为未切换，1为切换，static定义默认为0
typedef struct
{
	uint32_t chassis_flag : 1;
	uint32_t gimbal_flag : 1;
	uint32_t shoot_flag : 1;
	uint32_t lid_flag : 1;
	uint32_t friction_flag : 1;
	uint32_t Power_flag : 1;
} Referee_Interactive_Flag_t;

// 此结构体包含UI绘制与机器人车间通信的需要的其他非裁判系统数据
typedef struct
{
	Referee_Interactive_Flag_t Referee_Interactive_Flag;
	// 为UI绘制以及交互数据所用
	chassis_mode_e chassis_mode;			 // 底盘模式
	gimbal_mode_e gimbal_mode;				 // 云台模式
	shoot_mode_e shoot_mode;				 // 发射模式设置
	friction_mode_e friction_mode;			 // 摩擦轮关闭
	lid_mode_e lid_mode;					 // 弹舱盖打开
	Chassis_Power_Data_s Chassis_Power_Data; // 功率控制

	// 上一次的模式，用于flag判断
	chassis_mode_e chassis_last_mode;
	gimbal_mode_e gimbal_last_mode;
	shoot_mode_e shoot_last_mode;
	friction_mode_e friction_last_mode;
	lid_mode_e lid_last_mode;
	Chassis_Power_Data_s Chassis_last_Power_Data;

} Referee_Interactive_info_t;

// 裁判接收链路诊断信息，记录“中断触发->协议校验->离线判定”的阶段计数，便于快速定位首个失败环节
typedef struct
{
	uint32_t rx_callback_count;   // 接收回调触发次数，用于确认UART->模块回调路径是否畅通
	uint32_t rx_byte_count;       // 接收总字节数，用于判断链路是否长期无数据
	uint16_t last_rx_size;        // 最近一次接收长度，用于区分空包/短包/正常包
	uint32_t valid_frame_count;   // 成功通过CRC并完成解包的帧数，作为“有效数据到达”的核心指标
	uint32_t sof_miss_count;      // 非SOF字节跳过次数，偏高通常意味着链路噪声或波特率不匹配
	uint32_t short_frame_count;   // 帧截断次数，说明本次DMA数据包不足以形成完整帧
	uint32_t frame_len_error_count; // 帧长异常次数，说明头部DataLength不可信或数据已损坏
	uint32_t crc8_fail_count;     // 帧头CRC8失败次数，优先排查头部同步和链路质量
	uint32_t crc16_fail_count;    // 整帧CRC16失败次数，优先排查物理层噪声和串口参数
	uint32_t cmd_unknown_count;   // 未识别CmdID次数，用于识别协议版本不一致或数据错位
	uint32_t cmd_len_mismatch_count; // 已识别CmdID但长度不匹配次数，用于定位“协议版本不一致但链路正常”
	uint16_t last_cmd_id;         // 最近一次成功解析的CmdID，便于确认系统当前主要收到哪类包
	uint16_t last_data_len;       // 最近一次解析帧的DataLength，便于核对协议长度定义
	uint16_t last_len_mismatch_cmd_id; // 最近一次长度不匹配的CmdID，便于快速定位具体命令
	uint16_t last_frame_len;      // 最近一次成功解析的帧长，便于与协议期望长度对比
	uint32_t daemon_reload_count; // 成功喂狗次数，仅在有效帧后增加，避免脏数据掩盖离线
	uint32_t lost_count;          // 离线回调触发次数，用于衡量链路稳定性
	uint32_t last_valid_tick_ms;  // 最近一次成功解析时间戳，便于估算数据新鲜度
	uint32_t last_lost_tick_ms;   // 最近一次离线时间戳，便于关联系统其他异常事件
} RefereeRxDiag_s;

#pragma pack()

/**
 * @brief 裁判系统通信初始化,该函数会初始化裁判系统串口,开启中断
 *
 * @param referee_usart_handle 串口handle,C板一般用串口6
 * @return referee_info_t* 返回裁判系统反馈的数据,包括热量/血量/状态等
 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief UI绘制和交互数的发送接口,由UI绘制任务和多机通信函数调用
 * @note 内部包含了一个实时系统的延时函数,这是因为裁判系统接收CMD数据至高位10Hz
 *
 * @param send 发送数据首地址
 * @param tx_len 发送长度
 */
void RefereeSend(uint8_t *send, uint16_t tx_len);

/**
 * @brief 获取裁判接收链路诊断信息
 *
 * @return const RefereeRxDiag_s* 诊断信息指针
 */
const RefereeRxDiag_s *RefereeGetRxDiag(void);

#endif // !REFEREE_H
