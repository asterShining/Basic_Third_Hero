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

// 此结构体承载云台板裁判系统最新协议层的接收结果，同时保留旧工程的 0x0301 5 字节业务缓存入口，避免上层读取路径被顺手改坏。
typedef struct
{
	referee_id_t referee_id;

	xFrameHeader FrameHeader;					 // 接收到的帧头信息
	uint16_t CmdID;								 // 最近一次成功解析的命令字
	ext_game_state_t GameState;					 // 0x0001
	ext_game_result_t GameResult;				 // 0x0002
	ext_game_robot_HP_t GameRobotHP;			 // 0x0003
	ext_event_data_t EventData;					 // 0x0101
	ext_referee_warning_t RefereeWarning;		 // 0x0104
	ext_dart_info_t DartInfo;					 // 0x0105
	ext_game_robot_state_t GameRobotState;		 // 0x0201
	ext_power_heat_data_t PowerHeatData;		 // 0x0202
	ext_game_robot_pos_t GameRobotPos;			 // 0x0203
	ext_buff_musk_t BuffMusk;					 // 0x0204
	ext_robot_hurt_t RobotHurt;					 // 0x0206
	ext_shoot_data_t ShootData;					 // 0x0207
	ext_projectile_allowance_t ProjectileAllowance; // 0x0208
	ext_rfid_status_t RFIDStatus;				 // 0x0209
	ext_dart_client_cmd_t DartClientCmd;		 // 0x020A
	ext_ground_robot_position_t GroundRobotPosition; // 0x020B
	ext_radar_mark_data_t RadarMarkData;		 // 0x020C
	ext_sentry_info_t SentryInfo;				 // 0x020D
	ext_radar_info_t RadarInfo;					 // 0x020E
	ext_robot_interaction_data_t RobotInteractionData; // 0x0301 完整最大负载

	// 这里继续保留本工程历史上的 5 字节 0x0301 子内容缓存，作用是让旧 UI/车间通信调用继续读取原路径；
	// 原因是用户要求“同步协议层但不要顺手重构控制逻辑”，因此兼容层必须留在协议对象里。
	Communicate_ReceiveData_t ReceiveData;		 // 0x0301 本工程裁剪缓存
	ext_custom_robot_data_t CustomRobotData;	 // 0x0302
	ext_map_command_t MapCommand;				 // 0x0303
	ext_map_robot_data_t MapRobotData;			 // 0x0305
	ext_custom_client_data_t CustomClientData;	 // 0x0306
	ext_map_data_t MapData;						 // 0x0307
	ext_custom_info_t CustomInfo;				 // 0x0308
	ext_robot_custom_data_t RobotCustomData;	 // 0x0309
	ext_robot_custom_data_2_t RobotCustomData2; // 0x0310
	ext_robot_custom_data_3_t RobotCustomData3; // 0x0311

	uint32_t last_shoot_data_tick_ms; // 记录最近一次 0x0207 到达时刻，给上层做超时清零和调试用
	uint8_t init_flag;                // 保留旧工程初始化标志，避免 UI 任务接口编译回归
} referee_info_t;

// 模式是否切换标志位，0为未切换，1为切换，继续保留给云台旧 UI 逻辑使用。
typedef struct
{
	uint32_t chassis_flag : 1;
	uint32_t gimbal_flag : 1;
	uint32_t shoot_flag : 1;
	uint32_t lid_flag : 1;
	uint32_t friction_flag : 1;
	uint32_t Power_flag : 1;
} Referee_Interactive_Flag_t;

// 这里保持云台板旧 UI 所依赖的实时交互结构不变，作用是让协议层升级不连带打穿未启用但仍参与编译的 UI 代码。
typedef struct
{
	Referee_Interactive_Flag_t Referee_Interactive_Flag;
	chassis_mode_e chassis_mode;
	gimbal_mode_e gimbal_mode;
	shoot_mode_e shoot_mode;
	friction_mode_e friction_mode;
	lid_mode_e lid_mode;
	Chassis_Power_Data_s Chassis_Power_Data;

	chassis_mode_e chassis_last_mode;
	gimbal_mode_e gimbal_last_mode;
	shoot_mode_e shoot_last_mode;
	friction_mode_e friction_last_mode;
	lid_mode_e lid_last_mode;
	Chassis_Power_Data_s Chassis_last_Power_Data;
} Referee_Interactive_info_t;

// 裁判接收链路诊断信息，按“回调触发 -> 帧校验 -> 命令匹配 -> 离线回调”的阶段收集证据，便于快速定位云台板协议问题。
typedef struct
{
	uint32_t rx_callback_count;		  // 接收回调触发次数，用于确认 UART -> 模块回调路径是否畅通
	uint32_t rx_byte_count;			  // 接收总字节数，用于判断链路是否长期无数据
	uint16_t last_rx_size;			  // 最近一次接收长度，用于区分空包、短包和正常包
	uint32_t valid_frame_count;		  // 成功通过 CRC 并完成解包的帧数，作为“有效数据到达”的核心指标
	uint32_t sof_miss_count;		  // 非 SOF 字节跳过次数，偏高通常意味着链路错位或噪声
	uint32_t short_frame_count;		  // 帧截断次数，说明本次 DMA 数据不足以组成完整一帧
	uint32_t frame_len_error_count;	  // 帧长异常次数，说明头部 DataLength 已不可信
	uint32_t crc8_fail_count;		  // 帧头 CRC8 失败次数，优先排查头部同步问题
	uint32_t crc16_fail_count;		  // 整帧 CRC16 失败次数，优先排查物理层质量
	uint32_t cmd_unknown_count;		  // 未识别命令码次数，用于识别协议版本不一致
	uint32_t cmd_len_mismatch_count;  // 命令码已识别但长度不匹配次数，用于识别协议长度漂移
	uint16_t last_cmd_id;			  // 最近一次成功解析的命令码
	uint16_t last_data_len;			  // 最近一次解析到的 DataLength
	uint16_t last_len_mismatch_cmd_id; // 最近一次长度不匹配的命令码
	uint16_t last_frame_len;		  // 最近一次成功解析的整帧长度
	uint32_t daemon_reload_count;	  // 仅在有效帧后累计的喂狗次数，避免坏包掩盖真实离线
	uint32_t lost_count;			  // 离线回调触发次数
	uint32_t last_valid_tick_ms;	  // 最近一次成功解析时间戳
	uint32_t last_lost_tick_ms;		  // 最近一次离线时间戳
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
 * @brief 裁判系统原始发送接口
 *
 * @note 该接口不附带兼容延时，供协议层或后续新任务按自己的带宽预算调度发包
 *
 * @param send 发送数据首地址
 * @param tx_len 发送长度
 */
void RefereeSendRaw(uint8_t *send, uint16_t tx_len);

/**
 * @brief 获取裁判接收链路诊断信息
 *
 * @return const RefereeRxDiag_s* 诊断信息指针
 */
const RefereeRxDiag_s *RefereeGetRxDiag(void);

#endif // !REFEREE_H
