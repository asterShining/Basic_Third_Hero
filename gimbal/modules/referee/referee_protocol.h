/**
 * @file referee_protocol.h
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */

#ifndef referee_protocol_H
#define referee_protocol_H

#include "stdint.h"

/****************************宏定义部分****************************/

#define REFEREE_SOF 0xA5 // 起始字节,协议固定为0xA5
#define Robot_Red 0
#define Robot_Blue 1
#define Communicate_Data_LEN 5 // 自定义交互数据长度，该长度决定了我方发送和他方接收，自定义交互数据协议更改时只需要更改此宏定义即可

#pragma pack(1)

/****************************通信协议格式****************************/

/* 通信协议格式偏移，枚举类型,代替#define声明 */
typedef enum
{
	FRAME_HEADER_Offset = 0,
	CMD_ID_Offset = 5,
	DATA_Offset = 7,
} JudgeFrameOffset_e;

/* 通信协议长度 */
typedef enum
{
	LEN_HEADER = 5, // 帧头长
	LEN_CMDID = 2,	// 命令码长度
	LEN_TAIL = 2,	// 帧尾CRC16

	LEN_CRC8 = 4, // 帧头CRC8校验长度=帧头+数据长+包序号
} JudgeFrameLength_e;

/****************************帧头****************************/
/****************************帧头****************************/

/* 帧头偏移 */
typedef enum
{
	SOF = 0,		 // 起始位
	DATA_LENGTH = 1, // 帧内数据长度,根据这个来获取数据长度
	SEQ = 3,		 // 包序号
	CRC8 = 4		 // CRC8
} FrameHeaderOffset_e;

/* 帧头定义 */
typedef struct
{
	uint8_t SOF;
	uint16_t DataLength;
	uint8_t Seq;
	uint8_t CRC8;
} xFrameHeader;

/****************************cmd_id命令码说明****************************/
/****************************cmd_id命令码说明****************************/

/* 命令码ID,用来判断接收的是什么数据 */
/* 这里严格按 2026 最新通信协议保留命令集合，作用是把云台板协议层和底盘板统一到同一份命令表；
 * 原因是仅同步局部命令会让“链路正常但协议版本不一致”的问题继续隐蔽存在。 */
typedef enum
{
	ID_game_state = 0x0001,				   // 比赛状态数据
	ID_game_result = 0x0002,			   // 比赛结果数据
	ID_game_robot_survivors = 0x0003,	   // 比赛机器人血量数据
	ID_event_data = 0x0101,				   // 场地事件数据
	ID_referee_warning = 0x0104,		   // 裁判警告数据
	ID_dart_info = 0x0105,				   // 飞镖发射相关数据
	ID_game_robot_state = 0x0201,		   // 机器人状态数据
	ID_power_heat_data = 0x0202,		   // 实时功率热量数据
	ID_game_robot_pos = 0x0203,			   // 机器人位置数据
	ID_buff_musk = 0x0204,				   // 机器人增益数据
	ID_robot_hurt = 0x0206,				   // 伤害状态数据
	ID_shoot_data = 0x0207,				   // 实时射击数据
	ID_projectile_allowance = 0x0208,	   // 允许发弹量
	ID_rfid_status = 0x0209,			   // RFID状态
	ID_dart_client_cmd = 0x020A,		   // 飞镖选手端指令
	ID_ground_robot_position = 0x020B,	   // 地面机器人位置
	ID_radar_mark_data = 0x020C,		   // 雷达标记进度
	ID_sentry_info = 0x020D,			   // 哨兵决策信息
	ID_radar_info = 0x020E,				   // 雷达决策信息
	ID_student_interactive = 0x0301,	   // 机器人间交互数据
	ID_custom_robot_data = 0x0302,		   // 自定义控制器与机器人交互
	ID_map_command = 0x0303,			   // 小地图交互
	ID_map_robot_data = 0x0305,			   // 小地图接收雷达数据
	ID_custom_client_data = 0x0306,		   // 自定义控制器与选手端交互
	ID_map_data = 0x0307,				   // 小地图路径数据
	ID_custom_info = 0x0308,			   // 小地图自定义消息
	ID_robot_custom_data = 0x0309,		   // 自定义控制器接收机器人数据
	ID_robot_custom_data_2 = 0x0310,	   // 机器人发送给自定义客户端
	ID_robot_custom_data_3 = 0x0311,	   // 自定义客户端发送给机器人
} CmdID_e;

/* 命令码数据段长,根据通信协议.md定义 */
/* 这里把每个数据段长度显式固定成最新版协议值，作用是让解析层能基于 DataLength 做严格校验；
 * 原因是如果长度继续沿用旧协议，云台板即便收到合法新帧也会被误判成坏包。 */
typedef enum
{
	LEN_game_state = 11,						 // 0x0001
	LEN_game_result = 1,						 // 0x0002
	LEN_game_robot_HP = 16,						 // 0x0003
	LEN_event_data = 4,							 // 0x0101
	LEN_referee_warning = 3,					 // 0x0104
	LEN_dart_info = 3,							 // 0x0105
	LEN_game_robot_state = 13,					 // 0x0201
	LEN_power_heat_data = 14,					 // 0x0202
	LEN_game_robot_pos = 12,					 // 0x0203
	LEN_buff_musk = 8,							 // 0x0204
	LEN_robot_hurt = 1,							 // 0x0206
	LEN_shoot_data = 7,							 // 0x0207
	LEN_projectile_allowance = 8,				 // 0x0208
	LEN_rfid_status = 5,						 // 0x0209
	LEN_dart_client_cmd = 6,					 // 0x020A
	LEN_ground_robot_position = 40,			 // 0x020B
	LEN_radar_mark_data = 2,					 // 0x020C
	LEN_sentry_info = 6,						 // 0x020D
	LEN_radar_info = 1,							 // 0x020E
	LEN_robot_interaction_data = 118,			 // 0x0301(6字节头+112字节用户数据)
	LEN_receive_data = 6 + Communicate_Data_LEN, // 0x0301(本工程自定义子内容长度)
	LEN_custom_robot_data = 30,					 // 0x0302
	LEN_map_command = 12,						 // 0x0303
	LEN_map_robot_data = 24,					 // 0x0305
	LEN_custom_client_data = 6,					 // 0x0306
	LEN_map_data = 105,							 // 0x0307
	LEN_custom_info = 34,						 // 0x0308
	LEN_robot_custom_data = 30,					 // 0x0309
	LEN_robot_custom_data_2 = 300,				 // 0x0310
	LEN_robot_custom_data_3 = 30,				 // 0x0311

} JudgeDataLength_e;

/****************************接收数据的详细说明****************************/
/****************************接收数据的详细说明****************************/

/* ID: 0x0001  Byte:  11    比赛状态数据 */
typedef struct
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_state_t;

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef struct
{
	uint8_t winner;
} ext_game_result_t;

/* ID: 0x0003  Byte:  16    比赛机器人血量数据 */
typedef struct
{
	uint16_t ally_1_robot_HP;
	uint16_t ally_2_robot_HP;
	uint16_t ally_3_robot_HP;
	uint16_t ally_4_robot_HP;
	uint16_t reserved;
	uint16_t ally_7_robot_HP;
	uint16_t ally_outpost_HP;
	uint16_t ally_base_HP;
} ext_game_robot_HP_t;

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef struct
{
	uint32_t event_data;
} ext_event_data_t;

/* ID: 0X0201  Byte: 13    机器人状态数据 */
typedef struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
} ext_game_robot_state_t;

/* ID: 0X0202  Byte: 14    实时功率热量数据 */
typedef struct
{
	uint16_t reserved1;
	uint16_t reserved2;
	float reserved3;
	uint16_t buffer_energy;
	uint16_t shooter_17mm_barrel_heat;
	uint16_t shooter_42mm_barrel_heat;
} ext_power_heat_data_t;

/* ID: 0x0203  Byte: 12    机器人位置数据 */
typedef struct
{
	float x;
	float y;
	float angle;
} ext_game_robot_pos_t;

/* ID: 0x0204  Byte:  8    机器人增益数据 */
typedef struct
{
	uint8_t recovery_buff;
	uint16_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
	uint8_t remaining_energy;
} ext_buff_musk_t;

/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef struct
{
	uint8_t bullet_type;
	uint8_t shooter_number;
	uint8_t launching_frequency;
	float initial_speed;
} ext_shoot_data_t;

/* ID: 0x0104  Byte: 3    裁判警告数据(v2协议) */
typedef struct
{
	uint8_t level;
	uint8_t offending_robot_id;
	uint8_t count;
} ext_referee_warning_t;

/* ID: 0x0105  Byte: 3    飞镖发射相关数据(v2协议) */
typedef struct
{
	uint8_t dart_remaining_time;
	uint16_t dart_info;
} ext_dart_info_t;

/* ID: 0x0208  Byte: 8    允许发弹量(v2协议) */
typedef struct
{
	uint16_t projectile_allowance_17mm;
	uint16_t projectile_allowance_42mm;
	uint16_t remaining_gold_coin;
	uint16_t projectile_allowance_fortress;
} ext_projectile_allowance_t;

/* ID: 0x0209  Byte: 5    RFID状态(v2协议) */
typedef struct
{
	uint32_t rfid_status;
	uint8_t rfid_status_2;
} ext_rfid_status_t;

/* ID: 0x020A  Byte: 6    飞镖选手端指令(v2协议) */
typedef struct
{
	uint8_t dart_launch_opening_status;
	uint8_t reserved;
	uint16_t target_change_time;
	uint16_t latest_launch_cmd_time;
} ext_dart_client_cmd_t;

/* ID: 0x020B  Byte: 40    地面机器人位置(v2协议) */
typedef struct
{
	float hero_x;
	float hero_y;
	float engineer_x;
	float engineer_y;
	float standard_3_x;
	float standard_3_y;
	float standard_4_x;
	float standard_4_y;
	float reserved;
	float reserved_2;
} ext_ground_robot_position_t;

/* ID: 0x020C  Byte: 2    雷达标记进度(v2协议) */
typedef struct
{
	uint16_t mark_progress;
} ext_radar_mark_data_t;

/* ID: 0x020D  Byte: 6    哨兵决策信息(v2协议) */
typedef struct
{
	uint32_t sentry_info;
	uint16_t sentry_info_2;
} ext_sentry_info_t;

/* ID: 0x020E  Byte: 1    雷达决策信息(v2协议) */
typedef struct
{
	uint8_t radar_info;
} ext_radar_info_t;

/* ID: 0x0301  Byte: 118    机器人间交互数据(最大长度) */
typedef struct
{
	uint16_t data_cmd_id;
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[112];
} ext_robot_interaction_data_t;

/* ID: 0x0302  Byte: 30    自定义控制器与机器人交互(v2协议) */
typedef struct
{
	uint8_t data[30];
} ext_custom_robot_data_t;

/* ID: 0x0303  Byte: 12    小地图交互(v2协议) */
typedef struct
{
	float target_position_x;
	float target_position_y;
	uint8_t cmd_keyboard;
	uint8_t target_robot_id;
	uint16_t cmd_source;
} ext_map_command_t;

/* ID: 0x0305  Byte: 24    小地图接收雷达数据(v2协议) */
typedef struct
{
	uint16_t hero_position_x;
	uint16_t hero_position_y;
	uint16_t engineer_position_x;
	uint16_t engineer_position_y;
	uint16_t infantry_3_position_x;
	uint16_t infantry_3_position_y;
	uint16_t infantry_4_position_x;
	uint16_t infantry_4_position_y;
	uint16_t reserved;
	uint16_t reserved_2;
	uint16_t sentry_position_x;
	uint16_t sentry_position_y;
} ext_map_robot_data_t;

/* ID: 0x0306  Byte: 6    自定义控制器与选手端交互(v2协议) */
typedef struct
{
	uint16_t key_value;
	uint16_t x_position : 12;
	uint16_t mouse_left : 4;
	uint16_t y_position : 12;
	uint16_t mouse_right : 4;
	uint16_t reserved;
} ext_custom_client_data_t;

/* ID: 0x0307  Byte: 105    小地图路径数据(v2协议) */
typedef struct
{
	uint8_t intention;
	uint16_t start_position_x;
	uint16_t start_position_y;
	int8_t delta_x[49];
	int8_t delta_y[49];
	uint16_t sender_id;
} ext_map_data_t;

/* ID: 0x0308  Byte: 34    小地图自定义消息(v2协议) */
typedef struct
{
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[30];
} ext_custom_info_t;

/* ID: 0x0309  Byte: 30    自定义控制器接收机器人数据(v2协议) */
typedef struct
{
	uint8_t data[30];
} ext_robot_custom_data_t;

/* ID: 0x0310  Byte: 300    机器人发送给自定义客户端(v2协议) */
typedef struct
{
	uint8_t data[300];
} ext_robot_custom_data_2_t;

/* ID: 0x0311  Byte: 30    自定义客户端发送给机器人(v2协议) */
typedef struct
{
	uint8_t data[30];
} ext_robot_custom_data_3_t;

/* 编译期结构体尺寸校验：在编译阶段锁定协议结构长度，避免对齐变化导致运行时解析错位 */
_Static_assert(sizeof(ext_game_state_t) == LEN_game_state, "ext_game_state_t size mismatch");
_Static_assert(sizeof(ext_game_result_t) == LEN_game_result, "ext_game_result_t size mismatch");
_Static_assert(sizeof(ext_game_robot_HP_t) == LEN_game_robot_HP, "ext_game_robot_HP_t size mismatch");
_Static_assert(sizeof(ext_event_data_t) == LEN_event_data, "ext_event_data_t size mismatch");
_Static_assert(sizeof(ext_referee_warning_t) == LEN_referee_warning, "ext_referee_warning_t size mismatch");
_Static_assert(sizeof(ext_dart_info_t) == LEN_dart_info, "ext_dart_info_t size mismatch");
_Static_assert(sizeof(ext_game_robot_state_t) == LEN_game_robot_state, "ext_game_robot_state_t size mismatch");
_Static_assert(sizeof(ext_power_heat_data_t) == LEN_power_heat_data, "ext_power_heat_data_t size mismatch");
_Static_assert(sizeof(ext_game_robot_pos_t) == LEN_game_robot_pos, "ext_game_robot_pos_t size mismatch");
_Static_assert(sizeof(ext_buff_musk_t) == LEN_buff_musk, "ext_buff_musk_t size mismatch");
_Static_assert(sizeof(ext_robot_hurt_t) == LEN_robot_hurt, "ext_robot_hurt_t size mismatch");
_Static_assert(sizeof(ext_shoot_data_t) == LEN_shoot_data, "ext_shoot_data_t size mismatch");
_Static_assert(sizeof(ext_projectile_allowance_t) == LEN_projectile_allowance, "ext_projectile_allowance_t size mismatch");
_Static_assert(sizeof(ext_rfid_status_t) == LEN_rfid_status, "ext_rfid_status_t size mismatch");
_Static_assert(sizeof(ext_dart_client_cmd_t) == LEN_dart_client_cmd, "ext_dart_client_cmd_t size mismatch");
_Static_assert(sizeof(ext_ground_robot_position_t) == LEN_ground_robot_position, "ext_ground_robot_position_t size mismatch");
_Static_assert(sizeof(ext_radar_mark_data_t) == LEN_radar_mark_data, "ext_radar_mark_data_t size mismatch");
_Static_assert(sizeof(ext_sentry_info_t) == LEN_sentry_info, "ext_sentry_info_t size mismatch");
_Static_assert(sizeof(ext_radar_info_t) == LEN_radar_info, "ext_radar_info_t size mismatch");
_Static_assert(sizeof(ext_robot_interaction_data_t) == LEN_robot_interaction_data, "ext_robot_interaction_data_t size mismatch");
_Static_assert(sizeof(ext_custom_robot_data_t) == LEN_custom_robot_data, "ext_custom_robot_data_t size mismatch");
_Static_assert(sizeof(ext_map_command_t) == LEN_map_command, "ext_map_command_t size mismatch");
_Static_assert(sizeof(ext_map_robot_data_t) == LEN_map_robot_data, "ext_map_robot_data_t size mismatch");
/* 0x0306 使用位域，跨编译器位域布局可能存在实现差异，因此不做强制 sizeof 断言避免误报 */
_Static_assert(sizeof(ext_map_data_t) == LEN_map_data, "ext_map_data_t size mismatch");
_Static_assert(sizeof(ext_custom_info_t) == LEN_custom_info, "ext_custom_info_t size mismatch");
_Static_assert(sizeof(ext_robot_custom_data_t) == LEN_robot_custom_data, "ext_robot_custom_data_t size mismatch");
_Static_assert(sizeof(ext_robot_custom_data_2_t) == LEN_robot_custom_data_2, "ext_robot_custom_data_2_t size mismatch");
_Static_assert(sizeof(ext_robot_custom_data_3_t) == LEN_robot_custom_data_3, "ext_robot_custom_data_3_t size mismatch");

/****************************机器人交互数据****************************/
/****************************机器人交互数据****************************/
/* 发送的内容数据段最大为 113 检测是否超出大小限制?实际上图形段不会超，数据段最多30个，也不会超*/
/* 交互数据头结构 */
typedef struct
{
	uint16_t data_cmd_id; // 由于存在多个内容 ID，但整个cmd_id 上行频率最大为 10Hz，请合理安排带宽。注意交互部分的上行频率
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/* 机器人id */
typedef enum
{
	// 红方机器人ID
	RobotID_RHero = 1,
	RobotID_REngineer = 2,
	RobotID_RStandard1 = 3,
	RobotID_RStandard2 = 4,
	RobotID_RStandard3 = 5,
	RobotID_RAerial = 6,
	RobotID_RSentry = 7,
	RobotID_RRadar = 9,
	// 蓝方机器人ID
	RobotID_BHero = 101,
	RobotID_BEngineer = 102,
	RobotID_BStandard1 = 103,
	RobotID_BStandard2 = 104,
	RobotID_BStandard3 = 105,
	RobotID_BAerial = 106,
	RobotID_BSentry = 107,
	RobotID_BRadar = 109,
} Robot_ID_e;

/* 交互数据ID */
typedef enum
{
	UI_Data_ID_Del = 0x100,
	UI_Data_ID_Draw1 = 0x101,
	UI_Data_ID_Draw2 = 0x102,
	UI_Data_ID_Draw5 = 0x103,
	UI_Data_ID_Draw7 = 0x104,
	UI_Data_ID_DrawChar = 0x110,

	/* 自定义交互数据部分 */
	Communicate_Data_ID = 0x0200,

} Interactive_Data_ID_e;
/* 交互数据长度 */
typedef enum
{
	Interactive_Data_LEN_Head = 6,
	UI_Operate_LEN_Del = 2,
	UI_Operate_LEN_PerDraw = 15,
	UI_Operate_LEN_DrawChar = 15 + 30,

	/* 自定义交互数据部分 */
	// Communicate_Data_LEN = 5,

} Interactive_Data_Length_e;

/****************************自定义交互数据****************************/
/*
	学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	自定义交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz
*/
// 自定义交互数据协议，可更改，更改后需要修改最上方宏定义数据长度的值
typedef struct
{
	uint8_t data[Communicate_Data_LEN]; // 数据段,n需要小于113
} robot_interactive_data_t;

// 机器人交互信息_发送
typedef struct
{
	xFrameHeader FrameHeader;
	uint16_t CmdID;
	ext_student_interactive_header_data_t datahead;
	robot_interactive_data_t Data; // 数据段
	uint16_t frametail;
} Communicate_SendData_t;
// 机器人交互信息_接收
typedef struct
{
	ext_student_interactive_header_data_t datahead;
	robot_interactive_data_t Data; // 数据段
} Communicate_ReceiveData_t;

/* 编译期结构体尺寸校验：锁定本工程自定义0x0301接收长度，避免改动交互长度后出现静默错位 */
_Static_assert(sizeof(Communicate_ReceiveData_t) == LEN_receive_data, "Communicate_ReceiveData_t size mismatch");

/****************************UI交互数据****************************/

/* 图形数据 */
typedef struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye : 3;
	uint32_t graphic_tpye : 3;
	uint32_t layer : 4;
	uint32_t color : 4;
	uint32_t start_angle : 9;
	uint32_t end_angle : 9;
	uint32_t width : 10;
	uint32_t start_x : 11;
	uint32_t start_y : 11;
	uint32_t radius : 10;
	uint32_t end_x : 11;
	uint32_t end_y : 11;
} Graph_Data_t;

typedef struct
{
	Graph_Data_t Graph_Control;
	uint8_t show_Data[30];
} String_Data_t; // 打印字符串数据

/* 删除操作 */
typedef enum
{
	UI_Data_Del_NoOperate = 0,
	UI_Data_Del_Layer = 1,
	UI_Data_Del_ALL = 2, // 删除全部图层，后面的参数已经不重要了。
} UI_Delete_Operate_e;

/* 图形配置参数__图形操作 */
typedef enum
{
	UI_Graph_ADD = 1,
	UI_Graph_Change = 2,
	UI_Graph_Del = 3,
} UI_Graph_Operate_e;

/* 图形配置参数__图形类型 */
typedef enum
{
	UI_Graph_Line = 0,		// 直线
	UI_Graph_Rectangle = 1, // 矩形
	UI_Graph_Circle = 2,	// 整圆
	UI_Graph_Ellipse = 3,	// 椭圆
	UI_Graph_Arc = 4,		// 圆弧
	UI_Graph_Float = 5,		// 浮点型
	UI_Graph_Int = 6,		// 整形
	UI_Graph_Char = 7,		// 字符型

} UI_Graph_Type_e;

/* 图形配置参数__图形颜色 */
typedef enum
{
	UI_Color_Main = 0, // 红蓝主色
	UI_Color_Yellow = 1,
	UI_Color_Green = 2,
	UI_Color_Orange = 3,
	UI_Color_Purplish_red = 4, // 紫红色
	UI_Color_Pink = 5,
	UI_Color_Cyan = 6, // 青色
	UI_Color_Black = 7,
	UI_Color_White = 8,

} UI_Graph_Color_e;

#pragma pack()

#endif
