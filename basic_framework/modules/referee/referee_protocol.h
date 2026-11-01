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
#define SELF_CTRL_DATA_LEN 30 //自定义控制器数据长度
#define CLIENT_DATA_LEN 300 //自定义客户端数据长度
#define CLIENT_DATA_RX_LEN 60 //自定义客户端数据长度

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
typedef enum
{
	ID_game_state = 0x0001,				   // 比赛状态数据
	ID_game_result = 0x0002,			   // 比赛结果数据
	ID_game_robot_survivors = 0x0003,	   // 比赛机器人血量数据
	ID_event_data = 0x0101,				   // 场地事件数据
	ID_supply_projectile_booking = 0x0103, // 场地补给站预约子弹数据
	ID_referee_warning_t = 0x0104,		   // 裁判警告信息
	ID_dart_info_t = 0x0105,			   // 场地飞镖发射数据
	ID_game_robot_state = 0x0201,		   // 机器人状态数据
	ID_power_heat_data = 0x0202,		   // 实时功率热量数据
	ID_game_robot_pos = 0x0203,			   // 机器人位置数据
	ID_buff_musk = 0x0204,				   // 机器人增益数据
	ID_robot_hurt = 0x0206,				   // 伤害状态数据
	ID_shoot_data = 0x0207,				   // 实时射击数据
	ID_projectile_allowance_t = 0x0208,	   // 弹仓数据
	ID_rfid_status_t = 0x0209,			   // RFID状态数据
	ID_ground_robot_pos = 0x020B,		   // 地面机器人位置数据(哨兵专用)
	ID_sentry_info = 0x020D,			   // 哨兵自主决策信息同步(哨兵专用)
	ID_student_interactive = 0x0301,	   // 机器人间交互数据
	ID_custom2robot = 0x0302,                 // 自定义控制器数据
	ID_map_interact_data = 0x0303,          //小地图交互数据
	ID_remote_control = 0x0304,            // 图传链路键鼠数据
	ID_radar2server = 0x0305,              // 雷达链路数据
	ID_customl2player = 0x0306,           // 自定义控制器向操作端发送数据
	ID_map_sentry_path = 0x0307,           // 选手端小地图接收哨兵路径数据(哨兵专用)
	ID_custom_info = 0x0308,               // 选手端小地图接收机器人自定义消息
	ID_robot2custom = 0x0309,            // 机器人向自定义控制器发送数据
	ID_robot2selfclient = 0x0310,          // 机器人向自定义客户端发送数据
	ID_selfclient2robot = 0x0311          // 自定义客户端向机器人发送数据
} CmdID_e;

/* 命令码数据段长,根据官方协议来定义长度，还有自定义数据长度 */
typedef enum
{
	LEN_game_state = 11,						 // 0x0001
	LEN_game_result = 1,						 // 0x0002
	LEN_game_robot_HP = 24,						 // 0x0003, 2027更改为24字节(新增双方控制度)
	LEN_event_data = 8,							 // 0x0101, 2027更改为8字节(新增RMUL控制区状态)
	LEN_referee_warning_t = 3,					 // 0x0104
	LEN_dart_info_t = 3,						 // 0x0105
	LEN_game_robot_state = 17,					 // 0x0201
	LEN_power_heat_data = 14,					 // 0x0202
	LEN_game_robot_pos = 12,					 // 0x0203, 2027更改为12字节(去掉z轴)
	LEN_buff_musk = 7,							 // 0x0204, 2027更改为7字节
	LEN_robot_hurt = 7,							 // 0x0206, 2027更改为7字节(新增命中序号和坐标)
	LEN_shoot_data = 7,							 // 0x0207
	LEN_projectile_allowance_t = 8,				 // 0x0208
	LEN_rfid_status_t = 6,						 // 0x0209, 2027更改为6字节
	LEN_ground_robot_pos = 40,					 // 0x020B, 哨兵接收己方机器人位置(40字节)
	LEN_sentry_info = 14,						 // 0x020D, 2027更改为14字节(新增姿态剩余时长)
	LEN_receive_data = 6 + Communicate_Data_LEN, // 0x0301
	LEN_map_interact_data = 12,                  // 0x0303, 2027明确为12字节
	LEN_radar2server_data = 48,					 // 0x0305
	LEN_selfctrl2player_data = 8,				 // 0x0306
	LEN_map_sentry_path = 105,					 // 0x0307, 哨兵小地图路径数据(105字节)
	LEN_custom_info = 34,						 // 0x0308, 选手端自定义消息(34字节)
	/*以上为常规链路*/
	LEN_custom2robot_data = 30, 				 // 0x0302
	LEN_remote_control_data = 12,   			 // 0x0304
	LEN_robot2custom_data = 30,					 // 0x0309
	LEN_robot2selfclient_data = 300,			 // 0x0310 
	LEN_selfclient2robot = 60					 // 0x0311, 2027更改为60字节
} JudgeDataLength_e;

/****************************接收数据的详细说明****************************/
/****************************接收数据的详细说明****************************/

/* ID: 0x0001  Byte:  11    比赛状态数据 */
typedef  struct 
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

/* ID: 0x0003  Byte:  24    比赛机器人血量数据 (2027更新) */
typedef struct
{
	uint16_t self_1_robot_HP;      // 己方1号重装机器人血量
	uint16_t reserved;             // 保留位
	uint16_t self_3_robot_HP;      // 己方3号步兵血量
	uint16_t self_4_robot_HP;      // 己方4号步兵血量
	int16_t diff_damage;           // 己方全队总伤害与对方全队总伤害之差
	uint16_t self_7_robot_HP;      // 己方7号哨兵血量
	uint16_t outpost_R_HP;         // R前哨血量
	uint16_t self_base_HP;         // 己方基地血量
	uint16_t outpost_B_HP;         // B前哨血量
	uint16_t opponent_base_HP;     // 对方基地血量
	uint16_t ally_control_degree;  // 2027新增: 己方累计控制度(3V3核心)
	uint16_t enemy_control_degree; // 2027新增: 对方累计控制度(3V3核心)
} ext_game_robot_HP_t;

/* ID: 0x0101  Byte:  8    场地事件数据 (2027更新) */
typedef struct
{
	uint32_t event_data;   // bit0:己方补给区激活状态
	uint32_t event_data_2; // 2027新增: bit16-19:R高地控制区, bit20-23:B高地控制区, bit24-27:中央控制区(仅RMUL适用)
} ext_event_data_t;

/* ID: 0x0104  Byte:  3    裁判警告数据 */
typedef struct 
{ 
  uint8_t level; 
  uint8_t offending_robot_id; 
  uint8_t count; 
}ext_referee_warning_t;

/* ID: 0x0105  Byte:  3    场地飞镖发射数据 */
typedef struct
{
 uint8_t dart_remaining_time;
 uint16_t dart_info;
}ext_dart_info_t; 

/* ID: 0X0201  Byte: 17    机器人状态数据 */
typedef struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	float shooter_speed_limit;
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1; 
} ext_game_robot_state_t;

/* ID: 0X0202  Byte: 14    实时功率热量数据 */
typedef struct
{
	uint16_t reserved1; 
	uint16_t reserved2;
	uint32_t reserved3;
	uint16_t buffer_energy;
	uint16_t shooter_17mm_barrel_heat;
	uint16_t shooter_42mm_barrel_heat; 
} ext_power_heat_data_t;

/* ID: 0x0203  Byte: 12    机器人位置数据 (2027更新) */
typedef struct
{
	float x;
	float y;
	float angle; // 本机器人测速模块的朝向,单位:度,正北为0度
} ext_game_robot_pos_t;

/* ID: 0x0204  Byte:  7    机器人增益数据 (2027更新) */
typedef struct
{
	uint8_t recovery_buff;      // 回血增益百分比
	uint16_t cooling_buff;      // 热量冷却增益具体值
	uint8_t defence_buff;       // 防御增益百分比
	uint8_t vulnerability_buff; // 负防御增益百分比
	uint16_t attack_buff;       // 攻击增益百分比
} ext_buff_musk_t;

/* ID: 0x0206  Byte:  7    伤害状态数据 (2027更新) */
typedef struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;  // 0:弹丸攻击, 1:模块离线, 2:超电离线, 5:撞击
	uint16_t hit_sequence;  // 命中事件序号,每发递增1
	int16_t hit_position_x; // 命中位置x坐标(mm)
	int16_t hit_position_y; // 命中位置y坐标(mm)
} ext_robot_hurt_t;

/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef struct
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
} ext_shoot_data_t;

/* ID：0x0208  Byte:  8    机器人允许发弹量数据 */
typedef struct
{
	uint16_t projectile_allowance_17mm;
	uint16_t projectile_allowance_42mm;
	uint16_t remaining_gold_coin;
	uint16_t projectile_allowance_fortress;	//储备17mm弹丸允许发弹量
} ext_projectile_allowance_t;

/* ID: 0x0209  Byte:  6    RFID状态数据 (2027更新) */
typedef struct
{
	uint32_t rfid_status;   // bit0:补给区, bit11:R高地(RMUL), bit12:B高地(RMUL), bit13:中央控制区(RMUL)
	uint16_t rfid_status_2; // 2027改为uint16_t
} ext_rfid_status_t;

// /* ID：0x020A  Byte:  6    飞镖发射站状态 */
// typedef struct
// {
// 	uint8_t dart_launch_opening_status;
// 	uint8_t reserved;
// 	uint16_t target_change_time;
// 	uint16_t latest_launch_cmd_time;
// } ext_dart_client_cmd_t;

/* ID: 0x020B  Byte:  40   地面机器人位置数据 (哨兵专用,取消注释并更新) */
typedef struct
{
	float heavyengineer_x; // 己方重装机器人x坐标(m)
	float heavyengineer_y; // 己方重装机器人y坐标(m)
	float reserved1;
	float reserved2;
	float standard_3_x;    // 己方3号步兵x坐标(m)
	float standard_3_y;    // 己方3号步兵y坐标(m)
	float standard_4_x;    // 己方4号步兵x坐标(m)
	float standard_4_y;    // 己方4号步兵y坐标(m)
	float reserved3;
	float reserved4;
} ext_ground_robot_position_t;

// /* ID：0x020C  Byte:  2    雷达标识 */
// typedef struct
// {
// 	uint16_t mark_progress;
// } ext_radar_mark_data_t;

/* ID: 0x020D  Byte:  14   哨兵自主决策信息同步 (哨兵专用,取消注释并更新为14字节) */
typedef struct
{
	uint32_t sentry_info;   // bit0-10:已兑弹量, bit11-14:远程补弹次数, bit15-18:远程回血次数, bit19:可免费复活, bit20:可买活, bit21-30:买活金币
	uint16_t sentry_info_2; // bit0:脱战状态, bit1-11:剩余可兑弹数, bit12-13:当前姿态(1进攻/2防御/3移动), bit15:是否强化姿态
	uint64_t sentry_info_3; // 2027新增: 各姿态弱化前剩余可持续时长 & 强化姿态剩余可持续时长(单位:秒)
} ext_sentry_info_t;

// /* ID：0x020E  Byte:  1    雷达状态*/
// typedef struct
// {
// 	uint8_t radar_info;
// } ext_radar_info_t;

/* ID: 0x0303  Byte:  12   选手端小地图交互数据 (2027更新为12字节) */
typedef struct 
{ 
	float target_position_x; 
	float target_position_y; 
	uint8_t cmd_keyboard; 
	uint8_t target_robot_id; 
	uint16_t cmd_source; 
} ext_map_command_t;

/****************************机器人交互数据****************************/
/****************************机器人交互数据****************************/
/* 发送的内容数据段最大为 113 检测是否超出大小限制?实际上图形段不会超，数据段最多30个，也不会超*/
/* 交互数据头结构 */
typedef struct
{
	uint16_t data_cmd_id; // 由于存在多个内容 ID，但整个cmd_id 上行频率最大为 30Hz，请合理安排带宽。注意交互部分的上行频率
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
	Robot_Custom_Data_LEN = 30, //机器人发给自定义控制器数据，长度30

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

/****************************哨兵自主决策指令(0x0301 -> 0x0120)****************************/
#define REFEREE_SERVER_ID 0x8080  // 裁判系统服务器ID
#define Sentry_Decision_ID 0x0120 // 哨兵自主决策子内容ID
#define Sentry_Decision_LEN 4     // 哨兵自主决策指令数据段长度

typedef struct
{
	uint32_t confirm_respawn : 1;      // bit 0: 是否确认复活(0不复活, 1确认复活)
	uint32_t exchange_respawn : 1;     // bit 1: 是否确认兑换立即复活(0不兑换, 1兑换)
	uint32_t exchange_ammo_val : 11;   // bit 2-12: 将要兑换的发弹量累计值(开局为0,单调递增)
	uint32_t remote_ammo_req_cnt : 4;  // bit 13-16: 远程兑换发弹量请求次数(开局为0,单调递增每次+1)
	uint32_t remote_hp_req_cnt : 4;    // bit 17-20: 远程兑换血量请求次数(开局为0,单调递增每次+1)
	uint32_t posture_cmd : 3;          // bit 21-23: 修改当前姿态(1进攻,2防御,3移动,4强化进攻,5强化防御,6强化移动,默认3)
	uint32_t confirm_rune : 1;         // bit 24: 确认能量机关进入正在激活状态(3V3填0)
	uint32_t reserved : 7;             // bit 25-31: 保留位
} sentry_cmd_t;

typedef struct
{
	xFrameHeader FrameHeader;
	uint16_t CmdID;
	ext_student_interactive_header_data_t datahead;
	sentry_cmd_t sentry_cmd;
	uint16_t frametail;
} Sentry_Decision_SendData_t;

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

/****************************图传链路数据****************************/
/*cmd_id:0x0302 */
typedef struct
{
	uint8_t data[SELF_CTRL_DATA_LEN]; //长度固定，26赛季为30，视情况更改
} custom2robot_data_t; //自定义控制器发往机器人的数据

/*cmd_id:0x0309 */
typedef struct
{
	uint8_t data[SELF_CTRL_DATA_LEN]; //长度固定，26赛季为30，视情况更改
} robot2custom_data_t; //机器人发往自定义控制器的数据

/*cmd_id:0x0310 */
typedef struct
{
	uint8_t data[CLIENT_DATA_LEN]; //长度固定，26赛季为300，视情况更改
} robot2client_data_t; //机器人发往自定义客户端的数据

/*cmd_id:0x0311 */
typedef struct
{
	uint8_t data[CLIENT_DATA_RX_LEN]; //长度固定，26赛季为300，视情况更改
} client2robot_data_t; //自定义客户端发往机器人的数据

#pragma pack()

#endif // referee_protocol_H
