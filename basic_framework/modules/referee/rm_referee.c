/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon;		  // 裁判系统守护进程
static referee_info_t referee_info;			  // 裁判系统数据

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
static void JudgeReadData(uint8_t *buff)
{
	uint16_t judge_length; // 统计一帧数据长度
	if (buff == NULL)	   // 空数据包，则不作任何处理
		return;

	// 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
	memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);

	// 判断帧头数据(0)是否为0xA5
	if (buff[SOF] == REFEREE_SOF)
	{
		// 帧头CRC8校验
		if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
		{
			// 统计一帧数据长度(byte),使用16位DataLength防止截断,并限制不超过接收缓冲区大小
			judge_length = referee_info.FrameHeader.DataLength + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			if (judge_length > RE_RX_BUFFER_SIZE)
				return;

			// 帧尾CRC16校验
			if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
			{
				// 2个8位拼成16位int
				referee_info.CmdID = (buff[6] << 8 | buff[5]);
				// 解析数据命令码,将数据拷贝到相应结构体中
				switch (referee_info.CmdID)
				{
				case ID_game_state: // 0x0001
					memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_game_state);
					break;
				case ID_game_result: // 0x0002
					memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_game_result);
					break;
				case ID_game_robot_survivors: // 0x0003
					memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_game_robot_HP);
					break;
				case ID_event_data: // 0x0101
					memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_event_data);
					break;
				case ID_referee_warning_t: // 0x0104
					memcpy(&referee_info.WarnData, (buff + DATA_Offset), LEN_referee_warning_t);
					break;
				case ID_dart_info_t: // 0x0105
					memcpy(&referee_info.DartData, (buff + DATA_Offset), LEN_dart_info_t);
					break;
				case ID_game_robot_state: // 0x0201
					memcpy(&referee_info.GameRobotState, (buff + DATA_Offset), LEN_game_robot_state);
					break;
				case ID_power_heat_data: // 0x0202
					memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);
					break;
				case ID_game_robot_pos: // 0x0203
					memcpy(&referee_info.GameRobotPos, (buff + DATA_Offset), LEN_game_robot_pos);
					break;
				case ID_buff_musk: // 0x0204
					memcpy(&referee_info.BuffMusk, (buff + DATA_Offset), LEN_buff_musk);
					break;
				case ID_robot_hurt: // 0x0206
					memcpy(&referee_info.RobotHurt, (buff + DATA_Offset), LEN_robot_hurt);
					break;
				case ID_shoot_data: // 0x0207
					memcpy(&referee_info.ShootData, (buff + DATA_Offset), LEN_shoot_data);
					break;
				case ID_projectile_allowance_t: // 0x0208
					memcpy(&referee_info.AllowanceData, (buff + DATA_Offset), LEN_projectile_allowance_t);
					break;
				case ID_rfid_status_t: // 0x0209
					memcpy(&referee_info.RFIDdata, (buff + DATA_Offset), LEN_rfid_status_t);
					break;
				case ID_ground_robot_pos: // 0x020B 哨兵接收己方地面机器人坐标
					memcpy(&referee_info.GroundRobotPos, (buff + DATA_Offset), LEN_ground_robot_pos);
					break;
				case ID_sentry_info: // 0x020D 哨兵自主决策信息同步
					memcpy(&referee_info.SentryInfo, (buff + DATA_Offset), LEN_sentry_info);
					break;
				case ID_student_interactive: // 0x0301
					memcpy(&referee_info.ReceiveData, (buff + DATA_Offset), LEN_receive_data);
					break;
				case ID_map_interact_data: // 0x0303
					memcpy(&referee_info.MapData, (buff + DATA_Offset), LEN_map_interact_data);
					break;
				}

				// 仅在当前包CRC校验通过且未超出接收缓冲区边界时，才继续解析粘包的下一帧
				uint8_t *next_frame = buff + judge_length;
				if ((next_frame - referee_usart_instance->recv_buff) < (RE_RX_BUFFER_SIZE - LEN_HEADER) &&
					*next_frame == REFEREE_SOF)
				{
					JudgeReadData(next_frame);
				}
			}
		}
	}
}

/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback()
{
	DaemonReload(referee_daemon);
	JudgeReadData(referee_usart_instance->recv_buff);
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
	USARTServiceInit(referee_usart_instance);
	LOGWARNING("[rm_ref] lost referee data");
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = RefereeRxCallback;
	conf.usart_handle = referee_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	referee_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = RefereeLostCallback,
		.owner_id = referee_usart_instance,
		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	referee_daemon = DaemonRegister(&daemon_conf);

	return &referee_info;
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	osDelay(34);
}

/**
 * @brief 发送0x0120哨兵自主决策指令(通过0x0301发给服务器0x8080)
 * @param cmd 哨兵决策指令位域结构体
 */
void RefereeSendSentryCmd(sentry_cmd_t cmd)
{
	static Sentry_Decision_SendData_t tx_pack;
	uint16_t data_len = Interactive_Data_LEN_Head + Sentry_Decision_LEN; // 6 + 4 = 10字节

	if (referee_info.GameRobotState.robot_id == 0)
		return; // 尚未收到自身ID时不发送

	tx_pack.FrameHeader.SOF = REFEREE_SOF;
	tx_pack.FrameHeader.DataLength = data_len;
	tx_pack.FrameHeader.Seq = UI_Seq++;
	tx_pack.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&tx_pack.FrameHeader, LEN_CRC8, 0xFF);

	tx_pack.CmdID = ID_student_interactive; // 0x0301

	tx_pack.datahead.data_cmd_id = Sentry_Decision_ID; // 0x0120
	tx_pack.datahead.sender_ID = referee_info.GameRobotState.robot_id; // 7(红哨兵) 或 107(蓝哨兵)
	tx_pack.datahead.receiver_ID = REFEREE_SERVER_ID;  // 0x8080 裁判系统服务器

	tx_pack.sentry_cmd = cmd;

	tx_pack.frametail = Get_CRC16_Check_Sum((uint8_t *)&tx_pack, LEN_HEADER + LEN_CMDID + data_len, 0xFFFF);

	RefereeSend((uint8_t *)&tx_pack, sizeof(Sentry_Decision_SendData_t));
}

referee_info_t *GetRefereeData(void)
{
	return &referee_info;
}
