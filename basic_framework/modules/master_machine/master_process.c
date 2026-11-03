/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "rm_referee.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

uint8_t VisionIsOnline(void)
{
    return DaemonIsOnline(vision_daemon_instance);
}

/**
 * @brief 解析上位机发来的 16 位 flag_register
 *        低8位: bit0-1 fire_mode, bit2-3 target_state, bit4-7 posture_cmd(1~6)
 *        高8位: target_type
 */
static void ResolveVisionFlag(uint16_t flag_register)
{
    recv_data.fire_mode    = (Fire_Mode_e)(flag_register & 0x03);
    recv_data.target_state = (Target_State_e)((flag_register >> 2) & 0x03);
    recv_data.posture_cmd  = (uint8_t)((flag_register >> 4) & 0x0F);
    recv_data.target_type  = (Target_Type_e)((flag_register >> 8) & 0xFF);
}

/**
 * @brief 将2027裁判系统关键状态打包进 VisionSend 的 flag_register 和 float 数组
 * @return 组装好的 16 位 flag_register:
 *         bit 0-1: 敌方颜色 (0未知, 1蓝, 2红)
 *         bit 2-5: 比赛阶段 game_progress (4为正式比赛中)
 *         bit 6:   哨兵是否处于脱战状态 (0x020D)
 *         bit 7:   是否踩在己方补给区 RFID (0x0209 bit0)
 *         bit 8-10: RMUL 고地/中央控制区 RFID 状态 (0x0209 bit11-13)
 *         bit 11-12: RMUL 中央控制区归属 (0中立, 1己方, 2对方, 来自0x0101 event_data_2 bit26-27)
 *         bit 13-15: 哨兵当前实际姿态 (1进攻, 2防御, 3移动, 来自0x020D)
 */
static uint16_t PackRefereeToVision(void)
{
    referee_info_t *ref = GetRefereeData();
    uint16_t flag = 0;

    // 1. 根据本机ID判断敌方颜色 (本机1~11红方则打蓝=1; 本机>100蓝方则打红=2)
    uint8_t my_id = ref->GameRobotState.robot_id;
    uint8_t enemy_color = (my_id == 0) ? COLOR_NONE : ((my_id > 100) ? COLOR_RED : COLOR_BLUE);

    uint8_t progress     = ref->GameState.game_progress & 0x0F;
    uint8_t out_combat   = (ref->SentryInfo.sentry_info_2 >> 0) & 0x01;
    uint8_t rfid_supply  = (ref->RFIDdata.rfid_status >> 0) & 0x01;
    uint8_t rfid_zones   = (ref->RFIDdata.rfid_status >> 11) & 0x07;
    uint8_t center_owner = (ref->EventData.event_data_2 >> 26) & 0x03;
    uint8_t cur_posture  = (ref->SentryInfo.sentry_info_2 >> 12) & 0x03;

    flag |= (enemy_color & 0x03) << 0;
    flag |= (progress    & 0x0F) << 2;
    flag |= (out_combat  & 0x01) << 6;
    flag |= (rfid_supply & 0x01) << 7;
    flag |= (rfid_zones  & 0x07) << 8;
    flag |= (center_owner & 0x03) << 11;
    flag |= (cur_posture & 0x07) << 13;

    // 2. 填充连续 float 数组
    send_data.current_hp   = (float)ref->GameRobotState.current_HP;
    send_data.remain_ammo  = (float)ref->AllowanceData.projectile_allowance_17mm;
    send_data.map_target_x = ref->MapData.target_position_x;
    send_data.map_target_y = ref->MapData.target_position_y;

    return flag;
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance;

/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision()
{
    uint16_t flag_register;
    if (get_protocol_info(vision_usart_instance->recv_buff, &flag_register, (uint8_t *)&recv_data.pitch) != 0)
    {
        DaemonReload(vision_daemon_instance); // 仅当CRC校验通过时喂狗
        ResolveVisionFlag(flag_register);
    }
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = PackRefereeToVision();
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 7, send_buff, &tx_len);
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); // 和视觉通信使用IT,防止和接收使用的DMA冲突
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    UNUSED(recv_len);
    uint16_t flag_register;
    if (get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.pitch) != 0)
    {
        DaemonReload(vision_daemon_instance); // 修复: USB收到合法数据包后喂狗
        ResolveVisionFlag(flag_register);
    }
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = PackRefereeToVision();
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 7, send_buff, &tx_len);
    USBTransmit(send_buff, tx_len);
}

#endif // VISION_USE_VCP
