/**
 * @file referee_task.c
 * @brief 哨兵裁判系统上行决策任务 (2027 RMUL 3V3)
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"

static Referee_Interactive_info_t *Interactive_data;
static referee_info_t *referee_recv_info;
uint8_t UI_Seq; // 包序号，供整个referee文件使用

/* 哨兵自主决策指令缓存与请求标志 */
static sentry_cmd_t s_sentry_cmd = {.posture_cmd = 3}; // 默认3:移动姿态
static uint8_t s_target_posture = 3;                   // 外部期望姿态
static uint8_t s_req_remote_ammo = 0;                  // 远程补弹触发标志
static uint8_t s_req_remote_hp = 0;                    // 远程回血触发标志
static uint16_t s_req_supply_ammo_add = 0;             // 补给区新增兑弹量

static void DeterminRobotID(void)
{
    // 2027协议: 红方ID < 100 (红哨兵为7), 蓝方ID > 100 (蓝哨兵为107)
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 100 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID;
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle);
    Interactive_data = UI_data;
    referee_recv_info->init_flag = 1;
    return referee_recv_info;
}

void MyUIInit(void)
{
    if (!referee_recv_info->init_flag)
        vTaskDelete(NULL);

    // 持续等待裁判系统下发 0x0201 获取本机 robot_id (切勿超时删除任务,保证随时插线可用)
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100);

    DeterminRobotID();
}

/* ---------------- 外部调用的哨兵决策接口 ---------------- */
void SentrySetPosture(uint8_t posture)
{
    if (posture >= 1 && posture <= 6)
        s_target_posture = posture;
}

void SentryRequestConfirmRespawn(uint8_t enable)
{
    s_sentry_cmd.exchange_respawn = enable ? 1 : 0;
}

void SentryRequestSupplyAmmo(uint16_t add_ammo)
{
    s_req_supply_ammo_add = add_ammo;
}

void SentryRequestRemoteAmmo(void)
{
    s_req_remote_ammo = 1;
}

void SentryRequestRemoteHP(void)
{
    s_req_remote_hp = 1;
}

/* ---------------- 哨兵周期上行任务 (约5Hz) ---------------- */
void UITask(void)
{
    uint32_t info1 = referee_recv_info->SentryInfo.sentry_info;
    uint16_t info2 = referee_recv_info->SentryInfo.sentry_info_2;
    uint64_t info3 = referee_recv_info->SentryInfo.sentry_info_3;

    /* 1. 解析 0x020D 服务器已确认状态 */
    uint16_t acked_ammo_val = (info1 >> 0) & 0x7FF;  // bit 0-10: 已成功兑换发弹量
    uint8_t acked_ammo_cnt  = (info1 >> 11) & 0x0F;  // bit 11-14: 已成功远程补弹次数
    uint8_t acked_hp_cnt    = (info1 >> 15) & 0x0F;  // bit 15-18: 已成功远程回血次数
    uint8_t can_free_revive = (info1 >> 19) & 0x01;  // bit 19: 可免费复活
    uint8_t can_pay_revive  = (info1 >> 20) & 0x01;  // bit 20: 可金币买活
    uint16_t revive_cost    = (info1 >> 21) & 0x3FF; // bit 21-30: 买活所需金币

    uint8_t is_out_of_combat = (info2 >> 0) & 0x01;  // bit 0: 是否处于脱战状态

    /* 2. 复活与买活安全保护 (防止因金币不足导致 bit1 卡死后续高位指令) */
    if (referee_recv_info->GameRobotState.current_HP == 0)
    {
        s_sentry_cmd.confirm_respawn = can_free_revive ? 1 : 0;
        if (!can_pay_revive || referee_recv_info->AllowanceData.remaining_gold_coin < revive_cost)
            s_sentry_cmd.exchange_respawn = 0; // 不满足买活条件时强制清0，避免服务器丢弃后续指令
    }
    else
    {
        s_sentry_cmd.confirm_respawn = 0;
        s_sentry_cmd.exchange_respawn = 0;
    }

    /* 3. 补给点兑弹 (单调递增闭环) */
    if (s_req_supply_ammo_add > 0 && (referee_recv_info->RFIDdata.rfid_status & 0x01))
    {
        if (s_sentry_cmd.exchange_ammo_val <= acked_ammo_val)
        {
            s_sentry_cmd.exchange_ammo_val = acked_ammo_val + s_req_supply_ammo_add;
            s_req_supply_ammo_add = 0;
        }
    }

    /* 4. 远程补弹 & 远程回血 (严格每次 +1 单调递增闭环) */
    if (s_req_remote_ammo)
    {
        if (s_sentry_cmd.remote_ammo_req_cnt <= acked_ammo_cnt && acked_ammo_cnt < 15)
            s_sentry_cmd.remote_ammo_req_cnt = acked_ammo_cnt + 1;
        s_req_remote_ammo = 0;
    }
    if (s_req_remote_hp && is_out_of_combat)
    {
        if (s_sentry_cmd.remote_hp_req_cnt <= acked_hp_cnt && acked_hp_cnt < 15)
            s_sentry_cmd.remote_hp_req_cnt = acked_hp_cnt + 1;
        s_req_remote_hp = 0;
    }

    /* 5. 姿态控制与防弱化自动兜底 */
    uint8_t post = s_target_posture;
    uint8_t base_post = (post > 3) ? (post - 3) : post; // 映射为 1进攻 / 2防御 / 3移动
    uint8_t left_sec = (info3 >> ((base_post - 1) * 8)) & 0xFF;

    // 当剩余抗弱化时长 >0 且 <=5秒时，自动临时切到另一姿态防止进入弱化
    if (left_sec > 0 && left_sec <= 5)
    {
        post = (base_post == 3) ? 2 : 3; // 移动快弱化则切防御，进攻/防御快弱化则切移动
    }
    s_sentry_cmd.posture_cmd = post;

    RefereeSendSentryCmd(s_sentry_cmd);
    osDelay(166); // 连同 RefereeSend 内部的 34ms 延时，合计约 200ms (5Hz)
}