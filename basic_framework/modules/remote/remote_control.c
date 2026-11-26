#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

#define REMOTE_CONTROL_FRAME_SIZE 18u // 遥控器接收的buffer大小
#define SBUS_FRAME_SIZE 25u//fs遥控器的buffer

//新遥控器摇杆位置
#define SWITCH_HIGH 1807
#define SWITCH_MID  1024
#define SWITCH_LOW  240
#define SWITCH_TOLERANCE 200

// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static FS_RC_ctrl_t fs_rc_ctrl[2]; //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{
    // 摇杆,直接解算时减去偏置
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                 // 左侧拨轮
    RectifyRCjoystick();
    // 开关,0左1右
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // 鼠标解析
    rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
    rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

    //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
}

/* FS/SBUS 严格在线门控与字节流滑窗同步 (对齐 sentry_all/05_云台 实测稳定方案) */
#define FS_RC_TIMEOUT_MS            100u

static uint8_t fs_mode_flag = 0;
static volatile uint8_t fs_seen = 0;
static volatile uint8_t fs_fault = 1;
static volatile uint32_t fs_last_valid_tick = 0;
static uint8_t fs_partial[SBUS_FRAME_SIZE];
static uint8_t fs_partial_len = 0;
static uint32_t fs_partial_ms = 0;
static uint32_t fs_uart_errors_seen = 0;

/**
 * @brief 把 SBUS 原始开关值映射为 0(上)/1(中)/2(下), 落在非法区间返回 -1(拒收整帧, 不再兜底中档)
 */
static int16_t fs_decode_switch(uint16_t val)
{
    if (val > SWITCH_HIGH - SWITCH_TOLERANCE && val < SWITCH_HIGH + SWITCH_TOLERANCE)
        return FS_RC_SW_DOWN; /* 2: 下挡 */
    if (val > SWITCH_MID - SWITCH_TOLERANCE && val < SWITCH_MID + SWITCH_TOLERANCE)
        return FS_RC_SW_MID;  /* 1: 中挡 */
    if (val > SWITCH_LOW - SWITCH_TOLERANCE && val < SWITCH_LOW + SWITCH_TOLERANCE)
        return FS_RC_SW_UP;   /* 0: 上挡 */
    return -1;
}

/**
 * @brief 解析一帧已通过帧头/帧尾/flags校验的 SBUS 数据; 仅当 SB/SC 开关均合法才提交
 * @return 1=合法已提交, 0=开关值非法(整帧拒收)
 */
static uint8_t fs_parse_frame(const uint8_t *sbus_buf)
{
    FS_RC_ctrl_t parsed;
    memset(&parsed, 0, sizeof(parsed));

    parsed.rc.rocker_r_ = (int16_t)(((sbus_buf[1]      | sbus_buf[2] << 8) & 0x07FF) - 1024);
    parsed.rc.rocker_r1 = (int16_t)(((sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07FF) - 1024);
    parsed.rc.rocker_l1 = (int16_t)(((sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07FF) - 1024);
    parsed.rc.rocker_l_ = (int16_t)(((sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07FF) - 1024);

    int16_t *rockers[4] = {
        &parsed.rc.rocker_l_, &parsed.rc.rocker_l1,
        &parsed.rc.rocker_r_, &parsed.rc.rocker_r1
    };
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (*rockers[i] > 660)
            *rockers[i] = 660;
        else if (*rockers[i] < -660)
            *rockers[i] = -660;
    }

    /* SBUS 旋钮: 本机 LD/RD 实测 353~1694, 中位 1026, 减 1024 后最右 -671 / 中位 0 / 最左 +670 */
    parsed.rc.knob_l = (int16_t)(((sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF) - 1024);
    parsed.rc.knob_r = (int16_t)(((sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) & 0x07FF) - 1024);

    /* ET08A 8 通道仅使用 CH7=SB(channel[6]) 与 CH8=SC(channel[7]), 校验合法区间后才提交 */
    uint16_t raw_ch6 = (uint16_t)((sbus_buf[9]  >> 2 | sbus_buf[10] << 6) & 0x07FF);
    uint16_t raw_ch7 = (uint16_t)((sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF);
    int16_t sw_sb = fs_decode_switch(raw_ch6);
    int16_t sw_sc = fs_decode_switch(raw_ch7);
    if (sw_sb < 0 || sw_sc < 0)
        return 0;

    parsed.channel[6] = sw_sb;
    parsed.channel[7] = sw_sc;
    parsed.channel[8] = FS_RC_SW_MID;
    parsed.channel[9] = FS_RC_SW_MID;

    parsed.rc.switch_left_1  = (uint8_t)parsed.channel[6];
    parsed.rc.switch_left_2  = (uint8_t)parsed.channel[7];
    parsed.rc.switch_right_1 = (uint8_t)parsed.channel[8];
    parsed.rc.switch_right_2 = (uint8_t)parsed.channel[9];

    fs_rc_ctrl[LAST] = fs_rc_ctrl[TEMP];
    fs_rc_ctrl[TEMP] = parsed;
    return 1;
}

static void fs_uart_fault(void)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (rc_usart_instance != NULL)
    {
        uint32_t errors = rc_usart_instance->error_events;
        if (errors != fs_uart_errors_seen)
        {
            fs_uart_errors_seen = errors;
            fs_partial_len = 0;
            fs_fault = 1;
        }
    }
    __set_PRIMASK(irq);
}

static uint8_t fs_accept_frame(const uint8_t *frame, uint32_t now)
{
    /* flags 高 4 位必须为 0, 否则必为错位/坏帧 */
    if (frame[23] & 0xF0u)
    {
        fs_fault = 1;
        return 0;
    }
    /* bit3 = Failsafe(遥控关机或失联): 立即置故障, 不喂狗 */
    if (frame[23] & 0x08u)
    {
        fs_fault = 1;
        return 1;
    }
    /* bit2 = 单帧射频 FrameLost: 不立即置故障(防偶发丢包抖动), 仅不喂狗, 连续超时由 FS_RC_TIMEOUT_MS 判离线 */
    if (frame[23] & 0x04u)
    {
        return 1;
    }
    if (!fs_parse_frame(frame))
    {
        fs_fault = 1;
        return 0;
    }
    fs_last_valid_tick = now;
    fs_seen = 1;
    fs_fault = 0;
    DaemonReload(rc_daemon_instance);
    return 1;
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
    DaemonReload(rc_daemon_instance);         // 先喂狗
    sbus_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief SBUS 字节流滑窗同步回调: 拒绝短帧/错位帧/非法帧尾/Failsafe 帧
 */
static void FS_RemoteControlRxCallback(void)
{
    fs_uart_fault();
    uint32_t now = HAL_GetTick();
    if (fs_partial_len && (uint32_t)(now - fs_partial_ms) > 20u)
    {
        fs_partial_len = 0;
        fs_fault = 1;
    }

    for (uint16_t i = 0; i < rc_usart_instance->recv_len; ++i)
    {
        uint8_t byte = rc_usart_instance->recv_buff[i];
        if (!fs_partial_len && byte != 0x0Fu)
            continue;

        fs_partial[fs_partial_len++] = byte;
        fs_partial_ms = now;
        if (fs_partial_len != SBUS_FRAME_SIZE)
            continue;

        uint8_t tail_ok = (fs_partial[24] == 0x00u) || ((fs_partial[24] & 0x0Fu) == 0x04u);
        if (tail_ok && fs_accept_frame(fs_partial, now))
        {
            fs_partial_len = 0;
        }
        else
        {
            if (!tail_ok)
                fs_fault = 1;
            uint8_t skip = 1;
            while (skip < SBUS_FRAME_SIZE && fs_partial[skip] != 0x0Fu)
                skip++;
            fs_partial_len = (uint8_t)(SBUS_FRAME_SIZE - skip);
            if (fs_partial_len > 0)
                memmove(fs_partial, fs_partial + skip, fs_partial_len);
        }
    }
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

static void FS_RCLostCallback(void *id)
{
    (void)id;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* 防止守护线程刚判定超时时, 中断恰好收到了有效新帧 */
    if (fs_seen && !fs_fault && (uint32_t)(HAL_GetTick() - fs_last_valid_tick) <= FS_RC_TIMEOUT_MS)
    {
        __set_PRIMASK(primask);
        return;
    }
    uint8_t report = !fs_fault;
    fs_fault = 1;
    memset(fs_rc_ctrl, 0, sizeof(fs_rc_ctrl)); // 清空遥控器数据(channel=0 -> 急停上档)
    __set_PRIMASK(primask);
    if (report)
        LOGWARNING("[rc] remote control lost");
    /* 不在此重启 DMA: DMA 一直处于接收态, 离线时反复重启会把接收机 Failsafe 帧切碎成错位假帧 */
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    fs_mode_flag = 0;
    rc_init_flag = 1;
    return rc_ctrl;
}

FS_RC_ctrl_t *FS_RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    fs_seen = 0;
    fs_fault = 1;
    fs_last_valid_tick = 0;
    fs_partial_len = 0;
    fs_uart_errors_seen = 0;

    USART_Init_Config_s conf;
    conf.module_callback = FS_RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = SBUS_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = FS_RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    fs_mode_flag = 1;
    rc_init_flag = 1;
    return fs_rc_ctrl;
}

uint8_t FS_RemoteControlIsOnline()
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (rc_init_flag)
        fs_uart_fault();
    uint8_t online = (uint8_t)(rc_init_flag && fs_seen && !fs_fault &&
                               (uint32_t)(HAL_GetTick() - fs_last_valid_tick) <= FS_RC_TIMEOUT_MS);
    __set_PRIMASK(irq);
    return online;
}

uint8_t RemoteControlIsOnline()
{
    if (!rc_init_flag)
        return 0;
    if (fs_mode_flag)
        return FS_RemoteControlIsOnline();
    return DaemonIsOnline(rc_daemon_instance);
}
