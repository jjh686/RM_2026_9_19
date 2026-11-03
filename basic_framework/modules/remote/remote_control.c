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

/* SB(CH7) 异常值连续确认: 连续 N 帧都非法才按上档(急停)处理
 * 3 帧 × 14ms(SBUS 70Hz) ≈ 42ms, 既能挡单帧误码, 急停响应也不迟钝 */
#define SB_BAD_FRAME_THRESHOLD   3u

/* 连续非法帧计数, 仅 fs_to_rc 内部使用 */
static uint8_t s_sb_bad_cnt = 0;

/**
 * @brief 把 SBUS 原始开关值映射为 0(上)/1(中)/2(下), 非法值按规则兜底
 * @param idx  channel 下标, 6=SB(CH7), 7=SC(CH8), 8/9=备用
 * @param val  原始 11bit 值
 */
static uint16_t fs_map_switch(uint8_t idx, uint16_t val)
{
    uint16_t result = 0xFFu;   /* 0xFF 表示落在三个合法区间之外 */

    if (val > SWITCH_HIGH - SWITCH_TOLERANCE && val < SWITCH_HIGH + SWITCH_TOLERANCE)
    {
        result = 2;            /* 下挡 */
    }
    else if (val > SWITCH_MID - SWITCH_TOLERANCE && val < SWITCH_MID + SWITCH_TOLERANCE)
    {
        result = 1;            /* 中挡 */
    }
    else if (val > SWITCH_LOW - SWITCH_TOLERANCE && val < SWITCH_LOW + SWITCH_TOLERANCE)
    {
        result = 0;            /* 上挡 */
    }

    if (idx == 6)
    {
        /* SB: 合法帧清零计数, 非法帧累加, 达到阈值才判上档 */
        if (result != 0xFFu)
        {
            s_sb_bad_cnt = 0;
        }
        else
        {
            if (s_sb_bad_cnt < 0xFFu)
                s_sb_bad_cnt++;

            result = (s_sb_bad_cnt >= SB_BAD_FRAME_THRESHOLD) ? FS_RC_SW_UP
                                                              : FS_RC_SW_MID;
        }
    }
    else if (result == 0xFFu)
    {
        /* 其他开关: 非法值维持原来的默认中档 */
        result = FS_RC_SW_MID;
    }

    return result;
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void fs_to_rc(const uint8_t *sbus_buf)
{
    /* SBUS 帧头固定 0x0F, 校验失败直接丢弃本帧 */
    if (sbus_buf[0] != 0x0F)
        return;
    fs_rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[1]     | sbus_buf[2] << 8) & 0x07FF)-1024u;
    fs_rc_ctrl[TEMP].rc.rocker_r1 = ((sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07FF) -1024u;
    fs_rc_ctrl[TEMP].rc.rocker_l1 = ((sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07FF) - 1024u;
    fs_rc_ctrl[TEMP].rc.rocker_l_ = ((sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07FF) - 1024u;

    for (uint8_t i = 0; i < 4; ++i)
        if (*(&fs_rc_ctrl[TEMP].rc.rocker_l_ + i) > 660){
             *(&fs_rc_ctrl[TEMP].rc.rocker_l_ + i) = 660;
        }else if(*(&fs_rc_ctrl[TEMP].rc.rocker_l_ + i) < -660){
            *(&fs_rc_ctrl[TEMP].rc.rocker_l_ + i) = -660;
        }
           
    /* SBUS 旋钮: 本机 LD 实测 原始 353~1694, 中位 1026
     * 减 1024 使中位归 0, 与 DBUS 的 ±660 语义一致
     * 修后: 最右 -671 / 中位 0 / 最左 +670 */
    fs_rc_ctrl[TEMP].rc.knob_l = ((sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF) - 1024;
    fs_rc_ctrl[TEMP].rc.knob_r = ((sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) & 0x07FF) - 1024;
    //拨杆0上，1中，2下
    fs_rc_ctrl[TEMP].channel[6] = ((sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07FF);
    fs_rc_ctrl[TEMP].channel[7] = ((sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF);
    fs_rc_ctrl[TEMP].channel[8] = ((sbus_buf[12] | sbus_buf[13] << 8 ) & 0x07FF);
    fs_rc_ctrl[TEMP].channel[9] = ((sbus_buf[13] >> 3 | sbus_buf[14] << 5) & 0x07FF);
    for (int i = 6; i <= 9; i++)
    {
        fs_rc_ctrl[TEMP].channel[i] =
            fs_map_switch((uint8_t)i, fs_rc_ctrl[TEMP].channel[i]);
    }

    fs_rc_ctrl[TEMP].rc.switch_left_1 = fs_rc_ctrl[TEMP].channel[6];
    fs_rc_ctrl[TEMP].rc.switch_left_2 = fs_rc_ctrl[TEMP].channel[7];
    fs_rc_ctrl[TEMP].rc.switch_right_1 = fs_rc_ctrl[TEMP].channel[8];
    fs_rc_ctrl[TEMP].rc.switch_right_2 = fs_rc_ctrl[TEMP].channel[9];

    memcpy(&fs_rc_ctrl[LAST], &fs_rc_ctrl[TEMP], sizeof(FS_RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断

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
 * @brief 对fs_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void FS_RemoteControlRxCallback()
{
    DaemonReload(rc_daemon_instance);         // 先喂狗
    fs_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
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
    memset(fs_rc_ctrl, 0, sizeof(fs_rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
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

    rc_init_flag = 1;
    return rc_ctrl;
}

FS_RC_ctrl_t *FS_RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
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

    rc_init_flag = 1;
    return fs_rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}

uint8_t FS_RemoteControlIsOnline(){
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}
