#ifndef REFEREE_H
#define REFEREE_H

#include "rm_referee.h"
#include "robot_def.h"

/**
 * @brief 初始化裁判系统交互任务
 */
referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data);

/**
 * @brief 在referee task之前调用
 */
void MyUIInit(void);

/**
 * @brief 裁判系统上行发送任务(哨兵0x0120自主决策指令)
 */
void UITask(void);

/* ---------------- 哨兵自主决策外部调用接口 (0x0120) ---------------- */
void SentrySetPosture(uint8_t posture);           // 设置目标姿态: 1进攻, 2防御, 3移动, 4~6强化姿态
void SentryRequestConfirmRespawn(uint8_t enable); // 请求确认买活(消耗金币立即复活): 1兑换, 0不兑换
void SentryRequestSupplyAmmo(uint16_t add_ammo);  // 在补给点请求兑换 add_ammo 发弹丸
void SentryRequestRemoteAmmo(void);               // 请求一次远程兑换发弹量(内部自动遵循+1递增规则)
void SentryRequestRemoteHP(void);                 // 请求一次远程兑换血量(内部自动遵循+1递增规则)

#endif // REFEREE_H