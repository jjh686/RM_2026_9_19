// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
#include "referee_task.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "stdlib.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static FS_RC_ctrl_t *fs_rc_data;        // SBUS 原始数据,初始化时返回
static RC_ctrl_t rc_compat[2];          // 适配成 DBUS 语义的数据[TEMP/LAST]
static int8_t spin_dir = 1;             // +1 逆时针 / -1 顺时针, SC 中→下 切换一次翻转一次
static uint8_t shoot_enable = 0;        // CH6(SA): 1=下档开火, 0=上档停射

static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
// static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;

/* 摇杆单帧最大变化量: SBUS 70Hz 下正常操作每帧最多约 50, 800 足以覆盖极限手动 */
#define RC_ROCKER_MAX_STEP  800

/**
 * @brief 摇杆跳变抑制: 单帧变化量超过阈值视为误码, 沿用上一帧值
 * @param new_val 本帧从 SBUS 解析出的值
 * @param old_val 上一帧的最终值(已过死区)
 * @return 实际采用的值
 */
static int16_t clamp_rocker_step(int16_t new_val, int16_t old_val)
{
    int32_t diff = (int32_t)new_val - (int32_t)old_val;
    if (diff >  RC_ROCKER_MAX_STEP) return old_val;
    if (diff < -RC_ROCKER_MAX_STEP) return old_val;
    return new_val;
}

/* SBUS(CH1~CH8) → DBUS 语义适配, 让 RemoteControlSet/MouseKeySet 无需改动 */
static void FS2RC_Adapt(void)
{
    rc_compat[LAST] = rc_compat[TEMP];   /* 保留上一帧, 供开关/按键边沿判断 */

    /* 摇杆跳变抑制: 单帧变化超过 800 视为误码, 沿用上一帧
     * 防止误码让 rocker_r1 瞬间归零 → vx=0 → vt 斜坡下降 → 速度骤降 */
    rc_compat[TEMP].rc.rocker_r_  = clamp_rocker_step(fs_rc_data[TEMP].rc.rocker_r_,  rc_compat[LAST].rc.rocker_r_);
    rc_compat[TEMP].rc.rocker_r1  = clamp_rocker_step(fs_rc_data[TEMP].rc.rocker_r1, rc_compat[LAST].rc.rocker_r1);
    rc_compat[TEMP].rc.rocker_l_  = clamp_rocker_step(fs_rc_data[TEMP].rc.rocker_l_,  rc_compat[LAST].rc.rocker_l_);
    rc_compat[TEMP].rc.rocker_l1  = clamp_rocker_step(fs_rc_data[TEMP].rc.rocker_l1, rc_compat[LAST].rc.rocker_l1);

    /* SBUS 开关: 0上/1中/2下  →  框架: RC_SW_UP=1 / RC_SW_MID=3 / RC_SW_DOWN=2 */
    static const uint8_t sw_map[3] = { RC_SW_UP, RC_SW_MID, RC_SW_DOWN };
    rc_compat[TEMP].rc.switch_left  = sw_map[fs_rc_data[TEMP].channel[6]];  /* CH7 = SB */
    rc_compat[TEMP].rc.switch_right = sw_map[fs_rc_data[TEMP].channel[7]];  /* CH8 = SC */

    /* CH5(LD 旋钮): 只用于底盘调速, 不再控制陀螺方向 */
    rc_compat[TEMP].rc.dial = fs_rc_data[TEMP].rc.knob_l;

    /* SC(CH8) 中档 → 下档: 翻转一次陀螺方向
     * SC 下档 = 小陀螺, SC 中档 = NO_FOLLOW
     * 效果: 每次拨进小陀螺, 方向换一次 */
    if (switch_is_mid(rc_compat[LAST].rc.switch_right) &&
        switch_is_down(rc_compat[TEMP].rc.switch_right))
    {
        spin_dir = (spin_dir > 0) ? -1 : 1;
    }

    /* CH6(SA 两档): 上档 = 停射, 下档 = 开火
     * 阈值 400 按实测调整 */
    shoot_enable = (fs_rc_data[TEMP].rc.knob_r > 400) ? 1u : 0u;

    /* 摇杆死区, 防止中立点漂移导致底盘缓慢爬行 */
    if (abs(rc_compat[TEMP].rc.rocker_l_) < 40) rc_compat[TEMP].rc.rocker_l_ = 0;
    if (abs(rc_compat[TEMP].rc.rocker_l1) < 40) rc_compat[TEMP].rc.rocker_l1 = 0;
    if (abs(rc_compat[TEMP].rc.rocker_r_) < 40) rc_compat[TEMP].rc.rocker_r_ = 0;
    if (abs(rc_compat[TEMP].rc.rocker_r1) < 40) rc_compat[TEMP].rc.rocker_r1 = 0;
}

void RobotCMDInit()
{
    // BMI088_Init_Config_s bmi088_config = {
    //     .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
    //     .work_mode = BMI088_BLOCK_TRIGGER_MODE,
    //     .spi_acc_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOA,
    //         .cs_pin = GPIO_PIN_4,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .acc_int_config = {
    //         .GPIOx = GPIOC,
    //         .GPIO_Pin = GPIO_PIN_4,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .spi_gyro_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOB,
    //         .cs_pin = GPIO_PIN_0,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .gyro_int_config = {
    //         .GPIO_Pin = GPIO_PIN_5,
    //         .GPIOx = GPIOC,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .heat_pwm_config = {
    //         .htim = &htim10,
    //         .channel = TIM_CHANNEL_1,
    //         .period = 1,
    //     },
    //     .heat_pid_config = {
    //         .Kp = 0.5,
    //         .Ki = 0,
    //         .Kd = 0,
    //         .DeadBand = 0.1,
    //         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //         .IntegralLimit = 100,
    //         .MaxOut = 100,
    //     },
    // };
    //bmi088_test = BMI088Register(&bmi088_config);
    fs_rc_data = FS_RemoteControlInit(&huart3);   // FS/W.BUS SBUS 接收机
    vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .daemon_count = 10,
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

/* 是否处于正式比赛: 数据来自底盘回传; 双板掉线视为不在比赛 */
static uint8_t IsInMatch(void)
{
#ifdef GIMBAL_BOARD
    if (!CANCommIsOnline(cmd_can_comm))
        return 0;
#endif
    return chassis_fetch_data.game_progress == 4;
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_down(rc_compat[TEMP].rc.switch_right)) // 右侧开关状态[下],底盘跟随云台
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        chassis_cmd_send.wz = (float)spin_dir;   // ← 新增, 传方向 +1/-1
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_mid(rc_compat[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        chassis_cmd_send.wz = 0.0f;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    }

    else if (switch_is_up(rc_compat[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        chassis_cmd_send.wz = 0.0f;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    }

    /* 1. 云台控制 */
    if (switch_is_mid(rc_compat[TEMP].rc.switch_left) && VisionIsOnline())
    {
        // 左开关[中]且上位机在线: 视觉/导航接管
        if (vision_recv_data->target_state != NO_TARGET)
        {
            gimbal_cmd_send.yaw   = vision_recv_data->yaw;
            gimbal_cmd_send.pitch = vision_recv_data->pitch;
        }
        if (vision_recv_data->posture_cmd >= 1 && vision_recv_data->posture_cmd <= 6)
        {
            chassis_cmd_send.posture_cmd = vision_recv_data->posture_cmd;
        }
    }
    else if (robot_state == ROBOT_READY)
    {
        // 左开关[下]或视觉无目标: 遥控器摇杆控制云台
        gimbal_cmd_send.yaw   += 0.005f * (float)rc_compat[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch += 0.001f * (float)rc_compat[TEMP].rc.rocker_l1;
    }

    /* 2. 底盘速度控制: 左开关[中]由上位机导航控制, 左开关[下]由遥控器摇杆控制 */
    float ld = (float)rc_compat[TEMP].rc.dial;
    if (ld < -660.0f) ld = -660.0f;
    if (ld >  660.0f) ld =  660.0f;
    float ratio       = (ld + 660.0f) / 1320.0f;   // 0 ~ 1
    float speed_scale = 1.0f * ratio;     // 0.0 ~ 1.0
    chassis_cmd_send.chassis_speed_buff = (int)(speed_scale * 100.0f);

    if (switch_is_mid(rc_compat[TEMP].rc.switch_left))
    {
        // 左开关[中]: 上位机导航模式 (若上位机离线则安全清零速度)
        if (VisionIsOnline())
        {
            chassis_cmd_send.vx = vision_recv_data->vx;
            chassis_cmd_send.vy = vision_recv_data->vy;
            if (chassis_cmd_send.chassis_mode == CHASSIS_NO_FOLLOW)
                chassis_cmd_send.wz = vision_recv_data->wz;
        }
        else
        {
            chassis_cmd_send.vx = 0.0f;
            chassis_cmd_send.vy = 0.0f;
        }
    }
    else
    {
        // 左开关[下]: 纯遥控器手动控制底盘
        chassis_cmd_send.vx = 15.0f * speed_scale * (float)rc_compat[TEMP].rc.rocker_r1;
        chassis_cmd_send.vy = 15.0f * speed_scale * (float)rc_compat[TEMP].rc.rocker_r_;
    }

    // 发射参数
    if (switch_is_up(rc_compat[TEMP].rc.switch_right)) // 右侧开关状态[上],弹舱打开
        ;                                            // 弹舱舵机控制,待添加servo_motor模块,开启
    else
        ; // 弹舱舵机控制,待添加servo_motor模块,关闭

    /* CH6(SA): 上档=安全停射, 下档=摩擦轮开+连发
     * 条件里带 robot_state == ROBOT_READY, 急停时即使 SA 在下档也不出弹 */
    if (shoot_enable && robot_state == ROBOT_READY)
    {
        shoot_cmd_send.shoot_mode    = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.load_mode     = LOAD_BURSTFIRE;
        shoot_cmd_send.bullet_speed  = SMALL_AMU_18;   /* ← 原方案写的 SMALL_AMU_23 不存在 */
        shoot_cmd_send.shoot_rate = 15.0f * speed_scale;    
    }
    else
    {
        shoot_cmd_send.shoot_mode    = SHOOT_ON;       /* 不用 SHOOT_OFF, 避免电机使能全撤 */
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode     = LOAD_STOP;
        shoot_cmd_send.shoot_rate    = 0.0f;
    }

}

/**
 * @brief 左开关[中]: 哨兵视觉/导航自动控制模式
 */
static void SentryAutoSet(void)
{
    /* 1. 右开关仍然保留作为底盘模式/安全档位选择:
     *    右[下]: 边小陀螺边导航移动
     *    右[中]: 不小陀螺, 纯全向导航移动
     *    右[上]: 零力急停
     */
    if (!RemoteControlIsOnline() || switch_is_down(rc_compat[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        chassis_cmd_send.wz = (float)spin_dir;
        chassis_cmd_send.chassis_speed_buff = 80;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_mid(rc_compat[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        chassis_cmd_send.wz = 0.0f;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        return;
    }

    /* 2. 接收上位机(视觉/导航)控制指令, 若上位机离线则安全清零 */
    if (VisionIsOnline())
    {
        // 底盘导航速度
        chassis_cmd_send.vx = vision_recv_data->vx;
        chassis_cmd_send.vy = vision_recv_data->vy;
        if (chassis_cmd_send.chassis_mode == CHASSIS_NO_FOLLOW)
            chassis_cmd_send.wz = vision_recv_data->wz;

        // 云台自瞄角度
        if (vision_recv_data->target_state != NO_TARGET)
        {
            gimbal_cmd_send.yaw   = vision_recv_data->yaw;
            gimbal_cmd_send.pitch = vision_recv_data->pitch;
        }

        // 同步上位机期望的哨兵姿态 (1~6)
        if (vision_recv_data->posture_cmd >= 1 && vision_recv_data->posture_cmd <= 6)
        {
            chassis_cmd_send.posture_cmd = vision_recv_data->posture_cmd;
        }
    }
    else
    {
        // 上位机未连接或断线: 底盘原地停稳, 防止乱跑
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        if (chassis_cmd_send.chassis_mode == CHASSIS_NO_FOLLOW)
            chassis_cmd_send.wz = 0.0f;
    }

    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW; // 键盘模式同样需要明确模式, 否则可能残留 ZERO_FORCE
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;   // 云台同理, 否则可能残留 ZERO_FORCE
    chassis_cmd_send.vx = rc_compat[TEMP].key[KEY_PRESS].w * 300 - rc_compat[TEMP].key[KEY_PRESS].s * 300; // 系数待测
    chassis_cmd_send.vy = rc_compat[TEMP].key[KEY_PRESS].a * 300 - rc_compat[TEMP].key[KEY_PRESS].d * 300;

    gimbal_cmd_send.yaw += (float)rc_compat[TEMP].mouse.x / 660 * 10; // 系数待测
    gimbal_cmd_send.pitch += (float)rc_compat[TEMP].mouse.y / 660 * 10;

    switch (rc_compat[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    {
    case 0:
        shoot_cmd_send.bullet_speed = 15;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = 18;
        break;
    default:
        shoot_cmd_send.bullet_speed = 30;
        break;
    }
    switch (rc_compat[TEMP].key_count[KEY_PRESS][Key_E] % 4) // E键设置发射模式
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    case 1:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    case 2:
        shoot_cmd_send.load_mode = LOAD_3_BULLET;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    switch (rc_compat[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }
    switch (rc_compat[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    switch (rc_compat[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }
    switch (rc_compat[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:

        break;

    default:

        break;
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler(void)
{
    uint8_t sb = rc_compat[TEMP].rc.switch_right;
    uint8_t in_match = IsInMatch();

    /* 遥控器在线且SB在中/下档，或者正式比赛进行中且遥控器已关控，才允许运行 */
    uint8_t allow_run =
        (RemoteControlIsOnline() && (switch_is_mid(sb) || switch_is_down(sb))) ||
        (!RemoteControlIsOnline() && in_match);

    if (!allow_run)
    {
        if (robot_state != ROBOT_STOP)
            LOGERROR("[CMD] emergency stop!");

        robot_state = ROBOT_STOP;

        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        chassis_cmd_send.wz = 0.0f;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;

        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;

        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        return;
    }

    /* 在线且 SB 回到中/下档，立即解除急停 */
    if (robot_state == ROBOT_STOP)
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    FS2RC_Adapt(); // 必须放在所有读取遥控器数据之前
    // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    #ifndef CHASSIS_DEBUG_NO_GIMBAL
        CalcOffsetAngle();
    #endif
    // 根据遥控器状态和比赛阶段决定控制模式
    if (RemoteControlIsOnline())
    {
        if (switch_is_down(rc_compat[TEMP].rc.switch_left))      // 左开关[下]: 遥控器手动控制
            RemoteControlSet();
        else if (switch_is_mid(rc_compat[TEMP].rc.switch_left))  // 左开关[中]: 哨兵视觉/导航自动控制
            SentryAutoSet();
        else if (switch_is_up(rc_compat[TEMP].rc.switch_left))   // 左开关[上]: 键盘控制
            MouseKeySet();
    }
    else if (IsInMatch())
    {
        // 赛场正式比赛阶段(遥控器已关闭): 全自动哨兵模式
        SentryAutoSet();
    }

    /* 枪口热量保护: 剩余热量 < 30 停拨盘 (0xFFFF = 未接裁判系统, 不限制) */
    if (chassis_fetch_data.heat_remain < 30 &&
        shoot_cmd_send.load_mode == LOAD_BURSTFIRE)
        shoot_cmd_send.load_mode = LOAD_STOP;

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    VisionSetReferee(chassis_fetch_data.vision_flag,
                     (float)chassis_fetch_data.current_hp,
                     (float)chassis_fetch_data.remain_ammo,
                     chassis_fetch_data.map_target_x,
                     chassis_fetch_data.map_target_y);

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
