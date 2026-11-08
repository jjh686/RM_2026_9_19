#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

/* ---------------- 拨盘卡弹检测 ----------------
 * 实测依据(2026-11-08):
 *   正常运转: speed≈Ref(11000~17400), Output ±1500
 *   瞬时挤压: speed 掉到 543~3081, Output=5000, 只持续 1 个采样点(~100ms)
 *   真卡死:   speed≈0, Output=5000 持续 39s, 电流爬到 4.5A
 * 所以判定用「speed 低 + Output 高 + 持续 250ms」, 能过滤瞬时挤压
 */
#define LOADER_STALL_SPEED      1500.0f     /* aps, 低于此视为没在转 */
#define LOADER_STALL_OUT        4000.0f     /* 电流指令, 高于此视为硬顶 */
#define LOADER_STALL_MS         250.0f      /* 持续多久判定卡弹 */
#define LOADER_REVERSE_MS       300.0f      /* 反转退弹时长 */
#define LOADER_REVERSE_SPEED  (-6000.0f)    /* 反转速度 aps */
#define LOADER_RETRY_MAX        3           /* 连续退弹次数上限 */
#define LOADER_OK_RESET_MS      600.0f      /* 正常转满 600ms 归还配额 */
#define LOADER_COOLDOWN_MS      1000.0f     /* 初始冷却时长 */
#define LOADER_COOLDOWN_MAX     4000.0f     /* 冷却时长上限(指数退避) */
#define LOADER_STALL_TOTAL_MAX  3000.0f     /* 累计堵转 3s 强制停机(C610 无温度反馈) */

typedef enum
{
    LOADER_NORMAL = 0,      /* 正常 */
    LOADER_REVERSING,       /* 反转退弹中 */
    LOADER_COOLDOWN,        /* 冷却中, 期满自动重试 */
} loader_state_e;

static loader_state_e loader_state       = LOADER_NORMAL;
static float loader_stall_ms             = 0.0f;    /* 本次堵转起始时刻 */
static float loader_reverse_end          = 0.0f;
static float loader_cooldown_end         = 0.0f;
static float loader_ok_ms                = 0.0f;    /* 正常转动累计 */
static float loader_stall_total_ms       = 0.0f;    /* 累计堵转时间 */
static uint8_t loader_retry_cnt          = 0;
static float loader_cooldown_ms          = LOADER_COOLDOWN_MS;
static uint16_t loader_jam_count         = 0;       /* 总卡弹次数, 调试观察用 */


/* ============================================================
 * 调试开关: 无需遥控, 拨盘恒速旋转
 *   1 = 开启调试(忽略 shoot_mode / load_mode, 电机直接转)
 *   0 = 恢复正常遥控控制
 * 测试完务必改回 0
 * ============================================================ */
#define LOADER_DEBUG_RUN     1
#define LOADER_DEBUG_SPEED   3240.0f    /* aps, 约输出轴 90°/s (15 rpm) */

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

/* ============================================================
 * 调试用: 拨盘目标转速 (aps)，可在 EmberProbe 里直接改，无需重烧
 *   1620 → 45°/s   3240 → 90°/s   6480 → 180°/s
 * ============================================================ */
float loader_debug_speed = 3240.0f;

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 0, // 20
                .Ki = 0, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 1,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 2; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 0, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 200,
            },
            .speed_PID = {
                .Kp = 3, 
                .Ki = 0.3, 
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 1000,
                .MaxOut = 5000,
                .Output_LPF_RC = 0.05f, // 0 → 0.05 (50ms 滤波)
            },
            .current_PID = {
                .Kp = 0.7, 
                .Ki = 0.1, 
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/**
 * @brief 拨盘卡弹检测与退弹处理
 * @return 1 = 正在退弹/冷却/强制停机(调用方跳过本轮发射), 0 = 正常
 */
static uint8_t LoaderJamHandler(void)
{
    const float now = DWT_GetTimeline_ms();

    /* ========== 冷却中: 期满自动恢复 ========== */
    if (loader_state == LOADER_COOLDOWN)
    {
        DJIMotorSetRef(loader, 0.0f);
        if (now < loader_cooldown_end)
            return 1;

        loader_state     = LOADER_NORMAL;
        loader_retry_cnt = 0;
        loader_ok_ms     = 0.0f;
        loader_cooldown_ms = LOADER_COOLDOWN_MS;   /* 复位冷却时长 */
        LOGINFO("[SHOOT] loader cooldown done, resume");
        return 0;
    }

    /* ========== 退弹中 ========== */
    if (loader_state == LOADER_REVERSING)
    {
        if (now < loader_reverse_end)
        {
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, LOADER_REVERSE_SPEED);
            return 1;
        }
        loader_state    = LOADER_NORMAL;
        loader_stall_ms = 0.0f;
        return 0;
    }

    /* ========== 正常转动判定 ========== */
    const uint8_t rotating =
        (fabsf(loader->measure.speed_aps) >= LOADER_STALL_SPEED) ||
        (fabsf(loader->motor_controller.speed_PID.Output) < LOADER_STALL_OUT);

    if (rotating)
    {
        loader_stall_ms = 0.0f;

        /* 累计堵转时间衰减: 正常转 1ms 抵消 1ms, 避免长期累积误触发 */
        if (loader_stall_total_ms > 0.0f)
        {
            loader_stall_total_ms -= 5.0f;      /* 与 ShootTask 200Hz 对应 */
            if (loader_stall_total_ms < 0.0f)
                loader_stall_total_ms = 0.0f;
        }

        /* 正常转满 600ms, 归还退弹配额(连发时不会越积越多) */
        loader_ok_ms += 5.0f;                   /* ShootTask 200Hz */
        if (loader_ok_ms >= LOADER_OK_RESET_MS)
        {
            loader_retry_cnt   = 0;
            loader_ok_ms       = 0.0f;
            loader_cooldown_ms = LOADER_COOLDOWN_MS;
        }
        return 0;
    }

    /* 疑似卡住, 但还没到判定时间 */
    loader_ok_ms = 0.0f;
    if (loader_stall_ms == 0.0f)
    {
        loader_stall_ms = now;
        return 0;
    }
    if (now - loader_stall_ms < LOADER_STALL_MS)
        return 0;

    /* ========== 确认卡弹 ========== */
    const float stall_duration = now - loader_stall_ms;   /* ← 先算出来 */
    loader_stall_ms = 0.0f;
    loader_retry_cnt++;
    loader_jam_count++;

    /* 累计堵转保护: C610 没有温度反馈, 用时间替代 */
    loader_stall_total_ms += stall_duration;              /* ← 用时长 */
    if (loader_stall_total_ms > LOADER_STALL_TOTAL_MAX)
    {
        LOGERROR("[SHOOT] loader stall total %.0fms, FORCE STOP", loader_stall_total_ms);
        DJIMotorSetRef(loader, 0.0f);
        DJIMotorStop(loader);
        /* 进入长冷却, 给人时间处理 */
        loader_state        = LOADER_COOLDOWN;
        loader_cooldown_end = now + 5000.0f;
        loader_stall_total_ms = 0.0f;
        return 1;
    }

    /* 配额用尽 -> 冷却, 自动恢复 */
    if (loader_retry_cnt > LOADER_RETRY_MAX)
    {
        loader_state        = LOADER_COOLDOWN;
        loader_cooldown_end = now + loader_cooldown_ms;
        loader_retry_cnt    = 0;

        /* 指数退避: 1s -> 2s -> 4s */
        loader_cooldown_ms *= 2.0f;
        if (loader_cooldown_ms > LOADER_COOLDOWN_MAX)
            loader_cooldown_ms = LOADER_COOLDOWN_MAX;

        DJIMotorSetRef(loader, 0.0f);
        LOGWARNING("[SHOOT] loader cooldown %.0fms", loader_cooldown_ms);
        return 1;
    }

    /* ========== 反转退弹 ========== */
    loader_state       = LOADER_REVERSING;
    loader_reverse_end = now + LOADER_REVERSE_MS;
    DJIMotorOuterLoop(loader, SPEED_LOOP);
    DJIMotorSetRef(loader, LOADER_REVERSE_SPEED);
    LOGWARNING("[SHOOT] loader jam #%d, reversing (%d/%d)",
               loader_jam_count, loader_retry_cnt, LOADER_RETRY_MAX);
    return 1;
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    /* ===== 卡弹检测: 必须在所有发射逻辑之前, 调试模式也要保护 ===== */
    if (LoaderJamHandler())
        return;

#if LOADER_DEBUG_RUN
    /* ---------- 调试模式: 不理会遥控, 拨盘恒速转 ---------- */
    DJIMotorEnable(loader);              /* 清 stop_flag, 允许出力 */
    DJIMotorOuterLoop(loader, SPEED_LOOP);
    DJIMotorSetRef(loader, loader_debug_speed);
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
    return;                              /* 跳过后面所有遥控逻辑 */
#endif

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
        DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
        hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
        dead_time = 150;                                                                    // 完成1发弹丸发射的时间
        break;
    // 三连发,如果不需要后续可能删除
    case LOAD_3_BULLET:
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                                  // 切换到速度环
        DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
        hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
        dead_time = 300;                                                                        // 完成3发弹丸发射的时间
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, LOADER_REVERSE_SPEED);
        loader_retry_cnt = 0;      /* 手动退弹时重置配额 */
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        case SMALL_AMU_18:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
            DJIMotorSetRef(friction_l, 30000);
            DJIMotorSetRef(friction_r, 30000);
            break;
        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}