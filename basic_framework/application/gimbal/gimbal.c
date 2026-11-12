#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"          // ← 新增
#include "arm_math.h"    // arm_sin_f32 (只有 dm_test_mode == 3 用到)
#include "bsp_dwt.h"     // DWT_GetTimeline_s (同上, 如果 general_def.h 没带的话)
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor;     // yaw 保持 GM6020
static DMMotorInstance *pitch_motor;   // ← pitch 改成 DMMotorInstance

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static gimbal_mode_e prev_gimbal_mode = GIMBAL_ZERO_FORCE;
static float yaw_align_off = 0.0f, pitch_align_off = 0.0f;

#define YAW_MECH_CENTER_ECD 927.0f   /* 机械中位 yaw 电机 ecd, 按实物标定 */

/* ============================================================
 * 达妙 4310 独立测试开关
 *   1 = 测试模式: 完全绕过遥控, 用下面的变量控制 (推荐先跑这个)
 *   0 = 正式模式: 跟着 gimbal_cmd_recv.pitch 走 (上云台后再切)
 * ============================================================ */
#define DM_PITCH_TEST   0

/* 以下变量可在 EmberProbe 里实时改, 不用重烧 */
volatile uint8_t dm_test_mode    = 3;      /* 0=零力矩(验通信) 1=速度环 2=定点 3=正弦 */
volatile float   dm_target_speed = 6.0f;   /* rad/s, 对应约 0.5 圈/秒 */
volatile float   dm_target_angle = 3.0f;   /* rad, ±12.5 内 */

/* 测试总开关: 1=允许出力, 0=立刻卸力(急停用)
 * 用 EmberProbe 改即可, 不依赖遥控器 */
volatile uint8_t dm_test_enable = 1;

static float   yaw_mech_off  = 0.0f;
static uint8_t yaw_zero_done = 0;
static float   pitch_zero_off  = 0.0f;    /* pitch 上电位置基准(弧度) */
static uint8_t pitch_zero_done = 0;

// static BMI088Instance *bmi088; // 云台IMU
void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8, 
                .Ki = 0.2f,
                .Kd = 0,
                .DeadBand = 0.15f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 300,
            },
            .speed_PID = {
                .Kp = 80,  // 80
                .Ki = 0, 
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 2000,
                .Output_LPF_RC = 0.5f,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    // PITCH

    /* ---------------- PITCH: 达妙 DM-J4310-2EC (MIT 单发力矩, C 板自己算 PID) ----------------
     * DMMotorInit() 走 NORMAL_MODE: 发送帧里 p_des=0, v_des=0, Kp=0, Kd=0, 只发 t_ff
     * 即"电机只出力矩, 所有闭环都在 C 板上算"
     */
    DMMotor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,   // 按实际接线改: 电调中心板2 用 &hcan1, C 板 CAN2 口用 &hcan2
            .tx_id = 0x1,          // 达妙上位机的 CAN ID
            .rx_id = 0x0,          // 达妙上位机的 Master ID
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8.0f,             // 1 rad 角度误差 → 8 rad/s 速度目标
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3.0f,  // 积分最多贡献 3 rad/s
                .MaxOut = 15.0f,        // 最大目标角速度 15 rad/s
            },
            .speed_PID = {
                .Kp = 0.2f,             // 单位 N·m / (rad/s); 空载 0.1~0.3 即可
                .Ki = 0.01f,            // 先给很小, 排除积分反冲
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 0.5f,  // 积分最多贡献 0.5 N·m
                .MaxOut = 2.0f,         // 限 2 N·m (4310 峰值 10 N·m, 撞机械也不至于打坏)
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,   // 测试期用电机编码器
            .speed_feedback_source = MOTOR_FEED,   // 上云台时改成 OTHER_FEED + IMU 指针
            .outer_loop_type = SPEED_LOOP,         // 默认先测速度环
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };

    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&pitch_config);

   if (pitch_motor == NULL)
        LOGERROR("[GIMBAL] DM pitch motor init FAILED! check tx_id<=0x0F / DM_MOTOR_CNT");

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/**
 * @brief 就近限位: 把目标角度折到当前 pitch 反馈的 ±180 范围内
 *        atan2 型反馈(Pitch)在 ±180 处跳变, 直接当目标会让电机绕整圈
 * @param target robot_cmd 给出的目标角度(可能累积到 ±360 以上)
 * @return 与当前角度距离不超过 180° 的等价目标
 */
static float pitch_ref_near(float target)
{
    float cur = gimba_IMU_data->Pitch;   /* 当前反馈, ±180 */
    float err = target - cur;

    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;

    return cur + err;                    /* PID 的误差 = err, 恒在 ±180 内 */
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    /* 电机初始化失败时直接返回, 避免解引用 NULL */
    if (pitch_motor == NULL)
    {
        PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
        return;
    }

    /* 上电第一帧反馈到达后,把机械中位映射到 total_angle 坐标系(只算一次) */
    if (!yaw_zero_done && yaw_motor->dt > 0.0f)
    {
        float mech_now = ((float)yaw_motor->measure.ecd - YAW_MECH_CENTER_ECD) * (360.0f / 8192.0f);
        yaw_mech_off  = yaw_motor->measure.total_angle - mech_now;
        yaw_zero_done = 1;
    }

    /* pitch 上电基准: 让"上电时的位置"对应 0°, 避免上电猛甩 */
    if (!pitch_zero_done && pitch_motor->rx_refresh_dt <= 1)
    {
        pitch_zero_off  = pitch_motor->measure.total_position;
        pitch_zero_done = 1;
    }

    if (gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE && prev_gimbal_mode != GIMBAL_GYRO_MODE)
    {
        pitch_align_off = 0.0f;   /* pitch 用 IMU 绝对角, 水平为 0 */
    }
    prev_gimbal_mode = gimbal_cmd_recv.gimbal_mode;

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
#if DM_PITCH_TEST
    /* ============ 达妙测试: 完全不依赖遥控 ============ */
    if (!dm_test_enable)
    {
        DMMotorStop(pitch_motor);       /* 急停: 力矩清零 */
    }
    else
    {
        DMMotorEnable(pitch_motor);

        if (dm_test_mode == 1)          /* 速度环 */
        {
            DMMotorOuterLoop(pitch_motor, SPEED_LOOP);
            DMMotorSetRef(pitch_motor, dm_target_speed);
        }
        else if (dm_test_mode == 2)     /* 定点位置环 */
        {
            DMMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            DMMotorSetRef(pitch_motor, dm_target_angle);
        }
        else if (dm_test_mode == 3)     /* 正弦往复 */
        {
            float t = DWT_GetTimeline_s();
            DMMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            DMMotorSetRef(pitch_motor, 1.0f * arm_sin_f32(2.0f * 3.14159265f * 0.5f * t));
        }
        else                            /* mode 0: 零力矩, 只验通信 */
        {
            DMMotorOuterLoop(pitch_motor, SPEED_LOOP);
            DMMotorSetRef(pitch_motor, 0.0f);
        }
    }

    /* yaw 照常跟随遥控(测试期间不受影响) */
    DJIMotorEnable(yaw_motor);
    if (!yaw_zero_done) DJIMotorStop(yaw_motor);
    DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw + yaw_mech_off);

#else
    /* ================= 正式模式(上云台后切 DM_PITCH_TEST = 0) ================= */
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_ZERO_FORCE:                     /* 停止 */
        DJIMotorStop(yaw_motor);
        DMMotorStop(pitch_motor);
        break;

    case GIMBAL_GYRO_MODE:                      /* 陀螺仪模式 */
    case GIMBAL_FREE_MODE:                      /* 自由模式 */
        DJIMotorEnable(yaw_motor);
        DMMotorEnable(pitch_motor);
        if (!yaw_zero_done) DJIMotorStop(yaw_motor);

        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw + yaw_mech_off);

        /* ⚠️ gimbal_cmd_recv.pitch 是"度", 达妙是"弧度", 必须转换 */
        DMMotorOuterLoop(pitch_motor, ANGLE_LOOP);
        DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch * DEGREE_2_RAD + pitch_zero_off);
        break;

    default:
        break;
    }
#endif

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}