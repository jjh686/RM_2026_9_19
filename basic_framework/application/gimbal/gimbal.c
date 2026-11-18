#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"          // ← 新增
#include "arm_math.h"    // arm_cos_f32 (重力前馈 cos(imu) 用到)
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

static float   yaw_mech_off  = 0.0f;
static uint8_t yaw_zero_done = 0;
static float   pitch_zero_off  = 0.0f;    /* pitch 上电位置基准(弧度) */

/* 顶部 */
static float pitch_imu_rad  = 0.0f;
static float pitch_gyro_rad = 0.0f;

/* 重力前馈: τ = K × cos(pitch), K 可在 EmberProbe 里实时改 */
volatile float pitch_gravity_k  = -0.75f;    /* 0 = 关闭*/
static   float pitch_gravity_ff = 0.0f;
static   float pitch_soft_start = 0.0f;   /* 0~1, 使能后 0.3s 线性拉满 */
static   float pitch_cmd_ramp   = 0.0f;   /* 实际送给 PID 的目标, 斜坡跟随 */

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
                .MaxOut = 5000,
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
            .can_handle = &hcan2,   // sentry 云台板: 达妙接在 CAN2 口
            .tx_id = 0x02,         // 达妙上位机的 CAN ID
            .rx_id = 0x00,         // 达妙上位机的 Master ID
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = -4.4f,             /* 负号: 物理上负力矩 = 抬头, 而 PID 约定正误差 = 正输出 */
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 1.0f,  // 积分最多贡献 1 rad/s
                .MaxOut = 5.4f,      
                .DeadBand = 0.005f,
            },
            .speed_PID = {
                .Kp = 0.69f,
                .Ki = 0.05f,            // 先给很小, 排除积分反冲
                .Kd = 0.0f,
                /* gimbal.c speed_PID.Improve 补上位 */
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit| PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 0.25f,  // 积分最多贡献 0.25N·m
                .MaxOut = 4.4f,       
                .Output_LPF_RC = 0.02f,   /* 新增: 输出低通, 抑制振荡 */
                .DeadBand = 0.0f,
            },
            /* ← 补这两个, 否则 OTHER_FEED 解引用 NULL */
            .other_angle_feedback_ptr = &pitch_imu_rad,
            .other_speed_feedback_ptr = &pitch_gyro_rad,
            .force_feedforward_ptr    = &pitch_gravity_ff,     /* 新增 */
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,   /* 角度环反馈 = IMU 绝对角 pitch_imu_rad */
            .speed_feedback_source = OTHER_FEED,   /* 速度环反馈 = 陀螺 pitch_gyro_rad */
            .outer_loop_type = ANGLE_LOOP,         /* 串级外环 = 角度环 */
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,  
            .feedforward_flag = FORCE_FEEDFORWARD,              /* 新增 */

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
 * @brief 就近限位: 把目标角度折到当前 pitch 反馈的 ±π 范围内
 *        Pitch 在 ±180° 处跳变, 直接当目标会让电机绕整圈
 * @param target robot_cmd 给出的目标角度(弧度, 可能累积到 ±2π 以上)
 * @return 与当前姿态距离不超过 π 的等价目标(弧度)
 * @note  入参/出参均为弧度, 与主控制链(pitch_cmd_ramp / pitch_imu_rad)统一
 */
static float pitch_ref_near(float target)
{
    float cur = pitch_imu_rad;              /* 当前反馈(弧度), 与 PID 同源 */
    float err = target - cur;

    while (err >  PI) err -= 2.0f * PI;
    while (err < -PI) err += 2.0f * PI;

    return cur + err;                       /* PID 的误差 = err, 恒在 ±π 内 */
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

/* GimbalTask 开头(SubGetMessage 之后) */
    /* ⚠️ 轴系: Pitch 绕 X 轴 → Gyro[0]; Roll 绕 Y 轴 → Gyro[1]
     * 先按 Pitch/Gyro[0] 配, 上电卸力后手托枪管抬头, 看 EmberProbe 里
     * 是 Pitch 还是 Roll 在变 —— 变的那个才是俯仰轴, 用它和同轴陀螺 */
    pitch_imu_rad  = -gimba_IMU_data->Pitch * DEGREE_2_RAD;
    pitch_gyro_rad = gimba_IMU_data->Gyro[0];      /* 已是 rad/s, 不要再乘 */

    /* 软启动: 短暂丢帧(gimbal_mode 掉几帧)时不要把 ramp 清零,
     * 否则 MaxOut 一直被压到 0, PID 等于没开 */
    static uint16_t zf_cnt = 0;

    if (gimbal_cmd_recv.gimbal_mode == GIMBAL_ZERO_FORCE)
    {
        if (zf_cnt < 40) zf_cnt++;              /* 40 × 5ms = 200ms 容错 */
    }
    else
    {
        zf_cnt = 0;
    }

    if (zf_cnt >= 40)                           /* 真的停了才清零 */
    {
        pitch_soft_start = 0.0f;
    }
    else
    {
        pitch_soft_start += 0.005f / 0.3f;      /* 5ms 周期 → 0.3s 拉满 */
        if (pitch_soft_start > 1.0f) pitch_soft_start = 1.0f;
    }

    pitch_gravity_ff = pitch_gravity_k * arm_cos_f32(pitch_imu_rad) * pitch_soft_start;

    // /* 让 PID 输出上限也参与软启动: 使能后从 0 慢慢放到 3.0 N·m */
    // if (pitch_motor != NULL)
    //     pitch_motor->speed_PID.MaxOut = 3.0f * pitch_soft_start;

    /* ---------- 切档原地接管 ----------
     * 从 ZERO_FORCE(急停) 切回使能时, 把上电基准重新对齐到当前姿态,
     * 让 SetRef = pitch_imu_rad (误差 0), 云台不跳变 */
    static gimbal_mode_e prev_gimbal_mode = GIMBAL_ZERO_FORCE;

    if (prev_gimbal_mode == GIMBAL_ZERO_FORCE &&
        gimbal_cmd_recv.gimbal_mode != GIMBAL_ZERO_FORCE)
    {
        pitch_zero_off = 0.0f;
    }
    prev_gimbal_mode = gimbal_cmd_recv.gimbal_mode;

    /* 电机初始化失败时直接返回, 避免解引用 NULL */
    if (pitch_motor == NULL)
    {
        PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
        return;
    }

    /* 上电第一帧反馈到达后, 把机械中位映射到 total_angle 坐标系(只算一次) */
    if (!yaw_zero_done && yaw_motor->dt > 0.0f)
    {
        /* 过 0/8191 边界时走最短路径, 避免开机猛转 350° */
        float diff_ecd = (float)yaw_motor->measure.ecd - (float)YAW_CHASSIS_ALIGN_ECD;
        if (diff_ecd >  4096.0f) diff_ecd -= 8192.0f;
        if (diff_ecd < -4096.0f) diff_ecd += 8192.0f;

        float mech_now = diff_ecd * ECD_ANGLE_COEF_DJI;   /* 单位: 度 */
        yaw_mech_off  = yaw_motor->measure.total_angle - mech_now;
        yaw_zero_done = 1;
    }

    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_ZERO_FORCE:                     /* 停止 */
        DJIMotorStop(yaw_motor);
        DMMotorStop(pitch_motor);
        if (zf_cnt >= 40)                       /* 真停了才同步, 短暂丢帧不动 */
            pitch_cmd_ramp = pitch_imu_rad;
        break;

    case GIMBAL_GYRO_MODE:                      /* 陀螺仪模式 */
    case GIMBAL_FREE_MODE:                      /* 自由模式 */
        DJIMotorEnable(yaw_motor);
        DMMotorEnable(pitch_motor);
        if (!yaw_zero_done) DJIMotorStop(yaw_motor);

        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw + yaw_mech_off);

        /* 双保险: 即使上层 pitch 累积失控, 也把目标钳在机械行程内*/
        {
            const float LIM_UP   = PITCH_MAX_ANGLE * DEGREE_2_RAD;
            const float LIM_DOWN = PITCH_MIN_ANGLE * DEGREE_2_RAD;

            /* 先把目标折到当前姿态的 ±π 内(弧度), 再按机械行程钳位 */
            float tgt = pitch_ref_near(gimbal_cmd_recv.pitch * DEGREE_2_RAD + pitch_zero_off);

            if (tgt > pitch_zero_off + LIM_UP)   tgt = pitch_zero_off + LIM_UP;
            if (tgt < pitch_zero_off + LIM_DOWN) tgt = pitch_zero_off + LIM_DOWN;

            {
                float diff = tgt - pitch_cmd_ramp;
                const float STEP = 1.8f * 0.005f;      /* 1.8 rad/s × 5ms */

                if (diff >  STEP) diff =  STEP;
                if (diff < -STEP) diff = -STEP;

                pitch_cmd_ramp += diff;
            }

            DMMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            DMMotorSetRef(pitch_motor, pitch_cmd_ramp);
        }

        break;

    default:
        break;
    }

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}