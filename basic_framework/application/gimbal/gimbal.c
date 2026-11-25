#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "arm_math.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "bsp_dwt.h"
#include <math.h>
#include <string.h>

static attitude_t *gimba_IMU_data;      // 云台IMU数据
static DJIMotorInstance *yaw_motor;     // yaw 保持 GM6020
static DMMotorInstance *pitch_motor;    // pitch 达妙 DM-J4310-2EC

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static float   yaw_mech_off  = 0.0f;
static uint8_t yaw_zero_done = 0;

/* yaw 陀螺角速度 rad/s → 度/秒, 给速度内环(OTHER_FEED)用。
 * 必须在 1kHz 的 MotorControlTask 里刷新(见 GimbalUpdateYawGyroDeg),
 * 若放在 200Hz 的 GimbalTask, 速度内环会拿到 5ms 阶梯值 -> 力矩纹波。 */
float yaw_gyro_deg = 0.0f;

/* 调试计数: 云台实际处于"卸力"状态的帧数(1 帧 = 5ms), 只在 EmberProbe 里看 */
volatile uint32_t dbg_gimbal_stop_frm = 0;

/* IMU yaw 与电机 total_angle 的转向关系:
 * 上电后若 yaw 越跑越远(正反馈), 把这里改成 (-1.0f) */
#define YAW_IMU_DIR   (1.0f)

/* ---------------- PITCH 控制参数 (完整移植自 sentry_all/05_云台, 换算为 N·m 单位) ----------------
 * 坐标系约定(统一"抬头为正", 与 PITCH_MAX_ANGLE=+20.5° / PITCH_MIN_ANGLE=-31.5° / 摇杆方向完全一致):
 *   - pitch_imu_rad  = -IMU.Pitch * DEGREE_2_RAD  (抬头为正, rad)
 *   - pitch_gyro_rad = -IMU.Gyro[0]               (抬头为正, rad/s, 与 pitch_imu_rad 严格同号)
 *   - 控制器内部正输出 = 抬头力矩, 最终发给 DM4310 前统一取反(实车负力矩 = 抬头)
 *   - 等效换算系数: 05_云台 1 count = 10/20000 = 0.0005 N·m
 *     -> Speed Kp: 900 * 0.0005 = 0.45 N·m/(rad/s)
 *     -> Speed Ki: 500 * 0.0005 = 0.25 N·m/rad
 *     -> Speed I_Limit: 1200 * 0.0005 = 0.60 N·m
 */
#define PITCH_ANGLE_KP              25.0f
#define PITCH_SPEED_KP              0.45f
#define PITCH_SPEED_KI              0.25f
#define PITCH_SPEED_I_LIMIT         0.60f
#define PITCH_TORQUE_LIMIT_NM       4.0f
#define PITCH_SPEED_LIMIT_RAD_S     (75.0f  * DEGREE_2_RAD)
#define PITCH_ACCEL_RAD_S2          (720.0f * DEGREE_2_RAD)
#define PITCH_BRAKE_RAD_S2          (180.0f * DEGREE_2_RAD)
#define PITCH_GRAVITY_RAMP_NM_S     5.0f
#define PITCH_SPEED_PI_MAX_DT_S     0.030f

/* 1kHz 刷新的 pitch 反馈(不能放在 200Hz GimbalTask, 否则速度内环吃到 5ms 阶梯值抖动) */
static float pitch_imu_rad  = 0.0f;
static float pitch_gyro_rad = 0.0f;

/* 可在 EmberProbe 里在线微调的前馈系数:
 *   pitch_gravity_k   = -0.45 N·m (电机系常值重力补偿, 负号 = 抬头, 对齐 05_云台)
 *   pitch_speed_ff_nm =  0.75 N·m (速度型动摩擦前馈幅值, 按指令速度叠加, 对齐 05_云台) */
volatile float pitch_gravity_k   = -0.45f;
volatile float pitch_speed_ff_nm =  0.75f;
static   float pitch_gravity_ff  =  0.0f;   /* 当前斜坡后的重力前馈(电机系 N·m, 供调试观察) */

typedef struct
{
    float speed_ref_rad_s;    /* 速度规划器当前输出参考速度(rad/s, 抬头为正) */
    float gravity_motor_nm;   /* 斜坡软启动后的重力前馈(电机系 N·m) */
    float pi_i;               /* 速度环积分状态(抬头为正, N·m) */
    float pi_prev_err;        /* 上一拍速度误差(梯形积分用) */
    uint8_t pi_prev_valid;
} PitchCtrlState_s;

static PitchCtrlState_s pitch_ctrl;

static float PitchClamp(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

static float PitchApproach(float cur, float target, float step)
{
    return cur + PitchClamp(target - cur, -step, step);
}

/**
 * @brief 1kHz Pitch 速度环 + 速度规划/终端制动 + 条件抗饱和 PI + 速度摩擦前馈 + 重力前馈
 * @return 直接输出给 DM4310 的电机系力矩(N·m, 负力矩 = 抬头)
 */
static float GimbalPitchSpeedUpdate(void *context, PIDInstance *pid,
                                    float measure_rad_s, float raw_ref_rad_s)
{
    PitchCtrlState_s *st = (PitchCtrlState_s *)context;
    float dt_s = DWT_GetDeltaT(&pid->DWT_CNT);
    if (!isfinite(dt_s) || dt_s <= 0.0f) dt_s = 0.001f;
    if (dt_s > PITCH_SPEED_PI_MAX_DT_S)  dt_s = PITCH_SPEED_PI_MAX_DT_S;

    /* 1. 速度规划与终端平方根制动 (移植自 05_云台 GimbalPitchSpeedReference) */
    const float angle_err = pitch_motor->angle_PID.Err;
    const float angle_deg = pitch_imu_rad * RAD_2_DEGREE;
    const float pos_dist  = (PITCH_MAX_ANGLE - angle_deg) * DEGREE_2_RAD;
    const float neg_dist  = (angle_deg - PITCH_MIN_ANGLE) * DEGREE_2_RAD;

    const float pos_cap = fminf(PITCH_SPEED_LIMIT_RAD_S,
                                sqrtf(2.0f * PITCH_BRAKE_RAD_S2 * fmaxf(0.0f, fminf(angle_err,  pos_dist))));
    const float neg_cap = fminf(PITCH_SPEED_LIMIT_RAD_S,
                                sqrtf(2.0f * PITCH_BRAKE_RAD_S2 * fmaxf(0.0f, fminf(-angle_err, neg_dist))));

    float target_v = PitchClamp(raw_ref_rad_s, -neg_cap, pos_cap);
    float next_v   = PitchApproach(st->speed_ref_rad_s, target_v, PITCH_ACCEL_RAD_S2 * dt_s);
    float ref_rad_s = PitchClamp(next_v, -neg_cap, pos_cap);
    st->speed_ref_rad_s = ref_rad_s;

    /* 2. 条件抗饱和梯形积分速度 PI (移植自 05_云台 GimbalSpeedPiStep, 无输出低通) */
    const float kp        = fmaxf(pid->Kp, 0.0f);
    const float ki        = fmaxf(pid->Ki, 0.0f);
    const float i_limit   = fmaxf(pid->IntegralLimit, 0.0f);
    const float out_limit = (pid->MaxOut > 0.0f) ? pid->MaxOut : PITCH_TORQUE_LIMIT_NM;

    st->pi_i = PitchClamp(st->pi_i, -i_limit, i_limit);
    const float err = ref_rad_s - measure_rad_s;
    const float p   = kp * err;
    const float prev_err = st->pi_prev_valid ? st->pi_prev_err : err;
    const float delta_i  = ki * 0.5f * (err + prev_err) * dt_s;

    float cand_i   = PitchClamp(st->pi_i + delta_i, -i_limit, i_limit);
    float cand_out = p + cand_i;

    /* 仅拒绝加剧饱和的积分方向, 允许积分随时向退饱和方向卸荷 */
    if (cand_out > out_limit && cand_i > st->pi_i)
    {
        cand_i = (st->pi_i < 0.0f) ? fminf(cand_i, 0.0f) : st->pi_i;
    }
    else if (cand_out < -out_limit && cand_i < st->pi_i)
    {
        cand_i = (st->pi_i > 0.0f) ? fmaxf(cand_i, 0.0f) : st->pi_i;
    }

    float applied_delta = cand_i - st->pi_i;
    float pi_out_up     = PitchClamp(p + cand_i, -out_limit, out_limit);

    st->pi_i          = cand_i;
    st->pi_prev_err   = err;
    st->pi_prev_valid = 1u;

    /* 3. 速度型动摩擦前馈(按指令速度, 不放大陀螺高频振动) */
    const float ff_amp = fabsf(pitch_speed_ff_nm);
    float ff_up_nm = pitch_speed_ff_nm * ref_rad_s / PITCH_SPEED_LIMIT_RAD_S;
    ff_up_nm = PitchClamp(ff_up_nm, -ff_amp, ff_amp);

    /* 4. 常值重力前馈(电机系 N·m, 带 5.0 N·m/s 软启动斜坡) */
    float grav_target_motor_nm = pitch_gravity_k * arm_cos_f32(pitch_imu_rad);
    st->gravity_motor_nm = PitchApproach(st->gravity_motor_nm, grav_target_motor_nm,
                                         PITCH_GRAVITY_RAMP_NM_S * dt_s);
    pitch_gravity_ff = st->gravity_motor_nm;

    /* 5. 总力矩合成: 控制器内部"抬头为正"取反转为 DM4310 电机系("负力矩=抬头"), 再叠加重力前馈 */
    float req_motor_nm = -(pi_out_up + ff_up_nm) + st->gravity_motor_nm;
    float app_motor_nm = PitchClamp(req_motor_nm, -PITCH_TORQUE_LIMIT_NM, PITCH_TORQUE_LIMIT_NM);

    /* 总力矩触顶时回退本拍加剧饱和的积分增量 (移植自 GimbalSpeedPiApplyOutputLimit) */
    float excess_up = (-req_motor_nm) - (-app_motor_nm);
    if ((excess_up > 0.0f && applied_delta > 0.0f) ||
        (excess_up < 0.0f && applied_delta < 0.0f))
    {
        st->pi_i      = PitchClamp(st->pi_i - applied_delta, -i_limit, i_limit);
        applied_delta = 0.0f;
        pi_out_up     = PitchClamp(p + st->pi_i, -out_limit, out_limit);
    }

    /* 同步 PIDInstance 观测字段, 方便 EmberProbe 实时看波形与调参 */
    pid->dt           = dt_s;
    pid->Measure      = measure_rad_s;
    pid->Ref          = ref_rad_s;
    pid->Err          = err;
    pid->Pout         = p;
    pid->Iout         = st->pi_i;
    pid->Dout         = 0.0f;
    pid->ITerm        = applied_delta;
    pid->Output       = pi_out_up;
    pid->Last_Measure = pid->Measure;
    pid->Last_Err     = pid->Err;
    pid->Last_ITerm   = pid->ITerm;
    pid->Last_Output  = pid->Output;
    pid->Last_Dout    = 0.0f;

    return app_motor_nm;
}

static void GimbalPitchSpeedReset(void *context, PIDInstance *pid)
{
    PitchCtrlState_s *st = (PitchCtrlState_s *)context;
    memset(st, 0, sizeof(*st));
    pitch_gravity_ff = 0.0f;
    if (pid)
        pid->Iout = 0.0f;
}

void GimbalInit()
{
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源

    // YAW (保持不变)
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 6,
                .Ki = 0.2f,
                .Kd = 0,
                .DeadBand = 0.15f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 100,
                .MaxOut = 300,
            },
            .speed_PID = {
                .Kp = 80,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 3000,
                .MaxOut = 18000,
                .Output_LPF_RC = 0.004f,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,   /* 度, 带圈数累计, 不会在 360° 跳变 */
            .other_speed_feedback_ptr = &yaw_gyro_deg,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,   /* 角度环 = IMU YawTotalAngle(度, 绝对, 带圈数) */
            .speed_feedback_source = OTHER_FEED,   /* 速度环 = yaw_gyro_deg(度/秒, 1kHz 刷新) */
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};

    /* ---------------- PITCH: 达妙 DM-J4310-2EC (纯力矩 MIT, 完整四件套移植自 05_云台) ---------------- */
    DMMotor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,   // sentry 云台板: 达妙接在 CAN2 口
            .tx_id = 0x02,          // 达妙上位机的 CAN ID
            .rx_id = 0x00,          // 达妙上位机的 Master ID
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = PITCH_ANGLE_KP,        /* +25.0 (抬头为正, rad -> rad/s) */
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 1.0f,
                .MaxOut = 500.0f * DEGREE_2_RAD, /* 终端速度上限由 GimbalPitchSpeedUpdate 内的 45°/s + 制动曲线统一接管 */
                .DeadBand = 0.0f,            /* 取消死区, 避免边界抽动 */
            },
            .speed_PID = {
                .Kp = PITCH_SPEED_KP,        /* 0.45 N·m/(rad/s) = 05_云台 900 * 0.0005 */
                .Ki = PITCH_SPEED_KI,        /* 0.25 N·m/rad     = 05_云台 500 * 0.0005 */
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit, /* 无 OutputFilter, 避免相位滞后 */
                .IntegralLimit = PITCH_SPEED_I_LIMIT, /* 0.60 N·m = 05_云台 1200 * 0.0005 */
                .MaxOut = PITCH_TORQUE_LIMIT_NM,      /* 4.0 N·m */
                .Output_LPF_RC = 0.0f,
                .DeadBand = 0.0f,
            },
            .other_angle_feedback_ptr = &pitch_imu_rad,
            .other_speed_feedback_ptr = &pitch_gyro_rad,
            .force_feedforward_ptr    = NULL, /* 重力与速度摩擦前馈均在 GimbalPitchSpeedUpdate 内统一合成 */
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,   /* IMU 绝对角 pitch_imu_rad (1kHz 刷新) */
            .speed_feedback_source = OTHER_FEED,   /* 陀螺角速度 pitch_gyro_rad (1kHz 刷新) */
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = FEEDFORWARD_NONE,
        },
    };

    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&pitch_config);

    if (pitch_motor == NULL)
    {
        LOGERROR("[GIMBAL] DM pitch motor init FAILED! check tx_id<=0x0F / DM_MOTOR_CNT");
    }
    else
    {
        memset(&pitch_ctrl, 0, sizeof(pitch_ctrl));
        DMMotorBindSpeedController(pitch_motor, GimbalPitchSpeedUpdate, GimbalPitchSpeedReset, &pitch_ctrl);
        DMMotorStop(pitch_motor); /* 上电默认卸力, 等待 GimbalTask 收到有效使能模式 */
    }

    GimbalUpdatePitchFeedback();

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/**
 * @brief 就近限位: 把目标角度折到当前 pitch 反馈的 ±π 范围内
 */
static float pitch_ref_near(float target)
{
    float cur = pitch_imu_rad;
    float err = target - cur;

    while (err >  PI) err -= 2.0f * PI;
    while (err < -PI) err += 2.0f * PI;

    return cur + err;
}

/**
 * @brief yaw 陀螺角速度 rad/s → 度/秒 (1kHz)
 */
void GimbalUpdateYawGyroDeg(void)
{
    if (gimba_IMU_data != NULL)
        yaw_gyro_deg = gimba_IMU_data->Gyro[2] * RAD_2_DEGREE;
}

/**
 * @brief 1kHz 刷新 pitch 角度(rad)与陀螺角速度(rad/s), 统一"抬头为正"
 */
void GimbalUpdatePitchFeedback(void)
{
    if (gimba_IMU_data != NULL)
    {
        pitch_imu_rad  = -gimba_IMU_data->Pitch * DEGREE_2_RAD;
        pitch_gyro_rad = -gimba_IMU_data->Gyro[0];
    }
}

/* 机器人云台控制核心任务 (200Hz) */
void GimbalTask()
{
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    /* ---------- 模式双向消抖 ----------
     * 新模式必须连续 3 帧(15ms)保持, 才真正切换 */
    static gimbal_mode_e mode_stable = GIMBAL_ZERO_FORCE;
    static gimbal_mode_e mode_cand   = GIMBAL_ZERO_FORCE;
    static uint8_t cand_cnt = 0;
    gimbal_mode_e mode_in = gimbal_cmd_recv.gimbal_mode;
    gimbal_mode_e mode_eff;

    if (mode_in == mode_stable)
    {
        cand_cnt = 0;
        mode_cand = mode_stable;
    }
    else if (mode_in == mode_cand)
    {
        if (cand_cnt < 3) cand_cnt++;
        if (cand_cnt >= 3)
        {
            mode_stable = mode_cand;
            cand_cnt = 0;
        }
    }
    else
    {
        mode_cand = mode_in;
        cand_cnt = 1;
    }

    mode_eff = mode_stable;

    if (mode_eff == GIMBAL_ZERO_FORCE) dbg_gimbal_stop_frm++;

    /* 电机初始化失败时直接返回, 避免解引用 NULL */
    if (pitch_motor == NULL)
    {
        PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
        return;
    }

    /* 上电第一帧反馈到达后, 把机械中位映射到 total_angle 坐标系(只算一次) */
    if (!yaw_zero_done && yaw_motor->dt > 0.0f)
    {
        float diff_ecd = (float)yaw_motor->measure.ecd - (float)YAW_CHASSIS_ALIGN_ECD;
        if (diff_ecd >  4096.0f) diff_ecd -= 8192.0f;
        if (diff_ecd < -4096.0f) diff_ecd += 8192.0f;

        float mech_now = diff_ecd * ECD_ANGLE_COEF_DJI;
        yaw_mech_off  = gimba_IMU_data->YawTotalAngle - YAW_IMU_DIR * mech_now;
        yaw_zero_done = 1;
    }

    switch (mode_eff)
    {
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DMMotorStop(pitch_motor);
        break;

    case GIMBAL_GYRO_MODE:
    case GIMBAL_FREE_MODE:
        DJIMotorEnable(yaw_motor);
        DMMotorEnable(pitch_motor);
        if (!yaw_zero_done) DJIMotorStop(yaw_motor);

        /* Yaw: 折到离当前多圈 IMU 最近的等价目标 */
        {
            float tgt = gimbal_cmd_recv.yaw + yaw_mech_off;
            float cur = gimba_IMU_data->YawTotalAngle;
            float d   = tgt - cur;

            while (d >  180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;

            DJIMotorSetRef(yaw_motor, cur + d);
        }

        /* Pitch: 直接下发钳位后的绝对目标(rad), 速度斜坡与终端制动由 1kHz 规划器统一完成 */
        {
            const float LIM_UP   = PITCH_MAX_ANGLE * DEGREE_2_RAD;
            const float LIM_DOWN = PITCH_MIN_ANGLE * DEGREE_2_RAD;

            float tgt = pitch_ref_near(gimbal_cmd_recv.pitch * DEGREE_2_RAD);
            tgt = PitchClamp(tgt, LIM_DOWN, LIM_UP);

            DMMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            DMMotorSetRef(pitch_motor, tgt);
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
