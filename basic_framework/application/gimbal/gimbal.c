#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"      // ← 新增: DMMotorInstance / DMMotorInit / DMMotorSetRef
#include "arm_math.h"     // arm_sin_f32 (dm_test_mode == 3 用)
#include "bsp_dwt.h"      // DWT_GetTimeline_s (同上)
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor;      // yaw 保持 GM6020
static DMMotorInstance *pitch_motor;     // pitch 改成达妙

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
/* IMU 单位转换: 度 → 弧度。dmmotor 的 PID 全部按弧度计算 */
static float pitch_imu_rad  = 0.0f;   /* = gimba_IMU_data->Pitch × DEGREE_2_RAD */
static float pitch_gyro_rad = 0.0f;   /* = gimba_IMU_data->Gyro[1] × DEGREE_2_RAD */
static float   pitch_zero_off  = 0.0f;    /* pitch 上电位置基准(弧度) */
static uint8_t pitch_zero_done = 0;
/* 重力前馈: τ_g = K_g × cos(pitch), K_g 实测 0.3~1.2 N·m */
#define PITCH_GRAVITY_K   0.5f          /* 水平时的重力矩(N·m), 先给 0.5 再实测 */
static float pitch_gravity_ff = 0.0f;


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
                .Kp = 8, // 8
                .Ki = 0.2f,
                .Kd = 0,
                .DeadBand = 0.15f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 300.0f,
            },
            .speed_PID = {
                .Kp = 80,  // 80
                .Ki = 0, 
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 2000,
                .Output_LPF_RC = 0.5f
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    /* ---------------- PITCH: 达妙 DM-J4310-2EC (MIT 单发力矩, C 板自己算 PID) ----------------
     * DMMotorInit() 走 NORMAL_MODE: 发送帧里 p_des=0, v_des=0, Kp=0, Kd=0, 只发 t_ff
     * 即"电机只出力矩, 所有闭环都在 C 板上算"
     */
    DMMotor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,  
            .tx_id = 0x02,          // 达妙上位机的 CAN ID (已避开 C620 的 0x01)
            .rx_id = 0x00,          // 达妙上位机的 Master ID
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8.0f,             // 1 rad 角度误差 → 8 rad/s 速度目标
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3.0f,
                .MaxOut = 15.0f,        // 最大目标角速度 15 rad/s
            },
            .speed_PID = {
                .Kp = 0.8f,             // 单位 N·m / (rad/s)
                .Ki = 0.05f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 0.5f,
                .MaxOut = 3.0f,         // 限 3 N·m (4310 峰值 10 N·m)
            },
            /* ← 新增这两个指针, 否则 OTHER_FEED 会解引用 NULL */
            .other_angle_feedback_ptr = &pitch_imu_rad,
            .other_speed_feedback_ptr = &pitch_gyro_rad,
            /* ← 新增: 力矩前馈指针 */
            .force_feedforward_ptr = &pitch_gravity_ff,

        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED, // 外环采用 IMU 反馈
            .speed_feedback_source = MOTOR_FEED, // 内环采用达妙电机自身速度反馈
            .outer_loop_type = ANGLE_LOOP,         // 默认先测角度环
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = FORCE_FEEDFORWARD,   /* ← 新增 */
        },
    };

    yaw_motor   = DJIMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&pitch_config);

    if (pitch_motor == NULL)
        LOGERROR("[GIMBAL] DM pitch motor init FAILED! check tx_id<=0x0F / DM_MOTOR_CNT");

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/**
 * @brief 就近限位(弧度版): 把目标折到当前 IMU 反馈的 ±π 范围内
 */
static float pitch_ref_near_rad(float target_rad)
{
    float err = target_rad - pitch_imu_rad;

    while (err >  3.14159265f) err -= 6.2831853f;
    while (err < -3.14159265f) err += 6.2831853f;

    return pitch_imu_rad + err;
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    /* IMU 度 → 弧度, 供 dmmotor 的 PID 使用 */
    pitch_imu_rad  = gimba_IMU_data->Pitch   * DEGREE_2_RAD;
    pitch_gyro_rad = gimba_IMU_data->Gyro[1] * DEGREE_2_RAD;
    /* 重力前馈: 枪管重心偏前, 重力矩 = K_g × cos(pitch) */
    pitch_gravity_ff = PITCH_GRAVITY_K * arm_cos_f32(pitch_imu_rad);

    if (pitch_motor == NULL)
    {
        PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
        return;
    }

    /* 等电机反馈真正到达(系统已跑起来)再标定, 避免 IMU 未收敛时标错 */
    if (!pitch_zero_done && pitch_motor->rx_init_flag == 1)
    {
        pitch_zero_off  = pitch_imu_rad;
        pitch_zero_done = 1;
    }

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DMMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_motor);
        DMMotorEnable(pitch_motor);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DMMotorSetRef(pitch_motor,pitch_ref_near_rad(gimbal_cmd_recv.pitch * DEGREE_2_RAD + pitch_zero_off));
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_motor);
        DMMotorEnable(pitch_motor);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DMMotorSetRef(pitch_motor,pitch_ref_near_rad(gimbal_cmd_recv.pitch * DEGREE_2_RAD + pitch_zero_off));
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}