#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;

static gimbal_mode_e prev_gimbal_mode = GIMBAL_ZERO_FORCE;
static float yaw_align_off = 0.0f, pitch_align_off = 0.0f;

#define YAW_MECH_CENTER_ECD 927.0f    /* 机械中位时 yaw 电机 ecd(标定值) */
#define PITCH_MECH_CENTER_ECD 6825.0f    /* pitch 机械中位(水平) ecd */

static float   yaw_mech_off  = 0.0f;  /* 机械中位在多圈角坐标系里的位置 */
static uint8_t yaw_zero_done = 0;     /* 只算一次 */
static float   pitch_mech_off = 0.0f;    /* 新加 */
static uint8_t pitch_zero_done = 0;      /* 新加 */

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

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
                .Ki = 0, // 0
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 2000,
                .Output_LPF_RC = 0.5,   // ← 新增:滤掉 speed_aps 的噪声
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
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8, 
                .Ki = 0.05f, 
                .Kd = 0,
                .DeadBand = 1.0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 300,
            },
            .speed_PID = {
                .Kp = 80,  // 40
                .Ki = 0, // 0
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 2000,
                .Output_LPF_RC = 0.3,  // ← 新增
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Roll,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = GM6020,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    /* 上电第一帧反馈到达后,把机械中位映射到 total_angle 坐标系(只算一次) */
    if (!yaw_zero_done && yaw_motor->dt > 0.0f)
    {
        float mech_now = ((float)yaw_motor->measure.ecd - YAW_MECH_CENTER_ECD) * (360.0f / 8192.0f);
        yaw_mech_off  = yaw_motor->measure.total_angle - mech_now;
        yaw_zero_done = 1;
    }

    if (!pitch_zero_done && pitch_motor->dt > 0.0f)
    {
        float mech_now = ((float)pitch_motor->measure.ecd - PITCH_MECH_CENTER_ECD) * (360.0f / 8192.0f);
        pitch_mech_off  = pitch_motor->measure.total_angle - mech_now;
        pitch_zero_done = 1;
    }

    /* 每次从非GYRO切入GYRO时,把当前姿态记为"目标0" */
    if (gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE && prev_gimbal_mode != GIMBAL_GYRO_MODE)
    {
        /* 121-122 行改成:进入时的当前角直接作为"新的0点" */
        pitch_align_off = 0.0f;                             /* pitch:绝对(IMU 0° = 水平) */
    }    
    prev_gimbal_mode = gimbal_cmd_recv.gimbal_mode;

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        if (!yaw_zero_done) {
            DJIMotorStop(yaw_motor);   /* 等第一帧反馈,避免用错误的 0 点 */
        }
        if (!pitch_zero_done) {
            DJIMotorStop(pitch_motor);      /* ← 新加:等第一帧反馈,避免用错的 0 点 */
        }
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw     + yaw_mech_off); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, -(gimbal_cmd_recv.pitch + pitch_mech_off));
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorSetRef(yaw_motor,   gimbal_cmd_recv.yaw   + yaw_mech_off);
        DJIMotorSetRef(pitch_motor, -(gimbal_cmd_recv.pitch + pitch_mech_off));
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