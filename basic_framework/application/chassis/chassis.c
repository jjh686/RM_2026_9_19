/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "power_control.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
/* 参考值斜坡限速, 防止指令阶跃导致电流打满 */
#define VT_SLEW_APS_PER_S 100000.0f   // 200Hz 下每周期 500 aps


/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD

static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static PIDInstance buffer_PID;             // 用于底盘的缓冲能量PID
static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;                      // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;                  // 底盘速度解算后的临时输出,待进行限幅

static float vt_cmd_lf, vt_cmd_rf, vt_cmd_lb, vt_cmd_rb;  // 斜坡限速后的实际下发值

static float slew_limit(float target, float current, float max_step)
{
    float d = target - current;
    if (d > max_step)  return current + max_step;
    if (d < -max_step) return current - max_step;
    return target;
}

/* ---------------- 调试镜像变量 ----------------
 * 仅为方便调试器/波形观察而存在, 不参与任何控制逻辑.
 * 电机实例在堆上, 路径形如 chassis.c::motor_lf->measure.speed_aps 又长又不便输入,
 * 因此每周期把关心的量复制到这几个全局标量里, 之后直接看 dbg_* 即可.
 * 单位: ref/spd 均为转子度/秒(aps), 与 pid_ref / speed_aps 完全一致.
 * 正式比赛若要清理, 删掉本段与 ChassisTask 末尾的同步语句即可.
 */
volatile float dbg_ref_lf, dbg_spd_lf;                    // 左前 目标/实际
volatile float dbg_ref_rf, dbg_spd_rf;                    // 右前 目标/实际
volatile float dbg_ref_lb, dbg_spd_lb;                    // 左后 目标/实际
volatile float dbg_ref_rb, dbg_spd_rb;                    // 右后 目标/实际
volatile float dbg_out_lf;                                // 左前 速度环输出(电流设定值)

/* 底盘实际速度(m/s, deg/s), 由 EstimateSpeed 逆解算得到 */
volatile float dbg_real_vx, dbg_real_vy, dbg_real_wz;

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 3.0f, 
                .Ki = 0,   
                .Kd = 0,   
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 设置为开环，电机设定值由下面的功率控制设定，不走普通的pid
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    //使用功率控制的电机需要使用PowerControlInit()函数初始化,因为电机的控制方式不同
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = PowerControlInit(&chassis_motor_config);

    referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI

/* Buffer环暂未测试，逻辑是计算期望buffer与实际buffer的差值，转换为冗余的功率，todo：输入给功率控制部分，待完善 */
    PID_Init_Config_s Buffer_pid_conf = {
        .Kp = 0.1f,
        .Ki = 0,
        .Kd = 0,
        .IntegralLimit = 1000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 1000,
    };
    PIDInit(&buffer_PID, &Buffer_pid_conf); // 缓冲能量PID初始化
    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x302, // 超级电容默认接收id
            .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // 超级电容初始化

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

#if defined(CHASSIS_MECANUM)
#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = chassis_vx - chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER;
}

#else /* ---------------- CHASSIS_OMNI: 四轮X型全向轮 ---------------- */

/* 轮轴指向底盘中心, 驱动方向为切向
 * a=半轴距 b=半轮距 r0=轮心到旋转中心距离
 * 350/300 时: OMNI_KX=0.6508  OMNI_KY=0.7593  OMNI_KW=4.023
 */
#define OMNI_A  (WHEEL_BASE / 2.0f)
#define OMNI_B  (TRACK_WIDTH / 2.0f)
#define OMNI_R0 (sqrtf(OMNI_A * OMNI_A + OMNI_B * OMNI_B))
#define OMNI_KX (OMNI_B / OMNI_R0)
#define OMNI_KY (OMNI_A / OMNI_R0)
#define OMNI_KW (OMNI_R0 * DEGREE_2_RAD)

/**
 * @brief 四轮X型全向轮正运动学
 *        期望: vx>0 → [+ - + -] ; vy>0 → [+ + - -] ; wz>0 → [+ + + +]
 */
static void OmniCalculate()
{
    const float w = chassis_cmd_recv.wz * OMNI_KW;

    vt_lf =  OMNI_KX * chassis_vx + OMNI_KY * chassis_vy + w;
    vt_rf = -OMNI_KX * chassis_vx + OMNI_KY * chassis_vy + w;
    vt_lb =  OMNI_KX * chassis_vx - OMNI_KY * chassis_vy + w;
    vt_rb = -OMNI_KX * chassis_vx - OMNI_KY * chassis_vy + w;
}

#endif /* CHASSIS_OMNI */

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 功率限制待添加
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;

    /* 参考值斜坡限速: 防止 ref 阶跃(摇杆瞬断/离线/急停)导致电流打满 */
    const float max_step = VT_SLEW_APS_PER_S * 0.005f;   // 200Hz → 5ms

    vt_cmd_lf = slew_limit(vt_lf, vt_cmd_lf, max_step);
    vt_cmd_rf = slew_limit(vt_rf, vt_cmd_rf, max_step);
    vt_cmd_lb = slew_limit(vt_lb, vt_cmd_lb, max_step);
    vt_cmd_rb = slew_limit(vt_rb, vt_cmd_rb, max_step);

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_cmd_lf);
    DJIMotorSetRef(motor_rf, vt_cmd_rf);
    DJIMotorSetRef(motor_lb, vt_cmd_lb);
    DJIMotorSetRef(motor_rb, vt_cmd_rb);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
#if defined(CHASSIS_OMNI)
    const float lf = motor_lf->measure.speed_aps;
    const float rf = motor_rf->measure.speed_aps;
    const float lb = motor_lb->measure.speed_aps;
    const float rb = motor_rb->measure.speed_aps;

    /* 与 OmniCalculate 严格对应的伪逆 (单位: 转子 aps) */
    const float vx_aps = ( lf - rf + lb - rb) * (1.0f / (4.0f * OMNI_KX));
    const float vy_aps = ( lf + rf - lb - rb) * (1.0f / (4.0f * OMNI_KY));
    const float wz_aps = ( lf + rf + lb + rb) * (1.0f / (4.0f * OMNI_KW));

    /* aps → m/s: /6→rpm转子, /减速比→rpm轮, ×2πR/60→m/s */
    const float aps_2_mps = PI2 * (RADIUS_WHEEL / 1000.0f) /
                            (RPM_2_ANGLE_PER_SEC * REDUCTION_RATIO_WHEEL * 60.0f);

    chassis_feedback_data.real_vx = vx_aps * OMNI_KX * aps_2_mps;
    chassis_feedback_data.real_vy = vy_aps * OMNI_KY * aps_2_mps;
    /* wz 不能复用伪单位 wz_aps, 直接用平均轮速换算: ω = v_wheel / r0 */
    chassis_feedback_data.real_wz = (lf + rf + lb + rb) * 0.25f * aps_2_mps /
                                    (OMNI_R0 / 1000.0f) * RAD_2_DEGREE;
#else
    // 麦轮逆解待补
#endif
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

#if defined(CHASSIS_TEST_FORWARD)
    // 台架测试: 强制固定速度前进, 覆盖遥控器/离线保护的命令
    chassis_cmd_recv.vx = 0;
    chassis_cmd_recv.vy = 0;
    chassis_cmd_recv.wz = 0;
    chassis_cmd_recv.offset_angle = 0;
    chassis_cmd_recv.chassis_mode = CHASSIS_NO_FOLLOW; // 避免落到 ZERO_FORCE 分支停电机
#endif

    // 未接裁判系统时 chassis_power_limit 为 0,会把功率缩放成 0 导致底盘不动
    float power_limit = referee_data->GameRobotState.chassis_power_limit;
    SetPowerLimit(power_limit > 1.0f ? power_limit : 60.0f);

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
        chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle * fabsf(chassis_cmd_recv.offset_angle);
        break;
    case CHASSIS_ROTATE: // 自旋, 幅值固定, 方向来自命令符号
        chassis_cmd_recv.wz = (chassis_cmd_recv.wz < 0.0f) ? -200.0f : 200.0f;
        break;
    default:
        break;
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // 根据控制模式进行正运动学解算,计算底盘输出
    #if defined(CHASSIS_OMNI)
        OmniCalculate();
    #else
        MecanumCalculate();
    #endif

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

    // 同步调试镜像变量(不影响控制, 供波形观察)
    dbg_ref_lf = motor_lf->motor_controller.pid_ref;
    dbg_spd_lf = motor_lf->measure.speed_aps;
    dbg_out_lf = motor_lf->motor_controller.speed_PID.Output;
    dbg_ref_rf = motor_rf->motor_controller.pid_ref;
    dbg_spd_rf = motor_rf->measure.speed_aps;
    dbg_ref_lb = motor_lb->motor_controller.pid_ref;
    dbg_spd_lb = motor_lb->measure.speed_aps;
    dbg_ref_rb = motor_rb->motor_controller.pid_ref;
    dbg_spd_rb = motor_rb->measure.speed_aps;
    dbg_real_vx = chassis_feedback_data.real_vx;
    dbg_real_vy = chassis_feedback_data.real_vy;
    dbg_real_wz = chassis_feedback_data.real_wz;

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}