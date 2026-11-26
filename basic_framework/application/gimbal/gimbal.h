#ifndef GIMBAL_H
#define GIMBAL_H

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

/**
 * @brief 把 yaw 陀螺角速度(rad/s)换算成 度/秒, 供速度内环使用
 * @note  必须在 1kHz 的 MotorControlTask 里调用(不能放 200Hz 的 GimbalTask)
 */
void GimbalUpdateYawGyroDeg(void);

/**
 * @brief 1kHz 刷新 pitch 角度(rad)与陀螺角速度(rad/s), 避免 200Hz 阶梯反馈进入 1kHz 速度环
 */
void GimbalUpdatePitchFeedback(void);

/**
 * @brief 返回当前 IMU 姿态对应的 gimbal_cmd_send.yaw(度, 归一到 ±180), 用于急停/切档零跳变原地接管
 */
float GimbalGetYawCmdCurrent(void);

#endif // GIMBAL_H