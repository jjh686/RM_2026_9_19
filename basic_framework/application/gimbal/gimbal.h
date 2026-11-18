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

#endif // GIMBAL_H