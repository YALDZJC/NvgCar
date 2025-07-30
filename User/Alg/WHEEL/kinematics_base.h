/**
 * @file kinematics_base.h
 * @brief 轮式机器人运动学基类
 */

#ifndef KINEMATICS_BASE_H
#define KINEMATICS_BASE_H

#include "kinematics_types.h"

/**
 * @brief 轮式机器人运动学基类
 */
class KinematicsBase
{
  public:
    /**
     * @brief 虚析构函数
     */
    virtual ~KinematicsBase() = default;

    /**
     * @brief 正运动学：从轮子速度计算机器人速度
     * @param wheel_velocities 轮子速度
     * @param robot_velocity 计算得到的机器人速度
     */
    virtual void forwardKinematics(const WheelVelocities &wheel_velocities, RobotVelocity &robot_velocity) = 0;

    /**
     * @brief 逆运动学：从机器人速度计算轮子速度
     * @param robot_velocity 机器人速度
     * @param wheel_velocities 计算得到的轮子速度
     */
    virtual void inverseKinematics(const RobotVelocity &robot_velocity, WheelVelocities &wheel_velocities) = 0;
};

#endif // KINEMATICS_BASE_H