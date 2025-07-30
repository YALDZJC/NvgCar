/**
 * @file omni_kinematics.h
 * @brief 全向轮运动学实现
 */

#ifndef OMNI_KINEMATICS_H
#define OMNI_KINEMATICS_H

#include "User/Alg/WHEEL/kinematics_base.h"

/**
 * @brief 全向轮运动学实现类
 */
class OmniKinematics : public KinematicsBase
{
  private:
    float wheel_radius_; ///< 轮子半径 (m)
    float robot_radius_; ///< 机器人中心到轮子中心的距离 (m)

  public:
    /**
     * @brief 构造函数
     * @param wheel_radius 轮子半径 (m)
     * @param robot_radius 机器人中心到轮子中心的距离 (m)
     */
    OmniKinematics(float wheel_radius, float robot_radius);

    /**
     * @brief 正运动学：从轮子速度计算机器人速度
     * @param wheel_velocities 轮子速度
     * @param robot_velocity 计算得到的机器人速度
     */
    void forwardKinematics(const WheelVelocities &wheel_velocities, RobotVelocity &robot_velocity) override;

    /**
     * @brief 逆运动学：从机器人速度计算轮子速度
     * @param robot_velocity 机器人速度
     * @param wheel_velocities 计算得到的轮子速度
     */
    void inverseKinematics(const RobotVelocity &robot_velocity, WheelVelocities &wheel_velocities) override;
};

#endif // OMNI_KINEMATICS_H