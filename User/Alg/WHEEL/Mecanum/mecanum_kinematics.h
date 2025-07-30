/**
 * @file mecanum_kinematics.h
 * @brief 麦克纳姆轮运动学实现
 */

#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

#include "User/Alg/WHEEL/kinematics_base.h"

/**
 * @brief 麦克纳姆轮运动学实现类
 */
class MecanumKinematics : public KinematicsBase
{
  private:
    float wheel_radius_;     ///< 轮子半径 (m)
    float wheel_distance_x_; ///< 底盘中心X轴距离的一半 (m)
    float wheel_distance_y_; ///< 底盘中心Y轴距离的一半 (m)

  public:
    /**
     * @brief 构造函数
     * @param wheel_radius 轮子半径 (m)
     * @param wheel_distance_x 底盘中心X轴距离的一半 (m)
     * @param wheel_distance_y 底盘中心Y轴距离的一半 (m)
     */
    MecanumKinematics(float wheel_radius, float wheel_distance_x, float wheel_distance_y);

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

#endif // MECANUM_KINEMATICS_H