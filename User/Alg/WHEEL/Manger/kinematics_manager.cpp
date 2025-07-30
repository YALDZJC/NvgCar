/**
 * @file kinematics_manager.cpp
 * @brief 轮式机器人运动学管理器实现
 */

#include "kinematics_manager.h"

/**
 * @brief 构造函数
 * @param wheel_radius 轮子半径 (m)
 * @param wheel_distance_x 底盘中心X轴距离的一半 (m)
 * @param wheel_distance_y 底盘中心Y轴距离的一半 (m)
 * @param robot_radius 机器人中心到轮子中心的距离 (m)
 */
KinematicsManager::KinematicsManager(float wheel_radius, float wheel_distance_x, float wheel_distance_y,
                                     float robot_radius)
    : current_kinematics_(nullptr), // 先初始化为nullptr
      mecanum_kinematics_(wheel_radius, wheel_distance_x, wheel_distance_y),
      omni_kinematics_(wheel_radius, robot_radius), steering_kinematics_(wheel_radius, robot_radius),
      differential_kinematics_(wheel_radius, wheel_distance_x, wheel_distance_y)
{
    // 设置默认使用麦克纳姆轮
    current_kinematics_ = &mecanum_kinematics_;
}

/**
 * @brief 切换轮系类型
 * @param type 轮系类型
 */
void KinematicsManager::switchWheelSystem(WheelSystemType type)
{
    switch (type)
    {
    case WheelSystemType::MECANUM:
        current_kinematics_ = &mecanum_kinematics_;
        break;
    case WheelSystemType::OMNI:
        current_kinematics_ = &omni_kinematics_;
        break;
    case WheelSystemType::STEERING:
        current_kinematics_ = &steering_kinematics_;
        break;
    case WheelSystemType::DIFFERENTIAL:
        current_kinematics_ = &differential_kinematics_;
        break;
    }
}

/**
 * @brief 正运动学：从轮子速度计算机器人速度
 * @param wheel_velocities 轮子速度
 * @param robot_velocity 计算得到的机器人速度
 */
void KinematicsManager::forwardKinematics(const WheelVelocities &wheel_velocities, RobotVelocity &robot_velocity)
{
    current_kinematics_->forwardKinematics(wheel_velocities, robot_velocity);
}

/**
 * @brief 逆运动学：从机器人速度计算轮子速度
 * @param robot_velocity 机器人速度
 * @param wheel_velocities 计算得到的轮子速度
 */
void KinematicsManager::inverseKinematics(const RobotVelocity &robot_velocity, WheelVelocities &wheel_velocities)
{
    current_kinematics_->inverseKinematics(robot_velocity, wheel_velocities);
}

/**
 * @brief 舵轮特有的逆运动学
 * @param robot_velocity 机器人速度
 * @param steer_wheel_velocities 计算得到的舵轮速度
 */
void KinematicsManager::inverseKinematicsForSteering(const RobotVelocity &robot_velocity,
                                                     SteerWheelVelocities &steer_wheel_velocities)
{
    // 确保当前使用的是舵轮
    if (current_kinematics_ == &steering_kinematics_)
    {
        steering_kinematics_.inverseKinematics(robot_velocity, steer_wheel_velocities);
    }
    // 如果不是舵轮，不执行任何操作
}

/**
 * @brief 获取当前轮系
 * @return 当前轮系对象指针
 */
KinematicsBase *KinematicsManager::getCurrentKinematics()
{
    return current_kinematics_;
}