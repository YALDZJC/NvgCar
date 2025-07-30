/**
 * @file omni_kinematics.cpp
 * @brief 全向轮运动学实现
 */

#include "omni_kinematics.h"
#include <cmath>

// 定义常量以避免重复计算
constexpr float SQRT_2 = 1.41421356237f;
constexpr float SQRT_2_DIV_2 = 0.70710678118f; // sqrt(2)/2

/**
 * @brief 构造函数
 * @param wheel_radius 轮子半径 (m)
 * @param robot_radius 机器人中心到轮子中心的距离 (m)
 */
OmniKinematics::OmniKinematics(float wheel_radius, float robot_radius)
    : wheel_radius_(wheel_radius), robot_radius_(robot_radius)
{
}

/**
 * @brief 正运动学：从轮子速度计算机器人速度
 * @param wheel_velocities 轮子速度
 * @param robot_velocity 计算得到的机器人速度
 */
void OmniKinematics::forwardKinematics(const WheelVelocities &wheel_velocities, RobotVelocity &robot_velocity)
{
    // 全向轮排列假设：
    // 轮子0: 左前(45度)
    // 轮子1: 左后(135度)
    // 轮子2: 右后(225度)
    // 轮子3: 右前(315度)
    float w0 = wheel_velocities.wheel[0];
    float w1 = wheel_velocities.wheel[1];
    float w2 = wheel_velocities.wheel[2];
    float w3 = wheel_velocities.wheel[3];

    float scale = wheel_radius_ / 4.0f;

    // 正运动学计算
    robot_velocity.linear_x = scale * (SQRT_2 * (-w0 - w1 + w2 + w3));
    robot_velocity.linear_y = scale * (SQRT_2 * (w0 - w1 - w2 + w3));
    robot_velocity.angular_z = scale * (w0 + w1 + w2 + w3) / robot_radius_;
}

/**
 * @brief 逆运动学：从机器人速度计算轮子速度
 * @param robot_velocity 机器人速度
 * @param wheel_velocities 计算得到的轮子速度
 */
void OmniKinematics::inverseKinematics(const RobotVelocity &robot_velocity, WheelVelocities &wheel_velocities)
{
    float vx = robot_velocity.linear_x;
    float vy = robot_velocity.linear_y;
    float wz = robot_velocity.angular_z;
    float scale = 1.0f / wheel_radius_;

    // 根据逆运动学公式计算轮子速度
    // Motor0 = (-vx*sqrt(2)/2 + vy*sqrt(2)/2 + wz*r) / s
    wheel_velocities.wheel[0] = scale * (-SQRT_2_DIV_2 * vx + SQRT_2_DIV_2 * vy + wz * robot_radius_);

    // Motor1 = (-vx*sqrt(2)/2 - vy*sqrt(2)/2 + wz*r) / s
    wheel_velocities.wheel[1] = scale * (-SQRT_2_DIV_2 * vx - SQRT_2_DIV_2 * vy + wz * robot_radius_);

    // Motor2 = (vx*sqrt(2)/2 - vy*sqrt(2)/2 + wz*r) / s
    wheel_velocities.wheel[2] = scale * (SQRT_2_DIV_2 * vx - SQRT_2_DIV_2 * vy + wz * robot_radius_);

    // Motor3 = (vx*sqrt(2)/2 + vy*sqrt(2)/2 + wz*r) / s
    wheel_velocities.wheel[3] = scale * (SQRT_2_DIV_2 * vx + SQRT_2_DIV_2 * vy + wz * robot_radius_);
}