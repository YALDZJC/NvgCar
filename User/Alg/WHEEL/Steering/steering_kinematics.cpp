/**
 * @file steering_kinematics.cpp
 * @brief 舵轮运动学实现
 */

#include "steering_kinematics.h"
#include <cmath>

// 定义常量以避免重复计算
constexpr float PI = 3.14159265359f;
constexpr float COS_45 = 0.70710678118f; // cos(45°)
constexpr float SIN_45 = 0.70710678118f; // sin(45°)

/**
 * @brief 构造函数
 * @param wheel_radius 轮子半径 (m)
 * @param robot_radius 机器人中心到轮子中心的距离 (m)
 */
SteeringKinematics::SteeringKinematics(float wheel_radius, float robot_radius)
    : wheel_radius_(wheel_radius), robot_radius_(robot_radius)
{
}

/**
 * @brief 正运动学：从普通轮子速度计算机器人速度
 * @param wheel_velocities 轮子速度
 * @param robot_velocity 计算得到的机器人速度
 *
 * 注意：此方法是为了满足基类接口，实际上舵轮应使用特有的正运动学方法
 */
void SteeringKinematics::forwardKinematics(const WheelVelocities &wheel_velocities, RobotVelocity &robot_velocity)
{
    // 简单实现，将驱动速度设为wheel_velocities，方向假设为45度角
    SteerWheelVelocities steer_wheel_velocities;
    for (int i = 0; i < 4; ++i)
    {
        steer_wheel_velocities.drive[i] = wheel_velocities.wheel[i];
        steer_wheel_velocities.steer[i] = PI / 4.0f; // 45度角
    }

    // 调用特有的正运动学
    forwardKinematics(steer_wheel_velocities, robot_velocity);
}

/**
 * @brief 逆运动学：从机器人速度计算轮子速度
 * @param robot_velocity 机器人速度
 * @param wheel_velocities 计算得到的轮子速度
 *
 * 注意：此方法是为了满足基类接口，实际上舵轮应使用特有的逆运动学方法
 */
void SteeringKinematics::inverseKinematics(const RobotVelocity &robot_velocity, WheelVelocities &wheel_velocities)
{
    // 计算舵轮速度
    SteerWheelVelocities steer_wheel_velocities;
    inverseKinematics(robot_velocity, steer_wheel_velocities);

    // 将驱动速度复制到wheel_velocities
    for (int i = 0; i < 4; ++i)
    {
        wheel_velocities.wheel[i] = steer_wheel_velocities.drive[i];
    }
}

/**
 * @brief 舵轮特有的正运动学
 * @param steer_wheel_velocities 舵轮速度
 * @param robot_velocity 计算得到的机器人速度
 */
void SteeringKinematics::forwardKinematics(const SteerWheelVelocities &steer_wheel_velocities,
                                           RobotVelocity &robot_velocity)
{
    // 由于舵轮正向解算较为复杂，此处提供简化实现
    // 实际应用中需要根据舵轮的角度和速度综合计算
    robot_velocity.linear_x = 0.0f;
    robot_velocity.linear_y = 0.0f;
    robot_velocity.angular_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        float drive_speed = steer_wheel_velocities.drive[i] * wheel_radius_;
        float steer_angle = steer_wheel_velocities.steer[i];

        robot_velocity.linear_x += drive_speed * cosf(steer_angle);
        robot_velocity.linear_y += drive_speed * sinf(steer_angle);
    }

    robot_velocity.linear_x /= 4.0f;
    robot_velocity.linear_y /= 4.0f;

    // 角速度计算需要考虑每个轮子的贡献
    // 此实现为简化版本
}

/**
 * @brief 舵轮特有的逆运动学
 * @param robot_velocity 机器人速度
 * @param steer_wheel_velocities 计算得到的舵轮速度
 */
void SteeringKinematics::inverseKinematics(const RobotVelocity &robot_velocity,
                                           SteerWheelVelocities &steer_wheel_velocities)
{
    float vx = robot_velocity.linear_x;
    float vy = robot_velocity.linear_y;
    float wz = robot_velocity.angular_z;

    // 舵轮位置，假设四个轮子在机器人四角，45°角分布
    // 轮子0: 左前(45°)
    // 轮子1: 右前(135°)
    // 轮子2: 右后(225°)
    // 轮子3: 左后(315°)

    // 计算各个轮子位置处的速度向量
    float v0x = vx - wz * robot_radius_ * SIN_45;
    float v0y = vy + wz * robot_radius_ * COS_45;

    float v1x = vx - wz * robot_radius_ * SIN_45;
    float v1y = vy - wz * robot_radius_ * COS_45;

    float v2x = vx + wz * robot_radius_ * SIN_45;
    float v2y = vy - wz * robot_radius_ * COS_45;

    float v3x = vx + wz * robot_radius_ * SIN_45;
    float v3y = vy + wz * robot_radius_ * COS_45;

    // 计算转向角度 (弧度)
    steer_wheel_velocities.steer[0] = atan2f(v0y, v0x);
    steer_wheel_velocities.steer[1] = atan2f(v1y, v1x);
    steer_wheel_velocities.steer[2] = atan2f(v2y, v2x);
    steer_wheel_velocities.steer[3] = atan2f(v3y, v3x);

    // 计算轮子驱动速度 (rad/s)
    float inv_wheel_radius = 1.0f / wheel_radius_;
    steer_wheel_velocities.drive[0] = sqrtf(v0x * v0x + v0y * v0y) * inv_wheel_radius;
    steer_wheel_velocities.drive[1] = sqrtf(v1x * v1x + v1y * v1y) * inv_wheel_radius;
    steer_wheel_velocities.drive[2] = sqrtf(v2x * v2x + v2y * v2y) * inv_wheel_radius;
    steer_wheel_velocities.drive[3] = sqrtf(v3x * v3x + v3y * v3y) * inv_wheel_radius;
}