/**
 * @file mecanum_kinematics.cpp
 * @brief 麦克纳姆轮运动学实现
 */

#include "mecanum_kinematics.h"

/**
 * @brief 构造函数
 * @param wheel_radius 轮子半径 (m)
 * @param wheel_distance_x 底盘中心X轴距离的一半 (m)
 * @param wheel_distance_y 底盘中心Y轴距离的一半 (m)
 */
MecanumKinematics::MecanumKinematics(float wheel_radius, float wheel_distance_x, float wheel_distance_y)
    : wheel_radius_(wheel_radius), wheel_distance_x_(wheel_distance_x), wheel_distance_y_(wheel_distance_y)
{
}

/**
 * @brief 正运动学：从轮子速度计算机器人速度
 * @param wheel_velocities 轮子速度
 * @param robot_velocity 计算得到的机器人速度
 */
void MecanumKinematics::forwardKinematics(const WheelVelocities &wheel_velocities, RobotVelocity &robot_velocity)
{
    // 轮子0: 左前轮
    // 轮子1: 左后轮
    // 轮子2: 右后轮
    // 轮子3: 右前轮
    float w0 = wheel_velocities.wheel[0];
    float w1 = wheel_velocities.wheel[1];
    float w2 = wheel_velocities.wheel[2];
    float w3 = wheel_velocities.wheel[3];

    // 正运动学计算
    float scale = wheel_radius_ / 4.0f;
    robot_velocity.linear_x = scale * (-w0 - w1 + w2 + w3);
    robot_velocity.linear_y = scale * (w0 - w1 - w2 + w3);
    robot_velocity.angular_z = scale / (wheel_distance_x_ + wheel_distance_y_) * (w0 + w1 + w2 + w3);
}

/**
 * @brief 逆运动学：从机器人速度计算轮子速度
 * @param robot_velocity 机器人速度
 * @param wheel_velocities 计算得到的轮子速度
 */
void MecanumKinematics::inverseKinematics(const RobotVelocity &robot_velocity, WheelVelocities &wheel_velocities)
{
    float vx = robot_velocity.linear_x;
    float vy = robot_velocity.linear_y;
    float wz = robot_velocity.angular_z;
    float k = wheel_distance_x_ + wheel_distance_y_;
    float scale = 1.0f / wheel_radius_;

    // 根据逆运动学公式计算轮子速度
    // Motor0 = (-vx + vy + wz * (a + b)) / s
    wheel_velocities.wheel[0] = scale * (-vx + vy + wz * k);

    // Motor1 = (-vx - vy + wz * (a + b)) / s
    wheel_velocities.wheel[1] = scale * (-vx - vy + wz * k);

    // Motor2 = (vx - vy + wz * (a + b)) / s
    wheel_velocities.wheel[2] = scale * (vx - vy + wz * k);

    // Motor3 = (vx + vy + wz * (a + b)) / s
    wheel_velocities.wheel[3] = scale * (vx + vy + wz * k);
}