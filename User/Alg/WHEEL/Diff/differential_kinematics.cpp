/**
 * @file differential_kinematics.cpp
 * @brief 差速二轮运动学实现
 */

#include "differential_kinematics.h"
#include <cmath>

/**
 * @brief 构造函数
 * @param wheel_radius 轮子半径 (m)
 * @param wheel_distance_x 底盘中心X轴距离的一半 (m)
 * @param wheel_distance_y 底盘中心Y轴距离的一半 (m)
 */
DifferentialKinematics::DifferentialKinematics(float wheel_radius, float wheel_distance_x, float wheel_distance_y)
    : wheel_radius_(wheel_radius), wheel_distance_x_(wheel_distance_x), wheel_distance_y_(wheel_distance_y)
{
}

/**
 * @brief 正运动学：从轮子速度计算机器人速度
 * @param wheel_velocities 轮子速度
 * @param robot_velocity 计算得到的机器人速度
 */
void DifferentialKinematics::forwardKinematics(const WheelVelocities &wheel_velocities, RobotVelocity &robot_velocity)
{
    // 差速驱动只使用两个轮子（左右）
    float w_left = (wheel_velocities.wheel[0] + wheel_velocities.wheel[3]) / 2.0f;  // 左侧轮子平均角速度
    float w_right = (wheel_velocities.wheel[1] + wheel_velocities.wheel[2]) / 2.0f; // 右侧轮子平均角速度

    // 计算线速度和角速度
    float v_left = wheel_radius_ * w_left;        // 左侧线速度
    float v_right = wheel_radius_ * w_right;      // 右侧线速度
    float track_width = 2.0f * wheel_distance_y_; // 轮距

    // 差速运动学正解
    robot_velocity.linear_x = (v_right + v_left) / 2.0f;
    robot_velocity.linear_y = 0.0f; // 差速驱动无法实现横向移动
    robot_velocity.angular_z = (v_right - v_left) / track_width;
}

/**
 * @brief 逆运动学：从机器人速度计算轮子速度
 * @param robot_velocity 机器人速度
 * @param wheel_velocities 计算得到的轮子速度
 */
void DifferentialKinematics::inverseKinematics(const RobotVelocity &robot_velocity, WheelVelocities &wheel_velocities)
{
    float vx = robot_velocity.linear_x;
    float wz = robot_velocity.angular_z;
    float track_width = 2.0f * wheel_distance_y_; // 轮距

    // 计算左右轮线速度
    float v_left = vx - wz * track_width / 2.0f;
    float v_right = vx + wz * track_width / 2.0f;

    // 转换为角速度
    float w_left = v_left / wheel_radius_;
    float w_right = v_right / wheel_radius_;

    // 设置轮子速度，假设0,3是右侧轮，1,2是左侧轮
    wheel_velocities.wheel[0] = -w_left;
    wheel_velocities.wheel[3] = -w_left;
    wheel_velocities.wheel[1] = w_right;
    wheel_velocities.wheel[2] = w_right;
}