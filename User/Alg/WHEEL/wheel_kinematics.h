/**
 * @file wheel_kinematics.h
 * @brief 轮式机器人运动学库总头文件
 * @details 包含所有轮式机器人运动学库相关的头文件
 */

#ifndef WHEEL_KINEMATICS_H
#define WHEEL_KINEMATICS_H

// 包含所有基础类型和结构体定义
#include "kinematics_types.h"

// 包含运动学基类
#include "kinematics_base.h"

// 包含各种轮系的运动学实现
#include "differential_kinematics.h"
#include "mecanum_kinematics.h"
#include "omni_kinematics.h"
#include "steering_kinematics.h"

// 包含运动学管理器
#include "kinematics_manager.h"

/**
 * @brief 创建一个轮式机器人运动学管理器的实例
 * @param wheel_radius 轮子半径 (m)
 * @param wheel_distance_x 底盘中心X轴距离的一半 (m)
 * @param wheel_distance_y 底盘中心Y轴距离的一半 (m)
 * @param robot_radius 机器人中心到轮子中心的距离 (m)
 * @return KinematicsManager实例
 */
inline KinematicsManager createKinematicsManager(float wheel_radius, float wheel_distance_x, float wheel_distance_y,
                                                 float robot_radius)
{
    return KinematicsManager(wheel_radius, wheel_distance_x, wheel_distance_y, robot_radius);
}

#endif // WHEEL_KINEMATICS_H