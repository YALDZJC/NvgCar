/**
 * @file kinematics_manager.h
 * @brief 轮式机器人运动学管理器
 */

#ifndef KINEMATICS_MANAGER_H
#define KINEMATICS_MANAGER_H

#include "User/Alg/WHEEL/Diff/differential_kinematics.h"
#include "User/Alg/WHEEL/Mecanum/mecanum_kinematics.h"
#include "User/Alg/WHEEL/Omni/omni_kinematics.h"
#include "User/Alg/WHEEL/Steering/steering_kinematics.h"
#include "User/Alg/WHEEL/kinematics_base.h"

/**
 * @brief 轮式机器人运动学管理器类
 */
class KinematicsManager
{
  private:
    KinematicsBase *current_kinematics_; ///< 指向当前使用的运动学对象的指针

    MecanumKinematics mecanum_kinematics_;           ///< 麦克纳姆轮运动学实例
    OmniKinematics omni_kinematics_;                 ///< 全向轮运动学实例
    SteeringKinematics steering_kinematics_;         ///< 舵轮运动学实例
    DifferentialKinematics differential_kinematics_; ///< 差速二轮运动学实例

  public:
    /**
     * @brief 构造函数
     * @param wheel_radius 轮子半径 (m)
     * @param wheel_distance_x 底盘中心X轴距离的一半 (m)
     * @param wheel_distance_y 底盘中心Y轴距离的一半 (m)
     * @param robot_radius 机器人中心到轮子中心的距离 (m)
     */
    KinematicsManager(float wheel_radius, float wheel_distance_x, float wheel_distance_y, float robot_radius);

    /**
     * @brief 切换轮系类型
     * @param type 轮系类型
     */
    void switchWheelSystem(WheelSystemType type);

    /**
     * @brief 正运动学：从轮子速度计算机器人速度
     * @param wheel_velocities 轮子速度
     * @param robot_velocity 计算得到的机器人速度
     */
    void forwardKinematics(const WheelVelocities &wheel_velocities, RobotVelocity &robot_velocity);

    /**
     * @brief 逆运动学：从机器人速度计算轮子速度
     * @param robot_velocity 机器人速度
     * @param wheel_velocities 计算得到的轮子速度
     */
    void inverseKinematics(const RobotVelocity &robot_velocity, WheelVelocities &wheel_velocities);

    /**
     * @brief 舵轮特有的逆运动学
     * @param robot_velocity 机器人速度
     * @param steer_wheel_velocities 计算得到的舵轮速度
     */
    void inverseKinematicsForSteering(const RobotVelocity &robot_velocity,
                                      SteerWheelVelocities &steer_wheel_velocities);

    /**
     * @brief 获取当前轮系
     * @return 当前轮系对象指针
     */
    KinematicsBase *getCurrentKinematics();
};

#endif // KINEMATICS_MANAGER_H