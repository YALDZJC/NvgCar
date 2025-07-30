/**
 * @file kinematics_types.h
 * @brief 轮式机器人运动学库的基础数据类型
 */

#ifndef KINEMATICS_TYPES_H
#define KINEMATICS_TYPES_H

/**
 * @brief 轮系类型枚举
 */
enum class WheelSystemType
{
    MECANUM,     ///< 麦克纳姆轮
    OMNI,        ///< 全向轮
    STEERING,    ///< 舵轮
    DIFFERENTIAL ///< 差速二轮
};

/**
 * @brief 机器人速度结构体
 */
struct RobotVelocity
{
    float linear_x;  ///< X轴线速度 (m/s)
    float linear_y;  ///< Y轴线速度 (m/s)
    float angular_z; ///< Z轴角速度 (rad/s)
};

/**
 * @brief 轮子速度结构体
 */
struct WheelVelocities
{
    float wheel[4]; ///< 4个轮子的角速度 (rad/s)
};

/**
 * @brief 舵轮速度结构体
 */
struct SteerWheelVelocities
{
    float drive[4]; ///< 驱动电机速度 (rad/s)
    float steer[4]; ///< 转向电机角度 (rad)
};

#endif // KINEMATICS_TYPES_H