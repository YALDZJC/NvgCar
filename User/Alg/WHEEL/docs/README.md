# 轮式机器人运动学库

本库实现了四种常见轮式机器人的运动学解算，支持实时切换不同轮系。

## 支持的轮系

- 麦克纳姆轮 (Mecanum)
- 全向轮 (Omni)
- 舵轮 (Steering)
- 差速二轮 (Differential)

## 使用方法

### 基础使用

```cpp
#include "wheel_kinematics.h"

// 创建运动学管理器
float wheel_radius = 0.05f;       // 轮子半径 (m)
float wheel_distance_x = 0.2f;    // 底盘中心X轴距离的一半 (m)
float wheel_distance_y = 0.2f;    // 底盘中心Y轴距离的一半 (m)
float robot_radius = 0.25f;       // 机器人中心到轮子中心的距离 (m)

KinematicsManager kinematics_manager(wheel_radius, wheel_distance_x, wheel_distance_y, robot_radius);

// 定义期望的机器人速度
RobotVelocity desired_velocity = {
    .linear_x = 0.5f,   // 前进 0.5 m/s
    .linear_y = 0.0f,   // 不横移
    .angular_z = 0.2f   // 旋转 0.2 rad/s
};

// 计算轮子速度
WheelVelocities wheel_velocities;
kinematics_manager.inverseKinematics(desired_velocity, wheel_velocities);

// 应用到电机
for (int i = 0; i < 4; i++) {
    // 设置电机速度
    motor_controller.setSpeed(i, wheel_velocities.wheel[i]);
}
```

### 切换轮系

```cpp
// 切换到全向轮
kinematics_manager.switchWheelSystem(WheelSystemType::OMNI);

// 使用全向轮运动学
kinematics_manager.inverseKinematics(desired_velocity, wheel_velocities);
```

### 舵轮特殊接口

舵轮系统有两组电机（驱动和转向），需要使用特殊接口：

```cpp
// 切换到舵轮
kinematics_manager.switchWheelSystem(WheelSystemType::STEERING);

// 计算舵轮速度
SteerWheelVelocities steer_wheel_velocities;
kinematics_manager.inverseKinematicsForSteering(desired_velocity, steer_wheel_velocities);

// 应用到电机
for (int i = 0; i < 4; i++) {
    // 设置驱动电机速度
    motor_controller.setDriveSpeed(i, steer_wheel_velocities.drive[i]);
    // 设置转向电机角度
    motor_controller.setSteerAngle(i, steer_wheel_velocities.steer[i]);
}
```

### 正运动学（从轮子速度计算机器人速度）

```cpp
// 已知轮子速度
WheelVelocities wheel_velocities;
// ... 设置wheel_velocities的值 ...

// 计算机器人速度
RobotVelocity robot_velocity;
kinematics_manager.forwardKinematics(wheel_velocities, robot_velocity);

// 使用计算得到的机器人速度
float vx = robot_velocity.linear_x;
float vy = robot_velocity.linear_y;
float wz = robot_velocity.angular_z;
```

## 文件结构

- `kinematics_types.h` - 基础数据类型和结构体定义
- `kinematics_base.h` - 运动学基类
- `mecanum_kinematics.h/cpp` - 麦克纳姆轮运动学实现
- `omni_kinematics.h/cpp` - 全向轮运动学实现
- `steering_kinematics.h/cpp` - 舵轮运动学实现
- `differential_kinematics.h/cpp` - 差速二轮运动学实现
- `kinematics_manager.h/cpp` - 运动学管理器
- `wheel_kinematics.h` - 总头文件
- `kinematics_example.cpp` - 使用示例

## 注意事项

1. 所有角度计算使用弧度制
2. 所有物理单位使用米和秒
3. 轮子速度为角速度，单位为rad/s
4. 机器人中心速度为线速度，单位为m/s 