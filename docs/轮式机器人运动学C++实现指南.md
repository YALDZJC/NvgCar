# 轮式机器人运动学C++实现指南

## 需求描述

请实现一个用于嵌入式平台的轮式机器人运动学库，支持以下类型的轮式机器人：
- 麦克纳姆轮
- 全向轮
- 舵轮
- 差速二轮

## 技术要求

1. 使用现代C++（C++11/14/17），但避免使用可能在嵌入式平台上性能开销大的特性
2. 代码应具有良好的封装性和易用性，如有必要，可以使用设计模式
3. 提供清晰的接口和文档
4. 考虑嵌入式环境的资源限制（内存、计算能力）
5. 避免动态内存分配，优先使用栈内存
6. 使用固定大小的容器而非STL动态容器
7. 提供适当的错误处理机制
8. 支持运行时快速切换不同轮系

## 轮系切换设计

为了支持运行时快速切换不同轮系，建议采用策略模式结合工厂模式：

```cpp
// 轮系类型枚举
enum class WheelSystemType {
    MECANUM,
    OMNI,
    STEERING,
    DIFFERENTIAL
};

// 轮系管理器
class KinematicsManager {
private:
    KinematicsBase* current_kinematics_;
    MecanumKinematics mecanum_kinematics_;
    OmniKinematics omni_kinematics_;
    SteeringKinematics steering_kinematics_;
    DifferentialKinematics differential_kinematics_;
    
public:
    KinematicsManager(
        float wheel_radius,
        float wheel_distance_x,
        float wheel_distance_y,
        float robot_radius
    ) : 
        mecanum_kinematics_(wheel_radius, wheel_distance_x, wheel_distance_y),
        omni_kinematics_(wheel_radius, robot_radius),
        steering_kinematics_(wheel_radius, robot_radius),
        differential_kinematics_(wheel_radius, wheel_distance_x, wheel_distance_y),
        current_kinematics_(&mecanum_kinematics_) // 默认使用麦克纳姆轮
    {}
    
    // 切换轮系类型
    void switchWheelSystem(WheelSystemType type) {
        switch (type) {
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
    
    // 代理方法，转发到当前选择的轮系实现
    void forwardKinematics(const WheelVelocities& wheel_velocities, RobotVelocity& robot_velocity) {
        current_kinematics_->forwardKinematics(wheel_velocities, robot_velocity);
    }
    
    void inverseKinematics(const RobotVelocity& robot_velocity, WheelVelocities& wheel_velocities) {
        current_kinematics_->inverseKinematics(robot_velocity, wheel_velocities);
    }
    
    // 获取当前轮系类型
    KinematicsBase* getCurrentKinematics() {
        return current_kinematics_;
    }
};
```

## 类设计建议

创建一个基础的`KinematicsBase`类，然后为每种轮式系统派生专门的类：

```cpp
class KinematicsBase {
public:
    // 虚析构函数
    virtual ~KinematicsBase() = default;
    
    // 正运动学：从轮子速度计算机器人速度
    virtual void forwardKinematics(const WheelVelocities& wheel_velocities, RobotVelocity& robot_velocity) = 0;
    
    // 逆运动学：从机器人速度计算轮子速度
    virtual void inverseKinematics(const RobotVelocity& robot_velocity, WheelVelocities& wheel_velocities) = 0;
};
```

## 数据结构建议

为机器人速度和轮子速度创建简单的数据结构：

```cpp
struct RobotVelocity {
    float linear_x;  // X轴线速度 (m/s)
    float linear_y;  // Y轴线速度 (m/s)
    float angular_z; // Z轴角速度 (rad/s)
};

struct WheelVelocities {
    // 根据不同轮式系统设计不同的结构
    // 例如麦克纳姆轮和全向轮：
    float wheel[4];  // 4个轮子的角速度
    
    // 舵轮可能需要：
    // float steer[4];  // 转向电机角度
};
```

## 麦克纳姆轮实现

```cpp
class MecanumKinematics : public KinematicsBase {
private:
    float wheel_radius_;      // 轮子半径 (m)
    float wheel_distance_x_;  // 轮距的一半 (m)
    float wheel_distance_y_;  // 轴距的一半 (m)
    
public:
    MecanumKinematics(float wheel_radius, float wheel_distance_x, float wheel_distance_y);
    
    void forwardKinematics(const WheelVelocities& wheel_velocities, RobotVelocity& robot_velocity) override;
    void inverseKinematics(const RobotVelocity& robot_velocity, WheelVelocities& wheel_velocities) override;
};
```

## 全向轮实现

```cpp
class OmniKinematics : public KinematicsBase {
private:
    float wheel_radius_;  // 轮子半径 (m)
    float robot_radius_;  // 机器人中心到轮子中心的距离 (m)
    
public:
    OmniKinematics(float wheel_radius, float robot_radius);
    
    void forwardKinematics(const WheelVelocities& wheel_velocities, RobotVelocity& robot_velocity) override;
    void inverseKinematics(const RobotVelocity& robot_velocity, WheelVelocities& wheel_velocities) override;
};
```

## 舵轮实现

```cpp
class SteeringKinematics : public KinematicsBase {
private:
    float wheel_radius_;  // 轮子半径 (m)
    float robot_radius_;  // 机器人中心到轮子中心的距离 (m)
    
    // 舵轮特有数据结构
    struct SteerWheelVelocities {
        float drive[4];  // 驱动电机速度
        float steer[4];  // 转向电机角度 (rad)
    };
    
public:
    SteeringKinematics(float wheel_radius, float robot_radius);
    
    // 舵轮特有的接口
    void forwardKinematics(const SteerWheelVelocities& wheel_velocities, RobotVelocity& robot_velocity);
    void inverseKinematics(const RobotVelocity& robot_velocity, SteerWheelVelocities& wheel_velocities);
    
    // 基类接口实现（可能需要适配）
    void forwardKinematics(const WheelVelocities& wheel_velocities, RobotVelocity& robot_velocity) override;
    void inverseKinematics(const RobotVelocity& robot_velocity, WheelVelocities& wheel_velocities) override;
};
```

## 差速二轮实现

```cpp
class DifferentialKinematics : public KinematicsBase {
private:
    float wheel_radius_;      // 轮子半径 (m)
    float wheel_distance_x_;  // 轮距的一半 (m)
    float wheel_distance_y_;  // 轴距的一半 (m)
    
public:
    DifferentialKinematics(float wheel_radius, float wheel_distance_x, float wheel_distance_y);
    
    void forwardKinematics(const WheelVelocities& wheel_velocities, RobotVelocity& robot_velocity) override;
    void inverseKinematics(const RobotVelocity& robot_velocity, WheelVelocities& wheel_velocities) override;
};
```

## 使用示例

```cpp
// 创建运动学管理器
KinematicsManager kinematics_manager(0.05f, 0.2f, 0.2f, 0.25f);

// 默认使用麦克纳姆轮
RobotVelocity desired_velocity = {0.5f, 0.0f, 0.2f};
WheelVelocities wheel_velocities;
kinematics_manager.inverseKinematics(desired_velocity, wheel_velocities);

// 应用到电机
for (int i = 0; i < 4; i++) {
    motor_controller.setSpeed(i, wheel_velocities.wheel[i]);
}

// 切换到全向轮
kinematics_manager.switchWheelSystem(WheelSystemType::OMNI);

// 使用全向轮运动学
kinematics_manager.inverseKinematics(desired_velocity, wheel_velocities);

// 应用到电机
for (int i = 0; i < 4; i++) {
    motor_controller.setSpeed(i, wheel_velocities.wheel[i]);
}
```

## 优化建议

1. 使用常量表达式进行编译时计算
   ```cpp
   constexpr float SQRT_2 = 1.41421356237f;
   ```

2. 避免重复计算
   ```cpp
   // 计算一次并重用
   float wheel_sum = wheel_distance_x_ + wheel_distance_y_;
   ```

3. 使用内联函数提高性能
   ```cpp
   inline float radToDeg(float rad) { return rad * 57.29577951f; }
   ```

4. 使用固定点数学代替浮点运算（如果目标平台没有FPU）
   ```cpp
   // 使用整数乘以1000表示毫米，避免浮点运算
   int32_t position_mm = static_cast<int32_t>(position_m * 1000.0f);
   ```

## 单元测试建议

为每种运动学模型创建单元测试，验证：
1. 正运动学和逆运动学的一致性
2. 边界情况处理
3. 零输入情况
4. 最大速度情况

## 注意事项

1. 所有角度计算应统一使用弧度制
2. 确保所有物理单位一致（如全部使用米和秒）
3. 提供适当的调试输出选项 