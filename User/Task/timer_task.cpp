#include "tim.h"

#include "User/BSP/Motor/Dji/DjiMotor.hpp"
#include "User/HAL/LOGGER/logger.hpp"

#include "User/Alg/WHEEL/Manger/kinematics_manager.h"
#include "User/BSP/Remote/Dr16/Dr16.hpp"

#include "User/Alg/PID/PID.hpp"
#include "User/Task/timer_task.hpp"
#include "adc.h"

// 系统FSM实例
Class_FSM system_fsm;

// 运动学管理器实例（差速模型参数）
// 使用config.hpp中定义的机器人参数
static KinematicsManager kinematics_manager(ROBOT_WHEEL_RADIUS, ROBOT_HALF_TRACK, ROBOT_LONGITUDINAL_DIST,
                                            ROBOT_RADIUS);

// 轮子目标速度
WheelVelocities target_wheel_velocities;

// 轮子速度PID控制器
ALG::PID::PID wheel_vel_pid[4] = {
    ALG::PID::PID(WHEEL_VEL_PID_KP, WHEEL_VEL_PID_KI, WHEEL_VEL_PID_KD, WHEEL_VEL_PID_MAX),
    ALG::PID::PID(WHEEL_VEL_PID_KP, WHEEL_VEL_PID_KI, WHEEL_VEL_PID_KD, WHEEL_VEL_PID_MAX),
    ALG::PID::PID(WHEEL_VEL_PID_KP, WHEEL_VEL_PID_KI, WHEEL_VEL_PID_KD, WHEEL_VEL_PID_MAX),
    ALG::PID::PID(WHEEL_VEL_PID_KP, WHEEL_VEL_PID_KI, WHEEL_VEL_PID_KD, WHEEL_VEL_PID_MAX)};

/**
 * @brief 初始化系统FSM
 */
static void initSystemFSM()
{
    static bool fsm_initialized = false;
    if (!fsm_initialized)
    {
        system_fsm.Init(2, SYSTEM_ENABLED); // 两个状态：失能和使能，初始状态为使能
        fsm_initialized = true;
    }
}

/**
 * @brief 更新系统状态
 * @param dr16 遥控器实例
 * @param log 日志实例
 */
static void updateSystemState(BSP::Remote::Dr16 &dr16)
{
    // 检查遥控器左下和右下是否同时为1（DOWN状态）
    bool left_down = (dr16.switchLeft() == BSP::Remote::Dr16::Switch::DOWN);
    bool right_down = (dr16.switchRight() == BSP::Remote::Dr16::Switch::DOWN);

    // FSM状态更新逻辑
    if (left_down && right_down)
    {
        // 如果左下和右下同时为DOWN，切换到DISABLED状态
        if (system_fsm.Get_Now_Status_Serial() != SYSTEM_DISABLED)
        {
            system_fsm.Set_Status(SYSTEM_DISABLED);
            LOG_INFO("System: DISABLED");
        }
    }
    else
    {
        // 其他情况，切换到ENABLED状态
        if (system_fsm.Get_Now_Status_Serial() != SYSTEM_ENABLED)
        {
            system_fsm.Set_Status(SYSTEM_ENABLED);
            LOG_INFO("System: ENABLED");
        }
    }
}

/**
 * @brief 停止所有电机
 */
static void stopAllMotors()
{
    // 停止所有电机
    for (uint8_t i = 1; i <= 4; i++)
    {
        wheel_vel_pid[i - 1].reset(); // 重置PID控制器
        BSP::Motor::Dji::Motor2006.setCAN(0, i);
    }
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    BSP::Motor::Dji::Motor2006.sendCAN(&can1);
}

/**
 * @brief 获取当前轮子速度
 * @param wheel_velocities 输出的轮子速度
 */
static void getCurrentWheelVelocities(WheelVelocities &wheel_velocities)
{
    wheel_velocities.wheel[0] = BSP::Motor::Dji::Motor2006.getVelocityRads(1); // 左轮
    wheel_velocities.wheel[1] = BSP::Motor::Dji::Motor2006.getVelocityRads(2); // 右轮
    wheel_velocities.wheel[2] = BSP::Motor::Dji::Motor2006.getVelocityRads(3); // 第3个轮子
    wheel_velocities.wheel[3] = BSP::Motor::Dji::Motor2006.getVelocityRads(4); // 第4个轮子
}

/**
 * @brief 根据遥控器输入计算目标机器人速度
 * @param dr16 遥控器实例
 * @return 目标机器人速度
 */
static RobotVelocity calculateTargetRobotVelocity(BSP::Remote::Dr16 &dr16)
{
    RobotVelocity target_velocity = {
        static_cast<float>(dr16.remoteLeft().y),      // 前后移动
        0.0f,                                         // 横向移动(差速模型不支持)
        static_cast<float>(dr16.remoteLeft().x * 5.0) // 旋转速度
    };
    return target_velocity;
}

/**
 * @brief 应用PID控制并发送电机指令
 */
static void applyPIDControlAndSendCommands()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        wheel_vel_pid[i].setTarget(target_wheel_velocities.wheel[i]);
        wheel_vel_pid[i].setIntegralLimit(WHEEL_VEL_PID_INTEGRAL_LIMIT);
        wheel_vel_pid[i].setIntegralSeparation(WHEEL_VEL_PID_INTEGRAL_SEPARATION_THRESHOLD);

        wheel_vel_pid[i].Calc(BSP::Motor::Dji::Motor2006.getVelocityRads(i + 1));
        BSP::Motor::Dji::Motor2006.setCAN(wheel_vel_pid[i].getOutput(), i + 1);
    }

    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    BSP::Motor::Dji::Motor2006.sendCAN(&can1);
}

/**
 * @brief 定时器中断回调函数
 * @param htim 定时器句柄
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 只处理TIM5定时器中断
    if (htim != &htim5)
    {
        return;
    }
    auto &dr16 = BSP::Remote::Dr16::instance();

    // 初始化FSM
    initSystemFSM();

    // 更新系统状态
    updateSystemState(dr16);

    // 如果系统处于失能状态，停止所有电机并返回
    if (system_fsm.Get_Now_Status_Serial() == SYSTEM_DISABLED)
    {
        stopAllMotors();
        return;
    }

    // 获取当前轮子速度
    WheelVelocities current_wheel_velocities;
    getCurrentWheelVelocities(current_wheel_velocities);

    // 切换到差速运动学模型
    kinematics_manager.switchWheelSystem(WheelSystemType::MECANUM);

    // 计算当前机器人速度（正运动学）
    RobotVelocity current_robot_velocity;
    kinematics_manager.forwardKinematics(current_wheel_velocities, current_robot_velocity);

    // 根据遥控器输入计算目标机器人速度
    RobotVelocity target_robot_velocity = calculateTargetRobotVelocity(dr16);

    // 根据目标机器人速度计算目标轮子速度（逆运动学）
    kinematics_manager.inverseKinematics(target_robot_velocity, target_wheel_velocities);

    // 应用PID控制并发送电机指令
    applyPIDControlAndSendCommands();

    LOG_INFO("HELLO WORLD");
}