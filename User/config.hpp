#pragma once

// 遥控器串口配置（可根据实际硬件修改）
#define DR16_UART_ID HAL::UART::UartDeviceId::HAL_Uart1
// 上位机串口配置（可根据实际硬件修改）
#define UP_UART_ID HAL::UART::UartDeviceId::HAL_Uart6
// 遥控器在线状态检测超时时间（毫秒）
#define REMOTE_ONLINE_TIMEOUT_MS 100

/**
 * @brief 机器人运动学参数
 *
 */
// 轮子半径（单位：米）
#define ROBOT_WHEEL_RADIUS 0.06f
// 轮距的一半（单位：米）
#define ROBOT_HALF_TRACK 0.1f
// 纵向距离（单位：米）
#define ROBOT_LONGITUDINAL_DIST 0.15f
// 机器人半径（单位：米）
#define ROBOT_RADIUS 0.18f

/**
 * @brief 轮毂pid相关
 *
 */
#define WHEEL_VEL_PID_KP 0.00f
#define WHEEL_VEL_PID_KI 0.00f
#define WHEEL_VEL_PID_KD 0.00f
#define WHEEL_VEL_PID_MAX 16384.0f
// 积分限幅
#define WHEEL_VEL_PID_INTEGRAL_LIMIT 16384.0f
// 积分隔离阈值
#define WHEEL_VEL_PID_INTEGRAL_SEPARATION_THRESHOLD 16384.0f
