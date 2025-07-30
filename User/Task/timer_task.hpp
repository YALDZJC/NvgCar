#pragma once

#include "User/Alg/FSM/alg_fsm.hpp"
#include "User/BSP/Remote/Dr16/dr16.hpp"
#include "User/config.hpp" // 包含配置文件，其中已定义PID参数

/**
 * @brief 系统状态枚举
 */
enum SystemState
{
    SYSTEM_DISABLED = 0, // 系统失能
    SYSTEM_ENABLED = 1   // 系统使能
};

// 声明FSM实例
extern Class_FSM system_fsm;

/**
 * @brief 定时器中断回调函数
 * @param htim 定时器句柄
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
