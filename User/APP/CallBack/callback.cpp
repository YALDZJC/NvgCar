#include "User/BSP/Common/StateWatch/state_watch.hpp"
#include "User/BSP/Motor/Dji/DjiMotor.hpp"
#include "User/HAL/CAN/can_hal.hpp"
#include "User/HAL/LOGGER/logger.hpp"
#include "User/HAL/UART/uart_hal.hpp"

#include "User/BSP/Remote/Dr16/dr16.hpp"
// Replace the metal_uart.hpp with comm_remote.hpp
#include "User/APP/CommBringe/comm_remote.hpp"
#include "User/BSP/IMU/HI12H3_IMU.hpp"

#include "User/config.hpp"
#include "tim.h"

uint8_t buffer[3] = {0};
uint16_t speed = 0;

bool flag = false;
// CommRemote instance
APP::CommBringe::CommRemote *comm_remote = nullptr;

// ADC相关变量
uint16_t ADC_Value[10];
float ADC_V;
int i, adc;

extern "C"
{
    void Init()
    {
        HAL::CAN::get_can_bus_instance();

        BSP::Remote::Dr16::instance();

        // 初始化串口通信模块
        auto &up_uart = HAL::UART::get_uart_bus_instance().get_device(UP_UART_ID);
        comm_remote = new APP::CommBringe::CommRemote(&up_uart);
        comm_remote->Init();

        // 初始化IMU
        BSP::IMU::imu.Init();

        HAL_TIM_Base_Start_IT(&htim5);

        auto &log = HAL::LOGGER::Logger::getInstance();
        log.trace("Init");
    }

    void InWhile()
    {
        auto &logger = HAL::LOGGER::Logger::getInstance();
        // logger.trace("ADC_V: %d", ADC_V_int);
    }
} // extern "C"

HAL::CAN::Frame rx_frame;

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    can1.receive(rx_frame);
    if (hcan == can1.get_handle())
    {
        BSP::Motor::Dji::Motor2006.Parse(rx_frame);
    }
}

// UART中断
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    auto &dr16 = HAL::UART::get_uart_bus_instance().get_device(DR16_UART_ID);
    auto &imu_uart = HAL::UART::get_uart_bus_instance().get_device(IMU_UART_ID);

    if (huart == dr16.get_handle())
    {
        BSP::Remote::Dr16::instance().Parse(&dr16, Size);
    }
    if (huart == imu_uart.get_handle())
    {
        BSP::IMU::imu.Parse(huart, Size);
    }
}
