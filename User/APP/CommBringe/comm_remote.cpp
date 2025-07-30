#include "User/APP/CommBringe/comm_remote.hpp"
#include "User/HAL/LOGGER/logger.hpp"

namespace APP::CommBringe
{

/**
 * @brief 构造函数
 * @param uart_device UART设备指针
 */
CommRemote::CommRemote(HAL::UART::IUartDevice *uart_device) : uart_device_(uart_device), remote_data_({0})
{
}

/**
 * @brief 初始化函数
 */
void CommRemote::Init()
{
    if (uart_device_ == nullptr)
    {
        HAL::LOGGER::Logger::getInstance().error("CommRemote: UART device is nullptr");
        return;
    }

    // 初始化UART设备
    uart_device_->init();
    uart_device_->start();

    // 初始化数据
    remote_data_.Metal_detection_V = 0;
    // remote_data_.reserved = 0;

    HAL::LOGGER::Logger::getInstance().info("CommRemote: Initialized successfully");
}

/**
 * @brief 发送金属探测状态
 * @param detected 是否检测到金属
 */
void CommRemote::SetMetal_detection_V(float Metal_detection_V)
{
    if (uart_device_ == nullptr)
    {
        HAL::LOGGER::Logger::getInstance().error("CommRemote: UART device is nullptr");
        return;
    }

    // 更新数据
    remote_data_.Metal_detection_V = Metal_detection_V;

    // 发送数据
    HAL::UART::Data data;
    data.buffer = reinterpret_cast<uint8_t *>(&remote_data_);
    data.size = sizeof(remote_data_);

    if (uart_device_->transmit(data))
    {
        HAL::LOGGER::Logger::getInstance().trace("CommRemote: Sent metal detection status");
    }
    else
    {
        HAL::LOGGER::Logger::getInstance().error("CommRemote: Failed to send metal detection status");
    }
}

/**
 * @brief 周期性调用的更新函数
 */
void CommRemote::Update()
{
    // 这里可以添加定期发送数据的逻辑
    // 或者其他需要周期性执行的操作
}

} // namespace APP::CommBringe