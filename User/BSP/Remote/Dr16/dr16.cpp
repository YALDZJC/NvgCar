#include "User/BSP/Remote/Dr16/dr16.hpp"
#include "memory"
#include <cstring>

namespace BSP ::Remote
{
void Dr16::Init()
{
    auto &uart_dev = HAL::UART::get_uart_bus_instance().get_device(DR16_UART_ID);
    HAL::UART::Data rx_data{pData, sizeof(pData)};
    uart_dev.receive_dma_idle(rx_data);
}

/**
 * @brief 利用memcpy保存数据
 *
 * @param pData 缓冲区
 */
void Dr16::SaveData(const uint8_t *pData)
{
    uint64_t part1;
    memcpy(&part1, pData, 6);
    pData += 6;
    data_part1_ = part1;

    uint64_t part2;
    memcpy(&part2, pData, 8);
    pData += 8;
    data_part2_ = part2;

    uint64_t part3;
    memcpy(&part3, pData, 4);
    pData += 4;
    data_part3_ = part3;
}

/**
 * @brief 更新遥控器状态
 * 使用位域结构体将data_part强转为结构体，再赋值到对应的结构体成员
 * 使用C++风格reinterpret_cast强转指针
 */
void Dr16::UpdateStatus()
{
    auto &part1 alignas(uint64_t) = *reinterpret_cast<Dr16DataPart1 *>(&data_part1_);
    auto channel_to_double = [](uint16_t value) { return (static_cast<int32_t>(value) - 1024) / 660.0; };

    joystick_right_.y = -channel_to_double(static_cast<uint16_t>(part1.joystick_channel1));
    joystick_right_.x = -channel_to_double(static_cast<uint16_t>(part1.joystick_channel0));
    joystick_left_.y = channel_to_double(static_cast<uint16_t>(part1.joystick_channel3));
    joystick_left_.x = channel_to_double(static_cast<uint16_t>(part1.joystick_channel2));

    switch_right_ = static_cast<Switch>(part1.switch_right);
    switch_left_ = static_cast<Switch>(part1.switch_left);

    auto &part2 alignas(uint64_t) = *reinterpret_cast<Dr16DataPart2 *>(&data_part2_);
    mouse_vel_.x = -(part2.mouse_velocity_x / 32768.0);
    mouse_vel_.y = -(part2.mouse_velocity_y / 32768.0);

    mouse_.left = part2.mouse_left;
    mouse_.right = part2.mouse_right;

    auto &part3 alignas(uint64_t) = *reinterpret_cast<Dr16DataPart3 *>(&data_part3_);
    keyboard_ = part3.keyboard;
    sw_ = channel_to_double(static_cast<uint16_t>(part3.sw));
}

/**
 * @brief 数据解析
 *
 * @param huart 对应串口号
 * @param Size 数据的大小
 */
void Dr16::Parse(HAL::UART::IUartDevice *uart_dev, int Size)
{
    if (uart_dev && Size == sizeof(pData))
    {
        SaveData(pData);
        UpdateStatus();
        state_watch_.updateTimestamp();
    }
    // 重新启动DMA空闲接收
    HAL::UART::Data rx_data{pData, sizeof(pData)};
    uart_dev->receive_dma_idle(rx_data);
}

/**
 * @brief 清除ORE错误
 *
 * @param huart 对应串口handle
 * @param pData 缓冲区
 * @param Size 数据大小
 */
void Dr16::ClearORE(HAL::UART::IUartDevice *uart_dev, uint8_t *pData, int Size)
{
    if (uart_dev)
    {
        HAL::UART::Data rx_data{pData, (uint16_t)Size};
        uart_dev->clear_ore_error(rx_data);
    }
}

bool Dr16::ISDir()
{
    char Dir = 0;

    Dir |= (remoteRight().x < -1 || remoteRight().x > 1);
    Dir |= (remoteRight().y < -1 || remoteRight().y > 1);
    Dir |= (remoteLeft().x < -1 || remoteLeft().x > 1);
    Dir |= (remoteLeft().y < -1 || remoteLeft().y > 1);

    live_state_ = (state_watch_.check() == BSP::WATCH_STATE::Status::OFFLINE) | Dir;
    if (live_state_)
    {
        joystick_right_.x = 0;
        joystick_right_.y = 0;
        joystick_left_.x = 0;
        joystick_left_.y = 0;

        switch_right_ = Switch::UNKNOWN;
        switch_left_ = Switch::UNKNOWN;

        // 通过HAL库接口获取设备并清ORE
        auto &uart_dev = HAL::UART::get_uart_bus_instance().get_device(DR16_UART_ID);
        ClearORE(&uart_dev, pData, sizeof(pData));
    }

    return live_state_;
}
} // namespace BSP::Remote