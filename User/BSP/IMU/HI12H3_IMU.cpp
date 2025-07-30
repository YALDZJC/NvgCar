#include "User/BSP/IMU/HI12H3_IMU.hpp"
#include "User/config.hpp"

#include <cstring>

namespace BSP::IMU
{
void HI12::Init()
{
    // 获取UART设备实例
    pUartDevice = &HAL::UART::get_uart_bus_instance().get_device(IMU_UART_ID);

    // 使用UART设备接口进行DMA接收
    HAL::UART::Data data = {buffer, sizeof(buffer)};
    pUartDevice->receive_dma_idle(data);
}

bool HI12::ParseData()
{
    if (buffer[0] != 0x5A || buffer[1] != 0xA5) // 帧头错误
    {
        SlidingWindowRecovery();
    }

    uint8_t *pData = buffer; // 定义数组指针，指向缓冲区的起始地址
    const auto memcpy_safe = [&](auto &data) {
        std::memcpy(&data, pData, sizeof(data));
        pData += sizeof(data);
    };

    memcpy_safe(frame);
    memcpy_safe(system_telemetry);
    memcpy_safe(acc);
    memcpy_safe(gyr);
    memcpy_safe(mag);
    memcpy_safe(euler);
    memcpy_safe(quat);

    AddCaclu(addYaw, euler.Euler_yaw);

    // 使用UART设备接口重新启动DMA接收
    HAL::UART::Data data = {buffer, sizeof(buffer)};
    pUartDevice->receive_dma_idle(data);

    return true;
}

void HI12::Parse(UART_HandleTypeDef *huart, int Size)
{
    // 检查是否是IMU的UART设备
    if (huart == pUartDevice->get_handle() && Size == sizeof(buffer))
    {
        ParseData();
        dirTime.updateTimestamp();
    }
}

void HI12::ClearORE(UART_HandleTypeDef *huart, uint8_t *pData, int Size)
{
    // 使用UART设备接口清除ORE错误
    if (pUartDevice && huart == pUartDevice->get_handle())
    {
        HAL::UART::Data data = {pData, static_cast<uint8_t>(Size)};
        pUartDevice->clear_ore_error(data);
    }
}

bool HI12::ISDir()
{
    is_dir = (dirTime.check() == BSP::WATCH_STATE::Status::OFFLINE);

    if (is_dir && pUartDevice)
    {
        HAL::UART::Data data = {buffer, sizeof(buffer)};
        pUartDevice->clear_ore_error(data);
    }

    return is_dir;
}

void HI12::SlidingWindowRecovery()
{
    const int window_size = sizeof(buffer); // 窗口大小等于缓冲区长度
    for (int i = 0; i < window_size - 1; i++)
    {
        // 逐步滑动窗口
        // 检查当前位置是否为有效帧头
        if (buffer[i] == 0x5A && buffer[i + 1] == 0xA5)
        {
            // 找到有效帧头，调整缓冲区指针
            std::memcpy(buffer, &buffer[i], sizeof(buffer) - i);
            break;
        }
    }
    // 重新启动DMA接收

    HAL::UART::Data data = {buffer, sizeof(buffer)};
    pUartDevice->receive_dma_idle(data);
}

void HI12::AddCaclu(AddData &addData, float angle)
{
    double lastData = addData.last_angle;
    double Data = angle;

    if (Data - lastData < -180) // 正转
        addData.add_angle += (360 - lastData + Data);
    else if (Data - lastData > 180) // 反转
        addData.add_angle += -(360 - Data + lastData);
    else
        addData.add_angle += (Data - lastData);

    addData.last_angle = Data;
}
} // namespace BSP::IMU
