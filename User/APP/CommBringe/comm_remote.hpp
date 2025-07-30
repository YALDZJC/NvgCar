#pragma once

#include "User/HAL/UART/interface/uart_device.hpp"
#include <stdint.h> // 添加uint8_t定义

namespace APP::CommBringe
{
class CommRemote
{
    struct __attribute__((packed)) RemoteData
    {
        float Metal_detection_V;
        // uint8_t reserved : 7; // Reserved bits for future use
    };

  private:
    HAL::UART::IUartDevice *uart_device_;
    RemoteData remote_data_;

  public:
    CommRemote(HAL::UART::IUartDevice *uart_device);
    void Init();
    void SetMetal_detection_V(float Metal_detection_V);
    void Update();
};

} // namespace APP::CommBringe
