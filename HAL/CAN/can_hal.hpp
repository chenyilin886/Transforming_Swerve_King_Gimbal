/**
 * @file can_hal.hpp
 * @brief CAN 总线 HAL 层统一入口
 *
 * 设计原因：
 *   参考工程将 CAN 接口与实现分离(interface/ + impl/)，用户代码只需
 *   包含此头文件即可访问全部 CAN 功能，无需关心实现细节。
 *
 * 使用方法：
 *   #include "HAL/CAN/can_hal.hpp"
 *   auto& can1 = HAL::CAN::get_can_bus_instance().get_can1();
 *   can1.send(frame);
 *   can1.register_rx_callback([](const HAL::CAN::Frame& f){ ... });
 *
 * 继承说明：完整移植自参考工程。
 */

#pragma once

// 包含所有 CAN 接口头文件
#include "interface/can_bus.hpp"
#include "interface/can_device.hpp"

// 使用命名空间 HAL::CAN 即可访问所有 CAN 相关功能
