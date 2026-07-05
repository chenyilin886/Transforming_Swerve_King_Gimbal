/**
 * @file can_bus.hpp
 * @brief CAN 总线管理抽象接口
 *
 * 设计原因：
 *   将多路 CAN 外设(CAN1/CAN2)统一管理，上层通过 CanDeviceId 枚举获取设备，
 *   避免直接依赖具体的 hcan1/hcan2 句柄。新增 CAN3 时只需扩展枚举，
 *   符合开闭原则。
 *
 * 继承说明：完整移植自参考工程，无修改。
 */

#pragma once
#include "can_device.hpp"
#include <cstdint>

namespace HAL::CAN
{

/// CAN 设备 ID 枚举，对应各路 CAN 外设
enum class CanDeviceId : uint8_t
{
    HAL_Can1 = 0,   // CAN1(本工程云台电机总线)
    HAL_Can2 = 1,   // CAN2(预留，后续可接底盘/其他设备)
    // HAL_Can3 = 2, // 预留扩展位
    MAX_DEVICES
};

/**
 * @brief CAN 总线抽象接口
 *
 * 管理所有 CAN 设备实例，提供统一的设备获取接口。
 * 实现见 CanBus(can_bus_impl.hpp)，采用单例模式。
 */
class ICanBus
{
public:
    virtual ~ICanBus() = default;

    /**
     * @brief 获取指定 ID 的 CAN 设备
     * @param id 设备枚举
     * @return CAN 设备接口引用
     */
    virtual ICanDevice &get_device(CanDeviceId id) = 0;

    /// 兼容旧 API 的便捷方法：获取 CAN1
    ICanDevice &get_can1() { return get_device(CanDeviceId::HAL_Can1); }

    /// 兼容旧 API 的便捷方法：获取 CAN2
    ICanDevice &get_can2() { return get_device(CanDeviceId::HAL_Can2); }

    /**
     * @brief 检查指定 ID 的设备是否存在
     * @param id 设备枚举
     * @return true=已注册
     */
    virtual bool has_device(CanDeviceId id) const = 0;
};

/**
 * @brief 获取 CAN 总线单例实例
 * @return ICanBus 引用(首次调用时自动初始化)
 * @note   懒汉模式，在第一次获取时完成过滤器配置+启动外设+使能中断
 */
ICanBus &get_can_bus_instance();

} // namespace HAL::CAN
