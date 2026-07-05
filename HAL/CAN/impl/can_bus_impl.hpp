/**
 * @file can_bus_impl.hpp
 * @brief CAN 总线管理实现类(单例)
 *
 * 设计原因：
 *   CanBus 采用单例模式统一管理所有 CAN 设备实例(Can1/Can2)。
 *   上层通过 get_can_bus_instance() 获取，首次访问时自动初始化(懒汉模式)，
 *   避免静态初始化顺序问题。
 *
 * 继承说明：完整移植自参考工程，无 RTOS 依赖，无需修改。
 */

#pragma once
#include "../interface/can_bus.hpp"
#include "can_device_impl.hpp"

namespace HAL::CAN
{

/**
 * @brief CAN 总线管理实现类(单例)
 *
 * 持有 Can1/Can2 两个 CanDevice 实例，提供统一的设备获取接口。
 */
class CanBus : public ICanBus
{
public:
    /// 获取单例实例(首次调用时自动初始化)
    static CanBus &instance();

    ~CanBus() override = default;

    // 禁止拷贝
    CanBus(const CanBus &) = delete;
    CanBus &operator=(const CanBus &) = delete;

    // === ICanBus 接口实现 ===
    ICanDevice &get_device(CanDeviceId id) override;
    bool has_device(CanDeviceId id) const override;

private:
    /// 私有构造函数(单例)
    CanBus();

    /// 初始化所有已注册设备(配置过滤器 + 启动 + 使能中断)
    void init();

    /// 注册一个 CAN 设备到管理表
    void register_device(CanDeviceId id, CanDevice *device);

    bool initialized_ = false;   // 是否已初始化标志

    // 设备指针表(按 CanDeviceId 索引)
    CanDevice *devices_[(size_t)CanDeviceId::MAX_DEVICES] = {nullptr};

    // 实际设备实例
    CanDevice can1_;    // CAN1: 云台电机总线(Yaw/Pitch/Fold)
    CanDevice can2_;    // CAN2: 预留(后续接底盘等)
};

} // namespace HAL::CAN
