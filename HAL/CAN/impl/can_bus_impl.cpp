/**
 * @file can_bus_impl.cpp
 * @brief CAN 总线单例实现
 *
 * 继承说明：完整移植自参考工程，无修改。
 *
 * 初始化流程(懒汉模式)：
 *   首次调用 get_can_bus_instance()
 *     → CanBus::instance() 构造静态实例
 *       → 构造函数: 初始化 can1_(&hcan1) / can2_(&hcan2) + 注册到表
 *     → init(): 遍历设备表 → 各设备 init()(过滤器) + start()(启动+中断)
 *
 * @note 必须在 MX_CAN1_Init / MX_CAN2_Init 之后调用，否则 hcan 未初始化。
 *       本工程在 GimbalInit() 中首次调用，满足时序要求。
 */

#include "can_bus_impl.hpp"

namespace HAL::CAN
{

/**
 * @brief 获取单例实例(懒汉模式)
 * @return CanBus 单例引用
 * @note  首次调用时自动执行 init()
 */
CanBus &CanBus::instance()
{
    static CanBus instance;
    if (!instance.initialized_)
    {
        instance.init();
        instance.initialized_ = true;
    }
    return instance;
}

/**
 * @brief 构造函数
 *
 * 初始化两个 CanDevice 实例：
 *   can1_ → hcan1, 过滤器组 0, FIFO0
 *   can2_ → hcan2, 过滤器组 14, FIFO1
 * 并注册到设备管理表。
 */
CanBus::CanBus()
    : can1_(&hcan1, 0, CAN_FILTER_FIFO0),
      can2_(&hcan2, 14, CAN_FILTER_FIFO1)
{
    register_device(CanDeviceId::HAL_Can1, &can1_);
    register_device(CanDeviceId::HAL_Can2, &can2_);
}

/**
 * @brief 初始化所有已注册设备
 *
 * 遍历设备表，对每个非空设备执行 init()(配置过滤器) + start()(启动+中断)。
 */
void CanBus::init()
{
    for (size_t i = 0; i < (size_t)CanDeviceId::MAX_DEVICES; ++i)
    {
        if (devices_[i] != nullptr)
        {
            devices_[i]->init();
            devices_[i]->start();
        }
    }
}

/**
 * @brief 注册设备到管理表
 */
void CanBus::register_device(CanDeviceId id, CanDevice *device)
{
    if (id < CanDeviceId::MAX_DEVICES && device != nullptr)
    {
        devices_[(size_t)id] = device;
    }
}

/**
 * @brief 获取指定设备
 * @param id 设备枚举
 * @return 设备引用
 * @note  若设备不存在，返回 can1_ 作为兜底(保证永远有返回值)
 */
ICanDevice &CanBus::get_device(CanDeviceId id)
{
    if (id < CanDeviceId::MAX_DEVICES && devices_[(size_t)id] != nullptr)
    {
        return *devices_[(size_t)id];
    }
    return can1_;
}

/**
 * @brief 检查设备是否存在
 */
bool CanBus::has_device(CanDeviceId id) const
{
    return id < CanDeviceId::MAX_DEVICES && devices_[(size_t)id] != nullptr;
}

} // namespace HAL::CAN
