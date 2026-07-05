/**
 * @file can_bus.cpp
 * @brief CAN 总线接口实现(单例获取函数)
 *
 * 继承说明：完整移植自参考工程。get_can_bus_instance() 返回 CanBus 单例，
 * 具体实现在 impl/can_bus_impl.cpp 中。
 */

#include "can_bus.hpp"
#include "../impl/can_bus_impl.hpp"

namespace HAL::CAN
{

/**
 * @brief 获取 CAN 总线单例实例
 * @return ICanBus 引用
 * @note   内部委托给 CanBus::instance()，首次调用时自动初始化
 */
ICanBus &get_can_bus_instance()
{
    return CanBus::instance();
}

} // namespace HAL::CAN
