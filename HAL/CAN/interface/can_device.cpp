/**
 * @file can_device.cpp
 * @brief CAN 设备接口实现(extract_id 静态方法)
 *
 * 继承说明：完整移植自参考工程，无修改。
 */

#include "can_device.hpp"

namespace HAL::CAN
{

/**
 * @brief 从 HAL 接收头提取 CAN ID
 * @param rx_header HAL 接收头
 * @return 标准 ID(11位) 或 扩展 ID(29位)
 * @note   根据 IDE 字段自动判断标准/扩展帧
 */
ID_t ICanDevice::extract_id(const CAN_RxHeaderTypeDef &rx_header)
{
    return (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId : rx_header.ExtId;
}

} // namespace HAL::CAN
