/**
 * @file can_device.hpp
 * @brief CAN 设备抽象接口定义
 *
 * 设计原因：
 *   参考工程将 CAN 设备抽象为 ICanDevice 接口，使上层 Motor 层依赖抽象而非具体实现。
 *   这样新增 CAN 外设(如 CAN3)或替换底层实现时，不影响 Motor 层代码。
 *
 * 继承说明：
 *   完整移植自参考工程 H_SG_Gimbal HAL/CAN/interface/can_device.hpp。
 *   接口本身不依赖 RTOS，无需修改。
 */

#pragma once
#include "can.h"
#include <functional>

namespace HAL::CAN
{

/// CAN 消息 ID 类型
using ID_t = uint32_t;

/**
 * @brief CAN 数据帧结构体
 *
 * 统一封装一帧 CAN 消息的所有属性，上层(电机/遥控器)只需操作此结构体，
 * 无需直接接触 HAL 的 CAN_TxHeaderTypeDef / CAN_RxHeaderTypeDef。
 *
 * @note  data 固定 8 字节，对应经典 CAN 单帧最大载荷
 */
struct Frame
{
    uint8_t  data[8];            // 8字节数据载荷
    ID_t     id;                 // CAN ID(标准帧11位 / 扩展帧29位)
    uint8_t  dlc;                // 数据长度代码(0-8)
    uint32_t mailbox;            // 发送邮箱编号(发送时使用)
    bool     is_extended_id;     // true=扩展ID(29位), false=标准ID(11位)
    bool     is_remote_frame;   // true=远程帧, false=数据帧
};

/**
 * @brief CAN 接收回调函数类型
 *
 * 收到一帧数据后触发，参数为解析后的 Frame。
 * 可注册多个回调，按注册顺序依次执行。
 * @warning 回调在中断上下文中执行，必须简短高效
 */
using RxCallback = std::function<void(const Frame &)>;

/**
 * @brief CAN 设备抽象接口
 *
 * 定义了一路 CAN 外设的标准操作：初始化、启动、收发、回调注册。
 * 具体实现见 CanDevice(can_device_impl.hpp)。
 */
class ICanDevice
{
public:
    virtual ~ICanDevice() = default;

    /// 初始化 CAN 设备(配置过滤器)
    virtual void init() = 0;

    /// 启动 CAN 设备(开启外设 + 使能接收中断)
    virtual void start() = 0;

    /**
     * @brief 发送一帧 CAN 数据
     * @param frame 待发送的帧
     * @return true=发送成功, false=邮箱满或发送失败
     */
    virtual bool send(const Frame &frame) = 0;

    /**
     * @brief 接收一帧 CAN 数据(非阻塞)
     * @param frame 输出参数，接收到的帧
     * @return true=接收成功并自动触发回调, false=FIFO空
     * @note   成功接收后内部自动调用 trigger_rx_callbacks()
     */
    virtual bool receive(Frame &frame) = 0;

    /// 获取底层 HAL CAN 句柄(用于中断回调中判断是哪路 CAN)
    virtual CAN_HandleTypeDef *get_handle() const = 0;

    /**
     * @brief 注册接收回调函数
     * @param callback 回调函数(std::function)
     * @note   建议在系统初始化阶段注册，不要在中断中注册
     */
    virtual void register_rx_callback(RxCallback callback) = 0;

    /**
     * @brief 触发所有已注册的回调
     * @param frame 接收到的帧
     * @note   通常由 receive() 内部自动调用，无需手动触发
     */
    virtual void trigger_rx_callbacks(const Frame &frame) = 0;

    /**
     * @brief 从 RX 头中提取 CAN ID
     * @param rx_header HAL 接收头
     * @return 标准/扩展 ID
     */
    static ID_t extract_id(const CAN_RxHeaderTypeDef &rx_header);
};

} // namespace HAL::CAN
