/**
 * @file can_device_impl.hpp
 * @brief CAN 设备实现类
 *
 * 设计原因：
 *   CanDevice 实现 ICanDevice 接口，封装 STM32 HAL CAN 的收发逻辑。
 *   回调机制用 std::vector<RxCallback> 存储，支持多回调注册，
 *   使电机层可通过 register_rx_callback 注册 Parse 函数，实现收发解耦。
 *
 * 继承说明：
 *   移植自参考工程 H_SG_Gimbal。**裸机适配修改**：
 *   1. 移除 #include "cmsis_os2.h"(FreeRTOS/CMSIS-RTOS2 依赖)
 *   2. 移除 osMutexId_t tx_mutex_ 成员及 ensure_tx_mutex()
 *   3. send() 不再加锁——裸机下 CAN 发送仅从主循环调用，无并发竞争
 *
 * @note  std::vector 在 Keil ARMCLANG + libcxx 下可用。回调注册发生在
 *        初始化阶段(主循环前)，中断中仅遍历不 push_back，无重入风险。
 */

#pragma once
#include "../interface/can_device.hpp"
#include <vector>

namespace HAL::CAN
{

/**
 * @brief CAN 硬件设备实现类
 *
 * 一路 CAN 外设对应一个 CanDevice 实例，由 CanBus 统一管理。
 */
class CanDevice : public ICanDevice
{
public:
    /**
     * @brief 构造函数
     * @param handle      HAL CAN 句柄指针(如 &hcan1)
     * @param filter_bank 过滤器组号(CAN1:0-13, CAN2:14-27)
     * @param fifo        接收 FIFO(CAN_FILTER_FIFO0 / CAN_FILTER_FIFO1)
     */
    explicit CanDevice(CAN_HandleTypeDef *handle, uint32_t filter_bank, uint32_t fifo);

    ~CanDevice() override = default;

    // 禁止拷贝(持有硬件资源)
    CanDevice(const CanDevice &) = delete;
    CanDevice &operator=(const CanDevice &) = delete;

    // === ICanDevice 接口实现 ===
    void init() override;               // 配置过滤器(全通)
    void start() override;              // 启动 CAN + 使能接收中断
    bool send(const Frame &frame) override;
    bool receive(Frame &frame) override;
    CAN_HandleTypeDef *get_handle() const override;

    void register_rx_callback(RxCallback callback) override;
    void trigger_rx_callbacks(const Frame &frame) override;

private:
    CAN_HandleTypeDef *handle_;        // HAL CAN 句柄
    uint32_t filter_bank_;             // 过滤器组号
    uint32_t fifo_;                    // 接收 FIFO 编号
    uint32_t mailbox_;                 // 发送邮箱(暂存)

    std::vector<RxCallback> rx_callbacks_;  // 已注册的接收回调列表

    /// 配置 CAN 过滤器(当前设为全通，接收所有 ID)
    void configure_filter();
};

} // namespace HAL::CAN
