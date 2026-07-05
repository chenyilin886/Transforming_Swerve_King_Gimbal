/**
 * @file CanCallback.cpp
 * @brief CAN 接收中断回调实现
 *
 * 设计原因：
 *   STM32 HAL 库的 CAN 接收采用"弱函数 + 重写"机制：
 *   HAL_CAN_RxFifo0MsgPendingCallback 是 __weak 函数，用户需在 C 文件中
 *   提供同名实现覆盖。此文件集中处理 CAN1/CAN2 的接收中断回调。
 *
 * 数据流：
 *   CAN 总线数据到达
 *     → stm32f4xx_it.c: CAN1_RX0_IRQHandler()
 *       → HAL_CAN_IRQHandler(&hcan1)
 *         → HAL_CAN_RxFifo0MsgPendingCallback() [本文件重写]
 *           → can1.receive(frame)  [CanDevice::receive]
 *             → HAL_CAN_GetRxMessage 提取数据
 *             → trigger_rx_callbacks(frame)
 *               → DM4310::Parse / DM4340::Parse
 *
 * @note  此回调在中断上下文中执行，必须简短高效。
 *        不应在此函数中调用 HAL_Delay 或进行复杂计算。
 */

#include "main.h"
#include "can_hal.hpp"

/**
 * @brief CAN1 FIFO0 接收回调
 *
 * 当 CAN1 FIFO0 收到消息时由 HAL 触发。
 * 调用 can1.receive(frame) 提取数据并自动触发已注册的电机 Parse 回调。
 *
 * @param hcan CAN 句柄(判断是 CAN1 还是 CAN2)
 * @note  此函数在 CAN1_RX0_IRQHandler 中断上下文中执行
 * @note  **必须** 使用 extern "C" 包裹！HAL 回调是 __weak C 函数，
 *        C++ 编译器的 name mangling 会导致链接器找不到覆盖符号，
 *        回调永远不会被调用 —— 这是最常见的"数据全0"Bug。
 */
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        // 获取 CAN1 设备，提取数据并触发回调(电机 Parse)
        static auto &can1 = HAL::CAN::get_can_bus_instance().get_can1();
        HAL::CAN::Frame frame;
        can1.receive(frame);
    }
    else if (hcan->Instance == CAN2)
    {
        // CAN2 FIFO0(本工程 CAN2 用 FIFO1，此处预留)
    }
}

/**
 * @brief CAN2 FIFO1 接收回调(预留)
 *
 * 本工程 CAN2 暂未使用，预留接口供后续扩展。
 *
 * @param hcan CAN 句柄
 * @note  同样需要 extern "C"
 */
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN2)
    {
        // CAN2 FIFO1 接收处理(预留)
        // auto &can2 = HAL::CAN::get_can_bus_instance().get_can2();
        // HAL::CAN::Frame frame;
        // can2.receive(frame);
    }
}
