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
 * CAN2 接收：
 *   - 0x201/0x202: GM3508 摩擦轮电机反馈
 *   - 0x207/0x208: 底盘板间通信（裁判系统数据）
 *
 * @note  此回调在中断上下文中执行，必须简短高效。
 *        不应在此函数中调用 HAL_Delay 或进行复杂计算。
 */

#include "main.h"
#include "can_hal.hpp"
#include "Communication/BoardComm.hpp"

// ========================================================================
// 调试计数器：在 Keil Watch 中观察这些变量，验证 DM 电机是否持续回复
// ========================================================================
// 验证方法：
//   烧录运行后，can1_rx_id_0x01/0x02/0x03 应持续增长（不再是停在1）。
//   若仍停在 1，说明 On() 重使能无效，需用达妙上位机检查电机看门狗超时设置。
volatile uint32_t can1_rx_total      = 0;  // CAN1 接收中断总次数
volatile uint32_t can1_rx_last_id    = 0;  // 最后收到的帧 ID
volatile uint32_t can1_rx_id_0x01    = 0;  // ID=0x01 帧计数(DM4310 Yaw)
volatile uint32_t can1_rx_id_0x02    = 0;  // ID=0x02 帧计数(DM4310 Pitch)
volatile uint32_t can1_rx_id_0x03    = 0;  // ID=0x03 帧计数(DM4340 Fold)
volatile uint32_t can1_rx_id_0x141   = 0;  // ID=0x141 帧计数(LK4005)
volatile uint32_t can1_rx_id_0x201   = 0;  // ID=0x201 帧计数(GM3508 #1)
volatile uint32_t can1_rx_id_0x202   = 0;  // ID=0x202 帧计数(GM3508 #2)
volatile uint32_t can1_rx_id_other   = 0;  // 其他 ID 帧计数

// ========================================================================
// CAN2 调试计数器：验证 GM3508 摩擦轮电机是否持续回复
// ========================================================================
volatile uint32_t can2_rx_total      = 0;  // CAN2 接收中断总次数
volatile uint32_t can2_rx_id_0x201   = 0;  // ID=0x201 帧计数(GM3508 #1)
volatile uint32_t can2_rx_id_0x202   = 0;  // ID=0x202 帧计数(GM3508 #2)
volatile uint32_t can2_rx_id_other   = 0;  // 其他 ID 帧计数

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
        // 获取 CAN1 设备
        static auto &can1 = HAL::CAN::get_can_bus_instance().get_can1();

        // 【关键修复】循环读取直到 FIFO 清空
        //   STM32F4 CAN RX FIFO 深度为 3，中断在 FMP 从 0→非零 时触发一次。
        //   三台电机同时回传时 FIFO 可能堆积多帧，只读 1 帧会导致剩余帧
        //   卡在 FIFO（FMP 3→2 不触发新中断），最终溢出丢帧。
        HAL::CAN::Frame frame;
        while (can1.receive(frame))
        {
            // --- 调试统计(中断上下文, 仅简单自增, 安全) ---
            can1_rx_total++;
            can1_rx_last_id = frame.id;
            switch (frame.id)
            {
                case 0x01:  can1_rx_id_0x01++;  break;
                case 0x02:  can1_rx_id_0x02++;  break;
                case 0x03:  can1_rx_id_0x03++;  break;
                case 0x141: can1_rx_id_0x141++; break;
                case 0x201: can1_rx_id_0x201++; break;  // GM3508 #1
                case 0x202: can1_rx_id_0x202++; break;  // GM3508 #2
                default:    can1_rx_id_other++; break;
            }
        }
    }
    else if (hcan->Instance == CAN2)
    {
        // CAN2 FIFO0(本工程 CAN2 用 FIFO1，此处预留)
    }
}

/**
 * @brief CAN2 FIFO1 接收回调（GM3508 摩擦轮电机 + 底盘板间通信）
 *
 * CAN2 过滤器绑定到 FIFO1（can_bus_impl.cpp 中配置），
 * 因此使用 HAL_CAN_RxFifo1MsgPendingCallback 而非 FIFO0。
 *
 * 接收数据类型：
 *   - 0x201/0x202: GM3508 摩擦轮电机反馈
 *   - 0x207/0x208: 底盘板间通信（裁判系统数据）
 *
 * @param hcan CAN 句柄
 * @note  同样需要 extern "C"
 */
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN2)
    {
        static auto &can2 = HAL::CAN::get_can_bus_instance().get_can2();
        static auto &board_comm = BoardComm::Gimbal_to_Chassis::Instance();

        HAL::CAN::Frame frame;
        while (can2.receive(frame))
        {
            can2_rx_total++;

            // --- 摩擦轮电机反馈 ---
            if (frame.id == 0x201 || frame.id == 0x202)
            {
                // TODO: 摩擦轮电机解析（后续接入发射状态机）
                if (frame.id == 0x201) can2_rx_id_0x201++;
                else can2_rx_id_0x202++;
            }

            // --- 底盘板间通信 ---
            else if (frame.id == 0x207 || frame.id == 0x208)
            {
                board_comm.HandleCANMessage(frame.id, frame.data, frame.dlc);
            }

            // --- 其他 ID ---
            else
            {
                can2_rx_id_other++;
            }
        }
    }
}
