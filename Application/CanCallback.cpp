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
 *           → read_can_fifo()  [direct FIFO read]
 *             → HAL_CAN_GetRxMessage 提取数据
 *             → dispatch_can1_frame(frame)
 *               → DM / LK / DJI Parse by CAN ID
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
#include "DmMotor.hpp"
#include "LkMotor.hpp"
#include "DjiMotor.hpp"

// ========================================================================
// 调试计数器：在 Keil Watch 中观察这些变量，验证 DM 电机是否持续回复
// ========================================================================
// 验证方法：
//   DM 默认 Master ID=0，因此 can1_rx_id_0x00 应持续增长；具体电机需看
//   data[0] 中的 ID，Fold 对应 can1_rx_dm_fold_raw。
//
// 【使能/失能问题诊断】
//   观察 can1_rx_id_0x01/0x02/0x03 在使能前后的变化：
//   - 使能前（失能状态）：观察是否有反馈帧到达
//   - 使能后：观察反馈帧的CAN ID是什么（0x00还是0x01/0x02/0x03）
//   - 如果使能后计数停止增长 → CAN总线或电机固件问题
volatile uint32_t can1_rx_total      = 0;  // CAN1 接收中断总次数
volatile uint32_t can1_rx_last_id    = 0;  // 最后收到的帧 ID
volatile uint32_t can1_rx_id_0x00    = 0;  // DM feedback using Master ID 0
volatile uint32_t can1_rx_id_0x01    = 0;  // ID=0x01 帧计数(预留/旧 Yaw)
volatile uint32_t can1_rx_id_0x02    = 0;  // ID=0x02 帧计数(DM4310 Pitch)
volatile uint32_t can1_rx_id_0x03    = 0;  // ID=0x03 帧计数(DM4340 Fold)
volatile uint32_t can1_rx_id_0x04    = 0;  // ID=0x04 帧计数(DM4310 Yaw)
volatile uint32_t can1_rx_id_0x141   = 0;  // ID=0x141 帧计数(LK4005)
volatile uint32_t can1_rx_id_0x201   = 0;  // ID=0x201 帧计数(GM3508 #1)
volatile uint32_t can1_rx_id_0x202   = 0;  // ID=0x202 帧计数(GM3508 #2)
volatile uint32_t can1_rx_id_other   = 0;  // 其他 ID 帧计数
volatile uint32_t can1_rx_dm_fold_raw = 0; // data[0] 电机 ID=3 的原始 DM 帧
volatile uint32_t can1_rx_fifo0_full = 0;
volatile uint32_t can1_rx_fifo0_overrun = 0;
volatile uint32_t can1_error_callback_count = 0;
volatile uint32_t can1_last_error = HAL_CAN_ERROR_NONE;
volatile uint32_t can1_bus_off_count = 0;

// ========================================================================
// CAN2 调试计数器：当前用于板间通信；GM3508 实例实际挂在 CAN1
// ========================================================================
volatile uint32_t can2_rx_total      = 0;  // CAN2 接收中断总次数
volatile uint32_t can2_rx_id_0x201   = 0;  // ID=0x201 帧计数(GM3508 #1)
volatile uint32_t can2_rx_id_0x202   = 0;  // ID=0x202 帧计数(GM3508 #2)
volatile uint32_t can2_rx_id_other   = 0;  // 其他 ID 帧计数

static bool read_can_fifo(CAN_HandleTypeDef *hcan, uint32_t fifo, HAL::CAN::Frame &frame)
{
    if (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) == 0U)
    {
        return false;
    }

    CAN_RxHeaderTypeDef rx_header{};
    if (HAL_CAN_GetRxMessage(hcan, fifo, &rx_header, frame.data) != HAL_OK)
    {
        return false;
    }

    frame.id = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId : rx_header.ExtId;
    frame.dlc = rx_header.DLC;
    frame.mailbox = 0U;
    frame.is_extended_id = (rx_header.IDE == CAN_ID_EXT);
    frame.is_remote_frame = (rx_header.RTR == CAN_RTR_REMOTE);
    return true;
}

static void dispatch_can1_frame(const HAL::CAN::Frame &frame)
{
    if (frame.is_extended_id || frame.is_remote_frame || frame.dlc != 8U)
    {
        return;
    }

    switch (frame.id)
    {
        case 0x201:
        case 0x202:
            if (BSP::MOTOR::DJI::motor_3508 != nullptr)
            {
                BSP::MOTOR::DJI::motor_3508->Parse(frame);
            }
            break;

        case 0x141:
            if (BSP::MOTOR::LK::lk4005_motor != nullptr)
            {
                BSP::MOTOR::LK::lk4005_motor->Parse(frame);
            }
            break;

        case 0x00:
        case 0x01:
        case 0x02:
        case 0x03:
        case 0x04:
            if (BSP::MOTOR::DM::dm4310_yaw_pitch != nullptr)
            {
                BSP::MOTOR::DM::dm4310_yaw_pitch->Parse(frame);
            }
            if (BSP::MOTOR::DM::dm4340_fold != nullptr)
            {
                BSP::MOTOR::DM::dm4340_fold->Parse(frame);
            }
            break;

        default:
            break;
    }
}
/**
 * @brief CAN1 FIFO0 接收回调
 *
 * 当 CAN1 FIFO0 收到消息时由 HAL 触发。
 * 直接读取 FIFO，并按 CAN ID 定向调用对应电机 Parse。
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

        // 【关键修复】循环读取直到 FIFO 清空
        //   STM32F4 CAN RX FIFO 深度为 3，中断在 FMP 从 0→非零 时触发一次。
        //   三台电机同时回传时 FIFO 可能堆积多帧，只读 1 帧会导致剩余帧
        //   卡在 FIFO（FMP 3→2 不触发新中断），最终溢出丢帧。
        HAL::CAN::Frame frame{};
        while (read_can_fifo(hcan, CAN_RX_FIFO0, frame))
        {
            // --- 调试统计(中断上下文, 仅简单自增, 安全) ---
            can1_rx_total++;
            can1_rx_last_id = frame.id;
            if (!frame.is_extended_id && !frame.is_remote_frame &&
                frame.dlc == 8U && frame.id <= 0x0FU &&
                ((frame.data[0] >> 4) & 0x0FU) == 0x03U)
            {
                ++can1_rx_dm_fold_raw;
            }
            switch (frame.id)
            {
                case 0x00:  can1_rx_id_0x00++;  break;
                case 0x01:  can1_rx_id_0x01++;  break;
                case 0x02:  can1_rx_id_0x02++;  break;
                case 0x03:  can1_rx_id_0x03++;  break;
                case 0x04:  can1_rx_id_0x04++;  break;
                case 0x141: can1_rx_id_0x141++; break;
                case 0x201: can1_rx_id_0x201++; break;  // GM3508 #1
                case 0x202: can1_rx_id_0x202++; break;  // GM3508 #2
                default:    can1_rx_id_other++; break;
            }

            dispatch_can1_frame(frame);
        }
    }
    else if (hcan->Instance == CAN2)
    {
        // CAN2 FIFO0(本工程 CAN2 用 FIFO1，此处预留)
    }
}

extern "C" void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        ++can1_rx_fifo0_full;
    }
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1)
    {
        return;
    }

    const uint32_t error = HAL_CAN_GetError(hcan);
    ++can1_error_callback_count;
    can1_last_error = error;
    if ((error & HAL_CAN_ERROR_RX_FOV0) != 0U)
    {
        ++can1_rx_fifo0_overrun;
    }
    if ((error & HAL_CAN_ERROR_BOF) != 0U)
    {
        ++can1_bus_off_count;
    }

    // HAL accumulates ErrorCode. Counters retain the diagnosis; clear it so
    // one old overrun is not counted again on every later error callback.
    hcan->ErrorCode = HAL_CAN_ERROR_NONE;
}

/**
 * @brief CAN2 FIFO1 接收回调（当前为底盘板间通信）
 *
 * CAN2 过滤器绑定到 FIFO1（can_bus_impl.cpp 中配置），
 * 因此使用 HAL_CAN_RxFifo1MsgPendingCallback 而非 FIFO0。
 *
 * 接收数据类型：0x207/0x208 底盘板间通信。
 * 0x201/0x202 分支仅保留兼容性，当前 GM3508 注册在 CAN1。
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
