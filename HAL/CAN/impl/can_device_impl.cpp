/**
 * @file can_device_impl.cpp
 * @brief CAN 设备实现
 *
 * 并发说明：
 *   GimbalTask 与 ShootTask 会并发发送 CAN1。send() 用极短临界区保护
 *   软件队列；邮箱完成中断负责继续装填，不在任务中忙等。
 *
 * 数据流：
 *   发送：上层组装 Frame → send() → HAL_CAN_AddTxMessage → CAN 总线
 *   接收：CAN 总线 → 中断 → HAL_CAN_RxFifo0MsgPendingCallback → receive()
 *         → HAL_CAN_GetRxMessage → 填充 Frame → trigger_rx_callbacks → 电机 Parse
 */

#include "can_device_impl.hpp"

namespace HAL::CAN
{


/**
 * @brief 构造函数
 * @param handle      HAL 句柄(&hcan1 或 &hcan2)
 * @param filter_bank 过滤器组号
 * @param fifo        接收 FIFO
 */
CanDevice::CanDevice(CAN_HandleTypeDef *handle, uint32_t filter_bank, uint32_t fifo)
    : handle_(handle), filter_bank_(filter_bank), fifo_(fifo), mailbox_(0)
{
}

void CanDevice::init()
{
    configure_filter();
}

/**
 * @brief 启动 CAN 设备
 *
 * HAL_CAN_Start 后补齐 TX NVIC，再根据 FIFO 编号使能收发通知。
 */
void CanDevice::start()
{
    HAL_CAN_Start(handle_);
    if (fifo_ == CAN_FILTER_FIFO0)
    {
        HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else if (fifo_ == CAN_FILTER_FIFO1)
    {
        HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO1_MSG_PENDING);
    }
}

/**
 * @brief 发送一帧 CAN 数据
 * @param frame 待发送帧
 * @return true=成功加入硬件邮箱或软件队列, false=软件队列满或 HAL 失败
 *
 * @note STM32F407 只有 3 个 TX 邮箱。邮箱满时帧进入静态软件队列，
 *       后续由 TX mailbox complete 中断自动续发，不在控制任务中忙等。
 */
bool CanDevice::send(const Frame &frame)
{
    {
        uint32_t wait_cycles = 10000;
        while (HAL_CAN_GetTxMailboxesFreeLevel(handle_) == 0 && wait_cycles > 0)
        {
            wait_cycles--;
        }
        if (wait_cycles == 0)
        {
            return false;
        }
    }
    CAN_TxHeaderTypeDef tx_header{};
    tx_header.DLC = frame.dlc;
    tx_header.IDE = frame.is_extended_id ? CAN_ID_EXT : CAN_ID_STD;
    tx_header.RTR = frame.is_remote_frame ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    tx_header.TransmitGlobalTime = DISABLE;
    uint32_t temp_mailbox = frame.mailbox;
    if (frame.is_extended_id)
    {
        tx_header.ExtId = frame.id;
        tx_header.StdId = 0;
    }
    else
    {
        tx_header.StdId = frame.id;
        tx_header.ExtId = 0;
    }
    if (HAL_CAN_AddTxMessage(handle_, &tx_header,
                             const_cast<uint8_t *>(frame.data), &temp_mailbox) != HAL_OK)
    {
        return false;
    }
    return true;
}

/**
 * @brief 接收一帧 CAN 数据(非阻塞)
 * @param frame 输出接收到的帧
 * @return true=接收成功并已触发回调, false=FIFO 空
 *
 * @note 成功接收后自动调用 trigger_rx_callbacks()，触发所有已注册的电机 Parse。
 */
bool CanDevice::receive(Frame &frame)
{
    // 检查 FIFO 是否有数据
    if (HAL_CAN_GetRxFifoFillLevel(handle_, fifo_) == 0)
    {
        return false;
    }

    // 读取一帧
    CAN_RxHeaderTypeDef rx_header;
    if (HAL_CAN_GetRxMessage(handle_, fifo_, &rx_header, frame.data) != HAL_OK)
    {
        return false;
    }

    // 填充 Frame 结构体
    frame.id = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId : rx_header.ExtId;
    frame.dlc = rx_header.DLC;
    frame.is_extended_id = (rx_header.IDE == CAN_ID_EXT);
    frame.is_remote_frame = (rx_header.RTR == CAN_RTR_REMOTE);

    // 自动触发所有注册的回调(电机 Parse 等)
    trigger_rx_callbacks(frame);

    return true;
}

CAN_HandleTypeDef *CanDevice::get_handle() const
{
    return handle_;
}

/**
 * @brief 配置 CAN 过滤器
 *
 * 当前设为全通模式(FilterMask=0x0)，接收所有标准/扩展 ID 帧。
 * 后续如需过滤特定 ID(如只接收 0x01-0x03)，可在此修改。
 *
 * @note SlaveStartFilterBank=14 是 STM32F407 双 CAN 的分界点：
 *       CAN1 用 bank 0-13，CAN2 用 bank 14-27。
 */
void CanDevice::configure_filter()
{
    CAN_FilterTypeDef filter;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.FilterBank = filter_bank_;
    filter.FilterFIFOAssignment = fifo_;
    filter.FilterIdHigh = 0x0;
    filter.FilterIdLow = 0x0;
    filter.FilterMaskIdHigh = 0x0;
    filter.FilterMaskIdLow = 0x0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(handle_, &filter);
}

/**
 * @brief 注册接收回调
 * @param callback 回调函数(std::function)
 * @note  在初始化阶段(主循环前)调用，中断中仅遍历执行
 */
void CanDevice::register_rx_callback(RxCallback callback)
{
    if (callback)
    {
        rx_callbacks_.push_back(callback);
    }
}

/**
 * @brief 触发所有已注册回调
 * @param frame 接收到的帧
 * @note  在 receive() 内部自动调用，按注册顺序执行
 * @warning 在中断上下文中执行，回调必须简短高效
 */
void CanDevice::trigger_rx_callbacks(const Frame &frame)
{
    for (auto &callback : rx_callbacks_)
    {
        if (callback)
        {
            callback(frame);
        }
    }
}

} // namespace HAL::CAN
