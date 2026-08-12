/**
 * @file VisionComm.cpp
 * @brief 视觉通信协议实现（RCIA协议，全大端序）
 *
 * 实现内容：
 *   1. Manager::Init()    - 启动 USART6 DMA 空闲中断接收
 *   2. Manager::Parse()   - 解析视觉数据帧（帧头校验 + 大端解码）
 *   3. Manager::Send()    - 打包发送 IMU 四元数 + 弹速 + 状态
 *   4. Manager::IsConnected() - 视觉在线检测（500ms 超时）
 *
 * 协议细节：
 *   - 全大端序（与 gimbal(1).cpp 一致）
 *   - RX 帧：0x39 0x39 + 17字节数据 → 共 19 字节
 *   - TX 帧：0x39 0x39 + 27字节数据 → 共 29 字节
 *   - angle_scale = 100.0（pitch/yaw 整数 = deg × 100）
 *
 * @note 与 VOFA 共享 USART6，自动检测切换
 */

#include "VisionComm.hpp"
#include "../Variable.hpp"          // IMU_Data, BoardComm_Data, VisionComm_Data → Watch 数据
#include "BoardComm.hpp"            // Gimbal_to_Chassis → getLaunchSpeed()

namespace VisionComm
{

// ========================================================================
// Manager::Init() — 启动 DMA 空闲中断接收
// ========================================================================
void Manager::Init()
{
    // 启动 USART6 DMA 空闲中断接收（等待视觉上位机发来 0x39 0x39 帧）
    HAL_UARTEx_ReceiveToIdle_DMA(&VISION_UART, rx_buffer_, RCIA_RX_FRAME_SIZE);
}

// ========================================================================
// Manager::Parse() — 解析视觉数据帧
// ========================================================================
void Manager::Parse(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 仅处理 USART6
    if (huart->Instance != USART6)
    {
        return;
    }

    // 长度校验：必须恰好 19 字节
    if (Size != RCIA_RX_FRAME_SIZE)
    {
        // 长度不匹配 → 可能是噪声或 VOFA 数据 → 重启 DMA
        SlidingWindowRecovery(huart);
        return;
    }

    // 帧头校验
    if (rx_buffer_[0] != RCIA_HEADER || rx_buffer_[1] != RCIA_HEADER)
    {
        SlidingWindowRecovery(huart);
        return;
    }

    // 帧尾校验（第 12 字节偏移 = 2帧头 + 10数据 = byte[12]）
    if (rx_buffer_[12] != RCIA_TAIL)
    {
        // 帧尾不匹配 → 丢弃，重启 DMA
        HAL_UARTEx_ReceiveToIdle_DMA(&VISION_UART, rx_buffer_, RCIA_RX_FRAME_SIZE);
        return;
    }

    // ========== 大端解码 ==========
    // 字节布局（与 gimbal(1).cpp Send() 一致）：
    //   [0-1]:  head (0x39 0x39)
    //   [2-5]:  pitch_i   (int32 BE, deg × angle_scale)
    //   [6-9]:  yaw_i     (int32 BE, deg × angle_scale)
    //   [10]:   vision_ready
    //   [11]:   fire
    //   [12]:   tail (0xFF)
    //   [13-16]: timestamp (uint32 BE, ms)
    //   [17]:   aim_x
    //   [18]:   aim_y

    const int32_t pitch_raw = decode_i32_be(&rx_buffer_[2]);
    const int32_t yaw_raw   = decode_i32_be(&rx_buffer_[6]);

    rx_.pitch_angle  = static_cast<float>(pitch_raw) / ANGLE_SCALE;
    rx_.yaw_angle    = static_cast<float>(yaw_raw)   / ANGLE_SCALE;
    rx_.vision_ready = rx_buffer_[10];
    rx_.fire         = rx_buffer_[11];
    rx_.timestamp    = decode_u32_be(&rx_buffer_[13]);
    rx_.aim_x        = rx_buffer_[17];
    rx_.aim_y        = rx_buffer_[18];
    rx_.online       = 1;

    // 更新时间戳（视觉在线检测用）
    rx_watch_.UpdateLastTime();

    // 同步到 Watch 可观察变量
    VisionComm_Data.pitch_angle  = rx_.pitch_angle;
    VisionComm_Data.yaw_angle    = rx_.yaw_angle;
    VisionComm_Data.pitch_raw    = pitch_raw;
    VisionComm_Data.yaw_raw      = yaw_raw;
    VisionComm_Data.vision_ready = rx_.vision_ready;
    VisionComm_Data.fire         = rx_.fire;
    VisionComm_Data.timestamp    = rx_.timestamp;
    VisionComm_Data.aim_x        = rx_.aim_x;
    VisionComm_Data.aim_y        = rx_.aim_y;
    VisionComm_Data.online       = rx_.online;
    VisionComm_Data.rx_head0     = rx_buffer_[0];
    VisionComm_Data.rx_head1     = rx_buffer_[1];
    VisionComm_Data.rx_tail      = rx_buffer_[12];

    // 重启 DMA 接收（等待下一帧）
    HAL_UARTEx_ReceiveToIdle_DMA(&VISION_UART, rx_buffer_, RCIA_RX_FRAME_SIZE);
}

// ========================================================================
// Manager::Send() — 打包发送视觉数据帧（DMA非阻塞）
// ========================================================================
void Manager::Send()
{
    // DMA 忙检测：上一帧未完成则跳过
    if (tx_busy_)
    {
        tx_skip_count_++;
        return;
    }
    tx_busy_ = true;
    tx_count_++;

    // ========== 打包发送帧（29字节，全大端序）==========
    // 字节布局（与 gimbal(1).cpp read_thread 一致）：
    //   [0-1]:   head (0x39 0x39)
    //   [2-5]:   quat_w     (float BE) — IMU 四元数
    //   [6-9]:   quat_x     (float BE)
    //   [10-13]: quat_y     (float BE)
    //   [14-17]: quat_z     (float BE)
    //   [18-21]: bullet_speed (float BE)
    //   [22]:    enemy_color  (0x52=蓝, 0x42=红)
    //   [23]:    vision_mode
    //   [24]:    tail (0xFF)
    //   [25-28]: timestamp (uint32 BE, ms)

    tx_buffer_[0] = RCIA_HEADER;
    tx_buffer_[1] = RCIA_HEADER;

    // IMU 四元数（大端 float）
    encode_f32_be(&tx_buffer_[2],  IMU_Data.quat_w);
    encode_f32_be(&tx_buffer_[6],  IMU_Data.quat_x);
    encode_f32_be(&tx_buffer_[10], IMU_Data.quat_y);
    encode_f32_be(&tx_buffer_[14], IMU_Data.quat_z);

    // 弹速（大端 float，从裁判系统获取）
    float bullet_speed = BoardComm::Gimbal_to_Chassis::Instance().getLaunchSpeed();
    encode_f32_be(&tx_buffer_[18], bullet_speed);

    // 敌方颜色 / 视觉模式 / 帧尾
    tx_buffer_[22] = ENEMY_COLOR_BLUE;   // 写死蓝色
    tx_buffer_[23] = VISION_MODE_AUTO_AIM;   // 自瞄模式（视觉跑 YOLO + 解算 + 发控制指令）
    tx_buffer_[24] = RCIA_TAIL;

    // 时间戳（大端 uint32, ms）
    encode_u32_be(&tx_buffer_[25], HAL_GetTick());

    // DMA 发送（USART6，非阻塞）
    HAL_UART_Transmit_DMA(&VISION_UART, tx_buffer_, RCIA_TX_FRAME_SIZE);
}

// ========================================================================
// Manager::IsConnected() — 视觉在线检测
// ========================================================================
bool Manager::IsConnected()
{
    rx_watch_.UpdateTime();
    rx_watch_.CheckStatus();
    bool online = (rx_watch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE);

    // 离线时清零 online 标志
    if (!online)
    {
        rx_.online = 0;
        VisionComm_Data.online = 0;
    }

    return online;
}

// ========================================================================
// Manager::TxComplete() — DMA 发送完成回调
// ========================================================================
void Manager::TxComplete()
{
    tx_busy_ = false;
}

// ========================================================================
// Manager::SlidingWindowRecovery() — 滑动窗口恢复帧同步
// ========================================================================
void Manager::SlidingWindowRecovery(UART_HandleTypeDef *huart)
{
    // 在 buffer 中查找 0x39 0x39，将后续数据前移对齐帧头
    for (uint16_t i = 0; i < RCIA_RX_FRAME_SIZE - 1; ++i)
    {
        if (rx_buffer_[i] == RCIA_HEADER && rx_buffer_[i + 1] == RCIA_HEADER)
        {
            // 找到帧头 → 前移数据
            uint16_t remaining = RCIA_RX_FRAME_SIZE - i;
            if (remaining > 0)
            {
                std::memmove(rx_buffer_, &rx_buffer_[i], remaining);
            }
            // 重启 DMA 接收剩余字节
            HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer_ + remaining,
                                         RCIA_RX_FRAME_SIZE - remaining);
            return;
        }
    }

    // 未找到帧头 → 清空缓冲区，重新开始
    std::memset(rx_buffer_, 0, RCIA_RX_FRAME_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer_, RCIA_RX_FRAME_SIZE);
}

} // namespace VisionComm
