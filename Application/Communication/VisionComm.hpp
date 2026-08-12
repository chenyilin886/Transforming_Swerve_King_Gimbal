/**
 * @file VisionComm.hpp
 * @brief 视觉通信协议（RCIA协议，全大端序）
 *
 * 设计原因：
 *   三关节可变形云台需要与视觉上位机（gimbal(1).cpp）通过串口通信：
 *   - 电控→视觉：发送IMU四元数/弹速/敌方颜色/模式/时间戳（29字节）
 *   - 视觉→电控：接收目标yaw/pitch角度/视觉就绪/开火信号（19字节）
 *
 * 协议来源：
 *   继承自 gimbal(1).cpp（RCIA/RCIA_infantry协议），全大端序，
 *   替换参考工程 User_reference/Vision 的大端/小端混用方案。
 *
 * USART6 共享策略（VOFA + 视觉共存）：
 *   - RX 始终监听 0x39 0x39 帧头
 *   - 收到视觉帧 → vision_connected_ = true → TX 发 RCIA 帧（200Hz）
 *   - 500ms 无视觉帧 → vision_connected_ = false → TX 恢复 VOFA（500Hz）
 *   - 插上视觉上位机自动切换，拔掉自动恢复，无需手动干预
 *
 * 数据流（发送）：
 *   GimbalUpdate (200Hz降频)
 *     └─ VisionComm::Send()
 *          ├─ 读 IMU_Data.quat_w/x/y/z → encode_f32_be
 *          ├─ 读 BoardComm.getLaunchSpeed() → bullet_speed (float BE)
 *          ├─ 填 enemy_color (0x52蓝) / vision_mode / tail (0xFF)
 *          ├─ 填 timestamp (HAL_GetTick, uint32 BE)
 *          └─ HAL_UART_Transmit_DMA(&huart6, 29字节)
 *
 * 数据流（接收）：
 *   USART6 空闲中断 → HAL_UARTEx_RxEventCallback (UartCallback.cpp)
 *     └─ VisionComm::Parse(huart, Size)
 *          ├─ 扫描帧头 0x39 0x39（滑动窗口恢复同步）
 *          ├─ 校验 tail == 0xFF（不匹配则丢弃）
 *          ├─ decode_i32_be → pitch_angle / yaw_angle (deg)
 *          ├─ vision_ready / fire / timestamp / aim_x / aim_y
 *          ├─ 更新 StateWatch（标记视觉在线）
 *          └─ 写入 VisionComm_Data → Watch 观察
 *
 * CubeMX 配置（已完成，无需修改）：
 *   - USART6: 115200bps, 8N1, TX=PG14, RX=PG9
 *   - DMA RX: DMA2_Stream1, DMA TX: DMA2_Stream6
 *   - NVIC: USART6_IRQn, priority 5
 *
 * @note 全大端序，与 gimbal(1).cpp 一致
 *       angle_scale = 100.0（视觉发来的 int32 / 100 = 度数）
 */

#pragma once

#include "main.h"
#include "usart.h"
#include "state_watch.hpp"
#include <cstdint>
#include <cstring>

/// @brief 视觉通信专用串口（与 VOFA 共享 USART6）
#define VISION_UART huart6

namespace VisionComm
{

// ========================================================================
// 协议常量（与 gimbal(1).cpp 一致）
// ========================================================================
constexpr uint8_t  RCIA_HEADER       = 0x39;   ///< 帧头
constexpr uint8_t  RCIA_TAIL         = 0xFF;   ///< 帧尾
constexpr uint8_t  RCIA_RX_DATA_SIZE = 17;     ///< 视觉→电控 数据区（不含帧头）
constexpr uint8_t  RCIA_TX_DATA_SIZE = 27;     ///< 电控→视觉 数据区（不含帧头）
constexpr uint8_t  RCIA_RX_FRAME_SIZE = 2 + RCIA_RX_DATA_SIZE;  ///< RX 帧总长 19
constexpr uint8_t  RCIA_TX_FRAME_SIZE = 2 + RCIA_TX_DATA_SIZE;  ///< TX 帧总长 29
constexpr uint8_t  ENEMY_COLOR_BLUE  = 0x52;  ///< 我方蓝色（0x42=红色）
constexpr uint8_t  VISION_MODE_IDLE    = 0;     ///< 待机（视觉不跑识别）
constexpr uint8_t  VISION_MODE_AUTO_AIM = 1;  ///< 自瞄模式（视觉跑 YOLO + 解算）
constexpr uint8_t  VISION_MODE_SMALL    = 2;  ///< 小符模式
constexpr uint8_t  VISION_MODE_BIG      = 3;  ///< 大符模式
constexpr float    ANGLE_SCALE       = 100.0f; ///< 角度缩放（与 gimbal(1).cpp 一致）
constexpr uint32_t VISION_TIMEOUT_MS = 500;   ///< 视觉离线超时（500ms 无数据 → 切回 VOFA）

// ========================================================================
// 大端序编解码辅助函数（inline，与 gimbal(1).cpp 一致）
// ========================================================================

/// @brief 大端解码 int32
inline int32_t decode_i32_be(const uint8_t *data)
{
    const uint32_t value = (static_cast<uint32_t>(data[0]) << 24) |
                           (static_cast<uint32_t>(data[1]) << 16) |
                           (static_cast<uint32_t>(data[2]) << 8) |
                           static_cast<uint32_t>(data[3]);
    return static_cast<int32_t>(value);
}

/// @brief 大端解码 uint32
inline uint32_t decode_u32_be(const uint8_t *data)
{
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) |
           static_cast<uint32_t>(data[3]);
}

/// @brief 大端编码 uint32
inline void encode_u32_be(uint8_t *data, uint32_t value)
{
    data[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(value & 0xFF);
}

/// @brief 大端编码 float（通过 uint32 中转，避免依赖 CPU 端序）
inline void encode_f32_be(uint8_t *data, float value)
{
    uint32_t raw;
    std::memcpy(&raw, &value, sizeof(raw));
    encode_u32_be(data, raw);
}

// ========================================================================
// 接收数据结构（视觉→电控，解析后）
// ========================================================================

/**
 * @brief 视觉接收数据（Watch 可观察）
 *
 * @note pitch_angle / yaw_angle 单位：度（deg）
 *       视觉发来的 int32 = deg × angle_scale（默认×100），
 *       本端 decode_i32_be 后除以 angle_scale 得到度数
 */
struct RxData
{
    float    pitch_angle;    ///< Pitch 目标角度(deg)，绝对角度
    float    yaw_angle;      ///< Yaw 目标角度(deg)，绝对角度
    uint8_t  vision_ready;   ///< 视觉就绪标志（0/1）
    uint8_t  fire;           ///< 开火信号（0/1）
    uint32_t timestamp;      ///< 视觉时间戳(ms)
    uint8_t  aim_x;          ///< 瞄准点 X（预留）
    uint8_t  aim_y;          ///< 瞄准点 Y（预留）
    uint8_t  online;         ///< 视觉在线状态（0=离线, 1=在线）
};

// ========================================================================
// 视觉通信管理类
// ========================================================================

/**
 * @brief 视觉通信管理器（RCIA协议）
 *
 * 职责：
 *   1. 打包/发送 IMU 四元数 + 弹速 + 状态 到视觉上位机（TX, 29字节, DMA）
 *   2. 接收/解析视觉目标角度 + 控制信号（RX, 19字节, DMA空闲中断）
 *   3. 在线检测（500ms超时），自动切换 VOFA/视觉
 *
 * 单例模式（全局唯一实例）。
 */
class Manager
{
public:
    // 禁止拷贝
    Manager(const Manager &) = delete;
    Manager &operator=(const Manager &) = delete;

    /// @brief 获取单例实例
    static Manager &Instance()
    {
        static Manager instance;
        return instance;
    }

    /**
     * @brief 初始化视觉通信
     *
     * 启动 USART6 DMA 空闲中断接收，等待视觉上位机发送数据。
     *
     * @note 在 GimbalInit() 中调用，必须在 MX_USART6_UART_Init() 之后
     */
    void Init();

    /**
     * @brief 解析接收到的视觉数据
     *
     * @param huart UART 句柄（应为 &huart6）
     * @param Size  接收到的数据长度（应等于 19）
     *
     * 在 HAL_UARTEx_RxEventCallback 中调用。
     * 内部完成帧头校验（0x39 0x39）、帧尾校验（0xFF）、
     * 大端解码、StateWatch 更新时间戳。
     *
     * @note 帧头不匹配时自动滑动窗口恢复
     */
    void Parse(UART_HandleTypeDef *huart, uint16_t Size);

    /**
     * @brief 打包并发送视觉数据帧（29字节，DMA非阻塞）
     *
     * 发送内容：
     *   - IMU 四元数 (quat_w/x/y/z, float BE)
     *   - 弹速 (bullet_speed, float BE)
     *   - 敌方颜色 (0x52蓝) / 视觉模式 / 帧尾 (0xFF)
     *   - 时间戳 (uint32 BE, HAL_GetTick)
     *
     * 发送前检查 DMA 忙标志，若上一帧未完成则跳过。
     *
     * @note 调用频率：200Hz（GimbalUpdate 中降频控制）
     */
    void Send();

    /**
     * @brief 检测视觉上位机是否在线
     * @return true=在线（最近 500ms 内收到过数据），false=离线
     *
     * 用于自动切换 VOFA/视觉模式：
     *   - IsConnected() == true  → TX 发 RCIA 帧
     *   - IsConnected() == false → TX 发 VOFA 帧（原有行为）
     */
    bool IsConnected();

    /// @brief 获取接收数据（只读引用）
    const RxData &GetRxData() const { return rx_; }

    /// @brief TX DMA 是否忙
    bool IsTxBusy() const { return tx_busy_; }

    /// @brief 获取发送计数（Watch 验证发送是否正常递增）
    uint32_t GetTxCount() const { return tx_count_; }

    /// @brief 获取发送失败（DMA忙跳过）计数
    uint32_t GetTxSkipCount() const { return tx_skip_count_; }

    /// @brief TX DMA 完成回调（在 HAL_UART_TxCpltCallback 中调用）
    void TxComplete();

private:
    /// @brief 私有构造函数（单例）
    Manager()
        : rx_watch_(VISION_TIMEOUT_MS)
        , tx_busy_(false)
    {
        std::memset(rx_buffer_, 0, sizeof(rx_buffer_));
        std::memset(tx_buffer_, 0, sizeof(tx_buffer_));
        rx_ = RxData{};
    }

    /// @brief 在 rx_buffer_ 中滑动窗口查找 0x39 0x39 帧头，前移对齐后重启 DMA
    void SlidingWindowRecovery(UART_HandleTypeDef *huart);

    // --- 缓冲区 ---
    uint8_t  rx_buffer_[RCIA_RX_FRAME_SIZE];  ///< DMA 接收缓冲区（19字节）
    uint8_t  tx_buffer_[RCIA_TX_FRAME_SIZE];  ///< 发送缓冲区（29字节）

    // --- 接收数据 ---
    RxData   rx_;                              ///< 解析后的接收数据

    // --- 状态 ---
    BSP::WATCH_STATE::StateWatch rx_watch_;    ///< 视觉在线检测（500ms超时）
    volatile bool           tx_busy_;          ///< TX DMA 忙标志
    uint32_t                tx_count_ = 0;     ///< 成功启动DMA发送的帧数（Watch验证用）
    uint32_t                tx_skip_count_ = 0; ///< 因DMA忙跳过的帧数
};

} // namespace VisionComm
