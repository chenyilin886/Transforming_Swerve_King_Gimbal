/**
 * @file Vofa.hpp
 * @brief VOFA+ JustFloat 串口协议发送类
 *
 * 用途：
 *   通过串口发送 float 数据到 VOFA+ 上位机，实时观察 PID 波形。
 *   用于辅助 Pitch PID 调参，可在 VOFA+ 中绘制 target/feedback/error 等波形。
 *
 * 协议说明（JustFloat）：
 *   - 帧格式：N 个 float 数据 + 尾帧 0x7F800000
 *   - 每帧固定发送 6 个 float（24 字节）+ 尾帧（4 字节）= 28 字节
 *   - 尾帧 0x7F800000 = IEEE754 正无穷大，用作帧结束标识
 *   - VOFA+ 收到尾帧后开始渲染波形
 *
 * 数据流：
 *   调用点(GimbalUpdate) → Vofa.Send6Floats() → DMA 发送 → VOFA+ 显示
 *
 * 使用方式：
 *   - 在 GimbalUpdate 中周期调用 Vofa.Send6Floats(...)
 *   - 调用频率建议 ≤500Hz（避免串口带宽饱和）
 *   - DMA 忙检测：上一帧未完成时不发送新帧
 *
 * 硬件配置（CubeMX 已完成）：
 *   - USART6：115200 波特率，8 位数据位，无校验
 *   - GPIO：PG14=TX, PG9=RX
 *   - DMA TX：DMA2_Stream6, Channel5, Memory→Peripheral, Normal 模式
 *   - 中断：DMA2_Stream6_IRQn + USART6_IRQn 均已使能
 *
 * @note 当前仅实现发送功能，后续按需扩展接收（FireWater 协议）在线调参
 */
#ifndef APP_VOFA_HPP
#define APP_VOFA_HPP

#include "usart.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief VOFA DMA 发送完成回调
 *
 * 在 HAL_UART_TxCpltCallback 中调用，清除 DMA 忙标志。
 * extern "C" 确保C回调可调用。
 */
extern void Vofa_TxComplete(void);

#ifdef __cplusplus
}
#endif

namespace APP
{

/**
 * @brief VOFA+ JustFloat 协议发送类
 *
 * 封装 JustFloat 协议发送逻辑，提供简单接口供上层调用。
 * DMA 非阻塞发送，不影响控制实时性。
 */
class Vofa_t
{
public:
    /**
     * @brief 构造函数
     * 初始化发送缓冲区和状态
     */
    Vofa_t();

    /**
     * @brief 发送 6 个 float 数据到 VOFA+
     *
     * JustFloat 协议帧格式：
     *   - 数据：6 个 float（24 字节）
     *   - 尾帧：0x7F800000（4 字节）
     *   - 总帧长：28 字节
     *
     * DMA 忙检测：
     *   - 若上一帧还没发完（busy_=true），直接返回不发新帧
     *   - 防止数据错乱和 DMA 冲突
     *
     * @param x1~x6 要发送的 6 个 float 数据
     *
     * @note 调用频率建议：≤500Hz（避免串口带宽饱和）
     *       实际 1kHz 调用也 OK，因为忙时会跳过发送
     */
    void Send6Floats(float x1, float x2, float x3, float x4, float x5, float x6);

    /**
     * @brief 检查 DMA 是否正在发送
     * @retval true=忙（上一帧未完成），false=空闲
     */
    bool IsBusy() const { return busy_; }

    /**
     * @brief DMA 发送完成回调
     *
     * 在 HAL_UART_TxCpltCallback 中调用，清除忙标志。
     * 准备发送下一帧。
     */
    void TxComplete();

private:
    /// 发送缓冲区大小：6 个 float + 尾帧（共 28 字节）
    static constexpr uint8_t kFrameSize = sizeof(float) * 6 + sizeof(uint32_t);

    /// 发送缓冲区
    uint8_t tx_buffer_[kFrameSize];

    /// DMA 忙标志：true=正在发送，false=空闲
    volatile bool busy_;

    /// JustFloat 尾帧：0x7F800000（IEEE754 正无穷大）
    static constexpr uint32_t kTailFrame = 0x7F800000;
};

/// 全局 VOFA 实例
extern Vofa_t Vofa;

} // namespace APP

#endif // APP_VOFA_HPP
