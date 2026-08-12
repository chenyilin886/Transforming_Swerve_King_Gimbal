/**
 * @file Vofa.hpp
 * @brief VOFA+ JustFloat串口协议发送类
 *
 * 用于通过串口发送float数据到VOFA+上位机，实时观察PID波形。
 * 继承旧工程 vofa.c/h 的实现逻辑，适配C++风格。
 *
 * 协议说明（JustFloat）：
 * - 帧格式：N个float数据 + 尾帧0x7F800000
 * - 每帧固定发送6个float（24字节）+ 尾帧（4字节）= 28字节
 * - 尾帧0x7F800000 = IEEE754正无穷大，用作帧结束标识
 * - VOFA+收到尾帧后开始渲染波形
 *
 * 数据流：
 *   PID数据 → Vofa.Send6Floats() → DMA发送 → VOFA+显示
 *
 * 使用方式：
 * - 在控制任务中周期调用 Vofa.Send6Floats(data)
 * - 发送内容可配置：目标值、反馈值、误差、输出等
 * - DMA忙检测：上一帧未完成时不发送新帧
 *
 * 硬件配置：
 * - USART6：115200波特率，8位数据位，无校验
 * - DMA TX已配置，非阻塞发送
 *
 * @note 仅实现发送功能，后续按需扩展接收（FireWater协议）在线调参
 */
#pragma once

#include "usart.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief VOFA DMA发送完成回调
 *
 * 在 HAL_UART_TxCpltCallback 中调用，清除DMA忙标志。
 * extern "C" 确保C回调可调用。
 */
extern void Vofa_TxComplete(void);

#ifdef __cplusplus
}
#endif

namespace APP
{

/**
 * @brief VOFA+ JustFloat协议发送类
 *
 * 封装JustFloat协议发送逻辑，提供简单接口供上层调用。
 * DMA非阻塞发送，不影响控制实时性。
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
     * @brief 发送6个float数据到VOFA+
     * @param x1~x6 要发送的6个float数据
     *
     * JustFloat协议帧格式：
     * - 数据：6个float（24字节）
     * - 尾帧：0x7F800000（4字节）
     * - 总帧长：28字节
     *
     * DMA忙检测：
     * - 若上一帧还没发完（busy_=true），直接返回不发新帧
     * - 防止数据错乱和DMA冲突
     *
     * @note 调用频率建议：≤500Hz（避免串口带宽饱和）
     *       实际1kHz调用也OK，因为忙时会跳过发送
     */
    void Send6Floats(float x1, float x2, float x3, float x4, float x5, float x6);

    /**
     * @brief 检查DMA是否正在发送
     * @retval true=忙（上一帧未完成），false=空闲
     *
     * 上层可据此决定是否发送新帧。
     */
    bool IsBusy() const { return busy_; }

    /**
     * @brief DMA发送完成回调
     *
     * 在 HAL_UART_TxCpltCallback 中调用，清除忙标志。
     * 准备发送下一帧。
     */
    void TxComplete();

  private:
    // 发送缓冲区：6个float + 尾帧（共28字节）
    static constexpr uint8_t kFrameSize = sizeof(float) * 6 + sizeof(uint32_t);
    uint8_t tx_buffer_[kFrameSize];

    // DMA忙标志：true=正在发送，false=空闲
    volatile bool busy_;

    // JustFloat尾帧：0x7F800000（IEEE754正无穷大）
    static constexpr uint32_t kTailFrame = 0x7F800000;
};

// 全局VOFA实例
extern Vofa_t Vofa;

} // namespace APP