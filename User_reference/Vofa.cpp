/**
 * @file Vofa.cpp
 * @brief VOFA+ JustFloat串口协议发送实现
 *
 * 继承旧工程 vofa.c 的实现逻辑，适配C++风格。
 * 使用USART6 DMA发送，非阻塞，不影响控制实时性。
 *
 * JustFloat协议帧格式：
 * - 数据：6个float（24字节），小端序（STM32原生）
 * - 尾帧：0x7F800000（4字节），帧结束标识
 * - 总帧长：28字节
 *
 * DMA发送流程：
 * 1. 检查忙标志（busy_），若忙则跳过
 * 2. 将6个float写入缓冲区
 * 3. 写入尾帧0x7F800000
 * 4. 启动DMA发送（HAL_UART_Transmit_DMA）
 * 5. 等待DMA完成中断（HAL_UART_TxCpltCallback）
 * 6. 回调中清除忙标志
 *
 * @note 调用频率建议≤500Hz，实际1kHz调用OK（忙时跳过）
 */
#include "Vofa.hpp"
#include <cstring>

namespace APP
{

// 全局VOFA实例
Vofa_t Vofa;

Vofa_t::Vofa_t()
    : busy_(false)
{
    // 初始化缓冲区为0
    memset(tx_buffer_, 0, kFrameSize);
}

void Vofa_t::Send6Floats(float x1, float x2, float x3, float x4, float x5, float x6)
{
    // DMA忙检测：上一帧未完成则跳过
    if (busy_)
    {
        return;
    }
    busy_ = true;

    // 计算float大小（4字节）
    const uint8_t float_size = sizeof(float);

    // 打包6个float数据到缓冲区（小端序，STM32原生）
    // 使用memcpy避免对齐问题，直接拷贝float的二进制表示
    memcpy(&tx_buffer_[float_size * 0], &x1, float_size);
    memcpy(&tx_buffer_[float_size * 1], &x2, float_size);
    memcpy(&tx_buffer_[float_size * 2], &x3, float_size);
    memcpy(&tx_buffer_[float_size * 3], &x4, float_size);
    memcpy(&tx_buffer_[float_size * 4], &x5, float_size);
    memcpy(&tx_buffer_[float_size * 5], &x6, float_size);

    // 打包尾帧：0x7F800000（JustFloat帧结束标识）
    // 尾帧位置：第6个float之后，即 float_size * 6
    memcpy(&tx_buffer_[float_size * 6], &kTailFrame, sizeof(uint32_t));

    // 启动DMA发送（USART6，非阻塞）
    // 总帧长：6个float + 尾帧 = 28字节
    HAL_UART_Transmit_DMA(&huart6, tx_buffer_, kFrameSize);
}

void Vofa_t::TxComplete()
{
    // DMA发送完成，清除忙标志
    busy_ = false;
}

} // namespace APP

/**
 * @brief C接口：VOFA DMA发送完成回调
 *
 * 在 stm32f4xx_it.c 或 usart.c 的 HAL_UART_TxCpltCallback 中调用。
 * extern "C" 确保C回调可调用C++函数。
 */
extern "C" void Vofa_TxComplete(void)
{
    APP::Vofa.TxComplete();
}