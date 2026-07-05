/**
 * @file UartCallback.cpp
 * @brief UART 中断回调实现
 *
 * 设计原因：
 *   STM32 HAL 库的 UART 发送完成 / 错误回调采用"弱函数 + 重写"机制：
 *   HAL_UART_TxCpltCallback / HAL_UART_ErrorCallback 是 __weak 函数，
 *   用户需在 C 文件中提供同名实现覆盖。
 *
 * 数据流（VOFA 发送）：
 *   Vofa.Send6Floats() → HAL_UART_Transmit_DMA() → DMA 搬运
 *     → DMA 完成中断 → HAL_UART_TxCpltCallback() [本文件重写]
 *       → Vofa_TxComplete() 清除忙标志 → 准备发送下一帧
 *
 * 错误恢复（拔插串口线 / 上位机开关串口）：
 *   Frame Error / Overrun Error → HAL_UART_ErrorCallback()
 *     → 清除错误标志
 *     → 强制复位 VOFA 忙状态（防止 DMA 死锁）
 *     → 终止当前发送（HAL_UART_AbortTransmit）
 *
 * @note 此回调在中断上下文中执行，必须简短高效。
 *       不应在此函数中调用 HAL_Delay 或进行复杂计算。
 *       **必须** 使用 extern "C" 包裹！HAL 回调是 __weak C 函数。
 */

#include "main.h"
#include "usart.h"
#include "Vofa.hpp"

/**
 * @brief UART 发送完成回调
 *
 * DMA 发送完成后由 HAL 触发。
 * 仅处理 USART6（VOFA+ 数据发送）。
 *
 * @param huart UART 句柄
 * @note 此函数在 DMA2_Stream6 中断上下文中执行
 */
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        // 清除 VOFA 忙标志，准备发送下一帧
        Vofa_TxComplete();
    }
}

/**
 * @brief UART 错误回调
 *
 * 拔插串口线 / 上位机开关串口会产生 Frame Error / Overrun Error。
 * 如果不处理，huart6 会永久死锁，VOFA 永远无法发送新帧。
 *
 * 处理策略：
 *   1. 清除所有错误标志
 *   2. 强制复位 VOFA 忙状态（防止 DMA 死锁）
 *   3. 终止当前发送（HAL_UART_AbortTransmit）
 *
 * @param huart UART 句柄
 * @note 此函数在 USART6 中断上下文中执行
 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        // 清除 Overrun Error 标志
        __HAL_UART_CLEAR_OREFLAG(huart);
        // 清除 Frame Error 标志
        __HAL_UART_CLEAR_FEFLAG(huart);
        // 清除 Noise Error 标志
        __HAL_UART_CLEAR_NEFLAG(huart);
        // 清除 Parity Error 标志
        __HAL_UART_CLEAR_PEFLAG(huart);

        // 强制复位 VOFA 忙状态，防止 DMA 死锁
        Vofa_TxComplete();

        // 终止当前 DMA 发送，恢复 UART 到可用状态
        HAL_UART_AbortTransmit(huart);
    }
}
