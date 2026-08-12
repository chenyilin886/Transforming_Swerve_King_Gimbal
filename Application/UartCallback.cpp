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
 * 数据流（DR16 接收）：
 *   UART3 DMA 接收 → HAL_UARTEx_RxEventCallback() [本文件重写]
 *     → DR16.Parse() → 数据解析 → 更新摇杆/开关/鼠标/键盘状态
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
#include "DR16.hpp"
#include "HI12H3_IMU.hpp"
#include "Communication/VisionComm.hpp"  // 视觉通信（RCIA协议）

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
        // 清除 VOFA 忙标志（VOFA 模式下使用）
        Vofa_TxComplete();
        // 清除视觉通信 TX 忙标志（视觉模式下使用）
        // 两者互斥，同一时刻只有一个在发送，安全
        VisionComm::Manager::Instance().TxComplete();
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
        // 强制复位视觉通信 TX 忙状态
        VisionComm::Manager::Instance().TxComplete();

        // 终止当前 DMA 发送，恢复 UART 到可用状态
        HAL_UART_AbortTransmit(huart);

        // 重新启动 DMA 空闲中断接收（视觉通信需要持续监听）
        VisionComm::Manager::Instance().Init();
    }
}

/**
 * @brief UART 接收事件回调（DMA空闲中断）
 *
 * 当UART DMA接收到完整数据包后触发。
 * 处理两个串口：
 *   - USART3：DR16 遥控器接收（18 字节）
 *   - USART1：HI12H3 IMU 接收（82 字节）
 *
 * 数据流（DR16接收）：
 *   UART3 DMA接收 → 接收完成触发空闲中断
 *     → HAL_UARTEx_RxEventCallback() [本文件重写]
 *       → DR16.Instance().Parse() → 数据解析
 *         → 更新摇杆/开关/鼠标/键盘状态
 *
 * 数据流（IMU接收）：
 *   UART1 DMA接收 → 接收完成触发空闲中断
 *     → HAL_UARTEx_RxEventCallback() [本文件重写]
 *       → BSP::IMU::imu.Parse() → memcpy 解析 82 字节帧
 *         → 更新欧拉角/角速度/加速度/四元数/累计Yaw
 *
 * @param huart UART句柄
 * @param Size 接收到的数据长度（DR16=18字节, IMU=82字节）
 * @note 此函数在中断上下文中执行，必须简短高效
 *       使用HAL_UARTEx_ReceiveToIdle_DMA时，接收到完整数据包后自动触发
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        // 调用DR16驱动解析数据
        BSP::Remote::DR16::Instance().Parse(huart, Size);
    }
    else if (huart->Instance == USART1)
    {
        // 调用 IMU 驱动解析 82 字节数据帧
        BSP::IMU::imu.Parse(huart, Size);
    }
    else if (huart->Instance == USART6)
    {
        // 调用视觉通信驱动解析 RCIA 帧（19字节）
        VisionComm::Manager::Instance().Parse(huart, Size);
    }
}
