/**
 * @file UartCallback.cpp
 * @brief HAL UART callbacks for VOFA, DR16, and IMU.
 */

#include "main.h"
#include "usart.h"
#include "Vofa.hpp"
#include "DR16.hpp"
#include "HI12H3_IMU.hpp"

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        Vofa_TxComplete();
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        Vofa_TxComplete();
        HAL_UART_AbortTransmit(huart);
    }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        BSP::Remote::DR16::Instance().Parse(huart, Size);
    }
    else if (huart->Instance == USART1)
    {
        BSP::IMU::imu.Parse(huart, Size);
    }
}
