/**
 * @file HI12H3_IMU.cpp
 * @brief HI12H3 IMU 驱动 - 接收与解析实现
 *
 * 设计原因：
 *   实现 HI12H3 IMU 的数据接收与解析。使用 DMA + 空闲中断方式接收，
 *   提高接收效率，避免 CPU 频繁处理 UART 中断。
 *
 * 数据解析原理：
 *   HI12H3 0x91 数据包固定 82 字节，各字段内存连续存放：
 *     帧头(0x5A 0xA5) + data_len + crc + 系统遥测 + 加速度 + 角速度
 *     + 磁强度 + 欧拉角 + 四元数
 *   由于发送格式固定且内存连续，可利用 memcpy 按 packed 结构体顺序快速拷贝，
 *   无需逐字段手动计算偏移。
 *
 * 帧同步：
 *   每帧首字节应为 0x5A，次字节 0xA5。若帧头不在起始位置(偶发丢字节导致错位)，
 *   调用 SlidingWindowRecovery() 在 buffer 内滑动查找正确帧头并对齐，
 *   然后重启 DMA 接收，避免后续帧持续错位。
 *
 * 数据流：
 *   Init() → HAL_UARTEx_ReceiveToIdle_DMA 启动 DMA 接收
 *     → 收到完整帧触发空闲中断 → HAL_UARTEx_RxEventCallback (UartCallback.cpp)
 *       → imu.Parse(huart, Size)
 *         ├─ 校验 huart == &IMU_UART 且 Size == HI12_MAX_LEN
 *         ├─ ParseData() 解析数据
 *         └─ StateWatch.UpdateLastTime() 更新时间戳
 *     → 重新启动 DMA 接收(等待下一帧)
 *
 * 离线检测：
 *   IsOffline() → StateWatch.UpdateTime() + CheckStatus()
 *     → 超过 50ms 未收到数据判离线
 *     → 离线时 ClearORE() 清除溢出错误，防止 UART 死锁
 *
 * 调试观察点：
 *   - imu.euler.Euler_yaw / Euler_pitch / Euler_roll   欧拉角(deg)
 *   - imu.gyr.Gyr_x/y/z                                角速度(deg/s)
 *   - imu.acc.Acc_x/y/z                                加速度(g)
 *   - imu.quat.Quat_w/x/y/z                            四元数
 *   - imu.addYaw.add_angle                             Yaw 累计角度(deg)
 *   - imu.is_online_                                   在线状态(0/1)
 *
 * @note 继承自参考工程 H_SG_Gimbal 的 HI12H3_IMU，适配当前工程的
 *       StateWatch 离线检测与命名风格(对齐 DR16)
 */

#include "HI12H3_IMU.hpp"
#include <cstring>

namespace BSP::IMU
{

/**
 * @brief IMU 初始化
 *
 * 启动 USART1 DMA 空闲中断接收。
 * HAL_UARTEx_ReceiveToIdle_DMA 在收到完整数据包(空闲帧)后自动触发
 * HAL_UARTEx_RxEventCallback，无需手动判断数据包边界。
 *
 * 关键步骤：
 *   1. 清除所有 UART 错误标志（ORE/FE/NE 等），防止上次调试残留错误导致 DMA 无法启动
 *   2. 启动 DMA 接收
 *
 * @note 在 GimbalInit() 中调用
 */
void HI12::Init()
{
    // 清除 UART 错误标志（防止上次调试残留 ORE 导致无法接收）
    __HAL_UART_CLEAR_OREFLAG(&IMU_UART);
    __HAL_UART_CLEAR_FEFLAG(&IMU_UART);
    __HAL_UART_CLEAR_NEFLAG(&IMU_UART);

    // 清空缓冲区
    std::memset(buffer, 0, sizeof(buffer));

    // 启动 DMA 接收
    HAL_UARTEx_ReceiveToIdle_DMA(&IMU_UART, buffer, sizeof(buffer));
}

/**
 * @brief IMU 数据解析
 *
 * 解析流程：
 *   1. 帧头校验(0x5A 0xA5)，失败则滑动窗口恢复并返回
 *   2. 用 memcpy 按 packed 结构体顺序拷贝各数据域
 *   3. 累计 Yaw 角度(跨 ±180° 连续)
 *   4. 重启 DMA 接收(等待下一帧)
 *
 * @return true  解析成功
 * @return false 帧头错误(已触发滑动窗口恢复)
 */
bool HI12::ParseData()
{
    // 帧头校验：首字节 0x5A，次字节 0xA5
    if (buffer[0] != 0x5A || buffer[1] != 0xA5)
    {
        SlidingWindowRecovery();
        return false;
    }

    uint8_t *pData = buffer;  // 数据指针，从缓冲区起始地址开始

    // 逐字段 memcpy 拷贝（避免 generic lambda，兼容 Keil ARMCLANG V6）
    std::memcpy(&frame,             pData, sizeof(frame));             pData += sizeof(frame);
    std::memcpy(&system_telemetry, pData, sizeof(system_telemetry)); pData += sizeof(system_telemetry);
    std::memcpy(&acc,              pData, sizeof(acc));              pData += sizeof(acc);
    std::memcpy(&gyr,              pData, sizeof(gyr));              pData += sizeof(gyr);
    std::memcpy(&mag,              pData, sizeof(mag));              pData += sizeof(mag);
    std::memcpy(&euler,            pData, sizeof(euler));            pData += sizeof(euler);
    std::memcpy(&quat,             pData, sizeof(quat));             pData += sizeof(quat);

    // 累计 Yaw 角度(跨 ±180° 连续，避免欧拉角跳变)
    AddCaclu(addYaw, euler.Euler_yaw);

    // 重启 DMA 接收，等待下一帧
    HAL_UARTEx_ReceiveToIdle_DMA(&IMU_UART, buffer, sizeof(buffer));

    return true;
}

/**
 * @brief 外部调用的解析入口(在中断回调中调用)
 *
 * @param huart 串口句柄
 * @param Size   本次接收到的数据字节数
 *
 * 数据流：
 *   1. 判断 huart == &IMU_UART
 *   2. 如果 Size == 82（完整帧），调用 ParseData() 解析数据
 *   3. 无论数据是否有效，都重启 DMA 接收（防止 DMA 停止）
 *   4. StateWatch.UpdateLastTime() 更新时间戳(防止误判离线)
 *
 * 关键修复：
 *   - 原代码条件过于严格（Size != 82 不处理），导致 DMA 永久停止接收
 *   - 现改为：只要收到数据就更新时间戳并重启 DMA，即使数据无效也继续接收
 *   - 这样可以避免"有时能收到，有时收不到"的问题
 *
 * @note 在 HAL_UARTEx_RxEventCallback (UartCallback.cpp) 中调用
 */
void HI12::Parse(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &IMU_UART)
    {
        // 更新时间戳（只要收到数据就更新，防止误判离线）
        state_watch_.UpdateLastTime();

        // 只有完整帧才解析（82 字节）
        if (Size == sizeof(buffer))
        {
            ParseData();  // 内部会重启 DMA
        }
        else
        {
            // 长度不正确，直接重启 DMA（丢弃本帧，继续接收下一帧）
            HAL_UARTEx_ReceiveToIdle_DMA(&IMU_UART, buffer, sizeof(buffer));
        }
    }
}

/**
 * @brief 清除 UART ORE(Overrun Error) 错误标志并重启 DMA 接收
 *
 * 当 UART 接收溢出(新数据覆盖未读取的旧数据)时产生 ORE 错误。
 * 必须清除错误标志，否则 UART 会一直处于错误状态，无法继续接收。
 *
 * @param huart 串口句柄
 * @param pData 接收缓冲区
 * @param Size  数据大小
 * @note  通常在离线检测中调用，防止离线后 UART 死锁
 */
void HI12::ClearORE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size);
    }
}

/**
 * @brief 检测 IMU 是否离线
 *
 * @return true  离线(超过 50ms 未收到数据)
 * @return false 在线
 *
 * 检测流程：
 *   1. StateWatch.UpdateTime() + CheckStatus() 计算时间差并更新状态
 *   2. 读取状态
 *   3. 若离线，清除所有 UART 错误 + Abort DMA + 强制重启
 *
 * 增强恢复机制：
 *   - 清除所有常见错误标志（ORE/FE/NE）
 *   - 先 Abort DMA（避免 HAL_BUSY 导致重启失败）
 *   - 强制重启 DMA 接收
 *   - 确保在任何情况下都能恢复接收
 *
 * @note 在 GimbalUpdate() 中周期调用(1kHz)
 */
bool HI12::IsOffline()
{
    state_watch_.UpdateTime();
    state_watch_.CheckStatus();

    bool is_offline = (state_watch_.GetStatus() == WATCH_STATE::Status::OFFLINE);

    // 更新在线标志(Watch 观察用)
    is_online_ = is_offline ? 0 : 1;

    if (is_offline)
    {
        // 离线时强制恢复 UART + DMA（防止任何错误导致永久无法接收）

        // 1. 清除所有 UART 错误标志
        __HAL_UART_CLEAR_OREFLAG(&IMU_UART);
        __HAL_UART_CLEAR_FEFLAG(&IMU_UART);
        __HAL_UART_CLEAR_NEFLAG(&IMU_UART);

        // 2. Abort DMA 接收（避免 HAL_BUSY 导致重启失败）
        HAL_UART_AbortReceive(&IMU_UART);

        // 3. 强制重启 DMA 接收
        HAL_UARTEx_ReceiveToIdle_DMA(&IMU_UART, buffer, sizeof(buffer));
    }

    return is_offline;
}

/**
 * @brief 帧头不在起始位置时，通过滑动窗口恢复帧同步
 *
 * 在 buffer 内滑动查找 0x5A 0xA5 帧头，找到后将后续数据前移对齐，
 * 然后重启 DMA 接收。避免因偶发丢字节导致后续帧持续错位。
 *
 * @note 窗口大小等于缓冲区长度，找到第一个有效帧头即停止
 */
void HI12::SlidingWindowRecovery()
{
    const int window_size = sizeof(buffer);  // 窗口大小等于缓冲区长度

    for (int i = 0; i < window_size - 1; i++)
    {
        // 逐步滑动窗口，检查当前位置是否为有效帧头
        if (buffer[i] == 0x5A && buffer[i + 1] == 0xA5)
        {
            // 找到有效帧头，将后续数据前移对齐帧头
            std::memcpy(buffer, &buffer[i], sizeof(buffer) - i);
            break;
        }
    }
    // 重新启动 DMA 接收
    HAL_UARTEx_ReceiveToIdle_DMA(&IMU_UART, buffer, sizeof(buffer));
}

/**
 * @brief 累计角度计算(处理 ±180° 跳变)
 *
 * HI12H3 欧拉角 Yaw 范围 [-180°, 180°]，过 ±180° 会跳变。
 * 本函数将跳变转换为连续累计角度，用于 Yaw 连续旋转跟踪。
 *
 * 跳变判定：
 *   - 当前角 - 上次角 < -180°：判定正转(跨过 +180°)，累计 +(360 - 上次 + 当前)
 *   - 当前角 - 上次角 > +180°：判定反转(跨过 -180°)，累计 -(360 - 当前 + 上次)
 *   - 否则：累计 +(当前 - 上次)
 *
 * @param addData 累计角度结构体(含 last_angle 与 add_angle)
 * @param angle   当前角度(deg)
 */
void HI12::AddCaclu(AddData &addData, float angle)
{
    double lastData = addData.last_angle;
    double Data = angle;

    if (Data - lastData < -180)         // 正转(跨过 +180°)
        addData.add_angle += (360 - lastData + Data);
    else if (Data - lastData > 180)     // 反转(跨过 -180°)
        addData.add_angle += -(360 - Data + lastData);
    else                                // 未跨界，直接累加差值
        addData.add_angle += (Data - lastData);

    addData.last_angle = Data;
}

} // namespace BSP::IMU
