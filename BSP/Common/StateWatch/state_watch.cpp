/**
 * @file state_watch.cpp
 * @brief 设备在线状态监视器实现
 *
 * 继承说明：
 *   移植自参考工程。修复原工程将成员变量 TimeThreshold_ 当作临时变量
 *   存储时间差的设计缺陷——此处改为局部变量 diff_ms，语义清晰。
 *
 * 调试观察点：
 *   在 Keil Watch 中可观察 StateWatch 实例的 status_ 字段(0=在线, 1=离线)。
 */

#include "state_watch.hpp"

namespace BSP::WATCH_STATE
{

/**
 * @brief 更新当前时间戳为系统当前时间
 *
 * 将 last_update_time_ 设为 HAL_GetTick() 当前值。
 * 在收到设备数据(CAN反馈帧)时调用。
 */
void StateWatch::UpdateLastTime()
{
    last_update_time_ = HAL_GetTick();
}

/**
 * @brief 获取当前系统时间，供 CheckStatus 计算差值
 *
 * 在周期性检查设备状态前调用。
 */
void StateWatch::UpdateTime()
{
    // 当前时间通过 CheckStatus 内部直接调用 HAL_GetTick() 获取，
    // 此处保留接口以兼容参考工程的调用约定(MotorBase::isConnected 调用顺序)。
}

/**
 * @brief 检查设备是否超时并更新状态
 *
 * 算法：
 *   diff_ms = current_tick - last_update_time_ (处理32位回绕)
 *   若 diff_ms >= timeout_ms_ → OFFLINE
 *   否则 → ONLINE
 *
 * 回绕处理：
 *   HAL_GetTick() 返回 uint32_t，约49天溢出回绕到0。
 *   当 current < last 时，说明发生回绕，实际差值 = current + (0xFFFFFFFF - last)。
 */
void StateWatch::CheckStatus()
{
    uint32_t current_tick = HAL_GetTick();
    uint32_t diff_ms;

    // 处理 32 位计数器回绕
    if (current_tick < last_update_time_)
    {
        // 时间已溢出回绕，从0重新计数
        diff_ms = current_tick + (0xFFFFFFFF - last_update_time_);
    }
    else
    {
        diff_ms = current_tick - last_update_time_;
    }

    // 超时判定
    if (diff_ms >= timeout_ms_)
    {
        status_ = Status::OFFLINE;
    }
    else
    {
        status_ = Status::ONLINE;
    }
}

} // namespace BSP::WATCH_STATE
