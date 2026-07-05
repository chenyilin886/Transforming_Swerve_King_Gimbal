#ifndef STATE_WATCH_HPP
#define STATE_WATCH_HPP

/**
 * @file state_watch.hpp
 * @brief 设备在线状态监视器
 *
 * 设计原因：
 *   云台三台电机(Yaw/Pitch/Fold)通过CAN总线反馈数据，若电机掉线而软件无感知，
 *   将导致PID对错误数据运算甚至失控。因此需要一个通用的时间戳超时检测机制。
 *
 * 原理：
 *   每次收到设备数据时记录 HAL_GetTick() 时间戳(UpdateLastTime)；
 *   周期性检查时获取当前时间(UpdateTime)并计算差值(CheckStatus)，
 *   差值超过阈值则判定为 OFFLINE。
 *
 * 继承说明：
 *   移植自参考工程 H_SG_Gimbal，移除 buzzer 依赖(DAY01 阶段无需蜂鸣器)，
 *   修复原工程中 TimeThreshold_ 成员变量被当作临时变量误用的设计缺陷，
 *   改用局部变量 diff_ms 计算时间差，语义更清晰。
 */

#include "main.h"

namespace BSP::WATCH_STATE
{
    /**
     * @brief 设备在线状态枚举
     * @note  OFFLINE=1, ONLINE=0 是刻意设计：便于以 0 表示"正常"的默认期望状态
     */
    enum class Status
    {
        ONLINE  = 0,  // 设备在线
        OFFLINE = 1   // 设备离线
    };

    /**
     * @brief 设备在线状态监视器
     *
     * 通过定时检查设备数据更新时间来判断设备是否在线。
     * 当设备超过设定的时间阈值未更新数据时，判定为离线状态。
     *
     * 使用流程：
     *   1. 收到设备数据 → UpdateLastTime()
     *   2. 周期检查     → UpdateTime() + CheckStatus()
     *   3. 读取状态     → GetStatus()
     */
    class StateWatch
    {
    public:
        ~StateWatch() = default;

        /**
         * @brief 默认构造函数，超时阈值 100ms
         */
        StateWatch()
            : timeout_ms_(100), last_update_time_(0), status_(Status::OFFLINE) {}

        /**
         * @brief 带超时阈值的构造函数
         * @param TimeThreshold 超时时间阈值(毫秒)，建议电机设为 100ms
         */
        StateWatch(uint32_t TimeThreshold)
            : timeout_ms_(TimeThreshold), last_update_time_(0), status_(Status::OFFLINE) {}

        /**
         * @brief 更新当前时间戳
         * @note  在周期性检查设备状态前调用，获取当前系统时间
         */
        void UpdateTime();

        /**
         * @brief 更新上次数据更新时间戳
         * @note  在收到设备数据(CAN反馈)时调用，记录数据到达时刻
         */
        void UpdateLastTime();

        /**
         * @brief 检查设备是否超时，更新在线/离线状态
         * @note  内部处理 32 位计数器回绕(约49天溢出)情况
         */
        void CheckStatus();

        /**
         * @brief 获取设备当前状态
         * @return ONLINE / OFFLINE
         */
        Status GetStatus() const { return status_; }

        /**
         * @brief 获取超时阈值
         * @return 超时时间(毫秒)
         */
        uint32_t GetTimeout() const { return timeout_ms_; }

    private:
        uint32_t timeout_ms_;           // 超时阈值(毫秒)，超过此时间未收到数据则判定离线
        uint32_t last_update_time_;     // 上次收到数据时的系统时间戳(毫秒)
        Status   status_;               // 当前设备状态
    };
}

#endif // STATE_WATCH_HPP
