/**
 * @file ChassisModeManager.cpp
 * @brief 底盘模式管理器实现
 *
 * 实现内容：
 *   1. ChassisModeManager::Update() - 更新状态机
 *   2. ChassisModeManager::GetChassisMode() - 获取模式位域
 *   3. ChassisModeManager::SetEmergencyStop() - 手动触发急停
 *   4. ChassisModeManager::calculateRawState() - 计算原始状态
 *
 * 数据流：
 *   DR16遥控器
 *     ↓ S1/S2开关状态
 *   Update()
 *     ├─ 遥控器离线检测
 *     ├─ calculateRawState()
 *     ├─ 状态滤波
 *     └─ 同步到ChassisModeDebug全局变量
 *   GetChassisMode()
 *     └─ 转换为ChassisMode_t位域
 *
 * 状态滤波算法：
 *   连续10次（10ms）检测到相同状态才切换：
 *   - pending_state：待确认状态（候选）
 *   - stable_count：稳定计数（连续相同次数）
 *   - stable_state：稳定状态（滤波后）
 *   - current_state：当前状态（最终输出）
 *
 * @note 继承参考工程的状态机设计思想
 *       符合System Prompt的分层架构原则
 */

#include "ChassisModeManager.hpp"
#include "../BSP/Remote/DR16.hpp"
#include "../Application/Variable.hpp"
#include "main.h"

namespace BoardComm
{

// ========================================================================
// 状态机更新
// ========================================================================

/**
 * @brief 更新状态机（主任务调用）
 *
 * 实现步骤：
 *   1. 检查遥控器是否在线（最高优先级）
 *   2. 读取DR16开关状态（S1/S2）
 *   3. 计算原始状态（不考虑滤波）
 *   4. 状态滤波（防止开关抖动）
 *   5. 更新当前状态
 *   6. 同步到全局变量（Watch观察）
 *
 * 状态滤波原理：
 *   - 目的：防止机械开关抖动导致状态频繁切换
 *   - 方法：连续N次（10次=10ms）检测到相同状态才切换
 *   - 变量：
 *     - pending_state：待确认状态（候选）
 *     - stable_count：稳定计数（连续相同次数）
 *     - stable_state：稳定状态（滤波后）
 *
 * @note 状态滤波参数：STATE_FILTER_COUNT = 10（10ms）
 *       急停模式无滤波（立即切换）
 */
void ChassisModeManager::Update()
{
    // ========== 0. 遥控器离线检测（最高优先级） ==========
    if (BSP::Remote::DR16::Instance().IsOffline())
    {
        // 遥控器离线，强制急停
        if (current_state_ != ChassisMode::EMERGENCY_STOP)
        {
            current_state_ = ChassisMode::EMERGENCY_STOP;
            emergency_stop_trigger_ = EmergencyStopTrigger::REMOTE_OFFLINE;
            state_change_count_++;
        }

        // 同步到全局变量（Watch观察）
        ChassisModeDebug.current_state = static_cast<uint8_t>(current_state_);
        ChassisModeDebug.stable_state = static_cast<uint8_t>(stable_state_);
        ChassisModeDebug.pending_state = static_cast<uint8_t>(pending_state_);
        ChassisModeDebug.stable_count = stable_count_;
        ChassisModeDebug.state_change_count = state_change_count_;
        ChassisModeDebug.filter_reject_count = filter_reject_count_;
        ChassisModeDebug.emergency_stop_trigger = static_cast<uint8_t>(emergency_stop_trigger_);
        ChassisModeDebug.remote_online = 0;  // 遥控器离线
        ChassisModeDebug.last_update_time = HAL_GetTick();

        return;  // 提前返回，不处理开关状态
    }

    // ========== 1. 读取遥控器开关状态 ==========
    auto s1 = BSP::Remote::DR16::Instance().GetS1();
    auto s2 = BSP::Remote::DR16::Instance().GetS2();

    // ========== 2. 计算原始状态（无滤波） ==========
    ChassisMode raw_state = calculateRawState(s1, s2);

    // ========== 3. 状态滤波 ==========
    // 急停模式无滤波（立即切换）
    if (raw_state == ChassisMode::EMERGENCY_STOP)
    {
        // 立即切换到急停状态
        if (current_state_ != ChassisMode::EMERGENCY_STOP)
        {
            current_state_ = ChassisMode::EMERGENCY_STOP;
            stable_state_ = ChassisMode::EMERGENCY_STOP;
            pending_state_ = ChassisMode::EMERGENCY_STOP;
            emergency_stop_trigger_ = EmergencyStopTrigger::REMOTE_SWITCH;
            state_change_count_++;
            stable_count_ = 0;
        }
    }
    else
    {
        // 普通模式需要滤波
        if (raw_state == pending_state_)
        {
            // 状态持续相同，增加计数
            stable_count_++;

            // 达到滤波阈值，切换状态
            if (stable_count_ >= STATE_FILTER_COUNT)
            {
                if (pending_state_ != stable_state_)
                {
                    // 状态真正切换
                    stable_state_ = pending_state_;
                    current_state_ = stable_state_;
                    state_change_count_++;
                }
                stable_count_ = STATE_FILTER_COUNT;  // 防止溢出
            }
        }
        else
        {
            // 状态发生变化，重置计数
            pending_state_ = raw_state;
            stable_count_ = 1;
            filter_reject_count_++;  // 记录滤波拒绝次数
        }
    }

    // ========== 4. 同步到全局变量（Watch观察） ==========
    ChassisModeDebug.current_state = static_cast<uint8_t>(current_state_);
    ChassisModeDebug.stable_state = static_cast<uint8_t>(stable_state_);
    ChassisModeDebug.pending_state = static_cast<uint8_t>(pending_state_);
    ChassisModeDebug.stable_count = stable_count_;
    ChassisModeDebug.state_change_count = state_change_count_;
    ChassisModeDebug.filter_reject_count = filter_reject_count_;
    ChassisModeDebug.emergency_stop_trigger = static_cast<uint8_t>(emergency_stop_trigger_);
    ChassisModeDebug.remote_online = 1;  // 遥控器在线
    ChassisModeDebug.last_update_time = HAL_GetTick();
}

// ========================================================================
// 获取模式位域
// ========================================================================

/**
 * @brief 获取底盘模式位域
 *
 * @return ChassisMode_t 结构体（可直接CAN发送）
 *
 * 功能：
 *   将内部ChassisMode枚举转换为ChassisMode_t位域
 *
 * 映射关系：
 *   EMERGENCY_STOP → stop=1（其他位域=0）
 *   CHASSIS_FOLLOW → Follow_mode=1
 *   GYROSCOPE → Rotating_mode=1
 *   MANUAL → Universal_mode=1（底盘直接受摇杆控制）
 *
 * @note 使用位域节省CAN传输带宽（1字节）
 */
ChassisMode_t ChassisModeManager::GetChassisMode() const
{
    ChassisMode_t mode{};

    // 根据当前状态填充位域
    switch (current_state_)
    {
        case ChassisMode::EMERGENCY_STOP:
            mode.stop = 1;
            mode.Follow_mode = 0;
            mode.Rotating_mode = 0;
            break;

        case ChassisMode::CHASSIS_FOLLOW:
            mode.stop = 0;
            mode.Follow_mode = 1;
            mode.Rotating_mode = 0;
            break;

        case ChassisMode::GYROSCOPE:
            mode.stop = 0;
            mode.Follow_mode = 0;
            mode.Rotating_mode = 1;
            break;

        case ChassisMode::GYRO_FIXED_TRANSLATION:
            mode.stop = 0;
            mode.Follow_mode = 0;
            mode.Rotating_mode = 1;
            mode.Universal_mode = 1;
            mode.KeyBoard_mode = 0;
            break;

        case ChassisMode::MANUAL:
        default:
            // MANUAL 模式：设置 Universal_mode=1
            // 底盘进入 UniversalHandler（摇杆直接控制底盘平移+旋转）
            // 避免全零位域导致底盘无法匹配任何模式而卡在前一状态
            mode.stop = 0;
            mode.Follow_mode = 0;
            mode.Rotating_mode = 0;
            mode.Universal_mode = 1;
            mode.KeyBoard_mode = 0;
            break;
    }

    return mode;
}

// ========================================================================
// 手动触发急停
// ========================================================================

/**
 * @brief 手动触发急停（外部调用）
 *
 * @param trigger 急停触发源
 *
 * 用途：
 *   外部模块（如裁判系统）可主动触发急停
 *
 * @note 调用后current_state_立即变为EMERGENCY_STOP
 *       下一次Update()会检测到状态变化并记录
 */
void ChassisModeManager::SetEmergencyStop(EmergencyStopTrigger trigger)
{
    if (current_state_ != ChassisMode::EMERGENCY_STOP)
    {
        current_state_ = ChassisMode::EMERGENCY_STOP;
        emergency_stop_trigger_ = trigger;
        state_change_count_++;
    }
}

// ========================================================================
// 计算原始状态
// ========================================================================

/**
 * @brief 计算原始状态（无滤波）
 *
 * @param s1 左开关状态
 * @param s2 右开关状态
 * @return 原始状态枚举值
 *
 * 功能：
 *   根据开关状态直接计算模式（不考虑滤波）
 *
 * 状态判断逻辑：
 *   1. S1==DOWN && S2==DOWN → EMERGENCY_STOP（急停）
 *   2. S2==MIDDLE → CHASSIS_FOLLOW（跟随）
 *   3. S2==UP → GYROSCOPE（小陀螺）
 *   4. 其他 → MANUAL（手动）
 *
 * @note 此函数不考虑滤波和遥控器离线检测
 *       这些逻辑在Update()中统一处理
 */
ChassisMode ChassisModeManager::calculateRawState(
    BSP::Remote::DR16::Switch s1,
    BSP::Remote::DR16::Switch s2)
{
    // 急停（最高优先级）
    if (s1 == BSP::Remote::DR16::Switch::DOWN &&
        s2 == BSP::Remote::DR16::Switch::DOWN)
    {
        return ChassisMode::EMERGENCY_STOP;
    }

    // S1下：云台收起，底盘只允许普通平移。
    if (s1 == BSP::Remote::DR16::Switch::DOWN)
    {
        return ChassisMode::MANUAL;
    }

    // S1中：手动PID；S2上进入可变速小陀螺，其余为底盘跟随。
    if (s1 == BSP::Remote::DR16::Switch::MIDDLE)
    {
        if (s2 == BSP::Remote::DR16::Switch::UP)
        {
            return ChassisMode::GYROSCOPE;
        }
        return ChassisMode::CHASSIS_FOLLOW;
    }

    // S1上：S2中固定小陀螺且可平移，S2下/上为普通平移。
    if (s1 == BSP::Remote::DR16::Switch::UP)
    {
        if (s2 == BSP::Remote::DR16::Switch::MIDDLE)
        {
            return ChassisMode::GYRO_FIXED_TRANSLATION;
        }
        return ChassisMode::MANUAL;
    }

    // 手动模式（默认）
    return ChassisMode::MANUAL;
}

} // namespace BoardComm