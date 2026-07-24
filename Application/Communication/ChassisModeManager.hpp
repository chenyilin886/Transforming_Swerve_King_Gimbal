/**
 * @file ChassisModeManager.hpp
 * @brief 底盘模式管理器（状态机）
 *
 * 设计原因：
 *   RoboMaster比赛中，底盘需要根据遥控器开关状态切换多种模式：
 *   - 手动模式：wheel控制底盘旋转
 *   - 跟随模式：底盘自动跟随云台朝向
 *   - 小陀螺模式：底盘主动旋转，云台独立瞄准
 *   - 急停模式：紧急停止（最高优先级）
 *
 *   传统做法：在BoardComm中直接判断开关状态，逻辑分散。
 *   本设计：引入状态机统一管理，符合System Prompt的架构理念：
 *     Application → StateMachine → BoardComm → CAN
 *
 * 架构优势：
 *   1. 职责清晰：状态机负责模式判断，BoardComm负责数据打包
 *   2. 易于扩展：新增模式只需修改状态机
 *   3. 易于调试：状态切换计数、滤波拒绝次数可观察
 *   4. 可靠性高：状态滤波防止开关抖动，离线检测自动急停
 *
 * 状态转换优先级（从高到低）：
 *   1. 遥控器离线 → 强制急停
 *   2. S1==DOWN && S2==DOWN → 急停
 *   3. S2==MIDDLE → 跟随模式
 *   4. S2==UP → 小陀螺模式（预留）
 *   5. S2==DOWN → 手动模式
 *
 * 状态滤波设计：
 *   - 目的：防止机械开关抖动导致状态频繁切换
 *   - 方法：连续N次（10次=10ms）检测到相同状态才切换
 *   - 参数：STATE_FILTER_COUNT = 10（可调整）
 *
 * 数据流：
 *   DR16遥控器
 *     ↓ S1/S2开关状态
 *   ChassisModeManager::Update()
 *     ├─ 遥控器离线检测
 *     ├─ 状态判断（优先级）
 *     ├─ 状态滤波（连续10次）
 *     └─ 更新当前状态
 *   BoardComm::Update()
 *     ├─ 读取 ChassisModeManager::GetChassisMode()
 *     └─ 打包 chassis_mode 位域
 *         ↓ CAN2 发送（0x205/0x206）
 *   底盘板接收
 *     ↓ BoardComm_Data.chassis_mode
 *   RemoteControl::MapToChassis()
 *     └─ 根据模式执行不同控制策略
 *
 * 使用方式：
 *   1. 在 main.cpp 的 GimbalUpdate 任务中周期调用：
 *      ChassisModeManager::Instance().Update();
 *
 *   2. 在 BoardComm::Update() 中获取模式：
 *      chassis_mode = ChassisModeManager::Instance().GetChassisMode();
 *
 *   3. 在 Watch 中观察调试变量：
 *      ChassisModeDebug.current_state
 *      ChassisModeDebug.state_change_count
 *      ChassisModeDebug.filter_reject_count
 *
 * @note 继承参考工程的状态机设计思想
 *       符合System Prompt的分层架构原则
 *       状态机不负责PID、CAN、控制算法，只负责模式判断
 */

#pragma once

#include "main.h"
#include <cstdint>
#include "BoardComm.hpp"      // ChassisMode_t 定义
#include "../BSP/Remote/DR16.hpp"  // DR16::Switch 枚举

namespace BoardComm
{

// ========================================================================
// 底盘控制模式枚举
// ========================================================================

/**
 * @brief 底盘控制模式枚举
 *
 * 设计原因：
 *   RoboMaster比赛中底盘需要多种模式：
 *   - 手动：遥控器wheel直接控制旋转
 *   - 跟随：底盘自动旋转，对齐云台朝向
 *   - 小陀螺：底盘主动旋转，云台独立瞄准
 *   - 急停：裁判系统/遥控器触发，立即停止
 *
 * 优先级（从高到低）：
 *   EMERGENCY_STOP > CHASSIS_FOLLOW > GYROSCOPE > MANUAL
 *
 * @note 枚举值与ChassisMode_t位域不直接对应
 *       需通过GetChassisMode()转换
 */
enum class ChassisMode
{
    MANUAL = 0,          ///< 手动模式（wheel控制旋转）
    CHASSIS_FOLLOW = 1,  ///< 跟随模式（跟随云台朝向）
    GYROSCOPE = 2,       ///< 小陀螺模式（主动旋转）
    EMERGENCY_STOP = 3   ///< 急停（最高优先级）
};

// ========================================================================
// 急停触发源枚举
// ========================================================================

/**
 * @brief 急停触发源枚举（调试用）
 *
 * 用途：
 *   - 在Watch中观察急停是由哪个源触发的
 *   - 帮助调试故障原因
 */
enum class EmergencyStopTrigger
{
    NONE = 0,           ///< 无急停
    REMOTE_SWITCH = 1,  ///< S1+S2开关触发
    REMOTE_OFFLINE = 2, ///< 遥控器离线触发
    CAN_ERROR = 3,      ///< CAN通信错误（预留）
    REFEREE_SYSTEM = 4  ///< 裁判系统急停（预留）
};

// ========================================================================
// 底盘模式管理器类
// ========================================================================

/**
 * @brief 底盘模式管理器（状态机）
 *
 * 职责：
 *   1. 根据遥控器开关状态决定底盘模式
 *   2. 处理模式切换逻辑（优先级/互斥/滤波）
 *   3. 输出chassis_mode位域（供BoardComm使用）
 *
 * 设计原则：
 *   - 继承参考工程的状态机架构
 *   - 单例模式（全局唯一实例）
 *   - Watch可观察（状态/切换计数/滤波拒绝次数）
 *
 * 使用方式：
 *   1. 在主任务中周期调用Update()
 *   2. 在BoardComm中调用GetChassisMode()获取模式位域
 */
class ChassisModeManager
{
public:
    /**
     * @brief 构造函数
     */
    ChassisModeManager() = default;

    /**
     * @brief 更新状态机（主任务调用）
     *
     * 功能：
     *   1. 检查遥控器是否在线（最高优先级）
     *   2. 读取DR16开关状态（S1/S2）
     *   3. 根据优先级判断当前模式
     *   4. 状态滤波（防止开关抖动）
     *   5. 更新状态变量
     *   6. 同步到全局变量（Watch观察）
     *
     * 调用时机：
     *   建议在GimbalUpdate任务中周期调用（1kHz）
     *
     * @note 状态滤波参数：STATE_FILTER_COUNT = 10（10ms）
     */
    void Update();

    /**
     * @brief 获取底盘模式位域
     * @return ChassisMode_t 结构体（可直接CAN发送）
     *
     * 功能：
     *   将内部ChassisMode枚举转换为ChassisMode_t位域
     *
     * 映射关系：
     *   EMERGENCY_STOP → stop=1
     *   CHASSIS_FOLLOW → Follow_mode=1
     *   GYROSCOPE → Rotating_mode=1
     *   MANUAL → 所有位域=0
     */
    ChassisMode_t GetChassisMode() const;

    /**
     * @brief 获取当前状态（Watch观察）
     * @return 当前状态枚举值
     */
    ChassisMode GetCurrentState() const { return current_state_; }

    /**
     * @brief 获取急停触发源（Watch观察）
     * @return 急停触发源枚举值
     */
    EmergencyStopTrigger GetEmergencyStopTrigger() const { return emergency_stop_trigger_; }

    /**
     * @brief 手动触发急停（外部调用）
     * @param trigger 急停触发源
     *
     * 用途：
     *   外部模块（如裁判系统）可主动触发急停
     *
     * @note 调用后current_state_立即变为EMERGENCY_STOP
     */
    void SetEmergencyStop(EmergencyStopTrigger trigger);

    /**
     * @brief 获取单例实例
     * @return ChassisModeManager 实例引用
     */
    static ChassisModeManager& Instance()
    {
        static ChassisModeManager instance;
        return instance;
    }

private:
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
     *   1. S1==DOWN && S2==DOWN → EMERGENCY_STOP
     *   2. S2==MIDDLE → CHASSIS_FOLLOW
     *   3. S2==UP → GYROSCOPE
     *   4. 其他 → MANUAL
     */
    ChassisMode calculateRawState(
        BSP::Remote::DR16::Switch s1,
        BSP::Remote::DR16::Switch s2);

    // ========== 状态变量 ==========
    ChassisMode current_state_ = ChassisMode::MANUAL;    ///< 当前状态（稳定）
    ChassisMode stable_state_ = ChassisMode::MANUAL;     ///< 稳定状态（滤波后）
    ChassisMode pending_state_ = ChassisMode::MANUAL;    ///< 待确认状态（候选）

    // ========== 滤波变量 ==========
    uint32_t stable_count_ = 0;                          ///< 稳定计数（连续相同次数）

    // ========== 统计变量（Watch观察） ==========
    uint32_t state_change_count_ = 0;                    ///< 状态切换次数
    uint32_t filter_reject_count_ = 0;                   ///< 滤波拒绝次数
    uint32_t last_update_time_ = 0;                      ///< 最后更新时间戳

    // ========== 错误状态 ==========
    EmergencyStopTrigger emergency_stop_trigger_ = EmergencyStopTrigger::NONE; ///< 急停触发源

    // ========== 滤波参数 ==========
    static constexpr uint32_t STATE_FILTER_COUNT = 10;   ///< 滤波窗口（次，10ms）
};

} // namespace BoardComm