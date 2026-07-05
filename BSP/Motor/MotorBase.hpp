#ifndef MOTOR_BASE_HPP
#define MOTOR_BASE_HPP

/**
 * @file MotorBase.hpp
 * @brief 电机基类模板
 *
 * 设计原因：
 *   不同型号电机(DM4310/DM4340/...)共享相同的抽象：
 *     - 反馈数据(角度/速度/电流/温度)统一为 SI 国际单位
 *     - 在线检测(基于 StateWatch)
 *     - 与 CAN 回调绑定的 Parse 接口
 *   使用模板参数 N 支持多电机实例(如 N=2 表示一组两台)，
 *   避免动态分配内存，保证嵌入式实时性。
 *
 * 继承说明：
 *   移植自参考工程 H_SG_Gimbal。**裸机适配修改**：
 *   1. 移除 #include "BSP/Common/StateWatch/buzzer_manager.hpp"
 *   2. isConnected(id_state, id_ring) → isConnected(id_state)
 *      (DAY01 阶段无需蜂鸣器报警，后续阶段如需再加回)
 *
 * 数据流：
 *   CAN 接收中断 → CanDevice::receive → trigger_rx_callbacks
 *     → 子类 Parse(frame) → 解析数据填充 unit_data_[] + 更新 StateWatch
 *
 * 调试观察点：
 *   unit_data_[] 数组在 Keil Watch 中可观察。
 *   通过 Application 层的 watch_* 全局变量桥接(见 GimbalInit.hpp)。
 */

#include "state_watch.hpp"
#include "can_hal.hpp"
#include <cmath>

namespace BSP::MOTOR
{
/// 角度转弧度系数
constexpr float PI = 3.14159265358979323846f;
constexpr float DEG2RAD = PI / 180.0f;
constexpr float RAD2DEG = 180.0f / PI;

/**
 * @brief 单个电机的国际单位反馈数据
 *
 * 所有反馈字段统一为 SI 单位：
 *   angle       → 弧度(rad)，范围由电机型号决定
 *   velocity    → 弧度/秒(rad/s)
 *   current     → 安培(A)，此处 DM 协议实为力矩(N·m)
 *   temperature → 摄氏度(℃)
 *   accel       → 弧度/秒²(rad/s²)，DM 协议目前未提供，预留
 */
struct UnitData
{
    float angle;        // 角度(rad)
    float velocity;     // 角速度(rad/s)
    float current;      // 力矩(N·m)，DM 协议此字段表示力矩而非电流
    float temperature;  // 温度(℃)
    float accel;        // 角加速度(rad/s²)，预留(0填充)
};

/**
 * @class MotorBase
 * @brief 电机基类模板
 *
 * @tparam N 该实例管理的电机数量(如 Yaw+Pitch=2)
 */
template <uint8_t N>
class MotorBase
{
public:
    MotorBase() = default;
    virtual ~MotorBase() = default;

    /**
     * @brief 获取指定电机的角度(rad)
     * @param id 1-based 索引(1..N)
     */
    float getAngleRad(uint8_t id) const
    {
        if (id == 0 || id > N) return 0.0f;
        return unit_data_[id - 1].angle;
    }

    /**
     * @brief 获取指定电机的角度(度)
     * @param id 1-based 索引(1..N)
     */
    float getAngleDeg(uint8_t id) const
    {
        return getAngleRad(id) * RAD2DEG;
    }

    /**
     * @brief 获取指定电机的角速度(rad/s)
     */
    float getVelocityRad(uint8_t id) const
    {
        if (id == 0 || id > N) return 0.0f;
        return unit_data_[id - 1].velocity;
    }

    /**
     * @brief 获取指定电机的角速度(度/s)
     */
    float getVelocityDeg(uint8_t id) const
    {
        return getVelocityRad(id) * RAD2DEG;
    }

    /**
     * @brief 获取指定电机的力矩(N·m)
     * @note DM 协议此字段为力矩而非电流
     */
    float getTorque(uint8_t id) const
    {
        if (id == 0 || id > N) return 0.0f;
        return unit_data_[id - 1].current;
    }

    /**
     * @brief 获取指定电机的温度(℃)
     */
    float getTemperature(uint8_t id) const
    {
        if (id == 0 || id > N) return 0.0f;
        return unit_data_[id - 1].temperature;
    }

    /**
     * @brief 检查指定电机是否在线
     * @param id_state 1-based 索引(1..N)
     * @return true=在线, false=离线或索引越界
     *
     * @note 内部调用 StateWatch::CheckStatus() 刷新在线状态，
     *       超时阈值由子类在构造时通过 StateWatch 构造函数设置。
     */
    bool isConnected(uint8_t id_state)
    {
        if (id_state == 0 || id_state > N) return false;
        state_watch_[id_state - 1].CheckStatus();
        return state_watch_[id_state - 1].GetStatus() == BSP::WATCH_STATE::Status::ONLINE;
    }

    /**
     * @brief CAN 接收回调入口(由子类实现具体协议解析)
     * @param frame 接收到的 CAN 帧
     * @note 此函数需绑定到 CanDevice::register_rx_callback
     */
    virtual void Parse(const HAL::CAN::Frame &frame) = 0;

protected:
    /// 单位数据数组(模板大小 N，栈/静态分配，无动态内存)
    UnitData unit_data_[N] = {};

    /// 在线状态监视器数组(每个电机一个)
    BSP::WATCH_STATE::StateWatch state_watch_[N];

    /**
     * @brief 更新指定电机的"最后收到数据时间戳"
     * @param id 1-based 索引
     * @note 子类 Parse 成功解析数据后调用，触发在线状态刷新
     */
    void updateTimestamp(uint8_t id)
    {
        if (id > 0 && id <= N)
        {
            state_watch_[id - 1].UpdateLastTime();
        }
    }
};

} // namespace BSP::MOTOR

#endif // MOTOR_BASE_HPP
