#ifndef DJI_MOTOR_HPP
#define DJI_MOTOR_HPP

/**
 * @file DjiMotor.hpp
 * @brief DJI 电机驱动（GM3508/GM2006/GM6020）— CAN 协议电流控制模式
 *
 * 设计来源：
 *   参考工程 H_SG_Gimbal 和 H_SG-SG_New 的 BSP/Motor/Dji/DjiMotor.hpp。
 *   本工程为 RM2026 三关节可变形云台，需要 DJI 3508 作为摩擦轮电机。
 *
 * DJI CAN 协议帧格式：
 *   反馈帧（电机 → 主机，8 字节）：
 *     data[0-1] : angle  (int16_t, big-endian, 0-8191 → 0°-360°)
 *     data[2-3] : velocity (int16_t, big-endian, RPM, 电机端)
 *     data[4-5] : current (int16_t, big-endian, -16384 ~ +16384)
 *     data[6]   : temperature (uint8_t, °C)
 *     data[7]   : unused
 *
 *   控制帧（主机 → 电机，8 字节）：
 *     data[0-1] : motor1 current (int16_t, big-endian)
 *     data[2-3] : motor2 current
 *     data[4-5] : motor3 current
 *     data[6-7] : motor4 current
 *
 *   CAN ID 分配：
 *     反馈帧: 0x200 + motor_id (motor_id ∈ {1,2,3,4})
 *     控制帧: 0x200 (统一发送，一帧带 4 台电机)
 *
 * 继承关系：
 *   MotorBase<N> (国际单位 + 在线检测)
 *     ↑
 *   DjiMotorBase<N> (DJI 协议: 电流控制 + 反馈解析)
 *     ↑
 *   GM3508 (3508 电机参数预设)
 *
 * 参数说明（GM3508）：
 *   减速比:         1:1（摩擦轮直连，无减速箱）
 *   力矩常数:       0.3 N·m/A（原厂参数）
 *   反馈电流最大值:  16384（原始值）
 *   实际电流最大值:  20 A
 *   编码器分辨率:    8192 counts/rev
 *
 * @note  本驱动假设电机直连（reduction_ratio=1.0）。
 *        若使用减速箱，请在 GM3508 构造时修改 Parameters。
 */

#include "MotorBase.hpp"
#include "can_hal.hpp"
#include <cstring>

namespace BSP::MOTOR::DJI
{

// ========================================================================
// DJI 电机参数（International System 单位换算系数）
// ========================================================================
/**
 * @struct DjiParameters
 * @brief DJI 电机物理参数 & 派生换算系数
 *
 * 所有字段以 float 存储（与 MotorBase::UnitData 一致）。
 * 构造时自动预计算派生系数，避免运行时重复除法。
 */
struct DjiParameters
{
    float reduction_ratio;       // 减速比（电机端 / 输出端）
    float torque_constant;       // 力矩常数（N·m/A）
    float feedback_current_max;  // 反馈电流最大原始值（如 16384）
    float current_max;           // 实际电流最大值（A）
    float encoder_resolution;    // 编码器分辨率（counts/rev）

    // === 派生换算系数（构造时自动计算）===
    float encoder_to_deg;        // 编码器计数 → 度（电机端）
    float encoder_to_rpm;        // 编码器计数 → RPM（电机端速度换算系数）
    float rpm_to_radps;          // 电机端 RPM → 输出端 rad/s
    float current_to_torque;     // 反馈电流原始值 → 输出端力矩（N·m）
    float feedback_to_current;   // 反馈电流原始值 → 实际电流（A）
    float deg_to_real;           // 电机端角度 → 输出端角度（= 1/reduction_ratio）

    /**
     * @brief 构造参数并预计算所有换算系数
     * @param rr  减速比
     * @param tc  力矩常数（N·m/A）
     * @param fmc 反馈电流最大原始值
     * @param mc  实际电流最大值（A）
     * @param er  编码器分辨率（counts/rev）
     */
    DjiParameters(float rr, float tc, float fmc, float mc, float er)
        : reduction_ratio(rr), torque_constant(tc),
          feedback_current_max(fmc), current_max(mc), encoder_resolution(er)
    {
        // 编码器计数 → 角度（度）: 360° / resolution
        encoder_to_deg = 360.0f / encoder_resolution;

        // 编码器计数 → RPM:
        //   速度原始值已经是 RPM（电机端），此系数用于电机端 RPM → 输出端 RPM
        encoder_to_rpm = 1.0f / reduction_ratio;

        // 电机端 RPM → 输出端 rad/s:
        //   输出端 RPM = 电机端 RPM / reduction_ratio
        //   输出端 rad/s = 输出端 RPM × (2π/60)
        rpm_to_radps = (1.0f / reduction_ratio) * (2.0f * PI / 60.0f);

        // 反馈电流原始值 → 输出端力矩（N·m）:
        //   力矩 = 电流(A) × torque_constant(N·m/A) × reduction_ratio
        //   电流(A) = raw × (current_max / feedback_current_max)
        //   合并: raw × (current_max / feedback_current_max) × torque_constant × reduction_ratio
        current_to_torque = (current_max / feedback_current_max)
                          * torque_constant * reduction_ratio;

        // 反馈电流原始值 → 实际电流（A）
        feedback_to_current = current_max / feedback_current_max;

        // 电机端角度 → 输出端角度
        deg_to_real = 1.0f / reduction_ratio;
    }
};

// ========================================================================
// DJI 电机反馈原始数据（8 字节 CAN 帧解析中间存储）
// ========================================================================
/**
 * @struct DjiFeedback
 * @brief DJI 电机原始反馈数据
 *
 * alignas(uint64_t) 确保 8 字节对齐，支持 memcpy 直接拷贝。
 * 所有多字节字段为 big-endian，Parse 中使用字节序反转。
 */
struct alignas(uint64_t) DjiFeedback
{
    int16_t  angle;         // 角度原始值（0-8191, big-endian）
    int16_t  velocity;      // 速度原始值（RPM, 电机端, big-endian）
    int16_t  current;       // 电流原始值（±16384, big-endian）
    uint8_t  temperature;   // 温度（°C）
    uint8_t  unused;        // 保留字节
};

// ========================================================================
// DjiMotorBase — DJI 电机基类模板
// ========================================================================
/**
 * @class DjiMotorBase
 * @brief DJI 电机基类模板（实现 DJI CAN 协议通用逻辑）
 *
 * @tparam N 管理的电机数量
 *
 * 子类需实现：
 *   - 构造函数中设置 init_address / recv_idxs_ / send_idxs_ / params_
 *
 * 调用方式：
 *   1. CAN 中断 → Parse(frame) → 填充 unit_data_[] + 更新 online
 *   2. 控制线程 → ctrl_Current(id, raw) → 打包到发送帧
 *   3. 控制线程 → sendCAN() → can_device_ 发送 0x200 帧
 *
 * @note  ctrl_Current 仅设置发送缓冲区，不立即发送。
 *        需在 Control() 末尾统一调用 sendCAN() 发送一次。
 *        这样避免多电机重复发送。
 */
template <uint8_t N>
class DjiMotorBase : public BSP::MOTOR::MotorBase<N>
{
public:
    /**
     * @brief 构造函数
     *
     * @param can_device  关联的 CAN 设备（用于发送控制帧）
     * @param base_id     CAN ID 基地址（DJI 电机固定为 0x200）
     * @param recv_ids    反馈帧 ID 偏移数组（即电机 ID 列表: {1,2}）
     * @param send_id     控制帧 CAN ID（固定 0x200）
     * @param params      电机参数（型号预设）
     */
    DjiMotorBase(HAL::CAN::ICanDevice *can_device,
                 uint16_t base_id,
                 const uint8_t (&recv_ids)[N],
                 uint32_t send_id,
                 const DjiParameters &params)
        : can_device_(can_device), init_address(base_id),
          send_id_(send_id), params_(params)
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            recv_idxs_[i] = recv_ids[i];
            // DJI 电机反馈频率 1kHz，超时 100ms 判离线
            this->state_watch_[i] = BSP::WATCH_STATE::StateWatch(100);
        }
        // 初始化发送帧缓冲区
        memset(msd_.data, 0, 8);
    }

    virtual ~DjiMotorBase() = default;

    // ====================================================================
    // 反馈解析
    // ====================================================================

    /**
     * @brief CAN 接收回调 — 解析 DJI 电机反馈帧
     * @param frame 接收到的 CAN 帧
     *
     * 数据流：
     *   1. 遍历 recv_idxs_ 匹配帧 ID（0x200 + motor_id）
     *   2. memcpy 提取 8 字节 → 字节序反转（big-endian → little-endian）
     *   3. Configure(i) → 原始值 → SI 单位 → 填充 unit_data_[]
     *   4. 刷新在线时间戳
     *
     * @note  此函数注册到 CanDevice::register_rx_callback，在中断中执行
     */
    virtual void Parse(const HAL::CAN::Frame &frame) override
    {
        const uint16_t received_id = frame.id;

        for (uint8_t i = 0; i < N; ++i)
        {
            if (received_id == (uint16_t)(init_address + recv_idxs_[i]))
            {
                // 拷贝 8 字节原始数据
                memcpy(&feedback_[i], frame.data, sizeof(DjiFeedback));

                // DJI 协议多字节字段为 big-endian，需转为 little-endian
                feedback_[i].angle    = __builtin_bswap16(feedback_[i].angle);
                feedback_[i].velocity = __builtin_bswap16(feedback_[i].velocity);
                feedback_[i].current  = __builtin_bswap16(feedback_[i].current);

                // 原始值 → SI 单位
                Configure(i);

                // 刷新在线时间戳
                this->updateTimestamp(i + 1);
                break;
            }
        }
    }

    // ====================================================================
    // 电机控制
    // ====================================================================

    /**
     * @brief 设置单台电机的电流控制值（打包到发送帧）
     *
     * @param id     1-based 电机序号（1..N）
     * @param current_raw 电流原始值（int16_t, ±16384 → ±current_max A）
     *
     * 将 current_raw 按 big-endian 写入 msd_.data 的对应位置。
     * 不立即发送，需要后续调用 sendCAN() 统一发送。
     *
     * 发送帧布局（CAN ID=0x200）：
     *   data[0-1]: motor_id=1
     *   data[2-3]: motor_id=2
     *   data[4-5]: motor_id=3
     *   data[6-7]: motor_id=4
     *
     * @note  id 对应的是电机在 DJI 总线中的 motor_id，
     *        不是本实例内部的数组索引。
     */
    void ctrl_Current(uint8_t id, int16_t current_raw)
    {
        if (id == 0 || id > 4) return;
        uint8_t pos = (id - 1) * 2;
        // big-endian: 高字节在前
        msd_.data[pos]     = (uint8_t)((current_raw >> 8) & 0xFF);
        msd_.data[pos + 1] = (uint8_t)(current_raw & 0xFF);
    }

    /**
     * @brief 发送已组装的电流控制帧
     *
     * 将 msd_ 按 CAN ID=send_id_（通常为 0x200）通过 can_device_ 发送。
     *
     * @return true=发送成功, false=发送失败
     *
     * @note  建议在一轮控制循环末尾统一调用一次。
     *        不需要为每台电机单独调用。
     */
    bool sendCAN()
    {
        HAL::CAN::Frame frame;
        frame.id             = send_id_;
        frame.dlc            = 8;
        frame.is_extended_id = false;
        frame.is_remote_frame = false;
        memcpy(frame.data, msd_.data, 8);
        return can_device_->send(frame);
    }

    // ====================================================================
    // 便捷访问方法
    // ====================================================================

    /**
     * @brief 获取电机端 RPM（用于摩擦轮速度环反馈）
     * @param id 1-based 电机序号
     * @return 电机端 RPM，离线返回 0
     *
     * DJI 反馈帧的 velocity 字段就是电机端 RPM，
     * 此方法提供直接的 RPM 读（不走 rad/s 换算）。
     */
    float getVelocityRpm(uint8_t id) const
    {
        if (id == 0 || id > N) return 0.0f;
        return (float)feedback_[id - 1].velocity;
    }

protected:
    /**
     * @brief 将原始反馈数据转换为 SI 国际单位
     * @param i 电机槽位下标（0-based）
     *
     * 填充 unit_data_[] 各字段：
     *   angle       → 输出端角度（rad）
     *   velocity    → 输出端角速度（rad/s）
     *   current     → 输出端力矩（N·m）
     *   temperature → °C
     *   accel       → 0（DJI 协议无加速度字段）
     *
     * 多圈累计角度处理：
     *   编码器值只有 0-8191 对应 0°-360°，超出 360° 会回绕。
     *   通过检测跳变方向（>180° 或 <-180°）累加整圈偏移，
     *   保证 getAngleRad 返回连续角度。
     */
    void Configure(uint8_t i)
    {
        const auto &p = params_;

        // 电机端角度 → 输出端角度（度）
        float angle_deg = (float)feedback_[i].angle * p.encoder_to_deg;

        // --- 多圈累计角度（处理 0°/360° 边界跳变）---
        float last_deg = last_angle_deg_[i];

        if (angle_deg - last_deg < -180.0f)
        {
            // 正转跨越 0° → 360°：角度从 360° 附近跳回 0°
            add_angle_deg_[i] += (360.0f - last_deg + angle_deg) * p.deg_to_real;
        }
        else if (angle_deg - last_deg > 180.0f)
        {
            // 反转跨越 360° → 0°：角度从 0° 附近跳到 360°
            add_angle_deg_[i] += -(360.0f - angle_deg + last_deg) * p.deg_to_real;
        }
        else
        {
            // 正常增减
            add_angle_deg_[i] += (angle_deg - last_deg) * p.deg_to_real;
        }

        last_angle_deg_[i] = angle_deg;

        // --- 填充 SI 单位字段 ---
        // 角度：累计角度（输出端，度）→ rad
        this->unit_data_[i].angle = add_angle_deg_[i] * DEG2RAD;

        // 角速度：电机端 RPM → 输出端 rad/s
        this->unit_data_[i].velocity = (float)feedback_[i].velocity * p.rpm_to_radps;

        // 力矩：反馈电流原始值 → 输出端 N·m
        this->unit_data_[i].current = (float)feedback_[i].current * p.current_to_torque;

        // 温度：直接值（°C）
        this->unit_data_[i].temperature = (float)feedback_[i].temperature;

        // 加速度：DJI 协议不提供
        this->unit_data_[i].accel = 0.0f;
    }

    HAL::CAN::ICanDevice *can_device_;  // CAN 设备（用于发送控制帧）
    const uint16_t init_address;         // CAN ID 基地址（0x200）
    uint8_t recv_idxs_[N];               // 反馈帧 ID 偏移数组（{1, 2}）
    uint32_t send_id_;                   // 控制帧 CAN ID（0x200）
    DjiFeedback feedback_[N];            // 原始反馈数据缓存
    HAL::CAN::Frame msd_;               // 控制帧发送缓冲区（8 字节）
    DjiParameters params_;              // 单位换算参数
    float add_angle_deg_[N] = {};       // 多圈累计角度（输出端，度）
    float last_angle_deg_[N] = {};      // 上次角度（度，用于检测 0/360 跳变）
};

// ========================================================================
// GM3508 — DJI M3508 电机参数预设
// ========================================================================
/**
 * @class GM3508
 * @brief DJI M3508 减速电机参数预设
 *
 * @tparam N 电机数量
 *
 * 默认参数：
 *   reduction_ratio    = 1.0   （摩擦轮直连，无减速箱）
 *   torque_constant    = 0.3   （N·m/A，电机端）
 *   feedback_current_max = 16384（原始值满量程）
 *   current_max        = 20    （A）
 *   encoder_resolution = 8192  （counts/rev，电机端）
 *
 * 使用示例（本工程 2 台摩擦轮，motor_id=1, 2）：
 *   static GM3508<2> motor_3508(&can1, 0x200, {1, 2}, 0x200);
 *
 * @note  若使用减速箱（如原装 19:1 减速箱），
 *        请派生新类或直接使用 DjiMotorBase 并修改 Parameters 第一参数。
 */
template <uint8_t N>
class GM3508 : public DjiMotorBase<N>
{
public:
    /**
     * @brief 构造 GM3508 电机组
     * @param can_device CAN 设备指针
     * @param base_id    CAN ID 基地址（固定 0x200）
     * @param recv_ids   电机 ID 列表（如 {1, 2}）
     * @param send_id    控制帧 CAN ID（固定 0x200）
     */
    GM3508(HAL::CAN::ICanDevice *can_device,
           uint16_t base_id,
           const uint8_t (&recv_ids)[N],
           uint32_t send_id)
        : DjiMotorBase<N>(can_device, base_id, recv_ids, send_id,
                          DjiParameters(1.0f, 0.3f, 16384.0f, 20.0f, 8192.0f))
    {
    }
};

// ========================================================================
// 全局摩擦轮电机指针 extern 声明
// ========================================================================
/**
 * @brief 全局 GM3508 指针
 *
 * 定义于 GimbalInit.cpp，ShootFSM.cpp 可通过 extern 获取。
 * 用于摩擦轮速度环 PID 控制的反馈读取 + 电流发送。
 *
 * 两个电机 ID：
 *   - motor_id=1: 左摩擦轮（或右，取决于机械装配方向）
 *   - motor_id=2: 右摩擦轮
 *
 * 通过 motor_3508->getVelocityRpm(1) 读取转速，
 * 通过 motor_3508->ctrl_Current(1, raw) + sendCAN() 发送电流。
 */
extern GM3508<2> *motor_3508;

} // namespace BSP::MOTOR::DJI

#endif // DJI_MOTOR_HPP
