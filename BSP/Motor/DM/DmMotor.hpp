#ifndef DM_MOTOR_HPP
#define DM_MOTOR_HPP

/**
 * @file DmMotor.hpp
 * @brief 达妙(DM)电机驱动 - MIT 力矩控制模式
 *
 * 设计原因：
 *   达妙 DM4310(Yaw/Pitch) 与 DM4340(Fold) 共享相同的通信协议(MIT 模式)，
 *   仅参数范围(P_MAX/V_MAX/T_MAX 等)不同。因此抽取 DMMotorBase 基类实现
 *   通用协议解析与控制，子类只需填入型号特定参数。
 *
 * 继承关系：
 *   MotorBase<N> (国际单位 + 在线检测)
 *     ↑
 *   DMMotorBase<N> (DM 协议: MIT 控制 + 反馈解析 + 使能/失能)
 *     ↑
 *   DM4310 : DMMotorBase<2>  (Yaw + Pitch, N=2)
 *   DM4340 : DMMotorBase<1>  (Fold, N=1)
 *
 * 协议说明(MIT 模式)：
 *   命令帧(8字节): pos(16bit) + vel(12bit) + kp(12bit) + kd(12bit) + torque(12bit)
 *   反馈帧(8字节): id(4bit) + err(4bit) + angle(16bit) + vel(12bit)
 *                 + torque(12bit) + T_Mos(8bit) + T_Rotor(8bit)
 *
 *   物理量通过 N 位无符号数线性映射到 [MIN, MAX] 区间：
 *     raw = (physical - MIN) / (MAX - MIN) * (2^bits - 1)
 *     physical = raw / (2^bits - 1) * (MAX - MIN) + MIN
 *
 * 继承说明：
 *   移植自参考工程 H_SG_Gimbal。修改点：
 *   1. 类名 J4310 → DM4310(更符合达妙命名)
 *   2. 新增 DM4340 子类(参考 DM4310 结构，填入 DM4340 参数)
 *   3. 配置三关节 CAN ID: Yaw=0x01, Pitch=0x02, Fold=0x03
 *   4. uint_to_float 移除无意义的 int16_t 强转(值域恒正)
 */

#include "MotorBase.hpp"
#include <cstring>

namespace BSP::MOTOR::DM
{

extern volatile uint32_t dm_fold_feedback_parse_count;
extern volatile uint32_t dm_fold_feedback_header_fallback_count;
extern volatile uint32_t dm_fold_feedback_last_tick;
extern volatile uint32_t dm_fold_feedback_max_gap_ms;
extern volatile uint32_t dm_fold_control_tx_attempt_count;
extern volatile uint32_t dm_fold_control_tx_success_count;
extern volatile uint32_t dm_fold_control_tx_fail_count;

// 调试计数器：用于诊断"使能时收不到数据"问题
extern volatile uint32_t dm_parse_total_count;          // Parse函数调用总次数
extern volatile uint32_t dm_parse_frame_id_0x00;        // 收到CAN ID=0x00的帧数
extern volatile uint32_t dm_parse_frame_id_0x01;        // 收到CAN ID=0x01的帧数
extern volatile uint32_t dm_parse_frame_id_0x02;        // 收到CAN ID=0x02的帧数
extern volatile uint32_t dm_parse_frame_id_0x03;        // 收到CAN ID=0x03的帧数
extern volatile uint32_t dm_parse_normal_match_count;   // normal_feedback_matches匹配次数
extern volatile uint32_t dm_parse_legacy_match_count;   // legacy_feedback_matches匹配次数
extern volatile uint32_t dm_parse_no_match_count;       // 未匹配次数

/**
 * @brief 将浮点数线性映射为无符号整数
 * @param x     输入物理量
 * @param x_min 物理量下限
 * @param x_max 物理量上限
 * @param bits  编码位数(12 或 16)
 * @return 映射后的无符号整数(0 ~ 2^bits-1)
 * @note  超出范围的值会被截断到 [x_min, x_max]
 */
static inline uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief 将无符号整数线性映射回浮点数
 * @param x     输入无符号整数
 * @param x_min 物理量下限
 * @param x_max 物理量上限
 * @param bits  编码位数
 * @return 映射后的物理量
 */
static inline float uint_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    float pga = (float)(x) / (float)((1 << bits) - 1);
    return pga * span + offset;
}

/**
 * @brief DM 电机反馈帧结构(8字节)
 *
 * @note  使用位域组织字段，但 Parse 中仍手动按字节提取，
 *        不依赖位域内存布局(跨平台安全)。
 *        alignas(uint64_t) 确保 8 字节对齐。
 */
struct alignas(uint64_t) DMMotorFeedback
{
    uint8_t  id : 4;            // 电机 ID(低4位)
    uint8_t  err : 4;           // 错误码(高4位)
    uint16_t angle;             // 角度(16bit 无符号)
    uint16_t velocity : 12;     // 角速度(12bit 无符号)
    uint16_t torque : 12;       // 力矩(12bit 无符号)
    uint8_t  T_Mos;             // MOS管温度(℃)
    uint8_t  T_Rotor;           // 转子温度(℃)
};

/**
 * @brief DM 电机配置参数
 *
 * 每个型号的电机有不同的物理量范围，这些范围决定了
 * float ↔ uint 转换时的映射区间。
 */
struct DMParameters
{
    float P_MIN,  P_MAX;     // 位置范围(rad)
    float V_MIN,  V_MAX;     // 速度范围(rad/s)
    float T_MIN,  T_MAX;     // 力矩范围(N·m)
    float KP_MIN, KP_MAX;    // 位置刚度系数范围
    float KD_MIN, KD_MAX;    // 阻尼系数范围
};

/**
 * @class DMMotorBase
 * @brief 达妙电机基类模板(实现 DM 协议通用逻辑)
 *
 * @tparam N 管理的电机数量
 *
 * 子类需实现：
 *   - 构造函数中设置 init_address / recv_idxs_ / send_idxs_ / params_
 *   - 可选: 重写 Parse 以支持特殊帧格式
 */
template <uint8_t N>
class DMMotorBase : public BSP::MOTOR::MotorBase<N>
{
public:
    /**
     * @brief 构造函数
     * @param can_device  关联的 CAN 设备(用于发送命令帧)
     * @param address     CAN ID 基地址(默认 0)
     */
    DMMotorBase(HAL::CAN::ICanDevice *can_device, uint32_t address = 0)
        : can_device_(can_device), init_address(address)
    {
        // 默认超时 100ms(电机反馈周期 < 100ms)
        for (uint8_t i = 0; i < N; ++i)
        {
            this->state_watch_[i] = BSP::WATCH_STATE::StateWatch(100);
        }
    }

    virtual ~DMMotorBase() = default;

    // ====================================================================
    // 反馈解析
    // ====================================================================

    /**
     * @brief CAN 接收回调 - 解析 DM 电机反馈帧
     * @param frame 接收到的 CAN 帧
     *
     * 数据流：
     *   1. 遍历 recv_idxs_ 匹配帧 ID
     *   2. 匹配成功 → 手动提取 8 字节 → uint_to_float → 填充 unit_data_
     *   3. 调用 updateTimestamp 刷新在线状态
     *
     * @note  此函数注册到 CanDevice::register_rx_callback，在中断中执行
     */
    virtual void Parse(const HAL::CAN::Frame &frame) override
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            if (frame.id == init_address + recv_idxs_[i])
            {
                const uint8_t *pData = frame.data;
                feedback_[i].id  = (pData[0] >> 4) & 0x0F;
                feedback_[i].err = pData[0] & 0x0F;
                feedback_[i].angle = (pData[1] << 8) | pData[2];
                feedback_[i].velocity = (pData[3] << 4) | (pData[4] >> 4);
                feedback_[i].torque = ((pData[4] & 0x0F) << 8) | pData[5];
                feedback_[i].T_Mos   = pData[6];
                feedback_[i].T_Rotor = pData[7];
                this->unit_data_[i].angle       = uint_to_float(feedback_[i].angle,    params_.P_MIN,  params_.P_MAX,  16);
                this->unit_data_[i].velocity    = uint_to_float(feedback_[i].velocity, params_.V_MIN,  params_.V_MAX,  12);
                {
                    float torque_raw = uint_to_float(feedback_[i].torque, params_.T_MIN, params_.T_MAX, 12);
                    this->unit_data_[i].current = torque_is_output_side_ ? torque_raw : torque_raw * gear_ratio_;
                }
                this->unit_data_[i].temperature = (float)feedback_[i].T_Mos;
                this->unit_data_[i].accel        = 0.0f;
                this->updateTimestamp(i + 1);
                break;
            }
        }
    }
    // ====================================================================
    // Motor control commands
    // ====================================================================

    /**
     * @brief MIT 模式力矩控制(5 参数)
     * @param id      1-based 电机索引
     * @param pos     目标位置(rad, 输出端)
     * @param vel     目标速度(rad/s, 输出端)
     * @param kp      位置刚度(0~KP_MAX)
     * @param kd      阻尼系数(0~KD_MAX)
     * @param torque  前馈力矩(N·m, **输出端力矩**)
     *
     * 命令帧 8 字节:
     *   pos(16) + vel(12) + kp(12) + kd(12) + torque(12)
     *
     * @note  外部传入的 torque 视为输出端力矩，函数内部会除以 gear_ratio_
     *        换算为电机端力矩后发送给电机。这样上层代码无需关心减速比。
     *
     * @note  CAN 发送 ID = send_idxs_[id-1](电机 Master ID)
     */
    bool ctrl_Mit(uint8_t id, float pos, float vel, float kp, float kd, float torque)
    {
        if (id == 0 || id > N) return false;

        // 输出端力矩 → 电机端力矩（除以减速比）
        // DM 固件内部已换算 angle/velocity 为输出端，但 torque 仍是电机端
        //   例外: DM4340 固件 GR=40 已把 torque 换算为输出端 → 无需再除
        float torque_motor = torque_is_output_side_ ? torque : torque / gear_ratio_;

        HAL::CAN::Frame frame{};
        frame.id = send_idxs_[id - 1];
        frame.is_extended_id = false;
        frame.is_remote_frame = false;
        frame.dlc = 8;

        // 物理量 → 无符号数
        uint16_t pos_raw  = float_to_uint(pos,          params_.P_MIN,  params_.P_MAX,  16);
        uint16_t vel_raw = float_to_uint(vel,          params_.V_MIN,  params_.V_MAX,  12);
        uint16_t kp_raw  = float_to_uint(kp,           params_.KP_MIN, params_.KP_MAX, 12);
        uint16_t kd_raw  = float_to_uint(kd,           params_.KD_MIN, params_.KD_MAX, 12);
        uint16_t tq_raw  = float_to_uint(torque_motor, params_.T_MIN,  params_.T_MAX,  12);

        // 打包 8 字节
        frame.data[0] = (pos_raw >> 8) & 0xFF;
        frame.data[1] = pos_raw & 0xFF;
        frame.data[2] = (vel_raw >> 4) & 0xFF;
        frame.data[3] = ((vel_raw & 0xF) << 4) | ((kp_raw >> 8) & 0xF);
        frame.data[4] = kp_raw & 0xFF;
        frame.data[5] = (kd_raw >> 4) & 0xFF;
        frame.data[6] = ((kd_raw & 0xF) << 4) | ((tq_raw >> 8) & 0xF);
        frame.data[7] = tq_raw & 0xFF;

        const bool is_fold =
            N == 1U && (send_idxs_[id - 1] & 0x0FU) == 0x03U;
        if (is_fold)
        {
            ++dm_fold_control_tx_attempt_count;
        }

        const bool sent = can_device_->send(frame);
        if (is_fold)
        {
            if (sent)
            {
                ++dm_fold_control_tx_success_count;
            }
            else
            {
                ++dm_fold_control_tx_fail_count;
            }
        }
        return sent;
    }

    /**
     * @brief 位置+速度控制(kp/kd 固定，仅设目标位置和速度)
     * @param id      1-based 电机索引
     * @param pos     目标位置(rad)
     * @param vel     目标速度(rad/s)
     * @param kp      位置刚度(默认 5)
     * @param kd      阻尼系数(默认 0.5)
     * @note  力矩前馈为 0，适合位置伺服场景
     */
    void ctrl_AngleVelocity(uint8_t id, float pos, float vel, float kp = 5.0f, float kd = 0.5f)
    {
        ctrl_Mit(id, pos, vel, kp, kd, 0.0f);
    }

    /**
     * @brief 纯速度控制(kp=0, 仅设速度和阻尼)
     * @param id      1-based 电机索引
     * @param vel     目标速度(rad/s)
     * @param kd      阻尼系数(默认 0.5)
     */
    void ctrl_Velocity(uint8_t id, float vel, float kd = 0.5f)
    {
        ctrl_Mit(id, 0.0f, vel, 0.0f, kd, 0.0f);
    }

    // ====================================================================
    // 电机使能/失能/清错
    // ====================================================================

    /**
     * @brief 使能电机(进入闭环控制)
     * @param id 1-based 电机索引
     *
     * DM 协议使能命令：
     *   data[0..6] = 0xFF, data[7] = 0xFC
     *   CAN ID = send_idxs_[id-1] (电机 Master ID)
     *
     * 电机收到使能命令后回复一帧反馈帧，之后每次收到控制帧都会回复反馈。
     */
    bool On(uint8_t id)
    {
        if (id == 0 || id > N) return false;
        uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
        return sendRaw(send_idxs_[id - 1], data);
    }

    /**
     * @brief 失能电机(退出闭环)
     * @param id 1-based 电机索引
     *
     * DM 协议失能命令：
     *   data[0..6] = 0xFF, data[7] = 0xFD
     */
    bool Off(uint8_t id)
    {
        if (id == 0 || id > N) return false;
        uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
        return sendRaw(send_idxs_[id - 1], data);
    }

    /**
     * @brief 清除电机错误
     * @param id 1-based 电机索引
     *
     * DM 协议清错命令：
     *   data[0..6] = 0xFF, data[7] = 0xFB
     */
    bool ClearErr(uint8_t id)
    {
        if (id == 0 || id > N) return false;
        uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
        return sendRaw(send_idxs_[id - 1], data);
    }

    // ====================================================================
    // 保存参数(预留，DAY01 不使用)
    // ====================================================================

    /**
     * @brief 保存参数到电机 Flash(预留)
     * @param id 1-based 电机索引
     * @note  发送后电机将当前参数写入 Flash，重启后生效
     */
    void setSave(uint8_t id)
    {
        if (id == 0 || id > N) return;
        uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
        sendRaw(send_idxs_[id - 1], data);
    }

protected:
    HAL::CAN::ICanDevice *can_device_;       // 关联的 CAN 设备(发送命令帧)
    uint32_t init_address;                    // CAN ID 基地址(默认 0)
    uint32_t recv_idxs_[N] = {};             // 反馈帧 CAN ID 列表(电机 CAN ID)
    uint32_t send_idxs_[N] = {};             // 命令帧 CAN ID 列表(电机 Master ID)
    DMParameters params_ = {};                // 电机参数范围(子类初始化)
    DMMotorFeedback feedback_[N] = {};       // 原始反馈数据(中间存储)

    /**
     * @brief 减速比(电机端转数 / 输出端转数)
     *
     * 用于 torque 输入/反馈换算：
     * - 外部接口(上层代码)统一使用 **输出端 SI 单位**：
     *     angle / velocity / torque 都以输出端为参考
     * - 内部 CAN 协议：
     *     angle / velocity 已由电机固件换算为输出端
     *     torque 仍是电机端力矩，需在 Motor 层换算
     *
     * 换算关系：
     * - 发送：torque_motor = torque_output / gear_ratio_
     * - 反馈：torque_output = torque_motor × gear_ratio_
     *
     * 子类在构造函数中设置：
     * - DM4310: gear_ratio_ = 10.0f (10:1 减速比)
     * - DM4340: gear_ratio_ = 40.0f (40:1 减速比)
     */
    float gear_ratio_ = 1.0f;

    /**
     * @brief 力矩是否已由固件换算为输出端
     *
     * 用于 torque 输入/反馈换算的条件判断：
     *   false(DM4310 默认): torque 字段是电机端力矩，需在 Motor 层换算
     *                        - 发送: torque_motor = torque / gear_ratio_
     *                        - 反馈: torque_output = torque_motor × gear_ratio_
     *   true (DM4340 设置): 固件内部 GR=40 已把 torque 换算为输出端力矩
     *                        - 发送: 直接使用上层传入的输出端力矩
     *                        - 反馈: 直接使用固件反馈的输出端力矩
     *
     * @note DM4340 实测：TMAX=28 N·m 是输出端峰值（与规格书 27 N·m 接近）
     *       若仍按电机端处理（除以 40），发送 0.7 N·m 远小于重力 → Fold 抬不动
     *
     * 子类设置：
     * - DM4310: 保持默认 false（固件 torque 是电机端）
     * - DM4340: 构造时置 true（固件 GR=40 已换算为输出端）
     */
    bool torque_is_output_side_ = false;

    /**
     * @brief 发送原始 8 字节数据到指定 CAN ID
     * @param can_id  目标 CAN ID
     * @param data    8 字节数据数组
     * @return true=发送成功, false=邮箱满或发送失败
     */
    bool sendRaw(uint32_t can_id, const uint8_t data[8])
    {
        HAL::CAN::Frame frame{};
        frame.id = can_id;
        frame.is_extended_id = false;
        frame.is_remote_frame = false;
        frame.dlc = 8;
        memcpy(frame.data, data, 8);
        return can_device_->send(frame);
    }
};


// ========================================================================
// 具体型号子类
// ========================================================================

/**
 * @class DM4310
 * @brief DM4310 电机驱动(Yaw + Pitch 两台)
 *
 * 参数来源: 达妙 DM-J4310-2EC V1.2 减速电机说明书 + Seeed Studio Wiki
 *   - 默认 PMAX=12.5 rad, VMAX=30 rad/s, TMAX=10 Nm
 *   - 减速比 10:1，电机固件内 GR=10，反馈角度/速度已换算为输出轴值
 *   - P 范围必须与电机固件 PMAX 一致，否则反馈解码和控制编码都会出错
 * CAN 配置: Yaw ID=0x04, Pitch ID=0x02(均接 CAN1)
 */
class DM4310 : public DMMotorBase<2>
{
public:
    DM4310(HAL::CAN::ICanDevice *can_device, uint32_t address = 0)
        : DMMotorBase<2>(can_device, address)
    {
        // DM4310 MIT 模式参数范围(必须与电机固件默认值一致)
        params_ = {
            -12.5f, 12.5f,    // P: ±12.5 rad (输出轴，电机默认 PMAX=12.5)
            -30.0f, 30.0f,    // V: ±30 rad/s (输出轴，电机默认 VMAX=30)
            -10.0f, 10.0f,    // T: ±10 N·m (电机端力矩，电机默认 TMAX=10)
            0.0f, 500.0f,     // KP: 0~500
            0.0f, 5.0f        // KD: 0~5
        };

        // 减速比 10:1（DM-J4310-2EC 标准减速比）
        // 上层接口看到的全是输出端 SI 单位，Motor 层自动处理换算
        gear_ratio_ = 10.0f;

        // 反馈帧 ID(电机 CAN ID): Yaw=0x04, Pitch=0x02
        recv_idxs_[0] = 0x04;  // Yaw
        recv_idxs_[1] = 0x02;  // Pitch

        // 命令帧 ID(电机 Master ID): Yaw=0x04, Pitch=0x02
        send_idxs_[0] = 0x04;  // Yaw
        send_idxs_[1] = 0x02;  // Pitch
    }
};


/**
 * @class DM4340
 * @brief DM4340 电机驱动(Fold 变形关节一台)
 *
 * 参数来源: 达妙 DM-J4340-2EC V1.1 减速电机说明书 + Seeed Studio Wiki
 *   - 默认 PMAX=12.5 rad, VMAX=8 rad/s, TMAX=28 Nm
 *   - 减速比 40:1，电机固件内 GR=40
 *   - **力矩字段已是输出端力矩**（与 DM4310 不同）
 * CAN 配置: Fold ID=0x03(接 CAN1)
 *
 * @note DM4340 扭矩(28N·m)远大于 DM4310(10N·m)，适合 Fold 关节
 *      的大力矩需求。位置范围±12.5rad 允许多圈旋转。
 *
 * @note **力矩换算差异（重要）**:
 *   电机固件 GR=40 已把 P/V/T 三个字段全部换算为输出端 SI 单位：
 *     - TMAX=28 N·m 是输出端峰值（与规格书峰值 27 N·m 接近，留 1 N·m 余量）
 *     - 发送 torque = 输出端力矩，无需再除 GR
 *     - 反馈 torque = 输出端力矩，无需再乘 GR
 *   因此 DM4340 构造时设置 torque_is_output_side_ = true，跳过 Motor 层换算。
 *   若错误地按电机端处理（除以 40），28 N·m 会被缩成 0.7 N·m，Fold 完全抬不动。
 */
class DM4340 : public DMMotorBase<1>
{
public:
    DM4340(HAL::CAN::ICanDevice *can_device, uint32_t address = 0)
        : DMMotorBase<1>(can_device, address)
    {
        // DM4340 MIT 模式参数范围(必须与电机固件默认值一致)
        // 注意: T_MIN/T_MAX 是输出端力矩（固件 GR=40 已换算）
        params_ = {
            -12.5f, 12.5f,    // P: ±12.5 rad (输出轴，电机默认 PMAX=12.5)
            -10.0f, 10.0f,      // V: ±10 rad/s (输出轴，电机默认 VMAX=10)
            -28.0f, 28.0f,    // T: ±28 N·m (输出端力矩，固件 GR=40 已换算)
            0.0f, 500.0f,     // KP: 0~500
            0.0f, 5.0f        // KD: 0~5
        };

        // 减速比 40:1（DM-J4340-2EC 标准减速比）
        // 上层接口看到的全是输出端 SI 单位
        // 注意: DM4340 固件 GR=40 已把 torque 换算为输出端
        //       → torque_is_output_side_ = true，本字段仅用于参考
        gear_ratio_ = 40.0f;

        // 关键：固件已换算 torque 为输出端，跳过 Motor 层的除/乘 GR
        // 不设置此项会导致发送力矩被缩 40 倍（28 N·m → 0.7 N·m）
        torque_is_output_side_ = true;

        // 反馈载荷中的电机 ID: Fold=0x03；外层帧 ID 由 Master ID 决定
        recv_idxs_[0] = 0x03;

        // 命令帧 ID: Fold=0x03
        send_idxs_[0] = 0x03;
    }
};


// ========================================================================
// 全局实例声明(在 GimbalInit.cpp 中定义)
// ========================================================================
//
// 设计说明：
//   使用指针而非引用，避免静态初始化顺序问题。
//   电机构造需要 CAN 设备指针(来自 get_can_bus_instance())，
//   而 CAN 初始化必须在 HAL_Init() + MX_CAN1_Init() 之后进行。
//   因此电机实例在 GimbalInit() 中创建，全局指针初始为 nullptr。
//   使用前需确保 GimbalInit() 已执行(在 main() 中调用)。

extern DM4310* dm4310_yaw_pitch;     // Yaw + Pitch 电机组指针
extern DM4340* dm4340_fold;           // Fold 变形电机指针

} // namespace BSP::MOTOR::DM

#endif // DM_MOTOR_HPP
