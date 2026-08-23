#ifndef LK_MOTOR_HPP
#define LK_MOTOR_HPP

/**
 * @file LkMotor.hpp
 * @brief LK-TECH 电机驱动 - CAN 协议力矩控制模式
 *
 * 设计原因：
 *   LK-TECH MG4005/LK4005 电机使用独立 CAN 协议(非 MIT 模式)，
 *   与达妙 DM 系列协议不同，因此单独实现驱动。
 *   协议特点：
 *     1. 需要先发送使能指令(0x88)才会开始上报反馈
 *     2. 需要周期性发送控制指令(如 0xA1 力矩控制)才能维持反馈上报
 *     3. 反馈帧与命令帧使用相同 CAN ID(0x140 + 电机ID)
 *
 * 继承关系：
 *   MotorBase<N> (国际单位 + 在线检测)
 *     ↑
 *   LKMotorBase<N> (LK 协议: 力矩控制 + 反馈解析 + 使能/失能/清错)
 *     ↑
 *   LK4005 : LKMotorBase<1>  (单台 MG4005/LK4005, motor_id=1)
 *
 * 协议说明(CAN 帧格式)：
 *   命令帧(8字节):
 *     0x88: 使能电机
 *     0x81: 失能电机
 *     0x9A: 读取状态1(电压/错误状态)
 *     0x9B: 清除错误
 *     0xA1: 力矩控制, data[4-5] = torque(int16_t, little-endian, ±2048)
 *     0xA4: 位置控制, data[2-3] = speed, data[4-7] = angle
 *
 *   反馈帧(8字节):
 *     data[0]:    cmd(命令字节,标识本帧类型)
 *     data[1]:    temperature(°C)
 *     data[2-3]:  current(int16_t, little-endian, 反馈电流原始值)
 *     data[4-5]:  velocity(int16_t, little-endian, 1 dps/LSB)
 *     data[6-7]:  angle(uint16_t, little-endian, 输出端编码器计数)
 *
 *   状态1响应帧(0x9A/0x9B 响应):
 *     data[0]:    cmd(0x9A 或 0x9B)
 *     data[1]:    temperature(°C)
 *     data[3-4]:  voltage(uint16_t, little-endian, mV)
 *     data[7]:    error_state(错误状态位)
 *
 * CAN ID 分配：
 *   命令帧 ID = 0x140 + motor_id (主机 → 电机)
 *   反馈帧 ID = 0x140 + motor_id (电机 → 主机, 与命令帧同 ID)
 *   广播帧 ID = 0x280 (多电机广播控制, 本工程暂不使用)
 *
 * 参数说明(LK4005/MG4005):
 *   减速比:        10:1
 *   力矩常数:      0.06 N·m/A
 *   反馈电流最大值: 2048 (原始值)
 *   实际电流最大值: 4 A
 *   编码器分辨率:   65536 counts/rev (16位, 输出端)
 *
 * 继承说明：
 *   参考工程 H_SG-SG_New BSP/Motor/Lk/Lk_motor.hpp 适配移植。
 *   修改点：
 *   1. 命名空间 BSP::Motor::LK → BSP::MOTOR::LK(与 DM 电机一致)
 *   2. CAN 发送方式: 全局 get_can_bus_instance() → can_device_ 指针(与 DM 一致)
 *   3. UnitData 适配: 双精度 angle_Deg/angle_Rad/... → 单精度 angle/velocity/...
 *      (gimbal 工程 MotorBase 使用 float, 统一 SI 单位)
 *   4. 移除多圈累计(multi_angle_data_): gimbal 工程由 Joint 层处理多圈
 *   5. 移除 PendingReply 超时机制: 简化为直接解析, 状态1缓存可选
 */

#include "MotorBase.hpp"
#include <cstring>

namespace BSP::MOTOR::LK
{

/**
 * @brief LK 电机参数(型号相关)
 *
 * 每个型号的电机有不同的物理量范围和减速比，
 * 这些参数决定了原始反馈值到 SI 单位的转换系数。
 */
struct LKParameters
{
    float reduction_ratio;          // 减速比(电机端/输出端)
    float torque_constant;          // 力矩常数(N·m/A)
    float feedback_current_max;     // 反馈电流最大原始值(如 2048)
    float current_max;              // 实际电流最大值(A)
    float encoder_resolution;       // 编码器分辨率(counts/rev, 输出端)

    // --- 派生转换系数(构造时自动计算) ---
    float encoder_to_deg;           // 编码器计数 → 度(输出端)
    float dps_to_output_radps;      // speed feedback dps -> output-side rad/s
    float current_to_torque;        // 反馈电流原始值 → 输出端力矩(N·m)
    float feedback_to_current;      // 反馈电流原始值 → 实际电流(A)

    /**
     * @brief 构造函数(自动计算派生系数)
     * @param rr  减速比
     * @param tc  力矩常数(N·m/A)
     * @param fmc 反馈电流最大原始值
     * @param mc  实际电流最大值(A)
     * @param er  编码器分辨率(counts/rev)
     */
    LKParameters(float rr, float tc, float fmc, float mc, float er)
        : reduction_ratio(rr), torque_constant(tc),
          feedback_current_max(fmc), current_max(mc), encoder_resolution(er)
    {
        // 编码器计数 → 输出端角度(度): 360° / resolution
        encoder_to_deg = 360.0f / encoder_resolution;

        // LK feedback speed is 1 dps/LSB. Convert motor-side dps to
        // output-side rad/s through the reducer.
        dps_to_output_radps = (1.0f / reduction_ratio) * DEG2RAD;

        // 反馈电流原始值 → 输出端力矩(N·m)
        //   力矩 = 电流 × 力矩常数 × 减速比
        //   原始值 → 实际电流: raw × (current_max / feedback_current_max)
        //   实际电流 → 电机端力矩: I × torque_constant
        //   电机端力矩 → 输出端力矩: × 减速比
        //   合并: raw × (current_max / feedback_current_max) × torque_constant × reduction_ratio
        current_to_torque = (current_max / feedback_current_max)
                          * torque_constant * reduction_ratio;

        // 反馈电流原始值 → 实际电流(A)
        feedback_to_current = current_max / feedback_current_max;
    }
};

/**
 * @brief LK 电机原始反馈数据(8字节解析结果)
 *
 * @note  这是 CAN 帧解析后的中间存储，尚未转换为 SI 单位。
 *        Configure() 函数负责将其转换为 UnitData(国际单位)。
 */
struct LKFeedback
{
    uint8_t  cmd;           // 命令字节(标识本帧类型: 0xA1=力矩反馈, 0x9A=状态1响应, ...)
    uint8_t  temperature;   // 温度(°C, 直接值)
    int16_t  current;       // 电流原始值(反馈电流, ±2048 对应 ±current_max A)
    int16_t  velocity;      // Speed raw value, 1 dps/LSB.
    uint16_t angle;         // 角度原始值(输出端编码器计数, 0~65535)
};

/**
 * @brief 状态1缓存(0x9A/0x9B 响应)
 *
 * 用于存储电压、错误状态等信息。
 * 需要主动调用 ReadStatus1() 才会刷新(非周期上报)。
 */
struct LKStatus1
{
    uint8_t  temperature = 0;   // 温度(°C)
    uint16_t voltage = 0;       // 电压(mV)
    uint8_t  error_state = 0;   // 错误状态位(0=无错误)
    bool     is_valid = false;  // 是否已收到有效响应
};

/**
 * @class LKMotorBase
 * @brief LK 电机基类模板(实现 LK CAN 协议通用逻辑)
 *
 * @tparam N 管理的电机数量
 *
 * 子类需实现：
 *   - 构造函数中设置 init_address / recv_idxs_ / send_idxs_ / params_
 */
template <uint8_t N>
class LKMotorBase : public BSP::MOTOR::MotorBase<N>
{
public:
    /**
     * @brief 构造函数
     * @param can_device  关联的 CAN 设备(用于发送命令帧)
     * @param base_id     CAN ID 基地址(LK 电机固定为 0x140)
     * @param recv_ids    反馈帧 ID 偏移数组(即电机 ID 列表)
     * @param send_ids    命令帧 ID 偏移数组(通常与 recv_ids 相同)
     * @param params      电机参数
     */
    LKMotorBase(HAL::CAN::ICanDevice *can_device,
                uint16_t base_id,
                const uint8_t (&recv_ids)[N],
                const uint32_t (&send_ids)[N],
                const LKParameters &params)
        : can_device_(can_device), init_address(base_id), params_(params)
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            recv_idxs_[i] = recv_ids[i];
            send_idxs_[i] = send_ids[i];
            // LK 电机反馈周期取决于控制指令发送频率
            // 超时 200ms: 若 200ms 未收到反馈则判离线
            this->state_watch_[i] = BSP::WATCH_STATE::StateWatch(200);
        }
    }

    virtual ~LKMotorBase() = default;

    // ====================================================================
    // 反馈解析
    // ====================================================================

    /**
     * @brief CAN 接收回调 - 解析 LK 电机反馈帧
     * @param frame 接收到的 CAN 帧
     *
     * 数据流：
     *   1. 遍历 recv_idxs_ 匹配帧 ID(0x140 + motor_id)
     *   2. 根据 data[0](cmd) 判断帧类型：
     *      - 0x9A/0x9B → 状态1响应帧 → 更新 status1_ 缓存
     *      - 其他      → 周期反馈帧  → 解析 + Configure → 填充 unit_data_
     *   3. 调用 updateTimestamp 刷新在线状态
     *
     * @note  此函数注册到 CanDevice::register_rx_callback，在中断中执行
     */
    virtual void Parse(const HAL::CAN::Frame &frame) override
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            // 匹配反馈帧 ID = 0x140 + motor_id
            if (frame.id == (uint32_t)(init_address + recv_idxs_[i]))
            {
                const uint8_t *pData = frame.data;
                uint8_t cmd = pData[0];

                if (cmd == 0x9A || cmd == 0x9B)
                {
                    // --- 状态1响应帧(0x9A 查询 / 0x9B 清错 的响应) ---
                    status1_[i].temperature = pData[1];
                    status1_[i].voltage     = (uint16_t)((pData[4] << 8) | pData[3]);
                    status1_[i].error_state = pData[7];
                    status1_[i].is_valid    = true;
                }
                else
                {
                    // --- 周期反馈帧(正常上报数据) ---
                    feedback_[i].cmd         = cmd;
                    feedback_[i].temperature = pData[1];
                    feedback_[i].current     = (int16_t)((pData[3] << 8) | pData[2]);
                    feedback_[i].velocity    = (int16_t)((pData[5] << 8) | pData[4]);
                    feedback_[i].angle       = (uint16_t)((pData[7] << 8) | pData[6]);

                    // 原始值 → SI 单位
                    Configure(i, feedback_[i]);
                }

                // 刷新在线时间戳
                this->updateTimestamp(i + 1);
                break;  // 一帧只匹配一台电机
            }
        }
    }

    // ====================================================================
    // 电机控制命令
    // ====================================================================

    /**
     * @brief 使能电机(开始闭环控制 + 开始上报反馈)
     * @param id 1-based 电机索引
     *
     * LK 协议使能命令：
     *   data[0] = 0x88, 其余为 0
     *   CAN ID = 0x140 + motor_id
     *
     * @note  LK 电机需要发送使能命令后才开始上报反馈帧。
     *        使能后需周期性发送控制指令(如 ctrl_Torque)维持反馈。
     */
    bool On(uint8_t id)
    {
        if (id == 0 || id > N) return false;
        uint8_t data[8] = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        return sendRaw(init_address + send_idxs_[id - 1], data);
    }

    /**
     * @brief 失能电机(退出闭环 + 停止上报反馈)
     * @param id 1-based 电机索引
     *
     * LK 协议失能命令：
     *   data[0] = 0x81, 其余为 0
     */
    bool Off(uint8_t id)
    {
        if (id == 0 || id > N) return false;
        uint8_t data[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        return sendRaw(init_address + send_idxs_[id - 1], data);
    }

    /**
     * @brief 清除电机错误状态
     * @param id 1-based 电机索引
     *
     * LK 协议清错命令：
     *   data[0] = 0x9B, 其余为 0
     *   电机回复 0x9B 响应帧(含错误状态)
     */
    bool ClearErr(uint8_t id)
    {
        if (id == 0 || id > N) return false;
        uint8_t data[8] = {0x9B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        return sendRaw(init_address + send_idxs_[id - 1], data);
    }

    /**
     * @brief 读取电机状态1(电压/错误状态)
     * @param id 1-based 电机索引
     *
     * LK 协议状态查询命令：
     *   data[0] = 0x9A, 其余为 0
     *   电机回复 0x9A 响应帧 → 更新 status1_ 缓存
     *
     * @note  非周期上报，需主动调用。调用后检查 hasValidStatus1() 判断是否收到响应。
     */
    bool ReadStatus1(uint8_t id)
    {
        if (id == 0 || id > N) return false;
        uint8_t data[8] = {0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        return sendRaw(init_address + send_idxs_[id - 1], data);
    }

    /**
     * @brief 力矩控制指令
     * @param id      1-based 电机索引
     * @param torque  力矩原始值(±2048, 对应 ±current_max A)
     *
     * LK 协议力矩控制命令：
     *   data[0] = 0xA1
     *   data[4-5] = torque(int16_t, little-endian)
     *   CAN ID = 0x140 + motor_id
     *
     * @note  torque 原始值范围 ±2048:
     *          +2048 → 最大正向电流(current_max A)→ 最大正向力矩
     *          -2048 → 最大反向电流 → 最大反向力矩
     *        实际力矩 = torque × (current_max / 2048) × torque_constant × reduction_ratio
     *
     * @note  发送力矩控制指令后电机会回复一帧反馈帧(周期反馈)。
     *        需周期性发送此指令(或任意控制指令)才能维持反馈上报。
     */
    bool ctrl_Torque(uint8_t id, int16_t torque)
    {
        if (id == 0 || id > N) return false;

        // 力矩限幅 ±2048
        if (torque > 2048)  torque = 2048;
        if (torque < -2048) torque = -2048;

        uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        data[0] = 0xA1;
        data[4] = (uint8_t)(torque & 0xFF);
        data[5] = (uint8_t)((torque >> 8) & 0xFF);

        return sendRaw(init_address + send_idxs_[id - 1], data);
    }

    // ====================================================================
    // 原始数据访问(Watch 调试用)
    // ====================================================================

    // ====================================================================
    // 多圈累计角度接口(拨盘位置环专用)
    // ====================================================================
    //
    // 设计原因：
    //   LK 协议反馈为单圈角度(0~2π)，做位置环(如拨盘供弹累计角度)时
    //   需要连续累计角度，不能在 0/2π 边界跳变。
    //   参考 H_SG_Gimbal 参考工程 M2006.getAddAngleDeg() 接口设计。
    //
    // 维护方式：
    //   Configure() 中每次解析反馈时：
    //     delta = wrapToPi(curr_angle - last_angle_rad_[i])  // 跨边界处理
    //     multi_turn_angle_rad_[i] += delta
    //     last_angle_rad_[i] = curr_angle
    //
    // 使用场景：
    //   - 拨盘位置环 PID 反馈(DialController)
    //   - 任何需要多圈连续角度的场合

    /**
     * @brief 获取指定电机的多圈累计角度(rad)
     * @param id 1-based 索引(1..N)
     * @return 累计角度(rad)，上电后以当前位置为零位累加
     *
     * @note 与参考工程 M2006.getAddAngleDeg() 接口对齐(单位换算为 rad)。
     *       上电时累计角度初始化为 0(以当前单圈角度为起点)。
     *       若需要重置，调用 resetMultiTurn(id)。
     */
    float getAddAngleRad(uint8_t id) const
    {
        if (id == 0 || id > N) return 0.0f;
        return multi_turn_angle_rad_[id - 1];
    }

    /**
     * @brief 获取指定电机的多圈累计角度(度)
     * @param id 1-based 索引(1..N)
     * @return 累计角度(度)
     *
     * @note 与参考工程 M2006.getAddAngleDeg() 接口完全对齐。
     *       用于 DialController 位置环反馈(单位: 度)。
     */
    float getAddAngleDeg(uint8_t id) const
    {
        return getAddAngleRad(id) * 180.0f / PI;
    }

    /**
     * @brief 重置多圈累计角度(以当前单圈角度为新起点)
     * @param id 1-based 索引(1..N)
     *
     * 使用场景：
     *   - 拨盘校准(将当前位置设为零位)
     *   - 切换控制模式时清零累计
     *   - 卡弹解卡后重置目标参考
     */
    void resetMultiTurn(uint8_t id)
    {
        if (id == 0 || id > N) return;
        multi_turn_angle_rad_[id - 1] = 0.0f;
        last_angle_rad_[id - 1] = this->unit_data_[id - 1].angle;
        multi_turn_inited_[id - 1] = 1;
    }

    /**
     * @brief 获取原始反馈数据(未转 SI 单位)
     * @param id 1-based 电机索引
     * @return 原始反馈结构体引用
     */
    const LKFeedback& getFeedback(uint8_t id) const
    {
        return feedback_[id - 1];
    }

    /**
     * @brief 获取状态1缓存(电压/错误状态)
     * @param id 1-based 电机索引
     * @return 状态1结构体引用
     */
    const LKStatus1& getStatus1(uint8_t id) const
    {
        return status1_[id - 1];
    }

    /**
     * @brief 判断是否已缓存到有效的状态1响应
     * @param id 1-based 电机索引
     */
    bool hasValidStatus1(uint8_t id) const
    {
        return status1_[id - 1].is_valid;
    }

    /**
     * @brief 获取状态1中的错误状态
     * @param id 1-based 电机索引
     * @return 错误状态位(0=无错误)
     */
    uint8_t getErrorState(uint8_t id) const
    {
        return status1_[id - 1].error_state;
    }

protected:
    HAL::CAN::ICanDevice *can_device_;     // 关联的 CAN 设备(发送命令帧)
    uint16_t init_address;                  // CAN ID 基地址(LK 固定 0x140)
    uint8_t  recv_idxs_[N] = {};           // 反馈帧 ID 偏移(电机 ID)
    uint32_t send_idxs_[N] = {};           // 命令帧 ID 偏移(电机 ID)
    LKParameters params_;                   // 电机参数(型号相关)
    LKFeedback feedback_[N] = {};          // 原始反馈数据(中间存储)
    LKStatus1  status1_[N] = {};           // 状态1缓存(0x9A 响应)

    // --- 多圈累计角度相关(拨盘位置环专用) ---
    // 设计原因：LK 协议只反馈单圈角度(0~2π)，位置环需要连续累计角度。
    float    multi_turn_angle_rad_[N] = {};  // 多圈累计角度(rad)
    float    last_angle_rad_[N]       = {};  // 上次单圈角度(rad, 用于跨边界检测)
    uint8_t  multi_turn_inited_[N]    = {};  // 多圈累计是否已初始化(0=未初始化, 1=已初始化)

    /**
     * @brief 将角度差归一化到 [-π, π](跨 0/2π 边界处理)
     * @param diff 角度差(curr - last)
     * @return 归一化后的角度差
     *
     * 用途：多圈累计时，若 curr 从 0.01 跳到 6.27，实际只转了 -0.02 rad，
     *       而不是 +6.26 rad。wrapToPi 把 +6.26 修正为 -0.02。
     */
    static inline float wrapToPi(float diff)
    {
        while (diff >  PI) diff -= 2.0f * PI;
        while (diff < -PI) diff += 2.0f * PI;
        return diff;
    }

    /**
     * @brief 将原始反馈数据转换为 SI 国际单位
     * @param i        电机槽位下标(0-based)
     * @param fb       原始反馈数据
     *
     * 转换公式：
     *   angle(rad)     = raw_angle × (360/65536) × (π/180)   [电机端单圈角度, 0~2π]
     *   velocity(rad/s)= raw_velocity × (1/减速比) × (π/180) [输出端角速度]
     *   torque(N·m)    = raw_current × 力矩转换系数          [输出端力矩]
     *   temperature(°C)= raw_temperature                      [直接值]
     *
     * @note 关于"电机端 vs 输出端"角度:
     *   - LK 协议反馈的 fb.angle 是电机端(转子)单圈角度(0~65535 → 0~360°)
     *   - 单圈角度 unit_data_[i].angle 保存为电机端 rad(0~2π), 用于跨边界检测
     *   - 多圈累计角度 multi_turn_angle_rad_ 是输出端累计 rad, 通过 delta 除以
     *     减速比实现 (参考工程 Lk_motor.hpp deg_to_real = 1/reduction_ratio)
     *   - 对外暴露的 getAddAngleRad()/getAddAngleDeg() 返回输出端累计角度
     *
     * @warning 不能把单圈角度直接除以减速比!
     *   若 curr_angle 除以减速比, 范围会变成 [0, 2π/减速比], wrapToPi() 的跨边界
     *   检测会完全失效(它假设输入是 [0, 2π]), 导致每跨边界 multi_turn 被错误地
     *   减去一整圈, feedback_angle 永远上不去。
     *   正确做法: 单圈角度保持电机端, 在 delta 累计时除以减速比。
     *
     * 多圈累计维护：
     *   首次解析：以当前单圈角度为起点，累计角度=0
     *   后续解析：delta = wrapToPi(curr - last) ÷ 减速比 → 累加到 multi_turn_angle_rad_
     */
    void Configure(size_t i, const LKFeedback &fb)
    {
        // 单圈角度: 编码器计数 → 电机端度 → 电机端rad (0~2π)
        // 保持电机端单圈范围, 用于 wrapToPi 跨边界检测
        float curr_angle = fb.angle * params_.encoder_to_deg * DEG2RAD;
        this->unit_data_[i].angle = curr_angle;

        // Angular velocity: motor-side dps feedback -> output-side rad/s.
        this->unit_data_[i].velocity = fb.velocity * params_.dps_to_output_radps;

        // 力矩: 反馈电流原始值 → 输出端力矩(N·m)
        this->unit_data_[i].current = fb.current * params_.current_to_torque;

        // 温度: 直接值(°C)
        this->unit_data_[i].temperature = (float)fb.temperature;

        // 角加速度: LK 协议无此字段, 预留
        this->unit_data_[i].accel = 0.0f;

        // --- 多圈累计角度维护(输出端, rad) ---
        if (!multi_turn_inited_[i])
        {
            // 首次解析：以当前位置为零位
            multi_turn_angle_rad_[i] = 0.0f;
            last_angle_rad_[i]       = curr_angle;
            multi_turn_inited_[i]    = 1;
        }
        else
        {
            // 跨边界处理：wrapToPi 把 ±2π 跳变修正为 ±0 附近的小值
            // (curr_angle 是电机端单圈角度, 范围 [0, 2π], wrapToPi 假设此范围)
            float delta_motor = wrapToPi(curr_angle - last_angle_rad_[i]);
            // 电机端 delta → 输出端 delta: 除以减速比
            // (电机转 10 圈 = 输出端转 1 圈)
            float delta_output = delta_motor / params_.reduction_ratio;
            multi_turn_angle_rad_[i] += delta_output;
            last_angle_rad_[i]        = curr_angle;
        }
    }

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
 * @class LK4005
 * @brief LK4005 / MG4005 电机驱动(单台, motor_id=1)
 *
 * 参数来源: 参考工程 H_SG-SG_New + LK-TECH 说明书
 *   - 减速比:        10:1
 *   - 力矩常数:      0.06 N·m/A
 *   - 反馈电流最大值: 2048(原始值)
 *   - 实际电流最大值: 4 A
 *   - 编码器分辨率:   65536 counts/rev(16位, 电机端/转子端)
 *
 * @note 单位换算(关键):
 *   - LK 协议反馈的 fb.angle 是电机端单圈角度(0~65535 → 0~360°电机端)
 *   - 单圈角度 unit_data_[i].angle 保持为电机端 rad(0~2π), 用于 wrapToPi 跨边界检测
 *   - 多圈累计角度 multi_turn_angle_rad_ 是输出端累计 rad, 通过 delta 除以
 *     减速比实现 (参考工程 Lk_motor.hpp deg_to_real = 1/reduction_ratio)
 *   - 对外暴露的 getAddAngleRad()/getAddAngleDeg() 返回输出端累计角度
 *
 * CAN 配置: motor_id=1(接 CAN1)
 *   - 命令帧 ID = 0x141
 *   - 反馈帧 ID = 0x141
 *
 * @note  使用前必须：
 *   1. 调用 On(1) 使能电机(否则不会上报反馈)
 *   2. 周期性调用 ctrl_Torque(1, 0) 维持反馈(即使不输出力矩也要发)
 */
class LK4005 : public LKMotorBase<1>
{
public:
    /**
     * @brief 构造函数
     * @param can_device  关联的 CAN 设备
     * @param motor_id    电机 ID(默认 1, 对应 CAN ID 0x141)
     */
    LK4005(HAL::CAN::ICanDevice *can_device, uint8_t motor_id = 1)
        : LKMotorBase<1>(can_device,
                         0x140,              // 基地址
                         {motor_id},          // 接收 ID 偏移 = motor_id
                         {motor_id},          // 发送 ID 偏移 = motor_id
                         LKParameters(10.0f, 0.06f, 2048.0f, 4.0f, 65536.0f))
    {
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
extern LK4005* lk4005_motor;

} // namespace BSP::MOTOR::LK

#endif // LK_MOTOR_HPP
