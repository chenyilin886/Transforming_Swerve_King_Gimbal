/**
 * @file BoardComm.cpp
 * @brief 云台-底盘板间通信实现
 *
 * 实现内容：
 *   1. Gimbal_to_Chassis::Update() - 更新发送数据
 *   2. Gimbal_to_Chassis::Data_send() - 发送 CAN 帧
 *   3. Gimbal_to_Chassis::HandleCANMessage() - 处理接收数据
 *   4. Gimbal_to_Chassis::CalcuGimbalToChassisAngle() - 计算云台-底盘角度误差
 *
 * 数据流：
 *   遥控器DR16 → Update() → direction.LX/LY
 *   Joint_Data.yaw → CalcuGimbalToChassisAngle() → direction.Yaw_encoder_angle_err
 *   direction/chassis_mode/ui_list → Data_send() → CAN2 (0x205/0x206)
 *
 *   CAN2 接收 → HandleCANMessage() → rx_refree
 *
 * 继承参考工程设计：
 *   - 带重试机制的 CAN 发送（3次，间隔1ms）
 *   - 帧头校验 + 超时检测
 *   - packed struct 打包方式
 *
 * @note 完全继承参考工程 H_SG_Gimbal 的 CommunicationTask.cpp 实现
 */

#include "BoardComm.hpp"
#include "ChassisModeManager.hpp"  // 底盘模式状态机
#include "../BSP/Remote/DR16.hpp"
#include "../Application/Variable.hpp"
#include "can_hal.hpp"
#include "cmsis_os.h"

namespace BoardComm
{

// ========================================================================
// 辅助函数：通道值映射
// ========================================================================

/**
 * @brief 将归一化通道值 [-1.0, 1.0] 映射到 [0, 220]
 *
 * @param value 归一化值（范围 [-1.0, 1.0]）
 * @return 映射后的值（范围 [0, 220]，中值 110）
 *
 * 映射公式：output = (value * 110) + 110
 *   - value = -1.0 → output = 0
 *   - value =  0.0 → output = 110
 *   - value =  1.0 → output = 220
 *
 * @note 参考工程使用此映射方式，底盘端需按同样公式解析
 */
static inline uint8_t channel_to_uint8(float value)
{
    return static_cast<uint8_t>(value * 110.0f + 110.0f);
}

// ========================================================================
// CAN 发送辅助函数（带重试机制）
// ========================================================================

/**
 * @brief 发送 CAN 帧（带重试机制）
 *
 * @param can_dev CAN 设备引用
 * @param frame   要发送的帧
 * @param retry_times 重试次数（默认3次）
 * @param retry_delay_ms 重试间隔（默认1ms）
 * @return true=发送成功，false=发送失败
 *
 * 设计原因：
 *   CAN 总线可能因总线繁忙或邮箱满导致发送失败，
 *   重试机制提高发送可靠性。
 *
 * @note 继承参考工程的 send_can_frame_retry 实现
 */
static bool send_can_frame_retry(HAL::CAN::ICanDevice &can_dev,
                                  const HAL::CAN::Frame &frame,
                                  uint32_t retry_times = 3,
                                  uint32_t retry_delay_ms = 1)
{
    for (uint32_t i = 0; i < retry_times; ++i)
    {
        if (can_dev.send(frame))
        {
            return true;
        }
        osDelay(retry_delay_ms);
    }

    return false;
}

// ========================================================================
// Gimbal_to_Chassis 实现
// ========================================================================

/**
 * @brief 更新发送数据
 *
 * 实现步骤：
 *   1. 读取 DR16 遥控器左右摇杆数据 → direction.LX/LY
 *   2. 读取 DR16 拨轮数据 → direction.wheel
 *   3. 计算云台-底盘角度误差 → direction.Yaw_encoder_angle_err
 *   4. 填充模式状态（从状态机获取）→ chassis_mode
 *
 * 数据源：
 *   - 遥控器：BSP::Remote::DR16::Instance().GetRemoteRight()
 *   - 拨轮：BSP::Remote::DR16::Instance().GetWheel()
 *   - 云台角度：Joint_Data.yaw.real_angle
 */
void Gimbal_to_Chassis::Update()
{
    // ========== 1. 读取遥控器摇杆数据 ==========
    auto &dr16 = BSP::Remote::DR16::Instance();
    using Switch = BSP::Remote::DR16::Switch;

    auto right_stick = dr16.GetRemoteRight();
    auto s1 = dr16.GetS1();
    auto s2 = dr16.GetS2();

    if (s1 == Switch::MIDDLE && s2 == Switch::UP)
    {
        direction.LX = 110;
        direction.LY = 110;
        direction.Rotating_vel = channel_to_uint8(static_cast<float>(right_stick.x));
    }
    else
    {
        direction.LX = channel_to_uint8(static_cast<float>(right_stick.x));
        direction.LY = channel_to_uint8(static_cast<float>(right_stick.y));

        if (s1 == Switch::UP && s2 == Switch::MIDDLE)
        {
            direction.Rotating_vel = channel_to_uint8(0.5f);
        }
        else
        {
            direction.Rotating_vel = 110;
        }
    }

    // ========== 2. 读取拨轮数据 ==========
    //   GetWheel() 返回 [-1.0, 1.0]
    //   映射到 int8_t [-127, 127]，中值 0
    float wheel_raw = BSP::Remote::DR16::Instance().GetWheel();
    direction.wheel = static_cast<int8_t>(wheel_raw * 127.0f);

    // ========== 3. 计算云台-底盘角度误差 ==========
    // 参考工程：使用编码器角度计算误差
    // 当前工程：使用 Joint_Data.yaw.real_angle
    direction.Yaw_encoder_angle_err = CalcuGimbalToChassisAngle();

    // ========== 4. 填充模式状态（从状态机获取）【修改】 ==========
    // 原设计：直接判断 S1/S2 开关状态（硬编码）
    // 新设计：调用 ChassisModeManager 状态机（架构清晰）
    //   - 状态机负责模式判断 + 状态滤波 + 离线检测
    //   - BoardComm 只负责数据打包
    chassis_mode = ChassisModeManager::Instance().GetChassisMode();

    // ========== 5. 填充 UI/视觉数据（预留） ==========
    ui_list.friction_enabled = Shoot_Status.friction_enable ? 1 : 0;
    ui_list.Vision = 0;            // 暂时关闭，后续接入视觉模块
    // 其他字段保持默认值（零初始化）

    // ========== 6. 同步到全局变量（Watch 观察） ==========
    BoardComm_Data.LX = direction.LX;
    BoardComm_Data.LY = direction.LY;
    BoardComm_Data.Rotating_vel = direction.Rotating_vel;
    BoardComm_Data.wheel = direction.wheel;
    BoardComm_Data.Yaw_encoder_angle_err = direction.Yaw_encoder_angle_err;
    BoardComm_Data.chassis_mode = *reinterpret_cast<uint8_t*>(&chassis_mode);
}

/**
 * @brief 发送 CAN 帧
 *
 * 实现步骤：
 *   1. 打包数据：帧头 + direction + chassis_mode + ui_list
 *   2. 分两帧发送：
 *      - 帧1 (0x205): 前8字节
 *      - 帧2 (0x206): 后8字节
 *   3. 调用带重试机制的发送函数
 *
 * 数据布局：
 *   tx_data[0]     : 帧头 0xA5
 *   tx_data[1-7]   : Direction 结构体（7字节）
 *   tx_data[8]     : ChassisMode 结构体（1字节）
 *   tx_data[9-14]  : UiList 结构体（6字节）
 *   总计：15字节（分两帧：前8字节 + 后7字节）
 *
 * @note 继承参考工程的发送逻辑，使用 CAN2 发送
 */
void Gimbal_to_Chassis::Data_send()
{
    // ========== 1. 获取 CAN2 设备 ==========
    auto &can2 = HAL::CAN::get_can_bus_instance().get_can2();

    // ========== 2. 打包发送数据 ==========
    uint8_t tx_data[16];  // 发送缓冲区（最大16字节）
    uint8_t *temp_ptr = tx_data;

    // 写入帧头
    *temp_ptr = TX_FRAME_HEAD;
    ++temp_ptr;

    // 写入 Direction 结构体（7字节）
    std::memcpy(temp_ptr, &direction, sizeof(direction));
    temp_ptr += sizeof(direction);

    // 写入 ChassisMode 结构体（1字节）
    std::memcpy(temp_ptr, &chassis_mode, sizeof(chassis_mode));
    temp_ptr += sizeof(chassis_mode);

    // 写入 UiList 结构体（6字节）
    std::memcpy(temp_ptr, &ui_list, sizeof(ui_list));

    // ========== 3. 发送帧1 (0x205) ==========
    HAL::CAN::Frame frame1{};
    frame1.id = CAN_G2C_FRAME1_ID;
    frame1.dlc = 8;
    frame1.is_extended_id = false;
    frame1.is_remote_frame = false;

    std::memcpy(can_tx_buffer[0], tx_data, 8);
    std::memcpy(frame1.data, can_tx_buffer[0], 8);

    if (!send_can_frame_retry(can2, frame1))
    {
        // 发送失败，直接返回（参考工程做法）
        return;
    }

    // ========== 4. 发送帧2 (0x206) ==========
    HAL::CAN::Frame frame2{};
    frame2.id = CAN_G2C_FRAME2_ID;
    frame2.dlc = 8;
    frame2.is_extended_id = false;
    frame2.is_remote_frame = false;

    std::memcpy(can_tx_buffer[1], tx_data + 8, 8);
    std::memcpy(frame2.data, can_tx_buffer[1], 8);

    if (!send_can_frame_retry(can2, frame2))
    {
        // 发送失败，直接返回
        return;
    }
}

/**
 * @brief 处理底盘返回数据
 *
 * @param std_id CAN 标准帧 ID（0x207 或 0x208）
 * @param data   数据指针
 * @param dlc    数据长度
 *
 * 实现步骤：
 *   1. 帧头校验（检查 0x21 0x12）
 *   2. 超时检测（50ms 内未收到完整数据则丢弃）
 *   3. 解析裁判系统数据：
 *      - 帧1 (0x207): booster_heat_cd / booster_heat_max / booster_now_heat
 *      - 帧2 (0x208): launch_speed
 *
 * 状态机：
 *   - 收到帧1 → 设置 rx_refree_frame1_ready = true
 *   - 收到帧2 → 检查帧1是否就绪 → 解析并更新 rx_refree
 *
 * @note 继承参考工程的接收逻辑
 */
void Gimbal_to_Chassis::HandleCANMessage(uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
    const uint32_t now = HAL_GetTick();

    // ========== 1. 帧类型判断 ==========
    const bool is_frame1 =
        (std_id == CAN_CHASSIS_TO_GIMBAL_FRAME1_ID) &&
        (dlc >= sizeof(RxRefreeFrame1_t)) &&
        (data[0] == RX_FRAME_HEAD1) &&
        (data[1] == RX_FRAME_HEAD2);

    const bool is_frame2 =
        ((std_id == CAN_CHASSIS_TO_GIMBAL_FRAME2_ID) || (std_id == CAN_CHASSIS_TO_GIMBAL_FRAME1_ID)) &&
        (dlc >= sizeof(RxRefreeFrame2_t));

    // ========== 2. 超时检测 ==========
    if (rx_refree_frame1_ready && (now - last_frame_time > FRAME_TIMEOUT))
    {
        // 帧1 接收后超过 50ms 未收到帧2 → 丢弃帧1，重新等待
        rx_refree_frame1_ready = false;
    }

    // ========== 3. 处理帧1 ==========
    if (is_frame1)
    {
        RxRefreeFrame1_t frame1{};
        std::memcpy(&frame1, data, sizeof(frame1));

        // 更新裁判系统数据
        rx_refree.booster_heat_cd = frame1.booster_heat_cd;
        rx_refree.booster_heat_max = frame1.booster_heat_max;
        rx_refree.booster_now_heat = frame1.booster_now_heat;

        // 标记帧1 已就绪
        rx_refree_frame1_ready = true;
        last_frame_time = now;

        // 同步到全局变量（Watch 观察）
        BoardComm_Data.booster_heat_cd = rx_refree.booster_heat_cd;
        BoardComm_Data.booster_heat_max = rx_refree.booster_heat_max;
        BoardComm_Data.booster_now_heat = rx_refree.booster_now_heat;
        BoardComm_Data.rx_frame1_ready = rx_refree_frame1_ready ? 1 : 0;
        BoardComm_Data.last_rx_time = last_frame_time;

        return;
    }

    // ========== 4. 处理帧2 ==========
    if (is_frame2)
    {
        // 检查帧1 是否就绪（防止帧2 先于帧1 到达）
        if (!rx_refree_frame1_ready || is_frame1)
        {
            return;
        }

        RxRefreeFrame2_t frame2{};
        std::memcpy(&frame2, data, sizeof(frame2));

        // 更新发射速度
        rx_refree.launch_speed = frame2.launch_speed;

        // 清除帧1 就绪标志（等待下一轮）
        rx_refree_frame1_ready = false;
        last_frame_time = now;

        // 同步到全局变量（Watch 观察）
        BoardComm_Data.launch_speed = rx_refree.launch_speed;
        BoardComm_Data.rx_frame1_ready = 0;
        BoardComm_Data.last_rx_time = last_frame_time;
    }
}

/**
 * @brief 计算云台相对底盘角度误差（最短路径归一化）
 *
 * @return 角度误差(rad)，范围 [-π, π]
 *
 * 实现方式：
 *   使用 BSP::JOINT::wrapToPi() 将编码器累积角度归一化到 [-π, π]，
 *   确保底盘跟随时走最短路径。
 *   修复前：编码器累积角度可能超过 ±π（如小陀螺360° → 误差360° → 底盘转360°）
 *   修复后：归一化到 [-π, π]（如360° → 0° → 底盘不转，就近归位）
 *
 * @note Yaw 是连续旋转关节(continuous=1)，normalized_angle 已由 Joint::Update
 *       通过 wrapToPi(real_angle) 维护在 [-π, π]，直接使用即可。
 */
float Gimbal_to_Chassis::CalcuGimbalToChassisAngle()
{
    // Yaw 关节的 normalized_angle = wrapToPi(real_angle)
    // real_angle = (encoder - offset) * direction
    // 连续旋转关节(continuous=1)，normalized_angle 已被 wrap 到 [-π, π]
    float yaw_angle = Joint_Data.yaw.normalized_angle;

    return yaw_angle;
}

} // namespace BoardComm
