/**
 * @file BoardComm.hpp
 * @brief 云台-底盘板间通信管理（CAN2）
 *
 * 设计原因：
 *   云台和底盘分属两块MCU，需要通过CAN总线交换数据：
 *   - 云台→底盘：遥控器通道、云台角度、控制模式
 *   - 底盘→云台：裁判系统数据（热量、发射速度）
 *
 * 继承参考工程设计：
 *   - 帧头校验（0x21 0x12）
 *   - 超时检测（50ms）
 *   - 重试发送（3次）
 *   - packed struct 紧凑布局
 *   - 16字节数据分两帧传输（0x205/0x206 发送，0x207/0x208 接收）
 *
 * 数据流（发送）：
 *   遥控器DR16 右摇杆 / Joint_Data.yaw
 *     → Gimbal_to_Chassis::Update()
 *       → 填充 direction / chassis_mode 结构体
 *         → Data_send() → CAN2 发送 0x205/0x206
 *
 * 数据流（接收）：
 *   CAN2 中断 → HAL_CAN_RxFifo1MsgPendingCallback
 *     → Gimbal_to_Chassis::HandleCANMessage()
 *       → 解析 0x207/0x208 → rx_refree
 *
 * 使用方式：
 *   1. 在 main.cpp 中周期调用：
 *      board_comm.Update();    // 更新发送数据
 *      board_comm.Data_send(); // 发送CAN帧
 *
 *   2. 在 CanCallback.cpp 的 CAN2 中断回调中：
 *      board_comm.HandleCANMessage(frame.id, frame.data, frame.dlc);
 *
 *   3. 发射状态机可读取裁判系统数据：
 *      board_comm.getLaunchSpeed()
 *      board_comm.getBoosterNowHeat()
 *
 * @note 完全继承参考工程 H_SG_Gimbal 的 CommunicationTask 架构
 *       数据内容暂时保持一致，后续再根据三关节云台特性扩展
 */

#pragma once

#include "main.h"
#include <cstdint>
#include <cstring>

namespace BoardComm
{

// ========================================================================
// CAN 帧ID定义（完全继承参考工程）
// ========================================================================
#define CAN_G2C_FRAME1_ID 0x205  // 云台→底盘 帧1
#define CAN_G2C_FRAME2_ID 0x206  // 云台→底盘 帧2

#define CAN_CHASSIS_TO_GIMBAL_FRAME1_ID 0x207  // 底盘→云台 帧1
#define CAN_CHASSIS_TO_GIMBAL_FRAME2_ID 0x208  // 底盘→云台 帧2

// ========================================================================
// 云台→底盘数据结构（完全继承参考工程）
// ========================================================================

/**
 * @brief 方向控制数据
 *
 * 字段说明：
 *   LX/LY             : 遥控器右摇杆（X/Y轴），范围 0-220，中值 110
 *   Rotating_vel      : 小陀螺模式速度（范围 0-220，中值 110）
 *   Yaw_encoder_angle_err : 云台相对底盘的角度误差（rad）
 *   target_offset_angle    : 目标偏角（预留）
 *   wheel             : 遥控器拨轮值，范围 [-127, 127]，中值 0
 *
 * @note uint8_t 类型数据映射到 [0, 220]，中值为 110
 *       int8_t 类型数据范围 [-127, 127]，中值为 0
 *       float 类型直接传输原始值
 */
struct __attribute__((packed)) Direction_t
{
    uint8_t LX;                     // 右摇杆X通道（0-220，中值110）
    uint8_t LY;                     // 右摇杆Y通道（0-220，中值110）
    uint8_t Rotating_vel;           // 小陀螺速度（0-220，中值110）
    float Yaw_encoder_angle_err;    // 云台-底盘角度误差（rad）
    uint8_t target_offset_angle;    // 目标偏角（预留）
    int8_t wheel;                   // 拨轮值（-127~127，中值0）
};

/**
 * @brief 底盘控制模式（位域）
 *
 * 位域说明：
 *   Universal_mode : 通用模式
 *   Follow_mode    : 跟随模式（底盘跟随云台）
 *   Rotating_mode  : 小陀螺模式
 *   KeyBoard_mode  : 键盘模式
 *   stop           : 停止模式
 *
 * @note 使用位域节省传输带宽
 */
struct __attribute__((packed)) ChassisMode_t
{
    uint8_t Universal_mode : 1;  // 通用模式
    uint8_t Follow_mode : 1;     // 跟随模式
    uint8_t Rotating_mode : 1;   // 小陀螺模式
    uint8_t KeyBoard_mode : 1;   // 键盘模式
    uint8_t stop : 1;            // 停止模式
};

/**
 * @brief UI及视觉相关数据（预留）
 *
 * 字段说明：
 *   MCL/BP/UI_F5    : UI相关标志位（预留）
 *   Vision          : 视觉状态（2bit）
 *   friction_enabled: 摩擦轮使能状态
 *   aim_x/aim_y     : 瞄准点坐标（预留）
 *   projectile_count: 弹丸计数（预留）
 *
 * @note 当前阶段暂不使用，预留扩展
 */
struct __attribute__((packed)) UiList_t
{
    uint8_t MCL : 1;              // UI预留
    uint8_t BP : 1;               // UI预留
    uint8_t UI_F5 : 1;            // UI预留
    uint8_t Shift : 1;            // Shift键状态
    uint8_t Vision : 2;           // 视觉状态
    uint8_t friction_enabled : 1; // 摩擦轮使能
    uint8_t aim_x;                // 瞄准点X（预留）
    uint8_t aim_y;                // 瞄准点Y（预留）
    int16_t projectile_count;     // 弹丸计数（预留）
};

// ========================================================================
// 底盘→云台数据结构（完全继承参考工程）
// ========================================================================

/**
 * @brief 裁判系统数据帧1（底盘→云台）
 *
 * 帧头：0x21 0x12
 * 内容：电容冷却时间 / 热量上限 / 当前热量
 */
struct __attribute__((packed)) RxRefreeFrame1_t
{
    uint8_t head1;              // 帧头1: 0x21
    uint8_t head2;              // 帧头2: 0x12
    uint16_t booster_heat_cd;   // 电容冷却时间
    uint16_t booster_heat_max;  // 热量上限
    uint16_t booster_now_heat;  // 当前热量
};

/**
 * @brief 裁判系统数据帧2（底盘→云台）
 *
 * 内容：发射速度
 */
struct __attribute__((packed)) RxRefreeFrame2_t
{
    float launch_speed;         // 发射速度(m/s)
};

/**
 * @brief 裁判系统数据聚合
 *
 * 存储：冷却时间 / 热量上限 / 当前热量 / 发射速度
 */
struct RxRefree_t
{
    uint16_t booster_heat_cd = 0;   // 电容冷却时间
    uint16_t booster_heat_max = 0;  // 热量上限
    uint16_t booster_now_heat = 0;  // 当前热量
    float launch_speed = 0.0f;      // 发射速度(m/s)
};

// ========================================================================
// 云台→底盘板间通信管理类
// ========================================================================

/**
 * @brief 云台→底盘板间通信管理
 *
 * 职责：
 *   1. 定时发送遥控器通道 / 云台角度 / 模式状态（CAN2, 0x205/0x206）
 *   2. 接收底盘裁判系统数据（CAN2, 0x207/0x208）
 *   3. 数据打包 / 解包 / 校验
 *
 * 设计原则：
 *   - 继承参考工程架构
 *   - 数据内容暂时保持一致
 *   - 后续根据三关节云台特性扩展（如增加 Fold 状态）
 */
class Gimbal_to_Chassis
{
public:
    /**
     * @brief 构造函数
     */
    Gimbal_to_Chassis() = default;

    /**
     * @brief 更新发送数据（上层调用）
     *
     * 功能：
 *   1. 读取 DR16 遥控器右摇杆数据
 *   2. 读取 DR16 拨轮数据
 *   3. 读取 Joint_Data.yaw 角度（用于计算云台-底盘角度误差）
 *   4. 填充 direction / chassis_mode 结构体
     *
     * 调用时机：
     *   建议在主任务中周期调用（4ms 周期）
     *
     * @note 当前阶段模式状态暂不实现（状态机未完成）
     */
    void Update();

    /**
     * @brief 发送 CAN 帧（CAN2, 0x205/0x206）
     *
     * 功能：
     *   1. 打包 16 字节数据（direction + chassis_mode + ui_list + 帧头）
     *   2. 分两帧发送（0x205: 前8字节，0x206: 后8字节）
     *   3. 带重试机制（3次，间隔1ms）
     *
     * 调用时机：
     *   建议在主任务中周期调用（4ms 周期）
     */
    void Data_send();

    /**
     * @brief 处理底盘返回数据（CAN2, 0x207/0x208）
     *
     * @param std_id CAN 标准帧 ID
     * @param data   数据指针
     * @param dlc    数据长度
     *
     * 功能：
     *   1. 帧头校验（0x21 0x12）
     *   2. 超时检测（50ms）
     *   3. 解析裁判系统数据到 rx_refree
     *
     * @note 在 CanCallback.cpp 的 HAL_CAN_RxFifo1MsgPendingCallback 中调用
     */
    void HandleCANMessage(uint32_t std_id, const uint8_t *data, uint8_t dlc);

    // ========== Getter 接口（发射状态机使用） ==========

    /**
     * @brief 获取电容冷却时间
     * @return 冷却时间
     */
    uint16_t getBoosterHeatCd() const { return rx_refree.booster_heat_cd; }

    /**
     * @brief 获取热量上限
     * @return 热量上限
     */
    uint16_t getBoosterHeatLimit() const { return rx_refree.booster_heat_max; }

    /**
     * @brief 获取当前热量
     * @return 当前热量
     */
    uint16_t getBoosterNowHeat() const { return rx_refree.booster_now_heat; }

    /**
     * @brief 获取发射速度
     * @return 发射速度(m/s)
     */
    float getLaunchSpeed() const { return rx_refree.launch_speed; }

    /**
     * @brief 获取单例实例
     * @return Gimbal_to_Chassis 实例引用
     */
    static Gimbal_to_Chassis& Instance()
    {
        static Gimbal_to_Chassis instance;
        return instance;
    }

private:
    /**
     * @brief 计算云台相对底盘角度误差
     * @return 角度误差(rad)
     *
     * 实现方式：
     *   当前使用 Yaw 编码器角度计算误差
     *   后续可替换为 IMU 姿态（更准确）
     *
     * @note 参考工程使用 Init_Angle 作为底盘初始角度
     */
    float CalcuGimbalToChassisAngle();

    // --- 发送数据结构体 ---
    Direction_t direction{};        ///< 方向控制数据
    ChassisMode_t chassis_mode{};   ///< 模式控制数据
    UiList_t ui_list{};             ///< UI/视觉数据

    // --- 接收数据结构体 ---
    RxRefree_t rx_refree{};         ///< 裁判系统数据

    // --- CAN 发送缓冲 ---
    uint8_t can_tx_buffer[2][8];    ///< 两帧发送缓冲（0: 0x205, 1: 0x206）

    // --- 接收状态机 ---
    uint32_t last_frame_time = 0;                ///< 最后接收时间戳
    static constexpr uint32_t FRAME_TIMEOUT = 50; ///< 超时阈值(ms)
    bool rx_refree_frame1_ready = false;         ///< 帧1 接收就绪标志

    // --- 常量 ---
    static constexpr uint8_t RX_FRAME_HEAD1 = 0x21;  ///< 接收帧头1
    static constexpr uint8_t RX_FRAME_HEAD2 = 0x12;  ///< 接收帧头2
    static constexpr uint8_t TX_FRAME_HEAD = 0xA5;   ///< 发送帧头

    // --- 配置参数（临时，后续应移到 Variable.hpp） ---
    int16_t Init_Angle = 107;  ///< 底盘初始角度（参考工程默认值，后续需校准）
};

} // namespace BoardComm