/**
 * @file Variable.hpp
 * @brief 云台全局变量声明(集中管理)
 *
 * 数据流(DAY01 Joint 层)：
 *   CAN1 → DM4310/DM4340::Parse → unit_data_[]
 *   → JointManager::Update() → Joint.Update()
 *     ├─ encoder_angle   (编码器原始角度)
 *     ├─ real_angle      (减 offset × direction)
 *     ├─ normalized_angle (归一化/限位)
 *     └─ 写入 Joint_Data → Keil Watch 实时观察
 *
 * 后续扩展：
 *   - PID 层：追加 Kpid_t Kpid_yaw_angle 等参数实例
 *   - 状态机：追加 GimbalState_Data_t
 *   - 遥控器：追加 Remote_Data_t
 */
#ifndef GIMBAL_VARIABLE_HPP
#define GIMBAL_VARIABLE_HPP

#include "main.h"   // 提供 uint8_t 等基础类型(C/C++ 兼容)

#ifdef __cplusplus
extern "C" {
#endif

// ========================================================================
// 关节配置参数(Watch 可在线修改)
// ========================================================================
/**
 * @brief 单关节配置参数
 *
 * Watch 中可直接修改这些参数，实时影响 Joint 计算：
 *   offset      : 零位偏移(rad)，real_angle = (encoder - offset) * direction
 *   limit_min   : 机械限位下限(rad)，仅非连续关节生效
 *   limit_max   : 机械限位上限(rad)
 *   direction   : 方向系数(1.0=正, -1.0=反)
 *   continuous  : 1=连续旋转(Yaw), 0=有限位(Pitch/Fold)
 *   calib_enable: Watch 置 1 → 以当前编码器位置为零位(单次触发)
 *   calib_state : 0=未校准, 1=已校准(只读)
 */
typedef struct
{
    float    offset;        // 零位偏移(rad)
    float    limit_min;     // 限位下限(rad)
    float    limit_max;     // 限位上限(rad)
    float    direction;     // 方向系数(1.0 / -1.0)
    uint8_t  continuous;   // 连续旋转标志(1=Yaw, 0=Pitch/Fold)
    uint8_t  calib_enable;  // 校准使能(Watch 置 1 触发)
    uint8_t  calib_state;   // 校准状态(0=未校准, 1=已校准)
} Joint_Config_t;

// ========================================================================
// 单关节完整数据结构(Watch 可观察)
// ========================================================================
/**
 * @brief 单关节数据(状态 + 配置)
 *
 * Watch 中展开一个关节即可看到全部字段：
 *   --- 运行时状态(只读) ---
 *   encoder_angle   : 编码器原始角度(rad) - Motor 直接反馈值
 *   real_angle      : 真实角度(rad) = (encoder - offset) * direction
 *   normalized_angle: 归一化角度(rad) - Yaw: [-π,π], Pitch/Fold: clamped
 *   target_angle    : 目标角度(rad) - 上层设定(Day02 PID 使用)
 *   velocity        : 角速度(rad/s)
 *   torque          : 力矩(N·m)
 *   temperature     : 温度(℃)
 *   online          : 在线状态(1=在线)
 *   --- 配置参数(可调) ---
 *   config.offset / limit_min / limit_max / direction / continuous / calib_*
 */
typedef struct
{
    // --- 运行时状态(Joint.Update 填充) ---
    float    encoder_angle;    // 编码器角度(rad)
    float    real_angle;       // 真实角度(rad)
    float    normalized_angle; // 归一化角度(rad)
    float    target_angle;     // 目标角度(rad)
    float    velocity;         // 角速度(rad/s)
    float    torque;           // 力矩(N·m)
    float    temperature;      // 温度(℃)
    uint8_t  online;           // 在线状态(1=在线, 0=离线)

    // --- 配置参数(Watch 可调) ---
    Joint_Config_t config;     // 关节配置(offset/limit/direction/...)
} Joint_Data_Unit_t;

// ========================================================================
// 三关节聚合结构
// ========================================================================
/**
 * @brief 云台三关节数据聚合
 *
 * Watch 中添加 Joint_Data 即可展开三关节全部状态 + 配置：
 *   Joint_Data.yaw.encoder_angle / .real_angle / .normalized_angle / ...
 *   Joint_Data.pitch.encoder_angle / .real_angle / ...
 *   Joint_Data.fold.encoder_angle / .real_angle / ...
 *
 * 关节配置(DAY01 默认)：
 *   - yaw   : DM4310 #1, continuous=1, 无限位
 *   - pitch : DM4310 #2, continuous=0, ±90°(±1.5708rad)
 *   - fold  : DM4340 #1, continuous=0, ±90°(±1.5708rad)
 */
typedef struct
{
    Joint_Data_Unit_t yaw;     // Yaw 关节
    Joint_Data_Unit_t pitch;   // Pitch 关节
    Joint_Data_Unit_t fold;    // Fold 关节
} Joint_Data_t;

// ========================================================================
// 单关节 Controller 数据结构(Watch 可观察 + 在线调参)
// ========================================================================
/**
 * @brief 单关节 Controller 数据（位置式 PID）
 *
 *   --- 输入(可写) ---
 *   target_angle      : 目标角度(rad)
 *   kp/ki/kd          : PID 参数
 *   torque_limit       : 力矩限幅(N·m)
 *   break_i            : 积分隔离阈值(rad)
 *   limit_i            : 积分输出限幅(N·m)
 *   enabled            : 使能(1=控制中)
 *
 *   --- 输出(只读) ---
 *   feedback_angle    : 反馈角度(rad)
 *   error             : 位置误差(rad)
 *   torque_output     : 最终输出力矩(N·m)
 *   limit_min/max     : 关节限位(rad)
 */
typedef struct
{
    // --- 输入(Watch 可调) ---
    float    target_angle;     // 目标角度(rad)
    float    kp;               // 比例系数
    float    ki;               // 积分系数
    float    kd;               // 微分系数
    float    torque_limit;     // 输出力矩限幅(N·m)
    float    break_i;          // 积分隔离阈值(rad)
    float    limit_i;          // 积分输出限幅(N·m)
    uint8_t  enabled;          // 使能标志(1=控制中)

    // --- 输出(Watch 只读) ---
    float    feedback_angle;   // 反馈角度(rad)
    float    error;            // 位置误差(rad)
    float    torque_output;    // PID 输出力矩(N·m)
    float    limit_min;        // 关节限位下限(rad)，从 Joint 同步
    float    limit_max;        // 关节限位上限(rad)，从 Joint 同步
} Controller_Data_Unit_t;

// ========================================================================
// 三关节 Controller 聚合
// ========================================================================
/**
 * @brief 云台三关节 Controller 数据聚合
 *
 * Watch 中添加 Controller_Data 即可展开三关节全部控制状态：
 *   Controller_Data.pitch.target_angle  ← Watch 改这个值即可控制 Pitch
 *   Controller_Data.pitch.kp            ← Watch 改这个值即可在线调 P
 *   Controller_Data.pitch.error         ← Watch 观察跟踪误差
 *   Controller_Data.pitch.torque_output  ← Watch 观察 MIT 输出力矩
 *
 * Stage03 默认使能：
 *   pitch.enabled = 1
 *   yaw.enabled   = 0
 *   fold.enabled  = 0
 */
typedef struct
{
    Controller_Data_Unit_t yaw;     // Yaw 控制器数据
    Controller_Data_Unit_t pitch;  // Pitch 控制器数据
    Controller_Data_Unit_t fold;   // Fold 控制器数据
} Controller_Data_t;

// ========================================================================
// 全局变量 extern 声明(定义在 Variable.cpp)
// ========================================================================
extern Joint_Data_t       Joint_Data;        // 关节状态(Stage01-02)
extern Controller_Data_t  Controller_Data;   // 控制器状态(Stage03)

#ifdef __cplusplus
}
#endif

#endif // GIMBAL_VARIABLE_HPP
