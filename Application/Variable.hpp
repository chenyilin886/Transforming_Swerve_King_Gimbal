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
 * @brief 单关节 Controller 数据（位置式 PID，支持单级/串级）
 *
 *   --- 输入(可写) ---
 *   target_angle      : 目标角度(rad)
 *   kp/ki/kd          : 角度环 PID 参数（单级模式 = 主 PID；串级模式 = 外环）
 *   torque_limit       : 输出力矩限幅(N·m)
 *   break_i            : 角度环积分隔离阈值(rad)
 *   limit_i            : 角度环积分输出限幅(N·m)
 *   cascade_mode       : 串级模式开关(0=单级位置式, 1=串级 角度环+速度环均位置式)
 *   vel_kp/vel_ki/vel_kd : 速度环 PID 参数（仅 cascade_mode=1 时生效）
 *   vel_limit          : 速度环输出限幅(rad/s) = 角度环输出限幅
 *   break_i_vel        : 速度环积分隔离阈值(rad/s)
 *   limit_i_vel        : 速度环积分输出限幅(N·m)
 *   enabled            : 使能(1=控制中)
 *
 *   --- 输出(只读) ---
 *   feedback_angle    : 反馈角度(rad)
 *   error             : 角度环误差(rad)
 *   vel_target        : 速度环目标(rad/s) = 角度环 PID 输出（串级模式有效）
 *   vel_feedback      : 速度环反馈(rad/s) = Joint.velocity
 *   vel_error         : 速度环误差(rad/s)
 *   torque_output     : 最终输出力矩(N·m)
 *   limit_min/max     : 关节限位(rad)
 */
typedef struct
{
    // --- 输入(Watch 可调) ---
    float    target_angle;     // 目标角度(rad)
    float    kp;               // 角度环比例系数
    float    ki;               // 角度环积分系数
    float    kd;               // 角度环微分系数
    float    torque_limit;     // 输出力矩限幅(N·m)
    float    break_i;          // 角度环积分隔离阈值(rad)
    float    limit_i;          // 角度环积分输出限幅(N·m)
    uint8_t  cascade_mode;     // 串级模式开关: 0=单级位置式, 1=串级(角度环+速度环均位置式)
    float    vel_kp;           // 速度环比例系数(仅 cascade_mode=1 生效)
    float    vel_ki;           // 速度环积分系数
    float    vel_kd;           // 速度环微分系数
    float    vel_limit;        // 速度环输出限幅(rad/s) = 角度环输出限幅
    float    break_i_vel;      // 速度环积分隔离阈值(rad/s)
    float    limit_i_vel;      // 速度环积分输出限幅(N·m)
    uint8_t  enabled;          // 使能标志(1=控制中)

    // --- 重力补偿(Watch 可调, 仅串级模式生效) ---
    float    gravity_k;        // 重力补偿系数(N·m), 公式: m·g·r, Watch 在线标定
    uint8_t  gravity_enable;   // 重力补偿使能: 0=关, 1=开(串级模式专用, Fold Stage04)

    // --- 输出(Watch 只读) ---
    float    feedback_angle;   // 反馈角度(rad)
    float    error;            // 角度环误差(rad)
    float    vel_target;       // 速度环目标(rad/s) = 角度环 PID 输出
    float    vel_feedback;     // 速度环反馈(rad/s)
    float    vel_error;        // 速度环误差(rad/s)
    float    torque_output;    // PID 输出力矩(N·m)
    float    gravity_torque;   // 重力补偿输出(N·m), Watch 观察用
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
// Stage05 变形规划器配置参数（Watch 可调）
// ========================================================================
/**
 * @brief 变形动作规划器配置（Stage05）
 *
 * Watch 中可直接修改这些参数，实时影响变形动作：
 *   pitch_expand      : 展开后 Pitch 水平位(rad) [Fold 展开后 Pitch 的水平目标]
 *   pitch_contract    : 收起 Pitch 角度(rad)      [最收缩位]
 *   fold_expand       : 展开 Fold 角度(rad)       [最大上抬角度]
 *   fold_contract     : 收起 Fold 角度(rad)       [最小角度 0]
 *   arrive_eps        : 到位误差阈值(rad)         [默认 0.02 ≈ 1.1°]
 *   arrive_timeout_ms : 单步超时(ms)              [默认 3000，避免卡死]
 *   cmd               : 命令输入(单次触发)
 *                       0=NONE  1=EXPAND  2=CONTRACT  3=ABORT  4=RESET
 *
 * 动作序列（严格串行，避免 Fold 转动时干扰 Pitch）：
 *   展开: pitch→pitch_expand → 等 pitch 到位 → fold→fold_expand → EXPANDED
 *   收起: fold→fold_contract → 等 fold 到位  → pitch→pitch_contract → CONTRACTED
 *
 * @note cmd 由 Planner 消费后自动清零（cfg.cmd=0）
 */
typedef struct
{
    float    pitch_expand;       // 展开后 Pitch 水平位(rad), 实测 0.126987755
    float    pitch_contract;     // 收起 Pitch 角度(rad),     实测 -0.792750061
    float    fold_expand;        // 展开 Fold 角度(rad),      实测 0.901044846
    float    fold_contract;      // 收起 Fold 角度(rad),      实测 0.0
    float    arrive_eps;         // 到位误差阈值(rad), 默认 0.02
    uint16_t arrive_timeout_ms;  // 单步超时(ms),      默认 3000
    uint8_t  cmd;                // 命令: 0=NONE 1=EXPAND 2=CONTRACT 3=ABORT 4=RESET
} Transform_Config_t;

// ========================================================================
// Stage05 变形规划器状态（Watch 观察）
// ========================================================================
/**
 * @brief 变形动作规划器运行时状态（Stage05）
 *
 * Watch 中展开 Transform_Status 即可观察变形过程：
 *   state            : 当前状态(0=IDLE, 1=EXPAND_PITCH_PRE, 2=EXPAND_FOLD_DEPLOY,
 *                      3=EXPANDED, 4=CONTRACT_FOLD_RETURN, 5=CONTRACT_PITCH_RETURN,
 *                      6=CONTRACTED, 7=ABORT)
 *   step             : 当前步骤序号(0=待机/终态, 1=第一步, 2=第二步)
 *   pitch_target_now : 当前下发的 pitch target(rad)
 *                      [TRANSITION: Planner 写入值; 终态/IDLE/ABORT: Controller_Data 中的值]
 *   fold_target_now  : 当前下发的 fold target(rad)
 *   pitch_err        : 实时 pitch 误差(rad) = pitch_target_now - pitch_fb
 *   fold_err         : 实时 fold 误差(rad)  = fold_target_now  - fold_fb
 *   step_elapsed_ms  : 当前步已耗时(ms), 仅 TRANSITION 状态累加
 *   pitch_online     : pitch 在线状态(1=在线)
 *   fold_online      : fold 在线状态(1=在线)
 *   last_error       : ABORT 原因(0=正常, 1=TIMEOUT, 2=MOTOR_OFFLINE, 3=ABORT_CMD)
 */
typedef struct
{
    uint8_t  state;               // 当前状态(0..7)
    uint8_t  step;                // 当前步骤序号(0/1/2)
    float    pitch_target_now;    // 当前 pitch target(rad)
    float    fold_target_now;     // 当前 fold target(rad)
    float    pitch_err;           // pitch 误差(rad)
    float    fold_err;            // fold 误差(rad)
    uint16_t step_elapsed_ms;     // 当前步已耗时(ms)
    uint8_t  pitch_online;        // pitch 在线状态
    uint8_t  fold_online;         // fold 在线状态
    uint8_t  last_error;          // ABORT 原因(0/1/2/3)
} Transform_Status_t;

// ========================================================================
// 全局变量 extern 声明(定义在 Variable.cpp)
// ========================================================================
extern Joint_Data_t       Joint_Data;        // 关节状态(Stage01-02)
extern Controller_Data_t  Controller_Data;   // 控制器状态(Stage03)
extern Transform_Config_t  Transform_Config;  // 变形规划器配置(Stage05)
extern Transform_Status_t  Transform_Status;  // 变形规划器状态(Stage05)

// ========================================================================
// VOFA+ 调试通道发送函数(定义在 Variable.cpp)
// ========================================================================
/**
 * @brief VOFA+ 6 通道发送函数（在 GimbalUpdate 中调用）
 *
 * 数据来源：Controller_Data.pitch（方便在 Variable.cpp 中修改通道配置）
 *
 * 通道分配（Stage03 Pitch 串级 PID 调参观测）：
 *   CH0: pitch.target_angle    目标角度（rad）       外环输入
 *   CH1: pitch.feedback_angle  反馈角度（rad）       外环反馈
 *   CH2: pitch.error           角度环误差（rad）     外环误差
 *   CH3: pitch.torque_output   输出力矩（N·m）       内环输出
 *   CH4: pitch.vel_target      速度环目标（rad/s）   外环输出=内环输入
 *   CH5: pitch.vel_feedback    速度环反馈（rad/s）   内环反馈
 *
 * @note 调用频率：由 GimbalInit.cpp 控制（500Hz 降频）
 *       修改通道配置：只需改 Variable.cpp 中的实现，无需改 GimbalInit.cpp
 */
extern void VofaSendDebugChannels(void);

#ifdef __cplusplus
}
#endif

#endif // GIMBAL_VARIABLE_HPP
