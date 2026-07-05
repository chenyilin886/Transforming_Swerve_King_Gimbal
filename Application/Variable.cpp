/**
 * @file Variable.cpp
 * @brief 云台全局变量定义(集中存放)
 *
 * 当前内容(DAY01 Joint 层)：
 *   - Joint_Data：三关节(Yaw/Pitch/Fold)完整数据(状态 + 配置)
 *
 * 初始化策略：
 *   所有运行时状态初始化为 0，配置参数使用 DAY01 默认值。
 *   online=0 表示初始离线，首次收到反馈帧后置 1。
 *   calib_state=0 表示未校准，Watch 中置 calib_enable=1 触发校准。
 */
#include "Variable.hpp"

// ========================================================================
// 三关节数据全局实例
// ========================================================================
// 配置参数默认值：
//   Yaw  : offset=0, continuous=1, 无限位, direction=1
//   Pitch: offset=0, continuous=0, ±1.5708rad(±90°), direction=1
//   Fold : offset=0, continuous=0, ±1.5708rad(±90°), direction=1
Joint_Data_t Joint_Data = {
    // --- Yaw ---
    .yaw = {
        .encoder_angle    = 0.0f,
        .real_angle       = 0.0f,
        .normalized_angle = 0.0f,
        .target_angle     = 0.0f,
        .velocity         = 0.0f,
        .torque           = 0.0f,
        .temperature      = 0.0f,
        .online           = 0,
        .config = {
            .offset       = 0.0f,
            .limit_min    = -3.14159f,
            .limit_max    = 3.14159f,
            .direction    = 1.0f,
            .continuous   = 1,    // Yaw 连续旋转
            .calib_enable = 0,
            .calib_state  = 0,
        },
    },
    // --- Pitch ---
    .pitch = {
        .encoder_angle    = 0.0f,
        .real_angle       = 0.0f,
        .normalized_angle = 0.0f,
        .target_angle     = 0.0f,
        .velocity         = 0.0f,
        .torque           = 0.0f,
        .temperature      = 0.0f,
        .online           = 0,
        .config = {
            .offset       = 0.8825f,       // 实测标定
            .limit_min    = -0.8228f,      // 枪口最低(实测)
            .limit_max    = 0.6481f,       // 枪口最高(实测)
            .direction    = -1.0f,
            .continuous   = 0,
            .calib_enable = 0,
            .calib_state  = 0,
        },
    },
    // --- Fold ---
    .fold = {
        .encoder_angle    = 0.0f,
        .real_angle       = 0.0f,
        .normalized_angle = 0.0f,
        .target_angle     = 0.0f,
        .velocity         = 0.0f,
        .torque           = 0.0f,
        .temperature      = 0.0f,
        .online           = 0,
        .config = {
            .offset       = 0.0f,
            .limit_min    = -1.5708f,   // -90°
            .limit_max    = 1.5708f,    // +90°
            .direction    = -1.0f,      // 抬起时编码器减小，取反
            .continuous   = 0,    // Fold 有限位
            .calib_enable = 0,
            .calib_state  = 0,
        },
    },
};

// ========================================================================
// 三关节 Controller 数据全局实例
// ========================================================================
// 初始化策略：
//   - PID 参数(kp/ki/kd) 全 0：Stage03 调参起点，避免上电瞬间输出力矩
//   - torque_limit 按电机型号填入安全值：DM4310=10Nm, DM4340=28Nm
//   - break_i / limit_i 给保守初值
//   - enabled: 仅 Pitch 使能(Stage03 验证对象)，Yaw/Fold 待 Stage04
//   - target_angle=0：上电后由 Controller 自动初始化为当前角度
//
// 注意：
//   - kp=0 时 PID 输出恒为 0，电机不动作 → 安全启动
//   - Watch 中先调 kp → 给 target → 观察响应
Controller_Data_t Controller_Data = {
    // --- Yaw 控制器(DM4310 #1) ---
    .yaw = {
        .target_angle    = 0.0f,
        .kp              = 0.0f,
        .ki              = 0.0f,
        .kd              = 0.0f,
        .torque_limit    = 10.0f,    // DM4310 TMAX=10 N·m
        .break_i         = 0.1f,     // 误差<0.1rad 才积分
        .limit_i         = 2.0f,     // 积分限幅 2 N·m
        .enabled         = 0,        // Stage04 启用
        .feedback_angle  = 0.0f,
        .error           = 0.0f,
        .torque_output   = 0.0f,
        .limit_min       = -3.14159f,
        .limit_max       =  3.14159f,
    },
    // --- Pitch 控制器(DM4310 #2) ← Stage03 验证对象 ---
    .pitch = {
        .target_angle    = 0.0f,
        .kp              = 0.0f,    // 建议调参起点: kp=5.0
        .ki              = 0.0f,
        .kd              = 0.0f,
        .torque_limit    = 10.0f,    // DM4310 TMAX=10 N·m
        .break_i         = 0.1f,     // 误差<0.1rad 才积分
        .limit_i         = 2.0f,     // 积分限幅 2 N·m
        .enabled         = 1,        // Stage03 启用 Pitch
        .feedback_angle  = 0.0f,
        .error           = 0.0f,
        .torque_output   = 0.0f,
        .limit_min       = -0.8228f, // 枪口最低(实测)
        .limit_max       =  0.6481f, // 枪口最高(实测)
    },
    // --- Fold 控制器(DM4340 #1) ---
    .fold = {
        .target_angle    = 0.0f,
        .kp              = 0.0f,
        .ki              = 0.0f,
        .kd              = 0.0f,
        .torque_limit    = 28.0f,    // DM4340 TMAX=28 N·m
        .break_i         = 0.1f,
        .limit_i         = 5.0f,
        .enabled         = 0,        // Stage04 启用
        .feedback_angle  = 0.0f,
        .error           = 0.0f,
        .torque_output   = 0.0f,
        .limit_min       = -1.5708f, // -90°
        .limit_max       =  1.5708f, // +90°
    },
};
