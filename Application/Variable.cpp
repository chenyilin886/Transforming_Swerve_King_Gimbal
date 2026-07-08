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
#include "Vofa.hpp"  // VOFA+ 发送接口

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
            .limit_min    = -0.7927f,      // 枪口最低(实测)
            .limit_max    =  0.6481f,       // 枪口最高(实测)
            .direction    = -1.0f,
            .continuous   = 0,
            .calib_enable = 0,
            .calib_state  = 0,
        },
    },
    // --- Fold ---
    //   Stage04 实测标定(2026-07-06):
    //     - 零位: encoder_angle = -0.369840622 → real_angle = 0
    //     - 最大上抬角度: real_angle = 0.901044846 rad
    //     - 最小角度: real_angle = 0 (机械限位)
    //   限位范围: [0.0, 0.901044846] rad
    //
    //   ⚠️ direction 验证方法（关键）:
    //     当前 direction = -1.0f (基于"抬起编码器减小"的标定)
    //     验证步骤:
    //       ① 上电后 Watch 观察 Joint_Data.fold.real_angle 当前值
    //       ② 手动抬起 Fold (机械上"展开/上抬"方向)
    //       ③ 观察 real_angle 变化:
    //         - ✓ 正确: real_angle 增大 (朝 +limit_max 方向)
    //         - ✗ 反向: real_angle 减小 → 改 config.direction = 1.0f
    //
    //   offset 计算公式:
    //     real_angle = (encoder_angle - offset) * direction
    //     零位要求: real_angle = 0 → offset = encoder_angle
    //     实测 encoder_angle = -0.369840622 → offset = -0.369840622
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
            .offset       = -0.369840622f,   // ← 实测零位编码器位置
            .limit_min    = 0.0f,            // ← 机械最小角度 0 rad
            .limit_max    = 0.901044846f,    // ← 实测最大上抬角度
            .direction    = -1.0f,           // 待验证: 抬起时 real_angle 应增大
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
//   - PID 参数(kp/ki/kd, vel_kp/vel_ki/vel_kd) 全 0：Stage03 调参起点，避免上电瞬间输出力矩
//   - torque_limit 按电机型号填入安全值：DM4310=10Nm, DM4340=28Nm
//   - break_i / limit_i 给保守初值
//   - cascade_mode:
//       Yaw / Fold = 0  (单级位置式, Stage04 启用)
//       Pitch       = 1  (串级 角度环+速度环均位置式, Stage03 验证对象)
//   - vel_limit / break_i_vel / limit_i_vel 仅 Pitch 串级生效
//   - enabled: 仅 Pitch 使能(Stage03 验证对象)，Yaw/Fold 待 Stage04
//   - target_angle=0：上电后由 Controller 自动初始化为当前角度
//
// 注意：
//   - kp=0 时角度环输出恒为 0 → vel_target=0 → 速度环 error=0 → torque=0
//   - 即使 cascade_mode=1，上电也是安全的（不会输出冲击力矩）
//   - Watch 中先调 vel_kp(内环) → 再调 kp(外环) → 给 target → 观察响应
Controller_Data_t Controller_Data = {
    // --- Yaw 控制器(DM4310 #1, 单级位置式) ---
    .yaw = {
        .target_angle    = 0.0f,
        .kp              = 0.0f,
        .ki              = 0.0f,
        .kd              = 0.0f,
        .torque_limit    = 10.0f,    // 输出端力矩限幅(N·m) → 电机端 1 N·m (DM4310 TMAX=10)
        .break_i         = 0.1f,     // 误差<0.1rad 才积分
        .limit_i         = 2.0f,     // 积分限幅 2 N·m
        .cascade_mode    = 0,        // 单级位置式(Stage04 启用)
        .vel_kp          = 0.0f,
        .vel_ki          = 0.0f,
        .vel_kd          = 0.0f,
        .vel_limit       = 0.0f,
        .break_i_vel     = 0.0f,
        .limit_i_vel     = 0.0f,
        .enabled         = 0,        // Stage04 启用
        .gravity_k       = 0.0f,    // Yaw 不需要重力补偿(水平旋转)
        .gravity_enable  = 0,        // Yaw 不启用
        .feedback_angle  = 0.0f,
        .error           = 0.0f,
        .vel_target      = 0.0f,
        .vel_feedback    = 0.0f,
        .vel_error       = 0.0f,
        .torque_output   = 0.0f,
        .gravity_torque  = 0.0f,
        .limit_min       = -3.14159f,
        .limit_max       =  3.14159f,
    },
    // --- Pitch 控制器(DM4310 #2) ← Stage03 串级验证对象 ---
    //   外环: 角度环(位置式) → 输出速度目标 rad/s
    //   内环: 速度环(位置式) → 输出力矩 N·m
    //   调参顺序建议:
    //     ① vel_kp 内环 P(从 0.05 起调, 观察速度环跟随)
    //     ② vel_kd 内环 D(抑制速度环震荡, 0.001 起调)
    //     ③ kp    外环 P(从 5.0 起调, 观察角度跟随)
    //     ④ kd    外环 D(0.1 起调, 抑制角度超调)
    //     ⑤ ki/vel_ki 最后加, 消除稳态误差
    .pitch = {
        .target_angle    = 0.0f,
        .kp              = 0.0f,    // 角度环 P, 建议起点 5.0
        .ki              = 0.0f,    // 角度环 I
        .kd              = 0.0f,    // 角度环 D, 建议起点 0.1
        .torque_limit    = 10.0f,    // 输出端力矩限幅(N·m) → 电机端 1 N·m (DM4310 TMAX=10)
        .break_i         = 0.1f,     // 角度误差<0.1rad 才积分
        .limit_i         = 2.0f,     // 角度环 I 项 ≤ 2 N·m
        .cascade_mode    = 1,        // ← 启用串级模式(角度环+速度环均位置式)
        .vel_kp          = 0.0f,    // 速度环 P, 建议起点 0.05
        .vel_ki          = 0.0f,    // 速度环 I
        .vel_kd          = 0.0f,    // 速度环 D, 建议起点 0.001
        .vel_limit       = 10.0f,    // 速度目标限幅 10 rad/s (DM4310 VMAX=30, 保守)
        .break_i_vel     = 1.0f,     // 速度误差<1rad/s 才积分
        .limit_i_vel     = 2.0f,     // 速度环 I 项 ≤ 2 N·m
        .enabled         = 1,        // Stage03 启用 Pitch
        .gravity_k       = 0.0f,    // Pitch 暂不需要重力补偿
        .gravity_enable  = 0,        // Pitch 暂不启用
        .feedback_angle  = 0.0f,
        .error           = 0.0f,
        .vel_target      = 0.0f,
        .vel_feedback    = 0.0f,
        .vel_error       = 0.0f,
        .torque_output   = 0.0f,
        .gravity_torque  = 0.0f,
        .limit_min       = -0.7927f, // 枪口最低(实测)
        .limit_max       =  0.6481f, // 枪口最高(实测)
    },
    // --- Fold 控制器(DM4340 #1) ← Stage04 串级验证对象 ---
    //   DM4340: TMAX=28 N·m, VMAX=10 rad/s, 减速比 40:1
    //   Fold 是形态控制关节，力矩远大于 Pitch，调参起点必须更保守
    //
    //   外环: 角度环(位置式) → 输出速度目标 rad/s
    //   内环: 速度环(位置式) → 输出力矩 N·m
    //
    //   调参顺序建议(DM4340 力矩大，务必从小值起调):
    //     ① vel_kp 内环 P(从 0.02 起调, DM4340 力矩大，比 Pitch 更小)
    //     ② vel_kd 内环 D(0.0005 起调)
    //     ③ kp    外环 P(从 3.0 起调, Fold 转动惯量大)
    //     ④ kd    外环 D(0.2 起调, 抑制角度超调)
    //     ⑤ ki/vel_ki 最后加, 消除稳态误差
    //
    //   ⚠️ direction 验证(关键):
    //     Fold 当前 direction = -1.0f（Joint.hpp 中标定: 抬起编码器减小）
    //     验证方法：手动抬起 Fold → Watch 观察 Joint_Data.fold.real_angle 应增大
    //     若方向相反 → 改 Joint_Data.fold.config.direction = 1.0f
    //     （Controller 层 torque*direction 会自动跟随，无需改其他文件）
    .fold = {
        .target_angle    = 0.0f,
        .kp              = 0.0f,    // 角度环 P, 建议起点 3.0
        .ki              = 0.0f,    // 角度环 I
        .kd              = 0.0f,    // 角度环 D, 建议起点 0.2
        .torque_limit    = 2.0f,     // 输出端力矩限幅(N·m), 修复后实测值
                                      //   ← DmMotor.hpp 已修: DM4340 torque_is_output_side_=true
                                      //     固件 GR=40 已换算为输出端, 不再除 GR
                                      //   电机 TMAX=28 N·m(输出端峰值), 2 N·m 是测试安全值
                                      //   标定完成后再视情况放宽(建议≤20 N·m, 留余量)
        .break_i         = 0.1f,     // 角度误差<0.1rad 才积分
        .limit_i         = 5.0f,     // 角度环 I 项 ≤ 5 N·m
        .cascade_mode    = 1,        // ← 启用串级模式(角度环+速度环均位置式)
        .vel_kp          = 0.0f,    // 速度环 P, 建议起点 0.02
        .vel_ki          = 0.0f,    // 速度环 I
        .vel_kd          = 0.0f,    // 速度环 D, 建议起点 0.0005
        .vel_limit       = 5.0f,     // 速度目标限幅 5 rad/s (DM4340 VMAX=10, 保守取半)
        .break_i_vel     = 1.0f,     // 速度误差<1rad/s 才积分
        .limit_i_vel     = 5.0f,     // 速度环 I 项 ≤ 5 N·m
        .enabled         = 1,        // ← Stage04 启用 Fold

        // --- 重力补偿(Watch 在线标定) ---
        //   公式: gravity_torque = gravity_k * cos(feedback_angle)
        //   标定方法: 关补偿测多点保持力矩 → 拟合 K = torque / cos(angle)
        //   注意: gravity_enable=1 且 cascade_mode=1 时才生效
        .gravity_k       = 0.0f,    // 重力补偿系数(N·m), 标定后填入
        .gravity_enable  = 0,        // ← 默认关闭, 标定完成后再开启

        .feedback_angle  = 0.0f,
        .error           = 0.0f,
        .vel_target      = 0.0f,
        .vel_feedback    = 0.0f,
        .vel_error       = 0.0f,
        .torque_output   = 0.0f,
        .gravity_torque  = 0.0f,    // 重力补偿输出(N·m), Watch 观察用
        .limit_min       = 0.0f,            // ← 实测机械最小角度 0 rad
        .limit_max       = 0.901044846f,    // ← 实测最大上抬角度
    },
};

// ========================================================================
// Stage05 变形规划器配置 / 状态全局实例
// ========================================================================
// 初始化策略：
//   - 四个关键角度使用实测标定值（机械已标定，不需要现场改）
//   - arrive_eps = 0.02 rad (≈1.1°)，到位判定阈值
//   - arrive_timeout_ms = 3000 ms，单步超时保护
//   - cmd = 0 (NONE)，上电 IDLE，必须用户主动下达命令才会动作
//
// Watch 操作流程：
//   ① 展开: Watch 中改 Transform_Config.cmd = 1 → 状态机自动执行
//   ② 收起: Watch 中改 Transform_Config.cmd = 2
//   ③ 紧急中止: Watch 中改 Transform_Config.cmd = 3 → ABORT + Hold 当前位置
//   ④ 从 ABORT 恢复: Watch 中改 Transform_Config.cmd = 4 → 回到 IDLE
//
// @note cmd 是单次触发型，Planner 消费后自动清零
Transform_Config_t Transform_Config = {
    .pitch_expand      = 0.126987755f,    // 展开后 Pitch 水平位(实测)
    .pitch_contract    = -0.792750061f,   // 收起 Pitch 角度(实测)
    .fold_expand       = 0.901044846f,    // 展开 Fold 最大上抬(实测)
    .fold_contract     = 0.0f,            // 收起 Fold 最小角度(机械限位)
    .arrive_eps        = 0.02f,           // 到位阈值 0.02rad ≈ 1.1°
    .arrive_timeout_ms = 3000,            // 单步超时 3000ms
    .cmd               = 0,               // NONE（上电待命）
};

// 运行时状态实例（Planner 每周期更新）
Transform_Status_t Transform_Status = {
    .state            = 0,        // IDLE
    .step             = 0,
    .pitch_target_now = 0.0f,
    .fold_target_now  = 0.0f,
    .pitch_err        = 0.0f,
    .fold_err         = 0.0f,
    .step_elapsed_ms  = 0,
    .pitch_online     = 0,
    .fold_online      = 0,
    .last_error       = 0,         // NONE
};

// ========================================================================
// VOFA+ 调试通道发送函数
// ========================================================================
/**
 * @brief VOFA+ 6 通道发送函数
 *
 * 数据来源：Controller_Data.pitch（用户可在此修改通道配置）
 *
 * 当前通道分配（Stage03 Pitch 串级 PID 调参观测）：
 *   CH0: pitch.target_angle    目标角度（rad）       外环输入
 *   CH1: pitch.feedback_angle  反馈角度（rad）       外环反馈
 *   CH2: pitch.error           角度环误差（rad）     外环误差
 *   CH3: pitch.torque_output   输出力矩（N·m）       内环输出
 *   CH4: pitch.vel_target      速度环目标（rad/s）   外环输出=内环输入
 *   CH5: pitch.vel_feedback    速度环反馈（rad/s）   内环反馈
 *
 * 修改通道配置示例：
 *   - 观察 Yaw：改用 Controller_Data.yaw.target_angle 等
 *   - 观察 Joint：改用 Joint_Data.pitch.real_angle 等
 *   - 观察电机原始数据：改用 Motor 层接口（需 extern 声明）
 *
 * @note 调用频率：由 GimbalInit.cpp 降频控制（500Hz）
 */
void VofaSendDebugChannels(void)
{
    // === Stage05 变形规划器 6 通道（动作序列观测）===
    //   用于观察展开/收起过程中 Pitch/Fold 的 target/feedback 跟随
    //   调试方法：
    //     ① Watch 中改 Transform_Config.cmd = 1 (EXPAND) → 观察波形
    //     ② 观察 pitch_target_now 和 pitch_feedback 是否先后动作
    //     ③ 观察 step_elapsed_ms 是否符合预期
    //     ④ 异常时观察 state 是否进入 ABORT(7)
    //
    //   通道分配：
    //     CH0: Transform_Status.state          状态机(0..7)
    //     CH1: Controller_Data.pitch.target_angle  Pitch 目标(rad)
    //     CH2: Controller_Data.pitch.feedback_angle Pitch 反馈(rad)
    //     CH3: Controller_Data.fold.target_angle  Fold 目标(rad)
    //     CH4: Controller_Data.fold.feedback_angle Fold 反馈(rad)
    //     CH5: Transform_Status.step_elapsed_ms / 1000.0f  步骤耗时(s)
    APP::Vofa.Send6Floats(
        (float)Transform_Status.state,                        // CH0: 状态机
        Controller_Data.pitch.target_angle,                   // CH1: Pitch 目标
        Controller_Data.pitch.feedback_angle,                 // CH2: Pitch 反馈
        Controller_Data.fold.target_angle,                    // CH3: Fold 目标
        Controller_Data.fold.feedback_angle,                  // CH4: Fold 反馈
        (float)Transform_Status.step_elapsed_ms / 1000.0f     // CH5: 步骤耗时(s)
    );

    // === Fold 重力补偿 6 通道（Stage04 标定观测，已注释）===
    //   标定 gravity_k 时切回此配置
    /*
    APP::Vofa.Send6Floats(
        Controller_Data.fold.target_angle,     // CH0: 目标角度
        Controller_Data.fold.feedback_angle,   // CH1: 反馈角度
        Controller_Data.fold.torque_output,    // CH2: 总输出力矩
        Controller_Data.fold.gravity_torque,   // CH3: 重力补偿力矩（关键）
        Controller_Data.fold.vel_target,       // CH4: 速度环目标
        Controller_Data.fold.vel_feedback      // CH5: 速度环反馈
    );
    */

    // === Pitch 串级 PID 6 通道（Stage03 配置，已注释）===
    /*
    APP::Vofa.Send6Floats(
        Controller_Data.pitch.target_angle,    // CH0: 目标角度
        Controller_Data.pitch.feedback_angle,  // CH1: 反馈角度
        Controller_Data.pitch.error,           // CH2: 角度环误差
        Controller_Data.pitch.torque_output,   // CH3: 输出力矩
        Controller_Data.pitch.vel_target,      // CH4: 速度环目标
        Controller_Data.pitch.vel_feedback     // CH5: 速度环反馈
    );
    */

    // === 其他观测示例 ===
    /*
    // 示例：观察 Yaw 关节
    APP::Vofa.Send6Floats(
        Controller_Data.yaw.target_angle,      // CH0: Yaw 目标
        Controller_Data.yaw.feedback_angle,    // CH1: Yaw 反馈
        Controller_Data.yaw.error,             // CH2: Yaw 误差
        Controller_Data.yaw.torque_output,     // CH3: Yaw 输出
        0.0f,                                  // CH4: 预留
        0.0f                                   // CH5: 预留
    );
    */

    /*
    // 示例：观察 Joint 层原始角度
    APP::Vofa.Send6Floats(
        Joint_Data.fold.real_angle,            // CH0: Joint 真实角度
        Joint_Data.fold.encoder_angle,         // CH1: 编码器原始值
        Joint_Data.fold.velocity,              // CH2: 关节速度
        Joint_Data.fold.torque,                // CH3: 关节力矩
        (float)Joint_Data.fold.online,         // CH4: 在线状态
        0.0f                                   // CH5: 预留
    );
    */
}
