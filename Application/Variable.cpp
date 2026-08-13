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
//   Yaw  : offset=0.1, continuous=1, 无限位, direction=1
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
            .offset       = 0.1f,      // 临时偏移 0.1，用于测试
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
    //   实测标定(2026-07-18):
    //     - 零位: encoder_angle = 1.71339798 → real_angle = 0
    //     - 最大上抬: encoder_angle = 0.865377426 → real_angle = 0.848020554
    //     - 最小角度: real_angle = 0 (机械限位)
    //   限位范围: [0.0, 0.848020554] rad
    //
    //   direction = -1.0f (已验证: 抬起时编码器减小、real_angle 增大)
    //
    //   offset 计算公式:
    //     real_angle = (encoder_angle - offset) * direction
    //     零位要求: real_angle = 0 → offset = encoder_angle
    //     实测 encoder_angle = 1.71339798 → offset = 1.71339798
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
            .offset       = 1.71339798f,   // ← 实测零位编码器位置
            .limit_min    = 0.0f,            // ← 机械最小角度 0 rad
            .limit_max    = 0.848020554f,    // ← 实测最大上抬角度
            .direction    = -1.0f,           // 已验证: 抬起编码器减小、real增大
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
//       Yaw  = 1  (串级 角度环+速度环均位置式, 连续旋转, Stage04 验证对象)
//       Pitch = 1  (串级 角度环+速度环均位置式, Stage03 验证对象)
//       Fold = 1  (串级 角度环+速度环均位置式, Stage04 验证对象)
//   - vel_limit / break_i_vel / limit_i_vel 仅 cascade_mode=1 时生效
//   - enabled: Pitch/Fold 使能, Yaw 默认失能(Watch 在线使能)
//   - target_angle=0：上电后由 Controller 自动初始化为当前角度
//
// 注意：
//   - kp=0 时角度环输出恒为 0 → vel_target=0 → 速度环 error=0 → torque=0
//   - 即使 cascade_mode=1，上电也是安全的（不会输出冲击力矩）
//   - Watch 中先调 vel_kp(内环) → 再调 kp(外环) → 给 target → 观察响应
Controller_Data_t Controller_Data = {
    // --- Yaw 控制器(DM4310 #1, 串级 角度环+速度环均位置式) ---
    //   Stage04 验证对象：检查串级 PID + 连续旋转角度处理
    //   外环: 角度环(位置式) → 输出速度目标 rad/s
    //   内环: 速度环(位置式) → 输出力矩 N·m
    //   调参顺序建议:
    //     ① vel_kp 内环 P(从 0.05 起调, 观察速度环跟随)
    //     ② vel_kd 内环 D(抑制速度环震荡, 0.001 起调)
    //     ③ kp    外环 P(从 5.0 起调, 观察角度跟随)
    //     ④ kd    外环 D(0.1 起调, 抑制角度超调)
    //     ⑤ ki/vel_ki 最后加, 消除稳态误差
    .yaw = {
        .target_angle    = 0.0f,
        .kp              = 15.0f,    // 角度环 P, 15
        .ki              = 0.0f,    // 角度环 I
        .kd              = 0.0f,    // 角度环 D, 建议起点 0.1
        .torque_limit    = 30.0f,    // 输出端力矩限幅(N·m) → 电机端 1 N·m (DM4310 TMAX=10)
        .break_i         = 0.1f,     // 角度误差<0.1rad 才积分
        .limit_i         = 2.0f,     // 角度环 I 项 ≤ 2 N·m
        .cascade_mode    = 1,        // ← 启用串级模式(角度环+速度环均位置式)
        .vel_kp          = 5.0f,    // 速度环 P, 13
        .vel_ki          = 0.001f,    // 速度环 I
        .vel_kd          = 0.8f,    // 速度环 D, 建议起点 0.001
        .vel_limit       = 50.0f,    // 速度目标限幅 10 rad/s (DM4310 VMAX=30, 保守)
        .break_i_vel     = 1.0f,     // 速度误差<1rad/s 才积分
        .limit_i_vel     = 2.0f,     // 速度环 I 项 ≤ 2 N·m
        .enabled         = 1,        // 默认使能, 与 Pitch/Fold 一致
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
        .kp              = 16.0f,    // 角度环 P, 15
        .ki              = 0.0f,    // 角度环 I
        .kd              = 0.0f,    // 角度环 D, 建议起点 0.1
        .torque_limit    = 22.0f,    // 输出端力矩限幅(N·m) → 电机端 1 N·m (DM4310 TMAX=10)
        .break_i         = 0.1f,     // 角度误差<0.1rad 才积分
        .limit_i         = 2.0f,     // 角度环 I 项 ≤ 2 N·m
        .cascade_mode    = 1,        // ← 启用串级模式(角度环+速度环均位置式)
        .vel_kp          = 12.0f,    // 速度环 P, 12
        .vel_ki          = 0.1f,    // 速度环 I
        .vel_kd          = 0.0f,    // 速度环 D, 建议起点 0.001
        .vel_limit       = 30.0f,    // 速度目标限幅 10 rad/s (DM4310 VMAX=30, 保守)
        .break_i_vel     = 1.0f,     // 速度误差<1rad/s 才积分
        .limit_i_vel     = 2.0f,     // 速度环 I 项 ≤ 2 N·m
        .enabled         = 1,        // Stage03 启用 Pitch
        // --- 重力补偿(加摩擦轮后必需) ---
        //   物理模型: T_gravity = m·g·r·cos(θ)
        //     θ = 枪口绝对俯仰角 = IMU pitch(IMU 闭环下 feedback_angle)
        //     枪口水平(θ=0):    cos(0)=1,   重力矩最大 → 需要补偿最大
        //     枪口垂直(θ=π/2):  cos(π/2)=0, 重力矩为 0  → 补偿为 0
        //   标定方法: Watch 观察 gravity_torque, 从 2.0 起调:
        //     ① 抬不起来 → 增大 gravity_k
        //     ② 自发上抬 → 减小 gravity_k
        //     ③ 最终标定值 = 枪口水平时的静态保持力矩
        //   注意: gravity_enable=1 且 cascade_mode=1 时生效
        //         IMU 离线切编码器时, 若编码器零位≠枪口水平, 补偿会不准
        //         (当前编码器零位已标定为枪口水平, 可放心使用)
        .gravity_k       = 6.0f,    // 重力补偿系数(N·m), 保守初值, Watch 在线标定
        .gravity_enable  = 1,        // ← 启用 Pitch 重力补偿(加摩擦轮后必需)
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
        .kp              = 15.0f,    // 角度环 P, 建议起点 3.0
        .ki              = 0.0f,    // 角度环 I0
        .kd              = 0.0f,    // 角度环 D, 建议起点 0.2
        .torque_limit    = 5.0f,     // 输出端力矩限幅(N·m), 修复后实测值
                                      //   ← DmMotor.hpp 已修: DM4340 torque_is_output_side_=true
                                      //     固件 GR=40 已换算为输出端, 不再除 GR
                                      //   电机 TMAX=28 N·m(输出端峰值), 2 N·m 是测试安全值
                                      //   标定完成后再视情况放宽(建议≤20 N·m, 留余量)
        .break_i         = 0.1f,     // 角度误差<0.1rad 才积分
        .limit_i         = 5.0f,     // 角度环 I 项 ≤ 5 N·m
        .cascade_mode    = 1,        // ← 启用串级模式(角度环+速度环均位置式)
        .vel_kp          = 13.0f,    // 速度环 P, 建议起点 0.02
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
        .gravity_k       = 1.8f,    // 重力补偿系数(N·m), 标定后填入
        .gravity_enable  = 1,        // ← 启用 Fold 重力补偿

        .feedback_angle  = 0.0f,
        .error           = 0.0f,
        .vel_target      = 0.0f,
        .vel_feedback    = 0.0f,
        .vel_error       = 0.0f,
        .torque_output   = 0.0f,
        .gravity_torque  = 0.0f,    // 重力补偿输出(N·m), Watch 观察用
        .limit_min       = 0.0f,            // ← 实测机械最小角度 0 rad
        .limit_max       = 0.848020554f,    // ← 实测最大上抬角度
    },
};

// ========================================================================
// 底盘跟随模式专用数据（速度环单环控制）
// ========================================================================
// 设计原因：
//   底盘跟随模式(S2==MIDDLE)时，Yaw轴从串级PID切换到速度环单环控制，
//   松手后电机自由停止，不抵抗底盘跟随的超调反向修正，根治反转现象。
//   独立结构体管理跟随模式参数，职责清晰，易于扩展。
//
// 初始化策略：
//   - target_velocity = 0（初始静止）
//   - imu_velocity = 0（IMU未启动）
//   - follow_vel_kp/ki/kd = 0（用户在线调参）
//   - control_mode = 0（默认串级PID模式）
//
// 调参流程：
//   ① kp从5.0起调，观察速度响应
//   ② ki从0.1起调，消除稳态误差
//   ③ kd从0.05起调，抑制超调
Controller_Data_t Vision_Controller_Data = Controller_Data;

FollowMode_Data_t FollowMode_Data = {
    .target_velocity = 0.0f,    // 速度环目标(rad/s)
    .imu_velocity    = 0.0f,    // IMU角速度反馈(rad/s)
    .follow_vel_kp   = 5.0f,    // 速度环 kp（用户在线调整）
    .follow_vel_ki   = 0.15f,    // 速度环 ki（用户在线调整）
    .follow_vel_kd   = 0.0f,    // 速度环 kd（用户在线调整）
    .control_mode    = 0,       // 默认串级PID模式
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
    .fold_expand       = 0.848020554f,    // 展开 Fold 最大上抬(实测)
    .fold_contract     = 0.0f,            // 收起 Fold 最小角度(机械限位)
    .arrive_eps        = 0.1f,           // 到位阈值 0.1rad ≈ 1.1°
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
// DR16 遥控器数据全局实例（Stage04）
// ========================================================================
// 初始化策略：
//   - 所有值初始化为 0（遥控器离线状态）
//   - offline = 1（初始判定为离线）
//   - 首次收到遥控器数据后，DR16.Parse() 更新内部状态
//   - GimbalUpdate 中调用 DR16.IsOffline() → 更新离线状态
//   - 同步到 DR16_Data → Watch 观察全部遥控器状态
//
// 数据流：
//   UART3 DMA接收 → HAL_UARTEx_RxEventCallback → DR16.Parse()
//     → 更新内部状态(joystick_right_/switch_left_/keyboard_等)
//     → GimbalUpdate 每 1ms 调用 DR16.IsOffline() 检测离线
//     → Variable.cpp 同步到 DR16_Data → Watch 实时观察
DR16_Data_t DR16_Data = {
    // --- 摇杆状态（ch0~ch3） ---
    .ch0 = 0.0f,               // 右摇杆X轴
    .ch1 = 0.0f,               // 右摇杆Y轴
    .ch2 = 0.0f,               // 左摇杆X轴
    .ch3 = 0.0f,               // 左摇杆Y轴

    // --- 开关状态（S1/S2） ---
    .s1 = 0,                   // S1: 左开关 UNKNOWN
    .s2 = 0,                   // S2: 右开关 UNKNOWN

    // --- 鼠标状态 ---
    .mouse_vel_x      = 0.0f,
    .mouse_vel_y      = 0.0f,
    .mouse_left       = 0,
    .mouse_right      = 0,

    // --- 键盘按键状态 ---
    .key_w            = 0,
    .key_s            = 0,
    .key_a            = 0,
    .key_d            = 0,
    .key_shift        = 0,
    .key_ctrl         = 0,
    .key_q            = 0,
    .key_e            = 0,
    .key_r            = 0,
    .key_f            = 0,
    .key_g            = 0,
    .key_z            = 0,
    .key_x            = 0,
    .key_c            = 0,
    .key_v            = 0,
    .key_b            = 0,

    // --- 拨轮状态 ---
    .wheel            = 0.0f,

    // --- 离线状态 ---
    .online           = 0,     // 初始离线（0=离线, 1=在线）
};

// ========================================================================
// IMU 数据全局实例（Stage03 接入传感器）
// ========================================================================
// 初始化策略：
//   - 所有姿态数据初始化为 0（IMU 离线状态）
//   - online = 0（初始判定为离线）
//   - 首次收到 IMU 数据后，imu.Parse() 更新内部 euler/gyr/acc/quat/addYaw
//   - GimbalUpdate 每 1ms 调用 imu.IsOffline() 检测离线
//   - 同步到 IMU_Data → Watch 观察全部姿态数据
//
// 单位说明：
//   保留 IMU 原始单位(deg / deg/s / g)，便于 Watch 直接核对传感器输出。
//   后续接入控制环时再换算为 rad / rad/s。
//
// 数据流：
//   UART1 DMA接收 → HAL_UARTEx_RxEventCallback → imu.Parse()
//     → memcpy 解析(82字节) → 更新内部状态
//     → GimbalUpdate 同步到 IMU_Data → Watch 实时观察
IMU_Data_t IMU_Data = {
    // --- 欧拉角（deg） ---
    .yaw             = 0.0f,
    .pitch           = 0.0f,
    .roll            = 0.0f,

    // --- 角速度（deg/s） ---
    .gyro_x          = 0.0f,
    .gyro_y          = 0.0f,
    .gyro_z          = 0.0f,

    // --- 加速度（g） ---
    .acc_x           = 0.0f,
    .acc_y           = 0.0f,
    .acc_z           = 0.0f,

    // --- 四元数 ---
    .quat_w          = 0.0f,
    .quat_x          = 0.0f,
    .quat_y          = 0.0f,
    .quat_z          = 0.0f,

    // --- 累计角度（deg） ---
    .add_yaw         = 0.0f,

    // --- 状态 ---
    .temperature     = 0,       // 温度(°C)
    .online          = 0,       // 初始离线（0=离线, 1=在线）
};

// ========================================================================
// 遥控器状态机全局实例（Stage05+: 急停 + 展开/收起控制）
// ========================================================================
// 初始化策略：
//   estop_active=0    上电默认非急停
//   s1/s2/last_s1=0   UNKNOWN 状态（首周期强制走边沿检测 else 分支）
//   saved_*_en=0      急停前 enabled 备份（首周期无意义）
//   planner_cmd_sent=0 本周期未发命令
Remote_State_t Remote_State = {
    .estop_active    = 0,
    .remote_offline  = 0,    // 上电默认遥控器离线（等待首包）
    .s1              = 0,
    .s2              = 0,
    .last_s1         = 0,
    .planner_cmd_sent = 0,
    .saved_yaw_en    = 0,
    .saved_pitch_en  = 0,
    .saved_fold_en   = 0,
};

// ========================================================================
// LK4005 电机数据全局实例（数据接收验证用）
// ========================================================================
// 初始化策略：
//   - 所有字段初始化为 0
//   - online=0 初始离线，首次收到反馈帧后由 GimbalUpdate 同步置 1
//   - status1_valid=0 表示尚未调用 ReadStatus1() 或未收到响应
//
// 数据流：
//   CAN1 中断 → LK4005::Parse → Configure → unit_data_[0]
//     → GimbalUpdate 同步到 LK4005_Data → Watch 观察
LK4005_Data_t LK4005_Data = {
    .angle           = 0.0f,
    .velocity        = 0.0f,
    .torque          = 0.0f,
    .temperature     = 0.0f,
    .raw_angle       = 0,
    .raw_velocity    = 0,
    .raw_current     = 0,
    .raw_cmd         = 0,
    .voltage         = 0,
    .error_state     = 0,
    .status1_valid   = 0,
    .online          = 0,
};

// ========================================================================
// LK4005 拨盘双环控制配置 / 状态全局实例
// ========================================================================
// 设计来源：
//   参考 H_SG_Gimbal 参考工程 ShootTask.cpp 拨盘控制：
//     - 单击单发：wheel 上沿触发，每次目标角度 -= angle_per_shot_deg
//     - 长按连发：wheel 持续超过 long_press_ms，按 fire_hz 持续累加角度
//     - 双环 PID：位置环(位置式) → 速度环(位置式) → LK raw 命令
//     - 卡弹检测：力矩饱和 + 位置误差大持续 → 反转解卡
//
// 初始化策略：
//   - feature_enable=1, enabled=1：保留拨盘控制入口；PID 全 0 时仍不会输出力矩
//   - 位置环/速度环 PID 参数全 0：满足"4005 初始 PID 参数为 0，Debug 手动修改"
//   - 拨盘槽位数 9：参考工程默认值（单发角度 = 360/9 = 40°）
//   - 拨轮阈值 0.5：比旧版 0.8 更低，操作手体验更柔和
//   - 长按 1000ms：与参考工程一致
//   - raw_output_limit=500：初调阶段保守限幅
//   - jam_detect_enable=0：初调时关闭卡弹检测，速度环稳定后再开
//
// Watch 调试建议：
//   ① Watch 添加 Dial_Config / Dial_Status / LK4005_Data
//   ② 确认 LK4005_Data.online=1
//   ③ Dial_Config.enabled=1
//   ④ 先调速度环(内环): vel_kp=50, vel_kd=1.0, vel_ki=0
//      - 用 raw_override_enable=1 + raw_override_cmd=300 强制转一下，确认方向
//      - 然后关闭 override, 给 vel_kp=50, vel_kd=1.0
//      - 拨轮上抬 → 观察 feedback_velocity 是否跟随 target_velocity
//   ⑤ 再调位置环(外环): pos_kp=8.0, pos_kd=0.3, pos_ki=0
//      - 拨轮短暂上抬一次 → 观察 target_angle 应减 40°(≈0.698 rad)
//      - feedback_angle 应跟随到目标位置
//   ⑥ 速度环稳定后再开卡弹检测: jam_detect_enable=1
Dial_Config_t Dial_Config = {
    // === 功能开关 ===
    .feature_enable        = 1,       // 1=允许拨盘控制流程
    .enabled               = 1,       // 1=允许 wheel 触发双环控制
    .clear_pid             = 0,       // 0=不清PID; Watch 置1后清PID并自动回写0

    // === 拨盘几何 ===
    .slots_per_rotation    = 9.0f,    // 9 槽拨盘(参考工程默认)
    .angle_per_shot_deg    = 40.0f,   // 单发角度 = 360/9 = 40°

    // === 拨轮触发 ===
    .wheel_start_threshold = 0.5f,    // wheel > 0.5 触发(比旧版0.8更柔和)
    .long_press_ms         = 800,     // 持续 800ms 切换为连发
    .auto_fire_hz          = 20.0f,    // 连发基础频率 8 Hz (仅 wheel_to_hz<=0 时使用)
    .wheel_to_hz           = 0.0f,   // wheel 满幅映射到 0 Hz

    // === 位置环(外环) PID ===
    .pos_kp                = 40.0f,    // P, 调好速度环后改 8.0
    .pos_ki                = 0.0f,    // I, 保持 0
    .pos_kd                = 0.0f,    // D, 调好速度环后改 0.3
    .pos_break_i           = 0.1f,    // 位置误差<0.1rad 才积分
    .pos_limit_i           = 5.0f,    // 位置环 I 项限幅 5 rad/s
    .pos_vel_limit         = 50.0f,   // 速度目标上限 20 rad/s

    // === 速度环(内环) PID ===
    .vel_kp                = 13.0f,    // P, 起步 50
    .vel_ki                = 0.0f,    // I, 保持 0
    .vel_kd                = 0.0f,    // D, 起步 1.0
    .vel_break_i           = 5.0f,    // 速度误差<5rad/s 才积分
    .vel_limit_i           = 80.0f,   // 速度环 I 项限幅 80 raw
    .raw_output_limit      = 2048.0f,  // 最终 raw 命令限幅 500, 调好后可放宽到 1500

    // === raw_override 模式(调试用) ===
    .raw_override_enable   = 0,       // 默认关闭, 调试时置 1 + 设置 raw_override_cmd
    .raw_override_cmd      = 0,       // 原始命令覆盖值, 范围 [-2048, 2048]

    // === 卡弹检测 ===
    .jam_detect_enable     = 0,       // 初调时关闭, 速度环稳定后再开
    .jam_torque_threshold  = 0.9f,    // 力矩饱和阈值 90% 限幅
    .jam_err_threshold     = 0.5f,    // 位置误差阈值 0.5 rad (≈28.6°)
    .jam_duration_ms       = 300,     // 持续 300ms 触发解卡
    .jam_reverse_ms        = 200,     // 反转 200ms 解卡
    .jam_reverse_torque    = 250,     // 反转力矩 +250 raw (正负由用户标定)
};

Dial_Status_t Dial_Status = {
    .state              = 0,         // DISABLE
    .wheel_input        = 0.0f,
    .target_angle       = 0.0f,
    .feedback_angle     = 0.0f,
    .target_velocity    = 0.0f,
    .feedback_velocity  = 0.0f,
    .error              = 0.0f,
    .vel_target         = 0.0f,
    .vel_error          = 0.0f,
    .pid_p              = 0.0f,
    .pid_i              = 0.0f,
    .pid_d              = 0.0f,
    .torque_cmd         = 0,
    .control_source     = 0,         // 0=零力矩/未控制
    .jam_detected       = 0,
    .shot_count         = 0,
    .online             = 0,         // 0=LK4005 初始按离线处理
};

// ========================================================================
// 发射机构整体配置/状态（Class_ShootFSM 使用）
// ========================================================================
// 设计说明：
//   - Shoot_Config 是 ShootFSM 顶层状态机的配置
//   - Dial_Config 仍保留为独立全局变量（拨盘详细参数）
//   - ShootFSM 通过 enabled 字段联动 Dial_Config
//
// Watch 调参顺序：
//   ① Shoot_Config.feature_enable = 1, shoot_enabled = 1
//   ② Dial_Config 调拨盘 PID（见 Dial_Config 注释）
//   ③ 观察 Shoot_Status.state 应在 safety_ok 时为 3 (AUTO)
//   ④ 拨轮触发 → Dial_Status.target_angle 增大 → 电机正转 40°
// ========================================================================
Shoot_Config_t Shoot_Config = {
    .feature_enable      = 1,        // 1=允许发射机构控制流程
    .shoot_enabled       = 1,        // 1=safety_ok 时自动进 AUTO
    .target_state        = 3,        // 预留: 3=AUTO（当前未生效, 简化策略直接进 AUTO）
    .friction_target_rpm = 6000.0f,     // 摩擦轮目标转速, 默认 0 RPM, Watch 在线调
    .friction_kp         = 5.0f,     // 摩擦轮速度环 P, Watch 调参
    .friction_ki         = 0.0f,     // 摩擦轮速度环 I, Watch 调参
    .friction_kd         = 0.0f,     // 摩擦轮速度环 D, Watch 调参
    .friction_break_i    = 500.0f,   // 积分隔离阈值(RPM): 误差<500 才积分
    .friction_limit_i    = 3000.0f,  // 积分输出限幅: i_accum*ki 上限 3000
};

Shoot_Status_t Shoot_Status = {
    .state              = 0,         // DISABLE
    .safety_ok          = 0,
    .friction_enable    = 0,
    .friction_online_l  = 0,
    .friction_online_r  = 0,
    .friction_vel_l     = 0.0f,
    .friction_vel_r     = 0.0f,
};

// ========================================================================
// 摩擦轮电机反馈（GM3508 × 2, Watch 观察）
// ========================================================================
// 数据来源：BSP::MOTOR::DJI::motor_3508 全局指针
//   在 GimbalUpdate 每周期从 Motor 层读取并填入。
// Watch 添加 Friction_Data → 展开 left/right → 观察 velocity_rpm 和 online
Friction_Data_t Friction_Data = {
    .left  = { .angle_rad = 0.0f, .velocity_rpm = 0.0f, .velocity_radps = 0.0f,
                .torque_nm = 0.0f, .temperature = 0.0f, .online = 0 },
    .right = { .angle_rad = 0.0f, .velocity_rpm = 0.0f, .velocity_radps = 0.0f,
                .torque_nm = 0.0f, .temperature = 0.0f, .online = 0 },
};

// ========================================================================
// 板间通信数据全局实例（Stage03）
// ========================================================================
// 数据来源：
//   - 发送数据：BoardComm::Gimbal_to_Chassis::Update() 填充
//   - 接收数据：BoardComm::Gimbal_to_Chassis::HandleCANMessage() 更新
// Watch 添加 BoardComm_Data → 观察：
//   - LX/LY：遥控器左摇杆映射值（0-220，中值110）
//   - Yaw_encoder_angle_err：云台-底盘角度误差
//   - launch_speed：发射速度
//   - booster_now_heat：当前热量
BoardComm_Data_t BoardComm_Data = {
    .LX                  = 110,     // 中值
    .LY                  = 110,     // 中值
    .Rotating_vel        = 110,     // 小陀螺速度中值
    .wheel               = 0,       // 拨轮中值
    .Yaw_encoder_angle_err = 0.0f,
    .chassis_mode        = 0,
    .booster_heat_cd     = 0,
    .booster_heat_max    = 0,
    .booster_now_heat    = 0,
    .launch_speed        = 0.0f,
    .rx_frame1_ready     = 0,
    .last_rx_time        = 0,
};

// ========================================================================
// 视觉通信数据全局实例（Stage07: 视觉通信 — RCIA协议）
// ========================================================================
// 数据来源：
//   - VisionComm::Manager::Parse() → 解析视觉上位机发来的 RCIA 帧（RX, 19字节）
//   - 视觉发来的角度：int32 / 100.0 → 度数（angle_scale = 100.0）
// Watch 添加 VisionComm_Data → 观察：
//   - pitch_angle/yaw_angle：视觉目标角度(deg)
//   - vision_ready：视觉就绪(0/1)
//   - fire：开火信号(0/1)
//   - online：视觉在线状态(0=离线, 1=在线)
VisionComm_Data_t VisionComm_Data = {
    .pitch_angle  = 0.0f,
    .yaw_angle    = 0.0f,
    .pitch_raw    = 0,
    .yaw_raw      = 0,
    .vision_ready = 0,
    .fire         = 0,
    .timestamp    = 0,
    .aim_x        = 0,
    .aim_y        = 0,
    .online       = 0,       // 初始离线（等待视觉上位机连接）
    .rx_head0     = 0,
    .rx_head1     = 0,
    .rx_tail      = 0,
    .yaw_offset_deg = 87.0f,  // [标定] Yaw 视觉零点偏移(deg)，Watch 可调
};

// ========================================================================
// VOFA+ 调试通道发送函数
// ========================================================================
/**
 * @brief VOFA+ 6 通道发送函数
 *
 * 数据来源：Controller_Data.yaw（Yaw 串级 PID 调参观测）
 *
 * 当前通道分配（Stage04 Yaw 串级 PID 调参观测）：
 *   CH0: yaw.target_angle    目标角度（rad）       外环输入
 *   CH1: yaw.feedback_angle  反馈角度（rad）       外环反馈（normalized_angle）
 *   CH2: yaw.error           角度环误差（rad）     外环误差（最短路径）
 *   CH3: yaw.torque_output   输出力矩（N·m）       内环输出
 *   CH4: yaw.vel_target      速度环目标（rad/s）   外环输出=内环输入
 *   CH5: yaw.vel_feedback    速度环反馈（rad/s）   内环反馈
 *
 * 修改通道配置示例：
 *   - 观察 Pitch：改用 Controller_Data.pitch.target_angle 等
 *   - 观察 Joint：改用 Joint_Data.yaw.real_angle 等
 *   - 观察电机原始数据：改用 Motor 层接口（需 extern 声明）
 *
 * @note 调用频率：由 GimbalInit.cpp 降频控制（500Hz）
 */
void VofaSendDebugChannels(void)
{
    // === Yaw + Pitch target/feedback channels ===
    //   CH0: Controller_Data.yaw.target_angle       Yaw target(rad)
    //   CH1: Controller_Data.yaw.feedback_angle     Yaw feedback(rad)
    //   CH2: Controller_Data.pitch.target_angle     Pitch target(rad)
    //   CH3: Controller_Data.pitch.feedback_angle   Pitch feedback(rad)
    //   CH4: Reserved
    //   CH5: Reserved
    APP::Vofa.Send6Floats(
        Controller_Data.yaw.target_angle,       // CH0: Yaw target
        Controller_Data.yaw.feedback_angle,     // CH1: Yaw feedback
        Controller_Data.pitch.target_angle,     // CH2: Pitch target
        Controller_Data.pitch.feedback_angle,   // CH3: Pitch feedback
        0.0f,                                   // CH4: reserved
        0.0f                                    // CH5: reserved
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

// ========================================================================
// 底盘模式状态机调试数据（Stage06: 板间通信 - 模式状态机）
// ========================================================================

/**
 * @brief 底盘模式状态机调试全局变量
 *
 * 初始化说明：
 *   - current_state: 初始化为MANUAL（手动模式）
 *   - stable_state: 初始化为MANUAL（手动模式）
 *   - pending_state: 初始化为MANUAL（手动模式）
 *   - 其他字段：初始化为0
 *
 * Watch 观察建议：
 *   添加 ChassisModeDebug 到 Watch 窗口，展开观察：
 *   - current_state: 当前模式（0-3）
 *   - state_change_count: 状态切换次数
 *   - remote_online: 遥控器在线状态
 *
 * @note 由 ChassisModeManager.cpp 周期更新（1kHz）
 */
ChassisModeDebug_t ChassisModeDebug = {
    // ===== 核心状态 =====
    .current_state = 0,          // MANUAL（手动模式）
    .stable_state = 0,           // MANUAL
    .pending_state = 0,          // MANUAL

    // ===== 滤波过程 =====
    .stable_count = 0,           // 初始计数为0

    // ===== 统计信息 =====
    .state_change_count = 0,     // 初始切换次数为0
    .filter_reject_count = 0,    // 初始滤波拒绝次数为0

    // ===== 错误状态 =====
    .emergency_stop_trigger = 0, // NONE
    .remote_online = 0,          // 初始离线（DR16未初始化）

    // ===== 时间戳 =====
    .last_update_time = 0        // 初始时间戳为0
};
