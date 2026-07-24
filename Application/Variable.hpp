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
    uint8_t  feedback_source;  // 反馈源标识: 0=编码器, 1=IMU(Yaw 专用), Watch 观察用

} Controller_Data_Unit_t;

// ========================================================================
// 底盘跟随模式专用数据（速度环单环控制）
// ========================================================================
/**
 * @brief 底盘跟随模式专用数据结构
 *
 * 设计原因：
 *   底盘跟随模式(S2==MIDDLE)时，Yaw轴从串级PID切换到速度环单环控制，
 *   松手后电机自由停止，不抵抗底盘跟随的超调反向修正，根治反转现象。
 *   独立结构体管理跟随模式参数，职责清晰，易于扩展。
 *
 * 数据流：
 *   DR16.ch2 → 死区过滤 → 速度目标(target_velocity)
 *     ↓
 *   IMU.getGyroZ() → IMU角速度反馈(imu_velocity)
 *     ↓
 *   速度环PID(follow_vel_kp/ki/kd) → 力矩输出
 *     ↓
 *   DM4310.ctrl_Mit()
 *
 * Watch 调参步骤：
 *   ① follow_vel_kp 从 5.0 起调，观察速度响应
 *   ② follow_vel_ki 从 0.1 起调，消除稳态误差
 *   ③ follow_vel_kd 从 0.05 起调，抑制超调
 *
 * 模式切换：
 *   S2=MIDDLE → control_mode=1（速度环单环）
 *   S2=其他   → control_mode=0（串级PID）
 *   切换时清空PID状态，平滑过渡
 */
typedef struct
{
    float    target_velocity;  // 速度环目标(rad/s)，摇杆直接映射
    float    imu_velocity;     // IMU角速度反馈(rad/s)，来自陀螺仪
    float    follow_vel_kp;    // 跟随模式速度环 kp（用户在线调整）
    float    follow_vel_ki;    // 跟随模式速度环 ki（用户在线调整）
    float    follow_vel_kd;    // 跟随模式速度环 kd（用户在线调整）
    uint8_t  control_mode;     // 控制模式: 0=串级PID, 1=速度环单环（跟随模式）
} FollowMode_Data_t;

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
    float    fold_expand;        // 展开 Fold 角度(rad),      实测 0.848020554
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
 *   state            : 当前状态(0=IDLE, 1=EXPAND_SIMULTANEOUS, 2=EXPANDED,
 *                      3=CONTRACT_SIMULTANEOUS, 4=CONTRACTED, 5=ABORT)
 *   step             : 当前步骤序号(0=待机/终态, 1=执行中)
 *   yaw_target_now   : 当前下发 yaw target(rad)
 *                      [TRANSITION: 锁定到变形开始时的IMU角度; 终态/IDLE/ABORT: Controller_Data 中的值]
 *   pitch_target_now : 当前下发的 pitch target(rad)
 *                      [TRANSITION: Planner 写入值; 终态/IDLE/ABORT: Controller_Data 中的值]
 *   fold_target_now  : 当前下发的 fold target(rad)
 *   yaw_err          : 实时 yaw 误差(rad)   = yaw_target_now - yaw_fb
 *   pitch_err        : 实时 pitch 误差(rad) = pitch_target_now - pitch_fb
 *   fold_err         : 实时 fold 误差(rad)  = fold_target_now  - fold_fb
 *   step_elapsed_ms  : 当前步已耗时(ms), 仅 TRANSITION 状态累加
 *   yaw_online       : yaw 在线状态(1=在线)
 *   pitch_online     : pitch 在线状态(1=在线)
 *   fold_online      : fold 在线状态(1=在线)
 *   last_error       : ABORT 原因(0=正常, 1=TIMEOUT, 2=MOTOR_OFFLINE, 3=ABORT_CMD)
 */
typedef struct
{
    uint8_t  state;               // 当前状态(0..7)
    uint8_t  step;                // 当前步骤序号(0/1/2)
    float    yaw_target_now;      // 当前 yaw target(rad)
    float    pitch_target_now;    // 当前 pitch target(rad)
    float    fold_target_now;     // 当前 fold target(rad)
    float    yaw_err;             // yaw 误差(rad)
    float    pitch_err;           // pitch 误差(rad)
    float    fold_err;            // fold 误差(rad)
    uint16_t step_elapsed_ms;     // 当前步已耗时(ms)
    uint8_t  yaw_online;          // yaw 在线状态
    uint8_t  pitch_online;        // pitch 在线状态
    uint8_t  fold_online;         // fold 在线状态
    uint8_t  last_error;          // ABORT 原因(0/1/2/3)
} Transform_Status_t;

// ========================================================================
// 遥控器状态机数据结构（Stage05+: 急停 + 展开/收起控制）
// ========================================================================
/**
 * @brief 遥控器状态机数据（Watch 可观察）
 *
 * 职责：
 *   1. 急停状态管理（S1==DOWN && S2==DOWN）
 *   2. S1 边沿检测 → 生成 Planner 命令(EXPAND/CONTRACT)
 *   3. 急停时保存/恢复各关节 enabled 状态
 *
 * 数据流：
 *   DR16.S1/S2 → Remote_State(estop_active/last_s1)
 *                ├─ estop_active=1 → Controller_Data.{yaw,pitch,fold}.enabled=0
 *                └─ S1 边沿变化    → Transform_Config.cmd = EXPAND/CONTRACT
 *
 * Watch 观察点：
 *   estop_active       : 急停激活标志(1=急停中, 0=正常)
 *   remote_offline     : 遥控器离线标志(1=离线, 0=在线) 【建议 Watch】
 *   s1/s2              : 当前 S1/S2 原始值(1=UP, 2=DOWN, 3=MIDDLE)
 *   last_s1            : 上一周期 S1（边沿检测用）
 *   planner_cmd_sent   : 本周期发送的 Planner 命令
 *                        (0=NONE, 1=EXPAND, 2=CONTRACT, 3=ABORT)
 *   saved_yaw_en       : 急停前 yaw.enabled（退出急停时恢复）
 *   saved_pitch_en     : 急停前 pitch.enabled
 *   saved_fold_en      : 急停前 fold.enabled
 *
 * @note 急停触发条件：
 *       ① S1==DOWN && S2==DOWN
 *       ② 遥控器离线（remote_offline==1）
 *       急停退出条件：S1/S2 不都是 DOWN 且遥控器在线
 *       急停时摇杆积分失效（Step 2.6 跳过）
 */
typedef struct
{
    uint8_t estop_active;      // 急停激活标志(0=正常, 1=急停中) 【建议 Watch】
    uint8_t remote_offline;    // 遥控器离线标志(0=在线, 1=离线) 【建议 Watch】
    uint8_t s1;                // 当前 S1 原始值(1=UP, 2=DOWN, 3=MIDDLE) 【建议 Watch】
    uint8_t s2;                // 当前 S2 原始值(1=UP, 2=DOWN, 3=MIDDLE) 【建议 Watch】
    uint8_t last_s1;           // 上一周期 S1（边沿检测用）
    uint8_t planner_cmd_sent;  // 本周期发送的 Planner 命令(0/1/2/3) 【建议 Watch】
    uint8_t saved_yaw_en;      // 急停前 yaw.enabled（退出时恢复）
    uint8_t saved_pitch_en;    // 急停前 pitch.enabled
    uint8_t saved_fold_en;     // 急停前 fold.enabled
} Remote_State_t;

// ========================================================================
// DR16 遥控器数据结构（Stage04）
// ========================================================================
/**
 * @brief DR16 遥控器数据（Watch 可观察）
 *
 * Watch 中展开 DR16_Data 即可观察遥控器全部状态：
 *   ch0/ch1/ch2/ch3     : 摇杆值（ch0=右X, ch1=右Y, ch2=左X, ch3=左Y）
 *   s1/s2               : 拨杆开关状态（s1=左开关, s2=右开关）
 *   mouse_vel_x/y       : 鼠标速度，范围 [-1.0, 1.0]
 *   mouse_left/right    : 鼠标按键状态(0=未按下, 1=按下)
 *   keyboard            : 键盘按键状态（位域结构体）
 *   wheel               : 拨轮值，范围 [-1.0, 1.0]
 *   online              : 遥控器在线状态(0=离线, 1=在线)
 *
 * 数据流：
 *   UART3 DMA接收 → HAL_UARTEx_RxEventCallback → DR16.Parse()
 *     → 更新内部状态 → Variable.cpp 回写 DR16_Data → Watch 观察
 *
 * 命名规范（符合RoboMaster习惯）：
 *   - ch0: 右摇杆X轴（joystick_channel0）
 *   - ch1: 右摇杆Y轴（joystick_channel1）
 *   - ch2: 左摇杆X轴（joystick_channel2）
 *   - ch3: 左摇杆Y轴（joystick_channel3）
 *   - s1:  左开关（switch_left，遥控器左侧）
 *   - s2:  右开关（switch_right，遥控器右侧）
 *   - online: 遥控器在线状态（0=离线，1=在线）
 *
 * @note 所有值在遥控器离线时自动归零
 */
typedef struct
{
    // --- 摇杆状态（ch0~ch3） ---
    float ch0;                 // 右摇杆X轴，[-1.0, 1.0]
    float ch1;                 // 右摇杆Y轴，[-1.0, 1.0]
    float ch2;                 // 左摇杆X轴，[-1.0, 1.0]
    float ch3;                 // 左摇杆Y轴，[-1.0, 1.0]

    // --- 开关状态（S1/S2） ---
    uint8_t s1;                // S1: 左开关(0=UNKNOWN, 1=UP, 2=DOWN, 3=MIDDLE)
    uint8_t s2;                // S2: 右开关(0=UNKNOWN, 1=UP, 2=DOWN, 3=MIDDLE)

    // --- 鼠标状态 ---
    float mouse_vel_x;         // 鼠标X轴速度，[-1.0, 1.0]
    float mouse_vel_y;         // 鼠标Y轴速度，[-1.0, 1.0]
    uint8_t mouse_left;        // 鼠标左键(0=未按下, 1=按下)
    uint8_t mouse_right;       // 鼠标右键(0=未按下, 1=按下)

    // --- 键盘按键状态（位域） ---
    uint8_t key_w;             // W键
    uint8_t key_s;             // S键
    uint8_t key_a;             // A键
    uint8_t key_d;             // D键
    uint8_t key_shift;         // Shift键
    uint8_t key_ctrl;          // Ctrl键
    uint8_t key_q;             // Q键
    uint8_t key_e;             // E键
    uint8_t key_r;             // R键
    uint8_t key_f;             // F键
    uint8_t key_g;             // G键
    uint8_t key_z;             // Z键
    uint8_t key_x;             // X键
    uint8_t key_c;             // C键
    uint8_t key_v;             // V键
    uint8_t key_b;             // B键

    // --- 拨轮状态 ---
    float wheel;               // 拨轮值，[-1.0, 1.0]

    // --- 离线状态 ---
    uint8_t online;            // 遥控器在线状态(0=离线, 1=在线)
} DR16_Data_t;

// ========================================================================
// IMU 数据结构（Stage03 接入传感器）
// ========================================================================
/**
 * @brief HI12H3 IMU 数据（Watch 可观察）
 *
 * Watch 中展开 IMU_Data 即可观察 IMU 全部姿态数据：
 *   yaw/pitch/roll     : 欧拉角(单位: deg，IMU 原始输出)
 *   gyro_x/y/z         : 角速度(单位: deg/s)
 *   acc_x/y/z          : 加速度(单位: g)
 *   quat_w/x/y/z       : 四元数
 *   add_yaw            : Yaw 累计角度(单位: deg，跨 ±180° 连续累加)
 *   temperature        : 温度(单位: °C)
 *   online             : 在线状态(0=离线, 1=在线)
 *
 * 数据流：
 *   UART1 DMA接收 → HAL_UARTEx_RxEventCallback → imu.Parse()
 *     → memcpy 解析 → 更新内部 euler/gyr/acc/quat/addYaw
 *     → GimbalUpdate 同步到 IMU_Data → Watch 观察
 *
 * 单位说明：
 *   本阶段(正确解析传感器数据)保留 IMU 原始单位(deg / deg/s / g)，
 *   便于 Watch 直接核对传感器输出是否正确。
 *   后续接入控制环时，在 Controller/上层换算为 rad / rad/s。
 *
 * 轴向映射(与参考工程一致)：
 *   Euler_yaw   → 云台 Yaw
 *   Euler_pitch → 云台 Pitch
 *   Euler_roll  → Roll
 *
 * @note 传感器：HI12H3，输出频率 200Hz，波特率 256000(USART1)
 */
typedef struct
{
    // --- 欧拉角（deg） ---
    float yaw;             // 航向角(deg)，[-180, 180]
    float pitch;           // 俯仰角(deg)，[-180, 180]
    float roll;            // 横滚角(deg)，[-180, 180]

    // --- 角速度（deg/s） ---
    float gyro_x;          // X 轴角速度(deg/s)
    float gyro_y;          // Y 轴角速度(deg/s)
    float gyro_z;          // Z 轴角速度(deg/s)

    // --- 加速度（g） ---
    float acc_x;           // X 轴加速度(g)
    float acc_y;           // Y 轴加速度(g)
    float acc_z;           // Z 轴加速度(g)

    // --- 四元数 ---
    float quat_w;          // 四元数 W
    float quat_x;          // 四元数 X
    float quat_y;          // 四元数 Y
    float quat_z;          // 四元数 Z

    // --- 累计角度（deg） ---
    float add_yaw;         // Yaw 累计角度(deg，跨 ±180° 连续)

    // --- 状态 ---
    int8_t  temperature;   // 温度(°C)
    uint8_t online;        // 在线状态(0=离线, 1=在线)
} IMU_Data_t;

// ========================================================================
// LK4005 电机数据结构（数据接收验证用）
// ========================================================================
/**
 * @brief LK4005 电机数据（Watch 可观察）
 *
 * Watch 中展开 LK4005_Data 即可观察电机反馈状态，验证数据接收：
 *   --- SI 单位（输出端，由 MotorBase.unit_data_ 同步）---
 *   angle        : 输出端角度(rad)，0~2π（编码器单圈）
 *   velocity     : 输出端角速度(rad/s)
 *   torque       : 输出端力矩(N·m)
 *   temperature  : 温度(°C)
 *
 *   --- 原始反馈（LKFeedback 解析结果，便于核对协议解析）---
 *   raw_angle    : 编码器原始值(0~65535)
 *   raw_velocity : 电机端 RPM 原始值(int16)
 *   raw_current  : 反馈电流原始值(int16, ±2048)
 *   raw_cmd      : 命令字节(0xA1=力矩反馈, ...)
 *
 *   --- 状态1缓存（需主动调用 ReadStatus1() 后才有数据）---
 *   voltage      : 电压(mV)
 *   error_state  : 错误状态位(0=无错误)
 *   status1_valid: 状态1缓存是否有效
 *
 *   --- 在线状态 ---
 *   online       : 在线状态(0=离线, 1=在线)
 *
 * 数据流：
 *   CAN1 接收中断 → LK4005::Parse → Configure → unit_data_[0]
 *     → GimbalUpdate 同步到 LK4005_Data → Watch 观察
 *
 * @note 当前任务：仅验证数据接收。
 *       LK4005 需周期性发送 ctrl_Torque(1, 0) 维持反馈上报，
 *       GimbalUpdate 中每周期调用一次。
 */
typedef struct
{
    // --- SI 单位（输出端，与 MotorBase.unit_data_ 一致）---
    float    angle;            // 输出端角度(rad)，0~2π
    float    velocity;         // 输出端角速度(rad/s)
    float    torque;           // 输出端力矩(N·m)
    float    temperature;      // 温度(°C)

    // --- 原始反馈（调试用）---
    uint16_t raw_angle;        // 编码器原始值(0~65535)
    int16_t  raw_velocity;     // 电机端 RPM 原始值
    int16_t  raw_current;      // 反馈电流原始值(±2048)
    uint8_t  raw_cmd;          // 命令字节(0xA1=力矩反馈, ...)

    // --- 状态1缓存（0x9A 响应）---
    uint16_t voltage;          // 电压(mV)
    uint8_t  error_state;      // 错误状态位(0=无错误)
    uint8_t  status1_valid;    // 状态1缓存是否有效(0/1)

    // --- 在线状态 ---
    uint8_t  online;           // 在线状态(0=离线, 1=在线)
} LK4005_Data_t;

// ========================================================================
// LK4005 拨盘双环控制配置 / 状态（参考工程方式：位置环+速度环）
// ========================================================================
//
// 设计来源：
//   参考 H_SG_Gimbal 参考工程 ShootTask.cpp 拨盘控制：
//     - 单击单发：wheel 上沿触发，每次目标角度 -= angle_per_shot_deg
//     - 长按连发：wheel 持续超过 long_press_ms，按 fire_hz 持续累加角度
//     - 双环 PID：位置环(位置式) → 速度环(位置式) → LK raw 命令
//     - 卡弹检测：力矩饱和 + 位置误差大持续 → 反转解卡
//
// 与旧 Dial4005_Config_t（已移除）的区别：
//   - 旧版：只有速度环 PID，wheel 直接映射目标速度
//   - 新版：位置环+速度环双环，wheel 通过状态机生成目标"角度"
//   - 新版支持单击单发+长按连发，符合操作手习惯
//   - 新增卡弹检测，避免卡弹烧电机
//
// 拨盘几何：
//   - 槽位数 slots_per_rotation: 默认 9（参考工程，单发角度 360/9 = 40°）
//   - 单发角度 angle_per_shot_deg: 默认 40°
//   - 拨盘方向：target_angle -= 实现正向供弹（与参考工程一致）

/**
 * @brief 拨盘双环控制配置（Watch 可调）
 *
 * Watch 调试建议：
 *   ① 先调速度环(内环): vel_kp=50, vel_kd=1.0, vel_ki=0
 *   ② 再调位置环(外环): pos_kp=8.0, pos_kd=0.3, pos_ki=0
 *   ③ 拨轮短暂上抬一次 → 观察 target_angle 减 40°、反馈角度跟随
 *   ④ 卡弹检测初调时 jam_detect_enable=0，速度环稳定后再开
 *
 * @note feature_enable=0 / enabled=0 / 急停 → 自动发送零力矩保反馈
 */
typedef struct
{
    // === 功能开关 ===
    uint8_t  feature_enable;        // 总开关: 0=完全旁路, 1=允许拨盘控制
    uint8_t  enabled;               // 控制使能: 0=清PID+零力矩, 1=允许拨盘控制
    uint8_t  clear_pid;             // 单次清PID: Watch置1后清空PID, 本周期自动回写0

    // === 拨盘几何 ===
    float    slots_per_rotation;    // 拨盘槽位数, 默认 9 (单发角度 = 360/9 = 40°)
    float    angle_per_shot_deg;    // 单发角度(度), 默认 40°, 与 slots_per_rotation 对应

    // === 拨轮触发 ===
    float    wheel_start_threshold; // 拨轮启动阈值, 默认 0.5 (wheel > 该值才触发)
    uint32_t long_press_ms;         // 长按判定时间(ms), 默认 1000 (持续超过则切连发)
    float    auto_fire_hz;          // 连发基础频率(Hz), 默认 8 (仅 wheel_to_hz<=0 时使用)
    float    wheel_to_hz;           // wheel满幅映射频率(Hz), 默认 15 (>0时按wheel值线性映射)

    // === 位置环(外环) PID ===
    //   输入: 误差(rad), 输出: 速度目标(rad/s)
    float    pos_kp;                // 位置环 P, 建议起点 8.0
    float    pos_ki;                // 位置环 I, 建议保持 0 (拨盘是供弹, 无需消除稳态误差)
    float    pos_kd;                // 位置环 D, 建议起点 0.3
    float    pos_break_i;           // 位置环积分隔离阈值(rad), |误差|<此值才积分
    float    pos_limit_i;           // 位置环积分输出限幅(rad/s)
    float    pos_vel_limit;         // 位置环输出限幅(rad/s) = 速度目标上限, 默认 20

    // === 速度环(内环) PID ===
    //   输入: 误差(rad/s), 输出: LK raw 命令 (-2048~2048)
    float    vel_kp;                // 速度环 P, 建议起点 50
    float    vel_ki;                // 速度环 I, 建议保持 0
    float    vel_kd;                // 速度环 D, 建议起点 1.0
    float    vel_break_i;           // 速度环积分隔离阈值(rad/s)
    float    vel_limit_i;           // 速度环积分输出限幅(raw)
    float    raw_output_limit;      // 速度环总输出限幅(raw), 默认 500, 调好后可放宽到 1500

    // === raw_override 模式(调试用) ===
    //   绕过 PID 直接发送原始命令，用于验证 CAN 输出 / 电机方向
    uint8_t  raw_override_enable;   // 1=使用 raw_override_cmd, 0=走正常双环流程
    int16_t  raw_override_cmd;      // 原始命令值, 范围 [-2048, 2048]

    // === 卡弹检测 ===
    //   触发条件(同时满足且持续 jam_duration_ms):
    //     ① |torque_cmd| > jam_torque_threshold × raw_output_limit (力矩饱和)
    //     ② |pos_error| > jam_err_threshold (rad) (位置误差大)
    //   解卡动作: 发送 jam_reverse_torque, 持续 jam_reverse_ms
    uint8_t  jam_detect_enable;     // 卡弹检测开关, 默认 0 (调好速度环后再开)
    float    jam_torque_threshold;  // 力矩饱和阈值(0~1), 默认 0.9 (90% 限幅)
    float    jam_err_threshold;     // 位置误差阈值(rad), 默认 0.5
    uint32_t jam_duration_ms;       // 持续触发时长(ms), 默认 300
    uint32_t jam_reverse_ms;        // 反转解卡时长(ms), 默认 200
    int16_t  jam_reverse_torque;    // 反转力矩(raw), 默认 +250 (正负由用户标定)
} Dial_Config_t;

/**
 * @brief 拨盘双环控制运行状态（Watch 观察）
 *
 * 字段说明：
 *   state            : 当前状态(0=DISABLE, 1=STOP, 2=SINGLE, 3=AUTO)
 *   wheel_input      : 实际拨轮值(超过阈值时为原始值, 否则 0)
 *   target_angle     : 目标累计角度(rad, 多圈)
 *   feedback_angle   : 反馈累计角度(rad, 来自 LK4005.getAddAngleRad)
 *   target_velocity  : 速度环目标(rad/s) = 位置环输出
 *   feedback_velocity: 速度环反馈(rad/s) = LK4005 速度
 *   error            : 位置环误差(rad)
 *   vel_target       : 速度环目标(rad/s) (= target_velocity, 重复字段便于 Watch)
 *   vel_error        : 速度环误差(rad/s)
 *   pid_p/i/d        : 速度环 P/I/D 项(raw)
 *   torque_cmd       : 最终发送的 LK raw 命令
 *   control_source   : 0=零力矩, 1=双环PID, 2=raw_override
 *   jam_detected     : 卡弹检测触发标志(1=正在解卡)
 *   shot_count       : 单发累计计数(Watch 观察发弹数)
 *   online           : LK4005 在线状态
 */
typedef struct
{
    uint8_t  state;                 // 当前状态: 0=DISABLE, 1=STOP, 2=SINGLE, 3=AUTO
    float    wheel_input;           // 实际拨轮值, 未超阈值时为 0
    float    target_angle;          // 目标累计角度(rad, 多圈)
    float    feedback_angle;        // 反馈累计角度(rad, 来自 LK4005.getAddAngleRad)
    float    target_velocity;       // 速度环目标(rad/s)
    float    feedback_velocity;     // 速度环反馈(rad/s)
    float    error;                 // 位置环误差(rad)
    float    vel_target;            // 速度环目标(rad/s) (= target_velocity)
    float    vel_error;             // 速度环误差(rad/s)
    float    pid_p;                 // 速度环 P 项(raw)
    float    pid_i;                 // 速度环 I 项(raw)
    float    pid_d;                 // 速度环 D 项(raw)
    int16_t  torque_cmd;            // 最终发送的 LK raw 命令
    uint8_t  control_source;        // 0=零力矩, 1=双环PID, 2=raw_override
    uint8_t  jam_detected;          // 卡弹检测触发标志
    uint32_t shot_count;            // 单发累计计数
    uint8_t  online;                // LK4005 在线状态
} Dial_Status_t;

// ========================================================================
// 发射机构整体配置（ShootFSM 用，封装 Dial + 摩擦轮预留）
// ========================================================================
/**
 * @brief 发射机构整体配置（Class_ShootFSM 使用，Watch 可调）
 *
 * 设计说明：
 *   - 本结构是 ShootFSM 顶层状态机的配置
 *   - 拨盘详细配置仍用 Dial_Config（保持向下兼容，Watch 配置不丢失）
 *   - 摩擦轮配置预留，加 DJI 3508 后扩展
 *
 * 字段说明：
 *   feature_enable    : 总开关，0=完全旁路发射机构（拨盘+摩擦轮全停）
 *   shoot_enabled     : 发射使能，0=强制 DISABLE，1=允许状态机运行
 *   target_state      : 目标状态（Watch 可写，预留：未来用开关切换 STOP/AUTO）
 *                       当前简化策略：safety_ok 即进 AUTO，此字段暂未生效
 *   friction_target_rpm: 摩擦轮目标转速（RPM，Watch 可调，默认 0）
 *   friction_kp/ki/kd : 摩擦轮速度环 PID 参数（Watch 可调）
 *   friction_break_i  : 摩擦轮速度环积分隔离阈值（RPM，Watch 可调）
 *   friction_limit_i  : 摩擦轮速度环积分输出限幅（Watch 可调）
 *
 * @note Dial_Config 仍作为独立全局变量，ShootFSM 通过 enabled 字段联动
 */
typedef struct
{
    // === 整体功能开关 ===
    uint8_t  feature_enable;         // 总开关: 0=完全旁路, 1=允许发射机构控制
    uint8_t  shoot_enabled;          // 发射使能: 0=强制DISABLE, 1=允许状态机运行
    uint8_t  target_state;           // 目标状态(预留, 当前未生效): 0=DISABLE,1=STOP,2=STANDBY,3=AUTO

    // === 摩擦轮配置（Watch 可调）===
    float    friction_target_rpm;    // 摩擦轮目标转速(RPM), 默认 0, Watch 在线调
    float    friction_kp;            // 摩擦轮速度环 P, Watch 可调
    float    friction_ki;            // 摩擦轮速度环 I, Watch 可调
    float    friction_kd;            // 摩擦轮速度环 D, Watch 可调
    float    friction_break_i;       // 积分隔离阈值(RPM): 误差小于此值才积分, 默认 500
    float    friction_limit_i;       // 积分输出限幅: 限制 i_accum*ki 范围, 默认 3000
} Shoot_Config_t;

/**
 * @brief 发射机构整体状态（Class_ShootFSM 回写，Watch 观察）
 *
 * 字段说明：
 *   state              : 当前发射机构状态(0=DISABLE,1=STOP,3=AUTO)
 *   safety_ok          : 安全条件是否满足(1=可控制, 0=需失能)
 *   friction_enable    : 摩擦轮使能(0=停, 1=转)，由右拨杆 UP 控制
 *   friction_online_l/r: 摩擦轮在线状态(预留)
 *   friction_vel_l/r   : 摩擦轮实际转速(预留)
 *
 * @note Dial_Status 仍作为独立全局变量，由 DialController 直接回写
 */
typedef struct
{
    uint8_t  state;                  // 当前状态: 0=DISABLE,1=STOP,3=AUTO
    uint8_t  safety_ok;              // 安全条件: 1=可控制, 0=需失能
    uint8_t  friction_enable;        // 摩擦轮使能(0=停, 1=转)
    uint8_t  friction_online_l;      // 左摩擦轮在线(预留)
    uint8_t  friction_online_r;      // 右摩擦轮在线(预留)
    float    friction_vel_l;         // 左摩擦轮实际转速(RPM, 预留)
    float    friction_vel_r;         // 右摩擦轮实际转速(RPM, 预留)
} Shoot_Status_t;

// ========================================================================
// 摩擦轮数据（GM3508 × 2，Watch 观察电机原始反馈）
// ========================================================================
/**
 * @brief 单台 GM3508 摩擦轮电机反馈数据（Watch 观察）
 *
 * 数据来源：BSP::MOTOR::DJI::motor_3508 全局指针，
 *          每周期从 MotorBase 公开接口读取并填入。
 *
 * 字段说明：
 *   angle_rad      : 输出端累计角度（rad, 多圈连续）— Watch 建议
 *   velocity_rpm   : 电机端转速（RPM）— 速度环反馈，Watch 建议
 *   velocity_radps : 输出端角速度（rad/s）— 仅供参考
 *   torque_nm      : 输出端力矩（N·m）— 反馈电流换算，Watch 建议
 *   temperature    : 温度（℃）
 *   online         : 在线状态（1=在线, 0=离线）— Watch 建议
 */
typedef struct
{
    float    angle_rad;       // 输出端累计角度（rad, 多圈连续, 摩擦轮一般只看 RPM）
    float    velocity_rpm;    // 电机端转速（RPM, 速度环反馈核心字段）
    float    velocity_radps;  // 输出端角速度（rad/s, = RPM / 减速比 × 2π/60）
    float    torque_nm;       // 输出端力矩（N·m, 反馈电流换算）
    float    temperature;     // 温度（℃, 持续观察防止过热）
    uint8_t  online;          // 在线状态（1=在线, 0=离线, 超时 100ms 判离线）
} FrictionMotor_Data_t;

/**
 * @brief 两台 GM3508 摩擦轮电机数据聚合（Watch 观察）
 *
 * Watch 中添加 Friction_Data 即可展开两台电机全部反馈：
 *   Friction_Data.left.velocity_rpm   ← motor_id=1 当前 RPM
 *   Friction_Data.right.velocity_rpm  ← motor_id=2 当前 RPM
 *   Friction_Data.left.online         ← motor_id=1 在线状态
 *
 * 电机 ID 分配：
 *   left  : motor_id = 1（左摩擦轮）
 *   right : motor_id = 2（右摩擦轮，与左摩擦轮反向旋转）
 *
 * 调试场景：
 *   1. 上电后观察 left.online / right.online 是否都 = 1（确认 CAN2 通信正常）
 *   2. 设置摩擦轮目标 RPM 后观察 velocity_rpm 是否跟随
 *   3. 异常时对比 torque_nm 判断是否堵转，对比 temperature 判断是否过热
 */
typedef struct
{
    FrictionMotor_Data_t left;   // motor_id=1（左摩擦轮）
    FrictionMotor_Data_t right;  // motor_id=2（右摩擦轮）
} Friction_Data_t;

// ========================================================================
// 板间通信数据结构（Stage03：云台-底盘通信）
// ========================================================================
/**
 * @brief 板间通信状态（Watch 可观察）
 *
 * 数据来源：
 *   - tx_direction / tx_chassis_mode：发送数据（Update 填充）
 *   - rx_refree：接收数据（底盘返回的裁判系统数据）
 *
 * Watch 观察项：
 *   - tx_direction.LX/LY：遥控器右摇杆映射值（0-220）
 *   - tx_direction.Yaw_encoder_angle_err：云台-底盘角度误差
 *   - rx_refree.launch_speed：发射速度
 *   - rx_refree.booster_now_heat：当前热量
 */
typedef struct
{
    // --- 发送数据（云台→底盘） ---
    uint8_t LX;                     // 右摇杆X通道（0-220）
    uint8_t LY;                     // 右摇杆Y通道（0-220）
    uint8_t Rotating_vel;           // 小陀螺速度（0-220，中值110）
    int8_t wheel;                   // 拨轮值（-127~127，中值0）
    float Yaw_encoder_angle_err;    // 云台-底盘角度误差(rad)
    uint8_t chassis_mode;           // 底盘模式（位域打包后）

    // --- 接收数据（底盘→云台） ---
    uint16_t booster_heat_cd;       // 电容冷却时间
    uint16_t booster_heat_max;      // 热量上限
    uint16_t booster_now_heat;      // 当前热量
    float launch_speed;             // 发射速度(m/s)

    // --- 通信状态 ---
    uint8_t rx_frame1_ready;        // 帧1 接收就绪标志
    uint32_t last_rx_time;          // 最后接收时间戳(ms)
} BoardComm_Data_t;

// ========================================================================
// 底盘模式状态机调试数据（Stage06: 板间通信 - 模式状态机）
// ========================================================================
/**
 * @brief 底盘模式状态机调试数据（Watch 观察）
 *
 * 职责：
 *   观察底盘模式状态机的运行状态，帮助调试模式切换逻辑。
 *
 * Watch 观察点：
 *   current_state       : 当前模式(0=MANUAL, 1=CHASSIS_FOLLOW, 2=GYROSCOPE, 3=EMERGENCY_STOP)
 *   stable_state        : 稳定状态（滤波后）
 *   pending_state       : 待确认状态（候选）
 *   stable_count        : 稳定计数（连续相同次数，0-10）
 *   state_change_count  : 状态切换次数（累计）
 *   filter_reject_count : 滤波拒绝次数（累计）
 *   emergency_stop_trigger : 急停触发源（0=NONE, 1=REMOTE_SWITCH, 2=REMOTE_OFFLINE, ...）
 *   remote_online       : 遥控器在线状态（0=离线, 1=在线）
 *   last_update_time    : 最后更新时间戳（ms）
 *
 * 使用方式：
 *   在 Keil Watch 中添加 ChassisModeDebug 即可展开全部字段。
 *
 * 数据流：
 *   DR16.S1/S2 → ChassisModeManager::Update()
 *                → 模式判断 + 状态滤波
 *                → 同步到 ChassisModeDebug（Watch 观察）
 *
 * @note 由 ChassisModeManager.cpp 周期更新（1kHz）
 */
typedef struct
{
    // ===== 核心状态 =====
    uint8_t current_state;          // 当前模式（枚举值 0-3）
    uint8_t stable_state;           // 稳定状态（滤波后）

    // ===== 滤波过程 =====
    uint8_t pending_state;          // 待确认状态（候选）
    uint32_t stable_count;          // 稳定计数（连续相同次数，0-10）

    // ===== 统计信息 =====
    uint32_t state_change_count;    // 状态切换次数（累计）
    uint32_t filter_reject_count;   // 滤波拒绝次数（累计）

    // ===== 错误状态 =====
    uint8_t emergency_stop_trigger; // 急停触发源（枚举值 0-4）
    uint8_t remote_online;          // 遥控器在线状态（0/1）

    // ===== 时间戳 =====
    uint32_t last_update_time;      // 最后更新时间戳（ms）
} ChassisModeDebug_t;

// ========================================================================
// 全局变量 extern 声明(定义在 Variable.cpp)
// ========================================================================
extern Joint_Data_t       Joint_Data;        // 关节状态(Stage01-02)
extern Controller_Data_t  Controller_Data;   // 控制器状态(Stage03)
extern FollowMode_Data_t  FollowMode_Data;   // 底盘跟随模式专用数据(速度环单环)
extern Transform_Config_t  Transform_Config;  // 变形规划器配置(Stage05)
extern Transform_Status_t  Transform_Status;  // 变形规划器状态(Stage05)
extern DR16_Data_t        DR16_Data;         // 遥控器状态(Stage04)
extern IMU_Data_t         IMU_Data;          // IMU 姿态状态(Stage03 接入传感器)
extern Remote_State_t     Remote_State;      // 遥控器状态机(急停+展开/收起)
extern LK4005_Data_t      LK4005_Data;       // LK4005 电机反馈状态
extern Dial_Config_t      Dial_Config;       // 拨盘双环控制配置(Watch可调)
extern Dial_Status_t      Dial_Status;       // 拨盘双环控制状态(Watch观察)
extern Shoot_Config_t     Shoot_Config;      // 发射机构整体配置(Watch可调)
extern Shoot_Status_t     Shoot_Status;      // 发射机构整体状态(Watch观察)
extern Friction_Data_t    Friction_Data;     // 摩擦轮电机反馈(Watch观察)
extern BoardComm_Data_t   BoardComm_Data;    // 板间通信状态(Stage03)
extern ChassisModeDebug_t ChassisModeDebug;  // 底盘模式状态机调试数据(Stage06)

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
