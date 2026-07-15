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
 *   state            : 当前状态(0=IDLE, 1=EXPAND_SIMULTANEOUS, 2=EXPANDED,
 *                      3=CONTRACT_SIMULTANEOUS, 4=CONTRACTED, 5=ABORT)
 *   step             : 当前步骤序号(0=待机/终态, 1=执行中)
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
// 全局变量 extern 声明(定义在 Variable.cpp)
// ========================================================================
extern Joint_Data_t       Joint_Data;        // 关节状态(Stage01-02)
extern Controller_Data_t  Controller_Data;   // 控制器状态(Stage03)
extern Transform_Config_t  Transform_Config;  // 变形规划器配置(Stage05)
extern Transform_Status_t  Transform_Status;  // 变形规划器状态(Stage05)
extern DR16_Data_t        DR16_Data;         // 遥控器状态(Stage04)
extern IMU_Data_t         IMU_Data;          // IMU 姿态状态(Stage03 接入传感器)
extern Remote_State_t     Remote_State;      // 遥控器状态机(急停+展开/收起)
extern LK4005_Data_t      LK4005_Data;       // LK4005 电机反馈状态

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
