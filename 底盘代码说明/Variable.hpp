/**
 * @file Variable.hpp
 * @brief 底盘全局变量声明
 *
 * 集中管理底盘控制所需的全局变量，方便 Keil Watch 观察与在线调参。
 *
 * 调试变量分组：
 * - Kpid_3508_vel：3508 速度环 PID 参数（kp/ki/kd，Watch 可在线修改）
 * - pid_vel_Wheel[4]：4 个轮速度环 PID 实例（含误差、P/I/D 输出）
 * - Kpid_4005_angle：MG4005 角度环 PID 参数（kp/ki/kd，Watch 可在线修改）
 * - pid_angle_Steer[4]：4 个舵向角度环 PID 实例（含误差、P/I/D 输出）
 * - Chassis_Data：底盘目标值与输出值
 * - Steer_Data：舵向目标角度与输出力矩
 * - ChassisState_Data：底盘状态机变量（state/last_state/stop_trigger）【新增】
 *
 * 数据流：
 *   遥控器/上位机 → Chassis_Data.tar_speed[i] → PID → Motor3508.setCAN()
 *   遥控器/上位机 → Steer_Data.tar_angle[i] → PID → Motor4005.ctrl()
 *   DR16 开关 → ChassisState_Update() → ChassisState_Data.state → 控制 PID 是否执行
 */
#pragma once

#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Motor/Lk/Lk_motor.hpp"
#include "../Algorithm/PID.hpp"
#include "../Algorithm/Tools.hpp"

/**
 * @brief 轮向电机数据结构体（3508）
 *
 * 包含轮向电机的目标速度、输出电流等。
 * 在 Watch 中添加 Wheel_Data 可观察轮向控制状态。
 */
typedef struct
{
    float tar_speed[4];      // 4 个轮目标速度（rpm），Watch 可观察
    float final_3508_Out[4]; // 4 个 3508 最终输出电流（原始值 -16384~16384）

    // ===== 运动学相关字段【新增】=====
    float vx, vy, vw;       // 底盘目标速度（m/s, rad/s），运动学输入
                            // Watch 中可观察，遥控器/上位机输入
    float vx_fdb, vy_fdb, vw_fdb; // 底盘实际速度（m/s, rad/s），正运动学输出
                            // Watch 中可观察，用于调试和反馈控制

    uint8_t Wheel_Enable;    // 轮向使能标志，0=禁用（零电流），1=使能（PID 控制）
                             // Watch 中修改此值为 1 开始控制，上电默认 0 防失控
    uint8_t Kinematics_Enable; // 运动学使能标志，1=使用运动学解算，0=直接控制
                             // Watch 中修改此值为 1 开始运动学控制
} Wheel_Data_t;

/**
 * @brief 舵向数据结构体
 *
 * 包含舵向控制的目标角度、当前角度、速度、输出力矩等。
 * 在 Watch 中添加 Steer_Data 可观察舵向控制状态。
 *
 * 串级 PID 数据流：
 *   tar_angle[i] → 角度环PID → tar_speed[i] → 速度环PID → final_4005_Out[i]
 *
 * 安全保护：
 * - Steer_Enable：使能标志，Watch 中设 1 才开始控制，上电默认 0（防疯转）
 * - Steer_InitSync：首次使能时自动将 tar_angle 同步为 cur_angle
 * - 电机离线时自动输出零力矩
 */
typedef struct
{
    float tar_angle[4];     // 4 个舵向目标角度（度），范围 [0, 360)，Watch 可观察
    float cur_angle[4];     // 4 个舵向当前角度（度），范围 [0, 360)

    float tar_speed[4];     // 4 个舵向目标速度（rpm），角度环输出 / 速度环输入
    float cur_speed[4];     // 4 个舵向当前速度（rpm），MG4005 反馈

    float final_4005_Out[4];// 4 个 MG4005 最终输出力矩（原始值 -2048~2048）

    float torque_Nm[4];     // 4 个 MG4005 实际反馈扭矩（Nm），范围约 -0.24~0.24
                            // Watch 中可观察，来自 Motor4005.getTorque()

    float angle_diff[4];    // 4 个舵向角度差（度），范围 [-180, 180]，用于观察最短路径

    uint8_t Steer_Enable;   // 舵向使能标志，0=禁用（零力矩），1=使能（PID 控制）
                            // Watch 中修改此值为 1 开始控制，上电默认 0 防疯转
    uint8_t Steer_InitSync; // 初始角度同步标志，1=已同步，0=未同步
                            // 首次使能时自动置 1 并将 tar_angle 同步为 cur_angle
    uint8_t Steer_Online;   // 舵向电机在线状态，bit0~3 对应电机 1~4，1=在线

    // ===== 零位校准相关字段【新增】=====
    float offset[4];        // 零位偏移（度），encoder_angle - offset = real_angle
                            // Watch 中可观察，校准后记录
    float real_angle[4];    // 真实角度（度），范围 [0, 360)，运动学使用
                            // Watch 中可观察，encoder_angle - offset 计算得到
    float encoder_angle[4]; // 编码器原始角度（度），MG4005 反馈值
                            // Watch 中可观察，用于调试

    uint8_t Calib_Enable;   // 校准使能标志，1=开始校准，0=停止校准
                            // Watch 中修改此值为 1 开始校准流程
    uint8_t Calib_Status;   // 校准状态：0=未校准，1=已校准
                            // Watch 中可观察，判断是否需要重新校准
    uint8_t Calib_Step;     // 校准步骤：0=等待开始，1=记录offset，2=完成
                            // Watch 中可观察，调试校准流程
} Steer_Data_t;

// ===== 3508 速度环 PID 参数（Watch 可在线调参）=====
extern Kpid_t Kpid_3508_vel;
extern PID pid_vel_Wheel[4];

// ===== MG4005 角度环 PID 参数（Watch 可在线调参）=====
extern Kpid_t Kpid_4005_angle;
extern PID pid_angle_Steer[4];

// ===== MG4005 速度环 PID 参数（Watch 可在线调参）=====
extern Kpid_t Kpid_4005_vel;
extern PID pid_vel_Steer[4];

// ===== 轮向电机数据（3508）=====
extern Wheel_Data_t Wheel_Data;

// ===== 舵向数据 =====
extern Steer_Data_t Steer_Data;

// ===== 运动学参数【新增】=====
/**
 * @brief 运动学参数结构体
 *
 * 包含舵轮底盘运动学解算所需的几何参数。
 * 在 Watch 中添加 Kinematics_Params 可观察和修改参数。
 *
 * 参数说明：
 * - R：底盘半径（轮子投影点到中心距离，单位：m）
 * - S：轮子半径（单位：m）
 * - Wheel_Azimuth[4]：轮子安装方位角（弧度），从中心指向轮子的方向
 * - Phase[4]：相位补偿（弧度），舵轮安装角度偏差
 *
 * 典型值（标准 X 型布局）：
 * - Wheel_Azimuth = {45°, 135°, -135°, -45°}（弧度）
 * - Phase = {0, 0, 0, 0}（无相位补偿）
 */
typedef struct
{
    float R;                // 底盘半径（m），轮子投影点到中心距离
                            // Watch 中可修改，根据实际底盘尺寸调整
    float S;                // 轮子半径（m）
                            // Watch 中可修改，根据实际轮子尺寸调整

    float Wheel_Azimuth[4]; // 轮子安装方位角（弧度），从中心指向轮子的方向
                            // Watch 中可修改，根据实际安装位置调整
                            // 典型值：{π/4, 3π/4, -3π/4, -π/4}

    float Phase[4];         // 相位补偿（弧度），舵轮安装角度偏差
                            // Watch 中可修改，用于补偿机械安装误差
                            // 典型值：{0, 0, 0, 0}

    float SpeedGain;        // 速度增益系数
                            // Watch 中可修改，用于调整整体速度响应
                            // 典型值：1.0

    float RotationalGain;   // 旋转增益系数
                            // Watch 中可修改，用于调整旋转响应
                            // 典型值：1.0
} Kinematics_Params_t;

extern Kinematics_Params_t Kinematics_Params;

// ===== 遥控器数据结构【新增】=====
/**
 * @brief DR16 遥控器数据结构体
 *
 * 包含 DR16 遥控器的所有输入数据。
 * 在 Watch 中添加 Remote_Data 可观察遥控器输入状态。
 *
 * 数据流：
 *   DR16 → UART3 DMA → Parse() → Remote_Data → 速度映射 → Wheel_Data.vx/vy/vw
 *
 * 摇杆通道说明：
 * - channel[0]：右摇杆水平（右正左负，范围 -1.0~1.0）
 * - channel[1]：右摇杆垂直（上正下负，范围 -1.0~1.0）
 * - channel[2]：左摇杆水平（右正左负，范围 -1.0~1.0）
 * - channel[3]：左摇杆垂直（上正下负，范围 -1.0~1.0）
 *
 * 拨码开关说明：
 * - switch_left/switch_right：1=上, 2=下, 3=中, 0=未知
 *
 * 拨轮说明：
 * - dial_wheel：原始值 0~2047，归一化到 -1.0~1.0（后拉为正，前推为负）
 *
 * @note 暂时只解析摇杆、拨码开关和拨轮，鼠标/键盘数据结构后续按需添加
 */
typedef struct
{
    // ===== 摇杆通道（归一化到 -1.0~1.0）=====
    float channel[4];       // 4 个摇杆通道，Watch 可观察
                            // channel[0]：右水平，channel[1]：右垂直
                            // channel[2]：左水平，channel[3]：左垂直

    // ===== 拨码开关（1=上, 2=下, 3=中, 0=未知）=====
    uint8_t switch_left;    // 左拨码开关状态，Watch 可观察
    uint8_t switch_right;   // 右拨码开关状态，Watch 可观察

    // ===== 拨轮（归一化到 -1.0~1.0）=====
    float dial_wheel;      // 拨轮值，后拉为正，前推为负，Watch 可观察

    // ===== 状态标志 =====
    uint8_t online;         // 遥控器在线状态，1=在线，0=离线，Watch 可观察
    uint8_t offline_cnt;    // 离线计数（调试用），Watch 可观察
} Remote_Data_t;

extern Remote_Data_t Remote_Data;

// ===== 速度映射参数【新增】=====
/**
 * @brief 遥控器速度映射参数结构体
 *
 * 将遥控器摇杆值映射到底盘速度。
 * 在 Watch 中添加 Remote_Params 可观察和修改映射参数。
 *
 * 映射关系：
 * - 右摇杆垂直 → vx（前进/后退）
 * - 右摇杆水平 → vy（左移/右移）
 * - 左摇杆水平 → vw（旋转）
 *
 * 参数说明：
 * - max_vx：最大前进速度（m/s）
 * - max_vy：最大横移速度（m/s）
 * - max_vw：最大旋转速度（rad/s）
 * - dead_zone：摇杆死区（0.0~0.1），消除摇杆漂移
 *
 * 底盘跟随参数：
 * - follow_kp：跟随角度P控制增益（推荐初始值：3.0，Watch调参）
 * - follow_ki：跟随角度I控制增益（暂时为0）
 * - follow_kd：跟随角度D控制增益（暂时为0）
 * - max_vw_follow：跟随模式最大旋转速度限制（rad/s）
 */
typedef struct
{
    float max_vx;           // 最大前进速度（m/s），Watch 可修改
    float max_vy;           // 最大横移速度（m/s），Watch 可修改
    float max_vw;           // 最大旋转速度（rad/s），Watch 可修改
    float dead_zone;        // 摇杆死区，Watch 可修改

    // ===== 底盘跟随参数【新增】=====
    float follow_kp;        // 跟随角度P控制增益，Watch 可修改
                            // 推荐初始值：3.0（角度误差1rad → 旋转速度3 rad/s）
                            // 调参方法：从1.0开始，逐步增加直到响应满意
    float follow_ki;        // 跟随角度I控制增益，Watch 可修改（暂时为0）
    float follow_kd;        // 跟随角度D控制增益，Watch 可修改（暂时为0）
    float max_vw_follow;    // 跟随模式最大旋转速度限制（rad/s），Watch 可修改
                            // 推荐值：6.0（约360°/s）
} Remote_Params_t;

extern Remote_Params_t Remote_Params;

// ===== 底盘状态机【新增】=====
/**
 * @brief 底盘状态机枚举
 *
 * 状态说明：
 * - CHASSIS_STOP：急停模式（DR16 双下触发），所有电机输出 0
 * - CHASSIS_REMOTE：遥控器正常控制
 */
typedef enum
{
    CHASSIS_STOP = 0,    // 急停模式：所有电机输出清零，摇杆无效
    CHASSIS_REMOTE = 1,  // 遥控器控制：正常运动学解算 + PID
} ChassisState_e;

/**
 * @brief 底盘状态机数据（仅保留核心状态字段）
 */
typedef struct
{
    ChassisState_e state;  // 当前状态：0=STOP, 1=REMOTE
} ChassisState_Data_t;

extern ChassisState_Data_t ChassisState_Data;

// ===== 3508速度环斜坡规划参数【新增】=====
/**
 * @brief 3508速度环斜坡规划参数结构体
 *
 * 用于限制目标速度变化率，避免增量式PID的ΔP项过大导致失控。
 * 原理：快速拨动遥控器 → 目标速度突变 → 误差突变 → ΔP=kp*巨大值 → cout累积过大 → 超调震荡
 * 解决：限制目标速度变化率 → 误差逐步增大 → ΔP逐步增大 → cout平滑累积 → 电机平滑加速
 *
 * 参数说明：
 * - max_rate：最大变化率（rpm/s），Watch可修改
 *   推荐值：1000 rpm/s（每秒最多改变1000 rpm）
 *   调大：响应更快，但可能失控
 *   调小：响应更慢，但更稳定
 *
 * - control_period：控制周期（s），固定为0.001（1kHz任务）
 *   max_delta_per_frame = max_rate * control_period
 *   每帧最大变化 = 1000 * 0.001 = 1 rpm
 *
 * - last_tar_speed[4]：上次目标速度数组，用于计算斜坡
 *   每帧更新：last_tar_speed[i] = ramped_tar_speed[i]
 *
 * Watch观察：
 * - Ramp_Params.max_rate：可实时修改斜坡速率
 * - Ramp_Params.last_tar_speed[0~3]：观察上次目标值
 * - Wheel_Data.tar_speed[0~3]：原始目标值（可能突变）
 * - ramped_tar_speed：斜坡后目标值（逐步变化）
 *
 * 使用方式：
 * 1. 在ChassisTask.cpp的Wheel_Task_Loop中，PID计算前调用斜坡规划
 * 2. 用斜坡后的目标值替代原始目标值进行PID计算
 */
typedef struct
{
    float max_rate;            // 最大变化率（rpm/s），Watch可修改
    float control_period;      // 控制周期（s），固定为0.001
    float last_tar_speed[4];   // 上次目标速度数组（rpm），用于计算斜坡
} Ramp_Params_t;

extern Ramp_Params_t Ramp_Params;

// ===== 板间通信调试变量【新增】=====
/**
 * @brief 板间通信调试数据结构体
 *
 * 包含云台-底盘板间通信的所有调试数据。
 * 在 Watch 中添加 BoardComm_Data 可观察通信状态和数据。
 *
 * 数据流：
 *   云台 → CAN2 (0x205/0x206) → Gimbal_to_Chassis → BoardComm_Data
 *   裁判系统 → Chassis_to_Gimbal → CAN2 (0x207/0x208) → 云台
 *
 * 接收数据（云台→底盘）：
 * - LX/LY：遥控器右摇杆（0-220，中值110）
 * - Rotating_vel：小陀螺速度
 * - Yaw_encoder_angle_err：云台-底盘角度误差（rad）
 * - chassis_mode：底盘模式位域
 * - friction_enabled：摩擦轮使能
 * - Vision：视觉状态
 *
 * 连接状态：
 * - online：云台在线状态
 * - last_frame_time：最后接收时间戳
 * - frame_timeout_cnt：超时计数
 *
 * 发送数据（底盘→云台）：
 * - booster_now_heat：当前热量
 * - booster_heat_max：热量上限
 * - booster_heat_cd：冷却时间
 * - launch_speed：发射速度
 *
 * CAN统计：
 * - rx_frame1_cnt/rx_frame2_cnt：接收帧计数
 * - tx_frame_cnt：发送计数
 * - error_cnt：错误计数
 */
typedef struct
{
    // ===== 接收数据（云台→底盘）=====
    uint8_t LX;                        // 右摇杆X（0-220，中值110）
    uint8_t LY;                        // 右摇杆Y（0-220，中值110）
    uint8_t Rotating_vel;              // 小陀螺速度
    int8_t wheel;                      // 拨轮值（-127~127，中值0）
    float Yaw_encoder_angle_err;       // 云台-底盘角度误差（rad）
    uint8_t chassis_mode;              // 底盘模式位域
    uint8_t friction_enabled;          // 摩擦轮使能
    uint8_t Vision;                    // 视觉状态

    // ===== 连接状态 =====
    bool online;                       // 云台在线状态
    uint32_t last_frame_time;          // 最后接收时间戳（ms）
    uint32_t frame_timeout_cnt;        // 超时计数

    // ===== 发送数据（底盘→云台）=====
    uint16_t booster_now_heat;         // 当前热量
    uint16_t booster_heat_max;         // 热量上限
    uint16_t booster_heat_cd;          // 冷却时间
    float launch_speed;                // 发射速度（m/s）

    // ===== CAN统计 =====
    uint32_t rx_frame1_cnt;            // 接收帧1计数
    uint32_t rx_frame2_cnt;            // 接收帧2计数
    uint32_t tx_frame_cnt;             // 发送计数
    uint32_t error_cnt;                // 错误计数
} BoardCommDebug_t;

extern BoardCommDebug_t BoardComm_Data;