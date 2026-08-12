/**
 * @file Variable.cpp
 * @brief 底盘全局变量定义
 *
 * 所有全局变量在此集中定义，确保符号唯一。
 * 初始参数继承自旧工程经比赛验证的数值。
 */
#include "Variable.hpp"
#include <math.h> // 用于 M_PI 定义

// ===== 3508 速度环 PID 参数 =====
// 初始参数：kp=10, ki=0, kd=0（增量式 PID）
// Watch 中可直接修改 Kpid_3508_vel.kp/ki/kd 进行在线调参
Kpid_t Kpid_3508_vel(0, 0, 0);

// 4 个轮速度环 PID 实例（增量式）
// 增量式 PID 不需要积分隔离和积分限幅（积分项只依赖当前误差）
PID pid_vel_Wheel[4] = {
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
};

// ===== MG4005 角度环 PID 参数 =====
// 串级 PID 外环：角度环（位置式），输出目标速度（rpm）
// kp=5.0：角度误差 1° → 目标速度 5 rpm（适中响应）
// ki=0：角度环不使用积分（速度环会消除稳态误差）
// kd=0：暂不使用微分（避免噪声放大）
Kpid_t Kpid_4005_angle(0, 0, 0);

// 4 个舵向角度环 PID 实例（位置式）
// Break_I=10：角度误差小于 10° 时才积分（当前 ki=0，此参数无效）
// MixI=200：积分输出限幅 200（当前 ki=0，此参数无效）
PID pid_angle_Steer[4] = {
    {10, 200},
    {10, 200},
    {10, 200},
    {10, 200},
};

// ===== MG4005 速度环 PID 参数 =====
// 串级 PID 内环：速度环（增量式），输出力矩（-2048~2048）
// kp=20：速度误差 1 rpm → 力矩增量 20（快速响应）
// ki=0：增量式 PID 不需要积分项（增量本身包含积分效果）
// kd=0：暂不使用微分
Kpid_t Kpid_4005_vel(0, 0, 0);

// 4 个舵向速度环 PID 实例（增量式）
// 增量式 PID 不需要积分隔离和积分限幅
PID pid_vel_Steer[4] = {
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
};

// ===== 轮向电机数据（3508）=====
// 上电自动启动（安全保护：kp=0 时输出零电流）
Wheel_Data_t Wheel_Data = {
    .tar_speed = {0, 0, 0, 0},
    .final_3508_Out = {0, 0, 0, 0},
    
    // ===== 运动学相关字段 =====
    .vx = 0, .vy = 0, .vw = 0,           // 目标速度为 0（安全）
    .vx_fdb = 0, .vy_fdb = 0, .vw_fdb = 0, // 反馈速度为 0
    
    // ===== 使能标志（上电自动启动）=====
    .Wheel_Enable = 1,        // 轮向使能（kp=0 保护生效）
    .Kinematics_Enable = 1    // 运动学使能（目标速度为 0）
};

// ===== 舵向数据 =====
// 零位校准参数已固化（用户提供的 offset 值）
// 上电后自动加载校准参数，无需重复校准
// 上电自动启动（安全保护：kp=0 时输出零力矩，初始角度同步）
Steer_Data_t Steer_Data = {
    // 目标角度、当前角度、速度等字段初始化为 0
    .tar_angle = {0, 0, 0, 0},
    .cur_angle = {0, 0, 0, 0},
    .tar_speed = {0, 0, 0, 0},
    .cur_speed = {0, 0, 0, 0},
    .final_4005_Out = {0, 0, 0, 0},
    .angle_diff = {0, 0, 0, 0},
    
    // ===== 使能标志（上电自动启动）=====
    .Steer_Enable = 1,       // 舵向使能（触发初始角度同步）
    .Steer_InitSync = 0,     // 未同步（首次使能时自动同步）
    .Steer_Online = 0,       // 电机在线状态（运行时更新）
    
    // ===== 零位校准参数（固化）=====
    // 用户提供的 offset 值（度）
    // encoder_angle - offset = real_angle
    .offset = {254.119263f, 153.71521f, 64.6105957f, 318.78479f},
    
    // 真实角度、编码器角度初始化为 0（运行时计算）
    .real_angle = {0, 0, 0, 0},
    .encoder_angle = {0, 0, 0, 0},
    
    // 校准状态：已校准（固化）
    .Calib_Enable = 0,     // 不需要重新校准
    .Calib_Status = 1,     // 已校准
    .Calib_Step = 0        // 校准完成
};

// ===== 运动学参数 =====
// 初始参数继承自旧工程经比赛验证的数值
// 典型值：标准 X 型布局舵轮底盘
// R：底盘半径（轮子投影点到中心距离），根据实际底盘尺寸调整
// S：轮子半径，根据实际轮子尺寸调整
// Wheel_Azimuth：轮子安装方位角（弧度），从中心指向轮子的方向
// Phase：相位补偿（弧度），用于补偿机械安装误差
// SpeedGain：速度增益系数，用于调整整体速度响应
// RotationalGain：旋转增益系数，用于调整旋转响应

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

Kinematics_Params_t Kinematics_Params = {
    .R = 0.247f,            // 底盘半径
    .S = 0.06f,             // 轮子半径
    .Wheel_Azimuth = {      // 轮子安装方位角（标准 X 型布局）
        M_PI / 4.0f,        // 轮1：45°（右前方）
        3.0f * M_PI / 4.0f, // 轮2：135°（左前方）
        -3.0f * M_PI / 4.0f,// 轮3：-135°（左后方）
        -M_PI / 4.0f        // 轮4：-45°（右后方）
    },
    .Phase = {-0.11, -0.11, 0, 0},  // 相位补偿（无相位补偿）
    .SpeedGain = 1.0f,      // 速度增益系数
    .RotationalGain = 1.0f  // 旋转增益系数
};

// ===== 遥控器数据（DR16）=====
// 初始化为 0，上电后由 DR16.Parse() 更新
// Watch 中可观察 Remote_Data 的所有字段
Remote_Data_t Remote_Data = {0};

// ===== 速度映射参数 =====
// 将遥控器摇杆值（-1.0~1.0）映射到底盘速度（m/s, rad/s）
// Watch 中可修改 Remote_Params 的参数
Remote_Params_t Remote_Params = {
    .max_vx = 20.0f,         // 最大前进速度
    .max_vy = 20.0f,         // 最大横移速度
    .max_vw = 80.0f,         // 最大旋转速度
    .dead_zone = 0.05f      // 摇杆死区 5%
};

// ===== 底盘状态机数据 =====
// 上电默认 CHASSIS_REMOTE：正常遥控器控制
// 双下(S1==2 && S2==2) 时切换到 CHASSIS_STOP
ChassisState_Data_t ChassisState_Data = {
    .state = CHASSIS_REMOTE
};

// ===== 3508速度环斜坡规划参数【新增】=====
// 上电初始化：
// - max_rate = 1000 rpm/s：推荐初始值，可Watch实时调整
// - control_period = 0.001 s：固定为1kHz任务周期
// - last_tar_speed[4] = {0,0,0,0}：上次目标速度初始为0
//
// 调参方法：
// - Watch中修改Ramp_Params.max_rate可实时调整斜坡速率
// - max_rate调大：响应更快，但可能失控（如设为2000 rpm/s）
// - max_rate调小：响应更慢，但更稳定（如设为500 rpm/s）
//
// 计算关系：
// max_delta_per_frame = max_rate * control_period = 1000 * 0.001 = 1 rpm
// 每帧最多改变1 rpm，1秒内最多改变1000 rpm
Ramp_Params_t Ramp_Params = {
    .max_rate = 20000.0f,        // 1000 rpm/s（推荐初始值）
    .control_period = 0.001f,   // 1ms（1kHz任务）
    .last_tar_speed = {0, 0, 0, 0}  // 上次目标速度初始为0
};