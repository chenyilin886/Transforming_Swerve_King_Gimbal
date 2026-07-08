/**
 * @file GimbalInit.cpp
 * @brief 云台系统初始化与周期更新实现
 *
 * 数据流(DAY01 Joint 层)：
 *   CAN1 接收 → DM4310/DM4340::Parse → Motor unit_data_[]
 *   → JointManager::Update() → Joint.Update()
 *     ├─ encoder_angle   (Motor 原始角度)
 *     ├─ real_angle       ((encoder - offset) * direction)
 *     └─ normalized_angle (Yaw: [-π,π], Pitch/Fold: clamped)
 *   → 写入 Joint_Data → Keil Watch
 *
 * Watch 双向同步：
 *   Watch → Joint:  config(offset/limit/direction/calib_enable)
 *   Joint → Watch:  state(encoder/real/normalized/velocity/calib_state)
 */

#include "GimbalInit.hpp"
#include "Variable.hpp"
#include "Vofa.hpp"
#include "can.h"
#include "can_hal.hpp"
#include "DmMotor.hpp"
#include "Joint.hpp"
#include "Controller.hpp"
#include "TransformPlanner.hpp"

// ========================================================================
// 全局电机指针
// ========================================================================
namespace BSP::MOTOR::DM
{
DM4310* dm4310_yaw_pitch = nullptr;
DM4340* dm4340_fold       = nullptr;
}

// ========================================================================
// JointManager 全局实例
// ========================================================================
static BSP::JOINT::JointManager joint_manager;

// ========================================================================
// GimbalController 全局实例(Stage03 控制层)
// ========================================================================
// 设计原因：
//   - GimbalController 持有 Yaw/Pitch/Fold 三个 JointController
//   - 每周期 Update 从 JointManager 读 feedback → PID → Motor.ctrl_Mit
//   - Stage03 仅 Pitch 使能；Yaw/Fold 失能但发送零力矩保持在线
static BSP::CTRL::GimbalController gimbal_controller;

// ========================================================================
// TransformPlanner 全局实例(Stage05 变形规划器)
// ========================================================================
// 设计原因：
//   - Stage05 合并实现 Motion Planner + Morphology Manager
//   - 接收 Watch 命令(EXPAND/CONTRACT/ABORT/RESET)
//   - 按状态机执行两段串行动作（Pitch 先 → Fold 后 / Fold 先 → Pitch 后）
//   - TRANSITION 状态写入 Controller_Data.pitch/fold.target_angle
//   - 终态/IDLE/ABORT 释放 target 给 Watch
//   - 调用位置：JointManager.Update 之后，syncDataToController 之前（Step 2.5）
static BSP::PLANNER::TransformPlanner transform_planner;

// ========================================================================
// Watch ↔ Joint 同步辅助
// ========================================================================

/**
 * @brief Watch config → Joint config(每周期同步，允许在线修改参数)
 */
static inline void syncConfigToJoint(BSP::JOINT::Joint &joint, const Joint_Config_t &cfg)
{
    joint.config().offset       = cfg.offset;
    joint.config().limit_min    = cfg.limit_min;
    joint.config().limit_max    = cfg.limit_max;
    joint.config().direction    = cfg.direction;
    joint.config().continuous   = cfg.continuous;
    joint.config().calib_enable = cfg.calib_enable;
}

/**
 * @brief Joint state → Watch(每周期回写，Watch 实时观察)
 *
 * @note 必须回写 config 中的 offset / calib_enable / calib_state：
 *   - offset:       校准后 Joint 内部修改了 offset，需回写让 Watch 显示最新值
 *   - calib_enable: Joint.Update 消耗后清零，需回写避免下周期重复触发
 *   - calib_state: 0→1 变化需在 Watch 可见
 */
static inline void syncJointToData(const BSP::JOINT::Joint &joint, Joint_Data_Unit_t &data)
{
    data.encoder_angle    = joint.getEncoderAngle();
    data.real_angle       = joint.getRealAngle();
    data.normalized_angle = joint.getNormalizedAngle();
    data.velocity         = joint.getVelocity();
    data.torque           = joint.getTorque();
    data.temperature      = joint.getTemperature();
    data.online           = joint.isOnline() ? 1 : 0;
    data.target_angle     = joint.getTargetAngle();

    // 回写 config(Joint 内部可能修改了 offset / calib_enable)
    const auto &cfg = joint.getConfig();
    data.config.offset       = cfg.offset;
    data.config.calib_enable = cfg.calib_enable;
    data.config.calib_state  = joint.getCalibState();
}

// ========================================================================
// Watch ↔ Controller 同步辅助(Stage03)
// ========================================================================
/**
 * @brief Watch → JointController(每周期同步，允许在线调参/启停)
 *
 * 同步字段(Watch 可写)：
 *   target_angle        : 目标角度(rad)
 *   kp/ki/kd            : 角度环 PID 参数(单级模式=主PID; 串级模式=外环)
 *   torque_limit        : 输出力矩限幅
 *   break_i             : 角度环积分隔离阈值
 *   limit_i             : 角度环积分输出限幅
 *   cascade_mode        : 串级模式开关(0=单级, 1=串级)
 *   vel_kp/vel_ki/vel_kd: 速度环 PID 参数(仅 cascade_mode=1 生效)
 *   vel_limit           : 速度环输出限幅(rad/s)
 *   break_i_vel         : 速度环积分隔离阈值
 *   limit_i_vel         : 速度环积分输出限幅
 *   enabled             : 使能标志
 *
 * @note cascade_mode 切换时会清空两个 PID 状态，避免残留积分导致输出跳变
 */
static inline void syncDataToController(const Controller_Data_Unit_t &data,
                                        BSP::CTRL::JointController &ctrl)
{
    // 角度环 PID 参数(Watch 在线调参)
    ctrl.kpid.kp = data.kp;
    ctrl.kpid.ki = data.ki;
    ctrl.kpid.kd = data.kd;

    // 速度环 PID 参数(仅 cascade_mode=1 时使用)
    ctrl.kpid_vel.kp = data.vel_kp;
    ctrl.kpid_vel.ki = data.vel_ki;
    ctrl.kpid_vel.kd = data.vel_kd;

    // 限幅参数
    ctrl.torque_limit = data.torque_limit;
    ctrl.break_i      = data.break_i;
    ctrl.limit_i      = data.limit_i;
    ctrl.vel_limit     = data.vel_limit;
    ctrl.break_i_vel   = data.break_i_vel;
    ctrl.limit_i_vel   = data.limit_i_vel;

    // 重力补偿参数(Watch 在线标定)
    //   仅串级模式生效(gravity_enable && cascade_mode)
    //   公式: gravity_torque = gravity_k * cos(feedback_angle)
    ctrl.gravity_k     = data.gravity_k;
    ctrl.gravity_enable = data.gravity_enable ? 1 : 0;

    // 同步到 PID 内部(积分隔离 + 积分限幅)
    ctrl.position_pid.pid.Break_I = data.break_i;
    ctrl.position_pid.pid.MixI    = data.limit_i;
    ctrl.velocity_pid.pid.Break_I = data.break_i_vel;
    ctrl.velocity_pid.pid.MixI    = data.limit_i_vel;

    // 串级模式切换：检测到模式变化时清空两个 PID，避免状态污染
    //   单级→串级：清空速度环（之前没运行过）
    //   串级→单级：清空速度环（不再使用）
    {
        uint8_t new_mode = data.cascade_mode ? 1 : 0;
        if (new_mode != ctrl.cascade_mode)
        {
            ctrl.cascade_mode = new_mode;
            ctrl.position_pid.clearPID();
            ctrl.velocity_pid.clearPID();
        }
    }

    // 目标角度同步规则：
    //   - 控制器首次运行时(target_inited=0)：target 由 feedback 自动初始化
    //   - 控制器已初始化(target_inited=1)：Watch 中的 target_angle 生效
    //   这样保证上电瞬间 error=0，不会输出冲击力矩
    if (ctrl.target_inited)
    {
        ctrl.target_angle = data.target_angle;
    }

    // 使能控制(Watch 切换 0/1)
    if (data.enabled && !ctrl.enabled)
    {
        ctrl.Enable();
    }
    else if (!data.enabled && ctrl.enabled)
    {
        ctrl.Disable();
    }
}

/**
 * @brief JointController → Watch(每周期回写，Watch 实时观察)
 *
 * 回写字段(Watch 只读)：
 *   feedback_angle : 反馈角度
 *   error          : 角度环误差
 *   vel_target     : 速度环目标(=角度环输出, 串级模式有效)
 *   vel_feedback   : 速度环反馈(=Joint.velocity)
 *   vel_error      : 速度环误差
 *   torque_output  : 输出力矩
 *   limit_min/max  : 关节限位
 */
static inline void syncControllerToData(const BSP::CTRL::JointController &ctrl,
                                        Controller_Data_Unit_t &data,
                                        const BSP::JOINT::Joint &joint)
{
    data.feedback_angle = ctrl.feedback_angle;
    data.error          = ctrl.error;
    data.vel_target     = ctrl.vel_target;
    data.vel_feedback   = ctrl.vel_feedback;
    data.vel_error      = ctrl.vel_error;
    data.torque_output  = ctrl.torque_output;
    data.gravity_torque = ctrl.gravity_torque;  // 重力补偿输出(N·m)
    data.enabled        = ctrl.enabled;
    data.cascade_mode   = ctrl.cascade_mode;
    // 回写 target_angle：首次初始化时让 Watch 看到当前实际 target
    data.target_angle   = ctrl.target_angle;
    // 限位值从 Joint 同步
    data.limit_min      = joint.getConfig().limit_min;
    data.limit_max      = joint.getConfig().limit_max;
}

// ========================================================================
// 初始化
// ========================================================================
void GimbalInit()
{
    // 1. 获取 CAN 总线(首次调用自动初始化: 过滤器 + 启动 + 中断)
    auto &can1 = HAL::CAN::get_can_bus_instance().get_can1();

    // 2. 创建电机实例(static 局部，生命周期 = 程序整个运行期)
    static BSP::MOTOR::DM::DM4310 dm4310(&can1);
    static BSP::MOTOR::DM::DM4340 dm4340(&can1);

    // 3. 赋值全局指针
    BSP::MOTOR::DM::dm4310_yaw_pitch = &dm4310;
    BSP::MOTOR::DM::dm4340_fold       = &dm4340;

    // 4. 注册接收回调
    can1.register_rx_callback(
        [](const HAL::CAN::Frame &frame)
        {
            dm4310.Parse(frame);
        });

    can1.register_rx_callback(
        [](const HAL::CAN::Frame &frame)
        {
            dm4340.Parse(frame);
        });

    // 5. 初始化 JointManager
    joint_manager.Init();

    // 6. 初始化 GimbalController(Stage03 默认 pitch.Enable，yaw/fold Disable)
    gimbal_controller.Init();

    // 7. 使能电机
    HAL_Delay(500);

    dm4310.On(1);
    HAL_Delay(10);

    dm4310.On(2);
    HAL_Delay(10);

    dm4340.On(1);
    HAL_Delay(10);
}


// ========================================================================
// 周期更新
// ========================================================================
void GimbalUpdate()
{
    using namespace BSP::MOTOR::DM;

    // ---------------------------------------------------------------
    // Step 1: Watch config → Joint(允许在线修改 offset/limit/calib)
    // ---------------------------------------------------------------
    syncConfigToJoint(joint_manager.yaw,   Joint_Data.yaw.config);
    syncConfigToJoint(joint_manager.pitch, Joint_Data.pitch.config);
    syncConfigToJoint(joint_manager.fold,  Joint_Data.fold.config);

    // ---------------------------------------------------------------
    // Step 2: JointManager.Update 从 Motor 读取 → 计算 real_angle
    // ---------------------------------------------------------------
    joint_manager.Update(dm4310_yaw_pitch, dm4340_fold);

    // ---------------------------------------------------------------
    // Step 2.5: TransformPlanner.Update（Stage05 变形动作规划）
    //   位置：Joint feedback 已更新 / Controller target 尚未同步
    //   职责：
    //     - 读 Watch 命令(Transform_Config.cmd)
    //     - 状态机执行（IDLE/TRANSITION/终态/ABORT）
    //     - TRANSITION 状态: 写 Controller_Data.pitch/fold.target_angle
    //     - 终态/IDLE/ABORT: 不写（释放给 Watch）
    //     - 异常(超时/离线): snap target 到 feedback（Hold 当前位置）
    //   调用频率：1kHz（与 GimbalUpdate 一致）
    // ---------------------------------------------------------------
    transform_planner.Update(joint_manager,
                             Controller_Data,
                             Transform_Config,
                             Transform_Status);

    // ---------------------------------------------------------------
    // Step 3: Watch → GimbalController(在线调参 / 设目标 / 启停)
    //   注意：Planner 在 Step 2.5 已可能修改 pitch/fold.target_angle
    //         此处 syncDataToController 会将最新 target 同步到 JointController
    // ---------------------------------------------------------------
    syncDataToController(Controller_Data.yaw,   gimbal_controller.yaw);
    syncDataToController(Controller_Data.pitch, gimbal_controller.pitch);
    syncDataToController(Controller_Data.fold,  gimbal_controller.fold);

    // ---------------------------------------------------------------
    // Step 4: GimbalController.Update 控制循环
    //   读取 Joint.feedback → PID 计算 → Motor.ctrl_Mit(纯力矩)
    //   失能关节也发送零力矩，保持电机在线
    // ---------------------------------------------------------------
    gimbal_controller.Update(joint_manager, dm4310_yaw_pitch, dm4340_fold);

    // ---------------------------------------------------------------
    // Step 5: Joint state + Controller state → Watch(实时观察)
    // ---------------------------------------------------------------
    syncJointToData(joint_manager.yaw,   Joint_Data.yaw);
    syncJointToData(joint_manager.pitch, Joint_Data.pitch);
    syncJointToData(joint_manager.fold,  Joint_Data.fold);

    syncControllerToData(gimbal_controller.yaw,   Controller_Data.yaw,   joint_manager.yaw);
    syncControllerToData(gimbal_controller.pitch, Controller_Data.pitch, joint_manager.pitch);
    syncControllerToData(gimbal_controller.fold,  Controller_Data.fold,  joint_manager.fold);

    // ---------------------------------------------------------------
    // Step 6: VOFA+ 波形发送（降频到 500Hz）
    //   调用 Variable.cpp 中的 VofaSendDebugChannels()
    //   修改通道配置：只需改 Variable.cpp，无需改此文件
    // ---------------------------------------------------------------
    static uint8_t vofa_counter = 0;
    vofa_counter++;
    if (vofa_counter >= 2)
    {
        vofa_counter = 0;
        VofaSendDebugChannels();  // ← 在 Variable.cpp 中实现，方便修改通道
    }
}
