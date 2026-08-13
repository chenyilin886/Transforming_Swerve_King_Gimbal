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
 * 数据流(DR16 遥控器)：
 *   UART3 DMA接收 → HAL_UARTEx_RxEventCallback → DR16.Parse()
 *     → 更新摇杆/开关/鼠标/键盘状态 → Variable.cpp 回写 Watch
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
#include "LkMotor.hpp"
#include "DjiMotor.hpp"
#include "Joint.hpp"
#include "Controller.hpp"
// DialController.hpp 已移到 ShootFSM.hpp 中 include（发射机构控制迁移到 ShootTask）
#include "TransformPlanner.hpp"
#include "DR16.hpp"
#include "HI12H3_IMU.hpp"
#include "Communication/BoardComm.hpp"
#include "Communication/ChassisModeManager.hpp"  // 底盘模式状态机
#include "Communication/VisionComm.hpp"         // Vision communication (RCIA)

// ========================================================================
// 全局电机指针
// ========================================================================
namespace BSP::MOTOR::DM
{
DM4310* dm4310_yaw_pitch = nullptr;
DM4340* dm4340_fold       = nullptr;
volatile uint32_t dm_fold_feedback_parse_count = 0;
volatile uint32_t dm_fold_feedback_header_fallback_count = 0;
volatile uint32_t dm_fold_feedback_last_tick = 0;
volatile uint32_t dm_fold_feedback_max_gap_ms = 0;
volatile uint32_t dm_fold_control_tx_attempt_count = 0;
volatile uint32_t dm_fold_control_tx_success_count = 0;
volatile uint32_t dm_fold_control_tx_fail_count = 0;
volatile uint32_t dm_parse_total_count = 0;
volatile uint32_t dm_parse_frame_id_0x00 = 0;
volatile uint32_t dm_parse_frame_id_0x01 = 0;
volatile uint32_t dm_parse_frame_id_0x02 = 0;
volatile uint32_t dm_parse_frame_id_0x03 = 0;
volatile uint32_t dm_parse_normal_match_count = 0;
volatile uint32_t dm_parse_legacy_match_count = 0;
volatile uint32_t dm_parse_no_match_count = 0;
}

namespace BSP::MOTOR::LK
{
// LK4005 全局指针（定义在 LkMotor.hpp 中 extern 声明）
//   ID=1, CAN1, 用于 Pitch 关节力矩控制（数据接收验证阶段）
//   使用前必须：GimbalInit 中创建实例 + On(1) 使能 + GimbalUpdate 周期性 ctrl_Torque
LK4005* lk4005_motor = nullptr;
}

namespace BSP::MOTOR::DJI
{
// GM3508 全局指针（DjiMotor.hpp 中 extern 声明）
//   两个电机 ID=1, 2, CAN1, 用于摩擦轮速度控制
//   使用前必须：GimbalInit 中创建实例 + On() 使能 + ShootTask 周期性 ctrl_Current + sendCAN
GM3508<2>* motor_3508 = nullptr;
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
// 发射机构（拨盘 + 摩擦轮预留）控制已迁移到 ShootTask
// ========================================================================
// 迁移说明：
//   旧版：dial_controller 在 GimbalInit.cpp 作为 static 实例，由 GimbalUpdate()
//         每 1ms 同步调用 updateDialControl()。
//   新版：拨盘控制已迁移到独立的 FreeRTOS 任务 ShootTask（4ms 周期, 250Hz），
//         由 Class_ShootFSM 统一管理发射机构状态机：
//           shootTask (4ms)
//             └─→ Class_ShootFSM::Control()
//                   ├─→ updateStateMachine_()        // 安全检查 + 状态切换
//                   ├─→ applyStateToDialConfig_()    // state → Dial_Config.enabled
//                   ├─→ dial_ctrl.Update(...)        // 委托拨盘双环控制
//                   ├─→ updateFriction_()            // 预留: 摩擦轮控制(空)
//                   └─→ syncStatus_()                // 回写 Shoot_Status
//
// 设计理由：
//   - 与参考工程 ShootTask 架构一致（H_SG_Gimbal 参考工程）
//   - 发射机构与云台关节控制解耦，避免互相阻塞
//   - 未来加摩擦轮（DJI 3508）时只需扩展 Class_ShootFSM，任务接口不变
//
// 保留内容：
//   - LK4005 电机实例创建、CAN 注册、On(1) 仍在 GimbalInit()
//   - LK4005_Data 反馈同步仍在 GimbalUpdate() Step 6.6
//   - lk4005_motor 指针仍由 GimbalInit.cpp 提供给 ShootFSM.cpp 使用
//
// 已移除：
//   - static BSP::CTRL::DialController dial_controller（迁到 ShootFSM 内部）
//   - static void updateDialControl()（迁到 ShootFSM::Control()）
//   - GimbalUpdate() Step 0.6 对 updateDialControl() 的调用


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

/**
 * @brief GimbalController → Watch(Yaw + Pitch IMU 反馈源信息)
 *
 * 在 syncControllerToData 之后额外同步 IMU 反馈源字段：
 *   yaw_fb_source   : Yaw 反馈来源(1=IMU, 0=编码器回退)
 *   pitch_fb_source : Pitch 反馈来源(1=IMU, 0=编码器回退)
 *
 * @param gc    GimbalController 实例
 * @param yaw   Controller_Data.yaw
 * @param pitch Controller_Data.pitch
 */
static inline void syncImuToData(const BSP::CTRL::GimbalController &gc,
                                 Controller_Data_Unit_t &yaw,
                                 Controller_Data_Unit_t &pitch)
{
    yaw.feedback_source   = gc.yaw_fb_source;
    pitch.feedback_source = gc.pitch_fb_source;
}

// ========================================================================
// 初始化
// ========================================================================
void GimbalInit()
{
    // 1. 获取 CAN 总线(首次调用自动初始化: 过滤器 + 启动 + 中断)
    auto &can1 = HAL::CAN::get_can_bus_instance().get_can1();
    auto &can2 = HAL::CAN::get_can_bus_instance().get_can2();

    // 2. 创建电机实例(static 局部，生命周期 = 程序整个运行期)
    static BSP::MOTOR::DM::DM4310 dm4310(&can1);
    static BSP::MOTOR::DM::DM4340 dm4340(&can1);
    static BSP::MOTOR::LK::LK4005 lk4005(&can1, 1);
    // GM3508 摩擦轮电机: motor_id=1(左), motor_id=2(右), CAN1
    static BSP::MOTOR::DJI::GM3508<2> motor3508(&can1, 0x200, {1, 2}, 0x200);

    // 3. 赋值全局指针
    BSP::MOTOR::DM::dm4310_yaw_pitch = &dm4310;
    BSP::MOTOR::DM::dm4340_fold       = &dm4340;
    BSP::MOTOR::LK::lk4005_motor      = &lk4005;
    BSP::MOTOR::DJI::motor_3508       = &motor3508;

    // 4. 注册接收回调 — CAN1(三电机) + CAN2(3508 摩擦轮)
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

    can1.register_rx_callback(
        [](const HAL::CAN::Frame &frame)
        {
            lk4005.Parse(frame);
        });

    can1.register_rx_callback(
        [](const HAL::CAN::Frame &frame)
        {
            motor3508.Parse(frame);
        });

    // 5. 初始化 JointManager
    joint_manager.Init();

    // 6. 初始化 GimbalController(Stage03 默认 pitch.Enable，yaw/fold Disable)
    gimbal_controller.Init();

    // 7. 初始化 DR16 遥控器(Stage04 UART3 DMA接收)
    BSP::Remote::DR16::Instance().Init();

    // 8. 初始化 IMU(Stage03 接入传感器, USART1 DMA空闲中断接收)
    BSP::IMU::imu.Init();

    // Stage07 vision communication init (USART6 DMA idle RX)
    VisionComm::Manager::Instance().Init();

    // 9. 使能全部电机
    HAL_Delay(500);

    dm4310.On(1);
    HAL_Delay(10);

    dm4310.On(2);
    HAL_Delay(10);

    dm4340.On(1);
    HAL_Delay(10);

    lk4005.On(1);
    HAL_Delay(10);
    lk4005.ctrl_Torque(1, 0);
    HAL_Delay(10);

    // 3508 摩擦轮电机：无需使能（DJI 电机上电即反馈），仅发送电流 0 初始化
    motor3508.ctrl_Current(1, 0);
    motor3508.ctrl_Current(2, 0);
    motor3508.sendCAN();
    HAL_Delay(10);
}


// ========================================================================
// 周期更新
// ========================================================================
void GimbalUpdate()
{
    using namespace BSP::MOTOR::DM;

    // ---------------------------------------------------------------
    // Step 0: DR16 遥控器离线检测（已移至 Step 2.55 e-stop 状态机统一处理）
    //   避免多次 IsOffline() 导致开关值被提前重置为 UNKNOWN，
    //   干扰 S1==DOWN && S2==DOWN 的判断
    // ---------------------------------------------------------------

    // ---------------------------------------------------------------
    // Step 0.5: IMU 离线检测（每周期执行）
    //   HI12H3 输出 200Hz，超过 50ms(10帧)未收到数据判离线
    //   离线时自动清除 ORE 错误，防止 UART 死锁
    //   本阶段(正确解析传感器数据)仅检测，不影响控制环
    // ---------------------------------------------------------------
    BSP::IMU::imu.IsOffline();

    // ---------------------------------------------------------------
    // Step 0.6: 发射机构控制已迁移到 ShootTask（4ms 周期, FreeRTOS 独立任务）
    //
    // 【迁移说明】
    //   旧版：updateDialControl() 在此处每 1ms 同步调用
    //   新版：shootTask (4ms) → Class_ShootFSM::Control() → DialController.Update()
    //
    // 【GimbalUpdate 保留的发射机构相关逻辑】
    //   - LK4005_Data 反馈同步：Step 6.6（仅读反馈, 不发命令, 不冲突）
    //   - LK4005 电机实例创建/CAN 注册：GimbalInit() 中
    //
    // 【任务间共享资源分析】
    //   - LK4005 电机实例：
    //       ShootTask → ctrl_Torque() 写 CAN 邮箱（4ms 一次）
    //       GimbalTask → getAngleRad() 读 float（32位对齐, 原子性可接受）
    //       CAN 中断 → Parse() 更新 unit_data_
    //     无需加锁，参考工程也是这样做的
    //   - DR16 单例：ShootTask 读 wheel/S1/S2，GimbalTask 读摇杆，只读不冲突
    //
    // 【调试观察点】
    //   Watch 添加 Shoot_Status / Dial_Status / Dial_Config / Shoot_Config
    //   - Shoot_Status.state: 0=DISABLE, 3=AUTO（safety_ok 时应自动进 AUTO）
    //   - Dial_Status.target_angle: 拨轮触发时应增大 0.698 rad（40°）
    // ---------------------------------------------------------------

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
    // Step 2.1: DM 电机离线重使能
    //
    // 【问题根因】
    //   DM 电机在 GimbalInit 中 On() 使能后，如果看门狗超时时间内
    //   未收到 ctrl_Mit 控制命令（FreeRTOS 启动延迟约50-70ms），
    //   电机会自动失能。失能后 ctrl_Mit 到达也不回复 → online=0 死锁。
    //
    // 【解决方案】
    //   检测到 DM 电机离线时，每 200ms 重新发送 On() 命令重新使能。
    //   重使能后电机回复一帧 → online 恢复 → ctrl_Mit 维持在线。
    //   200ms 间隔避免频繁发送 On() 冲击总线。
    //
    // 【急停保护】
    //   急停时跳过重使能：Step 2.55 已显式 Off() 电机，不应再 On()。
    // ---------------------------------------------------------------
    if (!Remote_State.estop_active)
    {
        static uint32_t last_reenable_tick = 0;
        uint32_t now = HAL_GetTick();
        if (now - last_reenable_tick >= 200)
        {
            last_reenable_tick = now;
            if (dm4310_yaw_pitch != nullptr)
            {
                if (!dm4310_yaw_pitch->isConnected(1)) dm4310_yaw_pitch->On(1);
                if (!dm4310_yaw_pitch->isConnected(2)) dm4310_yaw_pitch->On(2);
            }
            if (dm4340_fold != nullptr)
            {
                if (!dm4340_fold->isConnected(1)) dm4340_fold->On(1);
            }
        }
    }

    // ---------------------------------------------------------------
    // Step 2.55: 遥控器状态机（急停 + S1 边沿检测 → Planner 命令）
    //
    // 【需求】
    //   - S1==DOWN(2) && S2==DOWN(2) → 急停：所有电机失能防止疯转
    //   - S1==UP(1)    → 展开（边沿触发，发一次 EXPAND）
    //   - S1==MIDDLE(3) → 收起（边沿触发，发一次 CONTRACT）
    //
    // 【急停状态机】
    //   进入急停(0→1)：
    //     ① 保存 Controller_Data.{yaw,pitch,fold}.enabled → saved_*_en
    //     ② 置 enabled = 0（Step 3 syncDataToController 会同步到 ctrl.Disable）
    //     ③ 显式 Off() 所有 DM 电机（物理退出闭环控制）
    //     ④ 发 ABORT 给 Planner（中止动作 + snap target 到 feedback）
    //   急停保持：不做事（电机已 Off() + Controller 已失能，Step 2.1 已 guard）
    //   退出急停(1→0)：S1 或 S2 任一离开 DOWN 且遥控器在线
    //     ① 显式 On() 重新使能 DM 电机（物理进入闭环）
    //     ② 恢复 enabled 从 saved_*_en
    //     ③ 发 RESET 给 Planner（从 ABORT 回到 IDLE）
    //     ④ 跳过本周期 S1 边沿检测（防止 S1 从 DOWN 变化误触发）
    //
    // 【S1 边沿检测】（仅非急停时）
    //   检测 S1 档位变化：
    //     其他 → UP(1)     → cmd = EXPAND
    //     其他 → MIDDLE(3)  → cmd = CONTRACT
    //   保持在同一档位不重复发（Planner cmd 是单次触发型）
    //
    // 【DR16 离线行为】
    //   离线时 s1/s2 = UNKNOWN(0)，不满足条件①（DOWN&&DOWN）
    //   但 remote_offline==1 满足条件② → 进入急停 → 所有电机 Off() 失能
    //
    // 【调试观察点】Watch 中展开 Remote_State：
    //   estop_active       : 急停标志(1=急停中)
    //   s1/s2              : 当前开关值(1=UP,2=DOWN,3=MIDDLE)
    //   last_s1            : 上一周期 S1（边沿检测用）
    //   planner_cmd_sent   : 本周期发送的命令(0/1/2/3/4)
    //   saved_*_en         : 急停前 enabled 备份
    // ---------------------------------------------------------------
    {
        auto &dr16 = BSP::Remote::DR16::Instance();
        using Switch = BSP::Remote::DR16::Switch;

        Switch s1 = dr16.GetS1();
        Switch s2 = dr16.GetS2();

        // 遥控器离线检测
        //   IsOffline() 内部会更新 StateWatch 并检查超时
        //   离线时摇杆归零、开关置UNKNOWN
        bool remote_offline = dr16.IsOffline();

        // 急停条件：
        //   ① S1 和 S2 都在下档(DOWN=2)
        //   ② 遥控器离线
        //   离线时 s1/s2=UNKNOWN(0)，不满足条件①，但条件②触发急停
        bool estop_condition = (s1 == Switch::DOWN && s2 == Switch::DOWN) || remote_offline;

        // 更新观察状态
        Remote_State.s1 = static_cast<uint8_t>(s1);
        Remote_State.s2 = static_cast<uint8_t>(s2);
        Remote_State.remote_offline = remote_offline ? 1 : 0;
        Remote_State.planner_cmd_sent = 0;  // 每周期清零，仅记录本周期发送的命令

        if (estop_condition)
        {
            // === 急停状态 ===
            if (!Remote_State.estop_active)
            {
                // 进入急停（0→1）
                Remote_State.estop_active = 1;

                // 保存急停前的 enabled 状态（退出时恢复）
                Remote_State.saved_yaw_en   = Controller_Data.yaw.enabled;
                Remote_State.saved_pitch_en = Controller_Data.pitch.enabled;
                Remote_State.saved_fold_en  = Controller_Data.fold.enabled;

                // 失能所有电机（Step 3 syncDataToController → ctrl.Disable）
                Controller_Data.yaw.enabled   = 0;
                Controller_Data.pitch.enabled = 0;
                Controller_Data.fold.enabled  = 0;

                // 显式 Off() DM 电机：物理退出闭环控制，防止电机继续发力
                //   Step 2.1 已 guarded 不会反向 On()
                if (dm4310_yaw_pitch != nullptr)
                {
                    dm4310_yaw_pitch->Off(1);  // Yaw 电机物理失能
                    dm4310_yaw_pitch->Off(2);  // Pitch 电机物理失能
                }
                if (dm4340_fold != nullptr)
                {
                    dm4340_fold->Off(1);        // Fold 电机物理失能
                }

                // 发 ABORT 给 Planner（中止动作 + snap target 到 feedback）
                Transform_Config.cmd = static_cast<uint8_t>(
                    BSP::PLANNER::TransformCmd::ABORT);
                Remote_State.planner_cmd_sent = static_cast<uint8_t>(
                    BSP::PLANNER::TransformCmd::ABORT);
            }
            // 急停保持：不更新 last_s1
            //   保持进入急停前的值（DOWN=2），退出急停后 S1 当前位置
            //   能自然触发边沿检测，无需手动再拨 S1
        }
        else
        {
            // === 非急停状态 ===
            if (Remote_State.estop_active)
            {
                // 退出急停（1→0）：
                //   条件：S1/S2 不都是 DOWN 且遥控器在线
                Remote_State.estop_active = 0;

                // 重新使能 DM 电机：物理进入闭环控制
                //   On() 后需在下一周期 Step 4 收到首个 ctrl_Mit 才能回复反馈
                if (dm4310_yaw_pitch != nullptr)
                {
                    dm4310_yaw_pitch->On(1);  // Yaw 电机重新使能
                    dm4310_yaw_pitch->On(2);  // Pitch 电机重新使能
                }
                if (dm4340_fold != nullptr)
                {
                    dm4340_fold->On(1);        // Fold 电机重新使能
                }

                // 恢复急停前的 enabled 状态
                Controller_Data.yaw.enabled   = Remote_State.saved_yaw_en;
                Controller_Data.pitch.enabled = Remote_State.saved_pitch_en;
                Controller_Data.fold.enabled  = Remote_State.saved_fold_en;

                // 发 RESET 给 Planner（从 ABORT 回到 IDLE）
                Transform_Config.cmd = static_cast<uint8_t>(
                    BSP::PLANNER::TransformCmd::RESET);
                Remote_State.planner_cmd_sent = static_cast<uint8_t>(
                    BSP::PLANNER::TransformCmd::RESET);

                // 不更新 last_s1（保持进入急停前的值 DOWN=2）
                //   本周期发 RESET，Planner: ABORT → IDLE
                //   下一周期边沿检测：S1=3 != last_s1=2 → 发 EXPAND
                //   退出急停后无需手动再拨 S1
            }
            else
            {
                // 正常状态：S1 边沿检测
                uint8_t cur_s1 = static_cast<uint8_t>(s1);

                if (cur_s1 != Remote_State.last_s1)
                {
                    // Planner 处于 ABORT 时，先发 RESET 恢复到 IDLE
                    // 不更新 last_s1，下一周期重新检测边沿发送实际命令
                    if (Transform_Status.state ==
                        static_cast<uint8_t>(BSP::PLANNER::TransformState::ABORT))
                    {
                        Transform_Config.cmd = static_cast<uint8_t>(
                            BSP::PLANNER::TransformCmd::RESET);
                        Remote_State.planner_cmd_sent = static_cast<uint8_t>(
                            BSP::PLANNER::TransformCmd::RESET);
                        // 不更新 last_s1：下周期 ABORT→IDLE 后重发 EXPAND/CONTRACT
                    }
                    else if (cur_s1 == static_cast<uint8_t>(Switch::UP) ||
                             cur_s1 == static_cast<uint8_t>(Switch::MIDDLE) ||
                             cur_s1 == static_cast<uint8_t>(Switch::DOWN))
                    {
                        // S1 → UP(1) 或 MIDDLE(3)：准备发送变形命令
                        // 【关键】先检查并修复 yaw 控制模式，确保变形期间 yaw 正常控制

                        // 检查当前是否处于跟随模式（yaw 速度环单环）
                        auto chassis_mode = BoardComm::ChassisModeManager::Instance().GetCurrentState();
                        bool in_follow_mode = (chassis_mode == BoardComm::ChassisMode::CHASSIS_FOLLOW);

                        if (in_follow_mode && gimbal_controller.yaw.cascade_mode == 0)
                        {
                            // 在跟随模式下，yaw 处于速度环单环（cascade_mode=0）
                            // 变形需要 yaw 进入串级模式，否则 yaw_online 检查失败 → ABORT
                            // 强制切换到串级模式（从 IMU 当前位置开始）
                            gimbal_controller.yaw.SwitchToCascadeMode(gimbal_controller.yaw_imu_angle);
                            Controller_Data.yaw.cascade_mode = 1;
                            Controller_Data.yaw.target_angle = gimbal_controller.yaw_imu_angle;
                            FollowMode_Data.control_mode = 0;  // 标记退出速度环模式
                        }

                        // 发送变形命令
                        if (cur_s1 == static_cast<uint8_t>(Switch::DOWN))
                        {
                            Transform_Config.cmd = static_cast<uint8_t>(
                                BSP::PLANNER::TransformCmd::CONTRACT);
                            Remote_State.planner_cmd_sent = static_cast<uint8_t>(
                                BSP::PLANNER::TransformCmd::CONTRACT);
                        }
                        else
                        {
                            Transform_Config.cmd = static_cast<uint8_t>(
                                BSP::PLANNER::TransformCmd::EXPAND);
                            Remote_State.planner_cmd_sent = static_cast<uint8_t>(
                                BSP::PLANNER::TransformCmd::EXPAND);
                        }
                        Remote_State.last_s1 = cur_s1;
                    }
                    else
                    {
                        // S1 → DOWN(2) 或 UNKNOWN(0)：不发命令
                        Remote_State.last_s1 = cur_s1;
                    }
                }
            }
        }
    }

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
    //   注意：Step 2.55 可能已写入 cmd(ABORT/RESET/EXPAND/CONTRACT)
    // ---------------------------------------------------------------
    transform_planner.Update(joint_manager,
                             Controller_Data,
                             Transform_Config,
                             Transform_Status);

    // ---------------------------------------------------------------
    // Step 2.58: Vision connection state and bumpless entry guard
    // ---------------------------------------------------------------
    VisionComm::Manager::Instance().IsConnected();

    auto &dr16_mode = BSP::Remote::DR16::Instance();
    using RemoteSwitch = BSP::Remote::DR16::Switch;
    RemoteSwitch s1_mode = dr16_mode.GetS1();
    RemoteSwitch s2_mode = dr16_mode.GetS2();
    const bool gimbal_deploy_requested =
        (s1_mode == RemoteSwitch::MIDDLE || s1_mode == RemoteSwitch::UP);
    const bool vision_requested =
        (s1_mode == RemoteSwitch::UP &&
         (s2_mode == RemoteSwitch::DOWN || s2_mode == RemoteSwitch::UP));
    const bool vision_ready =
        (vision_requested && VisionComm_Data.online && VisionComm_Data.vision_ready);

    static uint8_t vision_active = 0;
    uint8_t vision_just_entered = 0;

    if (vision_ready && !vision_active)
    {
        vision_active = 1;
        vision_just_entered = 1;

        if (joint_manager.pitch.isOnline() && gimbal_controller.imu_online)
        {
            Controller_Data.pitch.target_angle = gimbal_controller.pitch_imu_angle;
        }
        else if (joint_manager.pitch.isOnline())
        {
            Controller_Data.pitch.target_angle = joint_manager.pitch.getRealAngle();
        }

        if (joint_manager.yaw.isOnline())
        {
            if (FollowMode_Data.control_mode == 1)
            {
                gimbal_controller.yaw.SwitchToCascadeMode(gimbal_controller.yaw_imu_angle);
                Controller_Data.yaw.cascade_mode = 1;
                FollowMode_Data.control_mode = 0;
            }
            Controller_Data.yaw.target_angle = gimbal_controller.yaw_imu_angle;
        }
    }
    else if (!vision_ready && vision_active)
    {
        vision_active = 0;
        vision_just_entered = 0;
    }

    // ---------------------------------------------------------------
    // Step 2.6: 遥控器左摇杆Y轴 → Pitch 目标角度(速度积分模式)
    //
    // 【原理】
    //   任务要求："摇杆往上拨 pitch 往上抬，往下拨往下抬，归中时停止运动"
    //   "归中时停止运动" 决定必须采用【速度积分】而非位置映射：
    //     - 位置映射：ch3 → target_angle，归中时 target 回到中点 → 会运动 ✗
    //     - 速度积分：ch3 → target_velocity，target += v·dt
    //                 归中时 v=0 → target 保持 → 停止运动 ✓
    //
    // 【数据流】
    //   DR16.ch3 (-1~1)
    //     ↓ 死区过滤(消除摇杆归中噪声 ±0.05)
    //     ↓ × max_speed (1.0 rad/s)  → target_velocity (rad/s)
    //     ↓ × dt (0.001s, 1kHz)       → delta_angle (rad)
    //     ↓ 累加到 Controller_Data.pitch.target_angle
    //     ↓ 限位钳位 [limit_min, limit_max]  ← 读 Joint 实时配置
    //     ↓ Step 3 syncDataToController → PID.target_angle
    //     ↓ Step 4 串级 PID → Motor.ctrl_Mit
    //
    // 【位置选择】Step 2.5 之后、Step 3 之前
    //   - Planner 优先级高于遥控器(变形动作时不被干扰)
    //   - 当前阶段 Planner 未启用变形动作，恒为 IDLE 不写 target
    //   - 遥控器结果立即被 syncDataToController 同步到 PID
    //   - 复用现有串级 PID，零改动
    //
    // 【方向映射】(用户确认)
    //   ch3 > 0 (摇杆向上) → target_angle 增大 → pitch 上抬(枪口抬高)
    //   ch3 < 0 (摇杆向下) → target_angle 减小 → pitch 下抬
    //
    // 【离线行为】(用户确认)
    //   DR16 离线时摇杆自动归零(ch3=0) → target_velocity=0 → target 保持
    //   PID 维持 pitch 在最后位置，安全且符合机器人逻辑
    //
    // 【参数】(Watch 可在线调整: 见下方常量, 后续可提取到 Variable.hpp)
    //   max_speed = 1.0 rad/s  满幅时 1 秒转 1 rad ≈ 57°, 慢速调参起点
    //   dead_zone = 0.05       DR16 归中噪声约 ±0.03~0.05
    //   dt        = 0.001s     GimbalUpdate 1kHz
    //
    // 【调试观察点】
    //   Watch 中添加：
    //   - DR16_Data.ch3                       原始摇杆值
    //   - Controller_Data.pitch.target_angle  积分后的目标角度
    //   - Controller_Data.pitch.feedback_angle 实际角度(应跟随 target)
    //   - Joint_Data.pitch.config.limit_min/max 限位值
    //
    // 【未来扩展】
    //   Stage05+ Planner 启用变形动作时，需在此处增加状态判断：
    //     if (Transform_Status.state == IDLE/终态) { 遥控器生效 }
    //     else { 遥控器不写 target，让 Planner 控制 }
    //   当前阶段 Planner 恒 IDLE，直接生效即可。
    //
    // 【Stage05+ 更新】遥控器摇杆生效条件：
    //   ① 非急停（Remote_State.estop_active == 0）
    //   ② Planner 在 IDLE 或终态（不在 TRANSITION 动作中）
    //   满足两者才积分 target，否则跳过（让 Planner/急停控制）
    // ---------------------------------------------------------------
    {
        // 急停时跳过摇杆积分（电机已失能，不应改 target）
        // Planner TRANSITION 时跳过（让 Planner 控制 target）
        if (Remote_State.estop_active ||
            BSP::PLANNER::isTransitionState(
                static_cast<BSP::PLANNER::TransformState>(Transform_Status.state)))
        {
            // Skip this cycle.
        }
        else if (gimbal_deploy_requested && vision_active)
        {
            if (!vision_just_entered && joint_manager.pitch.isOnline())
            {
                float vision_pitch_rad = VisionComm_Data.pitch_angle * (3.14159265358979f / 180.0f);
                if (gimbal_controller.imu_online)
                {
                    const float pitch_imu_limit_max = 33.0f  * (3.14159265358979f / 180.0f);
                    const float pitch_imu_limit_min = -25.0f * (3.14159265358979f / 180.0f);
                    if (vision_pitch_rad > pitch_imu_limit_max) vision_pitch_rad = pitch_imu_limit_max;
                    if (vision_pitch_rad < pitch_imu_limit_min) vision_pitch_rad = pitch_imu_limit_min;
                }
                else
                {
                    const auto &cfg = joint_manager.pitch.getConfig();
                    if (!cfg.continuous)
                    {
                        if (vision_pitch_rad > cfg.limit_max) vision_pitch_rad = cfg.limit_max;
                        if (vision_pitch_rad < cfg.limit_min) vision_pitch_rad = cfg.limit_min;
                    }
                }
                Controller_Data.pitch.target_angle = vision_pitch_rad;
            }
        }
        else if (gimbal_deploy_requested)
        {
            auto &dr16 = BSP::Remote::DR16::Instance();

            // 左摇杆 Y 轴(ch3)，范围 [-1.0, 1.0]，向上为正
            float ch3 = (float)dr16.GetCh3();

        // 死区过滤：消除摇杆归中时的噪声(约 ±0.03~0.05)
        //   归中时强制速度为 0，保证"归中即停止"
        const float dead_zone  = 0.05f;
        if (ch3 > -dead_zone && ch3 < dead_zone)
        {
            ch3 = 0.0f;
        }

        // 速度映射：ch3 × max_speed → 目标角速度(rad/s)
        //   max_speed = 1.0 rad/s：满幅拨杆 1 秒转动 1 rad ≈ 57°
        //   慢速起点，调参稳定后可增大到 2~3 rad/s
        const float max_speed  = 2.8f;
        float target_velocity  = ch3 * max_speed;

        // 积分步长：GimbalUpdate 1kHz → dt = 0.001s
        const float dt          = 0.001f;

        // 仅当 pitch 关节在线时才积分(防止离线时 target 漂移)
        if (joint_manager.pitch.isOnline())
        {
            // 速度积分：target_angle += velocity × dt
            float new_target = Controller_Data.pitch.target_angle + target_velocity * dt;

            // 【限位钳位策略】
            //   IMU 闭环下：target 含义是枪口绝对俯仰(rad)，不在编码器坐标系
            //               → 用实测 IMU 限位（deg → rad）
            //   编码器闭环下：target 含义是电机相对 Fold 的角度(rad)
            //               → 用编码器限位（Joint.config.limit_min/max）
            //   imu_online 标志在 Step 3.5 设置，Step 2.6 还没执行到 Step 3.5
            //   → 用 gimbal_controller.imu_online 判断（上一周期状态，足够准确）
            if (gimbal_controller.imu_online)
            {
                // IMU 闭环：实测 IMU Pitch 限位（deg → rad）
                //   展开状态下实测值（2026-07-14）：
                //     上限：28 deg  → 0.576 rad（枪口抬起）
                //     下限：-25 deg → -0.506 rad（枪口压下）
                //   TODO: 根据 Fold 状态动态切换限位（Morphology Manager 阶段）
                const float pitch_imu_limit_max = 33.0f  * (3.14159265358979f / 180.0f);  
                const float pitch_imu_limit_min = -25.0f * (3.14159265358979f / 180.0f);  
                
                if (new_target > pitch_imu_limit_max) new_target = pitch_imu_limit_max;
                if (new_target < pitch_imu_limit_min) new_target = pitch_imu_limit_min;
            }
            else
            {
                // 编码器闭环：做编码器坐标系限位钳位
                //   pitch 范围：-0.238846481 ~ 0.599253953 rad ≈ -13.7° ~ 34.4°
                const auto &cfg = joint_manager.pitch.getConfig();
                if (!cfg.continuous)  // 非连续关节才限位
                {
                    if (new_target > cfg.limit_max) new_target = cfg.limit_max;
                    if (new_target < cfg.limit_min) new_target = cfg.limit_min;
                }
            }

            // 写入 Controller_Data(Step 3 会同步到 JointController.target_angle)
            Controller_Data.pitch.target_angle = new_target;
        }
        }  // end else (非急停 且 非TRANSITION) — Pitch
    }

    // ---------------------------------------------------------------
    // Step 2.7: Yaw轴控制（模式自适应：跟随模式速度环单环 vs 其他模式串级PID）
    //
    // 【核心改进】根治底盘跟随模式下的yaw轴反转现象
    //   - 跟随模式(S2==MIDDLE): 速度环单环控制（IMU速度反馈）
    //     * 摇杆直接映射到速度目标（不积分）
    //     * 松手→速度=0→电机自由停止，不抵抗底盘跟随
    //     * 根治反转：底盘超调反向修正时，yaw轴不抵抗
    //
    //   - 其他模式(S2==UP/DOWN): 串级PID（保持原有逻辑）
    //     * 摇杆速度积分到角度目标
    //     * IMU位置闭环，保持绝对航向
    //
    // 【数据流】
    //   DR16.ch2 → 死区过滤 → 模式判断
    //     ├─ 跟随模式: ch2*max_speed → 速度目标 → 速度环PID → 力矩
    //     └─ 其他模式: ch2*max_speed*dt → 角度积分 → 串级PID → 力矩
    //
    // 【平滑切换】
    //   模式切换时：
    //     跟随→其他: cascade_mode 0→1, target从IMU当前位置开始
    //     其他→跟随: cascade_mode 1→0, 速度从IMU当前速度开始
    //   清空PID状态，避免冲击
    //
    // 【调试观察点】
    //   FollowMode_Data.control_mode     : 0=串级PID, 1=速度环单环
    //   FollowMode_Data.target_velocity  : 速度环目标
    //   FollowMode_Data.imu_velocity     : IMU角速度反馈
    //   FollowMode_Data.follow_vel_kp/ki/kd: 跟随模式PID参数
    // ---------------------------------------------------------------
    {
        // 急停时跳过（电机已失能，不应改target）
        if (Remote_State.estop_active ||
            BSP::PLANNER::isTransitionState(
                static_cast<BSP::PLANNER::TransformState>(Transform_Status.state)))
        {
            // Skip this cycle.
        }
        else if (gimbal_deploy_requested && vision_active)
        {
            if (!vision_just_entered && joint_manager.yaw.isOnline())
            {
                float vision_yaw_rad = -(VisionComm_Data.yaw_angle - VisionComm_Data.yaw_offset_deg)
                                     * (3.14159265358979f / 180.0f);
                Controller_Data.yaw.target_angle = vision_yaw_rad;
            }
        }
        else if (gimbal_deploy_requested)
        {
            auto &dr16 = BSP::Remote::DR16::Instance();

            // ========== 1. 获取底盘模式 ==========
            auto chassis_mode = BoardComm::ChassisModeManager::Instance().GetCurrentState();
            bool is_follow_mode = (chassis_mode == BoardComm::ChassisMode::CHASSIS_FOLLOW);

            // ========== 2. 计算IMU角速度反馈 ==========
            float imu_yaw_vel = 0.0f;
            if (gimbal_controller.imu_online) {
                // IMU陀螺仪输出（deg/s → rad/s）
                // 注意：方向不取负（已确认）
                imu_yaw_vel = -BSP::IMU::imu.getGyroZ() * 0.0174532f;
                FollowMode_Data.imu_velocity = imu_yaw_vel;  // 写入Watch可观察
            }

            // ========== 3. 模式切换平滑处理 ==========
            static uint8_t last_follow_mode = 0;
            if (is_follow_mode != last_follow_mode) {
                if (is_follow_mode) {
                    // 进入跟随：串级→单级，速度从IMU当前速度开始
                    gimbal_controller.yaw.SwitchToVelocityMode(imu_yaw_vel);
                    FollowMode_Data.control_mode = 1;  // 标记速度环模式
                    // 同步到参数结构体，避免 Step 3 覆盖
                    Controller_Data.yaw.cascade_mode = 0;
                } else {
                    // 退出跟随：单级→串级，角度从IMU当前位置开始
                    gimbal_controller.yaw.SwitchToCascadeMode(gimbal_controller.yaw_imu_angle);
                    FollowMode_Data.control_mode = 0;  // 标记串级PID模式
                    // 【关键】同步到参数结构体，避免 Step 3 把 cascade_mode=0 同步回去
                    Controller_Data.yaw.cascade_mode = 1;
                    Controller_Data.yaw.target_angle = gimbal_controller.yaw_imu_angle;
                }
                last_follow_mode = is_follow_mode;
            }

            // ========== 4. 摇杆输入处理 ==========
            // 左摇杆 X 轴(ch2)，范围 [-1.0, 1.0]，向右为正
            float ch2 = (float)dr16.GetCh2();

            // 死区过滤：消除摇杆归中时的噪声(约 ±0.03~0.05)
            const float dead_zone = 0.05f;
            if (ch2 > -dead_zone && ch2 < dead_zone) {
                ch2 = 0.0f;
            }

            // 速度映射：ch2 × max_yaw_speed → 目标角速度(rad/s)
            const float max_yaw_speed = 2.8f;  // rad/s
            float target_velocity = ch2 * max_yaw_speed;

            // ========== 5. 控制计算（模式自适应） ==========
            // 仅当 yaw 关节在线时才控制
            if (joint_manager.yaw.isOnline())
            {
                if (is_follow_mode) {
                    // ===== 跟随模式：速度环单环 =====
                    // 写入Watch观察
                    FollowMode_Data.target_velocity = target_velocity;

                    // 速度环计算（IMU反馈）
                    Kpid_t follow_kpid = {
                        FollowMode_Data.follow_vel_kp,
                        FollowMode_Data.follow_vel_ki,
                        FollowMode_Data.follow_vel_kd
                    };
                    float torque = gimbal_controller.yaw.ComputeVelocity(
                        target_velocity,
                        imu_yaw_vel,
                        follow_kpid
                    );

                    // 输出到电机
                    dm4310_yaw_pitch->ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f,
                                                torque * joint_manager.yaw.getConfig().direction);

                } else {
                    // ===== 其他模式：串级PID（保持原有逻辑）=====
                    // 速度积分：target_angle += velocity × dt
                    const float dt = 0.001f;
                    Controller_Data.yaw.target_angle += target_velocity * dt;

                    // 注意：串级PID计算在 gimbal_controller.Update() 中完成
                    // 此处不调用，让后续 Step 3.5 → Step 4 流程统一处理
                }
            }
        }  // end else (非急停 且 非TRANSITION)
    }

    // ---------------------------------------------------------------
    // Step 2.7: 底盘模式状态机（板间通信 - 模式管理）【新增】
    //
    // 【需求】
    //   根据遥控器S2开关状态，决定底盘控制模式：
    //   - S2==MIDDLE → 跟随模式（底盘跟随云台朝向）
    //   - S2==UP     → 小陀螺模式（预留）
    //   - S2==DOWN   → 手动模式（wheel控制旋转）
    //   - S1+S2==DOWN → 急停（最高优先级）
    //   - 遥控器离线 → 强制急停
    //
    // 【架构】
    //   StateMachine（ChassisModeManager）← 状态判断 + 滤波 + 离线检测
    //     ↓ GetChassisMode()
    //   BoardComm::Update() ← 数据打包
    //     ↓ CAN2发送（0x205/0x206）
    //   底盘板接收 → RemoteControl::MapToChassis()
    //
    // 【状态滤波】
    //   连续10次（10ms）检测到相同状态才切换，防止开关抖动。
    //
    // 【调试观察点】Watch 中展开 ChassisModeDebug：
    //   current_state       : 当前模式(0-3)
    //   state_change_count  : 状态切换次数
    //   filter_reject_count : 滤波拒绝次数
    //   remote_online       : 遥控器在线状态
    // ---------------------------------------------------------------
    BoardComm::ChassisModeManager::Instance().Update();

    // ---------------------------------------------------------------
    // Step 3: Watch → GimbalController(在线调参 / 设目标 / 启停)
    //   注意：Planner 在 Step 2.5 已可能修改 pitch/fold.target_angle
    //         遥控器在 Step 2.6 已修改 pitch.target_angle
    //         此处 syncDataToController 会将最新 target 同步到 JointController
    //
    //   【关键】Yaw轴跟随模式时不同步参数！
    //     - 跟随模式（FollowMode_Data.control_mode=1）：
    //       Step 2.7已设置为速度环单环（cascade_mode=0）
    //       若此处同步会覆盖cascade_mode回串级（cascade_mode=1）
    //       导致Update()中仍执行串级PID → "转不动"
    //     - 其他模式（control_mode=0）：
    //       正常同步Yaw轴参数（串级PID）
    //
    //   【变形期间例外】TRANSITION 状态时必须同步 Yaw 参数
    //     - TransformPlanner 需要锁定 yaw 角度（cascade_mode=1）
    //     - 如果不同步，yaw 会保持速度环单环（cascade_mode=0）
    //     - GimbalController.Update() 会跳过 yaw 串级 PID → yaw 失控
    //     - 同时导致 yaw 离线检查失败 → 变形被 ABORT
    // ---------------------------------------------------------------
    bool is_transition = BSP::PLANNER::isTransitionState(
        static_cast<BSP::PLANNER::TransformState>(Transform_Status.state));

    Controller_Data_Unit_t yaw_sync = vision_requested ? Vision_Controller_Data.yaw : Controller_Data.yaw;
    yaw_sync.target_angle = Controller_Data.yaw.target_angle;
    yaw_sync.enabled = Controller_Data.yaw.enabled;

    Controller_Data_Unit_t pitch_sync = vision_requested ? Vision_Controller_Data.pitch : Controller_Data.pitch;
    pitch_sync.target_angle = Controller_Data.pitch.target_angle;
    pitch_sync.enabled = Controller_Data.pitch.enabled;

    // Yaw轴：非跟随模式 或 变形期间 都要同步
    if (FollowMode_Data.control_mode == 0 || is_transition) {
        syncDataToController(yaw_sync, gimbal_controller.yaw);
    }
    // Pitch/Fold：始终同步
    syncDataToController(pitch_sync, gimbal_controller.pitch);
    syncDataToController(Controller_Data.fold,  gimbal_controller.fold);

    // ---------------------------------------------------------------
    // Step 3.5: IMU 传感器反馈 → GimbalController（Yaw + Pitch IMU 闭环）
    //
    // 【原理】
    //   在 Update() 调用前，将 IMU 实时数据写入 GimbalController
    //   Update() 中 Yaw/Pitch 外环根据 imu_online 标志选择反馈源：
    //     IMU 在线 → IMU 反馈（传感器闭环）
    //     IMU 离线 → 编码器回退（安全降级）
    //
    // 【数据流】
    //   Yaw:
    //   BSP::IMU::imu.getAddYaw()  (deg, 连续累加, 支持 ±180° 跨越)
    //     ↓ × (-1)                 (IMU 方向与编码器相反: 向右转编码器+, IMU Euler_yaw-)
    //     ↓ × (π/180)             (deg → rad)
    //     ↓ → gimbal_controller.yaw_imu_angle
    //
    //   Pitch:
    //   BSP::IMU::imu.getPitch()   (deg, [-90°, 90°])
    //     ↓ × (π/180)             (deg → rad, 不取负，方向与编码器一致)
    //     ↓ → gimbal_controller.pitch_imu_angle
    //
    // 【方向映射说明】
    //   Yaw:   实测确认 IMU Euler_yaw 正方向与编码器方向相反
    //     编码器：向右转 → angle 增大
    //     IMU：   向右转 → Euler_yaw 减小
    //     → yaw_imu_angle = -addYaw × (π/180)
    //
    //   Pitch: 实测确认 IMU Pitch 正方向与编码器方向一致
    //     编码器：枪口抬起 → angle 增大
    //     IMU：   枪口抬起 → Pitch 增大
    //     → pitch_imu_angle = getPitch() × (π/180)
    //
    // 【Pitch IMU 闭环与 Fold 影响】
    //   IMU 安装在枪口端（Pitch 之后），IMU Pitch 直接测量枪口绝对俯仰
    //   不管 Fold 展开/收起，IMU Pitch 始终反映枪口真实俯仰
    //   → IMU 闭环下 target 含义一致：枪口绝对俯仰角，不受 Fold 影响
    //   → 编码器闭环下 target 含义随 Fold 变化
    //   机械限位仍依赖 Fold 状态，延后到 Morphology Manager 阶段处理
    //
    // 【离线保护】
    //   imu_online = 0 时，Update() 自动回退到编码器反馈
    //   不会疯车：编码器闭环保证基本功能
    //   测试方法：手动拔掉 IMU 串口线，观察 Yaw/Pitch 是否平稳切换
    //   Watch 观察：Controller_Data.yaw.feedback_source / Controller_Data.pitch.feedback_source
    //     1 = IMU 在线（传感器闭环）
    //     0 = 编码器回退（IMU 离线）
    // ---------------------------------------------------------------
    {
        gimbal_controller.imu_online = BSP::IMU::imu.isOnline() ? 1 : 0;

        if (gimbal_controller.imu_online)
        {
            // --- Yaw IMU 反馈 ---
            //   getAddYaw() 返回连续累加角度(deg)，已处理 ±180° 跳变
            //   取负：补偿 IMU 与编码器方向差异
            //   × (π/180)：deg → rad
            gimbal_controller.yaw_imu_angle =
                -BSP::IMU::imu.getAddYaw() * (3.14159265358979f / 180.0f);

            // --- Pitch IMU 反馈 ---
            //   getPitch() 返回欧拉角 Pitch(deg)，范围 [-90°, 90°]
            //   不取负：IMU Pitch 方向与编码器一致
            //   × (π/180)：deg → rad
            gimbal_controller.pitch_imu_angle =
                BSP::IMU::imu.getPitch() * (3.14159265358979f / 180.0f);
        }
        // IMU 离线时 yaw_imu_angle / pitch_imu_angle 不更新
        // （保持上一次值，但 Update 中不会使用）
    }

    // ---------------------------------------------------------------
    // Step 4: GimbalController.Update 控制循环
    //   读取 Joint.feedback / IMU → PID 计算 → Motor.ctrl_Mit(纯力矩)
    //   失能关节也发送零力矩，保持电机在线
    //   Yaw 外环：IMU 在线用 yaw_imu_angle，离线用编码器回退
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

    if (vision_requested)
    {
        syncControllerToData(gimbal_controller.yaw,   Vision_Controller_Data.yaw,   joint_manager.yaw);
        syncControllerToData(gimbal_controller.pitch, Vision_Controller_Data.pitch, joint_manager.pitch);
    }

    // Yaw + Pitch IMU 反馈源 → Watch（观察 feedback_source 判断当前闭环方式）
    syncImuToData(gimbal_controller, Controller_Data.yaw, Controller_Data.pitch);
    if (vision_requested)
    {
        syncImuToData(gimbal_controller, Vision_Controller_Data.yaw, Vision_Controller_Data.pitch);
    }

    // ---------------------------------------------------------------
    // Step 6: DR16 state → Watch（Stage04 遥控器数据同步）
    //   从 DR16.Instance() 读取状态 → 回写到 DR16_Data
    //   Watch 可观察：摇杆/开关/鼠标/键盘/拨轮/离线状态
    //
    //   命名规范：
    //     - ch0/ch1/ch2/ch3: 摇杆值（符合RoboMaster习惯）
    //     - s1/s2: 拨杆开关（S1=右开关, S2=左开关）
    // ---------------------------------------------------------------
    {
        auto &dr16 = BSP::Remote::DR16::Instance();

        // 摇杆状态（ch0~ch3）
        DR16_Data.ch0 = dr16.GetCh0();  // 右摇杆X轴
        DR16_Data.ch1 = dr16.GetCh1();  // 右摇杆Y轴
        DR16_Data.ch2 = dr16.GetCh2();  // 左摇杆X轴
        DR16_Data.ch3 = dr16.GetCh3();  // 左摇杆Y轴

        // 开关状态（S1/S2）
        DR16_Data.s1 = static_cast<uint8_t>(dr16.GetS1());  // S1: 左开关
        DR16_Data.s2 = static_cast<uint8_t>(dr16.GetS2());  // S2: 右开关

        // 鼠标状态
        DR16_Data.mouse_vel_x = dr16.GetMouseVelocity().x;
        DR16_Data.mouse_vel_y = dr16.GetMouseVelocity().y;
        DR16_Data.mouse_left  = dr16.GetMouse().left ? 1 : 0;
        DR16_Data.mouse_right = dr16.GetMouse().right ? 1 : 0;

        // 键盘按键状态
        auto kb = dr16.GetKeyboard();
        DR16_Data.key_w     = kb.w     ? 1 : 0;
        DR16_Data.key_s     = kb.s     ? 1 : 0;
        DR16_Data.key_a     = kb.a     ? 1 : 0;
        DR16_Data.key_d     = kb.d     ? 1 : 0;
        DR16_Data.key_shift = kb.shift ? 1 : 0;
        DR16_Data.key_ctrl  = kb.ctrl  ? 1 : 0;
        DR16_Data.key_q     = kb.q     ? 1 : 0;
        DR16_Data.key_e     = kb.e     ? 1 : 0;
        DR16_Data.key_r     = kb.r     ? 1 : 0;
        DR16_Data.key_f     = kb.f     ? 1 : 0;
        DR16_Data.key_g     = kb.g     ? 1 : 0;
        DR16_Data.key_z     = kb.z     ? 1 : 0;
        DR16_Data.key_x     = kb.x     ? 1 : 0;
        DR16_Data.key_c     = kb.c     ? 1 : 0;
        DR16_Data.key_v     = kb.v     ? 1 : 0;
        DR16_Data.key_b     = kb.b     ? 1 : 0;

        // 拨轮状态
        DR16_Data.wheel = dr16.GetWheel();

        // 离线状态（0=离线, 1=在线）
        DR16_Data.online = dr16.IsOffline() ? 0 : 1;
    }

    // ---------------------------------------------------------------
    // Step 6.5: IMU 姿态数据同步（Stage03 接入传感器）
    //   从 BSP::IMU::imu 读取解析后的姿态 → 回写到 IMU_Data
    //   Watch 可观察：欧拉角/角速度/加速度/四元数/累计Yaw/温度/在线状态
    //
    //   单位说明：
    //     本阶段保留 IMU 原始单位(deg / deg/s / g)，便于核对传感器输出
    //     后续接入控制环时再换算为 rad / rad/s
    //
    //   验证方法：
    //     ① Watch 添加 IMU_Data，观察 yaw/pitch/roll 是否随传感器姿态变化
    //     ② 旋转传感器，观察 yaw 单调变化、add_yaw 跨 ±180° 连续累加
    //     ③ 静止时 gyro 应接近 0，acc_z 应接近 1g(重力)
    //     ④ online 应为 1；拔掉串口线后 50ms 内变为 0
    // ---------------------------------------------------------------
    {
        IMU_Data.yaw         = BSP::IMU::imu.getYaw();       // 航向角(deg)
        IMU_Data.pitch       = BSP::IMU::imu.getPitch();     // 俯仰角(deg)
        IMU_Data.roll        = BSP::IMU::imu.getRoll();      // 横滚角(deg)

        IMU_Data.gyro_x      = BSP::IMU::imu.getGyroX();     // X轴角速度(deg/s)
        IMU_Data.gyro_y      = BSP::IMU::imu.getGyroY();     // Y轴角速度(deg/s)
        IMU_Data.gyro_z      = BSP::IMU::imu.getGyroZ();     // Z轴角速度(deg/s)

        IMU_Data.acc_x       = BSP::IMU::imu.getAccX();      // X轴加速度(g)
        IMU_Data.acc_y       = BSP::IMU::imu.getAccY();      // Y轴加速度(g)
        IMU_Data.acc_z       = BSP::IMU::imu.getAccZ();      // Z轴加速度(g)

        IMU_Data.quat_w      = BSP::IMU::imu.getQuat_w();    // 四元数 W
        IMU_Data.quat_x      = BSP::IMU::imu.getQuat_x();    // 四元数 X
        IMU_Data.quat_y      = BSP::IMU::imu.getQuat_y();    // 四元数 Y
        IMU_Data.quat_z      = BSP::IMU::imu.getQuat_z();    // 四元数 Z

        IMU_Data.add_yaw     = BSP::IMU::imu.getAddYaw();    // Yaw累计角度(deg)

        IMU_Data.temperature = BSP::IMU::imu.getTemperature(); // 温度(°C)
        IMU_Data.online      = BSP::IMU::imu.isOnline();     // 在线状态(0=离线, 1=在线)
    }

    // ---------------------------------------------------------------
    // Step 6.6: LK4005 电机数据同步（每周期执行）
    //   从 LK4005 实例读取反馈 → 回写到 LK4005_Data
    //   Watch 中展开 LK4005_Data 即可验证数据接收：
    //     ① online=1             反馈帧正常接收
    //     ② raw_cmd=0xA1         力矩控制反馈帧
    //     ③ angle 随手转动电机变化  编码器解析正确
    //     ④ velocity 随转速变化    速度解析正确
    //
    // 【验证步骤】
    //   1. Watch 添加 LK4005_Data，观察 online 是否变 1
    //   2. 手动转动 LK4005 输出轴，观察 angle / raw_angle 变化
    //   3. 快速转动，观察 velocity / raw_velocity 数值变化
    //   4. 若需查看电压/错误状态：调用 lk4005_motor->ReadStatus1(1) 后观察
    // ---------------------------------------------------------------
    if (BSP::MOTOR::LK::lk4005_motor != nullptr)
    {
        // SI 单位（输出端，来自 MotorBase::unit_data_）
        LK4005_Data.angle       = BSP::MOTOR::LK::lk4005_motor->getAngleRad(1);
        LK4005_Data.velocity    = BSP::MOTOR::LK::lk4005_motor->getVelocityRad(1);
        LK4005_Data.torque      = BSP::MOTOR::LK::lk4005_motor->getTorque(1);
        LK4005_Data.temperature = BSP::MOTOR::LK::lk4005_motor->getTemperature(1);

        // 原始反馈（调试用，便于核对协议解析）
        const auto &fb = BSP::MOTOR::LK::lk4005_motor->getFeedback(1);
        LK4005_Data.raw_angle    = fb.angle;
        LK4005_Data.raw_velocity = fb.velocity;
        LK4005_Data.raw_current  = fb.current;
        LK4005_Data.raw_cmd      = fb.cmd;

        // 状态1缓存（需主动调用 ReadStatus1(1) 后才有数据）
        const auto &st = BSP::MOTOR::LK::lk4005_motor->getStatus1(1);
        LK4005_Data.voltage       = st.voltage;
        LK4005_Data.error_state   = st.error_state;
        LK4005_Data.status1_valid = st.is_valid ? 1 : 0;

        // 在线状态（StateWatch 内部刷新）
        LK4005_Data.online = BSP::MOTOR::LK::lk4005_motor->isConnected(1) ? 1 : 0;
    }

    // ---------------------------------------------------------------
    // Step 6.7: GM3508 摩擦轮数据同步（每周期执行）
    //   从 motor_3508 读取反馈 → 回写到 Friction_Data
    //   Watch 中展开 Friction_Data → left/right 即可验证数据接收：
    //     验证方法：
    //       1. Watch 添加 Friction_Data，观察 left.online / right.online 是否都 = 1
    //       2. 若 online=1，再看 velocity_rpm 是否随电机转动变化
    //       3. 若 online=0 持续不变，检查 CAN2 线束和 motor_id 设置
    //     调试计数：
    //       同时观察 CanCallback.cpp 中 can2_rx_id_0x201 / can2_rx_id_0x202
    //       应持续增长。
    // ---------------------------------------------------------------
    if (BSP::MOTOR::DJI::motor_3508 != nullptr)
    {
        auto &m = *BSP::MOTOR::DJI::motor_3508;

        // motor_id = 1（左摩擦轮）
        Friction_Data.left.angle_rad      = m.getAngleRad(1);
        Friction_Data.left.velocity_rpm   = m.getVelocityRpm(1);
        Friction_Data.left.velocity_radps = m.getVelocityRad(1);
        Friction_Data.left.torque_nm      = m.getTorque(1);
        Friction_Data.left.temperature    = m.getTemperature(1);
        Friction_Data.left.online         = m.isConnected(1) ? 1 : 0;

        // motor_id = 2（右摩擦轮）
        Friction_Data.right.angle_rad      = m.getAngleRad(2);
        Friction_Data.right.velocity_rpm   = m.getVelocityRpm(2);
        Friction_Data.right.velocity_radps = m.getVelocityRad(2);
        Friction_Data.right.torque_nm      = m.getTorque(2);
        Friction_Data.right.temperature    = m.getTemperature(2);
        Friction_Data.right.online         = m.isConnected(2) ? 1 : 0;
    }

    // ---------------------------------------------------------------
    // Step 7: VOFA+ 波形发送（降频到 500Hz）
    //   调用 Variable.cpp 中的 VofaSendDebugChannels()
    //   修改通道配置：只需改 Variable.cpp，无需改此文件
    // ---------------------------------------------------------------
    if (true)  // Temporary: always send RCIA vision frame.
    {
        static uint8_t vision_counter = 0;
        vision_counter++;
        if (vision_counter >= 5)  // 1000Hz / 5 = 200Hz
        {
            vision_counter = 0;
            VisionComm::Manager::Instance().Send();
        }
    }
    else
    {
        static uint8_t vofa_counter = 0;
        vofa_counter++;
        if (vofa_counter >= 2)  // 1000Hz / 2 = 500Hz
        {
            vofa_counter = 0;
            VofaSendDebugChannels();
        }
    }

    // ---------------------------------------------------------------
    // Step 8: 板间通信更新（云台→底盘，4ms 周期）
    //   更新发送数据：遥控器通道 + 云台角度
    //   发送 CAN 帧：0x205/0x206
    //   接收处理：0x207/0x208（在 CanCallback.cpp 中断中处理）
    //
    // 【数据流】
    //   DR16 遥控器右摇杆 → BoardComm::Update() → direction.LX/LY
    //   Joint_Data.yaw → CalcuGimbalToChassisAngle() → direction.Yaw_encoder_angle_err
    //   direction/chassis_mode/ui_list → Data_send() → CAN2 发送
    //
    // 【调试观察点】
    //   Watch 添加 BoardComm_Data：
    //     - LX/LY：遥控器右摇杆（0-220，中值110）
    //     - Yaw_encoder_angle_err：云台-底盘角度误差
    //     - launch_speed：发射速度（底盘返回）
    //     - booster_now_heat：当前热量（底盘返回）
    // ---------------------------------------------------------------
    {
        static auto &board_comm = BoardComm::Gimbal_to_Chassis::Instance();

        // 更新发送数据（读取遥控器 + 云台角度）
        board_comm.Update();

        // 发送 CAN 帧（0x205/0x206）
        board_comm.Data_send();
    }
}
