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
#include "Joint.hpp"
#include "Controller.hpp"
#include "TransformPlanner.hpp"
#include "DR16.hpp"
#include "HI12H3_IMU.hpp"

// ========================================================================
// 全局电机指针
// ========================================================================
namespace BSP::MOTOR::DM
{
DM4310* dm4310_yaw_pitch = nullptr;
DM4340* dm4340_fold       = nullptr;
}

namespace BSP::MOTOR::LK
{
// LK4005 全局指针（定义在 LkMotor.hpp 中 extern 声明）
//   ID=1, CAN1, 用于 Pitch 关节力矩控制（数据接收验证阶段）
//   使用前必须：GimbalInit 中创建实例 + On(1) 使能 + GimbalUpdate 周期性 ctrl_Torque
LK4005* lk4005_motor = nullptr;
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

    // 2. 创建电机实例(static 局部，生命周期 = 程序整个运行期)
    static BSP::MOTOR::DM::DM4310 dm4310(&can1);
    static BSP::MOTOR::DM::DM4340 dm4340(&can1);

    // LK4005 电机实例（ID=1, 接 CAN1, 用于数据接收验证）
    //   注意：LK 协议要求使能后周期性发送控制指令才能维持反馈上报
    //   GimbalUpdate 中每周期调用 ctrl_Torque(1, 0) 维持反馈
    static BSP::MOTOR::LK::LK4005 lk4005(&can1, 1);

    // 3. 赋值全局指针
    BSP::MOTOR::DM::dm4310_yaw_pitch = &dm4310;
    BSP::MOTOR::DM::dm4340_fold       = &dm4340;
    BSP::MOTOR::LK::lk4005_motor      = &lk4005;

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

    can1.register_rx_callback(
        [](const HAL::CAN::Frame &frame)
        {
            lk4005.Parse(frame);
        });

    // 5. 初始化 JointManager
    joint_manager.Init();

    // 6. 初始化 GimbalController(Stage03 默认 pitch.Enable，yaw/fold Disable)
    gimbal_controller.Init();

    // 7. 初始化 DR16 遥控器(Stage04 UART3 DMA接收)
    BSP::Remote::DR16::Instance().Init();

    // 8. 初始化 IMU(Stage03 接入传感器, USART1 DMA空闲中断接收)
    //    HI12H3: 256000bps, 200Hz, 82字节固定帧
    //    Init() 启动 DMA 接收，首帧到达后由 HAL_UARTEx_RxEventCallback 解析
    BSP::IMU::imu.Init();

    // 9. 使能电机
    HAL_Delay(500);

    dm4310.On(1);
    HAL_Delay(10);

    dm4310.On(2);
    HAL_Delay(10);

    dm4340.On(1);
    HAL_Delay(10);

    // 10. 使能 LK4005（发送 0x88 使能命令后电机开始上报反馈）
    //     注意：使能后需 GimbalUpdate 周期性发送 ctrl_Torque(1, 0) 维持反馈
    //     首次发送零力矩控制指令，触发首帧反馈上报
    lk4005.On(1);
    HAL_Delay(10);
    lk4005.ctrl_Torque(1, 0);
    HAL_Delay(10);
}


// ========================================================================
// 周期更新
// ========================================================================
void GimbalUpdate()
{
    using namespace BSP::MOTOR::DM;

    // ---------------------------------------------------------------
    // Step 0: DR16 遥控器离线检测（每周期执行）
    //   检查是否超过50ms未收到遥控器数据
    //   离线时自动归零摇杆/开关/鼠标/键盘状态
    // ---------------------------------------------------------------
    BSP::Remote::DR16::Instance().IsOffline();

    // ---------------------------------------------------------------
    // Step 0.5: IMU 离线检测（每周期执行）
    //   HI12H3 输出 200Hz，超过 50ms(10帧)未收到数据判离线
    //   离线时自动清除 ORE 错误，防止 UART 死锁
    //   本阶段(正确解析传感器数据)仅检测，不影响控制环
    // ---------------------------------------------------------------
    BSP::IMU::imu.IsOffline();

    // ---------------------------------------------------------------
    // Step 0.6: LK4005 周期性控制指令（每周期执行，1kHz）
    //
    // 【协议要求】
    //   LK-TECH 电机使能后需周期性发送控制指令(0xA1 力矩控制)才能维持反馈上报。
    //   若超过 ~200ms 未发送任何控制指令，电机将停止上报反馈帧。
    //
    // 【当前阶段：数据接收验证】
    //   - 每周期发送零力矩 ctrl_Torque(1, 0)：维持反馈上报，不输出力矩
    //   - 后续接入 Pitch 关节控制时，替换为 PID 输出的力矩值
    //
    // 【数据流】
    //   ctrl_Torque(1, 0) → CAN1 发送 → 电机回复反馈帧
    //     → LK4005::Parse → Configure → unit_data_[0]
    //     → Step 6.6 同步到 LK4005_Data → Watch 观察
    // ---------------------------------------------------------------
    if (BSP::MOTOR::LK::lk4005_motor != nullptr)
    {
        BSP::MOTOR::LK::lk4005_motor->ctrl_Torque(1, 0);
    }

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
    // Step 2.55: 遥控器状态机（急停 + S1 边沿检测 → Planner 命令）
    //
    // 【需求】
    //   - S1==DOWN(2) && S2==DOWN(2) → 急停：所有电机失能防止疯转
    //   - S1==UP(1)    → 收起（边沿触发，发一次 CONTRACT）
    //   - S1==MIDDLE(3) → 展开（边沿触发，发一次 EXPAND）
    //
    // 【急停状态机】
    //   进入急停(0→1)：
    //     ① 保存 Controller_Data.{yaw,pitch,fold}.enabled → saved_*_en
    //     ② 置 enabled = 0（Step 3 syncDataToController 会同步到 ctrl.Disable）
    //     ③ 发 ABORT 给 Planner（中止动作 + snap target 到 feedback）
    //   急停保持：不做事（电机已失能，Planner 已 ABORT）
    //   退出急停(1→0)：S1 或 S2 任一离开 DOWN 即退出
    //     ① 恢复 enabled 从 saved_*_en
    //     ② 发 RESET 给 Planner（从 ABORT 回到 IDLE）
    //     ③ 跳过本周期 S1 边沿检测（防止 S1 从 DOWN 变化误触发）
    //
    // 【S1 边沿检测】（仅非急停时）
    //   检测 S1 档位变化：
    //     其他 → UP(1)     → cmd = CONTRACT
    //     其他 → MIDDLE(3)  → cmd = EXPAND
    //   保持在同一档位不重复发（Planner cmd 是单次触发型）
    //
    // 【DR16 离线行为】
    //   离线时 s1/s2 = UNKNOWN(0)，不满足 DOWN&&DOWN，不触发急停
    //   符合"离线时保持当前位置"的安全策略
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
                    if (cur_s1 == static_cast<uint8_t>(Switch::UP))
                    {
                        // S1 → 上档(1)：收起命令
                        Transform_Config.cmd = static_cast<uint8_t>(
                            BSP::PLANNER::TransformCmd::CONTRACT);
                        Remote_State.planner_cmd_sent = static_cast<uint8_t>(
                            BSP::PLANNER::TransformCmd::CONTRACT);
                    }
                    else if (cur_s1 == static_cast<uint8_t>(Switch::MIDDLE))
                    {
                        // S1 → 中档(3)：展开命令
                        Transform_Config.cmd = static_cast<uint8_t>(
                            BSP::PLANNER::TransformCmd::EXPAND);
                        Remote_State.planner_cmd_sent = static_cast<uint8_t>(
                            BSP::PLANNER::TransformCmd::EXPAND);
                    }
                    // S1 → DOWN(2) 或 UNKNOWN(0)：不发命令
                    //   S1=DOWN 单独不发命令（急停需要 S2 同时 DOWN）
                    Remote_State.last_s1 = cur_s1;
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
            // 跳过本周期摇杆积分
        }
        else
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
                //     上限：33 deg  → 0.576 rad（枪口抬起）
                //     下限：-29 deg → -0.506 rad（枪口压下）
                //   TODO: 根据 Fold 状态动态切换限位（Morphology Manager 阶段）
                const float pitch_imu_limit_max = 33.0f  * (3.14159265358979f / 180.0f);  // ≈ 0.576 rad
                const float pitch_imu_limit_min = -29.0f * (3.14159265358979f / 180.0f);  // ≈ -0.506 rad
                
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
    // Step 2.7: 遥控器左摇杆X轴 → Yaw 目标角度(速度积分模式)
    //
    // 【原理】
    //   与 Step 2.6 Pitch 相同的速度积分模式：
    //   ch2 × max_speed → 角速度 → 积分到 target_angle
    //   归中时 v=0 → target 保持 → 停止运动
    //
    // 【关键区别】Yaw 是连续旋转关节：
    //   - 不做限位钳位（可无限圈旋转）
    //   - Controller.hpp Compute() 中 continuous=1，已做 wrapToPi 最短路径处理
    //   - target_angle 可以超出 [-π, π]（PID 内部自动归一化误差）
    //   - 持续拨动摇杆，Yaw 会持续旋转多圈
    //
    // 【数据流】
    //   DR16.ch2 (-1~1)
    //     ↓ 死区过滤(±0.05)
    //     ↓ × max_yaw_speed (3.0 rad/s)  → target_velocity (rad/s)
    //     ↓ × dt (0.001s, 1kHz)          → delta_angle (rad)
    //     ↓ 累加到 Controller_Data.yaw.target_angle
    //     ↓ 无限位钳位（连续旋转关节）
    //     ↓ Step 3 syncDataToController → PID.target_angle
    //     ↓ Step 4 串级 PID → Motor.ctrl_Mit
    //
    // 【方向映射】
    //   ch2 > 0 (摇杆向右) → target_angle 增大 → Yaw 顺时针旋转
    //   ch2 < 0 (摇杆向左) → target_angle 减小 → Yaw 逆时针旋转
    //
    // 【安全性】
    //   - 急停时跳过积分（电机已失能）
    //   - Planner TRANSITION 时跳过（让 Planner 控制）
    //   - DR16 离线时 ch2=0 → target 保持 → PID 维持当前位置
    //   - Yaw 连续旋转：Compute() 中 wrapToPi 保证误差始终走最短路径
    //     即使 target 累积到很大值，PID 误差仍在 [-π, π]，不会疯车
    //
    // 【调试观察点】
    //   - DR16_Data.ch2                        左摇杆X轴原始值
    //   - Controller_Data.yaw.target_angle      积分后目标（可超 [-π,π]）
    //   - Controller_Data.yaw.feedback_angle    归一化反馈（[-π,π]）
    //   - Controller_Data.yaw.error             最短路径误差（应在 [-π,π]）
    //   - Controller_Data.yaw.torque_output     输出力矩
    // ---------------------------------------------------------------
    {
        // 急停时跳过摇杆积分（电机已失能，不应改 target）
        // Planner TRANSITION 时跳过（让 Planner 控制 target）
        if (Remote_State.estop_active ||
            BSP::PLANNER::isTransitionState(
                static_cast<BSP::PLANNER::TransformState>(Transform_Status.state)))
        {
            // 跳过本周期摇杆积分
        }
        else
        {
            auto &dr16 = BSP::Remote::DR16::Instance();

            // 左摇杆 X 轴(ch2)，范围 [-1.0, 1.0]，向右为正
            float ch2 = (float)dr16.GetCh2();

            // 死区过滤：消除摇杆归中时的噪声(约 ±0.03~0.05)
            //   归中时强制速度为 0，保证"归中即停止"
            const float dead_zone = 0.05f;
            if (ch2 > -dead_zone && ch2 < dead_zone)
            {
                ch2 = 0.0f;
            }

            // 速度映射：ch2 × max_yaw_speed → 目标角速度(rad/s)
            //   max_yaw_speed = 5.0 rad/s：满幅拨杆 1 秒转动 5 rad ≈ 29°
            //   Yaw 连续旋转，速度可比 Pitch 更快
            const float max_yaw_speed = 3.0f;
            float target_velocity     = ch2 * max_yaw_speed;

            // 积分步长：GimbalUpdate 1kHz → dt = 0.001s
            const float dt = 0.001f;

            // 仅当 yaw 关节在线时才积分(防止离线时 target 漂移)
            if (joint_manager.yaw.isOnline())
            {
                // 速度积分：target_angle += velocity × dt
                //   Yaw 是连续旋转关节，不做限位钳位
                //   target 可无限累积（PID 内部 wrapToPi 处理最短路径误差）
                Controller_Data.yaw.target_angle += target_velocity * dt;
            }
        }  // end else (非急停 且 非TRANSITION) — Yaw
    }

    // ---------------------------------------------------------------
    // Step 3: Watch → GimbalController(在线调参 / 设目标 / 启停)
    //   注意：Planner 在 Step 2.5 已可能修改 pitch/fold.target_angle
    //         遥控器在 Step 2.6 已修改 pitch.target_angle
    //         此处 syncDataToController 会将最新 target 同步到 JointController
    // ---------------------------------------------------------------
    syncDataToController(Controller_Data.yaw,   gimbal_controller.yaw);
    syncDataToController(Controller_Data.pitch, gimbal_controller.pitch);
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

    // Yaw + Pitch IMU 反馈源 → Watch（观察 feedback_source 判断当前闭环方式）
    syncImuToData(gimbal_controller, Controller_Data.yaw, Controller_Data.pitch);

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
    // Step 7: VOFA+ 波形发送（降频到 500Hz）
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
