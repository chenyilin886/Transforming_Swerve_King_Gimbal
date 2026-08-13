/**
 * @file ShootFSM.cpp
 * @brief 发射机构状态机（Class_ShootFSM）实现
 *
 * 实现 ShootFSM.hpp 中声明的 Class_ShootFSM 类方法。
 *
 * 调用链：
 *   shootTask (4ms)
 *     └─→ Class_ShootFSM::Control()
 *           ├─→ updateStateMachine_()        // 安全检查 + 状态切换
 *           ├─→ applyStateToDialConfig_()    // state → Dial_Config.enabled
 *           ├─→ dial_ctrl.Update(...)        // 委托拨盘双环控制
 *           ├─→ updateFriction_()            // 预留: 摩擦轮控制(空)
 *           └─→ syncStatus_()                // 回写 Shoot_Status
 *
 * 依赖：
 *   - Dial_Config / Dial_Status  (Variable.cpp 定义)
 *   - Shoot_Config / Shoot_Status(Variable.cpp 定义)
 *   - lk4005_motor 指针          (GimbalInit.cpp 定义)
 *   - DR16::Instance()           (单例)
 */
#include "ShootFSM.hpp"

// ============================================================
// 外部依赖声明
// ============================================================
// lk4005_motor 指针在 BSP::MOTOR::LK 命名空间内
// 在 LkMotor.hpp 中 extern 声明，在 GimbalInit.cpp 中定义并赋值
// (GimbalInit 创建 LK4005 实例后 BSP::MOTOR::LK::lk4005_motor = &lk4005)
// 注意：必须带命名空间限定，否则符号找不到（链接错误 L6218E）
using BSP::MOTOR::LK::lk4005_motor;

namespace BSP::FSM
{

// ========================================================================
// Control() - 周期更新入口
// ========================================================================
void Class_ShootFSM::Control()
{
    // ================================================================
    // Step 1: 安全条件检查 + 状态机切换
    // ================================================================
    updateStateMachine_();

    // ================================================================
    // Step 2: 把 state 映射到 Dial_Config.enabled（联动拨盘）
    // ================================================================
    applyStateToDialConfig_();

    // ================================================================
    // Step 3: 委托拨盘控制给 DialController
    // ================================================================
    // DialController 内部会处理：
    //   - 电机指针/离线检查
    //   - 单击/长按状态机
    //   - 双环 PID
    //   - 卡弹检测
    //   - 发送 ctrl_Torque + 回写 Dial_Status
    auto &dr16 = BSP::Remote::DR16::Instance();
    dial_ctrl.Update(lk4005_motor,
                     dr16,
                     Dial_Config,
                     Dial_Status);

    // ================================================================
    // Step 4: 摩擦轮速度环 PID 控制（S1上 + S2上 使能）
    // ================================================================
    //   - 读 GM3508 反馈速度
    //   - 位置式 PID 计算电流命令
    //   - ctrl_Current + sendCAN 发送
    updateFriction_();

    // ================================================================
    // Step 5: 回写 Shoot_Status 供 Watch 观察
    // ================================================================
    syncStatus_();
}

// ========================================================================
// updateStateMachine_() - 安全检查 + 状态机切换
// ========================================================================
void Class_ShootFSM::updateStateMachine_()
{
    auto &dr16 = BSP::Remote::DR16::Instance();

    // --- 安全条件判定 ---
    using Switch = BSP::Remote::DR16::Switch;
    const bool remote_offline = dr16.IsOffline();
    const bool remote_estop   =
        (dr16.GetS1() == Switch::DOWN && dr16.GetS2() == Switch::DOWN);
    const bool feature_ok = (Shoot_Config.feature_enable != 0);
    const bool shoot_en   = (Shoot_Config.shoot_enabled  != 0);

    last_safety_ok = safety_ok;
    safety_ok = (!remote_offline && !remote_estop && feature_ok && shoot_en) ? 1 : 0;

    // --- 状态机切换 ---
    switch (state)
    {
        case ShootState::DISABLE:
            // safety_ok 上沿: DISABLE → AUTO
            if (safety_ok)
            {
                state = ShootState::AUTO;
            }
            break;

        case ShootState::STOP:
            // 预留状态，当前不主动进入
            // 如果意外进入，safety_ok 则回 AUTO，否则回 DISABLE
            if (!safety_ok)
            {
                state = ShootState::DISABLE;
            }
            else
            {
                state = ShootState::AUTO;
            }
            break;

        case ShootState::AUTO:
            // !safety_ok: AUTO → DISABLE
            if (!safety_ok)
            {
                state = ShootState::DISABLE;
            }
            // 否则保持 AUTO
            break;
    }

    // --- 时间戳更新（用于未来扩展 dt 相关逻辑）---
    last_update_ms = HAL_GetTick();
}

// ========================================================================
// applyStateToDialConfig_() - state → Dial_Config.enabled 映射
// ========================================================================
void Class_ShootFSM::applyStateToDialConfig_()
{
    // 映射规则（见 ShootFSM.hpp 注释）：
    //   DISABLE → Dial_Config.enabled = 0（拨盘零力矩）
    //   STOP    → Dial_Config.enabled = 1（拨盘待命）
    //   AUTO    → Dial_Config.enabled = 1（拨盘可触发）
    //
    // 注意：friction_enable 不由此函数控制
    //       由 updateFriction_() 读右拨杆独立控制
    switch (state)
    {
        case ShootState::DISABLE:
            Dial_Config.enabled = 0;
            break;

        case ShootState::STOP:
            Dial_Config.enabled = 1;
            break;

        case ShootState::AUTO:
            Dial_Config.enabled = 1;
            break;
    }
}

// ========================================================================
// updateFriction_() - 摩擦轮速度环 PID 控制
// ========================================================================
void Class_ShootFSM::updateFriction_()
{
    // ==================================================================
    // 获取 GM3508 全局指针（GimbalInit.cpp 中创建并赋值）
    // ==================================================================
    auto *m = BSP::MOTOR::DJI::motor_3508;
    if (m == nullptr)
    {
        // 电机实例未创建，直接返回（不发送任何 CAN 帧）
        return;
    }

    // ==================================================================
    // 安全条件判定：摩擦轮使能独立于 ShootFSM 状态机
    // ==================================================================
    // 使能条件（全部满足）：
    //   1. S1上 + S2上
    //   2. 遥控器在线（!IsOffline()）
    //   3. 无急停（!(S1==DOWN && S2==DOWN)）
    //
    // 注：条件 2 和 3 是冗余保护
    //   - 离线时 GetS2() 返回 UNKNOWN（非 UP），条件 1 已覆盖
    //   - 急停时 S2==DOWN（非 UP），条件 1 已覆盖
    //   但显式写出更清晰，便于未来修改使能逻辑
    auto &dr16 = BSP::Remote::DR16::Instance();
    using Switch = BSP::Remote::DR16::Switch;

    const bool friction_switch = (dr16.GetS1() == Switch::UP &&
                                  dr16.GetS2() == Switch::UP);
    const bool remote_offline  = dr16.IsOffline();
    const bool remote_estop    = (dr16.GetS1() == Switch::DOWN &&
                                  dr16.GetS2() == Switch::DOWN);

    friction_enable = (friction_switch && !remote_offline && !remote_estop) ? 1 : 0;

    // ==================================================================
    // 摩擦轮控制主逻辑
    // ==================================================================
    if (remote_offline || remote_estop)
    {
        // --- 紧急停止：直接发 0 电流 + 清 PID（摩擦轮自由滑停）---
        m->ctrl_Current(1, 0);
        m->ctrl_Current(2, 0);
        pid_friction_l.clearPID();
        pid_friction_r.clearPID();
    }
    else
    {
        // --- 正常/停止：始终跑 PID ---
        //   S1上+S2上: target = friction_target_rpm（Watch 设定值）
        //   其他挡位: target = 0（PID 主动刹停，不自由滑停）
        float target   = friction_enable ? Shoot_Config.friction_target_rpm : 0.0f;
        float target_l = -target;
        float target_r = +target;

        // --- 1. 读 GM3508 反馈速度（电机端 RPM）---
        float vel_l = m->getVelocityRpm(1);
        float vel_r = m->getVelocityRpm(2);

        // --- 2. PID 参数从 Shoot_Config 同步（Watch 在线调参）---
        kpid_friction.kp = Shoot_Config.friction_kp;
        kpid_friction.ki = Shoot_Config.friction_ki;
        kpid_friction.kd = Shoot_Config.friction_kd;

        pid_friction_l.pid.Break_I = Shoot_Config.friction_break_i;
        pid_friction_l.pid.MixI    = Shoot_Config.friction_limit_i;
        pid_friction_r.pid.Break_I = Shoot_Config.friction_break_i;
        pid_friction_r.pid.MixI    = Shoot_Config.friction_limit_i;

        // --- 3. 位置式 PID 计算（限幅 16384）---
        pid_friction_l.GetPidPos(kpid_friction, target_l, vel_l, 16384.0f);
        pid_friction_r.GetPidPos(kpid_friction, target_r, vel_r, 16384.0f);

        // --- 4. 发送电流命令 ---
        m->ctrl_Current(1, (int16_t)pid_friction_l.pid.cout);
        m->ctrl_Current(2, (int16_t)pid_friction_r.pid.cout);
    }

    // ==================================================================
    // 6. 统一发送 CAN 帧（无论使能与否都要发送）
    // ==================================================================
    //   DJI 电机控制帧固定为 CAN ID=0x200，一帧带 4 台电机电流
    //   每周期必须发送，否则电机控制器超时判离线
    m->sendCAN();

    // ==================================================================
    // 状态回写说明：
    //   Friction_Data 已在 GimbalUpdate() 中每周期从 motor_3508 同步
    //   （left/right 的 velocity_rpm, online, torque_nm 等）
    //   此处不再重复回写 Shoot_Status.friction_vel_l/r
    // ==================================================================
}

// ========================================================================
// syncStatus_() - 回写 Shoot_Status 供 Watch 观察
// ========================================================================
void Class_ShootFSM::syncStatus_()
{
    // Dial_Status 已由 DialController.Update() 内部回写，无需重复
    // 这里只回写 ShootFSM 自身的状态字段
    Shoot_Status.state          = (uint8_t)state;
    Shoot_Status.safety_ok      = safety_ok;
    Shoot_Status.friction_enable = friction_enable;
}

} // namespace BSP::FSM
