/**
 * @file ShootFSM.hpp
 * @brief 发射机构状态机（Class_ShootFSM）
 *
 * 设计来源：
 *   参考 H_SG_Gimbal 参考工程 ShootTask.hpp 的 Class_ShootFSM。
 *   本工程为三关节可变形云台的发射机构统一状态机，管理：
 *     - 拨盘子系统（LK4005 双环 PID + 单击/长按 + 卡弹检测）
 *     - 摩擦轮子系统（预留，DJI 3508 速度环，暂未接入）
 *     - 整体状态机（DISABLE/STOP/STANDBY/AUTO）
 *
 * 架构位置：
 *   Application(shootTask) → StateMachine(Class_ShootFSM) → Controller(DialController) → Motor(LK4005)
 *
 * 与 DialController 的关系：
 *   - DialController 内部已有自己的拨盘状态机（DISABLE/STOP/SINGLE/AUTO）
 *   - Class_ShootFSM 是更上层的发射机构状态机，管"摩擦轮开关 + 拨盘 enable"
 *   - Class_ShootFSM 不重写 DialController 的状态机，只通过 Dial_Config.enabled 控制
 *   - 调用流程：ShootFSM.Control() → 设置 Dial_Config.enabled → DialController.Update()
 *
 * 状态机设计：
 *   ┌─────────────────────────────────────────────────────┐
 *   │                                                     │
 *   │   ┌─────────┐  safety_ok & shoot_enabled=1         │
 *   │   │DISABLE  │──────────────────────────┐           │
 *   │   │         │◀─────────────────────────┐│           │
 *   │   └─────────┘                          ││           │
 *   │        │                               ││           │
 *   │        │ 默认直接进 AUTO               ││           │
 *   │        ▼                               ││           │
 *   │   ┌─────────┐  !safety_ok              ││           │
 *   │   │  AUTO   │──────────────────────────┘│           │
 *   │   │         │                           │           │
 *   │   │ 拨盘 enabled=1                     │           │
 *   │   │ 摩擦轮由右拨杆独立控制              │           │
 *   │   └─────────┘                                       │
 *   │                                                     │
 *   │   ┌─────────┐                                       │
 *   │   │  STOP   │ (预留，当前不进入)                    │
 *   │   │拨盘en=1 │                                       │
 *   │   └─────────┘                                       │
 *   │                                                     │
 *   └─────────────────────────────────────────────────────┘
 *
 *   状态说明：
 *     DISABLE : 完全失能，拨盘零力矩，摩擦轮停
 *     STOP    : 拨盘待命(enabled=1 但无 wheel 触发)，摩擦轮由右拨杆控制
 *     AUTO    : 拨盘可触发(enabled=1)，摩擦轮由右拨杆控制
 *
 *   摩擦轮使能独立于状态机：
 *     - 右拨杆 UP 且无急停/离线 → friction_enable=1
 *     - 否则 friction_enable=0（直接发 0 电流 + 清 PID）
 *
 * 与参考工程差异：
 *   - 参考工程用 S1/S2 开关切换 ONLY/AUTO/STOP
 *   - 本工程当前简化：safety_ok 即进 AUTO，摩擦轮由右拨杆独立控制
 *
 * @note 本类不直接访问 Motor/Encoder，统一通过 DialController 间接访问
 */
#ifndef SHOOT_FSM_HPP
#define SHOOT_FSM_HPP

#include "DialController.hpp"
#include "LkMotor.hpp"
#include "DjiMotor.hpp"
#include "PID.hpp"
#include "DR16.hpp"
#include "Variable.hpp"

namespace BSP::FSM
{

// ========================================================================
// 发射机构状态枚举
// ========================================================================
/**
 * @brief 发射机构整体状态
 *
 * 状态转移：
 *   DISABLE ──(safety_ok & shoot_enabled)──→ AUTO
 *   AUTO    ──(!safety_ok / !shoot_enabled)──→ DISABLE
 *   STOP    ──(预留)──→ AUTO
 *
 * @note 摩擦轮使能独立于此状态机，由右拨杆 UP 控制（见 updateFriction_）
 */
enum class ShootState : uint8_t
{
    DISABLE = 0,   ///< 失能：拨盘零力矩，摩擦轮停
    STOP    = 1,   ///< 停止：拨盘待命(可被 wheel 触发)，当前不主动进入
    AUTO    = 3,   ///< 发射：拨盘可触发，摩擦轮由右拨杆控制
};


// ========================================================================
// Class_ShootFSM - 发射机构状态机
// ========================================================================
/**
 * @class Class_ShootFSM
 * @brief 发射机构顶层状态机
 *
 * 职责：
 *   1. 整体状态机管理（DISABLE/STOP/STANDBY/AUTO）
 *   2. 安全条件检查（遥控器离线 / 急停 / feature_enable）
 *   3. 委托拨盘控制给 DialController（通过 Dial_Config.enabled 联动）
 *   4. 预留摩擦轮控制接口（当前空实现）
 *   5. 回写 Shoot_Status 供 Watch 观察
 *
 * 与 DialController 的协作：
 *   Class_ShootFSM::Control()
 *     ├── updateStateMachine_()        // 决定本周期 state
 *     ├── applyStateToDialConfig_()    // 把 state 映射到 Dial_Config.enabled
 *     ├── dial_ctrl.Update(...)        // 委托拨盘双环控制
 *     ├── updateFriction_()            // 预留: 摩擦轮控制(空)
 *     └── syncStatus_()                // 回写 Shoot_Status
 *
 * 调用方式：
 *   shootTask 中每 4ms 调用一次：
 *     static Class_ShootFSM shoot_fsm;
 *     shoot_fsm.Control();
 *
 * @note 单例语义：本工程只有一个发射机构，shootTask 内 static 实例即可
 */
class Class_ShootFSM
{
public:
    // === 拨盘子系统（DialController 作为成员）===
    BSP::CTRL::DialController dial_ctrl;  ///< 拨盘双环控制器(位置环+速度环+单击/长按+卡弹)

    // === 摩擦轮子系统（GM3508 速度环 PID）===
    PID       pid_friction_l;     ///< 左摩擦轮速度环 PID(motor_id=1, 目标 -rpm)
    PID       pid_friction_r;     ///< 右摩擦轮速度环 PID(motor_id=2, 目标 +rpm)
    Kpid_t    kpid_friction;      ///< 摩擦轮 PID 参数(kp/ki/kd, 从 Shoot_Config 同步)

    // === 状态机字段 ===
    ShootState state;              ///< 当前发射机构状态
    uint32_t   last_update_ms;     ///< 上次 Control() 调用时间戳(ms, 用于 dt)
    uint8_t    safety_ok;          ///< 安全条件是否满足(1=可控制, 0=需失能)
    uint8_t    last_safety_ok;     ///< 上一周期 safety_ok(边沿检测用)

    // === 摩擦轮控制字段 ===
    uint8_t    friction_enable;    ///< 摩擦轮使能(0=停, 1=转), 由右拨杆 UP 控制

    /**
     * @brief 默认构造
     *
     * PID 初始化参数（Break_I/MixI 仅作为初始值，运行时从 Shoot_Config 同步）：
     *   Break_I = 500 RPM  (误差小于此值才积分, 防大误差积分饱和)
     *   MixI    = 3000     (积分输出限幅, 限制 i_accum*ki 范围)
     */
    Class_ShootFSM()
        : pid_friction_l(500.0, 3000.0),
          pid_friction_r(500.0, 3000.0),
          state(ShootState::DISABLE),
          last_update_ms(0),
          safety_ok(0),
          last_safety_ok(0),
          friction_enable(0)
    {}

    /**
     * @brief 重置状态机（用于模式切换 / 重新使能）
     */
    void Reset()
    {
        dial_ctrl.Reset();
        pid_friction_l.clearPID();
        pid_friction_r.clearPID();
        state              = ShootState::DISABLE;
        last_update_ms     = 0;
        safety_ok          = 0;
        last_safety_ok     = 0;
        friction_enable    = 0;
    }

    /**
     * @brief 周期更新入口（由 shootTask 每 4ms 调用）
     *
     * 流程：
     *   1. 安全条件检查（遥控器离线 / 急停 / feature_enable / shoot_enabled）
     *   2. 状态机切换（DISABLE ↔ AUTO）
     *   3. 把 state 映射到 Dial_Config.enabled（联动拨盘）
     *   4. 委托拨盘控制给 dial_ctrl.Update()
     *   5. 摩擦轮速度环 PID 控制（右拨杆 UP 使能）
     *   6. 回写 Shoot_Status 供 Watch 观察
     *
     * @note 拨盘通过 DialController 间接访问 Motor
     *       摩擦轮直接访问 motor_3508 全局指针（GM3508 无需 Joint 抽象）
     */
    void Control();

private:
    /**
     * @brief 安全条件检查 + 状态机切换
     *
     * 切换规则：
     *   - !safety_ok → 强制 DISABLE
     *   - safety_ok && state==DISABLE → 进 AUTO（当前简化策略）
     *   - safety_ok && state==AUTO → 保持 AUTO
     *   - STOP 预留，当前不主动进入
     *
     * safety_ok = !remote_offline && !remote_estop && feature_enable && shoot_enabled
     */
    void updateStateMachine_();

    /**
     * @brief 把 state 映射到 Dial_Config.enabled（联动拨盘）
     *
     * 映射规则：
     *   DISABLE → Dial_Config.enabled = 0（拨盘零力矩）
     *   STOP    → Dial_Config.enabled = 1（拨盘待命）
     *   AUTO    → Dial_Config.enabled = 1（拨盘可触发）
     *
     * @note 摩擦轮 enable 不由此函数控制，由 updateFriction_() 读右拨杆
     * @note 这是 ShootFSM 与 DialController 的唯一耦合点
     */
    void applyStateToDialConfig_();

    /**
     * @brief 摩擦轮速度环 PID 控制
     *
     * 使能条件（全部满足才转）：
     *   - 右拨杆 UP（GetS2() == Switch::UP）
     *   - 遥控器在线（!IsOffline()）
     *   - 无急停（!(S1==DOWN && S2==DOWN)）
     *
     * 控制流程：
     *   1. 读 GM3508 反馈速度（getVelocityRpm, 电机端 RPM）
     *   2. 目标 RPM 从 Shoot_Config 读取（左右反向: 左=-target, 右=+target）
     *   3. PID 参数从 Shoot_Config 同步（kp/ki/kd + Break_I + MixI）
     *   4. 位置式 PID 计算（限幅 16384）
     *   5. ctrl_Current 发送电流命令 + sendCAN
     *
     * 停止策略（friction_enable=0）：
     *   - 直接发 0 电流
     *   - 清 PID 状态（clearPID）避免残留积分
     *
     * @note 状态回写仅用 Friction_Data（GimbalUpdate 中同步），不写 Shoot_Status
     */
    void updateFriction_();

    /**
     * @brief 回写 Shoot_Status 供 Watch 观察
     *
     * 同步字段：
     *   - state → Shoot_Status.state
     *   - safety_ok → Shoot_Status.safety_ok
     *   - friction_enable → Shoot_Status.friction_enable
     *   - Dial_Status 已经被 DialController 回写，无需重复
     */
    void syncStatus_();
};

} // namespace BSP::FSM

#endif // SHOOT_FSM_HPP
