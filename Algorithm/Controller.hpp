#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

/**
 * @file Controller.hpp
 * @brief Controller 层 - 位置式 PID
 *
 * 设计原因：
 *   Joint 层只提供"关节真实角度"，Motor 层只提供"电机控制接口"。
 *   Controller 层把目标角度 → 力矩，采用位置式 PID：
 *     error = target_angle - feedback_angle
 *     torque = kp * error + ki * Σerror + kd * Δerror
 *
 * 分层设计：
 *   1. JointController(单关节控制器)：
 *      - motor-agnostic 纯计算单元
 *      - 输入: target_angle(rad) + feedback_angle(rad)
 *      - 输出: torque(N·m)
 *      - 位置式 PID，带积分隔离 + 积分限幅
 *      - Yaw / Pitch / Fold 共用同一份代码，仅参数不同
 *
 *   2. GimbalController(三关节控制器)：
 *      - 持有 3 个 JointController + JointManager 指针 + Motor 指针
 *      - 每周期 Update：读取 Joint feedback → PID → Motor.ctrl_Mit
 *      - 统一以 MIT 纯力矩模式输出: ctrl_Mit(id, 0, 0, 0, 0, torque)
 *
 * 数据流：
 *   target_angle ──┐
 *                  ├→ 位置式 PID → torque (限幅 torque_limit)
 *   feedback_angle ┘
 *        ↓
 *   Motor.ctrl_Mit(id, 0, 0, 0, 0, torque × direction)
 */

#include "PID.hpp"
#include "Joint.hpp"
#include "DmMotor.hpp"

namespace BSP::CTRL
{

/**
 * @brief 关节类型枚举
 *
 * @note 仅用于日志/调试区分，不影响控制逻辑。
 *       三个关节使用同一份 JointController 代码，仅参数不同。
 */
enum class JointType : uint8_t
{
    YAW   = 0,   // Yaw 关节(DM4310 #1, 连续旋转)
    PITCH = 1,   // Pitch 关节(DM4310 #2, 有限位)
    FOLD  = 2,   // Fold 关节(DM4340 #1, 有限位，形态控制)
};


// ========================================================================
// JointController - 单关节位置式 PID 控制器（纯计算单元）
// ========================================================================
/**
 * @class JointController
 * @brief 单关节位置式 PID 控制器
 *
 * 设计原则：
 *   - 不区分 Yaw / Pitch / Fold（参数不同，算法相同）
 *   - 不持有 Motor 指针（与具体电机解耦）
 *   - 不读取 Encoder（feedback 由外部传入）
 *   - 不发送 CAN 帧（torque 由外部派发）
 *
 * 算法：位置式 PID
 *   error = target_angle - feedback_angle
 *   P = kp * error
 *   I = ki * Σerror (带积分隔离: |error| < break_i 才积分; 带积分限幅: |I| < limit_i)
 *   D = kd * (error - last_error)
 *   torque = P + I + D, 限幅到 [-torque_limit, torque_limit]
 */
class JointController
{
public:
    /// 位置 PID 实例
    PID position_pid;

    /// PID 参数（kp/ki/kd，Watch 可在线修改）
    Kpid_t kpid;

    // --- 配置参数（Watch 可调）---
    float torque_limit;     ///< 输出力矩限幅(N·m)，DM4310=10, DM4340=28
    float break_i;          ///< 积分隔离阈值(rad)，|error| < 此值才积分
    float limit_i;          ///< 积分输出限幅(N·m)，|I| 不超过此值

    // --- 运行时状态（Watch 可观察）---
    float target_angle;     ///< 目标角度(rad)
    float feedback_angle;   ///< 反馈角度(rad)
    float error;            ///< 位置误差(rad) = target - feedback
    float torque_output;    ///< PID 输出力矩(N·m)
    uint8_t enabled;        ///< 使能标志: 1=控制中, 0=失能(输出 0)
    uint8_t target_inited;  ///< 目标值是否已初始化(0=待初始化为当前角度, 1=已初始化)

    // --- 关节身份（仅日志用）---
    JointType type;         ///< 关节类型(Yaw/Pitch/Fold)

    /**
     * @brief 默认构造
     */
    JointController()
        : kpid(0, 0, 0),
          torque_limit(0), break_i(0), limit_i(0),
          target_angle(0), feedback_angle(0),
          error(0), torque_output(0),
          enabled(0), target_inited(0),
          type(JointType::PITCH)
    {}

    /**
     * @brief 初始化控制器
     * @param t           关节类型(仅日志用)
     * @param torque_lim  输出力矩限幅(N·m)
     * @param break_i_val 积分隔离阈值(rad)
     * @param limit_i_val 积分输出限幅(N·m)
     *
     * @note 调用后 enabled=0，需显式调用 Enable() 才开始控制
     *       实际参数由 Variable.cpp 通过 syncDataToController 覆盖
     */
    void Init(JointType t, float torque_lim, float break_i_val, float limit_i_val)
    {
        type          = t;
        torque_limit  = torque_lim;
        break_i       = break_i_val;
        limit_i       = limit_i_val;

        // 初始化 PID 实例（Break_I/MixI 会在 syncDataToController 中被 Variable.cpp 覆盖）
        position_pid = PID((double)break_i, (double)limit_i);

        // 默认 PID 参数（全 0，需 Watch 在线调参）
        kpid = Kpid_t(0, 0, 0);

        // 状态清零
        target_angle   = 0;
        feedback_angle = 0;
        error          = 0;
        torque_output  = 0;
        enabled        = 0;
        target_inited  = 0;
    }

    /**
     * @brief 位置式 PID 计算
     *
     * 流程：
     *   1. 首次调用时：target 自动设为 feedback（bumpless transfer）
     *   2. 失能时直接返回 0
     *   3. 位置式 PID → torque（限幅到 torque_limit）
     *
     * @param target   目标角度(rad)
     * @param feedback 反馈角度(rad)
     * @retval 输出力矩(N·m)，范围 [-torque_limit, torque_limit]
     */
    float Compute(float target, float feedback)
    {
        feedback_angle = feedback;

        // 首次使能时：把 target 自动设为当前反馈角度（bumpless transfer）
        if (!target_inited)
        {
            target_angle  = feedback;
            target_inited = 1;
        }
        else
        {
            target_angle = target;
        }

        if (!enabled)
        {
            position_pid.clearPID();
            error = 0;
            torque_output = 0;
            return 0.0f;
        }

        // --- 位置式 PID → 力矩 ---
        torque_output = (float)position_pid.GetPidPos(
            kpid,
            (double)target_angle,
            (double)feedback_angle,
            (double)torque_limit
        );
        error = (float)position_pid.GetErr();

        return torque_output;
    }

    /**
     * @brief 使能控制器
     */
    void Enable()
    {
        if (!enabled)
        {
            position_pid.clearPID();
        }
        enabled = 1;
    }

    /**
     * @brief 失能控制器
     */
    void Disable()
    {
        enabled = 0;
        position_pid.clearPID();
        torque_output = 0;
    }

    /**
     * @brief 清除 PID 状态
     */
    void Clear()
    {
        position_pid.clearPID();
        torque_output = 0;
        error = 0;
    }

    /**
     * @brief 设置目标角度（供 StateMachine/MotionPlanner/Vision 调用）
     */
    inline void setTarget(float target) { target_angle = target; }

    /**
     * @brief 平滑切换目标（bumpless transfer）
     */
    void snapTargetToFeedback(float current_feedback)
    {
        target_angle = current_feedback;
        position_pid.clearPID();
    }
};


// ========================================================================
// GimbalController - 三关节控制器
// ========================================================================
/**
 * @class GimbalController
 * @brief 三关节统一控制器（位置式 PID）
 *
 * 职责：
 *   1. 持有 Yaw / Pitch / Fold 三个 JointController
 *   2. 每周期 Update:
 *      - 从 JointManager 读取 feedback_angle
 *      - 对 target 进行限位钳位
 *      - 位置式 PID → torque
 *      - Motor.ctrl_Mit() 输出 MIT 纯力矩
 *   3. 失能关节也发送零力矩（保持电机在线）
 */
class GimbalController
{
public:
    JointController yaw;      ///< Yaw 控制器
    JointController pitch;    ///< Pitch 控制器
    JointController fold;     ///< Fold 控制器

    /**
     * @brief 初始化三关节控制器
     */
    void Init()
    {
        // Init(关节类型, 力矩限幅(N·m), 积分隔离阈值(rad), 积分限幅(N·m))
        //   力矩限幅    : 电机最大输出力矩，DM4310=10Nm, DM4340=28Nm
        //   积分隔离阈值 : |error| < 此值才积分，防止大误差时积分饱和
        //   积分限幅     : |I项| 不超过此值，防止积分主导导致超调

        // Yaw: DM4310 TMAX=10 N·m
        yaw.Init(JointType::YAW, 10.0f, 0.1f, 2.0f);
        //          类型      力矩上限  误差<0.1rad才积分 I项≤2Nm
        yaw.Disable();

        // Pitch: DM4310 TMAX=10 N·m
        pitch.Init(JointType::PITCH, 10.0f, 0.1f, 2.0f);
        //            类型       力矩上限  误差<0.1rad才积分 I项≤2Nm
        pitch.Enable();  // Stage03 验证对象

        // Fold: DM4340 TMAX=28 N·m
        fold.Init(JointType::FOLD, 28.0f, 0.1f, 5.0f);
        //          类型      力矩上限  误差<0.1rad才积分 I项≤5Nm
        fold.Disable();
    }

    /**
     * @brief 周期更新（在 GimbalUpdate 中调用，1kHz）
     *
     * 数据流（每个关节）：
     *   target_angle → PID → torque
     *       ↑           ↑        ↑
     *   Watch/SM   feedback_angle  Motor.ctrl_Mit
     *
     * @param jm     JointManager（提供三个 Joint 的反馈角度）
     * @param dm4310 Yaw+Pitch 电机指针（可为 nullptr）
     * @param dm4340 Fold 电机指针（可为 nullptr）
     */
    void Update(BSP::JOINT::JointManager &jm,
                BSP::MOTOR::DM::DM4310 *dm4310,
                BSP::MOTOR::DM::DM4340 *dm4340)
    {
        // --- Yaw 关节（DM4310 #1）---
        if (dm4310 != nullptr && jm.yaw.isOnline())
        {
            float fb = jm.yaw.getRealAngle();
            float tgt = yaw.target_angle;
            float torque = yaw.Compute(tgt, fb);
            dm4310->ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, torque * jm.yaw.getConfig().direction);
        }
        else if (dm4310 != nullptr)
        {
            yaw.Clear();
            dm4310->ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }

        // --- Pitch 关节（DM4310 #2）---
        if (dm4310 != nullptr && jm.pitch.isOnline())
        {
            float fb = jm.pitch.getRealAngle();
            float tgt = clampTarget(jm.pitch, pitch.target_angle);
            float torque = pitch.Compute(tgt, fb);
            dm4310->ctrl_Mit(2, 0.0f, 0.0f, 0.0f, 0.0f, torque * jm.pitch.getConfig().direction);
        }
        else if (dm4310 != nullptr)
        {
            pitch.Clear();
            dm4310->ctrl_Mit(2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }

        // --- Fold 关节（DM4340 #1）---
        if (dm4340 != nullptr && jm.fold.isOnline())
        {
            float fb = jm.fold.getRealAngle();
            float tgt = clampTarget(jm.fold, fold.target_angle);
            float torque = fold.Compute(tgt, fb);
            dm4340->ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, torque * jm.fold.getConfig().direction);
        }
        else if (dm4340 != nullptr)
        {
            fold.Clear();
            dm4340->ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
    }

private:
    /**
     * @brief 对 target 做限位钳位
     */
    float clampTarget(const BSP::JOINT::Joint &joint, float target)
    {
        const auto &cfg = joint.getConfig();
        if (cfg.continuous)
        {
            return target;
        }
        if (target > cfg.limit_max) return cfg.limit_max;
        if (target < cfg.limit_min) return cfg.limit_min;
        return target;
    }
};

} // namespace BSP::CTRL

#endif // CONTROLLER_HPP
