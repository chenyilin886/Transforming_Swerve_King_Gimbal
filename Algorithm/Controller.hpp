#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

/**
 * @file Controller.hpp
 * @brief Controller 层 - 位置式 PID
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
 * @brief 单关节 PID 控制器（支持单级位置式 + 串级位置式）
 *
 * 设计原则：
 *   - 不区分 Yaw / Pitch / Fold（参数不同，算法相同）
 *   - 不持有 Motor 指针（与具体电机解耦）
 *   - 不读取 Encoder（feedback 由外部传入）
 *   - 不发送 CAN 帧（torque 由外部派发）
 *
 * 支持两种模式（cascade_mode 切换）：
 *
 *   ① cascade_mode = 0  → 单级位置式 PID（Yaw / Fold 默认）
 *      error = target_angle - feedback_angle
 *      P = kp * error
 *      I = ki * Σerror (带积分隔离 + 积分限幅)
 *      D = kd * (error - last_error)
 *      torque = P + I + D, 限幅 [-torque_limit, torque_limit]
 *
 *   ② cascade_mode = 1  → 串级 PID（Pitch Stage03 验证对象）
 *      外环：角度环（位置式 PID）
 *        error_angle = target_angle - feedback_angle
 *        vel_target  = Kp_angle*err + Ki_angle*Σerr + Kd_angle*Δerr
 *        vel_target 限幅到 [-vel_limit, vel_limit] (rad/s)
 *      内环：速度环（位置式 PID）
 *        error_vel = vel_target - vel_feedback
 *        torque    = Kp_vel*err + Ki_vel*Σerr + Kd_vel*Δerr
 *        torque    限幅到 [-torque_limit, torque_limit] (N·m)
 *
 *   两个环均使用位置式 PID（Stage03 设计要求）。
 *   速度环采用位置式 PID 的注意事项：
 *     - 速度误差持续存在时积分易饱和 → 用 break_i_vel 积分隔离 + limit_i_vel 积分限幅
 *     - 调参起点建议先 ki_vel=0，仅 P+D，确认稳定后再加 ki_vel
 *
 * 数据流：
 *   target_angle ──┐
 *                  ├→ [角度环 PID] ──→ vel_target ──┐
 *   feedback_angle ┘                                 ├→ [速度环 PID] ──→ torque
 *                                              vel_feedback ┘
 *   (cascade_mode=0 时跳过角度环输出，直接位置式 PID → torque)
 */
class JointController
{
public:
    // === 外环（角度环）PID ===
    PID position_pid;        ///< 角度环 PID 实例（单级模式 = 主 PID；串级模式 = 外环）
    Kpid_t kpid;             ///< 角度环 PID 参数(kp/ki/kd, Watch 可在线修改)

    // === 内环（速度环）PID — 仅 cascade_mode=1 时使用 ===
    PID velocity_pid;        ///< 速度环 PID 实例(串级内环, 位置式)
    Kpid_t kpid_vel;         ///< 速度环 PID 参数(kp/ki/kd, Watch 可在线修改)

    // --- 配置参数（Watch 可调）---
    float torque_limit;      ///< 输出力矩限幅(N·m), DM4310=10, DM4340=28
    float break_i;           ///< 角度环积分隔离阈值(rad), |error|<此值才积分
    float limit_i;           ///< 角度环积分输出限幅(N·m)
    float vel_limit;         ///< 速度环输出限幅(rad/s) = 角度环输出限幅, DM4310 VMAX=30, 保守取 10
    float break_i_vel;       ///< 速度环积分隔离阈值(rad/s), |vel_error|<此值才积分
    float limit_i_vel;       ///< 速度环积分输出限幅(N·m)
    uint8_t cascade_mode;    ///< 串级模式开关: 0=单级位置式, 1=串级(角度环+速度环均位置式)

    // --- 运行时状态（Watch 可观察）---
    float target_angle;      ///< 目标角度(rad)
    float feedback_angle;    ///< 反馈角度(rad)
    float error;             ///< 角度环误差(rad) = target - feedback
    float vel_target;        ///< 速度环目标(rad/s) = 角度环 PID 输出(串级模式有效)
    float vel_feedback;      ///< 速度环反馈(rad/s) = Joint.velocity
    float vel_error;         ///< 速度环误差(rad/s) = vel_target - vel_feedback
    float torque_output;     ///< 最终输出力矩(N·m)
    uint8_t enabled;         ///< 使能标志: 1=控制中, 0=失能(输出 0)
    uint8_t target_inited;   ///< 目标值是否已初始化(0=待初始化为当前角度, 1=已初始化)

    // === 前馈补偿（重力补偿）===
    float gravity_k;         ///< 重力补偿系数(N·m), Watch 在线标定, 公式: m·g·r
    float gravity_torque;    ///< 重力补偿输出(N·m), Watch 观察用, 公式: gravity_k * cos(feedback_angle)
    uint8_t gravity_enable;  ///< 重力补偿使能: 0=关, 1=开(串级模式专用, Fold Stage04 验证对象)

    // --- 关节身份（仅日志用）---
    JointType type;          ///< 关节类型(Yaw/Pitch/Fold)

    /**
     * @brief 默认构造
     */
    JointController()
        : kpid(0, 0, 0), kpid_vel(0, 0, 0),
          torque_limit(0), break_i(0), limit_i(0),
          vel_limit(0), break_i_vel(0), limit_i_vel(0),
          cascade_mode(0),
          target_angle(0), feedback_angle(0),
          error(0), vel_target(0), vel_feedback(0), vel_error(0),
          torque_output(0),
          enabled(0), target_inited(0),
          gravity_k(0), gravity_torque(0), gravity_enable(0),
          type(JointType::PITCH)
    {}

    /**
     * @brief 初始化控制器
     * @param t               关节类型(仅日志用)
     * @param torque_lim      输出力矩限幅(N·m)
     * @param break_i_val     角度环积分隔离阈值(rad)
     * @param limit_i_val     角度环积分输出限幅(N·m)
     * @param vel_lim         速度环输出限幅(rad/s), 仅串级模式生效, 默认 0
     * @param break_i_vel_val 速度环积分隔离阈值(rad/s), 默认 0
     * @param limit_i_vel_val 速度环积分输出限幅(N·m), 默认 0
     *
     * @note 调用后 enabled=0，需显式调用 Enable() 才开始控制
     *       实际参数由 Variable.cpp 通过 syncDataToController 覆盖
     *       cascade_mode 默认 0(单级)，Pitch 由 GimbalController.Init 置 1
     */
    void Init(JointType t, float torque_lim, float break_i_val, float limit_i_val,
              float vel_lim = 0.0f, float break_i_vel_val = 0.0f, float limit_i_vel_val = 0.0f)
    {
        type          = t;
        torque_limit  = torque_lim;
        break_i       = break_i_val;
        limit_i       = limit_i_val;

        vel_limit     = vel_lim;
        break_i_vel   = break_i_vel_val;
        limit_i_vel   = limit_i_vel_val;

        // 初始化 PID 实例（Break_I/MixI 会在 syncDataToController 中被 Variable.cpp 覆盖）
        position_pid = PID((double)break_i,       (double)limit_i);
        velocity_pid = PID((double)break_i_vel,   (double)limit_i_vel);

        // 默认 PID 参数（全 0，需 Watch 在线调参）
        kpid     = Kpid_t(0, 0, 0);
        kpid_vel = Kpid_t(0, 0, 0);

        // 状态清零
        target_angle   = 0;
        feedback_angle = 0;
        error          = 0;
        vel_target     = 0;
        vel_feedback   = 0;
        vel_error      = 0;
        torque_output  = 0;
        enabled        = 0;
        target_inited  = 0;
        cascade_mode   = 0;   // 默认单级，Pitch 由 GimbalController.Init 置 1

        // 重力补偿初始化（默认关闭，由 Variable.cpp 在线启用）
        gravity_k      = 0;
        gravity_torque = 0;
        gravity_enable = 0;
    }

    /**
     * @brief PID 计算（单级 / 串级自动切换）
     *
     * 流程：
     *   1. 首次调用：target 自动设为 feedback（bumpless transfer）
     *   2. 失能：直接返回 0，并清空两个 PID 状态
     *   3. cascade_mode=0：单级位置式 PID → torque
     *      cascade_mode=1：外环角度环 PID → vel_target；内环速度环 PID → torque
     *
     * @param target   目标角度(rad)
     * @param feedback 反馈角度(rad)
     * @param velocity 反馈角速度(rad/s) - 串级模式内环使用，单级模式忽略
     * @retval 输出力矩(N·m)，范围 [-torque_limit, torque_limit]
     */
    float Compute(float target, float feedback, float velocity)
    {
        feedback_angle = feedback;
        vel_feedback   = velocity;

        // 首次使能时：把 target 自动设为当前反馈角度（bumpless transfer）
        // 串级模式下同时也让 vel_target 起点为 0（速度环不冲击）
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
            velocity_pid.clearPID();
            error         = 0;
            vel_error     = 0;
            vel_target    = 0;
            torque_output = 0;
            return 0.0f;
        }

        if (cascade_mode)
        {
            // === 串级 PID：外环角度环（位置式）→ 内环速度环（位置式）===

            // 外环：角度误差 → 速度目标(rad/s), 限幅到 [-vel_limit, vel_limit]
            vel_target = (float)position_pid.GetPidPos(
                kpid,
                (double)target_angle,
                (double)feedback_angle,
                (double)vel_limit
            );
            error = (float)position_pid.GetErr();

            // 内环：速度误差 → 力矩(N·m), 限幅到 [-torque_limit, torque_limit]
            torque_output = (float)velocity_pid.GetPidPos(
                kpid_vel,
                (double)vel_target,
                (double)vel_feedback,
                (double)torque_limit
            );
            vel_error = (float)velocity_pid.GetErr();
        }
        else
        {
            // === 单级位置式 PID（Yaw/Fold 路径，与 Stage03 之前行为一致）===
            torque_output = (float)position_pid.GetPidPos(
                kpid,
                (double)target_angle,
                (double)feedback_angle,
                (double)torque_limit
            );
            error = (float)position_pid.GetErr();

            // 串级字段保持 0，避免 Watch 误读
            vel_target = 0;
            vel_error  = 0;
        }

        // === 重力补偿（前馈叠加）===
        //   仅串级模式生效（gravity_enable && cascade_mode）
        //   公式：gravity_torque = gravity_k * cosf(feedback_angle)
        //   用 feedback_angle（实时角度）而非 target_angle
        //   不乘 direction（重力补偿在机器人坐标系计算，Motor 输出层统一乘 direction）
        //
        //   Fold 机构几何（Stage04 验证）：
        //     - 零位（θ=0）：重心距 Fold 轴水平距离最大 → 力臂最大 → cos(0)=1，补偿最大
        //     - 展开（θ增大）：重心逐步移到 Fold 轴正上方 → 力臂减小 → cos(θ)减小，补偿减小
        //     - 完全展开（θ≈π/2）：重心竖直对齐 → 力臂=0 → cos(π/2)=0，补偿为零
        //
        //   标定方法：关补偿测多点的保持力矩 → 拟合 gravity_k = torque / cos(angle)
        if (gravity_enable && cascade_mode)
        {
            gravity_torque = gravity_k * cosf(feedback_angle);
            torque_output += gravity_torque;

            // 限幅（防止超限）
            if (torque_output > torque_limit)  torque_output = torque_limit;
            if (torque_output < -torque_limit) torque_output = -torque_limit;
        }
        else
        {
            gravity_torque = 0;  // 单级模式或未启用，补偿为零
        }

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
            velocity_pid.clearPID();
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
        velocity_pid.clearPID();
        torque_output = 0;
        vel_target    = 0;
        vel_error     = 0;
    }

    /**
     * @brief 清除 PID 状态（不清除 enabled）
     */
    void Clear()
    {
        position_pid.clearPID();
        velocity_pid.clearPID();
        torque_output = 0;
        error         = 0;
        vel_target    = 0;
        vel_error     = 0;
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
        velocity_pid.clearPID();
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
     *
     * Stage03 配置策略：
     *   - Yaw / Fold : 单级位置式 PID（cascade_mode=0），Stage04 启用
     *   - Pitch      : 串级 PID（cascade_mode=1），外环角度环 + 内环速度环，均位置式
     *
     * Pitch 串级参数说明：
     *   vel_limit     = 10 rad/s   (DM4310 VMAX=30, 保守取 10, 防止速度环目标过大)
     *   break_i_vel   = 1.0 rad/s  (速度误差<1才积分, 防止启停时积分饱和)
     *   limit_i_vel   = 2.0 N·m    (速度环 I 项≤2Nm, 防止积分主导)
     *
     * @note PID 参数(kp/ki/kd)由 Variable.cpp 通过 syncDataToController 在线覆盖
     */
    void Init()
    {
        // Init(关节类型, 力矩限幅(N·m), 积分隔离阈值(rad), 积分限幅(N·m)
        //      [, vel_limit(rad/s), break_i_vel(rad/s), limit_i_vel(N·m)])

        // --- Yaw: DM4310 TMAX=10 N·m, 单级位置式(Stage04) ---
        yaw.Init(JointType::YAW, 10.0f, 0.1f, 2.0f);
        //          类型      力矩上限  误差<0.1rad才积分 I项≤2Nm
        yaw.Disable();

        // --- Pitch: DM4310 TMAX=10 N·m, 串级(角度环+速度环均位置式) ---
        //   Stage03 验证对象
        pitch.Init(JointType::PITCH,
                   10.0f,    // torque_limit: 力矩上限 10 N·m
                   0.1f,     // break_i:      角度误差<0.1rad 才积分
                   2.0f,     // limit_i:      角度环 I 项 ≤ 2 N·m
                   10.0f,    // vel_limit:    速度目标限幅 10 rad/s
                   1.0f,     // break_i_vel:  速度误差<1rad/s 才积分
                   2.0f);    // limit_i_vel:  速度环 I 项 ≤ 2 N·m
        pitch.cascade_mode = 1;   // ← 启用串级模式
        pitch.Enable();

        // --- Fold: DM4340 TMAX=28 N·m, 串级(角度环+速度环均位置式) ---
        //   Stage04 验证对象
        //   DM4340 VMAX=10 rad/s, 保守取 vel_limit=5
        //   DM4340 力矩大(28Nm)，积分限幅给 5Nm 防止超调
        fold.Init(JointType::FOLD,
                  28.0f,    // torque_limit: 力矩上限 28 N·m
                  0.1f,     // break_i:      角度误差<0.1rad 才积分
                  5.0f,     // limit_i:      角度环 I 项 ≤ 5 N·m
                  5.0f,     // vel_limit:    速度目标限幅 5 rad/s (DM4340 VMAX=10, 保守)
                  1.0f,     // break_i_vel:  速度误差<1rad/s 才积分
                  5.0f);    // limit_i_vel:  速度环 I 项 ≤ 5 N·m
        fold.cascade_mode = 1;   // ← 启用串级模式
        fold.Enable();
    }

    /**
     * @brief 周期更新（在 GimbalUpdate 中调用，1kHz）
     *
     * 数据流（每个关节）：
     *   单级模式: target_angle + feedback_angle              → PID → torque
     *   串级模式: target_angle + feedback_angle + velocity   → [角度环→速度环] → torque
     *       ↑                  ↑                  ↑                ↓
     *   Watch/SM         Joint.getRealAngle()  Joint.getVelocity()  Motor.ctrl_Mit
     *
     * @param jm     JointManager（提供三个 Joint 的 feedback_angle + velocity）
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
            float fb  = jm.yaw.getRealAngle();
            float vel = jm.yaw.getVelocity();
            float tgt = yaw.target_angle;
            float torque = yaw.Compute(tgt, fb, vel);
            dm4310->ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, torque * jm.yaw.getConfig().direction);
        }
        else if (dm4310 != nullptr)
        {
            yaw.Clear();
            dm4310->ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }

        // --- Pitch 关节（DM4310 #2, Stage03 串级验证对象）---
        if (dm4310 != nullptr && jm.pitch.isOnline())
        {
            float fb  = jm.pitch.getRealAngle();
            float vel = jm.pitch.getVelocity();   // 串级内环速度反馈
            float tgt = clampTarget(jm.pitch, pitch.target_angle);
            float torque = pitch.Compute(tgt, fb, vel);
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
            float fb  = jm.fold.getRealAngle();
            float vel = jm.fold.getVelocity();
            float tgt = clampTarget(jm.fold, fold.target_angle);
            float torque = fold.Compute(tgt, fb, vel);
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
