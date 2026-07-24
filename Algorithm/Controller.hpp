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
 *      - 输出 MIT 力矩模式: ctrl_Mit(id, 0, 0, 0, kd_min, torque)
 *        kd_min=0.1 维持电机 MIT 活跃（kp=kd=0 时部分固件会停止回复）
 *
 * 数据流：
 *   target_angle ──┐
 *                  ├→ 位置式 PID → torque (限幅 torque_limit)
 *   feedback_angle ┘
 *        ↓
 *   Motor.ctrl_Mit(id, 0, 0, 0, kd_min, torque × direction)
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
    uint8_t continuous;      ///< 连续旋转标志: 1=Yaw(连续旋转, 角度走最短路径), 0=Pitch/Fold(有限位)

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
          cascade_mode(0), continuous(0),
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
        continuous     = 0;   // 默认有限位，Yaw 由 GimbalController.Init 置 1

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

        // === 连续旋转关节角度误差最短路径处理 ===
        //   Yaw 连续旋转时，target 与 feedback 可能跨越 ±π 边界。
        //   若直接用 target - feedback，误差会接近 ±2π，导致 PID 输出反向冲击。
        //   处理方法：把 target 限制在 feedback ± π 范围内，使误差落在 [-π, π]。
        //   wrapToPi(target - feedback) 返回最短路径误差，再加回 feedback 得到等效 target。
        //
        //   举例：feedback = 3.0 rad, target = -3.0 rad
        //     未处理：error = -3.0 - 3.0 = -6.0 rad（错误，绕远路）
        //     处理后：error = wrapToPi(-6.0) = 0.28 rad（正确，走最短路径）
        //     target_used = 3.0 + 0.28 = 3.28 rad（等效目标）
        float target_used = target_angle;
        if (continuous)
        {
            float err_short = BSP::JOINT::wrapToPi(target_angle - feedback_angle);
            target_used = feedback_angle + err_short;
        }

        if (cascade_mode)
        {
            // === 串级 PID：外环角度环（位置式）→ 内环速度环（位置式）===

            // 外环：角度误差 → 速度目标(rad/s), 限幅到 [-vel_limit, vel_limit]
            vel_target = (float)position_pid.GetPidPos(
                kpid,
                (double)target_used,
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
                (double)target_used,
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
        target_inited = 0;  // 复位 bumpless，下次 Enable 后首次 Compute 自动同步目标到反馈值
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

    /**
     * @brief 速度环单环计算（跟随模式专用）
     *
     * 用于底盘跟随模式，Yaw轴从串级PID切换到速度环单环控制。
     * 此时Yaw轴不抵抗底盘扰动，根治反转现象。
     *
     * @param target_vel     目标角速度(rad/s)
     * @param feedback_vel   反馈角速度(rad/s)，来自IMU陀螺仪
     * @param kpid_vel       速度环PID参数(kp/ki/kd)
     * @retval 输出力矩(N·m)，范围 [-torque_limit, torque_limit]
     */
    float ComputeVelocity(float target_vel, float feedback_vel, Kpid_t kpid_vel)
    {
        vel_target = target_vel;
        vel_feedback = feedback_vel;
        vel_error = vel_target - vel_feedback;

        if (!enabled)
        {
            velocity_pid.clearPID();
            vel_error = 0;
            torque_output = 0;
            return 0.0f;
        }

        // 单级速度环PID（位置式）
        torque_output = (float)velocity_pid.GetPidPos(
            kpid_vel,
            (double)vel_target,
            (double)vel_feedback,
            (double)torque_limit
        );

        return torque_output;
    }

    /**
     * @brief 切换到速度环单环模式（跟随模式）
     *
     * 平滑过渡：从串级PID切换到速度环单环
     * - cascade_mode = 0（单级模式）
     * - 清空PID状态，避免冲击
     * - 速度目标从当前IMU速度开始
     *
     * @param initial_vel 初始速度目标(rad/s)，建议使用当前IMU角速度
     */
    void SwitchToVelocityMode(float initial_vel)
    {
        cascade_mode = 0;  // 切换到单级模式
        vel_target = initial_vel;
        velocity_pid.clearPID();  // 清空积分，防止冲击
        target_inited = 0;  // 重置，避免下次Compute自动设target
    }

    /**
     * @brief 切换回串级PID模式（其他模式）
     *
     * 平滑过渡：从速度环单环切换回串级PID
     * - cascade_mode = 1（串级模式）
     * - 清空PID状态，避免冲击
     * - 角度目标从当前IMU角度开始
     *
     * @param initial_angle 初始角度目标(rad)，建议使用当前IMU角度
     */
    void SwitchToCascadeMode(float initial_angle)
    {
        cascade_mode = 1;  // 切换到串级模式
        target_angle = initial_angle;
        position_pid.clearPID();
        velocity_pid.clearPID();
        target_inited = 1;  // 防止Compute自动覆盖target
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

    // --- IMU 传感器闭环相关（由 GimbalUpdate 在 Update 前写入）---

    /**
     * @brief IMU Yaw 反馈角度(rad)
     *
     * 由 GimbalInit.cpp GimbalUpdate() 在调用 Update() 前写入：
     *   yaw_imu_angle = -BSP::IMU::imu.getAddYaw() * (π/180)
     *
     * 取负原因：IMU Euler_yaw 正方向与编码器方向相反
     *   （向右转时编码器角度增大，但 IMU Euler_yaw 减小）
     *
     * addYaw 说明：HI12H3 欧拉角 Yaw 范围 [-180°, 180°]，过 ±180° 跳变
     *   IMU 内部 AddCaclu() 将跳变转换为连续累计角度，支持多圈旋转
     *
     * 当 imu_online=1 时，Update() 中 Yaw 外环反馈使用此值
     * 当 imu_online=0 时，Update() 中 Yaw 外环反馈回退到编码器
     */
    float yaw_imu_angle;

    /**
     * @brief IMU 在线标志
     *
     * 1 = IMU 在线 → Yaw 外环使用 IMU 反馈（传感器闭环）
     * 0 = IMU 离线 → Yaw 外环回退到编码器反馈（安全回退）
     *
     * 由 GimbalInit.cpp 根据 BSP::IMU::imu.isOnline() 设置
     *
     * 离线回退测试方法：手动拔掉 IMU 串口线，观察 Yaw 是否平稳切换到编码器闭环
     */
    uint8_t imu_online;

    /**
     * @brief Yaw 反馈源标识（Watch 可观察）
     *
     * 1 = IMU 传感器闭环（imu_online=1 时）
     * 0 = 编码器回退（imu_online=0 时，安全回退）
     *
     * 在 Update() 中根据 imu_online 自动设置
     * Watch 中观察此值可判断当前 Yaw 反馈来源
     */
    uint8_t yaw_fb_source;

    /**
     * @brief IMU Pitch 反馈角度(rad)
     *
     * 由 GimbalInit.cpp GimbalUpdate() 在调用 Update() 前写入：
     *   pitch_imu_angle = BSP::IMU::imu.getPitch() * (π/180)
     *
     * 不取负原因：实测确认 IMU Pitch 正方向与编码器方向一致
     *   （枪口抬起时编码器角度增大，IMU Pitch 也增大）
     *
     * getPitch() 说明：HI12H3 欧拉角 Pitch，范围 [-90°, 90°]
     *   Pitch 是有限位关节，不需要 addAngle 累加
     *
     * 当 imu_online=1 时，Update() 中 Pitch 外环反馈使用此值
     * 当 imu_online=0 时，Update() 中 Pitch 外环反馈回退到编码器
     *
     * 【Fold 影响分析】
     *   IMU 安装在枪口端（Pitch 之后），IMU Pitch 直接测量枪口绝对俯仰
     *   不管 Fold 展开/收起，IMU Pitch 始终反映枪口在世界坐标系中的真实俯仰角
     *   → IMU 闭环下 target 含义一致：枪口俯仰角，不受 Fold 影响
     *   → 编码器闭环下 target 含义随 Fold 变化，需要 Fold 管理器动态调整
     *   注意：机械限位仍依赖 Fold 状态（不同 Fold 下可达范围不同），
     *         但当前阶段先不做动态限位，等 Morphology Manager 阶段再完善
     */
    float pitch_imu_angle;

    /**
     * @brief Pitch 反馈源标识（Watch 可观察）
     *
     * 1 = IMU 传感器闭环（imu_online=1 时）
     * 0 = 编码器回退（imu_online=0 时，安全回退）
     *
     * 在 Update() 中根据 imu_online 自动设置
     * Watch 中观察此值可判断当前 Pitch 反馈来源
     */
    uint8_t pitch_fb_source;

    /**
     * @brief 初始化三关节控制器
     *
     * 配置策略：
     *   - Yaw   : 串级 PID（cascade_mode=1），连续旋转（continuous=1），外环角度环 + 内环速度环
     *   - Pitch : 串级 PID（cascade_mode=1），有限位，外环角度环 + 内环速度环
     *   - Fold  : 串级 PID（cascade_mode=1），有限位，外环角度环 + 内环速度环
     *
     * 串级参数说明：
     *   vel_limit     = 10 rad/s   (DM4310 VMAX=30, 保守取 10, 防止速度环目标过大)
     *   break_i_vel   = 1.0 rad/s  (速度误差<1才积分, 防止启停时积分饱和)
     *   limit_i_vel   = 2.0 N·m    (速度环 I 项≤2Nm, 防止积分主导)
     *
     * Yaw 连续旋转处理：
     *   continuous=1 → Compute() 中对 target 做 wrapToPi 最短路径处理
     *   避免跨越 ±π 边界时误差突变（详见 Compute 注释）
     *
     * @note PID 参数(kp/ki/kd)由 Variable.cpp 通过 syncDataToController 在线覆盖
     */
    void Init()
    {
        // Init(关节类型, 力矩限幅(N·m), 积分隔离阈值(rad), 积分限幅(N·m)
        //      [, vel_limit(rad/s), break_i_vel(rad/s), limit_i_vel(N·m)])

        // --- Yaw: DM4310 TMAX=10 N·m, 串级(角度环+速度环均位置式), 连续旋转 ---
        //   Stage04 验证对象：检查串级 PID + 连续旋转角度处理
        //   continuous=1: Compute() 中 target 走最短路径，避免 ±π 边界误差突变
        yaw.Init(JointType::YAW,
                 10.0f,    // torque_limit: 力矩上限 10 N·m
                 0.1f,     // break_i:      角度误差<0.1rad 才积分
                 2.0f,     // limit_i:      角度环 I 项 ≤ 2 N·m
                 10.0f,    // vel_limit:    速度目标限幅 10 rad/s (DM4310 VMAX=30, 保守)
                 1.0f,     // break_i_vel:  速度误差<1rad/s 才积分
                 2.0f);    // limit_i_vel:  速度环 I 项 ≤ 2 N·m
        yaw.continuous    = 1;   // ← Yaw 连续旋转（启用最短路径角度处理）
        yaw.cascade_mode  = 1;   // ← 启用串级模式
        yaw.Enable();              // 默认使能，与 Pitch/Fold 一致

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

        // IMU 传感器闭环初始化
        yaw_imu_angle   = 0.0f;    // 由 GimbalUpdate 写入 IMU addYaw(deg→rad, 取负)
        pitch_imu_angle = 0.0f;    // 由 GimbalUpdate 写入 IMU pitch(deg→rad, 不取负)
        imu_online      = 0;        // 默认 IMU 离线，GimbalUpdate 中根据实际状态设置
        yaw_fb_source   = 0;        // 默认编码器反馈
        pitch_fb_source = 0;        // 默认编码器反馈
    }

    /**
     * @brief 周期更新（在 GimbalUpdate 中调用，1kHz）
     *
     * 数据流（每个关节）：
     *   单级模式: target_angle + feedback_angle              → PID → torque
     *   串级模式: target_angle + feedback_angle + velocity   → [角度环→速度环] → torque
     *       ↑                  ↑                  ↑                ↓
     *   Watch/SM     Yaw:   IMU addYaw(在线)         Joint.getVelocity()  Motor.ctrl_Mit
     *                Yaw:   getNormalizedAngle(离线回退)
     *                Pitch: IMU getPitch(在线)
     *                Pitch: getRealAngle(离线回退)
     *                Fold:  getRealAngle(无IMU)
     *
     * Yaw IMU 传感器闭环：
     *   - IMU 在线(imu_online=1)：外环反馈 = yaw_imu_angle（IMU addYaw, 取负+deg→rad）
     *     优势：云台世界坐标系绝对航向 → 自动抗底盘扰动
     *   - IMU 离线(imu_online=0)：外环反馈回退到编码器 getNormalizedAngle()
     *     安全：编码器闭环虽不能抗底盘扰动，但保证不会疯车
     *   - 内环速度反馈始终用编码器 getVelocity()（与电机力矩直接关联，闭环更紧）
     *
     * Pitch IMU 传感器闭环：
     *   - IMU 在线(imu_online=1)：外环反馈 = pitch_imu_angle（IMU getPitch, deg→rad）
     *     优势：枪口绝对俯仰角 → target 含义与 Fold 状态无关
     *   - IMU 离线(imu_online=0)：外环反馈回退到编码器 getRealAngle()
     *     安全：编码器闭环保证基本功能
     *   - IMU 闭环下不做编码器坐标系限位钳位（IMU 反馈不在编码器坐标系）
     *     机械限位延后到 Morphology Manager 阶段处理
     *
     * @param jm     JointManager（提供三个 Joint 的 feedback_angle + velocity）
     * @param dm4310 Yaw+Pitch 电机指针（可为 nullptr）
     * @param dm4340 Fold 电机指针（可为 nullptr）
     */
    void Update(BSP::JOINT::JointManager &jm,
                BSP::MOTOR::DM::DM4310 *dm4310,
                BSP::MOTOR::DM::DM4340 *dm4340)
    {
        // --- Yaw 关节（DM4310 #1, IMU 传感器闭环 + 编码器回退）---
        //   外环角度反馈：
        //     IMU 在线 → yaw_imu_angle（IMU addYaw, 取负+deg→rad, 连续累加支持多圈）
        //     IMU 离线 → jm.yaw.getNormalizedAngle()（编码器回退, [-π, π]）
        //   内环速度反馈：始终用编码器 jm.yaw.getVelocity()
        //   连续旋转：continuous=1 → Compute() 中 wrapToPi 最短路径处理
        //
        //   【跟随模式特殊处理】
        //   如果yaw处于速度环单环模式（cascade_mode=0），说明在跟随模式下，
        //   GimbalInit.cpp中已经直接调用ComputeVelocity()和ctrl_Mit()，
        //   此处跳过，避免重复控制。
        if (dm4310 != nullptr)
        {
            // 检查是否在跟随模式（速度环单环）
            if (yaw.cascade_mode == 0) {
                // 跟随模式：已在GimbalInit中直接控制，此处仅更新反馈源标志
                yaw_fb_source = imu_online ? 2 : 0;  // 2=IMU速度环模式
            } else {
                // 串级PID模式：正常计算
                float fb;
                if (jm.yaw.isOnline() && imu_online)
                {
                    // IMU 传感器闭环：世界坐标系绝对航向
                    //   yaw_imu_angle 由 GimbalUpdate 写入 = -addYaw × (π/180)
                    //   addYaw 已跨 ±180° 连续累加，天然支持多圈旋转
                    fb = yaw_imu_angle;
                    yaw_fb_source = 1;  // 标记当前使用 IMU 反馈
                }
                else
                {
                    // 编码器回退：IMU 离线时的安全降级
                    //   getNormalizedAngle() 归一化到 [-π, π]
                    //   wrapToPi 保证与累积 target 的误差走最短路径
                    fb = jm.yaw.getNormalizedAngle();
                    yaw_fb_source = 0;  // 标记当前使用编码器反馈
                }

                float vel = jm.yaw.getVelocity();  // 内环始终用编码器速度
                float tgt = yaw.target_angle;
                float torque = yaw.Compute(tgt, fb, vel);
                dm4310->ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, torque * jm.yaw.getConfig().direction);
            }
        }
        else
        {
            yaw.Clear();
            yaw_fb_source = 0;
        }

        // --- Pitch 关节（DM4310 #2, IMU 传感器闭环）---
        //   外环角度反馈：
        //     IMU 在线 → pitch_imu_angle（IMU getPitch, deg→rad, 方向与编码器一致不取负）
        //     IMU 离线 → 停控（零力矩保持，不输出 PID）
        //   内环速度反馈：始终用编码器 jm.pitch.getVelocity()
        //   IMU 闭环下：target 含义是枪口绝对俯仰角，不受 Fold 影响
        //   IMU 闭环下不做 clampTarget（编码器坐标系限位不适用于 IMU 反馈）
        //
        //   新方案：变形期间 Pitch 也用 IMU 闭环保持水平(target=0 rad)
        //   Planner 写入 pitch.target=0（IMU 水平目标），Controller 用 IMU 反馈
        //   IMU 离线时停控，不发 PID 力矩（仅用 kd_min 维持 MIT 活跃回复）
        if (dm4310 != nullptr)
        {
            if (jm.pitch.isOnline() && imu_online)
            {
                // IMU 传感器闭环：枪口绝对俯仰角
                //   pitch_imu_angle = getPitch() × (π/180)，不取负（方向一致）
                //   IMU Pitch 范围 [-90°, 90°]，对应 [-π/2, π/2]
                //   变形期间 target=0（枪口水平），非变形期间 target 由 Watch/Planner 设定
                float fb = pitch_imu_angle;
                // IMU 闭环下不做编码器坐标系限位钳位
                //   target 含义是枪口绝对俯仰(rad)，不在编码器坐标系
                //   机械限位延后到 Morphology Manager 阶段
                float tgt = pitch.target_angle;
                float vel = jm.pitch.getVelocity();
                float torque = pitch.Compute(tgt, fb, vel);
                dm4310->ctrl_Mit(2, 0.0f, 0.0f, 0.0f, 0.0f, torque * jm.pitch.getConfig().direction);
                pitch_fb_source = 1;  // 标记当前使用 IMU 反馈
            }
            else
            {
                // IMU 离线：停控，清 PID，发零力矩帧维持电机 MIT 在线
                pitch.Clear();
                dm4310->ctrl_Mit(2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                pitch_fb_source = 0;
            }
        }
        else
        {
            pitch.Clear();
            pitch_fb_source = 0;
        }

        // --- Fold 关节（DM4340 #1）---
        // 【修复死锁】
        //   原条件 jm.fold.isOnline() 导致：离线时不发 ctrl_Mit → 电机看门狗超时 → 永久离线
        //   修复：只要电机实例存在就发 ctrl_Mit（kp=kd=0 的帧也能维持电机反馈在线）
        if (dm4340 != nullptr)
        {
            float fb  = jm.fold.getRealAngle();
            float vel = jm.fold.getVelocity();
            float tgt = clampTarget(jm.fold, fold.target_angle);
            float torque = fold.Compute(tgt, fb, vel);
            dm4340->ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, torque * jm.fold.getConfig().direction);
        }
        else
        {
            fold.Clear();
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
