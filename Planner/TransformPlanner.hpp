#ifndef TRANSFORM_PLANNER_HPP
#define TRANSFORM_PLANNER_HPP

/**
 * @file TransformPlanner.hpp
 * @brief Stage05 变形动作规划器（Motion Planner + Morphology Manager 合并实现）
 *
 * 设计背景：
 *   RM2026 三关节可变形云台，Fold 改变云台几何结构。
 *   新方案：Yaw/Pitch/Fold 三关节并行控制：
 *     - Yaw: IMU 闭环锁定到变形开始时的角度（保持静止）
 *     - Pitch: IMU 闭环保持水平(target=0 rad)
 *     - Fold: 编码器闭环精确到位
 *
 * 用户动作序列要求（并行执行）：
 *
 *   展开(EXPAND):
 *     ① Yaw 锁定到当前IMU角度 + Pitch IMU 保持水平(target=0) + Fold→fold_expand
 *     ② 等待 Fold 到位 → EXPANDED 终态
 *
 *   收起(CONTRACT):
 *     ① Yaw 锁定到当前IMU角度 + Pitch IMU 保持水平(target=0) + Fold→fold_contract
 *     ② 等待 Fold 到位 → CONTRACTED 终态
 *
 * 关键设计：
 *   - Yaw 在变形开始时锁定 target 到当前 IMU 角度，变形期间保持静止
 *   - Pitch 始终用 IMU 闭环保持水平(target=0 rad)，不受 Fold 转动影响
 *   - Fold 用编码器闭环精确到位(target=fold_expand/fold_contract)
 *   - 到位判定只看 Fold，不看 Yaw/Pitch（Yaw/Pitch 是持续保持，无需"到位"）
 *   - IMU 离线时 Yaw/Pitch 回退到编码器闭环（Controller 层处理）
 *
 * 模块落点（按 System Prompt 分层）：
 *   本模块合并了 Motion Planner（动作顺序/等待/超时）和 Morphology Manager
 *   （形态状态管理）两层职责。
 *
 * 数据流（与现有架构兼容，仅插入 Step 2.5）：
 *   Watch.cmd ──→ TransformPlanner.Update()
 *                  │
 *                  ├─ 读 JointManager feedback（yaw/pitch/fold real_angle + online）
 *                  ├─ 读 Controller_Data.yaw/pitch.feedback_angle（IMU角度）
 *                  ├─ 状态机转移（IDLE/TRANSITION/终态/ABORT）
 *                  └─ TRANSITION 状态: 写 Controller_Data.yaw/pitch/fold.target_angle
 *                     终态/IDLE/ABORT: 不写（释放给 Watch）
 *
 * Target 归属规则（关键设计）：
 *   - TRANSITION 状态：Planner 每周期覆盖 target_angle
 *     yaw.target   = 锁定到变形开始时的IMU角度（保持静止）
 *     pitch.target = 0 (IMU 水平)
 *     fold.target  = fold_expand / fold_contract (编码器目标)
 *   - 终态（EXPANDED/CONTRACTED）/ IDLE / ABORT：Planner 不写 target
 *     Watch 可直接手动控制
 *
 * ABORT 安全策略：
 *   - 触发条件：用户 cmd=ABORT / 单步超时 / 电机离线
 *   - 处理：snap yaw/pitch/fold target 到当前 feedback（Hold 当前位置，不 limp）
 *   - 释放 target 给 Watch，用户可手动接管
 *   - last_error 记录原因，cmd=RESET 回到 IDLE
 *
 * 调用频率：1kHz（与 GimbalUpdate 一致），step_elapsed_ms 单位 = ms
 */

#include "Joint.hpp"
#include "Variable.hpp"

namespace BSP::PLANNER
{

/**
 * @brief 变形状态机枚举（Morphology + Motion 子状态合并）
 *
 * 状态流向：
 *   IDLE ──cmd=EXPAND──→ EXPAND_SIMULTANEOUS ──Fold到位──→ EXPANDED
 *   EXPANDED ──cmd=CONTRACT──→ CONTRACT_SIMULTANEOUS ──Fold到位──→ CONTRACTED
 *   CONTRACTED ──cmd=EXPAND──→ EXPAND_SIMULTANEOUS ──→ ...
 *   任意 TRANSITION ──cmd=ABORT/超时/离线──→ ABORT ──cmd=RESET──→ IDLE
 *
 * @note 数值固定，Watch 中可直接用整数观察（0..5）
 */
enum class TransformState : uint8_t
{
    IDLE                    = 0,  // 待机（上电默认）
    EXPAND_SIMULTANEOUS     = 1,  // 展开: Pitch IMU保持水平 + Fold→fold_expand
    EXPANDED                = 2,  // 展开终态（释放 target 给 Watch）
    CONTRACT_SIMULTANEOUS   = 3,  // 收起: Pitch IMU保持水平 + Fold→fold_contract
    CONTRACTED              = 4,  // 收起终态（释放 target 给 Watch）
    ABORT                   = 5,  // 异常退出（Hold 当前位置）
};

/**
 * @brief 外部命令枚举（Watch 写入 Transform_Config.cmd）
 *
 * 命令优先级：ABORT > RESET > EXPAND/CONTRACT
 *   - ABORT：任意非 ABORT 状态都接受
 *   - RESET：仅 ABORT 状态接受
 *   - EXPAND：IDLE/EXPANDED/CONTRACTED 接受
 *   - CONTRACT：IDLE/EXPANDED/CONTRACTED 接受
 *
 * @note cmd 是"单次触发"型，Planner 消费后自动清零（cfg.cmd=NONE）
 */
enum class TransformCmd : uint8_t
{
    NONE     = 0,  // 无命令
    EXPAND   = 1,  // 展开命令
    CONTRACT = 2,  // 收起命令
    ABORT    = 3,  // 紧急中止
    RESET    = 4,  // 从 ABORT 恢复到 IDLE
};

/**
 * @brief ABORT 原因错误码
 */
enum class TransformError : uint8_t
{
    NONE            = 0,  // 正常
    TIMEOUT         = 1,  // 单步超时
    MOTOR_OFFLINE  = 2,  // 电机离线
    ABORT_CMD       = 3,  // 用户主动 ABORT
};

/**
 * @brief 判断当前状态是否为 TRANSITION（动作执行中）
 */
static inline bool isTransitionState(TransformState s)
{
    return s == TransformState::EXPAND_SIMULTANEOUS ||
           s == TransformState::CONTRACT_SIMULTANEOUS;
}

/**
 * @brief 判断当前状态是否为终态（释放 target 给 Watch）
 */
static inline bool isTerminalState(TransformState s)
{
    return s == TransformState::EXPANDED ||
           s == TransformState::CONTRACTED;
}


/**
 * @class TransformPlanner
 * @brief 变形动作规划器（单实例，全局）
 *
 * 职责：
 *   1. 接收 Watch 命令（EXPAND/CONTRACT/ABORT/RESET）
 *   2. 按状态机执行并行动作（Pitch IMU保持水平 + Fold 编码器到位）
 *   3. 到位检测（仅 Fold，误差阈值 + 超时）
 *   4. 异常处理（ABORT 时 snap target 到 feedback）
 *   5. 写入 Controller_Data.pitch/fold.target_angle（仅 TRANSITION 状态）
 *
 * 不负责：
 *   - PID 计算（Controller 层负责）
 *   - Motor 控制（Controller 层负责）
 *   - Encoder 读取（Joint 层负责）
 *
 * 调用位置：GimbalUpdate() 中，JointManager.Update 之后，
 *           syncDataToController 之前（Step 2.5）
 */
class TransformPlanner
{
public:
    /**
     * @brief 默认构造，状态 = IDLE
     */
    TransformPlanner()
        : state_(TransformState::IDLE),
          step_elapsed_ms_(0),
          yaw_locked_target_(0.0f)
    {}

    /**
     * @brief 获取当前状态
     */
    TransformState getState() const { return state_; }

    /**
     * @brief 周期更新（1kHz 调用）
     *
     * 流程：
     *   1. 读取 Joint feedback（yaw/pitch/fold real_angle + online）
     *   2. 读取 IMU 反馈（yaw/pitch.feedback_angle）
     *   3. 命令解析（ABORT 优先级最高，ABORT 状态只接受 RESET）
     *   4. 状态机执行：
     *      - 离线/超时检查 → ABORT
     *      - 到位检查（仅 Fold）→ 进入终态
     *      - TRANSITION 状态开始时锁定 yaw target
     *   5. 写入 target（仅 TRANSITION 状态）
     *      yaw.target   = yaw_locked_target_（锁定值）
     *      pitch.target = 0 (IMU 水平)
     *      fold.target  = fold_expand / fold_contract (编码器目标)
     *   6. 更新观察状态（Transform_Status）
     *   7. 步进时间（step_elapsed_ms++）
     *
     * @param jm         JointManager（读 yaw/pitch/fold feedback）
     * @param ctrl_data  Controller_Data（写 target，仅 TRANSITION 状态）
     * @param cfg        Transform_Config（读参数 + 命令；消费后 cmd 清零）
     * @param status     Transform_Status（写观察状态，每周期更新）
     *
     * @note 调用频率必须 1kHz，否则 step_elapsed_ms / timeout 单位失真
     */
    void Update(BSP::JOINT::JointManager &jm,
                Controller_Data_t &ctrl_data,
                Transform_Config_t &cfg,
                Transform_Status_t &status)
    {
        // === 1. 读取 feedback ===
        //   Yaw:   读取编码器 feedback（仅用于ABORT snap）
        //          IMU feedback 从 ctrl_data.yaw.feedback_angle 获取（由 GimbalUpdate 写入）
        //   Pitch: 不需要读编码器 feedback（IMU 闭环保持水平，无需到位判定）
        //          IMU feedback 从 ctrl_data.pitch.feedback_angle 获取
        //   Fold:  需要读编码器 feedback 做到位判定
        float yaw_fb_encoder  = jm.yaw.getRealAngle();    // 编码器反馈（仅用于ABORT snap）
        float yaw_fb_imu      = ctrl_data.yaw.feedback_angle; // IMU反馈（Controller已设置）
        float pitch_fb_encoder = jm.pitch.getRealAngle(); // 编码器反馈（仅用于ABORT snap）
        float pitch_fb_imu     = ctrl_data.pitch.feedback_angle; // IMU反馈
        float fold_fb          = jm.fold.getRealAngle();

        bool  yaw_online   = jm.yaw.isOnline();
        bool  pitch_online = jm.pitch.isOnline();
        bool  fold_online  = jm.fold.isOnline();

        status.yaw_online   = yaw_online ? 1 : 0;
        status.pitch_online = pitch_online ? 1 : 0;
        status.fold_online  = fold_online  ? 1 : 0;

        // === 2. 命令解析 ===
        TransformCmd cmd = static_cast<TransformCmd>(cfg.cmd);

        // 2.1 ABORT 命令：任意非 ABORT 状态都接受
        if (cmd == TransformCmd::ABORT && state_ != TransformState::ABORT)
        {
            enterAbort(status, TransformError::ABORT_CMD, ctrl_data,
                       yaw_fb_imu, pitch_fb_imu, fold_fb);
            cfg.cmd = static_cast<uint8_t>(TransformCmd::NONE);
            return;
        }

        // 2.2 ABORT 状态：只接受 RESET，不接受其他命令
        if (state_ == TransformState::ABORT)
        {
            if (cmd == TransformCmd::RESET)
            {
                state_ = TransformState::IDLE;
                step_elapsed_ms_ = 0;
                status.last_error = static_cast<uint8_t>(TransformError::NONE);
                cfg.cmd = static_cast<uint8_t>(TransformCmd::NONE);
            }
            // ABORT 状态：不控制 target（保持 snap 后的值，Watch 可手动接管）
            status.state            = static_cast<uint8_t>(state_);
            status.step             = 0;
            status.yaw_target_now   = ctrl_data.yaw.target_angle;
            status.pitch_target_now = ctrl_data.pitch.target_angle;
            status.fold_target_now  = ctrl_data.fold.target_angle;
            status.yaw_err          = status.yaw_target_now - yaw_fb_imu;
            status.pitch_err        = status.pitch_target_now - pitch_fb_imu;
            status.fold_err         = status.fold_target_now  - fold_fb;
            status.step_elapsed_ms  = 0;
            return;
        }

        // 2.3 IDLE / 终态：接受 EXPAND / CONTRACT
        if (state_ == TransformState::IDLE ||
            state_ == TransformState::EXPANDED ||
            state_ == TransformState::CONTRACTED)
        {
            if (cmd == TransformCmd::EXPAND)
            {
                // 进入展开：Yaw锁定 + Pitch IMU保持水平 + Fold展开
                state_ = TransformState::EXPAND_SIMULTANEOUS;
                step_elapsed_ms_ = 0;
                // 锁定 yaw target 到当前 IMU 角度（保持静止）
                yaw_locked_target_ = yaw_fb_imu;
                cfg.cmd = static_cast<uint8_t>(TransformCmd::NONE);
            }
            else if (cmd == TransformCmd::CONTRACT)
            {
                // 进入收起：Yaw锁定 + Pitch IMU保持水平 + Fold收起
                state_ = TransformState::CONTRACT_SIMULTANEOUS;
                step_elapsed_ms_ = 0;
                // 锁定 yaw target 到当前 IMU 角度（保持静止）
                yaw_locked_target_ = yaw_fb_imu;
                cfg.cmd = static_cast<uint8_t>(TransformCmd::NONE);
            }
            // 其他命令（NONE/RESET）忽略
        }

        // === 3. 状态机执行 ===
        // 标记本周期是否由 Planner 控制 target
        bool  ctrl_yaw   = false;
        bool  ctrl_pitch = false;
        bool  ctrl_fold  = false;
        float yaw_tgt    = 0.0f;
        float pitch_tgt  = 0.0f;
        float fold_tgt   = 0.0f;
        uint8_t step_idx = 0;   // 0=待机/终态, 1=执行中

        switch (state_)
        {
            // --- 待机（释放 target 给 Watch）---
            case TransformState::IDLE:
                break;

            // --- 展开：Yaw锁定 + Pitch IMU保持水平 + Fold→fold_expand ---
            case TransformState::EXPAND_SIMULTANEOUS:
            {
                ctrl_yaw   = true;
                yaw_tgt    = yaw_locked_target_;  // Yaw: 锁定到变形开始时的IMU角度
                ctrl_pitch = true;
                pitch_tgt  = 0.0f;                // Pitch: IMU 水平目标(枪口绝对俯仰=0)
                ctrl_fold  = true;
                fold_tgt   = cfg.fold_expand;     // Fold: 编码器目标值
                step_idx   = 1;

                // 离线检查（Yaw/Fold 必须在线，Pitch 离线由 Controller 层处理）
                if (!yaw_online || !fold_online)
                {
                    enterAbort(status, TransformError::MOTOR_OFFLINE,
                               ctrl_data, yaw_fb_imu, pitch_fb_imu, fold_fb);
                    return;
                }
                // 超时检查
                if (step_elapsed_ms_ > cfg.arrive_timeout_ms)
                {
                    enterAbort(status, TransformError::TIMEOUT,
                               ctrl_data, yaw_fb_imu, pitch_fb_imu, fold_fb);
                    return;
                }
                // 到位检查：只看 Fold
                if (fabsf(fold_tgt - fold_fb) < cfg.arrive_eps)
                {
                    // Fold 到位 → 进入展开终态
                    state_ = TransformState::EXPANDED;
                    step_elapsed_ms_ = 0;
                }
                break;
            }

            // --- 展开终态（释放 target 给 Watch）---
            case TransformState::EXPANDED:
                break;

            // --- 收起：Yaw锁定 + Pitch IMU保持水平 + Fold→fold_contract ---
            case TransformState::CONTRACT_SIMULTANEOUS:
            {
                ctrl_yaw   = true;
                yaw_tgt    = yaw_locked_target_;  // Yaw: 锁定到变形开始时的IMU角度
                ctrl_pitch = true;
                pitch_tgt  = 0.0f;                // Pitch: IMU 水平目标(枪口绝对俯仰=0)
                ctrl_fold  = true;
                fold_tgt   = cfg.fold_contract;   // Fold: 编码器目标值
                step_idx   = 1;

                // 离线检查
                if (!yaw_online || !fold_online)
                {
                    enterAbort(status, TransformError::MOTOR_OFFLINE,
                               ctrl_data, yaw_fb_imu, pitch_fb_imu, fold_fb);
                    return;
                }
                // 超时检查
                if (step_elapsed_ms_ > cfg.arrive_timeout_ms)
                {
                    enterAbort(status, TransformError::TIMEOUT,
                               ctrl_data, yaw_fb_imu, pitch_fb_imu, fold_fb);
                    return;
                }
                // 到位检查：只看 Fold
                if (fabsf(fold_tgt - fold_fb) < cfg.arrive_eps)
                {
                    // Fold 到位 → 进入收起终态
                    state_ = TransformState::CONTRACTED;
                    step_elapsed_ms_ = 0;
                }
                break;
            }

            // --- 收起终态（释放 target 给 Watch）---
            case TransformState::CONTRACTED:
                break;

            // --- ABORT（前面已处理，不应到达）---
            case TransformState::ABORT:
                break;
        }

        // === 4. 写入 target（仅 TRANSITION 状态）===
        // TRANSITION 状态：Planner 覆盖 target
        //   yaw.target   = yaw_locked_target_（锁定值）→ Controller IMU 闭环保持 yaw 静止
        //   pitch.target = 0 (IMU 水平) → Controller IMU 闭环保持枪口水平
        //   fold.target  = fold_expand / fold_contract (编码器目标)
        // 终态/IDLE/ABORT：不写，Watch 直接控制
        if (ctrl_yaw)
        {
            ctrl_data.yaw.target_angle = yaw_tgt;
        }
        if (ctrl_pitch)
        {
            ctrl_data.pitch.target_angle = pitch_tgt;
        }
        if (ctrl_fold)
        {
            ctrl_data.fold.target_angle = fold_tgt;
        }

        // === 5. 更新观察状态 ===
        status.state            = static_cast<uint8_t>(state_);
        status.step             = step_idx;
        status.yaw_target_now   = ctrl_yaw   ? yaw_tgt   : ctrl_data.yaw.target_angle;
        status.pitch_target_now = ctrl_pitch ? pitch_tgt : ctrl_data.pitch.target_angle;
        status.fold_target_now  = ctrl_fold  ? fold_tgt  : ctrl_data.fold.target_angle;
        status.yaw_err          = status.yaw_target_now - yaw_fb_imu;
        status.pitch_err        = status.pitch_target_now - pitch_fb_imu;
        status.fold_err         = status.fold_target_now  - fold_fb;
        status.step_elapsed_ms  = step_elapsed_ms_;

        // === 6. 步进时间（仅 TRANSITION 状态）===
        if (isTransitionState(state_))
        {
            // 防止 uint16_t 溢出（65535ms ≈ 65s，足够覆盖任何合理 timeout）
            if (step_elapsed_ms_ < 0xFFFF)
            {
                step_elapsed_ms_++;
            }
        }
    }

private:
    /**
     * @brief 进入 ABORT 状态
     *
     * 处理：
     *   1. 切换状态 → ABORT
     *   2. snap yaw/pitch/fold target 到当前 feedback（Hold 当前位置，不 limp）
     *   3. 记录错误码
     *
     * @param status     观察状态引用
     * @param err        错误原因
     * @param ctrl_data  Controller_Data（写 target = feedback）
     * @param yaw_fb     当前 yaw IMU反馈角度(rad)
     * @param pitch_fb   当前 pitch IMU反馈角度(rad)
     * @param fold_fb    当前 fold 反馈角度(rad)
     */
    void enterAbort(Transform_Status_t &status,
                    TransformError err,
                    Controller_Data_t &ctrl_data,
                    float yaw_fb, float pitch_fb, float fold_fb)
    {
        state_ = TransformState::ABORT;
        step_elapsed_ms_ = 0;

        // snap target 到 feedback（Hold 当前位置）
        //   - 不 Disable 电机（比赛场景下 limp 风险大）
        //   - PID 继续运行，Hold 当前位置，等待用户手动接管
        ctrl_data.yaw.target_angle   = yaw_fb;
        ctrl_data.pitch.target_angle = pitch_fb;
        ctrl_data.fold.target_angle  = fold_fb;

        // 更新观察状态
        status.state            = static_cast<uint8_t>(state_);
        status.step             = 0;
        status.last_error       = static_cast<uint8_t>(err);
        status.yaw_target_now   = yaw_fb;
        status.pitch_target_now = pitch_fb;
        status.fold_target_now  = fold_fb;
        status.yaw_err          = 0.0f;
        status.pitch_err        = 0.0f;
        status.fold_err         = 0.0f;
        status.step_elapsed_ms  = 0;
    }

private:
    TransformState state_;           ///< 当前状态
    uint16_t       step_elapsed_ms_; ///< 当前步骤已耗时(ms)，仅 TRANSITION 状态累加
    float          yaw_locked_target_; ///< Yaw 锁定的 target（TRANSITION 开始时的 IMU 角度）
};

} // namespace BSP::PLANNER

#endif // TRANSFORM_PLANNER_HPP
