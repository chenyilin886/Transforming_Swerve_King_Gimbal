#ifndef TRANSFORM_PLANNER_HPP
#define TRANSFORM_PLANNER_HPP

/**
 * @file TransformPlanner.hpp
 * @brief Stage05 变形动作规划器（Motion Planner + Morphology Manager 合并实现）
 *
 * 设计背景：
 *   RM2026 三关节可变形云台，Fold 会改变整个云台几何结构。因此展开/收起
 *   必须严格串行执行，避免 Fold 转动过程中干扰 Pitch（机械干涉 / PID 冲击）。
 *
 * 用户动作序列要求（严格串行，先后执行，不同时转动）：
 *
 *   展开(EXPAND):
 *     ① Pitch 先摆到 pitch_expand（Fold 展开后 Pitch 的水平位）
 *     ② 等待 Pitch 到位
 *     ③ Fold 展开到 fold_expand
 *     ④ 等待 Fold 到位 → EXPANDED 终态
 *
 *   收起(CONTRACT):
 *     ① Fold 先收回 fold_contract（最小角度 0）
 *     ② 等待 Fold 到位
 *     ③ Pitch 收回到 pitch_contract（最收缩角度）
 *     ④ 等待 Pitch 到位 → CONTRACTED 终态
 *
 * 模块落点（按 System Prompt 分层）：
 *   本模块合并了 Motion Planner（动作顺序/等待/超时）和 Morphology Manager
 *   （形态状态管理）两层职责。Stage05 仅两段串行动作，合并实现避免"谁拥有
 *   target"的歧义。未来形态变复杂（半展开/运输/维修等）可再拆出独立的
 *   MorphologyManager，接口已为此预留（state 枚举可扩展）。
 *
 * 数据流（与现有架构兼容，仅插入 Step 2.5）：
 *   Watch.cmd ──→ TransformPlanner.Update()
 *                  │
 *                  ├─ 读 JointManager feedback（pitch/fold real_angle）
 *                  ├─ 状态机转移（IDLE/TRANSITION/终态/ABORT）
 *                  └─ TRANSITION 状态: 写 Controller_Data.pitch/fold.target_angle
 *                     终态/IDLE/ABORT: 不写（释放给 Watch）
 *
 * Target 归属规则（关键设计）：
 *   - TRANSITION 状态（4 个中间态）：Planner 每周期覆盖 target_angle
 *   - 终态（EXPANDED/CONTRACTED）/ IDLE / ABORT：Planner 不写 target
 *     Watch 可直接手动控制 → 方便到位后手动微调 / 调 PID
 *   - 进入终态瞬间，Controller_Data 中保留的就是终点目标值（如 pitch_expand）
 *
 * ABORT 安全策略：
 *   - 触发条件：用户 cmd=ABORT / 单步超时 / 电机离线
 *   - 处理：snap pitch/fold target 到当前 feedback（Hold 当前位置，不 limp）
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
 *   IDLE ──cmd=EXPAND──→ EXPAND_PITCH_PRE ──→ EXPAND_FOLD_DEPLOY ──→ EXPANDED
 *   EXPANDED ──cmd=CONTRACT──→ CONTRACT_FOLD_RETURN ──→ CONTRACT_PITCH_RETURN ──→ CONTRACTED
 *   CONTRACTED ──cmd=EXPAND──→ EXPAND_PITCH_PRE ──→ ...
 *   任意 TRANSITION ──cmd=ABORT/超时/离线──→ ABORT ──cmd=RESET──→ IDLE
 *
 * @note 数值固定，Watch 中可直接用整数观察（0..7）
 */
enum class TransformState : uint8_t
{
    IDLE                   = 0,  // 待机（上电默认）
    EXPAND_PITCH_PRE       = 1,  // 展开第1步: pitch → pitch_expand
    EXPAND_FOLD_DEPLOY     = 2,  // 展开第2步: fold → fold_expand
    EXPANDED               = 3,  // 展开终态（释放 target 给 Watch）
    CONTRACT_FOLD_RETURN  = 4,  // 收起第1步: fold → fold_contract
    CONTRACT_PITCH_RETURN = 5,  // 收起第2步: pitch → pitch_contract
    CONTRACTED             = 6,  // 收起终态（释放 target 给 Watch）
    ABORT                  = 7,  // 异常退出（Hold 当前位置）
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
    return s == TransformState::EXPAND_PITCH_PRE ||
           s == TransformState::EXPAND_FOLD_DEPLOY ||
           s == TransformState::CONTRACT_FOLD_RETURN ||
           s == TransformState::CONTRACT_PITCH_RETURN;
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
 *   2. 按状态机执行两段串行动作（Pitch 先 → Fold 后 / Fold 先 → Pitch 后）
 *   3. 到位检测（误差阈值 + 超时）
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
          step_elapsed_ms_(0)
    {}

    /**
     * @brief 获取当前状态
     */
    TransformState getState() const { return state_; }

    /**
     * @brief 周期更新（1kHz 调用）
     *
     * 流程：
     *   1. 读取 Joint feedback（pitch/fold real_angle + online）
     *   2. 命令解析（ABORT 优先级最高，ABORT 状态只接受 RESET）
     *   3. 状态机执行：
     *      - 离线/超时检查 → ABORT
     *      - 到位检查 → 进入下一状态
     *   4. 写入 target（仅 TRANSITION 状态）
     *   5. 更新观察状态（Transform_Status）
     *   6. 步进时间（step_elapsed_ms++）
     *
     * @param jm         JointManager（读 pitch/fold feedback）
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
        float pitch_fb = jm.pitch.getRealAngle();
        float fold_fb  = jm.fold.getRealAngle();
        bool  pitch_online = jm.pitch.isOnline();
        bool  fold_online  = jm.fold.isOnline();

        status.pitch_online = pitch_online ? 1 : 0;
        status.fold_online  = fold_online  ? 1 : 0;

        // === 2. 命令解析 ===
        TransformCmd cmd = static_cast<TransformCmd>(cfg.cmd);

        // 2.1 ABORT 命令：任意非 ABORT 状态都接受
        if (cmd == TransformCmd::ABORT && state_ != TransformState::ABORT)
        {
            enterAbort(status, TransformError::ABORT_CMD, ctrl_data, pitch_fb, fold_fb);
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
            status.pitch_target_now = ctrl_data.pitch.target_angle;
            status.fold_target_now  = ctrl_data.fold.target_angle;
            status.pitch_err        = status.pitch_target_now - pitch_fb;
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
                // 进入展开第一步：Pitch 先摆到 pitch_expand
                state_ = TransformState::EXPAND_PITCH_PRE;
                step_elapsed_ms_ = 0;
                cfg.cmd = static_cast<uint8_t>(TransformCmd::NONE);
            }
            else if (cmd == TransformCmd::CONTRACT)
            {
                // 进入收起第一步：Fold 先收回 fold_contract
                state_ = TransformState::CONTRACT_FOLD_RETURN;
                step_elapsed_ms_ = 0;
                cfg.cmd = static_cast<uint8_t>(TransformCmd::NONE);
            }
            // 其他命令（NONE/RESET）忽略
        }

        // === 3. 状态机执行 ===
        // 标记本周期是否由 Planner 控制 target
        bool  ctrl_pitch = false;
        bool  ctrl_fold  = false;
        float pitch_tgt  = 0.0f;
        float fold_tgt   = 0.0f;
        uint8_t step_idx = 0;   // 0=待机/终态, 1=第一步, 2=第二步

        switch (state_)
        {
            // --- 待机（释放 target 给 Watch）---
            case TransformState::IDLE:
                break;

            // --- 展开第1步：Pitch 先摆到 pitch_expand ---
            case TransformState::EXPAND_PITCH_PRE:
            {
                ctrl_pitch = true;
                pitch_tgt  = cfg.pitch_expand;
                step_idx   = 1;

                // 离线检查
                if (!pitch_online)
                {
                    enterAbort(status, TransformError::MOTOR_OFFLINE,
                               ctrl_data, pitch_fb, fold_fb);
                    return;
                }
                // 超时检查（用 > 而非 >=，避免边界误触发）
                if (step_elapsed_ms_ > cfg.arrive_timeout_ms)
                {
                    enterAbort(status, TransformError::TIMEOUT,
                               ctrl_data, pitch_fb, fold_fb);
                    return;
                }
                // 到位检查
                if (fabsf(pitch_tgt - pitch_fb) < cfg.arrive_eps)
                {
                    // 进入展开第2步：Fold 展开
                    state_ = TransformState::EXPAND_FOLD_DEPLOY;
                    step_elapsed_ms_ = 0;
                }
                break;
            }

            // --- 展开第2步：Fold 展开（Pitch 保持 pitch_expand）---
            case TransformState::EXPAND_FOLD_DEPLOY:
            {
                ctrl_pitch = true;
                pitch_tgt  = cfg.pitch_expand;   // Pitch 保持展开位，防止漂移
                ctrl_fold  = true;
                fold_tgt   = cfg.fold_expand;
                step_idx   = 2;

                if (!fold_online)
                {
                    enterAbort(status, TransformError::MOTOR_OFFLINE,
                               ctrl_data, pitch_fb, fold_fb);
                    return;
                }
                if (step_elapsed_ms_ > cfg.arrive_timeout_ms)
                {
                    enterAbort(status, TransformError::TIMEOUT,
                               ctrl_data, pitch_fb, fold_fb);
                    return;
                }
                if (fabsf(fold_tgt - fold_fb) < cfg.arrive_eps)
                {
                    // 进入展开终态
                    state_ = TransformState::EXPANDED;
                    step_elapsed_ms_ = 0;
                }
                break;
            }

            // --- 展开终态（释放 target 给 Watch）---
            case TransformState::EXPANDED:
                break;

            // --- 收起第1步：Fold 先收回（Pitch 保持 pitch_expand）---
            case TransformState::CONTRACT_FOLD_RETURN:
            {
                ctrl_pitch = true;
                pitch_tgt  = cfg.pitch_expand;   // Pitch 保持展开位，避免 Fold 收回时干涉
                ctrl_fold  = true;
                fold_tgt   = cfg.fold_contract;
                step_idx   = 1;

                if (!fold_online)
                {
                    enterAbort(status, TransformError::MOTOR_OFFLINE,
                               ctrl_data, pitch_fb, fold_fb);
                    return;
                }
                if (step_elapsed_ms_ > cfg.arrive_timeout_ms)
                {
                    enterAbort(status, TransformError::TIMEOUT,
                               ctrl_data, pitch_fb, fold_fb);
                    return;
                }
                if (fabsf(fold_tgt - fold_fb) < cfg.arrive_eps)
                {
                    // 进入收起第2步：Pitch 收回
                    state_ = TransformState::CONTRACT_PITCH_RETURN;
                    step_elapsed_ms_ = 0;
                }
                break;
            }

            // --- 收起第2步：Pitch 收回（Fold 保持 fold_contract）---
            case TransformState::CONTRACT_PITCH_RETURN:
            {
                ctrl_pitch = true;
                pitch_tgt  = cfg.pitch_contract;
                ctrl_fold  = true;
                fold_tgt   = cfg.fold_contract;   // Fold 保持收起位
                step_idx   = 2;

                if (!pitch_online)
                {
                    enterAbort(status, TransformError::MOTOR_OFFLINE,
                               ctrl_data, pitch_fb, fold_fb);
                    return;
                }
                if (step_elapsed_ms_ > cfg.arrive_timeout_ms)
                {
                    enterAbort(status, TransformError::TIMEOUT,
                               ctrl_data, pitch_fb, fold_fb);
                    return;
                }
                if (fabsf(pitch_tgt - pitch_fb) < cfg.arrive_eps)
                {
                    // 进入收起终态
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
        // 终态/IDLE/ABORT：不写，Watch 直接控制
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
        status.pitch_target_now = ctrl_pitch ? pitch_tgt : ctrl_data.pitch.target_angle;
        status.fold_target_now  = ctrl_fold  ? fold_tgt  : ctrl_data.fold.target_angle;
        status.pitch_err        = status.pitch_target_now - pitch_fb;
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
     *   2. snap pitch/fold target 到当前 feedback（Hold 当前位置，不 limp）
     *   3. 记录错误码
     *
     * @param status     观察状态引用
     * @param err        错误原因
     * @param ctrl_data  Controller_Data（写 target = feedback）
     * @param pitch_fb   当前 pitch 反馈角度(rad)
     * @param fold_fb    当前 fold 反馈角度(rad)
     */
    void enterAbort(Transform_Status_t &status,
                    TransformError err,
                    Controller_Data_t &ctrl_data,
                    float pitch_fb, float fold_fb)
    {
        state_ = TransformState::ABORT;
        step_elapsed_ms_ = 0;

        // snap target 到 feedback（Hold 当前位置）
        //   - 不 Disable 电机（比赛场景下 limp 风险大）
        //   - PID 继续运行，Hold 当前位置，等待用户手动接管
        ctrl_data.pitch.target_angle = pitch_fb;
        ctrl_data.fold.target_angle  = fold_fb;

        // 更新观察状态
        status.state            = static_cast<uint8_t>(state_);
        status.step             = 0;
        status.last_error       = static_cast<uint8_t>(err);
        status.pitch_target_now = pitch_fb;
        status.fold_target_now  = fold_fb;
        status.pitch_err        = 0.0f;
        status.fold_err         = 0.0f;
        status.step_elapsed_ms  = 0;
    }

private:
    TransformState state_;           ///< 当前状态
    uint16_t       step_elapsed_ms_;  ///< 当前步骤已耗时(ms)，仅 TRANSITION 状态累加
};

} // namespace BSP::PLANNER

#endif // TRANSFORM_PLANNER_HPP
