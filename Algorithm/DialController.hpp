#ifndef DIAL_CONTROLLER_HPP
#define DIAL_CONTROLLER_HPP

/**
 * @file DialController.hpp
 * @brief 拨盘双环控制器（位置环 + 速度环 + 单击/长按状态机 + 卡弹检测）
 *
 * 设计来源：
 *   参考 H_SG_Gimbal 参考工程 ShootTask.cpp 的拨盘控制：
 *     - 位置环(位置式 PID) → 速度目标
 *     - 速度环(位置式 PID) → 力矩 raw 命令
 *     - 单击单发：wheel / mouse_left 上沿触发，或视觉 fire 脉冲触发，每次目标角度 += angle_per_shot
 *     - 长按连发：持续超过 long_press_ms 后，按 fire_hz 连续线性增加目标
 *     - 直接连发：DIRECT_AUTO首周期进入AUTO，不提交起始40°单发
 *     - 连发停火立即制动并保持当前位置；单发仍完成40°
 *     - 上电时以当前A1累计角为目标，不执行机械槽位修正
 *     - 卡弹检测：力矩饱和 + 位置误差持续 → 反转解卡
 *
 * 与参考工程差异：
 *   - 参考工程用 M2006 自带 getAddAngleDeg() 多圈累计角度
 *   - 本工程 LK4005 只反馈单圈角度，已在 LkMotor.hpp 增加 multi_turn_angle_rad_
 *     并通过 getAddAngleRad()/getAddAngleDeg() 对外暴露
 *
 * 数据流：
 *   DR16.wheel ──┐
 *                 ├─→ 状态机 ──→ target_angle(rad, 多圈累计)
 *   时间戳 ──────┘                    │
 *                                      ↓
 *   LK4005.getAddAngleRad() ──→ 位置环 PID ──→ vel_target(rad/s)
 *                                                       │
 *   LK4005.getVelocityRad()  ──→ 速度环 PID ──→ torque_raw(int16)
 *                                                       │
 *   卡弹检测 ◀───────────────────────────────── 限幅 ◀───┘
 *                                                       ↓
 *                                          LK4005.ctrl_Torque(1, raw)
 *
 * 安全策略：
 *   - 遥控器离线 / 急停(S1&&S2 DOWN) / feature_enable=0 / enabled=0
 *     → 清 PID，发送零力矩(0xA1=0)，保持 LK4005 在线反馈
 *   - 电机离线 / A1过期或反馈无效 → 零力矩，不接收发弹
 *   - 卡弹检测触发 → 反转 jam_reverse_ms 后自动恢复
 *
 * 调参建议（参考工程实测 + LK4005 特性）：
 *   ① 速度环(内环)先调：
 *      vel_kp = 50   (rad/s 误差 → raw 命令, 起步)
 *      vel_kd = 1.0  (抑制速度环震荡)
 *      vel_ki = 0    (先不加 I, 防 P 调好前积分堆积)
 *   ② 位置环(外环)后调：
 *      pos_kp = 8.0  (rad 误差 → rad/s 目标, 起步)
 *      pos_kd = 0.3  (抑制角度超调)
 *      pos_ki = 0    (拨盘是供弹, 不需要消除稳态误差)
 *   ③ 单发触发测试：
 *      Watch 中 pos_kp/pos_kd 设好后, 拨轮短暂上抬一次
 *      观察 Dial_Status.target_angle 应增加 40°, 反馈角度跟随到位
 *   ④ 卡弹检测：
 *      手堵拨盘 → 力矩饱和 + 位置误差大 → 反转解卡
 *      初调时 jam_detect_enable=0, 防止误触发
 */

#include "PID.hpp"
#include "LkMotor.hpp"
#include "DR16.hpp"
#include "Variable.hpp"

constexpr float PI = 3.14159265358979323846f;

namespace BSP::CTRL
{

// ========================================================================
// 拨盘状态机枚举
// ========================================================================
/**
 * @brief 拨盘控制状态
 *
 * 状态转移：
 *   DISABLE ──(enabled=1 & safety_ok)──→ STOP
 *   STOP ──(wheel/fire 上沿)──→ SINGLE
 *   STOP ──(DIRECT_AUTO模式下wheel/fire上沿)──→ AUTO
 *   SINGLE ──(wheel/fire 持续 > long_press_ms)──→ AUTO
 *   SINGLE ──(wheel < threshold)──→ STOP
 *   AUTO ──(wheel < threshold)──→ STOP
 *   任意状态 ──(safety_stop / enabled=0)──→ DISABLE
 *   任意状态 ──(卡弹触发)──→ STOP(反转解卡期间保持原状态, 仅命令反转)
 */
enum class DialState : uint8_t
{
    DISABLE = 0,   ///< 失能：发送零力矩，PID 清零
    STOP    = 1,   ///< 停止：target_angle 保持，PID 把拨盘拉停
    SINGLE  = 2,   ///< 单发：wheel/fire 上沿瞬间，target_angle += angle_per_shot
    AUTO    = 3,   ///< 连发：每周期按频率线性增加目标，无起步加速曲线
};


// ========================================================================
// DialController - 拨盘双环控制器
// ========================================================================
/**
 * @class DialController
 * @brief 拨盘位置环 + 速度环 + 状态机 + 卡弹检测
 *
 * 与 JointController 的区别：
 *   - JointController 处理 Yaw/Pitch/Fold 关节，输出 MIT 力矩（N·m）
 *   - DialController 处理拨盘，输出 LK4005 raw 力矩命令（-2048~2048）
 *   - DialController 内置单击/长按状态机，JointController 无此逻辑
 *   - DialController 内置卡弹检测，JointController 无此逻辑
 *
 * 调用方式：
 *   GimbalUpdate() 中每周期调用 Update()：
 *     dial_controller.Update(lk4005_motor, dr16, Dial_Config, Dial_Status);
 */
class DialController
{
public:
    // === 位置环（外环）PID - 输入 rad, 输出 rad/s ===
    PID position_pid;
    Kpid_t kpid_pos;          ///< 位置环 PID 参数(kp/ki/kd, Watch 可在线修改)

    // === 速度环（内环）PID - 输入 rad/s, 输出 raw 命令 ===
    PID velocity_pid;
    Kpid_t kpid_vel;          ///< 速度环 PID 参数(kp/ki/kd, Watch 可在线修改)

    // === 状态机字段 ===
    DialState state;          ///< 当前状态
    uint8_t   last_trigger_high;   ///< 上一周期触发是否为高电平(边沿检测用)
    uint32_t  trigger_high_since_ms; ///< 触发持续高电平的起始时间(ms)
    uint32_t  last_update_ms;     ///< 上次 Update 调用时间戳(ms, 用于 dt 计算)

    // === 累计目标角度 ===
    float target_angle_rad;   ///< 目标累计角度(rad, 多圈)
    float feedback_angle_rad; ///< 反馈累计角度(rad, 来自 LK4005.getAddAngleRad)
    uint8_t target_inited;    ///< 是否已用当前A1累计角建立相对位置目标

    // === 卡弹检测字段 ===
    uint8_t  jam_active;         ///< 卡弹解卡中标志(1=正在反转)
    uint32_t jam_start_ms;       ///< 卡弹检测触发时刻(ms)
    uint32_t jam_torque_sat_ms;  ///< 力矩饱和持续时间(ms)
    uint32_t jam_pos_err_ms;     ///< 位置误差大持续时间(ms)

    // === PID 中间结果（Watch 观察用，回写到 Dial_Status）===
    float vel_target;         ///< 速度环目标(rad/s) = 位置环输出
    float vel_feedback;       ///< 速度环反馈(rad/s) = LK4005 速度
    float pos_error;          ///< 位置环误差(rad)
    float vel_error;          ///< 速度环误差(rad/s)

    /**
     * @brief 默认构造
     */
    DialController()
        : kpid_pos(0, 0, 0), kpid_vel(0, 0, 0),
          state(DialState::DISABLE),
          last_trigger_high(0), trigger_high_since_ms(0), last_update_ms(0),
          target_angle_rad(0), feedback_angle_rad(0),
          target_inited(0),
          jam_active(0), jam_start_ms(0),
          jam_torque_sat_ms(0), jam_pos_err_ms(0),
          vel_target(0), vel_feedback(0),
          pos_error(0), vel_error(0)
    {}

    /**
     * @brief 重置控制器状态（用于 mode 切换 / 重新使能 / 卡弹恢复）
     */
    void Reset()
    {
        position_pid.clearPID();
        velocity_pid.clearPID();
        state              = DialState::DISABLE;
        last_trigger_high    = 0;
        trigger_high_since_ms = 0;
        target_angle_rad   = 0.0f;
        feedback_angle_rad = 0.0f;
        target_inited      = 0;
        jam_active         = 0;
        jam_start_ms       = 0;
        jam_torque_sat_ms  = 0;
        jam_pos_err_ms     = 0;
        vel_target         = 0.0f;
        vel_feedback       = 0.0f;
        pos_error          = 0.0f;
        vel_error          = 0.0f;
        last_update_ms     = 0;
        trigger_context_inited_ = false;
        waiting_release_ = true;
        reference_was_ever_inited_ = false;
        home_ready_ = false;
        auto_shot_phase_ = 0.0;
        raw_override_was_active_ = false;
        single_pending_ = false;
        reference_after_ms_ = HAL_GetTick();
    }

    /**
     * @brief 周期更新（1kHz 调用）
     *
     * @param motor  LK4005 电机指针(必须非空)
     * @param dr16   DR16 单例引用
     * @param cfg    Dial_Config_t 配置(Watch 可调)
     * @param status Dial_Status_t 状态(Watch 观察)
     * @param config_mode 0=默认, 1=S1上S2上, 2=S1上S2下；切换后等待释放
     *
     * 流程：
     *   1. 安全检查(电机指针 / 离线 / 急停 / feature_enable / enabled)
     *   2. 时间戳与 dt 计算
     *   3. 单击/长按状态机 → 更新 target_angle_rad
     *   4. 双环 PID: 位置环 → 速度环 → raw 命令
     *   5. 卡弹检测（力矩饱和 + 位置误差大持续 → 反转解卡）
     *   6. 限幅 raw 命令到 [-raw_output_limit, raw_output_limit]
     *   7. 发送 ctrl_Torque(1, raw) + 回写 Dial_Status
     */
    void Update(BSP::MOTOR::LK::LK4005 *motor,
                BSP::Remote::DR16 &dr16,
                Dial_Config_t &cfg,
                Dial_Status_t &status,
                uint8_t config_mode = 0)
    {
        status.config_mode = config_mode;
        status.vision_control = 0;
        status.waiting_release = waiting_release_ ? 1U : 0U;
        status.home_delta_deg = 0.0f;
        // ================================================================
        // Step 1: 电机指针安全检查
        // ================================================================
        if (motor == nullptr)
        {
            Reset();
            status.home_state = 0;
            status.slot_reference_valid = 0;
            status.home_feedback_valid = 0;
            status.home_error_deg = 0.0f;
            trigger_context_inited_ = false;
            waiting_release_ = true;
            status.waiting_release = 1;
            status.control_source = 0;  // 0=未控制
            status.torque_cmd     = 0;
            status.online         = 0;
            status.state          = (uint8_t)DialState::DISABLE;
            status.trigger_source = 0;
            status.vision_fire    = 0;
            return;
        }

        // 同步反馈数据到 Dial_Status（Watch 可观察电机在线状态）
        status.online             = motor->isConnected(1) ? 1 : 0;
        status.feedback_velocity  = motor->getVelocityRad(1);
        status.feedback_angle     = motor->getAddAngleRad(1);  // 多圈累计角度(rad)
        feedback_angle_rad        = status.feedback_angle;
        const uint32_t now_ms = HAL_GetTick();
        const auto a1 = motor->getA1Position(1);
        status.feedback_angle = feedback_angle_rad = a1.accumulated_rad;
        status.feedback_velocity = a1.velocity_rad;
        status.a1_output_deg = a1.output_phase_deg;
        if (!status.online)
        {
            target_inited = 0;
            home_ready_ = false;
            reference_after_ms_ = now_ms;
        }
        if (target_inited && raw_override_was_active_ && !cfg.raw_override_enable)
        {
            // Leaving direct torque control must not invoke mechanical correction.
            // Resume position control from the actual current position.
            HoldCurrent();
            state = DialState::STOP;
            waiting_release_ = true;
            raw_override_was_active_ = false;
        }
        const bool feedback_valid = a1.valid && std::isfinite(a1.accumulated_rad) &&
            std::isfinite(a1.velocity_rad) &&
            now_ms - a1.received_ms <= 100U;
        // A fresh A1 sample is enough to establish the local accumulated-angle
        // coordinate. Requiring the feeder to stop here made a moving/recovering
        // mechanism stay disabled even though valid feedback was available.
        const bool reference_valid = feedback_valid &&
            int32_t(a1.received_ms - reference_after_ms_) >= 0;

        // ================================================================
        // Step 2: 安全条件检查（遥控器离线 / 急停 / feature 关闭）
        // ================================================================
        using Switch = BSP::Remote::DR16::Switch;
        const bool remote_offline = dr16.IsOffline();
        const bool remote_estop   =
            (dr16.GetS1() == Switch::DOWN && dr16.GetS2() == Switch::DOWN);
        const bool vision_mode_active =
            (dr16.GetS1() == Switch::UP &&
             (dr16.GetS2() == Switch::DOWN || dr16.GetS2() == Switch::UP));
        const bool vision_fire_allowed =
            (vision_mode_active &&
             VisionComm_Data.online != 0U &&
             Friction_Data.left.online != 0U &&
             Friction_Data.right.online != 0U);
        const bool vision_fire_high =
            (vision_fire_allowed && VisionComm_Data.fire != 0U);
        const bool safety_stop = remote_offline || remote_estop;
        // Ownership is independent of fire level and vision_ready. fire=0
        // under vision ownership must not fall back to wheel/mouse.
        const bool context_changed = trigger_context_inited_ &&
            (last_config_mode_ != config_mode || last_fire_mode_ != cfg.fire_mode ||
             last_vision_control_ != vision_fire_allowed);
        last_config_mode_ = config_mode;
        last_fire_mode_ = cfg.fire_mode;
        last_vision_control_ = vision_fire_allowed;
        trigger_context_inited_ = true;
        status.vision_control = vision_fire_allowed ? 1U : 0U;

        // Clear_PID 单次触发命令
        if (cfg.clear_pid)
        {
            position_pid.clearPID();
            velocity_pid.clearPID();
            cfg.clear_pid = 0;
        }

        const bool inhibited = !cfg.feature_enable || !cfg.enabled || safety_stop ||
            !status.online ||
            (cfg.fire_mode != DialFireMode::SINGLE_THEN_AUTO &&
             cfg.fire_mode != DialFireMode::DIRECT_AUTO) ||
            cfg.slots_per_rotation != 9.0f || cfg.angle_per_shot_deg != 40.0f;

        status.home_feedback_valid = feedback_valid ? 1U : 0U;
        status.home_error_deg = 0.0f;
        if (inhibited || !feedback_valid || (!target_inited && !reference_valid))
        {
            if (inhibited || !feedback_valid) reference_after_ms_ = now_ms;
            home_ready_ = false;
            auto_shot_phase_ = 0.0;
            raw_override_was_active_ = false;
            single_pending_ = false;
            status.home_state = inhibited ? 0U : 1U;
            status.slot_reference_valid = 0;
            // 安全停止路径：清 PID，发送零力矩，保持 LK4005 在线反馈
            position_pid.clearPID();
            velocity_pid.clearPID();
            state              = DialState::DISABLE;
            last_trigger_high    = 0;
            trigger_high_since_ms = 0;
            target_inited      = 0;
            jam_active         = 0;
            jam_torque_sat_ms  = 0;
            jam_pos_err_ms     = 0;
            waiting_release_ = true;
            last_update_ms = HAL_GetTick();
            status.waiting_release = 1;

            status.wheel_input       = 0.0f;
            status.target_angle      = target_angle_rad;
            status.target_velocity   = 0.0f;
            status.error             = 0.0f;
            status.vel_target        = 0.0f;
            status.vel_error         = 0.0f;
            status.pid_p             = 0.0f;
            status.pid_i             = 0.0f;
            status.pid_d             = 0.0f;
            status.torque_cmd        = 0;
            status.control_source    = 0;
            status.state             = (uint8_t)DialState::DISABLE;
            status.jam_detected      = 0;
            status.trigger_source    = 0;
            status.vision_fire       = 0;
            motor->ctrl_Torque(1, 0);  // 零力矩保反馈
            return;
        }

        // Establish only a local accumulated-angle reference. Startup never moves
        // the mechanism to a calibrated phase or a nominal 40-degree slot.
        if (!target_inited)
        {
            target_angle_rad = a1.accumulated_rad;
            trajectory_angle_rad_ = target_angle_rad;
            single_pending_ = false;
            status.home_delta_deg = 0.0f;
            home_ready_ = true;
            state = DialState::STOP;
            last_trigger_high = 0;
            auto_shot_phase_ = 0.0;
            // The first startup reference may immediately accept a held wheel.
            // A later re-reference follows a safety interruption and still
            // requires release to prevent an unintended restart.
            waiting_release_ = reference_was_ever_inited_;
            reference_was_ever_inited_ = true;
            position_pid.clearPID();
            velocity_pid.clearPID();
            vel_target = vel_error = pos_error = 0.0f;
            target_inited = 1;
        }

        status.slot_reference_valid = 1;
        const auto is_at_target = [&]() {
            status.home_error_deg = (target_angle_rad - feedback_angle_rad) * (180.0f / PI);
            return feedback_valid && fabsf(target_angle_rad - feedback_angle_rad) <= target_tolerance_rad_ &&
                fabsf(status.feedback_velocity) < 0.1f;
        };
        const bool at_target = is_at_target();
        if (single_pending_ && at_target) single_pending_ = false;

        // ================================================================
        // Step 3: 时间戳与 dt 计算
        // ================================================================
        // 使用 HAL_GetTick() 获取毫秒级时间戳
        if (last_update_ms == 0) last_update_ms = now_ms;
        uint32_t dt_ms = now_ms - last_update_ms;
        last_update_ms = now_ms;
        // 限幅 dt，避免首次调用或长时间挂起后 dt 过大导致目标角度跳变
        if (dt_ms > 50) dt_ms = 50;

        // ================================================================
        // Step 4: 状态机 - 单击/长按判定
        // ================================================================
        // 拨轮读取与阈值处理
        //   wheel 范围 [-1, 1]，参考工程用 wheel > 0 触发(向下拨)
        //   本工程用 wheel > threshold 触发，正常发弹统一增加 40°。
        // Vision fire is allowed only while the DR16 is still in vision-enabled
        // switch positions; leaving vision mode immediately drops this path.
        // SINGLE_THEN_AUTO commits a shot on an edge; DIRECT_AUTO integrates only
        // while fire is high, including short pulses, without an initial 40-degree step.
        float wheel = (float)dr16.GetWheel();
        const bool mouse_left_high = dr16.GetMouse().left;
        float wheel_threshold = clampFloatCfg(cfg.wheel_start_threshold, 0.0f, 0.99f);
        bool  wheel_high = (wheel > wheel_threshold);
        const bool vision_fire_mode = vision_fire_allowed;
        const bool raw_trigger_high = vision_fire_mode ? vision_fire_high : (mouse_left_high || wheel_high);

        if (jam_active && !raw_trigger_high && !single_pending_)
        {
            // Releasing AUTO also cancels its ongoing reverse unjam action.
            jam_active = 0;
            status.jam_detected = 0;
            HoldCurrent();
        }
        // A fresh A1 coordinate is the only position prerequisite; there is no
        // startup or idle slot-correction state.
        const bool firing_ready = target_inited && feedback_valid &&
            !jam_active && !cfg.raw_override_enable;
        if (state == DialState::AUTO &&
            (context_changed || !firing_ready || !raw_trigger_high))
        {
            // Cancel all unexecuted lead once; the velocity loop brakes immediately.
            HoldCurrent();
        }
        if (context_changed)
        {
            auto_shot_phase_ = 0.0;
            if (state != DialState::DISABLE) state = DialState::STOP;
            last_trigger_high = 0;
            trigger_high_since_ms = 0;
            waiting_release_ = true;
        }
        if (!firing_ready)
        {
            waiting_release_ = true;
            auto_shot_phase_ = 0.0;
            state = DialState::STOP;
        }
        if (firing_ready && waiting_release_ && !raw_trigger_high) waiting_release_ = false;
        const bool trigger_high = !waiting_release_ && raw_trigger_high;
        const uint8_t trigger_source = !trigger_high ? 0U
            : (vision_fire_mode ? 2U : (mouse_left_high ? 3U : 1U));
        status.waiting_release = waiting_release_ ? 1U : 0U;

        if (trigger_high)
        {
            home_ready_ = true;
        }

        switch (state)
        {
            case DialState::DISABLE:
            case DialState::STOP:
                state = DialState::STOP;
                if (trigger_high && !last_trigger_high)
                {
                    trigger_high_since_ms = now_ms;
                    if (cfg.fire_mode == DialFireMode::DIRECT_AUTO)
                    {
                        state = DialState::AUTO;
                        single_pending_ = false;
                        auto_shot_phase_ = 0.0;
                    }
                    else
                    {
                        state = DialState::SINGLE;
                        CommitShot(status);
                    }
                }
                break;
            case DialState::SINGLE:
                if (!trigger_high)
                {
                    // A single pulse still completes its full 40-degree target.
                    state = DialState::STOP;
                    trigger_high_since_ms = 0;
                }
                else if (now_ms - trigger_high_since_ms >= cfg.long_press_ms)
                {
                    state = DialState::AUTO;
                    single_pending_ = false;
                    auto_shot_phase_ = 0.0;
                }
                break;
            case DialState::AUTO:
                if (!trigger_high)
                {
                    state = DialState::STOP;
                    trigger_high_since_ms = 0;
                    auto_shot_phase_ = 0.0;
                }
                break;
        }

        // Integrate on every AUTO cycle, including the first: no acceleration ramp.
        if (state == DialState::AUTO && trigger_high)
        {
            float fire_hz = cfg.auto_fire_hz;
            if (!vision_fire_mode && wheel_high && cfg.wheel_to_hz > 0.0f)
            {
                const float wheel_norm = clampFloatCfg(
                    (wheel - wheel_threshold) / (1.0f - wheel_threshold), 0.0f, 1.0f);
                fire_hz = wheel_norm * cfg.wheel_to_hz;
            }
            if (std::isfinite(fire_hz) && fire_hz > 0.0f)
            {
                const double shots = double(fminf(fire_hz, 50.0f)) * double(dt_ms) / 1000.0;
                trajectory_angle_rad_ += shots * shot_step_rad_;
                target_angle_rad = float(trajectory_angle_rad_);
                auto_shot_phase_ += shots;
                while (auto_shot_phase_ + 1e-9 >= 1.0)
                {
                    ++status.shot_count; // Planned whole slots, not sensed bullets.
                    auto_shot_phase_ -= 1.0;
                }
            }
        }
        last_trigger_high = trigger_high ? 1 : 0;

        status.wheel_input = (trigger_high && !vision_fire_mode && wheel_high && !mouse_left_high) ? wheel : 0.0f;
        status.trigger_source = trigger_source;
        status.vision_fire = vision_fire_mode ? VisionComm_Data.fire : 0U;
        status.state = (uint8_t)state;
        const bool target_reached = is_at_target();
        status.home_state = target_reached ? 3U : 2U;

        // ================================================================
        // Step 5: raw_override 模式（绕过 PID，直接发送原始命令）
        // ================================================================
        if (cfg.raw_override_enable && home_ready_)
        {
            raw_override_was_active_ = true;
            int16_t raw_cmd = toLkRawCmd((float)cfg.raw_override_cmd);
            position_pid.clearPID();
            velocity_pid.clearPID();
            status.target_velocity = 0.0f;
            status.error           = 0.0f;
            status.vel_target      = 0.0f;
            status.vel_error       = 0.0f;
            status.pid_p           = 0.0f;
            status.pid_i           = 0.0f;
            status.pid_d           = 0.0f;
            status.torque_cmd      = raw_cmd;
            status.control_source  = 2;
            status.home_state = 2; // Direct torque is not a position-holding mode.
            motor->ctrl_Torque(1, raw_cmd);
            return;
        }

        // ================================================================
        // Step 6: 双环 PID 计算
        // ================================================================
        // 同步 PID 参数（Watch 在线调参）
        kpid_pos.kp = Dial_PID_Config.pos_kp;
        kpid_pos.ki = Dial_PID_Config.pos_ki;
        kpid_pos.kd = Dial_PID_Config.pos_kd;
        kpid_vel.kp = Dial_PID_Config.vel_kp;
        kpid_vel.ki = Dial_PID_Config.vel_ki;
        kpid_vel.kd = Dial_PID_Config.vel_kd;
        position_pid.pid.Break_I = Dial_PID_Config.pos_break_i;
        position_pid.pid.MixI    = Dial_PID_Config.pos_limit_i;
        velocity_pid.pid.Break_I = Dial_PID_Config.vel_break_i;
        velocity_pid.pid.MixI    = Dial_PID_Config.vel_limit_i;

        // 外环：位置环 PID（位置式）
        //   输入: target_angle_rad (rad), feedback_angle_rad (rad)
        //   输出: vel_target (rad/s), 限幅到 [-pos_vel_limit, pos_vel_limit]
        float pos_vel_limit = clampFloatCfg(Dial_PID_Config.pos_vel_limit, 0.0f, 100.0f);
        vel_target = (float)position_pid.GetPidPos(
            kpid_pos,
            (double)target_angle_rad,
            (double)feedback_angle_rad,
            (double)pos_vel_limit);
        pos_error = (float)position_pid.GetErr();

        // 内环：速度环 PID（位置式）
        //   输入: vel_target (rad/s), vel_feedback (rad/s)
        //   输出: raw 命令, 限幅到 [-raw_output_limit, raw_output_limit]
        vel_feedback = status.feedback_velocity;
        float raw_limit = clampFloatCfg(Dial_PID_Config.raw_output_limit, 0.0f, 2048.0f);
        float raw_output = (float)velocity_pid.GetPidPos(
            kpid_vel,
            (double)vel_target,
            (double)vel_feedback,
            (double)raw_limit);
        vel_error = (float)velocity_pid.GetErr();
        // ================================================================
        // Step 7: 卡弹检测
        // ================================================================
        //   触发条件（同时满足且持续 jam_duration_ms）：
        //     ① |torque_cmd| 接近 raw_output_limit (饱和)
        //     ② |pos_error| > jam_err_threshold (rad)
        //   解卡动作：
        //     反转 jam_reverse_ms 时间，期间命令取反方向
        //     解卡完成后清状态，恢复正常控制
        if (cfg.jam_detect_enable && home_ready_ &&
            (trigger_high || single_pending_ || jam_active))
        {
            // 检测条件
            bool torque_saturated =
                (fabsf(raw_output) > cfg.jam_torque_threshold * raw_limit);
            bool pos_err_large =
                (fabsf(pos_error) > cfg.jam_err_threshold);

            if (jam_active)
            {
                // 解卡中：检查是否到时间
                if ((now_ms - jam_start_ms) >= cfg.jam_reverse_ms)
                {
                    // 解卡完成：清 PID，继续完成原有槽位目标。
                    jam_active        = 0;
                    jam_start_ms      = 0;
                    jam_torque_sat_ms = 0;
                    jam_pos_err_ms    = 0;
                    position_pid.clearPID();
                    velocity_pid.clearPID();
                    status.jam_detected = 0;
                }
            }
            else
            {
                // 累加持续时长
                if (torque_saturated) jam_torque_sat_ms += dt_ms;
                else                  jam_torque_sat_ms  = 0;
                if (pos_err_large)    jam_pos_err_ms    += dt_ms;
                else                  jam_pos_err_ms     = 0;

                // 同时满足且持续 → 触发解卡
                if (torque_saturated && pos_err_large &&
                    jam_torque_sat_ms >= cfg.jam_duration_ms &&
                    jam_pos_err_ms    >= cfg.jam_duration_ms)
                {
                    jam_active   = 1;
                    jam_start_ms = now_ms;
                    status.jam_detected = 1;
                }
            }

            // 解卡中：发送反转命令
            if (jam_active)
            {
                raw_output = (float)cfg.jam_reverse_torque;  // 反转力矩
                // 注意：jam_reverse_torque 正负由用户在 Watch 标定（参考工程用 +250）
            }
        }
        else
        {
            status.jam_detected = 0;
            jam_active = 0;
        }

        // ================================================================
        // Step 8: 限幅 + 发送 CAN 命令 + 回写 Dial_Status
        // ================================================================
        int16_t raw_cmd = toLkRawCmd(raw_output);

        // 回写状态供 Watch 观察
        status.target_angle     = target_angle_rad;
        status.target_velocity  = vel_target;
        status.error            = pos_error;
        status.vel_target       = vel_target;
        status.vel_error        = vel_error;
        status.pid_p            = (float)velocity_pid.pid.p;
        status.pid_i            = (float)velocity_pid.pid.i;
        status.pid_d            = (float)velocity_pid.pid.d;
        status.torque_cmd       = raw_cmd;
        status.control_source   = 1;

        motor->ctrl_Torque(1, raw_cmd);
    }

private:
    static constexpr double shot_step_rad_ = 40.0 * 3.14159265358979323846 / 180.0;
    static constexpr float target_tolerance_rad_ = 0.3f * 3.14159265358979323846f / 180.0f;
    double trajectory_angle_rad_ = 0.0;
    bool single_pending_ = false;
    double auto_shot_phase_ = 0.0;
    bool home_ready_ = false;
    uint32_t reference_after_ms_ = 0;
    bool raw_override_was_active_ = false;

    void CommitShot(Dial_Status_t &status)
    {
        ++status.shot_count;
        trajectory_angle_rad_ += shot_step_rad_;
        target_angle_rad = float(trajectory_angle_rad_);
        single_pending_ = true;
    }

    void HoldCurrent()
    {
        target_angle_rad = feedback_angle_rad;
        trajectory_angle_rad_ = feedback_angle_rad;
        auto_shot_phase_ = 0.0;
        single_pending_ = false;
        position_pid.clearPID();
        velocity_pid.clearPID();
        jam_torque_sat_ms = jam_pos_err_ms = 0;
    }

    bool trigger_context_inited_ = false;
    bool waiting_release_ = true;
    bool reference_was_ever_inited_ = false;
    uint8_t last_config_mode_ = 0;
    DialFireMode last_fire_mode_ = DialFireMode::SINGLE_THEN_AUTO;
    bool last_vision_control_ = false;

    /**
     * @brief 浮点值限幅（私有，避免与 GimbalInit.cpp 中 clampFloat 重名）
     */
    static inline float clampFloatCfg(float value, float min_value, float max_value)
    {
        if (value > max_value) return max_value;
        if (value < min_value) return min_value;
        return value;
    }

    /**
     * @brief 浮点 raw 命令转 int16_t（四舍五入 + 限幅 [-2048, 2048]）
     */
    static inline int16_t toLkRawCmd(float value)
    {
        if (value >  2048.0f) value =  2048.0f;
        if (value < -2048.0f) value = -2048.0f;
        if (value >= 0.0f) return (int16_t)(value + 0.5f);
        return (int16_t)(value - 0.5f);
    }
};

} // namespace BSP::CTRL

#endif // DIAL_CONTROLLER_HPP
