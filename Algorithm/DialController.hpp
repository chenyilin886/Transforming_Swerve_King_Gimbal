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
 *     - 单击单发：wheel 上沿触发，每次目标角度 -= angle_per_shot
 *     - 长按连发：wheel 持续超过 long_press_ms，按 fire_hz 累加目标角度
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
 *   - 电机离线 → 不发送 CAN 命令（避免空指针解引用）
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
 *      观察 Dial_Status.target_angle 应减 40°, 反馈角度跟随到位
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
 *   STOP ──(wheel 上沿)──→ SINGLE
 *   SINGLE ──(wheel 持续 > long_press_ms)──→ AUTO
 *   SINGLE ──(wheel < threshold)──→ STOP
 *   AUTO ──(wheel < threshold)──→ STOP
 *   任意状态 ──(safety_stop / enabled=0)──→ DISABLE
 *   任意状态 ──(卡弹触发)──→ STOP(反转解卡期间保持原状态, 仅命令反转)
 */
enum class DialState : uint8_t
{
    DISABLE = 0,   ///< 失能：发送零力矩，PID 清零
    STOP    = 1,   ///< 停止：target_angle 保持，PID 把拨盘拉停
    SINGLE  = 2,   ///< 单发：wheel 上沿瞬间，target_angle -= angle_per_shot
    AUTO    = 3,   ///< 连发：wheel 持续触发，target_angle -= hz_to_angle(fire_hz) * dt
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
    uint8_t   last_wheel_high;   ///< 上一周期 wheel 是否高于阈值(边沿检测用)
    uint32_t  wheel_high_since_ms; ///< wheel 持续高于阈值的起始时间(ms)
    uint32_t  last_update_ms;     ///< 上次 Update 调用时间戳(ms, 用于 dt 计算)

    // === 累计目标角度 ===
    float target_angle_rad;   ///< 目标累计角度(rad, 多圈)
    float feedback_angle_rad; ///< 反馈累计角度(rad, 来自 LK4005.getAddAngleRad)

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
          last_wheel_high(0), wheel_high_since_ms(0), last_update_ms(0),
          target_angle_rad(0), feedback_angle_rad(0),
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
        last_wheel_high    = 0;
        wheel_high_since_ms = 0;
        target_angle_rad   = 0.0f;
        feedback_angle_rad = 0.0f;
        jam_active         = 0;
        jam_start_ms       = 0;
        jam_torque_sat_ms  = 0;
        jam_pos_err_ms     = 0;
        vel_target         = 0.0f;
        vel_feedback       = 0.0f;
        pos_error          = 0.0f;
        vel_error          = 0.0f;
    }

    /**
     * @brief 周期更新（1kHz 调用）
     *
     * @param motor  LK4005 电机指针(必须非空)
     * @param dr16   DR16 单例引用
     * @param cfg    Dial_Config_t 配置(Watch 可调)
     * @param status Dial_Status_t 状态(Watch 观察)
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
                Dial_Status_t &status)
    {
        // ================================================================
        // Step 1: 电机指针安全检查
        // ================================================================
        if (motor == nullptr)
        {
            status.control_source = 0;  // 0=未控制
            status.torque_cmd     = 0;
            status.online         = 0;
            status.state          = (uint8_t)DialState::DISABLE;
            return;
        }

        // 同步反馈数据到 Dial_Status（Watch 可观察电机在线状态）
        status.online             = motor->isConnected(1) ? 1 : 0;
        status.feedback_velocity  = motor->getVelocityRad(1);
        status.feedback_angle     = motor->getAddAngleRad(1);  // 多圈累计角度(rad)
        feedback_angle_rad        = status.feedback_angle;

        // ================================================================
        // Step 2: 安全条件检查（遥控器离线 / 急停 / feature 关闭）
        // ================================================================
        using Switch = BSP::Remote::DR16::Switch;
        const bool remote_offline = dr16.IsOffline();
        const bool remote_estop   =
            (dr16.GetS1() == Switch::DOWN && dr16.GetS2() == Switch::DOWN);
        const bool safety_stop = remote_offline || remote_estop;

        // Clear_PID 单次触发命令
        if (cfg.clear_pid)
        {
            position_pid.clearPID();
            velocity_pid.clearPID();
            cfg.clear_pid = 0;
        }

        if (!cfg.feature_enable || !cfg.enabled || safety_stop)
        {
            // 安全停止路径：清 PID，发送零力矩，保持 LK4005 在线反馈
            position_pid.clearPID();
            velocity_pid.clearPID();
            state              = DialState::DISABLE;
            last_wheel_high    = 0;
            wheel_high_since_ms = 0;
            jam_active         = 0;
            jam_torque_sat_ms  = 0;
            jam_pos_err_ms     = 0;

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
            status.shot_count        = status.shot_count;  // 保留累计
            motor->ctrl_Torque(1, 0);  // 零力矩保反馈
            return;
        }

        // ================================================================
        // Step 3: 时间戳与 dt 计算
        // ================================================================
        // 使用 HAL_GetTick() 获取毫秒级时间戳
        uint32_t now_ms = HAL_GetTick();
        if (last_update_ms == 0) last_update_ms = now_ms;
        uint32_t dt_ms = now_ms - last_update_ms;
        last_update_ms = now_ms;
        // 限幅 dt，避免首次调用或长时间挂起后 dt 过大导致目标角度跳变
        if (dt_ms > 50) dt_ms = 50;
        float dt_s = (float)dt_ms * 0.001f;

        // ================================================================
        // Step 4: 状态机 - 单击/长按判定
        // ================================================================
        // 拨轮读取与阈值处理
        //   wheel 范围 [-1, 1]，参考工程用 wheel > 0 触发(向下拨)
        //   本工程也用 wheel > threshold 触发，方向由 wheel_to_speed 正负决定
        //   （wheel_to_speed < 0 表示拨盘反转，对应参考工程 Dail_target_pos -= angle_per_shot）
        float wheel = (float)dr16.GetWheel();
        float wheel_threshold = clampFloatCfg(cfg.wheel_start_threshold, 0.0f, 0.99f);
        bool  wheel_high = (wheel > wheel_threshold);

        // 状态机转移
        switch (state)
        {
            case DialState::DISABLE:
                // 首次进入：初始化目标为当前反馈角度（bumpless transfer）
                if (target_angle_rad == 0.0f && feedback_angle_rad != 0.0f)
                {
                    target_angle_rad = feedback_angle_rad;
                }
                state = DialState::STOP;
                last_wheel_high = 0;
                wheel_high_since_ms = 0;
                break;

            case DialState::STOP:
                if (wheel_high && !last_wheel_high)
                {
                    // 上沿：触发单发
                    state = DialState::SINGLE;
                    wheel_high_since_ms = now_ms;
                    // 单发：目标角度增加 angle_per_shot_deg（一个弹槽）
                    //   方向说明:
                    //     本工程 LK4005 raw 命令为正 → 电机正转 → feedback_angle 增大
                    //     (已用 raw_override 模式实测确认)
                    //     拨盘供弹方向 = 电机正转方向, 因此 target_angle 应往正方向走
                    //     使位置环输出正 raw 命令, 电机正转, feedback_angle 增大并朝 target 靠近
                    //   与参考工程差异:
                    //     参考工程 ShootTask.cpp 用 -= 是因其电机正转方向与供弹方向相反
                    float angle_per_shot_rad =
                        cfg.angle_per_shot_deg * (PI / 180.0f);
                    target_angle_rad += angle_per_shot_rad;
                    status.shot_count++;
                }
                break;

            case DialState::SINGLE:
                if (!wheel_high)
                {
                    // 拨轮释放 → 回到 STOP
                    // 注意：不重置 target，单发是"已提交"动作，必须走完 40°
                    state = DialState::STOP;
                    wheel_high_since_ms = 0;
                }
                else if (wheel_high && (now_ms - wheel_high_since_ms) >= cfg.long_press_ms)
                {
                    // 长按时间到 → 切换为连发
                    state = DialState::AUTO;
                }
                break;

            case DialState::AUTO:
                if (!wheel_high)
                {
                    // 拨轮释放 → 回到 STOP
                    // 重置目标到当前反馈：松手即停，不追连发攒下的历史 target
                    target_angle_rad = feedback_angle_rad;
                    state = DialState::STOP;
                    wheel_high_since_ms = 0;
                }
                else
                {
                    // 连发：每周期累加目标角度
                    //   angle_per_frame = fire_hz * (360 / slots) * dt
                    //   方向：与单发一致，使用 += (供弹方向 = feedback 增大方向)
                    //   wheel 满幅映射到 wheel_to_hz 频率
                    float fire_hz = cfg.auto_fire_hz;
                    if (cfg.wheel_to_hz > 0.0f)
                    {
                        // 拨轮值越大，连发越快（线性映射）
                        float wheel_norm =
                            (wheel - wheel_threshold) / (1.0f - wheel_threshold);
                        if (wheel_norm < 0.0f) wheel_norm = 0.0f;
                        if (wheel_norm > 1.0f) wheel_norm = 1.0f;
                        fire_hz = wheel_norm * cfg.wheel_to_hz;
                    }
                    float angle_per_frame_deg =
                        fire_hz * (360.0f / cfg.slots_per_rotation) * dt_s;
                    target_angle_rad += angle_per_frame_deg * (PI / 180.0f);
                }
                break;
        }
        last_wheel_high = wheel_high ? 1 : 0;

        // 回写拨轮输入到 Dial_Status
        status.wheel_input = wheel_high ? wheel : 0.0f;
        status.state = (uint8_t)state;

        // ================================================================
        // Step 5: raw_override 模式（绕过 PID，直接发送原始命令）
        // ================================================================
        if (cfg.raw_override_enable)
        {
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
            motor->ctrl_Torque(1, raw_cmd);
            return;
        }

        // ================================================================
        // Step 6: 双环 PID 计算
        // ================================================================
        // 同步 PID 参数（Watch 在线调参）
        kpid_pos.kp = cfg.pos_kp;
        kpid_pos.ki = cfg.pos_ki;
        kpid_pos.kd = cfg.pos_kd;
        kpid_vel.kp = cfg.vel_kp;
        kpid_vel.ki = cfg.vel_ki;
        kpid_vel.kd = cfg.vel_kd;
        position_pid.pid.Break_I = cfg.pos_break_i;
        position_pid.pid.MixI    = cfg.pos_limit_i;
        velocity_pid.pid.Break_I = cfg.vel_break_i;
        velocity_pid.pid.MixI    = cfg.vel_limit_i;

        // 外环：位置环 PID（位置式）
        //   输入: target_angle_rad (rad), feedback_angle_rad (rad)
        //   输出: vel_target (rad/s), 限幅到 [-pos_vel_limit, pos_vel_limit]
        float pos_vel_limit = clampFloatCfg(cfg.pos_vel_limit, 0.0f, 100.0f);
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
        float raw_limit = clampFloatCfg(cfg.raw_output_limit, 0.0f, 2048.0f);
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
        if (cfg.jam_detect_enable)
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
                    // 解卡完成：清状态，重置目标到当前反馈角度
                    jam_active        = 0;
                    jam_start_ms      = 0;
                    jam_torque_sat_ms = 0;
                    jam_pos_err_ms    = 0;
                    target_angle_rad  = feedback_angle_rad;  // 重置目标
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
