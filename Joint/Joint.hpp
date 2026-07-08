#ifndef JOINT_HPP
#define JOINT_HPP

/**
 * @file Joint.hpp
 * @brief 关节层 - Motor 与 Application 之间的抽象层
 *
 * 设计原因：
 *   Motor 层只提供编码器原始角度(encoder_angle)，不知道零位在哪里、
 *   正反方向如何、机械限位是多少。Application 层(算法/PID/状态机)
 *   只应关心"关节真实角度(real_angle)"，不应该接触编码器原始值。
 *
 *   Joint 层职责：
 *     1. 从 Motor 读取 encoder_angle → 减去 offset → 得到 real_angle
 *     2. 根据 direction 系数修正正反方向
 *     3. 归一化(Normalize): Yaw 连续旋转 → [-π, π]; Pitch/Fold → 限位钳位
 *     4. 管理零位校准(Calibrate)
 *     5. 管理关节限位(Limit)
 *
 * 数据流：
 *   Motor.getAngleRad() ──→ Joint.Update() ──→ Joint_Data (Watch)
 *                           │
 *                           ├─ encoder_angle   (编码器原始值)
 *                           ├─ real_angle       (减offset × direction)
 *                           ├─ normalized_angle (归一化/限位后)
 *                           ├─ target_angle     (上层设定)
 *                           └─ calib_state      (校准状态)
 *
 * 关键设计：
 *   - Joint 不依赖 MotorBase，只接收 float 参数(解耦)
 *   - Joint 不区分 DM4310/DM4340(型号无关)
 *   - JointManager 绑定 Motor 指针，统一管理三个关节
 *   - 所有可调参数(offset/limit/direction)在 Variable.hpp 中，
 *     Watch 可在线修改
 */

#include "MotorBase.hpp"
#include "DmMotor.hpp"

namespace BSP::JOINT
{

/// 圆周率常量(不依赖 Motor 层)
constexpr float PI = 3.14159265358979323846f;

/// 角度归一化辅助：将角度映射到 [-π, π]
static inline float wrapToPi(float angle)
{
    while (angle >  PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

/**
 * @brief 关节配置参数(Watch 可调)
 *
 * 字段说明：
 *   offset      : 零位偏移(rad)，real_angle = (encoder - offset) * direction
 *   limit_min   : 机械限位下限(rad)，仅对非连续关节生效
 *   limit_max   : 机械限位上限(rad)
 *   direction   : 方向系数(1.0=正方向, -1.0=反方向)
 *   continuous  : 1=连续旋转(Yaw), 0=有限位(Pitch/Fold)
 *   calib_enable: Watch 置 1 触发校准(将当前位置设为零位)
 */
struct JointConfig
{
    float offset;         // 零位偏移(rad)
    float limit_min;      // 限位下限(rad)
    float limit_max;      // 限位上限(rad)
    float direction;      // 方向系数(1.0 / -1.0)
    uint8_t continuous;   // 连续旋转标志
    uint8_t calib_enable; // 校准使能(Watch 触发)
};

/**
 * @class Joint
 * @brief 单关节抽象
 *
 * 每个 Joint 实例对应一个物理关节(Yaw/Pitch/Fold)。
 * Joint 不持有 Motor 指针，由 JointManager 负责数据传递。
 */
class Joint
{
public:
    Joint() = default;

    /**
     * @brief 初始化关节参数
     * @param offset      零位偏移
     * @param limit_min   限位下限
     * @param limit_max   限位上限
     * @param direction   方向系数
     * @param continuous  是否连续旋转
     */
    void Init(float offset, float limit_min, float limit_max,
              float direction, bool continuous)
    {
        config_.offset      = offset;
        config_.limit_min   = limit_min;
        config_.limit_max   = limit_max;
        config_.direction   = direction;
        config_.continuous  = continuous ? 1 : 0;
        config_.calib_enable = 0;
        calib_state_ = 0;  // 未校准
    }

    /**
     * @brief 更新关节状态(由 JointManager 调用)
     *
     * 流程：
     *   1. 保存编码器原始角度
     *   2. 检查校准请求(calib_enable)
     *   3. 计算 real_angle = (encoder - offset) * direction
     *   4. 归一化: continuous → wrapToPi; 有限位 → clamp
     *
     * @param enc_angle 编码器角度(rad)
     * @param vel       角速度(rad/s)
     * @param torque    力矩(N·m)
     * @param temp      温度(℃)
     * @param online    在线状态
     */
    void Update(float enc_angle, float vel, float torque, float temp, bool online)
    {
        encoder_angle_ = enc_angle;
        velocity_      = vel * config_.direction;  // 速度也要乘 direction，与 real_angle 方向一致
        torque_        = torque;
        temperature_   = temp;
        online_        = online;

        // --- 校准请求处理 ---
        // Watch 中将 calib_enable 置 1 → 以当前编码器位置为零位
        if (config_.calib_enable)
        {
            config_.offset = enc_angle;
            calib_state_ = 1;       // 标记已校准
            config_.calib_enable = 0;  // 清除请求(单次触发)
        }

        // --- 真实角度计算 ---
        // real_angle = (encoder - offset) * direction
        real_angle_ = (enc_angle - config_.offset) * config_.direction;

        // --- 归一化 / 限位 ---
        if (config_.continuous)
        {
            // 连续旋转关节(Yaw): 归一化到 [-π, π]
            normalized_angle_ = wrapToPi(real_angle_);
        }
        else
        {
            // 有限位关节(Pitch/Fold): 钳位到 [limit_min, limit_max]
            if (real_angle_ > config_.limit_max)
                normalized_angle_ = config_.limit_max;
            else if (real_angle_ < config_.limit_min)
                normalized_angle_ = config_.limit_min;
            else
                normalized_angle_ = real_angle_;
        }
    }

    // --- Getter ---
    float getEncoderAngle()   const { return encoder_angle_; }
    float getRealAngle()      const { return real_angle_; }
    float getNormalizedAngle()const { return normalized_angle_; }
    float getVelocity()       const { return velocity_; }
    float getTorque()         const { return torque_; }
    float getTemperature()    const { return temperature_; }
    bool  isOnline()          const { return online_; }
    uint8_t getCalibState()   const { return calib_state_; }

    /// 配置引用(Watch 同步用，可读可写)
    JointConfig& config()             { return config_; }
    const JointConfig& getConfig() const { return config_; }

    /**
     * @brief 设置目标角度(由上层 PID/状态机调用)
     */
    void setTargetAngle(float target) { target_angle_ = target; }
    float getTargetAngle() const { return target_angle_; }

private:
    // --- 配置参数 ---
    JointConfig config_ = {};

    // --- 运行时状态 ---
    float encoder_angle_   = 0.0f;  // 编码器原始角度(rad)
    float real_angle_      = 0.0f;  // 真实角度(rad)
    float normalized_angle_ = 0.0f;  // 归一化/限位后角度(rad)
    float target_angle_    = 0.0f;  // 目标角度(rad)
    float velocity_        = 0.0f;  // 角速度(rad/s)
    float torque_          = 0.0f;  // 力矩(N·m)
    float temperature_     = 0.0f;  // 温度(℃)
    bool  online_          = false;  // 在线状态
    uint8_t calib_state_   = 0;     // 校准状态: 0=未校准, 1=已校准
};


/**
 * @class JointManager
 * @brief 三关节统一管理器
 *
 * 职责：
 *   1. 持有 Yaw/Pitch/Fold 三个 Joint 实例
 *   2. 绑定 Motor 指针(DM4310/DM4340)
 *   3. Update() 从 Motor 读取数据 → Joint.Update() → 写入 Joint_Data
 *
 * 数据流：
 *   DM4310.getAngleRad(1) → JointManager.yaw.Update()
 *   DM4310.getAngleRad(2) → JointManager.pitch.Update()
 *   DM4340.getAngleRad(1) → JointManager.fold.Update()
 *
 * @note JointManager 不发送控制帧，控制由 Controller 层(Day02)负责
 */
class JointManager
{
public:
    Joint yaw;      ///< Yaw 关节(DM4310 #1, 连续旋转)
    Joint pitch;    ///< Pitch 关节(DM4310 #2, 有限位)
    Joint fold;     ///< Fold 关节(DM4340 #1, 有限位)

    /**
     * @brief 初始化三关节参数
     *
     * 默认参数：
     *   Yaw  : offset=0, 无限位, direction=1, continuous=true
     *   Pitch: offset=0, ±90°(±1.5708rad), direction=1, continuous=false
     *   Fold : offset=0, ±90°(±1.5708rad), direction=1, continuous=false
     *
     * @note 参数可在 Watch 中实时修改(通过 Joint_Data 中的 config 字段)
     */
    void Init()
    {
        // Yaw: 连续旋转，无机械限位
        yaw.Init(
            0.0f,        // offset
            -PI,         // limit_min (不生效，continuous)
            PI,          // limit_max
            1.0f,        // direction
            true         // continuous
        );

        // Pitch: 有限位, direction=-1(抬枪编码器减小), 实测标定值
        pitch.Init(
            0.8825f,     // offset (实测)
            -0.7927f,    // limit_min (枪口最低, 实测)
             0.6481f,     // limit_max (枪口最高, 实测)
            -1.0f,       // direction
            false        // continuous
        );

        // Fold: 有限位 ±90° (±1.5708 rad), direction=-1(抬起编码器减小)
        fold.Init(
            0.0f,        // offset
            -1.5708f,    // limit_min (-90°)
            1.5708f,     // limit_max (+90°)
            -1.0f,       // direction (抬起时编码器减小，取反)
            false        // continuous
        );
    }

    /**
     * @brief 周期更新：从 Motor 读取 → Joint 计算
     *
     * @param dm4310  DM4310 电机组指针(Yaw #1 + Pitch #2)
     * @param dm4340  DM4340 电机指针(Fold #1)
     *
     * @note 此函数不发送控制帧，仅读取反馈数据
     */
    void Update(BSP::MOTOR::DM::DM4310 *dm4310, BSP::MOTOR::DM::DM4340 *dm4340)
    {
        if (dm4310 != nullptr)
        {
            // Yaw (DM4310 #1)
            yaw.Update(
                dm4310->getAngleRad(1),
                dm4310->getVelocityRad(1),
                dm4310->getTorque(1),
                dm4310->getTemperature(1),
                dm4310->isConnected(1)
            );

            // Pitch (DM4310 #2)
            pitch.Update(
                dm4310->getAngleRad(2),
                dm4310->getVelocityRad(2),
                dm4310->getTorque(2),
                dm4310->getTemperature(2),
                dm4310->isConnected(2)
            );
        }

        if (dm4340 != nullptr)
        {
            // Fold (DM4340 #1)
            fold.Update(
                dm4340->getAngleRad(1),
                dm4340->getVelocityRad(1),
                dm4340->getTorque(1),
                dm4340->getTemperature(1),
                dm4340->isConnected(1)
            );
        }
    }
};

} // namespace BSP::JOINT

#endif // JOINT_HPP
