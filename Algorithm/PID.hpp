/**
 * @file PID.hpp
 * @brief 位置式 + 增量式 PID 控制器
 *
 * 继承来源：
 *   工程 PIDreference.hpp / PIDreference.cpp(已通过比赛验证)
 *   仅调整 include 路径与文件组织，算法逻辑保持一致。
 *
 * 核心组件：
 * - Kpid_t：PID 参数结构体(kp/ki/kd)，方便 Watch 在线调参
 * - Pid_t：PID 内部状态(误差、P/I/D 各项、积分限幅等)
 * - PID 类：
 *   - GetPidPos()：位置式 PID(角度环、位置环)
 *   - GetPidInc()：增量式 PID(速度环)
 *
 * 使用原则：
 * - 速度环 → 增量式 PID(输出增量，不易积分饱和，平滑过渡)
 * - 角度环 → 位置式 PID(输出绝对值，消除稳态误差)
 *
 * 调试变量：
 * - Kpid_*.kp/ki/kd：各关节 PID 参数(Watch 可在线修改)
 * - pid_*.pid：PID 内部状态(误差、P/I/D 输出)
 *
 * 数据流：
 *   目标值 cin ──┐
 *                 ├─→ GetPidPos/Inc() ──→ cout(力矩原始值)
 *   反馈 feedback ─┘
 */
#pragma once

#include "stdxxx.hpp"

/**
 * @brief PID 参数结构体(kp/ki/kd)
 *
 * 在 Watch 中可直接修改 kp/ki/kd 实现在线调参。
 * 例如 Kpid_pitch_pos.kp = 10，可在 Watch 中修改。
 */
struct Kpid_t
{
    double kp, ki, kd;
    Kpid_t(double kp = 0, double ki = 0, double kd = 0)
        : kp(kp), ki(ki), kd(kd)
    {}
};

/**
 * @brief PID 内部状态结构体
 *
 * 包含 PID 计算的所有中间变量，方便 Watch 观察调试。
 *
 * 位置式 PID 使用：cin, cout, feedback, p, i, d, now_e, last_e, i_accum
 * 增量式 PID 使用：cin, cout, feedback, p, i, d, now_e, last_e, last_last_e
 */
typedef struct
{
    double cin;            // 期望值(目标值)，单位取决于控制量
    double cout;           // PID 最终输出，位置式为绝对值，增量式为累积值
    double feedback;       // 实际反馈值，单位与 cin 一致

    double p, i, d;        // P/I/D 各项输出，方便 Watch 观察各项贡献

    double now_e;          // 当前误差(cin - feedback)
    double last_e;         // 上一次误差(位置式/增量式共用：微分计算)
    double last_last_e;    // 上上次误差(仅增量式使用：二阶微分)

    double i_accum;        // 积分累积器(仅位置式使用：误差累积值，与 ki 分离)
    double MixI;           // 积分限幅值(位置式：限制 i_accum * ki 的范围)
    float Break_I;         // 积分隔离阈值(位置式：误差小于此值才积分，防止大误差时积分饱和)
} Pid_t;

/**
 * @brief PID 控制器(支持位置式 + 增量式)
 *
 * 位置式 PID(GetPidPos)：
 * - 输出绝对值，适合角度环/位置环
 * - P = kp * 误差
 * - I = ki * 误差累积(带隔离与限幅)
 * - D = kd * (当前误差 - 上次误差)
 * - 输出 = P + I + D，带限幅
 *
 * 增量式 PID(GetPidInc)：
 * - 输出增量，适合速度环
 * - ΔP = kp * (当前误差 - 上次误差)
 * - ΔI = ki * 当前误差
 * - ΔD = kd * (当前误差 - 2*上次误差 + 上上次误差)
 * - 输出 = 上次输出 + ΔP + ΔI + ΔD，带限幅
 *
 * 调试方法：
 * - Watch 中修改 Kpid_t 的 kp/ki/kd
 * - 观察 pid.now_e(误差)、pid.cout(输出)
 * - 观察 pid.p/i/d 各项贡献
 */
class PID
{
  private:
  public:
    Pid_t pid;

    /**
     * @brief PID 构造函数
     * @param Ierror 积分隔离阈值(位置式：误差小于此值才积分)
     * @param MixI 积分限幅值(位置式：限制积分输出范围)
     */
    PID(double Ierror = 0, double MixI = 0)
    {
        this->pid.Break_I = Ierror;
        this->pid.MixI = MixI;
        this->pid.last_e = 0;
        this->pid.last_last_e = 0;
        this->pid.i_accum = 0;
    }

    /**
     * @brief 位置式 PID 计算(角度环/位置环)
     * @param kpid PID 参数(kp/ki/kd)
     * @param cin 目标值
     * @param feedback 反馈值
     * @param max 输出限幅
     * @retval PID 输出值(绝对值)
     * @note 调用频率：1kHz(与控制周期一致)
     */
    double GetPidPos(Kpid_t kpid, double cin, double feedback, double max);

    /**
     * @brief 增量式 PID 计算(速度环)
     * @param kpid PID 参数(kp/ki/kd)
     * @param cin 目标值
     * @param feedback 反馈值
     * @param max 输出限幅
     * @retval PID 输出值(累积绝对值)
     * @note 调用频率：1kHz(与控制周期一致)
     *
     * 增量式 PID 优势：
     * - 不易积分饱和(积分项只累积当前误差)
     * - 输出平滑过渡(每次只输出变化量)
     * - 目标改变时无输出跳变
     */
    double GetPidInc(Kpid_t kpid, double cin, double feedback, double max);

    /**
     * @brief 清除 PID 状态(P/I/D 清零，积分累积器清零)
     * @note 用于电机失能或模式切换时复位
     */
    void clearPID();

    /// 获取当前误差
    inline float GetErr() { return this->pid.now_e; }

    /// 获取 PID 输出
    inline float GetCout() { return this->pid.cout; }

    /// 获取目标值
    inline float GetCin() { return this->pid.cin; }
};
