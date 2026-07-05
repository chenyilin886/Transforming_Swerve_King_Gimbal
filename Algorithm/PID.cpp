/**
 * @file PID.cpp
 * @brief 位置式 + 增量式 PID 控制器实现
 *
 * 继承来源：
 *   工程 PIDreference.cpp(已通过比赛验证)
 *   算法逻辑保持完全一致，仅调整 include 路径。
 *
 * 修复旧版积分累积逻辑 bug：
 * - 旧版：pid.i 既做累积器又乘 ki，导致每次调用都重复乘 ki
 * - 新版：i_accum 专门做误差累积，pid.i = ki * i_accum，逻辑清晰
 *
 * 新增增量式 PID(GetPidInc)：
 * - 适合速度环控制
 * - 输出增量，不易积分饱和
 * - 输出平滑过渡
 */
#include "PID.hpp"

/**
 * @brief 位置式 PID 计算(角度环/位置环)
 *
 * 算法流程：
 * 1. 计算误差：now_e = cin - feedback
 * 2. 比例项：P = kp * now_e
 * 3. 积分累积：误差小于 Break_I 时累积，否则清零(积分隔离)
 * 4. 积分输出：I = ki * i_accum，限幅在 [-MixI, MixI]
 * 5. 微分项：D = kd * (now_e - last_e)
 * 6. 输出 = P + I + D，限幅在 [-max, max]
 *
 * @param kpid PID 参数
 * @param cin 目标值
 * @param feedback 反馈值
 * @param max 输出限幅
 * @retval PID 输出(绝对值)
 */
double PID::GetPidPos(Kpid_t kpid, double cin, double feedback, double max)
{
    // 保存输入
    this->pid.cin = cin;
    this->pid.feedback = feedback;

    // 当前误差
    this->pid.now_e = cin - feedback;

    // 比例项
    this->pid.p = kpid.kp * this->pid.now_e;

    // 积分隔离：误差较小时才累积，避免大误差时积分饱和
    // 如果不隔离：大误差时积分快速累积 → 输出饱和 → 超调严重
    if (fabs(this->pid.now_e) < this->pid.Break_I)
    {
        this->pid.i_accum += this->pid.now_e;
    }
    else
    {
        this->pid.i_accum = 0;
    }

    // 积分输出 = ki * 误差累积值
    this->pid.i = kpid.ki * this->pid.i_accum;

    // 积分限幅：防止积分项过大导致输出饱和
    if (this->pid.i > this->pid.MixI)
        this->pid.i = this->pid.MixI;
    if (this->pid.i < -this->pid.MixI)
        this->pid.i = -this->pid.MixI;

    // ki=0 时清除积分输出和累积器
    if (kpid.ki == 0.0)
    {
        this->pid.i = 0;
        this->pid.i_accum = 0;
    }

    // 微分项(直接微分：当前误差 - 上次误差)
    this->pid.d = kpid.kd * (this->pid.now_e - this->pid.last_e);

    // 更新误差历史
    this->pid.last_e = this->pid.now_e;

    // PID 输出
    this->pid.cout = this->pid.p + this->pid.i + this->pid.d;

    // 输出限幅
    if (this->pid.cout > max)
        this->pid.cout = max;
    if (this->pid.cout < -max)
        this->pid.cout = -max;

    return this->pid.cout;
}

/**
 * @brief 增量式 PID 计算(速度环)
 *
 * 算法原理：
 * - 位置式 PID 输出绝对值，增量式 PID 输出变化量(增量)
 * - 增量式公式：
 *   ΔP = kp * (e(k) - e(k-1))
 *   ΔI = ki * e(k)
 *   ΔD = kd * (e(k) - 2*e(k-1) + e(k-2))
 *   u(k) = u(k-1) + ΔP + ΔI + ΔD
 *
 * 速度环使用增量式的优势：
 * 1. 不易积分饱和：积分项 ΔI = ki*e(k) 只依赖当前误差，不会无限累积
 * 2. 输出平滑：每次只调整变化量，目标改变时不会跳变
 * 3. 抗干扰：误差突变只影响当前增量，不会像位置式那样影响积分累积
 *
 * @param kpid PID 参数
 * @param cin 目标值
 * @param feedback 反馈值
 * @param max 输出限幅
 * @retval PID 输出(累积绝对值)
 */
double PID::GetPidInc(Kpid_t kpid, double cin, double feedback, double max)
{
    // 保存输入
    this->pid.cin = cin;
    this->pid.feedback = feedback;

    // 当前误差
    this->pid.now_e = cin - feedback;

    // 增量式 PID 各项
    // ΔP = kp * (当前误差 - 上次误差)：响应误差变化趋势
    this->pid.p = kpid.kp * (this->pid.now_e - this->pid.last_e);

    // ΔI = ki * 当前误差：消除稳态误差(不会累积历史误差，不易饱和)
    this->pid.i = kpid.ki * this->pid.now_e;

    // ΔD = kd * (当前误差 - 2*上次误差 + 上上次误差)：二阶微分，抑制误差加速度
    this->pid.d = kpid.kd * (this->pid.now_e - 2 * this->pid.last_e + this->pid.last_last_e);

    // 计算增量
    double delta_u = this->pid.p + this->pid.i + this->pid.d;

    // 累积输出：当前输出 = 上次输出 + 增量
    this->pid.cout += delta_u;

    // 输出限幅
    if (this->pid.cout > max)
        this->pid.cout = max;
    if (this->pid.cout < -max)
        this->pid.cout = -max;

    // 更新误差历史(增量式需要保存两次历史)
    this->pid.last_last_e = this->pid.last_e;
    this->pid.last_e = this->pid.now_e;

    return this->pid.cout;
}

/**
 * @brief 清除 PID 状态
 *
 * 将 P/I/D 各项、输出、误差历史、积分累积器全部清零。
 * 用于电机失能或模式切换时复位，防止残留状态导致输出跳变。
 */
void PID::clearPID()
{
    this->pid.p = 0;
    this->pid.i = 0;
    this->pid.d = 0;
    this->pid.cout = 0;
    this->pid.now_e = 0;
    this->pid.last_e = 0;
    this->pid.last_last_e = 0;
    this->pid.i_accum = 0;
}
