/**
 * @file RemoteControl.cpp
 * @brief 遥控器速度映射实现
 *
 * 数据源选择（优先级）：
 * 1. 云台数据（如果云台在线）→ 从板间通信接收
 * 2. DR16 遥控器（如果云台离线）→ 从 UART3 接收
 *
 * 映射流程：
 * 1. 检查数据源在线状态
 * 2. 读取摇杆值并应用死区
 * 3. 乘以最大速度参数
 * 4. 更新到 Wheel_Data.vx/vy/vw
 *
 * 安全保护：
 * - 云台离线且遥控器离线时，底盘速度清零
 * - 摇杆死区处理，消除漂移
 *
 * 调试方法：
 * - Watch 中观察 BoardComm_Data.LX/LY（云台摇杆）
 * - Watch 中观察 Remote_Data.channel[0~3]（遥控器摇杆）
 * - Watch 中观察 Wheel_Data.vx/vy/vw（底盘目标速度）
 * - Watch 中修改 Remote_Params.max_vx/vy/vw（最大速度）
 * - Watch 中修改 Remote_Params.dead_zone（死区）
 */
#include "RemoteControl.hpp"
#include "../BSP/Remote/DR16.hpp"
#include "Variable.hpp"
#include "Communication/BoardComm.hpp"

// 全局遥控器速度映射实例
RemoteControl_t RemoteControl;

void RemoteControl_t::MapToChassis()
{
    // ========== 优先使用云台数据【新增】==========
    if (BoardComm::gimbal_to_chassis.isConnectOnline())
    {
        // 云台在线：使用云台遥控器数据
        // getLX/getLY 已归一化到 [-1.0, 1.0]
        // getWheel 归一化到 [-1.0, 1.0]（拨轮值）
        float lx = BoardComm::gimbal_to_chassis.getLX();
        float ly = BoardComm::gimbal_to_chassis.getLY();
        float wheel = BoardComm::gimbal_to_chassis.getWheel();

        // 死区处理（平移速度）
        lx = ApplyDeadZone(lx, Remote_Params.dead_zone);
        ly = ApplyDeadZone(ly, Remote_Params.dead_zone);

        // 速度映射（与DR16路径保持一致的坐标系）
        // 右摇杆Y(ly) → vx_raw（前进/后退）
        // 右摇杆X(lx) → vy_raw（左移/右移）
        float vx_raw = ly * Remote_Params.max_vx;
        float vy_raw = lx * Remote_Params.max_vy;

        // ========== 底盘跟随模式处理【新增】==========
        // 检查是否进入跟随模式
        // 条件：云台在线 且 Follow_mode 标志位为 1
        if (BoardComm::gimbal_to_chassis.getFollow())
        {
            // ========== 跟随模式：底盘跟随云台 ==========
            //
            // 实现原理：
            // 1. 读取云台-底盘角度误差（弧度）
            // 2. 计算跟随PID输出（旋转速度）
            // 3. 坐标系变换（云台系 → 底盘系）
            // 4. 用跟随PID输出替换遥控器vw
            //
            // 数据流：
            //   云台 Joint_Data.yaw.real_angle → Yaw_encoder_angle_err → CAN发送
            //   底盘 CAN接收 → getEncoderAngleErr() → 角度PID → vw_follow
            //   遥控器摇杆 → vx_gimbal/vy_gimbal → 坐标系变换 → vx_chassis/vy_chassis
            //

            // 1. 读取云台-底盘角度误差（弧度）
            // 注意：getEncoderAngleErr() 返回值已经是弧度，无需再转换
            float yaw_err = BoardComm::gimbal_to_chassis.getEncoderAngleErr();

            // 2. 计算跟随PID输出（简单的P控制）
            // 公式：vw = kp * yaw_err
            // 物理意义：角度误差1rad → 旋转速度3 rad/s（当kp=3.0时）
            float vw_follow = yaw_err * Remote_Params.follow_kp;

            // 限幅（防止过快旋转）
            // 限制在 [-max_vw_follow, max_vw_follow] 范围内
            if (vw_follow > Remote_Params.max_vw_follow)
                vw_follow = Remote_Params.max_vw_follow;
            else if (vw_follow < -Remote_Params.max_vw_follow)
                vw_follow = -Remote_Params.max_vw_follow;

            // 3. 坐标系变换：云台坐标系 → 底盘坐标系
            //
            // 变换公式（旋转矩阵）：
            //   vx_chassis = vx_gimbal * cos(yaw_err) + vy_gimbal * sin(yaw_err)
            //   vy_chassis = -vx_gimbal * sin(yaw_err) + vy_gimbal * cos(yaw_err)
            //
            // 物理意义：
            //   云台坐标系：以云台朝向为X轴，右手系
            //   底盘坐标系：以底盘朝向为X轴，右手系
            //   yaw_err：云台相对底盘的角度（正值=云台在底盘左侧）
            //
            // 效果：
            //   云台转向左侧（yaw_err > 0）→ 底盘前进方向偏向左侧
            //   推遥控器前进（vx_gimbal > 0）→ 底盘向云台朝向前进
            //
            float vx_chassis = vx_raw * cos(yaw_err) + vy_raw * sin(yaw_err);
            float vy_chassis = -vx_raw * sin(yaw_err) + vy_raw * cos(yaw_err);

            // 4. 输出到全局变量
            Wheel_Data.vx = vx_chassis;
            Wheel_Data.vy = vy_chassis;
            Wheel_Data.vw = vw_follow;  // 用跟随PID输出替换遥控器vw

            return;  // 跟随模式处理完成，直接返回
        }

        // ========== 非跟随模式：手动控制旋转 ==========
        // 旋转速度映射（wheel 控制）
        // 控制方式：wheel 拨轮（替代左摇杆）
        // 有效范围：wheel ∈ [0, -1.0]（负值区域）
        // 旋转方向：顺时针（vw < 0）
        // 映射方式：线性映射
        float vw = 0.0f;
        if (wheel < 0.0f)  // 只有负值有效
        {
            // wheel ∈ [-1.0, 0) → vw ∈ [-max_vw, 0)
            // 线性映射：wheel 越小（越负），旋转越快
            vw = wheel * Remote_Params.max_vw;  // 顺时针旋转
        }
        // wheel >= 0 时不旋转（vw = 0）

        // 坐标系旋转：逆时针旋转 90 度 + 左右修正（与DR16路径一致）
        Wheel_Data.vx = vy_raw;
        Wheel_Data.vy = vx_raw;
        Wheel_Data.vw = vw;  // 手动控制旋转

        return;
    }

    // ========== 云台离线：回退到DR16遥控器 ==========
    // 检查遥控器在线状态
    if (!DR16.IsOnline())
    {
        // 遥控器离线：底盘速度清零（安全保护）
        Wheel_Data.vx = 0;
        Wheel_Data.vy = 0;
        Wheel_Data.vw = 0;
        return;
    }

    // 读取摇杆值
    float ch0 = Remote_Data.channel[0];  // 右摇杆水平
    float ch1 = Remote_Data.channel[1];  // 右摇杆垂直
    float ch2 = Remote_Data.channel[2];  // 左摇杆水平

    // 死区处理
    ch0 = ApplyDeadZone(ch0, Remote_Params.dead_zone);
    ch1 = ApplyDeadZone(ch1, Remote_Params.dead_zone);
    ch2 = ApplyDeadZone(ch2, Remote_Params.dead_zone);

    // 速度映射（遥控器坐标系）
    // 右摇杆垂直 → vx_raw（前进/后退）
    // 右摇杆水平 → vy_raw（左移/右移）
    // 左摇杆水平 → vw（旋转）
    float vx_raw = ch1 * Remote_Params.max_vx;
    float vy_raw = ch0 * Remote_Params.max_vy;

    // 坐标系旋转：逆时针旋转 90 度 + 左右修正
    // new_vx = old_vy（取反修正左右方向）
    // new_vy = old_vx
    Wheel_Data.vx = vy_raw;
    Wheel_Data.vy = vx_raw;
    Wheel_Data.vw = -ch2 * Remote_Params.max_vw;
}

float RemoteControl_t::ApplyDeadZone(float value, float dead_zone)
{
    // 死区处理：|value| < dead_zone 时返回 0
    if (value > -dead_zone && value < dead_zone)
    {
        return 0.0f;
    }
    return value;
}
