/**
 * @file HI12H3_IMU.hpp
 * @brief HI12H3 IMU 驱动 - 接收与解析
 *
 * 设计原因：
 *   三关节可变形云台的姿态控制依赖绝对姿态参考。HI12H3 输出欧拉角/角速度/
 *   加速度/四元数，是 Yaw/Pitch 闭环与后续自瞄、弹道解算的姿态源。
 *   本阶段(Stage03 接入传感器)目标：正确解析传感器数据，并在 Watch 中可观测。
 *
 * 继承说明：
 *   协议解析逻辑继承自参考工程 H_SG_Gimbal/User_reference/BSP/IMU/HI12H3_IMU。
 *   适配当前工程：
 *     ① 离线检测由参考工程的 RM_StaticTime 替换为当前工程已验证的 StateWatch
 *        (与 DR16 / Motor 离线检测机制一致，稳定优先)
 *     ② 串口句柄由 IMUHuart 改名为 IMU_UART，与 DR16_UART 命名风格对齐
 *     ③ 接口命名对齐 DR16：ISDir() → IsOffline()
 *     ④ 移除参考工程类末尾冗余的 public Quat_x/y/z 成员(与 quat 结构体重复)
 *     ⑤ 不引入参考工程的 HAL/UART 抽象层，直接使用 HAL 函数(与 DR16 一致)
 *
 * 数据格式(HI12H3 0x91 数据包，固定 82 字节)：
 *   帧头 0x5A 0xA5 + data_len(2B) + crc(2B)
 *   + 系统遥测(12B: tag/状态字/温度/气压/时间戳)
 *   + 加速度(12B: Acc_x/y/z, 单位 G)
 *   + 角速度(12B: Gyr_x/y/z, 单位 deg/s)
 *   + 磁强度(12B: Mag_x/y/z, 单位 μT)
 *   + 欧拉角(12B: roll/pitch/yaw, 单位 deg)
 *   + 四元数(16B: w/x/y/z)
 *   总计 = 6 + 12 + 12 + 12 + 12 + 12 + 16 = 82 字节
 *
 * 数据流：
 *   Init() → HAL_UARTEx_ReceiveToIdle_DMA 启动 DMA 接收
 *     → 收到完整帧触发空闲中断 → HAL_UARTEx_RxEventCallback (UartCallback.cpp)
 *       → imu.Parse(huart, Size) → ParseData()
 *         ├─ 帧头校验(0x5A 0xA5)，失败则滑动窗口恢复
 *         ├─ memcpy 顺序拷贝各 packed 结构体
 *         └─ AddCaclu() 累计 Yaw(跨 ±180° 连续)
 *       → StateWatch.UpdateLastTime() 更新时间戳
 *     → 重新启动 DMA 接收(等待下一帧)
 *
 * 离线检测：
 *   IsOffline() → StateWatch 检查超时(50ms，200Hz 输出下 10 帧未到判离线)
 *   离线时调用 ClearORE() 清除溢出错误，防止 UART 死锁
 *
 * 调试观察点(Watch)：
 *   - imu.euler.Euler_yaw / Euler_pitch / Euler_roll  欧拉角(deg)
 *   - imu.gyr.Gyr_x/y/z                                角速度(deg/s)
 *   - imu.acc.Acc_x/y/z                                加速度(g)
 *   - imu.quat.Quat_w/x/y/z                            四元数
 *   - imu.addYaw.add_angle                             Yaw 累计角度(deg)
 *   - imu.state_watch_.GetStatus()                     在线状态
 *
 * @note 传感器输出频率 200Hz；波特率 256000(USART1)
 *       轴向映射与参考工程一致：Euler_yaw→云台Yaw, Euler_pitch→云台Pitch
 */

#pragma once

#include "state_watch.hpp"   // 当前工程离线检测工具(替代参考工程 RM_StaticTime)
#include "stdxxx.hpp"        // 基础类型与标准库统一入口
#include "usart.h"           // huart1 句柄

/// @brief IMU 专用串口(USART1, 256000bps, DMA RX 已在 CubeMX 配置)
/// @note  与 DR16_UART(huart3) / VOFA(huart6) 命名风格对齐
#define IMU_UART huart1

/// @brief HI12H3 单帧最大长度(82 字节，0x91 完整数据包)
#define HI12_MAX_LEN 82

namespace BSP::IMU
{
    /**
     * @brief HI12H3 IMU 驱动类
     *
     * 职责：
     *   - 初始化 USART1 DMA 空闲中断接收
     *   - 解析 82 字节固定帧(memcpy + packed 结构体)
     *   - 提供欧拉角/角速度/加速度/四元数/累计Yaw 查询接口
     *   - 检测 IMU 离线(50ms 超时)
     *
     * 使用示例：
     *   imu.Init();                         // 初始化(在 GimbalInit 中)
     *   imu.Parse(huart, Size);             // 中断回调中调用
     *   imu.IsOffline();                    // 周期调用检测离线
     *   float yaw = imu.getYaw();           // 获取航向角(deg)
     */
    class HI12
    {
    public:
        /**
         * @brief 默认构造函数
         *
         * 初始化离线检测器为 50ms 超时(200Hz 输出下 10 帧未到判离线)。
         * 累计角度辅助结构 last_angle 初始化为 -1000(首帧强制走 else 分支)。
         */
        HI12() : state_watch_(50), is_online_(0)
        {
            addYaw.last_angle = -1000.0f;
            addYaw.add_angle  = 0.0f;
        }

        /**
         * @brief IMU 初始化
         *
         * 启动 USART1 DMA 空闲中断接收，等待第一帧数据。
         *
         * @note 在 GimbalInit() 中调用
         */
        void Init();

        /**
         * @brief IMU 数据解析
         *
         * 利用 memcpy 直接顺序拷贝 packed 结构体。因发送格式固定且内存连续，
         * 可用 memcpy 快速拷贝。
         *
         * @return true  解析成功
         * @return false 帧头错误(已触发滑动窗口恢复)
         */
        bool ParseData();

        /**
         * @brief 外部调用的解析入口(在中断回调中调用)
         *
         * @param huart 串口句柄
         * @param Size   本次接收到的数据字节数
         * @note  在 HAL_UARTEx_RxEventCallback 中调用
         *        仅处理 USART1 且长度等于 HI12_MAX_LEN 的帧
         */
        void Parse(UART_HandleTypeDef *huart, uint16_t Size);

        /**
         * @brief 检测 IMU 是否离线
         *
         * @return true  离线(超过 50ms 未收到数据)
         * @return false 在线
         * @note  离线时自动清除 ORE 错误并重启 DMA 接收，防止 UART 死锁
         *        在 GimbalUpdate() 中周期调用(1kHz)
         */
        bool IsOffline();

        // ========== Getter 接口(单位见注释) ==========

        /// @brief 获取 Yaw 航向角(单位: deg)
        inline float getYaw()   const { return euler.Euler_yaw; }
        /// @brief 获取 Pitch 俯仰角(单位: deg)
        inline float getPitch() const { return euler.Euler_pitch; }
        /// @brief 获取 Roll 横滚角(单位: deg)
        inline float getRoll()  const { return euler.Euler_roll; }

        /// @brief 获取 X 轴角速度(单位: deg/s)
        inline float getGyroX() const { return gyr.Gyr_x; }
        /// @brief 获取 Y 轴角速度(单位: deg/s)
        inline float getGyroY() const { return gyr.Gyr_y; }
        /// @brief 获取 Z 轴角速度(单位: deg/s)
        inline float getGyroZ() const { return gyr.Gyr_z; }

        /// @brief 获取 X 轴加速度(单位: g)
        inline float getAccX()  const { return acc.Acc_x; }
        /// @brief 获取 Y 轴加速度(单位: g)
        inline float getAccY()  const { return acc.Acc_y; }
        /// @brief 获取 Z 轴加速度(单位: g)
        inline float getAccZ()  const { return acc.Acc_z; }

        /// @brief 获取 Yaw 累计角度(单位: deg, 跨 ±180° 连续累加)
        inline float getAddYaw() const { return addYaw.add_angle; }

        /// @brief 获取四元数 W
        inline float getQuat_w() const { return quat.Quat_w; }
        /// @brief 获取四元数 X
        inline float getQuat_x() const { return quat.Quat_x; }
        /// @brief 获取四元数 Y
        inline float getQuat_y() const { return quat.Quat_y; }
        /// @brief 获取四元数 Z
        inline float getQuat_z() const { return quat.Quat_z; }

        /// @brief 获取温度(单位: °C)
        inline int8_t getTemperature() const { return system_telemetry.temperature; }

        /// @brief 获取在线状态(0=离线, 1=在线) — Watch 观察用
        inline uint8_t isOnline() const { return is_online_; }

    private:
        // ========== 原始接收缓冲区 ==========
        uint8_t buffer[HI12_MAX_LEN] = {0};   // DMA 接收缓冲区

        // ========== 离线检测(50ms 超时，200Hz 输出下 10 帧未到判离线) ==========
        WATCH_STATE::StateWatch state_watch_;     // 离线检测器
        uint8_t is_online_ = 0;                   // 在线标志(0=离线, 1=在线), Watch 观察

        // ========== 帧格式与数据域(packed, 与 HI12H3 0x91 协议一一对应) ==========

        /// @brief 帧头 + 数据长度 + CRC(6 字节)
        struct __attribute__((packed)) Frame_format
        {
            uint8_t  header_1;   // 帧头 0x5A
            uint8_t  header_2;   // 双帧头 0xA5
            uint16_t data_len;   // 数据域长度
            uint16_t crc;        // CRC 校验
        };

        /// @brief 系统遥测(12 字节)
        struct __attribute__((packed)) System_telemetry
        {
            uint8_t  tag;           // 0x91
            uint16_t main_status;   // 状态字
            int8_t   temperature;   // 温度(°C)
            float    air_pressure;  // 气压(Pa)
            uint32_t system_time;   // 时间戳(ms)
        };

        /// @brief 加速度(12 字节, 单位: G)
        struct __attribute__((packed)) Acc
        {
            float Acc_x;
            float Acc_y;
            float Acc_z;
        };

        /// @brief 角速度(12 字节, 单位: deg/s)
        struct __attribute__((packed)) Gyr
        {
            float Gyr_x;
            float Gyr_y;
            float Gyr_z;
        };

        /// @brief 磁强度(12 字节, 单位: μT)
        struct __attribute__((packed)) Mag
        {
            float Mag_x;
            float Mag_y;
            float Mag_z;
        };

        /// @brief 欧拉角(12 字节, 单位: deg)
        struct __attribute__((packed)) Euler
        {
            float Euler_roll;   // 横滚角
            float Euler_pitch;  // 俯仰角
            float Euler_yaw;    // 航向角
        };

        /// @brief 四元数(16 字节, 顺序 w,x,y,z)
        struct __attribute__((packed)) Quat
        {
            float Quat_w;
            float Quat_x;
            float Quat_y;
            float Quat_z;
        };

        /// @brief 累计角度辅助结构(用于 Yaw 跨 ±180° 连续累加)
        struct AddData
        {
            float last_angle;   // 上一次角度(deg)
            float add_angle;    // 累计角度(deg)
        };

        // ========== 解析后的数据成员(Watch 可观察) ==========
        Frame_format    frame;             // 帧头/长度/CRC
        System_telemetry system_telemetry; // 系统遥测
        Acc             acc;               // 加速度
        Gyr             gyr;               // 角速度
        Mag             mag;               // 磁强度
        Euler           euler;             // 欧拉角
        Quat            quat;              // 四元数
        AddData         addYaw;            // Yaw 累计角度

        // ========== 私有方法 ==========

        /**
         * @brief 清除 ORE(Overrun Error) 标志并重启 DMA 接收
         *
         * @param huart 串口句柄
         * @param pData 接收缓冲区
         * @param Size  数据大小
         * @note  UART 溢出时不清除会导致永久死锁，无法继续接收
         */
        void ClearORE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

        /**
         * @brief 帧头不在起始位置时，通过滑动窗口恢复帧同步
         *
         * 在 buffer 中查找 0x5A 0xA5，将后续数据前移对齐帧头，
         * 然后重启 DMA 接收。
         */
        void SlidingWindowRecovery();

        /**
         * @brief 累计角度计算(处理 ±180° 跳变)
         *
         * @param addData 累计角度结构体
         * @param angle   当前角度(deg)
         * @note  当角度从 179° 跳到 -179° 时判定正转，累计 +2°；
         *        反之判定反转，累计 -2°
         */
        void AddCaclu(AddData &addData, float angle);
    };

    /// @brief 全局 IMU 实例(继承自参考工程的 inline 全局实例风格)
    ///        访问方式：BSP::IMU::imu.Init() / .Parse() / .getYaw()
    inline HI12 imu;

} // namespace BSP::IMU
