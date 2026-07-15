/**
 * @file DR16.hpp
 * @brief DR16遥控器驱动 - 接收与解析
 *
 * 设计原因：
 *   RoboMaster比赛中，DR16遥控器是主要的人机交互设备，通过UART3以100k波特率
 *   传输遥控器数据。数据包含：
 *   - 左右摇杆（各2通道）
 *   - 左右拨杆开关
 *   - 鼠标移动速度
 *   - 鼠标左右键
 *   - 键盘按键状态
 *   - 拨轮
 *
 * 数据格式：
 *   DR16遥控器数据包固定18字节，分为3个部分：
 *   - Part1 (6字节)：摇杆通道 + 开关状态
 *   - Part2 (8字节)：鼠标数据
 *   - Part3 (4字节)：键盘 + 拨轮
 *
 * 数据流：
 *   UART3 DMA接收 → HAL_UARTEx_RxEventCallback → DR16.Parse()
 *     → SaveData() → UpdateStatus() → 更新摇杆/开关/鼠标/键盘状态
 *     → StateWatch更新时间戳
 *
 * 断联检测：
 *   使用StateWatch检测遥控器是否离线（50ms超时）
 *   离线时：摇杆归零、开关置UNKNOWN、清除ORE错误标志
 *
 * @note 继承自参考工程H_SG_Gimbal的Dbus驱动
 *       移除buzzer依赖，简化为当前阶段需要的接收解析功能
 */

#pragma once

#include "state_watch.hpp"
#include "main.h"
#include "usart.h"
#include <cstdint>
#include <cstring>

#define DR16_MAX_LEN 18  // DR16数据包固定18字节

/**
 * @brief UART3为DR16遥控器专用串口
 * 波特率：100000
 * 数据位：9位
 * 停止位：1位
 * 校验位：偶校验
 */
#define DR16_UART huart3

namespace BSP::Remote
{
    /**
     * @brief 键盘按键状态结构体（位域）
     * @note 使用位域节省内存，方便按位访问
     */
    struct __attribute__((packed)) Keyboard
    {
        bool w : 1;      // W键
        bool s : 1;      // S键
        bool a : 1;      // A键
        bool d : 1;      // D键
        bool shift : 1;  // Shift键
        bool ctrl : 1;   // Ctrl键
        bool q : 1;      // Q键
        bool e : 1;      // E键
        bool r : 1;      // R键
        bool f : 1;      // F键
        bool g : 1;      // G键
        bool z : 1;      // Z键
        bool x : 1;      // X键
        bool c : 1;      // C键
        bool v : 1;      // V键
        bool b : 1;      // B键

        /**
         * @brief 生成全零的Keyboard对象
         * @return 全零状态
         */
        static inline Keyboard zero()
        {
            constexpr uint16_t zero = 0;
            return *reinterpret_cast<const Keyboard *>(&zero);
        }
    };

    /**
     * @brief DR16遥控器驱动类
     *
     * 职责：
     *   - 初始化UART3 DMA接收
     *   - 解析遥控器数据包
     *   - 提供摇杆/开关/鼠标/键盘状态查询接口
     *   - 检测遥控器离线
     *
     * 数据流（接收）：
     *   HAL_UARTEx_ReceiveToIdle_DMA → DMA接收完成中断
     *     → HAL_UARTEx_RxEventCallback → DR16.Parse()
     *       → SaveData() → UpdateStatus()
     *
     * 使用示例：
     *   DR16::Instance().Init();  // 初始化
     *   // 在中断回调中调用
     *   DR16::Instance().Parse(&huart3, Size);
     *   // 在任务中查询状态
     *   auto rc = DR16::Instance().remoteRight();
     *   auto sw_left = DR16::Instance().switchLeft();
     */
    class DR16
    {
    public:
        /**
         * @brief 拨杆开关状态枚举
         * @note UP=上档，DOWN=下档，MIDDLE=中档，UNKNOWN=未知（离线或未初始化）
         */
        enum class Switch
        {
            UNKNOWN = 0,  // 未知状态
            UP      = 1,   // 上档
            DOWN    = 2,   // 下档
            MIDDLE  = 3    // 中档
        };

        /**
         * @brief 二维向量结构体（用于摇杆和鼠标速度）
         */
        struct Vector
        {
            double x;  // X轴分量，归一化到[-1.0, 1.0]
            double y;   // Y轴分量，归一化到[-1.0, 1.0]

            /**
             * @brief 生成全零向量
             * @return (0, 0)
             */
            static constexpr inline Vector zero()
            {
                return {0.0, 0.0};
            }
        };

        /**
         * @brief 鼠标按键状态结构体（位域）
         */
        struct __attribute__((packed)) Mouse
        {
            bool left : 1;   // 左键
            bool right : 1;   // 右键

            /**
             * @brief 生成全零的Mouse对象
             * @return 左右键都未按下
             */
            static inline Mouse zero()
            {
                constexpr uint16_t zero = 0;
                return *reinterpret_cast<const Mouse *>(&zero);
            }
        };

        // 删除拷贝构造和赋值运算符（单例模式）
        DR16(const DR16 &) = delete;
        DR16 &operator=(const DR16 &) = delete;

        /**
         * @brief 获取单例实例
         * @return DR16实例引用
         */
        static DR16 &Instance()
        {
            static DR16 instance;
            return instance;
        }

        /**
         * @brief 初始化DR16遥控器
         * @note 启动UART3 DMA空闲中断接收
         */
        void Init();

        /**
         * @brief 解析遥控器数据
         * @param huart UART句柄
         * @param Size 接收到的数据长度
         * @note 在HAL_UARTEx_RxEventCallback中调用
         */
        void Parse(UART_HandleTypeDef *huart, uint16_t Size);

        /**
         * @brief 检测遥控器是否离线
         * @return true=离线，false=在线
         * @note 超过50ms未收到数据判定为离线
         */
        bool IsOffline();

        // ========== Getter接口 ==========

        /**
         * @brief 获取右摇杆值（ch0/ch1）
         * @return Vector{x, y}，范围[-1.0, 1.0]
         * @note ch0: 右摇杆X轴，ch1: 右摇杆Y轴
         */
        inline Vector GetRemoteRight() const { return joystick_right_; }

        /**
         * @brief 获取左摇杆值（ch2/ch3）
         * @return Vector{x, y}，范围[-1.0, 1.0]
         * @note ch2: 左摇杆X轴，ch3: 左摇杆Y轴
         */
        inline Vector GetRemoteLeft() const { return joystick_left_; }

        /**
         * @brief 获取ch0值（右摇杆X轴）
         * @return 范围[-1.0, 1.0]
         */
        inline double GetCh0() const { return joystick_right_.x; }

        /**
         * @brief 获取ch1值（右摇杆Y轴）
         * @return 范围[-1.0, 1.0]
         */
        inline double GetCh1() const { return joystick_right_.y; }

        /**
         * @brief 获取ch2值（左摇杆X轴）
         * @return 范围[-1.0, 1.0]
         */
        inline double GetCh2() const { return joystick_left_.x; }

        /**
         * @brief 获取ch3值（左摇杆Y轴）
         * @return 范围[-1.0, 1.0]
         */
        inline double GetCh3() const { return joystick_left_.y; }

        /**
         * @brief 获取左开关状态（S1）
         * @return Switch枚举
         * @note S1: 左开关，位于遥控器左侧
         */
        inline Switch GetS1() const { return switch_left_; }

        /**
         * @brief 获取右开关状态（S2）
         * @return Switch枚举
         * @note S2: 右开关，位于遥控器右侧
         */
        inline Switch GetS2() const { return switch_right_; }

        /**
         * @brief 获取右开关状态（兼容旧接口）
         * @return Switch枚举
         * @deprecated 请使用 GetS2()
         */
        inline Switch GetSwitchRight() const { return switch_right_; }

        /**
         * @brief 获取左开关状态（兼容旧接口）
         * @return Switch枚举
         * @deprecated 请使用 GetS1()
         */
        inline Switch GetSwitchLeft() const { return switch_left_; }

        /**
         * @brief 获取鼠标移动速度
         * @return Vector{x, y}，范围[-1.0, 1.0]
         */
        inline Vector GetMouseVelocity() const { return mouse_vel_; }

        /**
         * @brief 获取鼠标按键状态
         * @return Mouse结构体
         */
        inline Mouse GetMouse() const { return mouse_; }

        /**
         * @brief 获取键盘按键状态
         * @return Keyboard结构体
         */
        inline Keyboard GetKeyboard() const { return keyboard_; }

        /**
         * @brief 获取拨轮值
         * @return 拨轮值，范围[-1.0, 1.0]
         */
        inline double GetWheel() const { return wheel_; }

    private:
        /**
         * @brief 私有构造函数（单例模式）
         */
        DR16()
            : joystick_right_(Vector::zero())
            , joystick_left_(Vector::zero())
            , mouse_vel_(Vector::zero())
            , switch_right_(Switch::UNKNOWN)
            , switch_left_(Switch::UNKNOWN)
            , mouse_(Mouse::zero())
            , keyboard_(Keyboard::zero())
            , wheel_(0.0)
            , remote_state_watch_(50)  // 50ms超时
        {
        }

        /**
         * @brief 保存原始数据到内部缓冲区
         * @param pData 接收缓冲区指针
         * @note 将18字节数据按Part1/Part2/Part3分别保存
         */
        void SaveData(const uint8_t *pData);

        /**
         * @brief 更新遥控器状态（从原始数据解析）
         * @note 使用位域结构体解析数据包，并归一化摇杆/鼠标/拨轮值
         */
        void UpdateStatus();

        /**
         * @brief 清除UART ORE（Overrun Error）错误标志
         * @param huart UART句柄
         * @param pData 缓冲区指针
         * @param Size 数据大小
         * @note 当UART接收溢出时调用，清除错误并重启DMA接收
         */
        void ClearORE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

    private:
        // ========== 原始数据结构体（位域） ==========

        /**
         * @brief DR16数据包Part1：摇杆 + 开关（6字节）
         * @note 位域按协议顺序排列，编译器自动处理字节序
         */
        struct __attribute__((packed)) Dr16DataPart1
        {
            uint64_t joystick_channel0 : 11;  // 右摇杆X轴
            uint64_t joystick_channel1 : 11;  // 右摇杆Y轴
            uint64_t joystick_channel2 : 11;  // 左摇杆X轴
            uint64_t joystick_channel3 : 11;  // 左摇杆Y轴
            uint64_t switch_right     : 2;   // 右开关
            uint64_t switch_left      : 2;   // 左开关
            uint64_t padding          : 16;  // 保留位
        };

        /**
         * @brief DR16数据包Part2：鼠标数据（8字节）
         */
        struct __attribute__((packed)) Dr16DataPart2
        {
            int16_t mouse_velocity_x;  // 鼠标X轴移动速度
            int16_t mouse_velocity_y;  // 鼠标Y轴移动速度
            int16_t mouse_velocity_z;  // 鼠标Z轴移动速度（未使用）
            bool    mouse_left;         // 鼠标左键
            bool    mouse_right;        // 鼠标右键
        };

        /**
         * @brief DR16数据包Part3：键盘 + 拨轮（4字节）
         */
        struct __attribute__((packed)) Dr16DataPart3
        {
            Keyboard  keyboard;  // 键盘按键状态
            uint16_t  wheel;     // 拨轮原始值
        };

    private:
        // ========== 数据成员 ==========

        uint8_t rx_buffer_[DR16_MAX_LEN];  // DMA接收缓冲区

        // 原始数据存储（64位对齐，方便位域解析）
        uint64_t data_part1_;  // Part1：摇杆+开关
        uint64_t data_part2_;  // Part2：鼠标
        uint64_t data_part3_;  // Part3：键盘+拨轮

        // 解析后的状态数据（便于Watch观察）
        Vector  joystick_right_;     // 右摇杆值
        Vector  joystick_left_;       // 左摇杆值
        Vector  mouse_vel_;           // 鼠标速度
        Switch  switch_right_;        // 右开关状态
        Switch  switch_left_;         // 左开关状态
        Mouse   mouse_;               // 鼠标按键状态
        Keyboard keyboard_;           // 键盘按键状态
        double  wheel_;               // 拨轮值

        // 离线检测
        WATCH_STATE::StateWatch remote_state_watch_;  // 遥控器离线检测（50ms超时）
    };

}  // namespace BSP::Remote