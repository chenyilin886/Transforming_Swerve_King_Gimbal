/**
 * @file DR16.cpp
 * @brief DR16遥控器驱动 - 接收与解析实现
 *
 * 设计原因：
 *   实现DR16遥控器的数据接收和解析功能。使用DMA+空闲中断方式接收，
 *   提高接收效率，避免CPU频繁处理UART中断。
 *
 * 数据解析原理：
 *   DR16数据包固定18字节，分为3部分：
 *   1. Part1（6字节）：摇杆通道（11bit×4）+ 开关状态（2bit×2）
 *   2. Part2（8字节）：鼠标移动速度（int16×3）+ 鼠标按键（bool×2）
 *   3. Part3（4字节）：键盘按键（16bit位域）+ 拨轮（16bit）
 *
 *   使用位域结构体可以直接解析数据包，无需手动计算位偏移。
 *   摇杆值：0~1024~2046（中值1024），归一化到[-1.0, 1.0]
 *   鼠标速度：int16范围，归一化到[-1.0, 1.0]
 *   拨轮：0~1024~2046，归一化到[-1.0, 1.0]
 *
 * 数据流：
 *   Init() → HAL_UARTEx_ReceiveToIdle_DMA启动DMA接收
 *     → 接收到数据 → HAL_UARTEx_RxEventCallback
 *       → Parse() → SaveData()保存原始数据
 *         → UpdateStatus()解析并更新状态
 *           → StateWatch.UpdateLastTime()更新时间戳
 *     → 重新启动DMA接收（等待下一包）
 *
 * 离线检测：
 *   IsOffline() → StateWatch.UpdateTime()获取当前时间
 *     → StateWatch.CheckStatus()检查超时
 *       → 返回状态（ONLINE/OFFLINE）
 *
 * 调试观察点：
 *   - joystick_right_.x/y：右摇杆值
 *   - joystick_left_.x/y：左摇杆值
 *   - switch_right_/switch_left_：开关状态
 *   - mouse_vel_.x/y：鼠标速度
 *   - mouse_.left/right：鼠标按键
 *   - keyboard_.w/s/a/d等：键盘按键
 *   - wheel_：拨轮值
 *   - remote_state_watch_.GetStatus()：遥控器在线状态
 *
 * @note 继承自参考工程H_SG_Gimbal的Dbus驱动
 */

#include "DR16.hpp"
#include <cstring>

namespace BSP::Remote
{
    /**
     * @brief 初始化DR16遥控器
     *
     * 启动UART3 DMA空闲中断接收。
     * 使用HAL_UARTEx_ReceiveToIdle_DMA可以在收到完整数据包后自动触发回调，
     * 无需手动判断数据包边界。
     *
     * @note DR16数据包固定18字节，接收完成后自动触发HAL_UARTEx_RxEventCallback
     */
    void DR16::Init()
    {
        // 启动DMA空闲中断接收
        // 参数：UART句柄、接收缓冲区、数据长度
        HAL_UARTEx_ReceiveToIdle_DMA(&DR16_UART, rx_buffer_, DR16_MAX_LEN);
    }

    /**
     * @brief 保存原始数据到内部缓冲区
     * @param pData 接收缓冲区指针
     *
     * 将18字节数据按Part1/Part2/Part3分别保存到data_part1_/2/3_。
     * 使用memcpy保证内存对齐，避免位域解析时出现未定义行为。
     *
     * 数据分布：
     *   pData[0~5]   → Part1（摇杆+开关）
     *   pData[6~13]  → Part2（鼠标）
     *   pData[14~17] → Part3（键盘+拨轮）
     */
    void DR16::SaveData(const uint8_t *pData)
    {
        // 保存Part1（6字节）
        std::memcpy(&data_part1_, pData, 6);
        pData += 6;

        // 保存Part2（8字节）
        std::memcpy(&data_part2_, pData, 8);
        pData += 8;

        // 保存Part3（4字节）
        std::memcpy(&data_part3_, pData, 4);
        pData += 4;
    }

    /**
     * @brief 更新遥控器状态（从原始数据解析）
     *
     * 使用reinterpret_cast将data_part_强转为位域结构体，直接访问各字段。
     * 摇杆和拨轮值归一化到[-1.0, 1.0]，便于后续控制算法使用。
     *
     * 摇杆归一化公式：
     *   value_norm = (value_raw - 1024) / 660.0
     *   其中：
     *   - 1024为中值（摇杆居中）
     *   - 660为最大偏移范围（0~1024或1024~2046）
     *   - 结果范围：-1.0（最小）~ 0.0（居中）~ 1.0（最大）
     *
     * 鼠标速度归一化公式：
     *   value_norm = value_raw / 32768.0
     *   其中：
     *   - 32768为int16最大值（±32768）
     *   - 结果范围：-1.0 ~ 1.0
     *
     * @note 使用alignas确保data_part_地址对齐，避免未定义行为
     */
    void DR16::UpdateStatus()
    {
        // 解析Part1：摇杆 + 开关
        auto &part1 alignas(uint64_t) = *reinterpret_cast<Dr16DataPart1 *>(&data_part1_);

        // 摇杆值归一化函数：0~1024~2046 → -1.0~0.0~1.0
        auto channel_to_double = [](uint16_t value) {
            return (static_cast<int32_t>(value) - 1024) / 660.0;
        };

        // 右摇杆（ch0: X轴左右, ch1: Y轴前后）
        // 用户反馈：向上（向前）推右摇杆，ch1应该为正；向左推右摇杆，ch0应该为负
        joystick_right_.y = channel_to_double(static_cast<uint16_t>(part1.joystick_channel1));  // Y轴（前后），向前推为正
        joystick_right_.x = channel_to_double(static_cast<uint16_t>(part1.joystick_channel0));  // X轴（左右），向左推为负

        // 左摇杆（ch2: X轴左右, ch3: Y轴前后）
        joystick_left_.y = channel_to_double(static_cast<uint16_t>(part1.joystick_channel3));
        joystick_left_.x = channel_to_double(static_cast<uint16_t>(part1.joystick_channel2));

        // 开关状态（直接映射）
        switch_right_ = static_cast<Switch>(part1.switch_right);
        switch_left_  = static_cast<Switch>(part1.switch_left);

        // 解析Part2：鼠标数据
        auto &part2 alignas(uint64_t) = *reinterpret_cast<Dr16DataPart2 *>(&data_part2_);

        // 鼠标速度归一化（int16 → -1.0~1.0）
        // 用户反馈：根据实际测试调整方向（暂不取负，需实测验证）
        mouse_vel_.x = (part2.mouse_velocity_x / 32768.0);
        mouse_vel_.y = (part2.mouse_velocity_y / 32768.0);

        // 鼠标按键状态
        mouse_.left  = part2.mouse_left;
        mouse_.right = part2.mouse_right;

        // 解析Part3：键盘 + 拨轮
        auto &part3 alignas(uint64_t) = *reinterpret_cast<Dr16DataPart3 *>(&data_part3_);

        // 键盘按键状态（直接赋值）
        keyboard_ = part3.keyboard;

        // 拨轮归一化（与摇杆相同）
        wheel_ = channel_to_double(static_cast<uint16_t>(part3.wheel));
    }

    /**
     * @brief 解析遥控器数据
     * @param huart UART句柄
     * @param Size 接收到的数据长度
     *
     * 在HAL_UARTEx_RxEventCallback中调用。
     * 仅处理UART3（DR16专用串口），且数据长度必须为18字节。
     *
     * 数据流：
     *   1. 判断UART和长度是否正确
     *   2. SaveData()保存原始数据
     *   3. UpdateStatus()解析并更新状态
     *   4. StateWatch.UpdateLastTime()更新时间戳（防止误判离线）
     *   5. 重新启动DMA接收（等待下一包）
     *
     * @note DMA空闲中断接收完成后自动调用此函数
     */
    void DR16::Parse(UART_HandleTypeDef *huart, uint16_t Size)
    {
        ++rx_event_count_;
        last_rx_size_ = Size;
        last_rx_tick_ = HAL_GetTick();

        if (huart == &DR16_UART && Size == DR16_MAX_LEN)
        {
            ++valid_frame_count_;
            SaveData(rx_buffer_);
            UpdateStatus();
            remote_state_watch_.UpdateLastTime();
        }
        else
        {
            ++invalid_frame_count_;
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&DR16_UART, rx_buffer_, DR16_MAX_LEN);
    }
    void DR16::ClearORE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
    {
        // 检测ORE（Overrun Error）标志
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
        {
            // 清除ORE标志
            __HAL_UART_CLEAR_OREFLAG(huart);

            // 重新启动DMA接收
            HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size);
        }
    }

    /**
     * @brief 检测遥控器是否离线
     * @return true=离线，false=在线
     *
     * 使用StateWatch检测遥控器是否超过50ms未收到数据。
     * 如果离线，清除ORE错误标志，防止UART死锁。
     *
     * 检测流程：
     *   1. UpdateTime()获取当前时间戳
     *   2. CheckStatus()计算时间差并更新状态
     *   3. GetStatus()返回状态
     *   4. 如果离线，清除ORE错误
     *
     * @note 在主循环或任务中周期性调用（推荐10ms周期）
     */
    bool DR16::IsOffline()
    {
        // 更新当前时间
        remote_state_watch_.UpdateTime();

        // 检查状态（计算时间差）
        remote_state_watch_.CheckStatus();

        // 获取状态
        bool is_offline = (remote_state_watch_.GetStatus() == WATCH_STATE::Status::OFFLINE);

        // 如果离线，清除ORE错误并归零摇杆/开关
        if (is_offline)
        {
            // 归零摇杆
            joystick_right_ = Vector::zero();
            joystick_left_  = Vector::zero();
            mouse_vel_      = Vector::zero();

            // 开关置UNKNOWN
            switch_right_ = Switch::UNKNOWN;
            switch_left_  = Switch::UNKNOWN;

            // 归零鼠标按键和键盘
            mouse_    = Mouse::zero();
            keyboard_ = Keyboard::zero();
            wheel_    = 0.0;

            // 清除ORE错误，防止UART死锁
            ClearORE(&DR16_UART, rx_buffer_, DR16_MAX_LEN);
        }

        return is_offline;
    }

}  // namespace BSP::Remote