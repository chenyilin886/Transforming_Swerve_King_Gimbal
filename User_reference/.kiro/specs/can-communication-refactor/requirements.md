# Requirements Document

## Introduction

本文档定义了云台板间CAN通信改造的需求规格。目标是将现有的BSP层CAN调用迁移到HAL/CAN抽象层，优化数据帧结构，并在现有代码基础上进行最小化修改。

## Glossary

- **HAL_CAN**: HAL/CAN文件夹下的CAN抽象层接口，提供统一的CAN设备访问方式
- **BSP_CAN**: BSP/CAN文件夹下的旧版CAN调用接口
- **Gimbal_Board**: 云台板，负责发送方向、模式、UI状态数据
- **Chassis_Board**: 底盘板，负责发送裁判系统数据
- **Frame**: CAN数据帧，每帧最大8字节
- **Frame_Header**: 帧头，用于数据校验和识别
- **Direction_Data**: 方向数据，包含摇杆值、角度误差等
- **Mode_Data**: 模式数据，包含底盘运动模式标志位
- **UI_Status_Data**: UI状态数据，包含界面显示相关标志位
- **Referee_Data**: 裁判系统数据，包含热量、冷却等信息

## Requirements

### Requirement 1: HAL CAN接口迁移

**User Story:** 作为开发者，我希望将板间通信从BSP CAN迁移到HAL CAN接口，以便获得更好的抽象和可维护性。

#### Acceptance Criteria

1. WHEN CommunicationTask.cpp发送CAN数据 THEN THE System SHALL 使用HAL::CAN::ICanDevice::send()方法替代CAN::BSP::Can_Send()
2. WHEN 初始化CAN通信 THEN THE System SHALL 通过HAL::CAN::get_can_bus_instance().get_can1()获取CAN1设备实例
3. WHEN 发送CAN帧 THEN THE System SHALL 构造HAL::CAN::Frame结构体并设置正确的id、dlc、data字段
4. THE System SHALL 移除对BSP/CAN/Bsp_Can.hpp的依赖
5. THE System SHALL 保持与现有HAL CAN回调机制的兼容性

### Requirement 2: 云台发送数据帧结构优化

**User Story:** 作为开发者，我希望将云台发送的数据从三帧优化为两帧，以减少CAN总线负载并简化通信协议。

#### Acceptance Criteria

1. THE Gimbal_Board SHALL 将方向数据、模式数据、UI状态数据打包为两帧CAN数据发送
2. WHEN 发送第一帧 THEN THE System SHALL 包含帧头(1字节)和方向数据(Direction结构体)
3. WHEN 发送第二帧 THEN THE System SHALL 包含模式数据(ChassisMode)和UI状态数据(UiList)
4. THE System SHALL 使用0x401作为第一帧CAN ID
5. THE System SHALL 使用0x402作为第二帧CAN ID
6. WHEN 数据长度不足8字节 THEN THE System SHALL 用0填充至8字节

### Requirement 3: 云台接收裁判系统数据

**User Story:** 作为开发者，我希望云台能够通过单帧CAN接收底盘发送的裁判系统数据，并进行帧头校验。

#### Acceptance Criteria

1. THE Gimbal_Board SHALL 通过单帧CAN(8字节)接收裁判系统数据
2. WHEN 接收CAN帧 THEN THE System SHALL 检查帧头是否为预定义值(0x21和0x12双字节帧头)
3. IF 帧头校验失败 THEN THE System SHALL 丢弃该帧数据并保持原有状态
4. WHEN 帧头校验成功 THEN THE System SHALL 解析RxRefree结构体数据
5. THE System SHALL 使用0x501作为接收帧CAN ID
6. THE System SHALL 在CallBack.cpp中处理接收到的裁判系统数据帧

### Requirement 4: 代码修改追踪

**User Story:** 作为开发者，我希望清楚地知道哪些文件被修改了，以便进行代码审查和版本控制。

#### Acceptance Criteria

1. THE System SHALL 在CommunicationTask.cpp中修改Data_send()方法
2. THE System SHALL 在CommunicationTask.hpp中更新CAN ID定义(移除FRAME3相关定义)
3. THE System SHALL 在CommunicationTask.hpp中调整数据结构以适应两帧发送
4. THE System SHALL 在CallBack.cpp中更新CAN接收回调处理逻辑
5. THE System SHALL 保留原有的Vision类通信逻辑不变
6. WHEN 修改完成 THEN THE System SHALL 提供修改文件清单和修改点说明

### Requirement 5: 数据结构兼容性

**User Story:** 作为开发者，我希望保持现有数据结构的兼容性，避免影响其他模块。

#### Acceptance Criteria

1. THE System SHALL 保持Direction、ChassisMode、UiList、RxRefree结构体定义不变
2. THE System SHALL 保持Gimbal_to_Chassis类的公共接口不变
3. THE System SHALL 保持getter/setter方法的功能不变
4. IF 需要调整数据打包顺序 THEN THE System SHALL 确保底盘端同步更新解析逻辑
5. THE System SHALL 移除不再使用的IMU结构体(i_mu)从发送数据中

### Requirement 6: 错误处理与健壮性

**User Story:** 作为开发者，我希望通信模块具有基本的错误处理能力，确保系统稳定运行。

#### Acceptance Criteria

1. WHEN CAN发送失败 THEN THE System SHALL 不阻塞当前任务继续执行
2. WHEN 接收帧头校验失败 THEN THE System SHALL 记录错误但不影响后续接收
3. THE System SHALL 保持现有的帧超时检测机制(FRAME_TIMEOUT = 50ms)
4. IF 连续多帧丢失 THEN THE System SHALL 重置接收状态机
