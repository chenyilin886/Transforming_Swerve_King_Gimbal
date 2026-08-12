# Requirements Document

## Introduction

本功能实现遥控器、Yaw电机和Pitch电机的断联检测系统。当设备断联时，系统通过蜂鸣器发出不同的声音提示，并通过LED显示不同颜色来指示具体是哪个设备掉线。

## Glossary

- **StateWatch**: 设备在线状态监视器，通过定时检查设备数据更新时间来判断设备是否在线
- **BuzzerManager**: 蜂鸣器管理器，提供基于队列的蜂鸣器管理功能
- **Dir_Event**: 断联事件管理器，使用观察者模式通知各个观察者（LED、蜂鸣器）
- **Yaw_Motor**: Yaw轴电机（DM4310），用于云台水平旋转
- **Pitch_Motor**: Pitch轴电机（DM4310），用于云台俯仰
- **Remote**: 遥控器（DR16），用于操控机器人

## Requirements

### Requirement 1: 遥控器断联检测

**User Story:** As a 操作手, I want 系统能检测遥控器断联, so that 我能及时发现通信问题并采取措施。

#### Acceptance Criteria

1. WHEN 遥控器数据超过50ms未更新, THE StateWatch SHALL 判定遥控器为离线状态
2. WHEN 遥控器断联, THE BuzzerManager SHALL 发出一声长音提示
3. WHEN 遥控器断联, THE LED SHALL 显示特定颜色（建议：白色）
4. WHEN 遥控器恢复连接, THE StateWatch SHALL 判定遥控器为在线状态

### Requirement 2: Yaw电机断联检测

**User Story:** As a 操作手, I want 系统能检测Yaw电机断联, so that 我能及时发现电机故障。

#### Acceptance Criteria

1. WHEN Yaw电机数据超过100ms未更新, THE StateWatch SHALL 判定Yaw电机为离线状态
2. WHEN Yaw电机断联, THE BuzzerManager SHALL 发出对应电机ID次数的短音提示
3. WHEN Yaw电机断联, THE LED SHALL 显示红色
4. WHEN Yaw电机恢复连接, THE StateWatch SHALL 判定Yaw电机为在线状态

### Requirement 3: Pitch电机断联检测

**User Story:** As a 操作手, I want 系统能检测Pitch电机断联, so that 我能及时发现电机故障。

#### Acceptance Criteria

1. WHEN Pitch电机数据超过100ms未更新, THE StateWatch SHALL 判定Pitch电机为离线状态
2. WHEN Pitch电机断联, THE BuzzerManager SHALL 发出对应电机ID次数的短音提示
3. WHEN Pitch电机断联, THE LED SHALL 显示蓝色
4. WHEN Pitch电机恢复连接, THE StateWatch SHALL 判定Pitch电机为在线状态

### Requirement 4: LED颜色指示

**User Story:** As a 操作手, I want 通过LED颜色快速判断掉线设备, so that 我能快速定位问题。

#### Acceptance Criteria

1. WHEN 遥控器断联, THE LED SHALL 显示白色
2. WHEN Yaw电机断联, THE LED SHALL 显示红色
3. WHEN Pitch电机断联, THE LED SHALL 显示蓝色
4. WHEN IMU断联, THE LED SHALL 显示粉色（已实现）
5. WHEN 所有设备在线, THE LED SHALL 显示正常流水灯效果
6. IF 多个设备同时断联, THEN THE LED SHALL 按优先级显示（遥控器 > IMU > Yaw > Pitch）

### Requirement 5: 事件更新机制

**User Story:** As a 系统, I want 在EvenTask中周期性检测设备状态, so that 能及时发现断联情况。

#### Acceptance Criteria

1. THE EvenTask SHALL 每1ms调用一次UpEvent()更新设备状态
2. WHEN 设备状态变化, THE Dir_Event SHALL 通知所有观察者（LED、Buzzer）
3. THE Dir_Data_t SHALL 包含所有设备的断联状态标志
