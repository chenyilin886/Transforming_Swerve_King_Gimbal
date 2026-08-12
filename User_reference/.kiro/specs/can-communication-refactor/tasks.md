# Implementation Plan: CAN Communication Refactor

## Overview

本实现计划将板间CAN通信从BSP层迁移到HAL层，优化发送为两帧，接收为单帧带帧头校验。所有修改基于现有代码进行最小化改动。

## Tasks

- [x] 1. 更新CommunicationTask.hpp头文件
  - 移除FRAME3相关宏定义
  - 更新can_tx_buffer为2帧
  - 简化接收相关变量
  - 添加HAL CAN头文件引用
  - _Requirements: 1.4, 2.1, 4.2_

- [x] 2. 重构Data_send()发送函数
  - [x] 2.1 替换BSP CAN调用为HAL CAN
    - 移除 `#include "../BSP/CAN/Bsp_Can.hpp"`
    - 添加 `#include "../HAL/CAN/can_hal.hpp"`
    - 获取CAN1设备实例
    - _Requirements: 1.1, 1.2, 1.4_
  
  - [x] 2.2 实现两帧数据打包和发送
    - 重新组织tx_data数组为14字节
    - Frame1: 帧头 + Direction前7字节
    - Frame2: Direction后2字节 + ChassisMode + UiList + 填充
    - 使用HAL::CAN::Frame结构体发送
    - _Requirements: 2.1, 2.2, 2.3, 2.4, 2.5, 2.6_

- [x] 3. 简化接收逻辑为单帧处理
  - [x] 3.1 更新HandleCANMessage()方法
    - 移除多帧重组逻辑
    - 实现单帧直接解析
    - 添加双字节帧头校验(0x21, 0x12)
    - _Requirements: 3.1, 3.2, 3.3, 3.4_
  
  - [x] 3.2 清理不再使用的接收变量
    - 移除frame1_received, frame2_received, frame3_received
    - 移除can_rx_buffer多帧缓冲
    - 移除ParseCANFrame()和ProcessReceivedData()方法
    - _Requirements: 4.2, 5.5_

- [x] 4. 更新CallBack.cpp回调处理
  - 确认CAN1回调中0x501 ID的处理
  - 简化为单帧处理调用
  - _Requirements: 3.5, 3.6_

- [x] 5. Checkpoint - 编译验证
  - 确保代码编译通过
  - 检查所有修改点是否正确
  - 如有问题请告知

- [ ]* 6. 编写单元测试
  - [ ]* 6.1 测试两帧数据打包正确性
    - **Property 2: 两帧数据打包完整性**
    - **Validates: Requirements 2.2, 2.3, 2.6**
  
  - [ ]* 6.2 测试帧头校验逻辑
    - **Property 3: 帧头校验正确性**
    - **Validates: Requirements 3.2, 3.3, 3.4**

## 修改点汇总

完成后将提供以下修改清单：

| 文件 | 修改类型 | 具体修改 |
|-----|---------|---------|
| Task/CommunicationTask.hpp | 修改 | 宏定义、缓冲区、变量 |
| Task/CommunicationTask.cpp | 修改 | 发送函数、接收函数、头文件 |
| Task/CallBack.cpp | 修改 | 回调处理逻辑 |

## Notes

- 任务标记 `*` 为可选测试任务
- 每个任务引用具体的需求条款以便追踪
- Checkpoint用于验证阶段性成果
- 保持Vision类通信逻辑不变
