/**
 * @file GimbalInit.hpp
 * @brief 云台系统初始化与周期更新接口
 *
 * 设计原因：
 *   集中管理云台系统的初始化流程(CAN + 电机 + 回调注册 + Controller)，
 *   并提供周期更新接口用于刷新可观测变量 + 运行控制循环。
 *   将初始化逻辑与 main.c 解耦，保持 main.c 简洁。
 *
 * 使用方法：
 *   main() {
 *     HAL_Init();
 *     SystemClock_Config();
 *     MX_GPIO_Init();
 *     MX_CAN1_Init();
 *     MX_CAN2_Init();
 *     GimbalInit();           // ← 初始化 CAN + 电机 + Joint + Controller
 *     while (1) {
 *       GimbalUpdate();      // ← 控制循环 + 刷新 Watch 变量
 *       HAL_Delay(1);
 *     }
 *   }
 *
 * Stage03 数据流：
 *   GimbalInit():
 *     get_can_bus_instance() → 初始化 CAN1/CAN2(过滤器+启动+中断)
 *     创建 DM4310(&can1) + DM4340(&can1) 实例
 *     can1.register_rx_callback(DM4310::Parse)
 *     can1.register_rx_callback(DM4340::Parse)
 *     JointManager.Init()
 *     GimbalController.Init()  ← Stage03 新增(默认 pitch.Enable)
 *     DM4310/DM4340.On(id)
 *
 *   GimbalUpdate()(每周期 5 步):
 *     Step1: Watch → Joint config(在线改 offset/limit/...)
 *     Step2: JointManager.Update(Motor → Joint feedback)
 *     Step3: Watch → GimbalController(在线改 kp/ki/kd/target/enabled)
 *     Step4: GimbalController.Update → Motor.ctrl_Mit(纯力矩)
 *     Step5: Joint state + Controller state → Watch
 *
 *   Watch → Joint → Controller → Motor 完整闭环(Stage03):
 *     Controller_Data.pitch.target_angle (Watch 写入)
 *       → GimbalController.pitch.target_angle
 *       → PID(target, Joint.feedback_angle) → torque
 *       → DM4310.ctrl_Mit(2, 0, 0, 0, 0, torque)
 *       → 电机响应 → Encoder → Joint → Controller_Data.feedback_angle (Watch 观察)
 */
#ifndef GIMBAL_INIT_HPP
#define GIMBAL_INIT_HPP

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 云台系统初始化
 *
 * 执行以下操作(顺序敏感)：
 *   1. 获取 CAN 总线单例(触发过滤器配置 + 启动外设 + 使能接收中断)
 *   2. 创建 DM4310(Yaw+Pitch) 和 DM4340(Fold) 电机实例
 *   3. 在 CAN1 上注册电机 Parse 回调
 *
 * @note  必须在 MX_CAN1_Init / MX_CAN2_Init 之后调用
 */
void GimbalInit();

/**
 * @brief 云台系统周期更新(在主循环中调用)
 *
 * 执行以下操作：
 *   1. 发送 MIT 零力矩控制帧(保持电机在线)
 *   2. Watch config → Joint(允许在线修改 offset/limit/direction)
 *   3. JointManager.Update → Joint.Update(计算 real_angle/normalized_angle)
 *   4. Joint state → Joint_Data(Watch 实时观察)
 *
 * @note  调用频率建议 ≥100Hz，确保在线检测时间戳精度
 */
void GimbalUpdate();

#ifdef __cplusplus
}
#endif

#endif // GIMBAL_INIT_HPP
