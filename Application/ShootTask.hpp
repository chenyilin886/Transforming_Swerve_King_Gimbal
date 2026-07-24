/**
 * @file ShootTask.hpp
 * @brief 发射机构 FreeRTOS 任务接口声明
 *
 * 设计来源：
 *   参考 H_SG_Gimbal 参考工程 Task/ShootTask.hpp 的 C/C++ 桥接方式。
 *   CubeMX 在 freertos.c 中以 "As external" 模式生成任务创建代码：
 *     extern void shootTask(void *argument);
 *     ShootTaskHandle = osThreadNew(shootTask, NULL, &ShootTask_attributes);
 *   任务函数体由本工程在 ShootTask.cpp 实现，CubeMX 重新生成不会覆盖。
 *
 * 任务职责：
 *   周期性调用 Class_ShootFSM::Control()，驱动整个发射机构：
 *     - 拨盘子系统（LK4005 位置环+速度环+单击/长按+卡弹检测）
 *     - 摩擦轮子系统（预留，DJI 3508 速度环，暂未接入）
 *     - 整体状态机（DISABLE/STOP/STANDBY/AUTO）
 *
 * 任务参数（CubeMX 配置）：
 *   - 任务名:      ShootTask
 *   - 栈大小:      768 words = 3072 bytes (3KB)
 *   - 优先级:      osPriorityNormal (与 GimbalTask 同级)
 *   - 周期:        4ms = 250Hz (osDelay(4))
 *
 * 数据流：
 *   DR16.wheel/S1/S2 ──┐
 *                       ├─→ Class_ShootFSM::Control()
 *   LK4005 反馈 ────────┤     │
 *   (预留) 3508 反馈 ───┤     ├─→ 状态机切换
 *                       │     │
 *                       │     ├─→ DialController.Update() → LK4005.ctrl_Torque
 *                       │     │
 *                       │     └─→ (预留) FrictionController → 3508.ctrl_Current
 *                       │
 *                       └─→ Shoot_Status (Watch 观察)
 *
 * 共享资源分析：
 *   - dial_ctrl 实例：仅在 ShootTask 中访问，无竞争
 *   - Dial_Config / Dial_Status：ShootTask 读写，Watch 异步只读
 *   - DR16 单例：ShootTask 读 wheel/S1/S2，GimbalTask 读摇杆，只读不冲突
 *   - LK4005 电机实例：
 *       ShootTask → ctrl_Torque() 写 CAN 邮箱
 *       GimbalTask → getAngleRad() 读 float (32位对齐, 单指令, 原子性可接受)
 *       CAN 中断 → Parse() 更新 unit_data_
 *     无需加锁（参考工程也是这样做的）
 *
 * @note 本头文件只声明任务函数，实现见 ShootTask.cpp
 */
#ifndef SHOOT_TASK_HPP
#define SHOOT_TASK_HPP

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 发射机构 FreeRTOS 任务函数
 *
 * 由 CubeMX 在 freertos.c 中通过 osThreadNew() 创建。
 * 函数体内执行无限循环，每周期调用 Class_ShootFSM::Control()。
 *
 * @param argument 任务参数（未使用，CubeMX 传 NULL）
 *
 * @note C 链接签名，匹配 CubeMX 生成的 extern "C" 声明
 */
void shootTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif // SHOOT_TASK_HPP
