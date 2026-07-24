/**
 * @file ShootTask.cpp
 * @brief 发射机构 FreeRTOS 任务函数实现
 *
 * 实现 ShootTask.hpp 中声明的 shootTask() 函数。
 *
 * 任务职责：
 *   周期性调用 Class_ShootFSM::Control()，驱动整个发射机构。
 *
 * 任务周期：4ms = 250Hz
 *   - 与参考工程 ShootTask 一致
 *   - 拨盘双环 PID 4ms 够用（位置环带宽 < 50Hz）
 *   - 摩擦轮速度环 4ms 够用（速度环带宽 < 100Hz）
 *
 * 数据流：
 *   shootTask (4ms)
 *     └─→ shoot_fsm.Control()
 *           ├─→ updateStateMachine_()        // 安全检查 + 状态切换
 *           ├─→ applyStateToDialConfig_()    // state → Dial_Config.enabled
 *           ├─→ dial_ctrl.Update(...)        // 拨盘双环控制
 *           ├─→ updateFriction_()            // 预留: 摩擦轮控制(空)
 *           └─→ syncStatus_()                // 回写 Shoot_Status
 *
 * 实例管理：
 *   - shoot_fsm 作为函数内 static 局部变量，单例语义
 *   - 不需要外部初始化，首次进入任务时自动构造
 *
 * 与 GimbalTask 的关系：
 *   - GimbalTask (1ms): 处理 Yaw/Pitch/Fold 关节控制 + DR16 摇杆
 *   - ShootTask  (4ms): 处理拨盘 + 摩擦轮(预留) + DR16 wheel/S1/S2
 *   - 两者优先级相同 (osPriorityNormal)，FreeRTOS 时间片轮转
 *   - 共享资源：DR16 单例（只读）、LK4005 电机实例（读反馈+写命令）
 *   - 不需要加锁（参考工程也是这样做的）
 *
 * @note CubeMX 在 freertos.c 中以 "As external" 模式生成调用：
 *         ShootTaskHandle = osThreadNew(shootTask, NULL, &ShootTask_attributes);
 *       本文件实现函数体，CubeMX 重新生成不会覆盖。
 */
#include "ShootTask.hpp"
#include "ShootFSM.hpp"
#include "cmsis_os.h"  // osDelay

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 发射机构 FreeRTOS 任务函数
 *
 * @param argument 任务参数（未使用，CubeMX 传 NULL）
 *
 * @note 无限循环，每 4ms 调用一次 shoot_fsm.Control()
 *       函数永不返回（FreeRTOS 任务约定）
 */
void shootTask(void *argument)
{
    (void)argument;  // 未使用参数，消除警告

    // ================================================================
    // 发射机构状态机实例（static = 单例，首次进入任务时构造）
    // ================================================================
    // - 包含 DialController（拨盘双环 PID）
    // - 包含状态机字段（DISABLE/STOP/STANDBY/AUTO）
    // - 预留摩擦轮控制接口
    static BSP::FSM::Class_ShootFSM shoot_fsm;

    // ================================================================
    // 任务主循环：4ms 周期 (250Hz)
    // ================================================================
    // 周期选择理由：
    //   - 4ms 与参考工程 ShootTask 一致
    //   - 拨盘位置环带宽 < 50Hz，4ms 足够
    //   - 摩擦轮速度环带宽 < 100Hz，4ms 足够
    //   - 不需要 1ms 高频，减轻 CPU 负载
    for (;;)
    {
        // ----------------------------------------------------------
        // 发射机构周期控制
        // ----------------------------------------------------------
        // 内部执行（见 ShootFSM.cpp）：
        //   1. 安全条件检查（遥控器离线 / 急停 / feature_enable）
        //   2. 状态机切换（DISABLE ↔ AUTO）
        //   3. state → Dial_Config.enabled 联动
        //   4. dial_ctrl.Update() 拨盘双环 PID + 卡弹检测
        //   5. updateFriction_() 摩擦轮控制（当前空）
        //   6. 回写 Shoot_Status 供 Watch 观察
        // ----------------------------------------------------------
        shoot_fsm.Control();

        // ----------------------------------------------------------
        // FreeRTOS 延时 4ms = 4 个 tick（tick 周期 1ms）
        // ----------------------------------------------------------
        // osDelay 会让出 CPU，FreeRTOS 调度器切换到其他任务
        // （GimbalTask / defaultTask / IDLE）
        // ----------------------------------------------------------
        osDelay(4);
    }
}

#ifdef __cplusplus
}
#endif
