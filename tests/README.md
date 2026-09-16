# 双视觉挡位回归检查

运行 `powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_dual_vision.ps1`。
需要 PATH 中有 MinGW `g++`。脚本在临时目录生成并清理测试程序。

测试执行真实的 `ShootFSM.cpp`、`DialController.hpp` 和 `PID.cpp`，仅用
`stubs/` 替换时间、遥控器和电机 IO，不连接硬件。覆盖：

- 上上、上下挡位摩擦轮开启、视觉独占；`fire=0` 屏蔽 wheel，`vision_ready=0` 不影响拨盘仲裁。
- 视觉断连/恢复、任一摩擦轮离线后的触发源切换及等待释放。
- 切换配置保留已提交单发目标，停止连发累计目标，不重复触发。
- 两套配置的连发延时、视觉连发频率、手动拨轮阈值和幅度调速独立生效。
- 经过中位使用原配置并关闭摩擦轮，鼠标左键回退仍有效。
- 当前配置的 enabled/clear_pid、关闭功能、遥控器离线和急停优先于 fire。

## Watch 调参

| 挡位 | 配置 | `Shoot_Status.dial_config_mode` |
| --- | --- | --- |
| S1上 S2上 | `Dial_Config_UpUp` | 1 |
| S1上 S2下 | `Dial_Config_UpDown` | 2 |
| 其他 | `Dial_Config` | 0 |

两套新配置启动时各自复制当前公共默认值，此后独立存储，不会周期性互相覆盖。
`Application/Variable.cpp` 中各自的四项触发参数也可独立修改后编译。
本次保留工作区已有的 `long_press_ms=800`；两套初始参数均为
`wheel_start_threshold=0.5`、`long_press_ms=800`、`auto_fire_hz=5`、`wheel_to_hz=0`。

`long_press_ms` 和 `auto_fire_hz` 同时影响视觉持续 fire；拨轮阈值和
`wheel_to_hz` 只影响手动拨轮。视觉在线且两侧摩擦轮在线时，视觉独占拨盘，
不额外检查 ready 或转速到位。`Shoot_Status.vision_control=1` 表示该独占状态。
`trigger_source` 为实际接受的触发：0=无，1=wheel，2=fire，3=鼠标左键。
`waiting_release=1` 时先释放当前选中信号，再重新触发；启动/重新使能也要求先释放。

云台瞄准另要求 `vision_ready`，变形和急停优先。视觉实际接管时使用视觉 PID
和 Yaw 编码器速度反馈；退出时恢复手动参数并贴合反馈目标，不覆盖变形规划目标。
S2 经过中位被采样到时会执行原有中位模式，不延迟保留摩擦轮或视觉。

## 实机验证范围

主机测试不覆盖真实 CAN、IMU、USB 时序或机械运动。固件编译通过后，仍需实机
确认两挡位瞄准、视觉丢失后的手动接管、PID/反馈源恢复、变形优先，以及供弹频率。
本测试不烧录固件。
