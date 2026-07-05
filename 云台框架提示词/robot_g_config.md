# RM2026 三关节可变形云台配置（Robot Config）

---

# 项目信息

项目名称：RM2026 Transform Gimbal

机器人类型：RoboMaster 三关节可变形云台

赛季：RM2026

软件版本：RM2026 Gimbal Control System V1.0

开发方式：模块化开发

开发理念：
- 继承优先
- 架构优先
- 调试优先
- 稳定优先
- 重构最后

---

# 云台机构

本项目采用三自由度可变形云台。

关节拓扑如下：
```
        Pitch
          │
          │
      Fold Joint
          │
          │
         Yaw
          │
       Chassis
```

三个关节分别为：
- Yaw：控制整个云台左右旋转。
- Fold：控制云台展开、折叠及机器人形态变化。
- Pitch：控制枪口俯仰。

其中：Fold 不属于瞄准关节。Fold 属于机器人形态控制关节。

---

# 主控配置

主控芯片：STM32F407ZGT6

开发环境：Keil MDK5

HAL库：STM32 HAL

系统频率：168MHz

控制周期：1000Hz（1ms）

RTOS：待确定（当前裸机开发）

---

# 电机配置

## Yaw 电机

型号：DM4310

数量：1

通信方式：CAN

CAN ID：0x01

Master ID：0x01

控制模式：MIT力矩模式

功能：控制整个云台水平旋转。

---

## Pitch 电机

型号：DM4310

数量：1

通信方式：CAN

CAN ID：0x02

Master ID：0x02

控制模式：MIT力矩模式

功能：控制枪管俯仰。

---

## Fold 电机

型号：DM4340

数量：1

通信方式：CAN

CAN ID：0x03

Master ID：0x03

控制模式：MIT力矩模式

功能：控制云台展开、折叠。属于机器人形态控制关节。

---

# 通信配置

通信总线：CAN

CAN接口：CAN1

波特率：1Mbps

控制频率：1000Hz

通信协议：DM Motor CAN Protocol

---

# 遥控器配置

型号：DR16

通信方式：UART