/**
 * @file stdxxx.hpp
 * @brief 基础类型与标准库统一入口
 *
 * 设计原因：
 *   工程内所有 C++ 模块统一通过 stdxxx.hpp 获取 uint8_t / uint16_t / uint32_t
 *   等基础类型，以及 math / string 等常用标准库。
 *   这样后续从参考工程(H_SG_Gimbal)继承模块时，include 路径保持一致，
 *   无需逐文件修改 include。
 *
 * 继承说明：
 *   内容与参考工程 User_reference/BSP/stdxxx.hpp 完全一致，
 *   保持兼容性便于后续移植 IMU / 遥控器 / 视觉等模块。
 */
#pragma once

#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
