#pragma once
#include <stdint.h>
extern "C" uint32_t HAL_GetTick(void);
inline uint32_t test_irq_mask = 0;
inline uint32_t __get_PRIMASK() { return test_irq_mask; }
inline void __disable_irq() { test_irq_mask = 1; }
inline void __set_PRIMASK(uint32_t value) { test_irq_mask = value; }
