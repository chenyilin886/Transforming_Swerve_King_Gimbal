#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void VisionComm_USBReceive(const uint8_t *data, uint32_t len);
void VisionComm_USBTxComplete(void);

#ifdef __cplusplus
}
#endif
