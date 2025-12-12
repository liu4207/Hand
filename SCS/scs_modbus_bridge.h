#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

void SCS_Bridge_Init(void);
void SCS_Bridge_OnWrite(uint16_t startAddr, uint16_t len);
void SCS_Bridge_Poll_20ms(void);

#ifdef __cplusplus
}
#endif

