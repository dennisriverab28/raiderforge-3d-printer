#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

void TIMEBASE_Init(void);
uint32_t millis(void);
void TIMEBASE_OnTickISR(void);

#endif
