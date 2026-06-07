#include "timebase.h"
#include "stm32g0xx_hal.h"

static volatile uint32_t milliseconds = 0;

void TIMEBASE_Init(void)
{
    milliseconds = 0;
}

uint32_t millis(void)
{
    return milliseconds;
}

void TIMEBASE_OnTickISR(void)
{
    milliseconds++;  // ← NO underscores!
}
