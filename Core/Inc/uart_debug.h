#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include "stm32g0xx_hal.h"
#include <stdarg.h>
#include <stdint.h>

void UARTDBG_Init(UART_HandleTypeDef *huart);
void UARTDBG_Print(const char *fmt, ...);

#endif
