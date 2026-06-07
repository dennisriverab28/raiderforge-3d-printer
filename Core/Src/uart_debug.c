#include "uart_debug.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *g_huart = NULL;
static SemaphoreHandle_t g_uartdbg_mutex = NULL;

void UARTDBG_Init(UART_HandleTypeDef *huart)
{
    g_huart = huart;
    if (g_uartdbg_mutex == NULL) {
        g_uartdbg_mutex = xSemaphoreCreateMutex();
    }
}

void UARTDBG_Print(const char *fmt, ...)
{
    if (!g_huart) return;

    char buf[256];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (g_uartdbg_mutex != NULL &&
        xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        if (xSemaphoreTake(g_uartdbg_mutex, pdMS_TO_TICKS(100U)) != pdTRUE) {
            return;
        }
    }

    HAL_UART_Transmit(g_huart, (uint8_t*)buf, (uint16_t)strlen(buf), 100U);

    if (g_uartdbg_mutex != NULL &&
        xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xSemaphoreGive(g_uartdbg_mutex);
    }
}
