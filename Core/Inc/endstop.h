#ifndef ENDSTOP_H
#define ENDSTOP_H

#include "stm32g0xx_hal.h"
#include <math.h>
#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════
// ENDSTOP STRUCT
// ═══════════════════════════════════════════════════════════════════════════

typedef struct {
    // Configuration (set during init)
    GPIO_TypeDef *port;      // GPIO port (GPIOA, GPIOB, GPIOC, etc.)
    uint16_t pin;            // GPIO pin (GPIO_PIN_0, GPIO_PIN_1, etc.)
    uint8_t is_nc;           // 1 = Normally Closed, 0 = Normally Open

    // State (updated by Endstop_Update)
    uint8_t triggered;       // Debounced state: 1 = triggered, 0 = not
    uint8_t raw;             // Raw GPIO reading
    uint8_t last_stable;     // Last stable state (for debounce)
    uint32_t last_change_ms; // Time of last state change
    uint8_t just_triggered;  // 1 = just became triggered this update
    uint8_t just_released;   // 1 = just became released this update
} Endstop_t;

// ═══════════════════════════════════════════════════════════════════════════
// FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

// Initialize endstop
// port: GPIO port (e.g., GPIOC)
// pin: GPIO pin (e.g., GPIO_PIN_3)
// is_nc: 1 for Normally Closed switch, 0 for Normally Open
void Endstop_Init(Endstop_t *es, GPIO_TypeDef *port, uint16_t pin, uint8_t is_nc);

// Update endstop state - call this in main loop
void Endstop_Update(Endstop_t *es);

// Check if currently triggered (debounced)
uint8_t Endstop_IsTriggered(Endstop_t *es);

// Check if currently triggered from the raw GPIO state (no debounce)
uint8_t Endstop_IsTriggeredImmediate(Endstop_t *es);

// Check if just became triggered this update cycle
uint8_t Endstop_JustTriggered(Endstop_t *es);

// Check if just became released this update cycle
uint8_t Endstop_JustReleased(Endstop_t *es);

// Get raw GPIO reading (for debugging)
uint8_t Endstop_GetRaw(Endstop_t *es);

#endif
