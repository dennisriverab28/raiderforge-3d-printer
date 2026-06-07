#include "endstop.h"

// Debounce time in milliseconds
#define DEBOUNCE_MS 10

// ═══════════════════════════════════════════════════════════════════════════
// INIT
// ═══════════════════════════════════════════════════════════════════════════

void Endstop_Init(Endstop_t *es, GPIO_TypeDef *port, uint16_t pin, uint8_t is_nc)
{
    // Store config
    es->port = port;
    es->pin = pin;
    es->is_nc = is_nc ? 1 : 0;

    // Init state
    es->triggered = 0;
    es->raw = 0;
    es->last_stable = 0;
    es->last_change_ms = HAL_GetTick();
    es->just_triggered = 0;
    es->just_released = 0;

    // Do initial read
    Endstop_Update(es);

    // Clear edge flags after initial read
    es->just_triggered = 0;
    es->just_released = 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// GET RAW GPIO
// ═══════════════════════════════════════════════════════════════════════════

uint8_t Endstop_GetRaw(Endstop_t *es)
{
    return (HAL_GPIO_ReadPin(es->port, es->pin) == GPIO_PIN_SET) ? 1 : 0;
}

uint8_t Endstop_IsTriggeredImmediate(Endstop_t *es)
{
    uint8_t raw = Endstop_GetRaw(es);
    if (es->is_nc) {
        return raw;
    }
    return !raw;
}

// ═══════════════════════════════════════════════════════════════════════════
// UPDATE (call every loop)
// ═══════════════════════════════════════════════════════════════════════════

void Endstop_Update(Endstop_t *es)
{
    // Clear edge flags
    es->just_triggered = 0;
    es->just_released = 0;

    // Read raw state
    es->raw = Endstop_GetRaw(es);

    // Determine if triggered based on switch type
    //
    // NC (Normally Closed) with pull-up:
    //   - Not pressed: switch closed -> GPIO pulled to GND -> raw = 0
    //   - Pressed: switch open -> pull-up pulls HIGH -> raw = 1
    //   - So: raw = 1 means TRIGGERED
    //
    // NO (Normally Open) with pull-up:
    //   - Not pressed: switch open -> pull-up pulls HIGH -> raw = 1
    //   - Pressed: switch closed -> GPIO pulled to GND -> raw = 0
    //   - So: raw = 0 means TRIGGEREDA

    uint8_t is_triggered_now;
    if (es->is_nc) {
        is_triggered_now = es->raw;      // NC: HIGH = triggered
    } else {
        is_triggered_now = !es->raw;     // NO: LOW = triggered
    }

    // Debounce logic
    if (is_triggered_now != es->last_stable) {
        // State changed, restart debounce timer
        es->last_stable = is_triggered_now;
        es->last_change_ms = HAL_GetTick();
    }

    // Check if stable long enough
    if ((HAL_GetTick() - es->last_change_ms) >= DEBOUNCE_MS) {
        // Stable - check if different from current triggered state
        if (es->last_stable != es->triggered) {
            // State change confirmed
            uint8_t old = es->triggered;
            es->triggered = es->last_stable;

            // Set edge flags
            if (es->triggered && !old) {
                es->just_triggered = 1;
            }
            if (!es->triggered && old) {
                es->just_released = 1;
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// QUERIES
// ═══════════════════════════════════════════════════════════════════════════

uint8_t Endstop_IsTriggered(Endstop_t *es)
{
    return es->triggered;
}

uint8_t Endstop_JustTriggered(Endstop_t *es)
{
    return es->just_triggered;
}

uint8_t Endstop_JustReleased(Endstop_t *es)
{
    return es->just_released;
}
