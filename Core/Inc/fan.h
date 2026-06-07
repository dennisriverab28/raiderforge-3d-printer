#ifndef FAN_H
#define FAN_H

#include "stm32g0xx_hal.h"
#include "heater_types.h"
#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════
// FAN CONTROL MODULE
// Supports both PWM speed control and simple on/off switching
// ═══════════════════════════════════════════════════════════════════════════

typedef enum {
    FAN_MODE_OFF = 0,           // Fan completely off
    FAN_MODE_ON_FULL,           // Fan at 100% (no PWM control)
    FAN_MODE_PWM                // Fan speed controlled by PWM (0-100%)
} Fan_Mode_t;

typedef struct {
    TIM_HandleTypeDef *pwm_timer;   // Timer for PWM generation (can be NULL if on/off only)
    uint32_t pwm_channel;           // Timer channel for PWM
    GPIO_TypeDef *gpio_port;        // GPIO port for on/off control (optional)
    uint16_t gpio_pin;              // GPIO pin for on/off control (optional)

    Fan_Mode_t mode;                // Current operating mode
    uint8_t speed_percent;          // Current speed (0-100%)
    uint8_t enabled;                // 1 if fan is running, 0 if off
} Fan_t;

// ═══════════════════════════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Initialize fan control
 * @param fan Pointer to fan struct
 * @param pwm_timer Timer handle for PWM (NULL if on/off only)
 * @param pwm_channel Timer channel for PWM
 * @param gpio_port GPIO port for on/off control (NULL if PWM only)
 * @param gpio_pin GPIO pin for on/off control
 */
void Fan_Init(Fan_t *fan,
              TIM_HandleTypeDef *pwm_timer,
              uint32_t pwm_channel,
              GPIO_TypeDef *gpio_port,
              uint16_t gpio_pin);

/**
 * Turn fan on at full speed (100%)
 * @param fan Pointer to fan struct
 */
void Fan_On(Fan_t *fan);

/**
 * Turn fan off
 * @param fan Pointer to fan struct
 */
void Fan_Off(Fan_t *fan);

/**
 * Set fan speed (PWM mode)
 * @param fan Pointer to fan struct
 * @param speed_percent Speed as percentage (0-100%)
 */
void Fan_SetSpeed(Fan_t *fan, uint8_t speed_percent);

/**
 * Get current fan speed
 * @param fan Pointer to fan struct
 * @return Current speed (0-100%)
 */
uint8_t Fan_GetSpeed(Fan_t *fan);

/**
 * Check if fan is running
 * @param fan Pointer to fan struct
 * @return 1 if running, 0 if off
 */
uint8_t Fan_IsRunning(Fan_t *fan);

/**
 * Get current operating mode
 * @param fan Pointer to fan struct
 * @return Current mode (OFF, ON_FULL, or PWM)
 */
Fan_Mode_t Fan_GetMode(Fan_t *fan);

void PartFan_ModuleInit(void);
void PartFan_Task(void *pvParameters);
Fan_State_t PartFan_GetState(void);

#endif // FAN_H
