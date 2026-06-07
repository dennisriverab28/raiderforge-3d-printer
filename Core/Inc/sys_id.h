#ifndef SYS_ID_H
#define SYS_ID_H

#include "stm32g0xx_hal.h"
#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════
// SYSTEM IDENTIFICATION MODULE
//
// Purpose: Find the transfer function G(s) = K/(τs+1) for the heater
//
// What this does:
// 1. Set PWM to a fixed value (e.g., 50%)
// 2. Watch temperature rise over time
// 3. Calculate K (gain) and τ (time constant)
// 4. Use these to design better PID controller
//
// This is NOT PID control - it's open-loop testing!
// ═══════════════════════════════════════════════════════════════════════════

// ───────────────────────────────────────────────────────────────────────────
// Test Configuration Structure
//
// Defines the parameters for a step response test
// ───────────────────────────────────────────────────────────────────────────
typedef struct {
    uint8_t step_pwm;           // PWM level to test (0-100%)
                                // Example: 50 means test at 50% power

    uint16_t settle_time_s;     // How long to wait at 0% before step (seconds)
                                // Why: Let heater cool to room temp first
                                // Example: 30 seconds

    uint16_t test_duration_s;   // How long to record after step (seconds)
                                // Why: Need to reach steady-state temperature
                                // Example: 120 seconds (2 minutes)

    uint16_t sample_period_ms;  // How often to measure temp (milliseconds)
                                // Why: Balance between data resolution and time
                                // Example: 1000ms = 1 reading per second
} SysID_Config_t;

// ───────────────────────────────────────────────────────────────────────────
// Test Results Structure
//
// Holds the calculated transfer function parameters
// ───────────────────────────────────────────────────────────────────────────
typedef struct {
    float initial_temp;         // Temperature before step input (°C)
                                // Example: 21.3°C (room temperature)

    float final_temp;           // Final steady-state temperature (°C)
                                // Example: 85.4°C (after heating at 50% PWM)

    float time_constant;        // τ (tau) - how fast system responds (seconds)
                                // Definition: Time to reach 63.2% of final value
                                // Example: 87.3 seconds

    float gain;                 // K - temperature change per %PWM (°C/%PWM)
                                // Formula: (final_temp - initial_temp) / step_pwm
                                // Example: 1.28 °C per %PWM

    uint8_t step_pwm;          // PWM level that was tested (%)
                                // Example: 50%
} SysID_Results_t;

// ═══════════════════════════════════════════════════════════════════════════
// PUBLIC FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

// ───────────────────────────────────────────────────────────────────────────
// Run a single step response test
//
// What it does:
// 1. Wait at 0% PWM to settle at room temp
// 2. Apply step input (e.g., jump to 50% PWM)
// 3. Record temperature every second
// 4. Calculate K and τ
// 5. Print transfer function G(s)
//
// Inputs:
//   pwm_timer  - Timer peripheral for PWM (e.g., &htim16)
//   pwm_channel - Timer channel (e.g., TIM_CHANNEL_1)
//   adc        - ADC peripheral for thermistor (e.g., &hadc1)
//   config     - Test parameters (PWM level, duration, etc.)
//   results    - Where to store calculated K, τ, etc.
// ───────────────────────────────────────────────────────────────────────────
void SysID_StepResponse(TIM_HandleTypeDef *pwm_timer,
                        uint32_t pwm_channel,
                        ADC_HandleTypeDef *adc,
                        SysID_Config_t *config,
                        SysID_Results_t *results);

// ───────────────────────────────────────────────────────────────────────────
// Run multiple step tests at different PWM levels
//
// What it does:
// Runs step response test at each PWM level (e.g., 10%, 20%, 30%...100%)
// Creates a PWM vs Steady-State Temperature graph
//
// Inputs:
//   pwm_timer  - Timer peripheral for PWM
//   pwm_channel - Timer channel
//   adc        - ADC peripheral
//   pwm_levels - Array of PWM percentages to test (e.g., [10,20,30,...,100])
//   num_levels - How many levels in the array (e.g., 10)
// ───────────────────────────────────────────────────────────────────────────
void SysID_MultiStep(TIM_HandleTypeDef *pwm_timer,
                     uint32_t pwm_channel,
                     ADC_HandleTypeDef *adc,
                     uint8_t *pwm_levels,
                     uint8_t num_levels);

// ───────────────────────────────────────────────────────────────────────────
// Calculate time constant (τ) from recorded temperature data
//
// What it does:
// Finds when temperature reaches 63.2% of final value
// That time is the time constant τ
//
// Why 63.2%? It's a property of first-order systems (e^-1 ≈ 0.632)
//
// Inputs:
//   initial_temp      - Starting temperature (°C)
//   final_temp        - Ending temperature (°C)
//   temp_array        - Array of all temperature readings
//   array_size        - Number of readings in array
//   sample_period_ms  - Time between readings (ms)
//
// Returns: Time constant τ in seconds
// ───────────────────────────────────────────────────────────────────────────
float SysID_CalculateTimeConstant(float initial_temp,
                                   float final_temp,
                                   float *temp_array,
                                   uint16_t array_size,
                                   uint16_t sample_period_ms);

// ───────────────────────────────────────────────────────────────────────────
// Print the transfer function G(s) in pretty format
//
// What it does:
// Prints G(s) = K/(τs + 1) with actual calculated values
// Example output:
//          1.28
//   G(s) = ──────────
//          87.3s + 1
// ───────────────────────────────────────────────────────────────────────────
void SysID_PrintTransferFunction(SysID_Results_t *results);

// ───────────────────────────────────────────────────────────────────────────
// Read temperature from thermistor
//
// What it does:
// 1. Takes 10 ADC readings
// 2. Filters out noise (removes highest/lowest)
// 3. Converts ADC to resistance using voltage divider
// 4. Converts resistance to temperature using Steinhart-Hart
// 5. Applies calibration offset
//
// Returns: Temperature in °C
// ───────────────────────────────────────────────────────────────────────────
float SysID_ReadTemp(ADC_HandleTypeDef *adc);

#endif // SYS_ID_H
