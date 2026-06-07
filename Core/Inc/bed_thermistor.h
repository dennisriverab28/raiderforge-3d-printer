/*
 * bed_thermistor.h
 *
 * Bed thermistor module — separate static state from thermistor.c
 * so hotend and bed configs don't clobber each other.
 *
 * API mirrors thermistor.h exactly (BedTherm_ prefix).
 * Default config: 100kΩ NTC, B=3950, series_resistor=4700Ω (standard bed circuit)
 *
 * Created: Feb 2026
 * Author: Cherryman125
 */

#ifndef BED_THERMISTOR_H
#define BED_THERMISTOR_H

#include "stm32g0xx_hal.h"
#include "thermistor.h"
#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════

typedef struct {
    float thermistor_nominal;      // NTC resistance at temp_nominal (ohms)
    float temp_nominal;            // Temperature at nominal resistance (°C)
    float b_coefficient;           // Beta coefficient (K)
    float series_resistor;         // Fixed series resistor value (ohms)
    float temp_offset_low;         // Calibration offset below crossover (°C)
    float temp_offset_high;        // Calibration offset above crossover (°C)
    float calibration_crossover;   // Temp where offset transitions (°C)
} BedTherm_Config_t;

// Default config — same 100kΩ NTC as hotend but standard 4.7kΩ series resistor
// TODO: Measure your actual series resistor and update series_resistor if needed
extern const BedTherm_Config_t BED_THERMISTOR_DEFAULT_CONFIG;

// ═══════════════════════════════════════════════════════════════════════════
// API
// ═══════════════════════════════════════════════════════════════════════════

void     BedTherm_Init(const BedTherm_Config_t *user_config);

uint16_t BedTherm_ReadADC(ADC_HandleTypeDef *adc);
uint16_t BedTherm_ReadADC_Single(ADC_HandleTypeDef *adc);

float    BedTherm_ADCToTemp(uint16_t adc_value);
float    BedTherm_ReadTemp(ADC_HandleTypeDef *adc);
float    BedTherm_GetRawTemp(uint16_t adc_value);

uint16_t BedTherm_GetLastADC(void);
uint8_t  BedTherm_IsFault(uint16_t adc_value);
Thermistor_Status_t BedTherm_GetLastStatus(void);
const char *BedTherm_GetStatusText(Thermistor_Status_t status);

#endif /* BED_THERMISTOR_H */
