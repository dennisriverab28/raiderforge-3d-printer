#ifndef THERMISTOR_H
#define THERMISTOR_H

#include "main.h"
#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════
// THERMISTOR MODULE
// Handles ADC reading and temperature conversion
// Shared by heater.c and sys_id.c to eliminate code duplication
// ═══════════════════════════════════════════════════════════════════════════

typedef struct {
    float thermistor_nominal;    // Resistance at 25°C (100kΩ for MF52B)
    float temp_nominal;          // Datasheet reference temperature (25°C)
    float b_coefficient;         // Beta value from datasheet (3950)
    float series_resistor;       // Series resistor value (measured with multimeter)
    float temp_offset_low;       // Calibration offset at room temp
    float temp_offset_high;      // Calibration offset at high temp (220°C)
    float calibration_crossover; // Temperature where offset method switches
} Thermistor_Config_t;

typedef enum {
    THERMISTOR_STATUS_OK = 0,
    THERMISTOR_STATUS_SENSOR_SHORT,
    THERMISTOR_STATUS_SENSOR_OPEN,
    THERMISTOR_STATUS_INVALID_RESISTANCE,
    THERMISTOR_STATUS_INVALID_TEMPERATURE
} Thermistor_Status_t;

extern const Thermistor_Config_t THERMISTOR_DEFAULT_CONFIG;

void     Thermistor_Init(const Thermistor_Config_t *config);
void     Thermistor_RegisterHeaterPWM(TIM_HandleTypeDef *timer, uint32_t channel);
uint16_t Thermistor_ReadADC(ADC_HandleTypeDef *adc);
uint16_t Thermistor_ReadADC_Single(ADC_HandleTypeDef *adc);
float    Thermistor_ADCToTemp(uint16_t adc_value);
float    Thermistor_ReadTemp(ADC_HandleTypeDef *adc);
float    Thermistor_GetRawTemp(uint16_t adc_value);
float    Thermistor_ADCToResistance(uint16_t adc_value);
uint16_t Thermistor_GetLastADC(void);
uint8_t  Thermistor_IsFault(uint16_t adc_value);
Thermistor_Status_t Thermistor_GetLastStatus(void);
const char *Thermistor_GetStatusText(Thermistor_Status_t status);

#endif // THERMISTOR_H
