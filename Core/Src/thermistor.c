/*
 * thermistor.c
 *
 *  Created on: Feb 22, 2026
 *      Author: Cherryman125
 */

#include "thermistor.h"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "qbc_globals.h"

const Thermistor_Config_t THERMISTOR_DEFAULT_CONFIG = {
    .thermistor_nominal    = 100000.0f, // MF52B 100kΩ NTC — confirmed from datasheet
    .temp_nominal          = 25.0f,
    .b_coefficient         = 3950.0f,   // confirmed from datasheet
    .series_resistor       = 4700.0f,   // 4.7k ohm series resistor — confirmed from hardware
    .temp_offset_low       = 0.0f,      // TODO: fine-tune after confirming room temp
    .temp_offset_high      = 0.0f,      // TODO: calibrate at 220°C with IR gun
    .calibration_crossover = 100.0f
};

static Thermistor_Config_t config;
static uint16_t last_adc_value = 0;
static Thermistor_Status_t last_status = THERMISTOR_STATUS_INVALID_TEMPERATURE;
static uint16_t filtered_adc_value = 0;
static uint8_t filter_valid = 0U;

#define ADC_SAMPLES              16
#define ADC_DISCARD_EXTREMES      3
#define ADC_MAX                4095.0f
#define HOTEND_ADC_CHANNEL     ADC_CHANNEL_9
#define ADC_JUMP_LIMIT           180U
#define ADC_EMA_ALPHA_NUM          1U
#define ADC_EMA_ALPHA_DEN          4U

static void switch_to_hotend_channel(ADC_HandleTypeDef *adc)
{
    ADC_ChannelConfTypeDef ch = {0};

    if (adc == NULL) {
        return;
    }

    HAL_ADC_Stop(adc);

    ch.Channel = HOTEND_ADC_CHANNEL;
    ch.Rank = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLINGTIME_COMMON_2;
    HAL_ADC_ConfigChannel(adc, &ch);
}


void Thermistor_Init(const Thermistor_Config_t *user_config)
{
    if (user_config != NULL) {
        config = *user_config;
    } else {
        config = THERMISTOR_DEFAULT_CONFIG;
    }

    last_adc_value = 0;
    last_status = THERMISTOR_STATUS_INVALID_TEMPERATURE;
    filtered_adc_value = 0;
    filter_valid = 0U;
}

void Thermistor_RegisterHeaterPWM(TIM_HandleTypeDef *timer, uint32_t channel)
{
    (void)timer;
    (void)channel;
}

// ─── OVERSAMPLED READ ────────────────────────────────────────────────────────
// 10 samples, trims 1 high + 1 low, averages middle 8. ~10ms blocking.
uint16_t Thermistor_ReadADC(ADC_HandleTypeDef *adc)
{
    uint16_t median;
    uint16_t candidate;
    uint16_t delta;
    uint32_t sum = 0U;
    int count = 0;

    if (adc == NULL) return 0;

    uint16_t samples[ADC_SAMPLES];

    if (xHeaterUpdateMutex != NULL &&
        xSemaphoreTake(xHeaterUpdateMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return last_adc_value;
    }

    switch_to_hotend_channel(adc);

    HAL_ADC_Start(adc);
    (void)HAL_ADC_PollForConversion(adc, 100);
    (void)HAL_ADC_GetValue(adc);

    for (int i = 0; i < ADC_SAMPLES; i++) {
        HAL_ADC_Start(adc);
        if (HAL_ADC_PollForConversion(adc, HAL_MAX_DELAY) == HAL_OK) {
            samples[i] = HAL_ADC_GetValue(adc);
        } else {
            samples[i] = 0;
        }
    }

    for (int i = 0; i < ADC_SAMPLES - 1; i++) {
        for (int j = 0; j < ADC_SAMPLES - i - 1; j++) {
            if (samples[j] > samples[j + 1]) {
                uint16_t tmp   = samples[j];
                samples[j]     = samples[j + 1];
                samples[j + 1] = tmp;
            }
        }
    }

    median = samples[ADC_SAMPLES / 2];

    for (int i = ADC_DISCARD_EXTREMES; i < ADC_SAMPLES - ADC_DISCARD_EXTREMES; i++) {
        delta = (samples[i] > median) ? (samples[i] - median) : (median - samples[i]);
        if (delta > ADC_JUMP_LIMIT) {
            continue;
        }
        sum += samples[i];
        count++;
    }

    candidate = (count > 0) ? (uint16_t)((sum + (uint32_t)(count / 2)) / (uint32_t)count) : median;

    if (filter_valid != 0U) {
        delta = (candidate > filtered_adc_value) ? (candidate - filtered_adc_value)
                                                 : (filtered_adc_value - candidate);
        if (delta > ADC_JUMP_LIMIT) {
            candidate = filtered_adc_value;
        } else {
            candidate = (uint16_t)((((uint32_t)filtered_adc_value * (ADC_EMA_ALPHA_DEN - ADC_EMA_ALPHA_NUM)) +
                                    ((uint32_t)candidate * ADC_EMA_ALPHA_NUM) +
                                    (ADC_EMA_ALPHA_DEN / 2U)) / ADC_EMA_ALPHA_DEN);
        }
    }

    filtered_adc_value = candidate;
    filter_valid = 1U;
    last_adc_value = candidate;

    if (xHeaterUpdateMutex != NULL) {
        xSemaphoreGive(xHeaterUpdateMutex);
    }

    return candidate;
}

// ─── SINGLE READ (no oversampling) ───────────────────────────────────────────
// One bare ADC conversion. ~0.1ms, noisier. Used in Test 1 Part B.
uint16_t Thermistor_ReadADC_Single(ADC_HandleTypeDef *adc)
{
    uint16_t val = 0;

    if (adc == NULL) return 0;

    if (xHeaterUpdateMutex != NULL &&
        xSemaphoreTake(xHeaterUpdateMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return last_adc_value;
    }

    switch_to_hotend_channel(adc);

    HAL_ADC_Start(adc);
    (void)HAL_ADC_PollForConversion(adc, 100);
    (void)HAL_ADC_GetValue(adc);

    HAL_ADC_Start(adc);
    if (HAL_ADC_PollForConversion(adc, HAL_MAX_DELAY) == HAL_OK) {
        val = HAL_ADC_GetValue(adc);
    }

    if (xHeaterUpdateMutex != NULL) {
        xSemaphoreGive(xHeaterUpdateMutex);
    }

    last_adc_value = val;
    return val;
}

// ─── TEMPERATURE CONVERSION ───────────────────────────────────────────────────
// Beta-form Steinhart-Hart: 1/T = 1/T₀ + (1/B) × ln(R/R₀)
// Circuit: 3.3V → [R_therm] → ADC_pin → [R_series] → GND
// R_therm = R_series × (4095 - ADC) / ADC
float Thermistor_ADCToTemp(uint16_t adc_value)
{
    if (adc_value < 5) {
        last_status = THERMISTOR_STATUS_SENSOR_SHORT;
        return 350.0f;
    }
    if (adc_value > 4090) {
        last_status = THERMISTOR_STATUS_SENSOR_OPEN;
        return -50.0f;
    }

    float resistance = config.series_resistor * (ADC_MAX - (float)adc_value) / (float)adc_value;

    if (resistance < 10.0f || resistance > 2000000.0f) {
        last_status = THERMISTOR_STATUS_INVALID_RESISTANCE;
        return -999.0f;
    }

    float steinhart = logf(resistance / config.thermistor_nominal);
    steinhart /= config.b_coefficient;
    steinhart += 1.0f / (config.temp_nominal + 273.15f);
    steinhart = 1.0f / steinhart;
    steinhart -= 273.15f;

    if (steinhart < config.calibration_crossover) {
        steinhart += config.temp_offset_low;
    } else {
        float blend = (steinhart - config.calibration_crossover) /
                      (220.0f - config.calibration_crossover);
        if (blend > 1.0f) blend = 1.0f;

        float offset = config.temp_offset_low  * (1.0f - blend) +
                       config.temp_offset_high * blend;
        steinhart += offset;
    }

    if (steinhart < -50.0f || steinhart > 300.0f) {
        last_status = THERMISTOR_STATUS_INVALID_TEMPERATURE;
        return -999.0f;
    }

    last_status = THERMISTOR_STATUS_OK;
    return steinhart;
}

float Thermistor_ADCToResistance(uint16_t adc_value)
{
    if (adc_value < 5U || adc_value > 4090U) {
        return NAN;
    }

    return config.series_resistor * (ADC_MAX - (float)adc_value) / (float)adc_value;
}

float Thermistor_ReadTemp(ADC_HandleTypeDef *adc)
{
    uint16_t adc_value = Thermistor_ReadADC(adc);
    return Thermistor_ADCToTemp(adc_value);
}

float Thermistor_GetRawTemp(uint16_t adc_value)
{
    if (adc_value < 5 || adc_value > 4090) {
        return -999.0f;
    }

    float resistance = config.series_resistor * (ADC_MAX - (float)adc_value) / (float)adc_value;

    if (resistance < 10.0f || resistance > 2000000.0f) {
        return -999.0f;
    }

    float steinhart = logf(resistance / config.thermistor_nominal);
    steinhart /= config.b_coefficient;
    steinhart += 1.0f / (config.temp_nominal + 273.15f);
    steinhart = 1.0f / steinhart;
    steinhart -= 273.15f;

    return steinhart;
}

uint16_t Thermistor_GetLastADC(void)
{
    return last_adc_value;
}

uint8_t Thermistor_IsFault(uint16_t adc_value)
{
    if (adc_value < 5 || adc_value > 4090) {
        return 1;
    }

    float temp = Thermistor_ADCToTemp(adc_value);
    if (temp == -999.0f || temp == 350.0f || temp == -50.0f) {
        return 1;
    }

    return 0;
}

Thermistor_Status_t Thermistor_GetLastStatus(void)
{
    return last_status;
}

const char *Thermistor_GetStatusText(Thermistor_Status_t status)
{
    switch (status) {
        case THERMISTOR_STATUS_OK:
            return "OK";
        case THERMISTOR_STATUS_SENSOR_SHORT:
            return "SENSOR_SHORT";
        case THERMISTOR_STATUS_SENSOR_OPEN:
            return "SENSOR_OPEN";
        case THERMISTOR_STATUS_INVALID_RESISTANCE:
            return "INVALID_RESISTANCE";
        case THERMISTOR_STATUS_INVALID_TEMPERATURE:
        default:
            return "INVALID_TEMPERATURE";
    }
}
