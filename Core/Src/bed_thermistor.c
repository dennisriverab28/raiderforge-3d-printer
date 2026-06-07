/*
 * bed_thermistor.c
 * PB10 → ADC1_IN11 | hadc1 shared with hotend — channel switched per read
 * Created: Feb 2026 | Author: Cherryman125
 */

#include "bed_thermistor.h"
#include <math.h>
#include "qbc_globals.h"

// TODO: Measure actual series resistor with DMM and update series_resistor
const BedTherm_Config_t BED_THERMISTOR_DEFAULT_CONFIG = {
    .thermistor_nominal    = 100000.0f,
    .temp_nominal          = 25.0f,
    .b_coefficient         = 3950.0f,
    .series_resistor       = 100000.0f,
    .temp_offset_low       = 0.0f,      // TODO: calibrate at room temp
    .temp_offset_high      = 0.0f,      // TODO: calibrate at 90°C with IR gun
    .calibration_crossover = 60.0f
};

static BedTherm_Config_t bed_config;
static uint16_t          bed_last_adc = 0;
static Thermistor_Status_t bed_last_status = THERMISTOR_STATUS_INVALID_RESISTANCE;
static uint16_t          bed_filtered_adc = 0;
static uint8_t           bed_filter_valid = 0U;

#define BED_ADC_SAMPLES          16
#define BED_ADC_DISCARD_EXTREMES  3
#define BED_ADC_MAX            4095.0f
#define BED_ADC_CHANNEL        ADC_CHANNEL_11
#define BED_ADC_JUMP_LIMIT       160U
#define BED_ADC_EMA_ALPHA_NUM      1U
#define BED_ADC_EMA_ALPHA_DEN      4U

static float bed_therm_convert_adc_to_temp(uint16_t adc_value, Thermistor_Status_t *status_out)
{
    Thermistor_Status_t status = THERMISTOR_STATUS_OK;
    float steinhart = NAN;

    if (adc_value < 5U) {
        status = THERMISTOR_STATUS_SENSOR_OPEN;
        goto done;
    }
    if (adc_value > 4090U) {
        status = THERMISTOR_STATUS_SENSOR_SHORT;
        goto done;
    }

    {
        float resistance = bed_config.series_resistor
                         * (BED_ADC_MAX - (float)adc_value) / (float)adc_value;

        if (resistance < 10.0f || resistance > 2000000.0f) {
            status = THERMISTOR_STATUS_INVALID_RESISTANCE;
            goto done;
        }

        steinhart = logf(resistance / bed_config.thermistor_nominal);
        steinhart /= bed_config.b_coefficient;
        steinhart += 1.0f / (bed_config.temp_nominal + 273.15f);
        steinhart  = 1.0f / steinhart;
        steinhart -= 273.15f;

        if (steinhart < bed_config.calibration_crossover) {
            steinhart += bed_config.temp_offset_low;
        } else {
            float blend = (steinhart - bed_config.calibration_crossover)
                        / (110.0f - bed_config.calibration_crossover);
            if (blend > 1.0f) blend = 1.0f;
            steinhart += bed_config.temp_offset_low  * (1.0f - blend)
                       + bed_config.temp_offset_high * blend;
        }

        if (steinhart < -50.0f || steinhart > 150.0f) {
            status = THERMISTOR_STATUS_INVALID_TEMPERATURE;
            steinhart = NAN;
            goto done;
        }
    }

done:
    bed_last_status = status;
    if (status_out != NULL) {
        *status_out = status;
    }
    return steinhart;
}

static void switch_to_bed_channel(ADC_HandleTypeDef *adc)
{
    ADC_ChannelConfTypeDef ch = {0};

    HAL_ADC_Stop(adc);

    ch.Channel      = BED_ADC_CHANNEL;
    ch.Rank         = ADC_REGULAR_RANK_1;
    /* Bed thermistor source impedance is high enough that the shortest sample
       time can produce occasional bogus low counts after a channel switch. */
    ch.SamplingTime = ADC_SAMPLINGTIME_COMMON_2;
    HAL_ADC_ConfigChannel(adc, &ch);
}

void BedTherm_Init(const BedTherm_Config_t *user_config)
{
    bed_config   = (user_config != NULL) ? *user_config : BED_THERMISTOR_DEFAULT_CONFIG;
    bed_last_adc = 0;
    bed_last_status = THERMISTOR_STATUS_INVALID_RESISTANCE;
    bed_filtered_adc = 0;
    bed_filter_valid = 0U;
}

uint16_t BedTherm_ReadADC(ADC_HandleTypeDef *adc)
{
    uint16_t samples[BED_ADC_SAMPLES];
    uint16_t median;
    uint32_t sum = 0U;
    uint16_t candidate;
    uint16_t delta;
    int count = 0;

    if (adc == NULL) return 0;

    if (xHeaterUpdateMutex != NULL &&
        xSemaphoreTake(xHeaterUpdateMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return bed_last_adc;
    }

    switch_to_bed_channel(adc);

    /* Throw away the first conversion after switching channels so the ADC
       sampling capacitor can settle to the bed thermistor network. */
    HAL_ADC_Start(adc);
    (void)HAL_ADC_PollForConversion(adc, 100);
    (void)HAL_ADC_GetValue(adc);

    for (int i = 0; i < BED_ADC_SAMPLES; i++) {
        HAL_ADC_Start(adc);
        samples[i] = (HAL_ADC_PollForConversion(adc, 100) == HAL_OK)
                     ? HAL_ADC_GetValue(adc) : 0;
    }

    for (int i = 0; i < BED_ADC_SAMPLES - 1; i++)
        for (int j = 0; j < BED_ADC_SAMPLES - i - 1; j++)
            if (samples[j] > samples[j + 1]) {
                uint16_t tmp = samples[j];
                samples[j]   = samples[j + 1];
                samples[j + 1] = tmp;
            }

    median = samples[BED_ADC_SAMPLES / 2];

    for (int i = BED_ADC_DISCARD_EXTREMES; i < BED_ADC_SAMPLES - BED_ADC_DISCARD_EXTREMES; i++) {
        /* Reject values still far from the sample-set median. This catches
           transient ADC glitches without touching the hotend path. */
        delta = (samples[i] > median) ? (samples[i] - median) : (median - samples[i]);
        if (delta > BED_ADC_JUMP_LIMIT) {
            continue;
        }
        sum += samples[i];
        count++;
    }

    candidate = (count > 0) ? (uint16_t)((sum + (uint32_t)(count / 2)) / (uint32_t)count) : median;

    if (bed_filter_valid != 0U) {
        delta = (candidate > bed_filtered_adc) ? (candidate - bed_filtered_adc) : (bed_filtered_adc - candidate);
        if (delta > BED_ADC_JUMP_LIMIT) {
            /* Single large excursions are clamped to the previous filtered
               value and must repeat in later reads to move the estimate. */
            candidate = bed_filtered_adc;
        } else {
            candidate = (uint16_t)(((uint32_t)bed_filtered_adc * (BED_ADC_EMA_ALPHA_DEN - BED_ADC_EMA_ALPHA_NUM)) +
                                   ((uint32_t)candidate * BED_ADC_EMA_ALPHA_NUM) +
                                   (BED_ADC_EMA_ALPHA_DEN / 2U)) / BED_ADC_EMA_ALPHA_DEN;
        }
    }

    bed_filtered_adc = candidate;
    bed_filter_valid = 1U;
    bed_last_adc = candidate;

    if (xHeaterUpdateMutex != NULL) {
        xSemaphoreGive(xHeaterUpdateMutex);
    }

    return candidate;
}

uint16_t BedTherm_ReadADC_Single(ADC_HandleTypeDef *adc)
{
    if (adc == NULL) return 0;

    if (xHeaterUpdateMutex != NULL &&
        xSemaphoreTake(xHeaterUpdateMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return bed_last_adc;
    }

    switch_to_bed_channel(adc);

    HAL_ADC_Start(adc);
    (void)HAL_ADC_PollForConversion(adc, 100);
    (void)HAL_ADC_GetValue(adc);

    HAL_ADC_Start(adc);
    uint16_t val = (HAL_ADC_PollForConversion(adc, 100) == HAL_OK)
                   ? HAL_ADC_GetValue(adc) : 0;

    if (xHeaterUpdateMutex != NULL) {
        xSemaphoreGive(xHeaterUpdateMutex);
    }

    bed_last_adc = val;
    return val;
}

float BedTherm_ADCToTemp(uint16_t adc_value)
{
    return bed_therm_convert_adc_to_temp(adc_value, NULL);
}

float BedTherm_ReadTemp(ADC_HandleTypeDef *adc)
{
    return BedTherm_ADCToTemp(BedTherm_ReadADC(adc));
}

float BedTherm_GetRawTemp(uint16_t adc_value)
{
    if (adc_value < 5U || adc_value > 4090U) return NAN;

    float resistance = bed_config.series_resistor
                       * (BED_ADC_MAX - (float)adc_value) / (float)adc_value;

    if (resistance < 10.0f || resistance > 2000000.0f) return NAN;

    float steinhart = logf(resistance / bed_config.thermistor_nominal);
    steinhart /= bed_config.b_coefficient;
    steinhart += 1.0f / (bed_config.temp_nominal + 273.15f);
    steinhart  = 1.0f / steinhart;
    steinhart -= 273.15f;

    return steinhart;
}

uint16_t BedTherm_GetLastADC(void) { return bed_last_adc; }

uint8_t BedTherm_IsFault(uint16_t adc_value)
{
    Thermistor_Status_t status = THERMISTOR_STATUS_OK;
    (void)bed_therm_convert_adc_to_temp(adc_value, &status);
    return (status == THERMISTOR_STATUS_OK) ? 0U : 1U;
}

Thermistor_Status_t BedTherm_GetLastStatus(void)
{
    return bed_last_status;
}

const char *BedTherm_GetStatusText(Thermistor_Status_t status)
{
    return Thermistor_GetStatusText(status);
}
