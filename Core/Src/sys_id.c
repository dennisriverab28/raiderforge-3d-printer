#include "sys_id.h"
#include "thermistor.h"
#include "uart_debug.h"
#include <math.h>
#include <string.h>

// ═══════════════════════════════════════════════════════════════════════════
// REMOVED: ~60 lines of duplicated ADC reading and temperature conversion
// Now uses shared thermistor.c module (same code as heater.c)
// Benefits: single source of truth, consistent calibration, fewer bugs
// ═══════════════════════════════════════════════════════════════════════════

float SysID_ReadTemp(ADC_HandleTypeDef *adc)
{
    return Thermistor_ReadTemp(adc);
}

static void set_pwm(TIM_HandleTypeDef *timer, uint32_t channel, uint8_t percent)
{
    if (percent > 100) percent = 100;
    uint16_t duty = (uint16_t)percent * 10;
    __HAL_TIM_SET_COMPARE(timer, channel, duty);
}

static void print_temp(float temp)
{
    int whole = (int)temp;
    int frac = (int)((temp - (float)whole) * 10.0f);
    if (frac < 0) frac = -frac;
    UARTDBG_Print("%d.%d", whole, frac);
}

void SysID_StepResponse(TIM_HandleTypeDef *pwm_timer,
                        uint32_t pwm_channel,
                        ADC_HandleTypeDef *adc,
                        SysID_Config_t *config,
                        SysID_Results_t *results)
{
    UARTDBG_Print("\r\n");
    UARTDBG_Print("╔════════════════════════════════════════════════╗\r\n");
    UARTDBG_Print("║  SYSTEM IDENTIFICATION - STEP RESPONSE TEST   ║\r\n");
    UARTDBG_Print("╚════════════════════════════════════════════════╝\r\n");
    UARTDBG_Print("\r\n");
    UARTDBG_Print("  Test Parameters:\r\n");
    UARTDBG_Print("  - Step PWM: %u%%\r\n", config->step_pwm);
    UARTDBG_Print("  - Settle time: %u seconds\r\n", config->settle_time_s);
    UARTDBG_Print("  - Test duration: %u seconds\r\n", config->test_duration_s);
    UARTDBG_Print("  - Sample period: %u ms\r\n", config->sample_period_ms);
    UARTDBG_Print("\r\n");

    HAL_TIM_PWM_Start(pwm_timer, pwm_channel);
    set_pwm(pwm_timer, pwm_channel, 0);

    UARTDBG_Print("Phase 1: Settling at 0%% PWM for %u seconds...\r\n", config->settle_time_s);
    UARTDBG_Print("\r\nCSV Data: Time,Temp,PWM\r\n");

    float settle_temp_sum = 0.0f;
    uint16_t settle_samples = 0;

    for (uint16_t i = 0; i < config->settle_time_s; i++) {
        float temp = SysID_ReadTemp(adc);
        UARTDBG_Print("%u,", i);
        print_temp(temp);
        UARTDBG_Print(",0\r\n");

        if (i >= config->settle_time_s - 10) {
            settle_temp_sum += temp;
            settle_samples++;
        }

        HAL_Delay(1000);
    }

    results->initial_temp = settle_temp_sum / settle_samples;

    UARTDBG_Print("\r\n");
    UARTDBG_Print("Phase 2: STEP INPUT APPLIED - %u%% PWM\r\n", config->step_pwm);
    UARTDBG_Print("\r\n");

    set_pwm(pwm_timer, pwm_channel, config->step_pwm);

    #define MAX_SAMPLES 600
    float temp_data[MAX_SAMPLES];
    uint16_t sample_count = 0;

    uint16_t total_samples = (config->test_duration_s * 1000) / config->sample_period_ms;
    if (total_samples > MAX_SAMPLES) total_samples = MAX_SAMPLES;

    for (uint16_t i = 0; i < total_samples; i++) {
        float temp = SysID_ReadTemp(adc);

        uint32_t time_ms = config->settle_time_s * 1000 + i * config->sample_period_ms;

        UARTDBG_Print("%lu,", time_ms / 1000);
        print_temp(temp);
        UARTDBG_Print(",%u\r\n", config->step_pwm);

        temp_data[sample_count++] = temp;

        HAL_Delay(config->sample_period_ms);
    }

    results->final_temp = temp_data[sample_count - 1];
    results->step_pwm = config->step_pwm;

    results->gain = (results->final_temp - results->initial_temp) / (float)config->step_pwm;

    results->time_constant = SysID_CalculateTimeConstant(
        results->initial_temp,
        results->final_temp,
        temp_data,
        sample_count,
        config->sample_period_ms
    );

    set_pwm(pwm_timer, pwm_channel, 0);

    UARTDBG_Print("\r\n");
    UARTDBG_Print("╔════════════════════════════════════════════════╗\r\n");
    UARTDBG_Print("║  TEST COMPLETE - RESULTS                       ║\r\n");
    UARTDBG_Print("╚════════════════════════════════════════════════╝\r\n");
    UARTDBG_Print("\r\n");
    UARTDBG_Print("  Initial Temperature: ");
    print_temp(results->initial_temp);
    UARTDBG_Print(" °C\r\n");
    UARTDBG_Print("  Final Temperature:   ");
    print_temp(results->final_temp);
    UARTDBG_Print(" °C\r\n");
    UARTDBG_Print("  Step PWM:            %u%%\r\n", results->step_pwm);
    UARTDBG_Print("  Gain (K):            ");
    print_temp(results->gain);
    UARTDBG_Print(" °C/%%PWM\r\n");
    UARTDBG_Print("  Time Constant (τ):   ");
    print_temp(results->time_constant);
    UARTDBG_Print(" seconds\r\n");
    UARTDBG_Print("\r\n");

    SysID_PrintTransferFunction(results);
}

float SysID_CalculateTimeConstant(float initial_temp,
                                   float final_temp,
                                   float *temp_array,
                                   uint16_t array_size,
                                   uint16_t sample_period_ms)
{
    float delta_temp = final_temp - initial_temp;
    float target_temp = initial_temp + 0.632f * delta_temp;

    for (uint16_t i = 0; i < array_size; i++) {
        if (temp_array[i] >= target_temp) {
            return (float)(i * sample_period_ms) / 1000.0f;
        }
    }

    return 0.0f;
}

void SysID_PrintTransferFunction(SysID_Results_t *results)
{
    UARTDBG_Print("╔════════════════════════════════════════════════╗\r\n");
    UARTDBG_Print("║  TRANSFER FUNCTION G(s)                        ║\r\n");
    UARTDBG_Print("╚════════════════════════════════════════════════╝\r\n");
    UARTDBG_Print("\r\n");
    UARTDBG_Print("         K\r\n");
    UARTDBG_Print("  G(s) = ───────\r\n");
    UARTDBG_Print("         τs + 1\r\n");
    UARTDBG_Print("\r\n");
    UARTDBG_Print("  Where:\r\n");
    UARTDBG_Print("    K = ");
    print_temp(results->gain);
    UARTDBG_Print(" °C/%%PWM\r\n");
    UARTDBG_Print("    τ = ");
    print_temp(results->time_constant);
    UARTDBG_Print(" seconds\r\n");
    UARTDBG_Print("\r\n");
    UARTDBG_Print("  Final Form:\r\n");
    UARTDBG_Print("         ");
    print_temp(results->gain);
    UARTDBG_Print("\r\n");
    UARTDBG_Print("  G(s) = ─────────────\r\n");
    UARTDBG_Print("         ");
    print_temp(results->time_constant);
    UARTDBG_Print("s + 1\r\n");
    UARTDBG_Print("\r\n");
}

void SysID_MultiStep(TIM_HandleTypeDef *pwm_timer,
                     uint32_t pwm_channel,
                     ADC_HandleTypeDef *adc,
                     uint8_t *pwm_levels,
                     uint8_t num_levels)
{
    UARTDBG_Print("\r\n");
    UARTDBG_Print("╔════════════════════════════════════════════════╗\r\n");
    UARTDBG_Print("║  MULTI-STEP TEST - PWM vs STEADY-STATE TEMP   ║\r\n");
    UARTDBG_Print("╚════════════════════════════════════════════════╝\r\n");
    UARTDBG_Print("\r\n");
    UARTDBG_Print("Testing %u PWM levels\r\n", num_levels);
    UARTDBG_Print("\r\nCSV Summary: PWM,SteadyStateTemp\r\n");

    SysID_Config_t config = {
        .settle_time_s = 30,
        .test_duration_s = 120,
        .sample_period_ms = 1000
    };

    SysID_Results_t results;

    for (uint8_t i = 0; i < num_levels; i++) {
        config.step_pwm = pwm_levels[i];

        UARTDBG_Print("\r\n");
        UARTDBG_Print("════════════════════════════════════════════════\r\n");
        UARTDBG_Print("  Test %u of %u: %u%% PWM\r\n", i + 1, num_levels, pwm_levels[i]);
        UARTDBG_Print("════════════════════════════════════════════════\r\n");

        SysID_StepResponse(pwm_timer, pwm_channel, adc, &config, &results);

        UARTDBG_Print("\r\nResult: %u%%,", results.step_pwm);
        print_temp(results.final_temp);
        UARTDBG_Print("°C\r\n");

        if (i < num_levels - 1) {
            UARTDBG_Print("\r\n⏳ Cooling down for 60 seconds before next test...\r\n");

            for (int j = 60; j > 0; j -= 10) {
                UARTDBG_Print("   %d seconds remaining...\r\n", j);
                HAL_Delay(10000);
            }
        }
    }

    UARTDBG_Print("\r\n");
    UARTDBG_Print("╔════════════════════════════════════════════════╗\r\n");
    UARTDBG_Print("║  ALL TESTS COMPLETE                            ║\r\n");
    UARTDBG_Print("╚════════════════════════════════════════════════╝\r\n");
}
