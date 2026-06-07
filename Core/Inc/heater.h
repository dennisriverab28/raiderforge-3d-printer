#ifndef HEATER_H
#define HEATER_H

#include "stm32g0xx_hal.h"
#include "thermistor.h"
#include "heater_types.h"
#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════
// GRADUATED FAULT LEVELS (Professor feedback: replace binary flag)
// ═══════════════════════════════════════════════════════════════════════════

typedef enum {
    FAULT_NONE = 0,                // Normal operation
    FAULT_WARNING_TEMP_DRIFT,      // Temperature changing faster than expected
    FAULT_WARNING_ADC_NOISY,       // ADC readings unstable
    FAULT_ERROR_SENSOR_SHORT,      // Thermistor shorted (ADC too low)
    FAULT_ERROR_SENSOR_OPEN,       // Thermistor disconnected (ADC too high)
    FAULT_ERROR_TEMP_RUNAWAY,      // Temperature exceeded safe limits
    FAULT_CRITICAL_SHUTDOWN        // Immediate shutdown required
} Heater_Fault_t;

enum {
    HEATER_DEBUG_FLAG_ENABLED        = (1U << 0),
    HEATER_DEBUG_FLAG_TARGET_WINDOW  = (1U << 1),
    HEATER_DEBUG_FLAG_STEADY_STATE   = (1U << 2),
    HEATER_DEBUG_FLAG_FAULT_ACTIVE   = (1U << 3),
    HEATER_DEBUG_FLAG_PWM_AT_MIN     = (1U << 4),
    HEATER_DEBUG_FLAG_PWM_AT_MAX     = (1U << 5),
    HEATER_DEBUG_FLAG_SENSOR_FAULT   = (1U << 6)
};

// ═══════════════════════════════════════════════════════════════════════════
// HEATER CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════

typedef struct Heater_Config_s {
    float kp;          // Proportional gain
    float ki;          // Integral gain
    float kd;          // Derivative gain
    float min_temp;    // Minimum safe temperature (°C)
    float max_temp;    // Maximum safe temperature (°C)
    uint8_t sensor_fault_debounce;
} Heater_Config_t;

typedef float (*Heater_ReadTempFn)(ADC_HandleTypeDef *adc);
typedef uint16_t (*Heater_GetLastADCFn)(void);
typedef Thermistor_Status_t (*Heater_GetLastStatusFn)(void);



typedef enum {
    HOTEND_FAN_MODE_OFF = 0,
    HOTEND_FAN_MODE_AUTO,
    HOTEND_FAN_MODE_MANUAL
} Hotend_FanMode_t;

typedef enum {
    HOTEND_CMD_SET_TARGET = 0,
    HOTEND_CMD_SET_PART_FAN_PERCENT,
    HOTEND_CMD_ALL_OFF
} Hotend_CommandType_t;



// ═══════════════════════════════════════════════════════════════════════════
// HEATER STATE
// ═══════════════════════════════════════════════════════════════════════════

typedef struct Heater_Diagnostics_s {
    float current_temp;
    float raw_temp;
    float target_temp;
    float error;
    float p_term;
    float i_term;
    float d_term;
    float integral;
    float filtered_derivative;
    float output_percent;
    float dt_s;
    uint16_t pwm_duty;
    uint16_t adc_raw;
    uint8_t enabled;
    Heater_Fault_t fault_level;
    Thermistor_Status_t thermistor_status;
    uint32_t debug_flags;
    float steady_time_s;
} Heater_Diagnostics_t;

typedef struct Heater_s {
    // Hardware handles
    TIM_HandleTypeDef  *pwm_timer;
    uint32_t            pwm_channel;
    ADC_HandleTypeDef  *adc;
    Heater_ReadTempFn   read_temp_fn;
    Heater_GetLastADCFn get_last_adc_fn;
    Heater_GetLastStatusFn get_last_status_fn;

    // Configuration
    Heater_Config_t     config;

    // Current state
    float               current_temp;
    float               raw_temp;
    float               target_temp;
    uint16_t            pwm_duty;       // 0-1000 (0.0% to 100.0%)
    uint8_t             enabled;

    // Graduated fault system
    Heater_Fault_t      fault_level;

    // PID state
    float               integral;
    float               last_error;
    float               last_temp;              // For derivative-on-measurement
    float               filtered_derivative;    // EMA-filtered derivative (was static local)
    float               last_p_term;
    float               last_i_term;
    float               last_d_term;
    float               last_output_percent;
    float               last_dt_s;
    uint16_t            last_adc;
    Thermistor_Status_t thermistor_status;
    uint32_t            debug_flags;
    float               steady_time_s;
    uint32_t            last_update_ms;
    uint8_t             temp_filter_valid;
    uint8_t             sensor_fault_streak;
    uint8_t             warning_fault_streak;

    Heater_State_t 		state;
} Heater_t;

// ═══════════════════════════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════════════════════════

// Initialization
void Heater_Init(Heater_t *heater,
                 TIM_HandleTypeDef *pwm_timer,
                 uint32_t pwm_channel,
                 ADC_HandleTypeDef *adc,
                 Heater_Config_t *config);
void Heater_InitWithSource(Heater_t *heater,
                           TIM_HandleTypeDef *pwm_timer,
                           uint32_t pwm_channel,
                           ADC_HandleTypeDef *adc,
                           Heater_Config_t *config,
                           Heater_ReadTempFn read_temp_fn,
                           Heater_GetLastADCFn get_last_adc_fn,
                           Heater_GetLastStatusFn get_last_status_fn);

// Temperature reading (uses thermistor.c module)
float Heater_ReadTemp(Heater_t *heater);
void Heater_GetTempInt(Heater_t *heater, int *whole, int *frac);
float Heater_GetRawTemp(Heater_t *heater);

// Control
void Heater_Update(Heater_t *heater);              // Call at 10 Hz in main loop
void Heater_SetTarget(Heater_t *heater, float temp_c);
void Heater_Enable(Heater_t *heater, uint8_t enable);

// Status
float Heater_GetTemp(Heater_t *heater);
float Heater_GetTarget(Heater_t *heater);
uint8_t Heater_GetPWM(Heater_t *heater);           // Returns 0-100%
uint16_t Heater_GetRawADC(Heater_t *heater);       // For debugging
uint8_t Heater_IsEnabled(Heater_t *heater);
uint8_t Heater_AtTarget(Heater_t *heater, float tolerance);

// Fault system (graduated)
uint8_t Heater_HasFault(Heater_t *heater);          // 1 if any fault active
Heater_Fault_t Heater_GetFaultLevel(Heater_t *heater);
void Heater_ClearFault(Heater_t *heater);
void Heater_GetDiagnostics(Heater_t *heater, Heater_Diagnostics_t *diag);
uint32_t Heater_GetDebugFlags(Heater_t *heater);
uint8_t Heater_IsSteadyState(Heater_t *heater);

// Manual control (for testing)
void Heater_SetPWM(Heater_t *heater, uint8_t percent);

/* Hotend/nozzle module orchestration */
void Hotend_ModuleInit(Heater_t *heater);
void Hotend_Task(void *pvParameters);
void Hotend_TaskStep(void);
uint8_t Hotend_RequestTarget(float temp_c);
uint8_t Hotend_RequestPartFanPercent(uint8_t percent);
uint8_t Hotend_RequestAllOff(void);
Heater_State_t Hotend_GetState(void);
Hotend_FanMode_t Hotend_GetHotendFanMode(void);
Hotend_FanMode_t Hotend_GetPartFanMode(void);
uint8_t Hotend_Ready(void);
uint8_t Hotend_WaitReady(uint32_t timeout_ms);

#endif // HEATER_H
