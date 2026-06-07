#include "heater.h"
#include "ddr_globals.h"
#include "qbc_globals.h"
#include "fan.h"
#include "FreeRTOS.h"
#include "task.h"
#include "thermistor.h"
#include "uart_debug.h"
#include <math.h>
#include <heater_types.h>

#define HOTEND_KP                    8.0f
#define HOTEND_KI                    0.3f
#define HOTEND_KD                   50.0f
#define HOTEND_MAX_TEMP            280.0f
#define HOTEND_MIN_TEMP            -10.0f
#define HOTEND_PWM_TIMER            htim16
#define HOTEND_PWM_CHANNEL          TIM_CHANNEL_1
#define HOTEND_ADC_HANDLE           hadc1

extern TIM_HandleTypeDef htim16;
extern ADC_HandleTypeDef hadc1;

#define HEATER_STEADY_ERROR_C         1.5f
#define HEATER_STEADY_SLOPE_C_PER_S   0.20f
#define HEATER_STEADY_MIN_TIME_S     15.0f
#define HEATER_TEMP_FILTER_ALPHA      0.15f
#define HEATER_SPIKE_REJECT_C        15.0f
#define HEATER_SPIKE_CONFIRM_C        5.0f
#define HEATER_WARN_RATE_C_PER_S     80.0f
#define HEATER_WARNING_DEBOUNCE       3U
#define HOTEND_READY_TOLERANCE_C      2.0f
#define HOTEND_COOLING_MIN_TEMP_C    40.0f
#define HOTEND_FAN_ON_TEMP_C         50.0f
#define HOTEND_FAN_OFF_TEMP_C        45.0f
#define HEATER_DEBUG_PERIOD_MS       5000U

//static Heater_State_t hotend_state = HEATER_STATE_INIT;
//static Hotend_FanMode_t hotend_fan_mode = HOTEND_FAN_MODE_OFF;
//static Hotend_FanMode_t part_fan_mode = HOTEND_FAN_MODE_OFF;

static void hotend_apply_command(Heater_t *hotend, const Hotend_Command_t *cmd);
static void hotend_set_state(Heater_t *heater,Heater_State_t new_state);
static void hotend_clear_ready_sem(void);
static Heater_State_t hotend_get_state(Heater_t *hotend);
static void hotend_handle_init_state(Heater_t *hotend);
static void hotend_handle_off_state(Heater_t *hotend);
static void hotend_handle_heating_state(Heater_t *hotend, const Hotend_Command_t *cmd);
static void hotend_handle_holding_state(Heater_t *hotend);
static void hotend_handle_cooling_state(Heater_t *hotend);
static void hotend_handle_fault_state(Heater_t *hotend);
static const char *hotend_state_name(Heater_State_t state);
static void hotend_debug_log(Heater_t *hotend);
static const char *heater_fault_name(Heater_Fault_t fault);
static uint8_t heater_has_hard_fault(Heater_t *heater);

static Heater_t hotend_heater;
// ═══════════════════════════════════════════════════════════════════════════
// PRIVATE HELPERS
// ═══════════════════════════════════════════════════════════════════════════

static void set_pwm(Heater_t *heater, uint16_t duty)
{
    uint32_t timer_period;
    uint32_t compare;

    if (heater == NULL) return;
    if (duty > 1000) duty = 1000;
    heater->pwm_duty = duty;

    timer_period = __HAL_TIM_GET_AUTORELOAD(heater->pwm_timer);
    compare = ((uint32_t)duty * (timer_period + 1U)) / 1000U;
    if (compare > timer_period) compare = timer_period;

    __HAL_TIM_SET_COMPARE(heater->pwm_timer, heater->pwm_channel, compare);
}

// ═══════════════════════════════════════════════════════════════════════════
// FAULT DETECTION (graduated levels)
// Professor feedback: "kinda ass approach" on binary flag → 7-level system
// ═══════════════════════════════════════════════════════════════════════════

static Heater_Fault_t check_faults(Heater_t *heater, float temp, float dt_s)
{
    Thermistor_Status_t status;

    if (heater == NULL) return FAULT_CRITICAL_SHUTDOWN;

    status = heater->thermistor_status;

    if (status == THERMISTOR_STATUS_SENSOR_SHORT) {
        return FAULT_ERROR_SENSOR_SHORT;
    }
    if (status == THERMISTOR_STATUS_SENSOR_OPEN) {
        return FAULT_ERROR_SENSOR_OPEN;
    }
    if (status != THERMISTOR_STATUS_OK || !isfinite(temp)) {
        return FAULT_CRITICAL_SHUTDOWN;
    }

    // ERROR: Temperature outside safe operating range
    if (temp < heater->config.min_temp || temp > heater->config.max_temp) {
        return FAULT_ERROR_TEMP_RUNAWAY;
    }

    // ERROR: Thermal runaway - heater disabled but temp dangerously high
    if (!heater->enabled && temp > (heater->config.max_temp - 20.0f)) {
        return FAULT_ERROR_TEMP_RUNAWAY;
    }

    // WARNING: Temperature changing faster than this hotend can physically move.
    if (heater->temp_filter_valid && heater->last_temp != 0.0f && dt_s > 0.001f) {
        float temp_rate = fabsf(temp - heater->last_temp) / dt_s;
        if (temp_rate > HEATER_WARN_RATE_C_PER_S) {
            return FAULT_WARNING_TEMP_DRIFT;
        }
    }

    if (!isfinite(heater->raw_temp)) {
        return FAULT_WARNING_TEMP_DRIFT;
    }

    return FAULT_NONE;
}

static float read_filtered_temp(Heater_t *heater)
{
    float raw_temp;
    float retry_temp;

    if (heater == NULL) return NAN;

    if (heater->read_temp_fn == NULL ||
        heater->get_last_adc_fn == NULL ||
        heater->get_last_status_fn == NULL) {
        return NAN;
    }

    raw_temp = heater->read_temp_fn(heater->adc);
    heater->last_adc = heater->get_last_adc_fn();
    heater->thermistor_status = heater->get_last_status_fn();

    if (heater->thermistor_status == THERMISTOR_STATUS_OK &&
        isfinite(raw_temp) &&
        heater->temp_filter_valid &&
        fabsf(raw_temp - heater->current_temp) > HEATER_SPIKE_REJECT_C) {
        retry_temp = heater->read_temp_fn(heater->adc);

        if (heater->get_last_status_fn() == THERMISTOR_STATUS_OK && isfinite(retry_temp)) {
            heater->last_adc = heater->get_last_adc_fn();
            heater->thermistor_status = heater->get_last_status_fn();

            if (fabsf(retry_temp - heater->current_temp) <= HEATER_SPIKE_REJECT_C) {
                raw_temp = retry_temp;
            } else if (fabsf(raw_temp - retry_temp) <= HEATER_SPIKE_CONFIRM_C) {
                raw_temp = 0.5f * (raw_temp + retry_temp);
            } else {
                raw_temp = heater->current_temp;
            }
        } else {
            raw_temp = heater->current_temp;
        }
    }

    heater->raw_temp = raw_temp;

    if (!isfinite(raw_temp) || heater->thermistor_status != THERMISTOR_STATUS_OK) {
        heater->current_temp = raw_temp;
        return raw_temp;
    }

    if (!heater->temp_filter_valid || !isfinite(heater->current_temp)) {
        heater->current_temp = raw_temp;
        heater->temp_filter_valid = 1U;
    } else {
        heater->current_temp = ((1.0f - HEATER_TEMP_FILTER_ALPHA) * heater->current_temp) +
                               (HEATER_TEMP_FILTER_ALPHA * raw_temp);
    }

    return heater->current_temp;
}

static void update_debug_state(Heater_t *heater)
{
    float abs_error;

    if (heater == NULL) return;

    heater->debug_flags = 0U;

    if (heater->enabled) {
        heater->debug_flags |= HEATER_DEBUG_FLAG_ENABLED;
    }
    if (heater->fault_level != FAULT_NONE) {
        heater->debug_flags |= HEATER_DEBUG_FLAG_FAULT_ACTIVE;
    }
    if (heater->thermistor_status != THERMISTOR_STATUS_OK) {
        heater->debug_flags |= HEATER_DEBUG_FLAG_SENSOR_FAULT;
    }
    if (heater->last_output_percent <= 0.1f) {
        heater->debug_flags |= HEATER_DEBUG_FLAG_PWM_AT_MIN;
    }
    if (heater->last_output_percent >= 99.9f) {
        heater->debug_flags |= HEATER_DEBUG_FLAG_PWM_AT_MAX;
    }

    abs_error = fabsf(heater->target_temp - heater->current_temp);
    if (heater->enabled && heater->target_temp > 0.0f && abs_error <= HEATER_STEADY_ERROR_C) {
        heater->debug_flags |= HEATER_DEBUG_FLAG_TARGET_WINDOW;
    }

    if ((heater->debug_flags & HEATER_DEBUG_FLAG_TARGET_WINDOW) != 0U &&
        fabsf(heater->filtered_derivative) <= HEATER_STEADY_SLOPE_C_PER_S) {
        heater->steady_time_s += heater->last_dt_s;
    } else {
        heater->steady_time_s = 0.0f;
    }

    if (heater->steady_time_s >= HEATER_STEADY_MIN_TIME_S) {
        heater->debug_flags |= HEATER_DEBUG_FLAG_STEADY_STATE;
    }
}

//static Heater_State_t hotend_eval_state(Heater_t *hotend)
//{
//    float temp = Heater_GetTemp(hotend);
//    float target = Heater_GetTarget(hotend);
//
//    if (Heater_HasFault(hotend)) return HEATER_STATE_FAULT;
//
//    if (Heater_IsEnabled(hotend) &&
//        target > 0.0f &&
//        temp < (target - HOTEND_READY_TOLERANCE_C)) {
//        return HEATER_STATE_HEATING;
//    }
//    if (target > 0.0f &&
//        (Heater_AtTarget(hotend, HOTEND_READY_TOLERANCE_C) ||
//         Heater_IsSteadyState(hotend))) {
//        return HEATER_STATE_HOLD;
//    }
//    if (temp > HOTEND_COOLING_MIN_TEMP_C &&
//        temp > (target + HOTEND_READY_TOLERANCE_C)) {
//        return HEATER_STATE_COOLING;
//    }
//    if (!Heater_IsEnabled(hotend) || target <= 0.0f) {
//        return HEATER_STATE_OFF;
//    }
//    return HEATER_STATE_COOLING;
//}
//
static void hotend_set_ready_sem()
{
    if (xHotendReadySem == NULL) return;
    (void)xSemaphoreGive(xHotendReadySem);
}

static void hotend_clear_ready_sem(void)
{
    if (xHotendReadySem == NULL) return;
    while (xSemaphoreTake(xHotendReadySem, 0U) == pdTRUE) {
    }
}

//static void hotend_refresh_state(void)
//{
//    hotend_set_state(hotend_eval_state());
//    hotend_set_ready_sem((hotend_get_state() == HEATER_RUN_STATE_HOLDING) ? 1U : 0U);
//}

static void hotend_set_state(Heater_t *heater, Heater_State_t new_state)
{
	if(xSemaphoreTake(xHeaterStateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
	{
	    heater->state = new_state;
	    xSemaphoreGive(xHeaterStateMutex);
	}
}

static Heater_State_t hotend_get_state(Heater_t *heater)
{
	if(xSemaphoreTake(xHeaterStateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
	{
	    xSemaphoreGive(xHeaterStateMutex);
	    return heater->state;
	}

	else return HEATER_STATE_FAULT;
}

static void hotend_update_outputs(Heater_t *hotend)
{
    float hotend_temp;

    Heater_Update(hotend);

    hotend_temp = Heater_GetTemp(hotend);
    if (hotend_temp >= HOTEND_FAN_ON_TEMP_C) {
        if (!Fan_IsRunning(&DDRlo.hotend_fan)) {
            Fan_On(&DDRlo.hotend_fan);
        }
    } else if (hotend_temp < HOTEND_FAN_OFF_TEMP_C &&
               Heater_IsEnabled(hotend) == 0U &&
               Fan_IsRunning(&DDRlo.hotend_fan)) {
        Fan_Off(&DDRlo.hotend_fan);
    }
}

static void hotend_handle_off_state(Heater_t *hotend)
{
    hotend_clear_ready_sem();

    if (heater_has_hard_fault(hotend)) {
        hotend_set_state(hotend, HEATER_STATE_FAULT);
        return;
    }

    if (Heater_IsEnabled(hotend) &&
        Heater_GetTarget(hotend) > 0.0f) {
        hotend_set_state(hotend, HEATER_STATE_HEATING);
    }

    hotend_update_outputs(hotend);
}

static void hotend_handle_init_state(Heater_t *hotend)
{
    hotend_clear_ready_sem();

    if (heater_has_hard_fault(hotend)) {
        hotend_set_state(hotend, HEATER_STATE_FAULT);
        return;
    }

    if (Heater_IsEnabled(hotend) &&
        Heater_GetTarget(hotend) > 0.0f) {
        hotend_set_state(hotend, HEATER_STATE_HEATING);
    } else {
        hotend_set_state(hotend, HEATER_STATE_OFF);
    }

    hotend_update_outputs(hotend);
}

static void hotend_handle_heating_state(Heater_t *hotend, const Hotend_Command_t *cmd)
{
    (void)cmd;
    hotend_clear_ready_sem();

    if (heater_has_hard_fault(hotend)) {
        hotend_set_state(hotend, HEATER_STATE_FAULT);
        return;
    }

    if (!Heater_IsEnabled(hotend) ||
        Heater_GetTarget(hotend) <= 0.0f) {
        hotend_set_state(hotend, HEATER_STATE_OFF);
    }

    hotend_update_outputs(hotend);

    if (Heater_AtTarget(hotend, HOTEND_READY_TOLERANCE_C)) {
        hotend_set_state(hotend, HEATER_STATE_HOLD);
    }
}

static void hotend_handle_holding_state(Heater_t *hotend)
{
    if (heater_has_hard_fault(hotend)) {
        hotend_set_state(hotend, HEATER_STATE_FAULT);
        hotend_clear_ready_sem();
        return;
    }

    if (!Heater_IsEnabled(hotend) ||
        Heater_GetTarget(hotend) <= 0.0f) {
        hotend_set_state(hotend, HEATER_STATE_OFF);
        hotend_clear_ready_sem();
    } else {
        hotend_set_ready_sem();
    }

    hotend_update_outputs(hotend);

    if (hotend_get_state(hotend) != HEATER_STATE_HOLD) {
        hotend_clear_ready_sem();
    }
}

static void hotend_handle_cooling_state(Heater_t *hotend)
{
    hotend_clear_ready_sem();

    if (heater_has_hard_fault(hotend)) {
        hotend_set_state(hotend, HEATER_STATE_FAULT);
        return;
    }

    hotend_update_outputs(hotend);
}

static void hotend_handle_fault_state(Heater_t *hotend)
{
    if (!heater_has_hard_fault(hotend)) {
        if (Heater_IsEnabled(hotend) && Heater_GetTarget(hotend) > 0.0f) {
            hotend_set_state(hotend, HEATER_STATE_HEATING);
        } else {
            hotend_set_state(hotend, HEATER_STATE_OFF);
        }
        return;
    }

    hotend_clear_ready_sem();
    Heater_Enable(hotend, 0U);

    hotend_update_outputs(hotend);
}

static const char *hotend_state_name(Heater_State_t state)
{
    switch (state) {
    case HEATER_STATE_INIT: return "INIT";
    case HEATER_STATE_IDLE: return "IDLE";
    case HEATER_STATE_OFF: return "OFF";
    case HEATER_STATE_HEATING: return "HEATING";
    case HEATER_STATE_HOLD: return "HOLD";
    case HEATER_STATE_COOLING: return "COOLING";
    case HEATER_STATE_FAULT: return "FAULT";
    default: return "?";
    }
}

static const char *heater_fault_name(Heater_Fault_t fault)
{
    switch (fault) {
    case FAULT_NONE: return "NONE";
    case FAULT_WARNING_TEMP_DRIFT: return "TEMP_DRIFT";
    case FAULT_WARNING_ADC_NOISY: return "ADC_NOISY";
    case FAULT_ERROR_SENSOR_SHORT: return "SENSOR_SHORT";
    case FAULT_ERROR_SENSOR_OPEN: return "SENSOR_OPEN";
    case FAULT_ERROR_TEMP_RUNAWAY: return "TEMP_RUNAWAY";
    case FAULT_CRITICAL_SHUTDOWN: return "CRITICAL";
    default: return "?";
    }
}

static uint8_t heater_has_hard_fault(Heater_t *heater)
{
    Heater_Fault_t fault;

    if (heater == NULL) {
        return 1U;
    }

    fault = Heater_GetFaultLevel(heater);
    return (fault >= FAULT_ERROR_SENSOR_SHORT) ? 1U : 0U;
}

static void hotend_debug_log(Heater_t *hotend)
{
    static uint32_t next_log_ms = 0U;
    uint32_t now;

    if (hotend == NULL) {
        return;
    }

    if (!Heater_IsEnabled(hotend) &&
        Heater_GetTarget(hotend) <= 0.0f &&
        !Heater_HasFault(hotend)) {
        next_log_ms = 0U;
        return;
    }

    now = HAL_GetTick();
    if (next_log_ms != 0U && now < next_log_ms) {
        return;
    }

    UARTDBG_Print("[hotend] %s T=%.1f/%.1f PWM=%u%% ADC=%u R=%.0fohm fault=%s status=%s\r\n",
                  hotend_state_name(hotend_get_state(hotend)),
                  Heater_GetTemp(hotend),
                  Heater_GetTarget(hotend),
                  Heater_GetPWM(hotend),
                  Heater_GetRawADC(hotend),
                  Thermistor_ADCToResistance(Heater_GetRawADC(hotend)),
                  heater_fault_name(Heater_GetFaultLevel(hotend)),
                  Thermistor_GetStatusText(hotend->thermistor_status));
    next_log_ms = now + HEATER_DEBUG_PERIOD_MS;
}

// ═══════════════════════════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════

void Heater_Init(Heater_t *heater,
                 TIM_HandleTypeDef *pwm_timer,
                 uint32_t pwm_channel,
                 ADC_HandleTypeDef *adc,
                 Heater_Config_t *config)
{
    Heater_InitWithSource(heater,
                          pwm_timer,
                          pwm_channel,
                          adc,
                          config,
                          Thermistor_ReadTemp,
                          Thermistor_GetLastADC,
                          Thermistor_GetLastStatus);
}

void Heater_InitWithSource(Heater_t *heater,
                           TIM_HandleTypeDef *pwm_timer,
                           uint32_t pwm_channel,
                           ADC_HandleTypeDef *adc,
                           Heater_Config_t *config,
                           Heater_ReadTempFn read_temp_fn,
                           Heater_GetLastADCFn get_last_adc_fn,
                           Heater_GetLastStatusFn get_last_status_fn)
{
    if (heater == NULL || pwm_timer == NULL || adc == NULL || config == NULL) return;

    heater->pwm_timer = pwm_timer;
    heater->pwm_channel = pwm_channel;
    heater->adc = adc;
    heater->read_temp_fn = read_temp_fn;
    heater->get_last_adc_fn = get_last_adc_fn;
    heater->get_last_status_fn = get_last_status_fn;
    heater->config = *config;

    heater->current_temp = 0.0f;
    heater->raw_temp = 0.0f;
    heater->target_temp = 0.0f;
    heater->pwm_duty = 0;
    heater->enabled = 0;
    heater->fault_level = FAULT_NONE;

    heater->integral = 0.0f;
    heater->last_error = 0.0f;
    heater->last_temp = 0.0f;
    heater->filtered_derivative = 0.0f;
    heater->last_p_term = 0.0f;
    heater->last_i_term = 0.0f;
    heater->last_d_term = 0.0f;
    heater->last_output_percent = 0.0f;
    heater->last_dt_s = 0.0f;
    heater->last_adc = 0;
    heater->thermistor_status = THERMISTOR_STATUS_INVALID_RESISTANCE;
    heater->debug_flags = HEATER_DEBUG_FLAG_PWM_AT_MIN;
    heater->steady_time_s = 0.0f;
    heater->last_update_ms = HAL_GetTick();
    heater->temp_filter_valid = 0U;
    heater->sensor_fault_streak = 0U;
    heater->warning_fault_streak = 0U;

    HAL_TIM_PWM_Start(heater->pwm_timer, heater->pwm_channel);
    set_pwm(heater, 0);

    // Take initial reading (thermistor module must be initialized first)
    Heater_ReadTemp(heater);
    heater->last_temp = heater->current_temp;
}

// ═══════════════════════════════════════════════════════════════════════════
// TEMPERATURE READING (delegates to thermistor.c module)
// Eliminates ~100 lines of duplicated ADC/Steinhart-Hart code
// Now uses two-point calibration, proper rounding, etc.
// ═══════════════════════════════════════════════════════════════════════════

float Heater_ReadTemp(Heater_t *heater)
{
    if (heater == NULL) return NAN;
    return read_filtered_temp(heater);
}

void Heater_GetTempInt(Heater_t *heater, int *whole, int *frac)
{
    if (heater == NULL || whole == NULL || frac == NULL) return;

    float temp = heater->current_temp;
    *whole = (int)temp;
    float remainder = temp - (float)(*whole);
    if (remainder < 0) remainder = -remainder;
    *frac = (int)(remainder * 10.0f);
}

float Heater_GetRawTemp(Heater_t *heater)
{
    if (heater == NULL) return NAN;
    return heater->raw_temp;
}

// ═══════════════════════════════════════════════════════════════════════════
// PID UPDATE — Call at 10 Hz (100ms intervals)
// ═══════════════════════════════════════════════════════════════════════════

void Heater_Update(Heater_t *heater)
{
    if (heater == NULL) return;

    uint32_t now = HAL_GetTick();
    float dt = (float)(now - heater->last_update_ms) / 1000.0f;
    heater->last_update_ms = now;

    if (dt <= 0.001f || dt > 2.0f) {
        dt = 0.1f;  // Fallback to expected 10 Hz rate
    }
    heater->last_dt_s = dt;

    float temp = Heater_ReadTemp(heater);

    // ── Graduated fault check ──
    Heater_Fault_t fault = check_faults(heater, temp, dt);
    if (fault >= FAULT_ERROR_SENSOR_SHORT) {
        if (heater->sensor_fault_streak < 0xFFU) {
            heater->sensor_fault_streak++;
        }
        heater->warning_fault_streak = 0U;

        if (heater->sensor_fault_streak < heater->config.sensor_fault_debounce) {
            heater->fault_level = FAULT_WARNING_ADC_NOISY;
            update_debug_state(heater);
            return;
        }

        heater->fault_level = fault;
        heater->last_p_term = 0.0f;
        heater->last_i_term = 0.0f;
        heater->last_d_term = 0.0f;
        heater->last_output_percent = 0.0f;
        heater->steady_time_s = 0.0f;
        set_pwm(heater, 0);
        update_debug_state(heater);
        return;
    } else if (fault > FAULT_NONE) {
        heater->sensor_fault_streak = 0U;
        if (heater->warning_fault_streak < 0xFFU) {
            heater->warning_fault_streak++;
        }
        if (heater->warning_fault_streak >= HEATER_WARNING_DEBOUNCE) {
            heater->fault_level = fault;
        } else {
            heater->fault_level = FAULT_NONE;
        }
    } else {
        heater->sensor_fault_streak = 0U;
        heater->warning_fault_streak = 0U;
        heater->fault_level = FAULT_NONE;
    }

    // ── Not enabled → PWM off, reset integrator ──
    if (!heater->enabled) {
        heater->sensor_fault_streak = 0U;
        set_pwm(heater, 0);
        heater->integral = 0.0f;
        heater->last_p_term = 0.0f;
        heater->last_i_term = 0.0f;
        heater->last_d_term = 0.0f;
        heater->last_output_percent = 0.0f;
        heater->steady_time_s = 0.0f;
        heater->last_temp = temp;
        update_debug_state(heater);
        return;
    }

    // ── PID calculation ──
    float error = heater->target_temp - temp;
    heater->last_error = error;

    // Proportional
    float p_term = heater->config.kp * error;

    // Integral with conditional accumulation:
    //   - Within 20°C of target: normal accumulation
    //   - Beyond 20°C: reset so it doesn't wind up during long approach
    //
    // FIX: integral_limit raised from 20 → 60 so the i_term can actually
    // drive PWM to 100% when far below target at high setpoints.
    // At ki=1.5, integral=60 → i_term=90. Combined with p_term at small
    // error, this allows full 100% output when needed.
    if (fabsf(error) < 20.0f) {
        heater->integral += error * dt;
    } else {
        heater->integral = 0.0f;
    }

    float integral_limit = 60.0f;  // was 20 — raised to allow full PWM output
    if (heater->integral > integral_limit)  heater->integral =  integral_limit;
    if (heater->integral < -integral_limit) heater->integral = -integral_limit;

    float i_term = heater->config.ki * heater->integral;

    // Derivative-on-measurement (not on error) with EMA filter
    float derivative = -(temp - heater->last_temp) / dt;
    heater->filtered_derivative = 0.3f * derivative + 0.7f * heater->filtered_derivative;
    float d_term = heater->config.kd * heater->filtered_derivative;

    heater->last_temp = temp;

    // ── Output clamping ──
    float output = p_term + i_term + d_term;
    if (output < 0.0f)   output = 0.0f;
    if (output > 100.0f) output = 100.0f;

    heater->last_p_term = p_term;
    heater->last_i_term = i_term;
    heater->last_d_term = d_term;
    heater->last_output_percent = output;

    uint16_t duty = (uint16_t)(output * 10.0f);
    set_pwm(heater, duty);
    update_debug_state(heater);
}

// ═══════════════════════════════════════════════════════════════════════════
// CONTROL
// ═══════════════════════════════════════════════════════════════════════════

void Heater_SetTarget(Heater_t *heater, float temp_c)
{
    if (heater == NULL) return;
    if (temp_c < 0.0f) temp_c = 0.0f;
    if (temp_c > heater->config.max_temp) temp_c = heater->config.max_temp;
    heater->target_temp = temp_c;
    heater->integral = 0.0f;
    heater->steady_time_s = 0.0f;
}

void Heater_Enable(Heater_t *heater, uint8_t enable)
{
    if (heater == NULL) return;
    heater->enabled = enable ? 1 : 0;
    if (!enable) {
        set_pwm(heater, 0);
        heater->integral = 0.0f;
        heater->steady_time_s = 0.0f;
    }
    update_debug_state(heater);
}

// ═══════════════════════════════════════════════════════════════════════════
// STATUS GETTERS
// ═══════════════════════════════════════════════════════════════════════════

float Heater_GetTemp(Heater_t *heater)
{
    if (heater == NULL) return NAN;
    return heater->current_temp;
}

float Heater_GetTarget(Heater_t *heater)
{
    if (heater == NULL) return 0.0f;
    return heater->target_temp;
}

uint8_t Heater_GetPWM(Heater_t *heater)
{
    if (heater == NULL) return 0;
    return (uint8_t)(heater->pwm_duty / 10);
}

uint16_t Heater_GetRawADC(Heater_t *heater)
{
    if (heater == NULL) return 0U;
    return heater->last_adc;
}

uint8_t Heater_IsEnabled(Heater_t *heater)
{
    if (heater == NULL) return 0;
    return heater->enabled;
}

uint8_t Heater_AtTarget(Heater_t *heater, float tolerance)
{
    if (heater == NULL) return 0;
    float diff = heater->current_temp - heater->target_temp;
    if (diff < 0.0f) diff = -diff;
    return (diff <= tolerance) ? 1 : 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// FAULT SYSTEM (graduated)
// ═══════════════════════════════════════════════════════════════════════════

uint8_t Heater_HasFault(Heater_t *heater)
{
    if (heater == NULL) return 1;  // Null = fault
    return (heater->fault_level >= FAULT_ERROR_SENSOR_SHORT) ? 1 : 0;
}

Heater_Fault_t Heater_GetFaultLevel(Heater_t *heater)
{
    if (heater == NULL) return FAULT_CRITICAL_SHUTDOWN;
    return heater->fault_level;
}

void Heater_ClearFault(Heater_t *heater)
{
    if (heater == NULL) return;
    heater->fault_level = FAULT_NONE;
    heater->sensor_fault_streak = 0U;
}

void Heater_GetDiagnostics(Heater_t *heater, Heater_Diagnostics_t *diag)
{
    if (heater == NULL || diag == NULL) return;

    diag->current_temp = heater->current_temp;
    diag->raw_temp = heater->raw_temp;
    diag->target_temp = heater->target_temp;
    diag->error = heater->target_temp - heater->current_temp;
    diag->p_term = heater->last_p_term;
    diag->i_term = heater->last_i_term;
    diag->d_term = heater->last_d_term;
    diag->integral = heater->integral;
    diag->filtered_derivative = heater->filtered_derivative;
    diag->output_percent = heater->last_output_percent;
    diag->dt_s = heater->last_dt_s;
    diag->pwm_duty = heater->pwm_duty;
    diag->adc_raw = heater->last_adc;
    diag->enabled = heater->enabled;
    diag->fault_level = heater->fault_level;
    diag->thermistor_status = heater->thermistor_status;
    diag->debug_flags = heater->debug_flags;
    diag->steady_time_s = heater->steady_time_s;
}

//uint32_t Heater_GetDebugFlags(Heater_t *heater)
//{
//    if (heater == NULL) return HEATER_DEBUG_FLAG_FAULT_ACTIVE | HEATER_DEBUG_FLAG_SENSOR_FAULT;
//    return heater->debug_flags;
//}
//
//uint8_t Heater_IsSteadyState(Heater_t *heater)
//{
//    if (heater == NULL) return 0U;
//    return ((heater->debug_flags & HEATER_DEBUG_FLAG_STEADY_STATE) != 0U) ? 1U : 0U;
//}

// ═══════════════════════════════════════════════════════════════════════════
// MANUAL CONTROL (for testing)
// ═══════════════════════════════════════════════════════════════════════════

void Heater_SetPWM(Heater_t *heater, uint8_t percent)
{
    if (heater == NULL) return;
    if (percent > 100) percent = 100;
    uint16_t duty = (uint16_t)percent * 10;
    set_pwm(heater, duty);
}

static void hotend_apply_command(Heater_t *hotend, const Hotend_Command_t *cmd)
{


    if (cmd == NULL) return;

	Heater_SetTarget(hotend, cmd->hotend_target);
	Heater_Enable(hotend, (cmd->hotend_target > 0.0f) ? 1U : 0U);
	hotend_set_state(hotend, (cmd->hotend_target > 0.0f) ? HEATER_STATE_HEATING : HEATER_STATE_OFF);
	hotend_clear_ready_sem();

}

void Hotend_ModuleInit(Heater_t *heater)
{
    Heater_Config_t config = {
        .kp = HOTEND_KP,
        .ki = HOTEND_KI,
        .kd = HOTEND_KD,
        .min_temp = HOTEND_MIN_TEMP,
        .max_temp = HOTEND_MAX_TEMP,
        .sensor_fault_debounce = 3U
    };

    if (heater == NULL) return;

    Thermistor_Init(NULL);
    Heater_InitWithSource(heater,
                          &HOTEND_PWM_TIMER,
                          HOTEND_PWM_CHANNEL,
                          &HOTEND_ADC_HANDLE,
                          &config,
                          Thermistor_ReadTemp,
                          Thermistor_GetLastADC,
                          Thermistor_GetLastStatus);
    hotend_set_state(heater, HEATER_STATE_INIT);
    hotend_clear_ready_sem();

}

void Hotend_Task(void *pvParameters)
{
	static Hotend_Command_t command;

    (void)pvParameters;
    Hotend_ModuleInit(&hotend_heater);
    for (;;) {
    	    switch (hotend_get_state(&hotend_heater)) {
    	        case HEATER_STATE_INIT:
    	            hotend_handle_init_state(&hotend_heater);
    	            hotend_set_state(&hotend_heater, HEATER_STATE_IDLE);
    	            break;
    	        case HEATER_STATE_IDLE:
    	        	if(xQueueReceive(xHotendQueue, &command, portMAX_DELAY) == pdTRUE)
    	        	{
                        hotend_apply_command(&hotend_heater, &command);
    	        	}
                    break;
    	        case HEATER_STATE_OFF:
    	            hotend_handle_off_state(&hotend_heater);
    	            break;

    	        case HEATER_STATE_HEATING:
    	            hotend_handle_heating_state(&hotend_heater, &command);
    	            break;

    	        case HEATER_STATE_HOLD:
    	            hotend_handle_holding_state(&hotend_heater);
    	            break;

    	        case HEATER_STATE_COOLING:
    	            hotend_handle_cooling_state(&hotend_heater);
    	            break;

    	        case HEATER_STATE_FAULT:
    	        default:
    	            hotend_handle_fault_state(&hotend_heater);
    	            break;
    	    }
        hotend_debug_log(&hotend_heater);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


//uint8_t Hotend_RequestTarget(float temp_c)
//{
//    Hotend_Command_t cmd = {
//        .type = HOTEND_CMD_SET_TARGET,
//        .value = temp_c
//    };
//
//    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && xHotendQueue != NULL) {
//        return (xQueueSend(xHotendQueue, &cmd, 0U) == pdPASS) ? 1U : 0U;
//    }
//
//    hotend_apply_command(&cmd);
//    return 1U;
//}
//
//uint8_t Hotend_RequestPartFanPercent(uint8_t percent)
//{
//    Hotend_Command_t cmd = {
//        .type = HOTEND_CMD_SET_PART_FAN_PERCENT,
//        .value = (float)percent
//    };
//
//    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && xHotendQueue != NULL) {
//        return (xQueueSend(xHotendQueue, &cmd, 0U) == pdPASS) ? 1U : 0U;
//    }
//
//    hotend_apply_command(&cmd);
//    return 1U;
//}
//
//uint8_t Hotend_RequestAllOff(void)
//{
//    Hotend_Command_t cmd = {
//        .type = HOTEND_CMD_ALL_OFF,
//        .value = 0.0f
//    };
//
//    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && xHotendQueue != NULL) {
//        return (xQueueSend(xHotendQueue, &cmd, 0U) == pdPASS) ? 1U : 0U;
//    }
//
//    hotend_apply_command(&cmd);
//    return 1U;
//}

//Heater_State_t Hotend_GetState(void)
//{
//    return hotend_state;
//}
//
//Hotend_FanMode_t Hotend_GetHotendFanMode(void)
//{
//    return hotend_fan_mode;
//}
//
//Hotend_FanMode_t Hotend_GetPartFanMode(void)
//{
//    return part_fan_mode;
//}
//
//uint8_t Hotend_Ready(void)
//{
//    return (hotend_state == HEATER_RUN_STATE_HOLDING) ? 1U : 0U;
//}

//uint8_t Hotend_WaitReady(uint32_t timeout_ms)
//{
//    uint32_t start_ms;
//
//    if (Hotend_Ready()) return 1U;
//    if (hotend_state == HEATER_RUN_STATE_FAULT) return 0U;
//
//    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && xHotendReadySem != NULL) {
//        if (xSemaphoreTake(xHotendReadySem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
//            return Hotend_Ready();
//        }
//        return Hotend_Ready();
//    }
//
//    start_ms = HAL_GetTick();
//    while ((HAL_GetTick() - start_ms) < timeout_ms) {
//        Hotend_TaskStep();
//        if (Hotend_Ready()) return 1U;
//        if (hotend_state == HEATER_RUN_STATE_FAULT) return 0U;
//        HAL_Delay(100);
//    }
//
//    return 0U;
//}
