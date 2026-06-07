/*
 * bed_heater.c
 *
 * Bed heater module using the same task/state/queue structure style as the
 * hotend orchestration in heater.c, while preserving the bed's existing
 * HEATING/HOLD/COOLING behavior.
 */

#include "bed_heater.h"
#include "qbc_globals.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart_debug.h"

#define BED_READY_TOLERANCE_C      2.0f
#define BED_COOLING_MIN_TEMP_C    40.0f
#define BED_DEBUG_PERIOD_MS     5000U

static Heater_t bed_heater;
static Bed_Command_t command;

static void bed_apply_command(Heater_t *bed, const Bed_Command_t *cmd);
static void bed_set_ready_sem(void);
static void bed_clear_ready_sem(void);
static void bed_set_state(Heater_t *bed, Heater_State_t new_state);
static Heater_State_t bed_get_state(Heater_t *bed);
static uint8_t bed_is_steady_state(Heater_t *bed);
static void bed_update_state_from_conditions(Heater_t *bed);
static void bed_update_outputs(Heater_t *bed);
static void bed_process_queue(Heater_t *bed);
static void bed_handle_init_state(Heater_t *bed);
static void bed_handle_off_state(Heater_t *bed);
static void bed_handle_heating_state(Heater_t *bed, const Bed_Command_t *cmd);
static void bed_handle_holding_state(Heater_t *bed);
static void bed_handle_cooling_state(Heater_t *bed);
static void bed_handle_fault_state(Heater_t *bed);
static const char *bed_state_name(Heater_State_t state);
static void bed_debug_log(Heater_t *bed);

static void bed_set_ready_sem(void)
{
    if (xBedReadySem == NULL) return;
    (void)xSemaphoreGive(xBedReadySem);
}

static void bed_clear_ready_sem(void)
{
    if (xBedReadySem == NULL) return;

    while (xSemaphoreTake(xBedReadySem, 0U) == pdTRUE) {
    }
}

static void bed_set_state(Heater_t *bed, Heater_State_t new_state)
{
    if (bed == NULL) return;

    bed->state = new_state;


}

static Heater_State_t bed_get_state(Heater_t *bed)
{
    if (bed == NULL) return HEATER_STATE_FAULT;

    return bed->state;
}

static uint8_t bed_is_steady_state(Heater_t *bed)
{
    if (bed == NULL) return 0U;
    return ((bed->debug_flags & HEATER_DEBUG_FLAG_STEADY_STATE) != 0U) ? 1U : 0U;
}

static void bed_update_state_from_conditions(Heater_t *bed)
{
    float temp;
    float target;

    if (bed == NULL) return;

    temp = Heater_GetTemp(bed);
    target = Heater_GetTarget(bed);

    if (Heater_HasFault(bed)) {
        bed_set_state(bed, HEATER_STATE_FAULT);
    } else if (Heater_IsEnabled(bed) &&
               target > 0.0f &&
               temp < (target - BED_READY_TOLERANCE_C)) {
        bed_set_state(bed, HEATER_STATE_HEATING);
    } else if (target > 0.0f &&
               (Heater_AtTarget(bed, BED_READY_TOLERANCE_C) ||
                bed_is_steady_state(bed))) {
        bed_set_state(bed, HEATER_STATE_HOLD);
    } else if (temp > BED_COOLING_MIN_TEMP_C &&
               temp > (target + BED_READY_TOLERANCE_C)) {
        bed_set_state(bed, HEATER_STATE_COOLING);
    } else if (!Heater_IsEnabled(bed) || target <= 0.0f) {
        bed_set_state(bed, HEATER_STATE_OFF);
    } else {
        bed_set_state(bed, HEATER_STATE_COOLING);
    }
}

static void bed_update_outputs(Heater_t *bed)
{
    if (bed == NULL) return;

    Heater_Update(bed);
    bed_update_state_from_conditions(bed);
}

static void bed_process_queue(Heater_t *bed)
{
    if (bed == NULL || xBedQueue == NULL) return;

    if(xQueueReceive(xBedQueue, &command, 0U) == pdPASS)
    	bed_apply_command(bed, &command);

}

static void bed_handle_off_state(Heater_t *bed)
{
    bed_clear_ready_sem();

    if (Heater_HasFault(bed)) {
        bed_set_state(bed, HEATER_STATE_FAULT);
        return;
    }

    if (Heater_IsEnabled(bed) &&
        Heater_GetTarget(bed) > 0.0f) {
        bed_set_state(bed, HEATER_STATE_HEATING);
    }

    bed_update_outputs(bed);
}

static void bed_handle_init_state(Heater_t *bed)
{
    bed_clear_ready_sem();

    if (Heater_HasFault(bed)) {
        bed_set_state(bed, HEATER_STATE_FAULT);
        return;
    }

    if (Heater_IsEnabled(bed) &&
        Heater_GetTarget(bed) > 0.0f) {
        bed_set_state(bed, HEATER_STATE_HEATING);
    } else {
        bed_set_state(bed, HEATER_STATE_OFF);
    }

    bed_update_outputs(bed);
}

static void bed_handle_heating_state(Heater_t *bed, const Bed_Command_t *cmd)
{


    bed_clear_ready_sem();

    if (Heater_HasFault(bed)) {
        bed_set_state(bed, HEATER_STATE_FAULT);
        return;
    }

    if (!Heater_IsEnabled(bed) ||
        Heater_GetTarget(bed) <= 0.0f) {
        bed_set_state(bed, HEATER_STATE_OFF);
    }


    bed_update_outputs(bed);
}

static void bed_handle_holding_state(Heater_t *bed)
{
    if (Heater_HasFault(bed)) {
        bed_set_state(bed, HEATER_STATE_FAULT);
        bed_clear_ready_sem();
        return;
    }

    if (!Heater_IsEnabled(bed) ||
        Heater_GetTarget(bed) <= 0.0f) {
        bed_set_state(bed, HEATER_STATE_OFF);
        bed_clear_ready_sem();
    } else {
        bed_set_ready_sem();
    }

    bed_update_outputs(bed);

    if (bed_get_state(bed) != HEATER_STATE_HOLD) {
        bed_clear_ready_sem();
    }
}

static void bed_handle_cooling_state(Heater_t *bed)
{
    bed_clear_ready_sem();

    if (Heater_HasFault(bed)) {
        bed_set_state(bed, HEATER_STATE_FAULT);
        return;
    }

    bed_update_outputs(bed);
}

static void bed_handle_fault_state(Heater_t *bed)
{
    bed_clear_ready_sem();
    Heater_Enable(bed, 0U);

    bed_update_outputs(bed);
}

static const char *bed_state_name(Heater_State_t state)
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

static void bed_debug_log(Heater_t *bed)
{
    static uint32_t next_log_ms = 0U;
    uint32_t now;

    if (bed == NULL) {
        return;
    }

    if (!Heater_IsEnabled(bed) &&
        Heater_GetTarget(bed) <= 0.0f &&
        !Heater_HasFault(bed)) {
        next_log_ms = 0U;
        return;
    }

    now = HAL_GetTick();
    if (next_log_ms != 0U && now < next_log_ms) {
        return;
    }

    UARTDBG_Print("[bed] %s T=%.1f/%.1f PWM=%u%% ADC=%u fault=%d\r\n",
                  bed_state_name(bed_get_state(bed)),
                  Heater_GetTemp(bed),
                  Heater_GetTarget(bed),
                  Heater_GetPWM(bed),
                  Heater_GetRawADC(bed),
                  (int)Heater_GetFaultLevel(bed));
    next_log_ms = now + BED_DEBUG_PERIOD_MS;
}

static void bed_apply_command(Heater_t *bed, const Bed_Command_t *cmd)
{
    if (bed == NULL || cmd == NULL) return;

    Heater_SetTarget(bed, cmd->bed_target);
    Heater_Enable(bed, cmd->bed_en);
    bed_set_state(bed, cmd->state);
    bed_clear_ready_sem();
}

void BedHeater_ModuleInit(void)
{
    bed_set_state(&bed_heater, HEATER_STATE_INIT);
    bed_clear_ready_sem();
}

void BedHeater_Init(TIM_HandleTypeDef *pwm_timer,
                    uint32_t pwm_channel,
                    ADC_HandleTypeDef *adc)
{
    Heater_Config_t config = {
        .kp = BED_KP,
        .ki = BED_KI,
        .kd = BED_KD,
        .min_temp = BED_MIN_TEMP,
        .max_temp = BED_MAX_TEMP,
        .sensor_fault_debounce = 3U
    };

    if (pwm_timer == NULL || adc == NULL) return;

    BedTherm_Init(NULL);
    Heater_InitWithSource(&bed_heater,
                          pwm_timer,
                          pwm_channel,
                          adc,
                          &config,
                          BedTherm_ReadTemp,
                          BedTherm_GetLastADC,
                          BedTherm_GetLastStatus);
    bed_update_state_from_conditions(&bed_heater);
    bed_clear_ready_sem();
}

void BedHeater_InitDefault(void)
{
    BedHeater_Init(&BED_PWM_TIMER, BED_PWM_CHANNEL, &BED_ADC_HANDLE);
}

void BedHeater_Task(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        bed_process_queue(&bed_heater);

        switch (bed_get_state(&bed_heater)) {
            case HEATER_STATE_INIT:
                bed_handle_init_state(&bed_heater);
                break;

            case HEATER_STATE_IDLE:
                if (xBedQueue != NULL &&
                    xQueueReceive(xBedQueue, &command, portMAX_DELAY) == pdTRUE) {
                    bed_apply_command(&bed_heater, &command);
                }
                break;

            case HEATER_STATE_OFF:
                bed_handle_off_state(&bed_heater);
                break;

            case HEATER_STATE_HEATING:
                bed_handle_heating_state(&bed_heater, &command);
                break;

            case HEATER_STATE_HOLD:
                bed_handle_holding_state(&bed_heater);
                break;

            case HEATER_STATE_COOLING:
                bed_handle_cooling_state(&bed_heater);
                break;

            case HEATER_STATE_FAULT:
            default:
                bed_handle_fault_state(&bed_heater);
                break;
        }

        bed_debug_log(&bed_heater);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

float BedHeater_ReadTemp(void)
{
    return Heater_ReadTemp(&bed_heater);
}

float BedHeater_GetTemp(void)
{
    return Heater_GetTemp(&bed_heater);
}

void BedHeater_Update(void)
{
    Heater_Update(&bed_heater);
}

void BedHeater_SetTarget(float temp_c)
{
    command.bed_en = (temp_c > 0.0f) ? 1U : 0U;
    command.bed_target = temp_c;
    command.state = (temp_c > 0.0f) ? HEATER_STATE_HEATING : HEATER_STATE_OFF;


    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && xBedQueue != NULL) {
        if (xQueueSend(xBedQueue, &command, 0U) == pdPASS) {
            return;
        }
    }

    bed_apply_command(&bed_heater, &command);
}

void BedHeater_Enable(uint8_t enable)
{


    if (enable) {
        if (Heater_GetTarget(&bed_heater) <= 0.0f) {
            return;
        }

        command.bed_en = 1U;
        command.bed_target = Heater_GetTarget(&bed_heater);
        command.state = HEATER_STATE_HEATING;

        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && xBedQueue != NULL) {
            if (xQueueSend(xBedQueue, &command, 0U) == pdPASS) {
                return;
            }
        }

        bed_apply_command(&bed_heater, &command);
        return;
    }

    command.bed_en = 0U;
    command.bed_target = 0.0f;
    command.state = HEATER_STATE_OFF;

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && xBedQueue != NULL) {
        if (xQueueSend(xBedQueue, &command, 0U) == pdPASS) {
            return;
        }
    }

    bed_apply_command(&bed_heater, &command);
}

float BedHeater_GetTarget(void)
{
    return Heater_GetTarget(&bed_heater);
}

//uint8_t BedHeater_GetPWM(void)
//{
//    return Heater_GetPWM(&bed_heater);
//}

//uint8_t BedHeater_IsEnabled(void)
//{
//    return Heater_IsEnabled(&bed_heater);
//}

//uint8_t BedHeater_AtTarget(float tolerance)
//{
//    return Heater_AtTarget(&bed_heater, tolerance);
//}

//Heater_State_t BedHeater_GetState(void)
//{
//    return bed_get_state(&bed_heater);
//}


uint8_t BedHeater_HasFault(void)
{
    return Heater_HasFault(&bed_heater);
}

Heater_Fault_t BedHeater_GetFaultLevel(void)
{
    return Heater_GetFaultLevel(&bed_heater);
}

void BedHeater_ClearFault(void)
{
    Heater_ClearFault(&bed_heater);
}

uint32_t BedHeater_GetDebugFlags(void)
{
    return bed_heater.debug_flags;
}

uint8_t BedHeater_IsSteadyState(void)
{
    return bed_is_steady_state(&bed_heater);
}

void BedHeater_SetPWM(uint8_t percent)
{
    Heater_SetPWM(&bed_heater, percent);
}

Heater_t *BedHeater_GetHandle(void)
{
    return &bed_heater;
}

void BedHeater_GetDiagnostics(Heater_Diagnostics_t *diag)
{
    Heater_GetDiagnostics(&bed_heater, diag);
}
