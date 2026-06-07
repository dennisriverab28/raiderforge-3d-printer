#include "fan.h"
#include "ddr_globals.h"
#include "qbc_globals.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart_debug.h"

static Fan_State_t part_fan_state = FAN_STATE_INIT;
static Fan_Command_t part_fan_command;

static void part_fan_set_ready_sem(void)
{
    if (xFanReadySem == NULL) return;
    (void)xSemaphoreGive(xFanReadySem);
}

static void part_fan_clear_ready_sem(void)
{
    if (xFanReadySem == NULL) return;
    while (xSemaphoreTake(xFanReadySem, 0U) == pdTRUE) {
    }
}

static void part_fan_set_state(Fan_State_t new_state)
{
    if (xFanStateMutex != NULL &&
        xSemaphoreTake(xFanStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        part_fan_state = new_state;
        xSemaphoreGive(xFanStateMutex);
        return;
    }

    part_fan_state = new_state;
}

Fan_State_t PartFan_GetState(void)
{
    if (xFanStateMutex != NULL &&
        xSemaphoreTake(xFanStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Fan_State_t state = part_fan_state;
        xSemaphoreGive(xFanStateMutex);
        return state;
    }

    return FAN_STATE_FAULT;
}

static void part_fan_apply_command(const Fan_Command_t *cmd)
{
    uint8_t speed_percent;

    if (cmd == NULL) return;

    if (!cmd->fan_en || cmd->fan_target <= 0.0f) {
        Fan_Off(&DDRlo.part_fan);
        part_fan_set_state(FAN_STATE_IDLE);
        part_fan_set_ready_sem();
        return;
    }

    speed_percent = (cmd->fan_target < 0.0f) ? 0U :
                    (cmd->fan_target > 100.0f) ? 100U :
                    (uint8_t)(cmd->fan_target + 0.5f);

    Fan_SetSpeed(&DDRlo.part_fan, speed_percent);
    part_fan_set_state(FAN_STATE_HOLD);
    part_fan_set_ready_sem();
}

void PartFan_ModuleInit(void)
{
    Fan_Off(&DDRlo.part_fan);
    part_fan_set_state(FAN_STATE_INIT);
    part_fan_clear_ready_sem();
}

void PartFan_Task(void *pvParameters)
{
    (void)pvParameters;

    PartFan_ModuleInit();

    for (;;) {
        switch (PartFan_GetState()) {
        case FAN_STATE_INIT:
            part_fan_set_state(FAN_STATE_IDLE);
            break;

        case FAN_STATE_IDLE:
            if (xQueueReceive(xFanQueue, &part_fan_command, portMAX_DELAY) == pdTRUE) {
                part_fan_set_state(FAN_STATE_SET);
            }
            break;

        case FAN_STATE_SET:
            part_fan_apply_command(&part_fan_command);
            break;

        case FAN_STATE_HOLD:
            if (xQueueReceive(xFanQueue, &part_fan_command, 0U) == pdTRUE) {
                part_fan_clear_ready_sem();
                part_fan_set_state(FAN_STATE_SET);
            }
            break;

        case FAN_STATE_FAULT:
        default:
            Fan_Off(&DDRlo.part_fan);
            part_fan_set_state(FAN_STATE_IDLE);
            part_fan_set_ready_sem();
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════
void Fan_Init(Fan_t *fan,
              TIM_HandleTypeDef *pwm_timer,
              uint32_t pwm_channel,
              GPIO_TypeDef *gpio_port,
              uint16_t gpio_pin)
{
    if (fan == NULL) return;

    // Store hardware handles
    fan->pwm_timer = pwm_timer;
    fan->pwm_channel = pwm_channel;
    fan->gpio_port = gpio_port;
    fan->gpio_pin = gpio_pin;

    // Initialize state
    fan->mode = FAN_MODE_OFF;
    fan->speed_percent = 0;
    fan->enabled = 0;

    // Start PWM timer if provided
    if (fan->pwm_timer != NULL) {
        HAL_TIM_PWM_Start(fan->pwm_timer, fan->pwm_channel);
        __HAL_TIM_SET_COMPARE(fan->pwm_timer, fan->pwm_channel, 0);
    }

    // Set GPIO to off if provided
    if (fan->gpio_port != NULL) {
        HAL_GPIO_WritePin(fan->gpio_port, fan->gpio_pin, GPIO_PIN_RESET);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ON/OFF CONTROL
// ═══════════════════════════════════════════════════════════════════════════
void Fan_On(Fan_t *fan)
{
    if (fan == NULL) return;

    fan->enabled = 1;
    fan->speed_percent = 100;

    // If we have PWM, set to 100%
    if (fan->pwm_timer != NULL) {
        fan->mode = FAN_MODE_PWM;
        __HAL_TIM_SET_COMPARE(fan->pwm_timer, fan->pwm_channel, 1000);  // 100% = 1000/1000
    } else {
        // Otherwise just use GPIO
        fan->mode = FAN_MODE_ON_FULL;
    }

    // Set GPIO high if available
    if (fan->gpio_port != NULL) {
        HAL_GPIO_WritePin(fan->gpio_port, fan->gpio_pin, GPIO_PIN_SET);
    }
}

void Fan_Off(Fan_t *fan)
{
    if (fan == NULL) return;

    fan->enabled = 0;
    fan->speed_percent = 0;
    fan->mode = FAN_MODE_OFF;

    // Turn off PWM if available
    if (fan->pwm_timer != NULL) {
        __HAL_TIM_SET_COMPARE(fan->pwm_timer, fan->pwm_channel, 0);
    }

    // Turn off GPIO if available
    if (fan->gpio_port != NULL) {
        HAL_GPIO_WritePin(fan->gpio_port, fan->gpio_pin, GPIO_PIN_RESET);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// PWM SPEED CONTROL
// ═══════════════════════════════════════════════════════════════════════════
void Fan_SetSpeed(Fan_t *fan, uint8_t speed_percent)
{
    if (fan == NULL) return;
    if (fan->pwm_timer == NULL) return;  // Can't do PWM without timer

    // Clamp to 0-100%
    if (speed_percent > 100) speed_percent = 100;

    fan->speed_percent = speed_percent;

    if (speed_percent == 0) {
        // Speed 0 = turn off completely
        Fan_Off(fan);
    } else {
        // Set PWM duty cycle
        fan->enabled = 1;
        fan->mode = FAN_MODE_PWM;

        // Convert percentage to timer counts (0-1000 for ARR=999)
        uint16_t duty = (uint16_t)speed_percent * 10;
        __HAL_TIM_SET_COMPARE(fan->pwm_timer, fan->pwm_channel, duty);

        // Enable GPIO if available (PWM handled by timer alternate function)
        if (fan->gpio_port != NULL) {
            HAL_GPIO_WritePin(fan->gpio_port, fan->gpio_pin, GPIO_PIN_SET);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// STATUS GETTERS
// ═══════════════════════════════════════════════════════════════════════════
uint8_t Fan_GetSpeed(Fan_t *fan)
{
    if (fan == NULL) return 0;
    return fan->speed_percent;
}

uint8_t Fan_IsRunning(Fan_t *fan)
{
    if (fan == NULL) return 0;
    return fan->enabled;
}

Fan_Mode_t Fan_GetMode(Fan_t *fan)
{
    if (fan == NULL) return FAN_MODE_OFF;
    return fan->mode;
}
