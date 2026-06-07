/*
 * stepper.c
 *
 * Stepper motor driver for RaiderForge CoreXY printer.
 *
 * Key change from timing-based version:
 *   Stepper_MoveSteps() enables the timer update interrupt.
 *   Each interrupt = one step pulse (one PWM period).
 *   ISR decrements steps_remaining and stops the timer at exactly zero.
 *   No more elapsed-ms guessing. Motor takes exactly N steps.
 *
 * Jog mode (Stepper_Jog) is unchanged — continuous PWM, ISR ignores it.
 * Used by homing only.
 *
 * Updated: Apr 2026 | Author: Cherryman125 (Dennis)
 */

#include "stepper.h"
#include <stdlib.h>

/* ─── Timer registration table ──────────────────────────────────────────── */

static Stepper_t *g_tim1_stepper  = NULL;  /* Reserved for TIM1-based steppers */
static Stepper_t *g_tim3_ch1      = NULL;  /* Motor A — TIM3 CH1 */
static Stepper_t *g_tim3_ch2      = NULL;  /* Motor B — TIM3 CH2 */
static Stepper_t *g_tim15_ch1     = NULL;  /* Motor Z — TIM15 CH1 */
static Stepper_t *g_tim17_ch1     = NULL;  /* Motor E — TIM17 CH1 */

/* ─── Private helpers ───────────────────────────────────────────────────── */

static void write_en(Stepper_t *s, uint8_t enable)
{
    if (s->en_active_low) {
        HAL_GPIO_WritePin(s->EN_Port, s->EN_Pin,
                          enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(s->EN_Port, s->EN_Pin,
                          enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/*
 * apply_pwm_frequency — configure timer prescaler/ARR for freq_hz steps/sec.
 *
 * G0B1 runs at 16 MHz HSI (no PLL).
 *   PSC=1599 → timer clock = 10 kHz   (freq < 100 Hz)
 *   PSC=159  → timer clock = 100 kHz  (100–999 Hz)
 *   PSC=15   → timer clock = 1 MHz    (>= 1000 Hz)
 *
 * One timer period = one step pulse. Does NOT touch the update interrupt.
 */
static void apply_pwm_frequency(Stepper_t *s, uint32_t freq_hz)
{
    if (freq_hz == 0) {
        HAL_TIM_PWM_Stop(s->htim, s->tim_channel);
        return;
    }

    uint32_t prescaler, arr;

    if (freq_hz < 100) {
        prescaler = 1599;
        arr = (10000 / freq_hz) - 1;
    } else if (freq_hz < 1000) {
        prescaler = 159;
        arr = (100000 / freq_hz) - 1;
    } else {
        prescaler = 15;
        arr = (1000000 / freq_hz) - 1;
    }

    if (arr > 65535) arr = 65535;
    if (arr < 1)     arr = 1;

    __HAL_TIM_SET_PRESCALER(s->htim, prescaler);
    __HAL_TIM_SET_AUTORELOAD(s->htim, arr);
    __HAL_TIM_SET_COMPARE(s->htim, s->tim_channel, arr / 2);  /* 50% duty */

    HAL_TIM_PWM_Start(s->htim, s->tim_channel);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * INITIALIZATION
 * ═══════════════════════════════════════════════════════════════════════════ */

void Stepper_Init(Stepper_t *s,
                  TIM_HandleTypeDef *htim,
                  uint32_t channel,
                  GPIO_TypeDef *dirPort, uint16_t dirPin,
                  GPIO_TypeDef *enPort,  uint16_t enPin,
                  uint8_t en_active_low,
                  uint32_t steps_per_rev)
{
    s->htim          = htim;
    s->tim_channel   = channel;
    s->DIR_Port      = dirPort;
    s->DIR_Pin       = dirPin;
    s->EN_Port       = enPort;
    s->EN_Pin        = enPin;
    s->en_active_low = en_active_low;
    s->steps_per_rev = steps_per_rev;

    s->enabled          = 0;
    s->dir              = 0;
    s->current_speed_hz = 0;
    s->target_speed_hz  = 0;
    s->accel_rate       = 500;
    s->last_update_ms   = 0;
    s->counted_soft_start_hz = 0;

    /* Step counting init */
    s->steps_remaining      = STEPPER_JOG_SENTINEL;
    s->move_complete        = 1;
    s->shared_move          = 0;
    s->pulse_armed          = 0;
    s->dda_accum            = 0;
    s->dda_numer            = 0;
    s->dda_denom            = 0;
    s->step_count           = 0;
    s->step_count_reset_val = 0;

    HAL_GPIO_WritePin(s->DIR_Port, s->DIR_Pin, GPIO_PIN_RESET);
    write_en(s, 0);
    HAL_TIM_PWM_Stop(s->htim, s->tim_channel);

    /* TIM1 update interrupt must stay enabled — it is the HAL timebase tick.
     * For all other timers disable it here; MoveSteps will re-enable it. */
    if (s->htim->Instance != TIM1) {
        __HAL_TIM_DISABLE_IT(s->htim, TIM_IT_UPDATE);
        __HAL_TIM_CLEAR_FLAG(s->htim, TIM_FLAG_UPDATE);
    }


}

void Stepper_RegisterTimer(Stepper_t *s,
                            TIM_HandleTypeDef *htim,
                            uint32_t channel)
{
    s->htim        = htim;
    s->tim_channel = channel;

    if (htim->Instance == TIM1) {
        g_tim1_stepper = s;

        /* TIM1 is an advanced timer AND the HAL timebase — TIM_IT_UPDATE must
         * remain enabled at all times (HAL_IncTick runs from it).
         * Clear/disable only the non-UPDATE advanced-timer interrupt sources. */
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_BREAK | TIM_IT_TRIGGER | TIM_IT_COM |
                                   TIM_IT_CC1 | TIM_IT_CC2 |
                                   TIM_IT_CC3 | TIM_IT_CC4);
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_BREAK | TIM_FLAG_TRIGGER | TIM_FLAG_COM |
                                   TIM_FLAG_CC1 | TIM_FLAG_CC2 |
                                   TIM_FLAG_CC3 | TIM_FLAG_CC4);

        /* Priority 3 = same level as TIM6 HAL timebase (TICK_INT_PRIORITY).
         * Stepper ISRs must NEVER outrank TIM6 or HAL_Delay will stall.   */
        HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    }
    else if (htim->Instance == TIM3) {
        if (channel == TIM_CHANNEL_1) g_tim3_ch1 = s;
        else                          g_tim3_ch2 = s;

        __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE | TIM_IT_TRIGGER |
                                   TIM_IT_CC1 | TIM_IT_CC2 |
                                   TIM_IT_CC3 | TIM_IT_CC4);
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE | TIM_FLAG_TRIGGER |
                                   TIM_FLAG_CC1 | TIM_FLAG_CC2 |
                                   TIM_FLAG_CC3 | TIM_FLAG_CC4);

        HAL_NVIC_SetPriority(TIM3_TIM4_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(TIM3_TIM4_IRQn);
    }
    else if (htim->Instance == TIM15) {
        g_tim15_ch1 = s;

        __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE | TIM_IT_TRIGGER |
                                   TIM_IT_BREAK  | TIM_IT_CC1);
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE | TIM_FLAG_TRIGGER |
                                   TIM_FLAG_BREAK  | TIM_FLAG_CC1);

        HAL_NVIC_SetPriority(TIM15_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(TIM15_IRQn);
    }
    else if (htim->Instance == TIM17) {
        g_tim17_ch1 = s;

        __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE | TIM_IT_TRIGGER |
                                   TIM_IT_BREAK  | TIM_IT_CC1);
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE | TIM_FLAG_TRIGGER |
                                   TIM_FLAG_BREAK  | TIM_FLAG_CC1);

        HAL_NVIC_SetPriority(TIM17_FDCAN_IT1_IRQn, 3, 0);
        HAL_NVIC_ClearPendingIRQ(TIM17_FDCAN_IT1_IRQn);
        HAL_NVIC_EnableIRQ(TIM17_FDCAN_IT1_IRQn);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * BASIC CONTROL
 * ═══════════════════════════════════════════════════════════════════════════ */

void Stepper_Enable(Stepper_t *s, uint8_t en)
{
    s->enabled = en ? 1 : 0;
    write_en(s, s->enabled);
    if (!s->enabled) {
        if (s->htim->Instance != TIM1) __HAL_TIM_DISABLE_IT(s->htim, TIM_IT_UPDATE);
        HAL_TIM_PWM_Stop(s->htim, s->tim_channel);
        s->current_speed_hz = 0;
        s->steps_remaining  = STEPPER_JOG_SENTINEL;
        s->move_complete    = 1;
        s->shared_move      = 0;
        s->pulse_armed      = 0;
        s->dda_accum        = 0;
        s->dda_numer        = 0;
        s->dda_denom        = 0;
    }
}

void Stepper_SetDir(Stepper_t *s, uint8_t dir)
{
    s->dir = dir ? 1 : 0;
    HAL_GPIO_WritePin(s->DIR_Port, s->DIR_Pin,
                      s->dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Stepper_Stop(Stepper_t *s)
{
    if (s->htim->Instance != TIM1) __HAL_TIM_DISABLE_IT(s->htim, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(s->htim, TIM_FLAG_UPDATE);
    HAL_TIM_PWM_Stop(s->htim, s->tim_channel);
    s->current_speed_hz = 0;
    s->target_speed_hz  = 0;
    s->steps_remaining  = STEPPER_JOG_SENTINEL;
    s->move_complete    = 1;
    s->shared_move      = 0;
    s->pulse_armed      = 0;
    s->dda_accum        = 0;
    s->dda_numer        = 0;
    s->dda_denom        = 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * JOG MODE — continuous, for homing
 * ═══════════════════════════════════════════════════════════════════════════ */

void Stepper_Jog(Stepper_t *s, uint32_t steps_per_sec)
{
    s->steps_remaining  = STEPPER_JOG_SENTINEL;
    s->move_complete    = 0;
    s->shared_move      = 0;
    s->pulse_armed      = 0;
    s->dda_accum        = 0;
    s->dda_numer        = 0;
    s->dda_denom        = 0;
    s->current_speed_hz = steps_per_sec;
    s->target_speed_hz  = steps_per_sec;

    /* Jog never counts steps — disable update interrupt (except TIM1: HAL tick) */
    if (s->htim->Instance != TIM1) __HAL_TIM_DISABLE_IT(s->htim, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(s->htim, TIM_FLAG_UPDATE);

    if (s->enabled) {
        apply_pwm_frequency(s, steps_per_sec);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * COUNTED MOVE MODE — exact N steps
 * ═══════════════════════════════════════════════════════════════════════════ */

void Stepper_MoveSteps(Stepper_t *s, uint32_t n_steps, uint32_t freq_hz)
{
    if (n_steps == 0) {
        s->steps_remaining = 0;
        s->move_complete   = 1;
        return;
    }
    if (!s->enabled) {
        s->steps_remaining = 0;
        s->move_complete   = 1;
        return;
    }

    /* Disable interrupt during config to prevent spurious counts (not TIM1: HAL tick) */
    if (s->htim->Instance != TIM1) __HAL_TIM_DISABLE_IT(s->htim, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(s->htim, TIM_FLAG_UPDATE);

    s->move_complete    = 0;
    s->shared_move      = 0;
    s->pulse_armed      = 1;
    s->dda_accum        = 0;
    s->dda_numer        = 0;
    s->dda_denom        = 0;
    s->steps_remaining  = (int32_t)n_steps;
    s->target_speed_hz  = freq_hz;
    s->last_update_ms   = HAL_GetTick();

    if (s->counted_soft_start_hz > 0U &&
        freq_hz > s->counted_soft_start_hz) {
        s->current_speed_hz = s->counted_soft_start_hz;
    } else {
        s->current_speed_hz = freq_hz;
    }

    apply_pwm_frequency(s, s->current_speed_hz);

    /* Enable update interrupt — ISR counts from here */
    __HAL_TIM_CLEAR_FLAG(s->htim, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(s->htim, TIM_IT_UPDATE);
}

static void arm_shared_pulse(Stepper_t *s)
{
    uint32_t arr;

    if (s == NULL) return;

    if (s->move_complete || s->steps_remaining <= 0 || s->dda_denom == 0U) {
        __HAL_TIM_SET_COMPARE(s->htim, s->tim_channel, 0);
        s->pulse_armed = 0;
        return;
    }

    s->dda_accum += s->dda_numer;
    if (s->dda_accum >= s->dda_denom) {
        s->dda_accum -= s->dda_denom;
        arr = __HAL_TIM_GET_AUTORELOAD(s->htim);
        __HAL_TIM_SET_COMPARE(s->htim, s->tim_channel, (arr > 1U) ? (arr / 2U) : 1U);
        s->pulse_armed = 1;
    } else {
        __HAL_TIM_SET_COMPARE(s->htim, s->tim_channel, 0);
        s->pulse_armed = 0;
    }
}

void Stepper_MoveStepsSharedTimer(Stepper_t *s1, uint32_t n1_steps,
                                  Stepper_t *s2, uint32_t n2_steps,
                                  uint32_t freq_hz)
{
    uint32_t max_steps;

    if (s1 == NULL || s2 == NULL) return;

    max_steps = (n1_steps > n2_steps) ? n1_steps : n2_steps;
    if (max_steps == 0U || freq_hz == 0U) {
        Stepper_Stop(s1);
        Stepper_Stop(s2);
        return;
    }

    if (!s1->enabled || !s2->enabled) {
        if (!s1->enabled) {
            s1->steps_remaining = 0;
            s1->move_complete = 1;
        }
        if (!s2->enabled) {
            s2->steps_remaining = 0;
            s2->move_complete = 1;
        }
        return;
    }

    if (s1->htim->Instance != TIM1) __HAL_TIM_DISABLE_IT(s1->htim, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(s1->htim, TIM_FLAG_UPDATE);

    s1->shared_move = 1;
    s2->shared_move = 1;
    s1->move_complete = (n1_steps == 0U) ? 1U : 0U;
    s2->move_complete = (n2_steps == 0U) ? 1U : 0U;
    s1->steps_remaining = (int32_t)n1_steps;
    s2->steps_remaining = (int32_t)n2_steps;
    s1->target_speed_hz = freq_hz;
    s2->target_speed_hz = freq_hz;
    s1->last_update_ms = HAL_GetTick();
    s2->last_update_ms = s1->last_update_ms;
    s1->dda_accum = 0U;
    s2->dda_accum = 0U;
    s1->dda_numer = n1_steps;
    s2->dda_numer = n2_steps;
    s1->dda_denom = max_steps;
    s2->dda_denom = max_steps;
    s1->pulse_armed = 0U;
    s2->pulse_armed = 0U;

    if (s1->counted_soft_start_hz > 0U &&
        freq_hz > s1->counted_soft_start_hz) {
        s1->current_speed_hz = s1->counted_soft_start_hz;
    } else {
        s1->current_speed_hz = freq_hz;
    }

    if (s2->counted_soft_start_hz > 0U &&
        freq_hz > s2->counted_soft_start_hz) {
        s2->current_speed_hz = s2->counted_soft_start_hz;
    } else {
        s2->current_speed_hz = freq_hz;
    }

    apply_pwm_frequency(s1, s1->current_speed_hz);
    apply_pwm_frequency(s2, s2->current_speed_hz);

    arm_shared_pulse(s1);
    arm_shared_pulse(s2);

    __HAL_TIM_CLEAR_FLAG(s1->htim, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(s1->htim, TIM_IT_UPDATE);
}

uint8_t Stepper_IsDone(Stepper_t *s)
{
    if (s->steps_remaining == STEPPER_JOG_SENTINEL) return 1;
    return s->move_complete;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ISR HANDLERS
 *
 * Keep minimal: check update flag, decrement counter, stop at zero.
 * Never call HAL_TIM_IRQHandler here — that calls PeriodElapsedCallback
 * which we don't need and adds ISR latency.
 * ═══════════════════════════════════════════════════════════════════════════ */

static uint8_t isr_step(Stepper_t *s)
{
    if (s == NULL) return 0;
    if (s->steps_remaining == STEPPER_JOG_SENTINEL) return 0;
    if (s->steps_remaining <= 0) return 0;

    s->steps_remaining--;
    s->step_count++;  /* legacy counter */

    if (s->steps_remaining == 0) {
        /* Pull compare to 0 — output goes low immediately, no half-pulse glitch */
        __HAL_TIM_SET_COMPARE(s->htim, s->tim_channel, 0);
        s->move_complete    = 1;
        s->current_speed_hz = 0;
        return 1;
    }
    return 0;
}

static void isr_step_shared(Stepper_t *s)
{
    if (s == NULL || !s->shared_move) return;

    if (s->pulse_armed && s->steps_remaining > 0) {
        s->steps_remaining--;
        s->step_count++;
        if (s->steps_remaining == 0) {
            s->move_complete = 1;
            s->current_speed_hz = 0;
        }
    }

    arm_shared_pulse(s);
}

void Stepper_TIM1_UpdateISR(void)
{
    if (g_tim1_stepper == NULL) return;
    isr_step(g_tim1_stepper);
    /* Never disable TIM1 TIM_IT_UPDATE — TIM1 is the HAL timebase tick.
     * isr_step returns immediately (no-op) when not in a counted move. */
}

void Stepper_TIM3_UpdateISR(void)
{
    if ((g_tim3_ch1 && g_tim3_ch1->shared_move) ||
        (g_tim3_ch2 && g_tim3_ch2->shared_move)) {
        if (g_tim3_ch1) isr_step_shared(g_tim3_ch1);
        if (g_tim3_ch2) isr_step_shared(g_tim3_ch2);
    } else {
        if (g_tim3_ch1) isr_step(g_tim3_ch1);
        if (g_tim3_ch2) isr_step(g_tim3_ch2);
    }

    uint8_t ch1_idle = (g_tim3_ch1 == NULL) ||
                       (g_tim3_ch1->move_complete) ||
                       (g_tim3_ch1->steps_remaining == STEPPER_JOG_SENTINEL);
    uint8_t ch2_idle = (g_tim3_ch2 == NULL) ||
                       (g_tim3_ch2->move_complete) ||
                       (g_tim3_ch2->steps_remaining == STEPPER_JOG_SENTINEL);

    if (ch1_idle && ch2_idle) {
        if (g_tim3_ch1)
            __HAL_TIM_DISABLE_IT(g_tim3_ch1->htim, TIM_IT_UPDATE);
        else if (g_tim3_ch2)
            __HAL_TIM_DISABLE_IT(g_tim3_ch2->htim, TIM_IT_UPDATE);
    }
}

void Stepper_TIM15_UpdateISR(void)
{
    if (g_tim15_ch1) isr_step(g_tim15_ch1);

    if (g_tim15_ch1 == NULL ||
        g_tim15_ch1->move_complete ||
        g_tim15_ch1->steps_remaining == STEPPER_JOG_SENTINEL) {
        if (g_tim15_ch1)
            __HAL_TIM_DISABLE_IT(g_tim15_ch1->htim, TIM_IT_UPDATE);
    }
}

void Stepper_TIM17_UpdateISR(void)
{
    if (g_tim17_ch1) isr_step(g_tim17_ch1);

    if (g_tim17_ch1 == NULL ||
        g_tim17_ch1->move_complete ||
        g_tim17_ch1->steps_remaining == STEPPER_JOG_SENTINEL) {
        if (g_tim17_ch1)
            __HAL_TIM_DISABLE_IT(g_tim17_ch1->htim, TIM_IT_UPDATE);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ACCELERATION (unchanged)
 * ═══════════════════════════════════════════════════════════════════════════ */

void Stepper_SetAcceleration(Stepper_t *s, uint32_t accel_steps_per_sec2)
{
    s->accel_rate = accel_steps_per_sec2;
}

void Stepper_SetTargetSpeed(Stepper_t *s, uint32_t steps_per_sec)
{
    s->target_speed_hz = steps_per_sec;
    s->last_update_ms  = HAL_GetTick();
}

void Stepper_SetCountedSoftStart(Stepper_t *s, uint32_t start_steps_per_sec)
{
    s->counted_soft_start_hz = start_steps_per_sec;
}

void Stepper_Task(Stepper_t *s)
{
    if (!s->enabled) return;
    if (s->current_speed_hz == s->target_speed_hz) return;
    if (s->steps_remaining != STEPPER_JOG_SENTINEL &&
        s->counted_soft_start_hz == 0U) return;

    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - s->last_update_ms;
    if (elapsed_ms < 10) return;
    s->last_update_ms = now;

    uint32_t delta = (s->accel_rate * elapsed_ms) / 1000;
    if (delta < 1) delta = 1;

    if (s->current_speed_hz < s->target_speed_hz) {
        s->current_speed_hz += delta;
        if (s->current_speed_hz > s->target_speed_hz)
            s->current_speed_hz = s->target_speed_hz;
    } else {
        s->current_speed_hz = (s->current_speed_hz > delta)
                               ? s->current_speed_hz - delta : 0;
        if (s->current_speed_hz < s->target_speed_hz)
            s->current_speed_hz = s->target_speed_hz;
    }

    apply_pwm_frequency(s, s->current_speed_hz);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * STATUS
 * ═══════════════════════════════════════════════════════════════════════════ */

uint8_t Stepper_IsRunning(Stepper_t *s)
{
    if (!s->enabled) return 0;
    if (s->steps_remaining == STEPPER_JOG_SENTINEL)
        return (s->current_speed_hz > 0) ? 1 : 0;
    return (s->steps_remaining > 0) ? 1 : 0;
}

uint32_t Stepper_GetCurrentSpeed(Stepper_t *s)
{
    return s->current_speed_hz;
}

void Stepper_SetSpeedRPM(Stepper_t *s, uint32_t rpm)
{
    Stepper_Jog(s, (rpm * s->steps_per_rev) / 60);
}

void Stepper_ResetStepCount(Stepper_t *s)
{
    s->step_count = 0;
}

uint32_t Stepper_GetStepCount(Stepper_t *s)
{
    return s->step_count;
}
