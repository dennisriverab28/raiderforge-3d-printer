/*
 * stepper.h
 *
 * Stepper motor driver for RaiderForge CoreXY printer.
 *
 * TWO OPERATING MODES:
 *
 *   JOG (continuous) — Stepper_Jog() / Stepper_SetSpeed()
 *     PWM runs indefinitely at fixed frequency.
 *     steps_remaining = STEPPER_JOG_SENTINEL (-1).
 *     ISR ignores jog-mode steppers.
 *     Used exclusively by homing (JogX/JogY/JogStop in corexy.c).
 *     Stop with Stepper_Stop().
 *
 *   COUNTED MOVE — Stepper_MoveSteps()
 *     PWM runs for exactly N pulses then stops automatically.
 *     Timer update ISR counts each pulse, stops timer at zero.
 *     Completion signalled by move_complete = 1.
 *     Poll with Stepper_IsDone().
 *     Used by CoreXY_MoveAbsolute(), Z_MoveAbsolute(), E_Extrude/Retract().
 *
 * ISR REGISTRATION:
 *   Call Stepper_RegisterTimer() once per motor after Stepper_Init(),
 *   before osKernelStart(). TIM3 and TIM15 are shared timers.
 *
 * Updated: Apr 2026 — step counting replaces timing-based move completion.
 */

#ifndef STEPPER_H
#define STEPPER_H

#include "stm32g0xx_hal.h"
#include <stdint.h>

/* Sentinel: steps_remaining == this value means jog mode (ISR does nothing) */
#define STEPPER_JOG_SENTINEL  (-1)

typedef struct {
    /* ── Hardware ── */
    TIM_HandleTypeDef *htim;
    uint32_t           tim_channel;
    GPIO_TypeDef      *DIR_Port;
    uint16_t           DIR_Pin;
    GPIO_TypeDef      *EN_Port;
    uint16_t           EN_Pin;
    uint8_t            en_active_low;
    uint32_t           steps_per_rev;

    /* ── Motion state ── */
    uint8_t  enabled;
    uint8_t  dir;
    uint32_t current_speed_hz;
    uint32_t target_speed_hz;
    uint32_t accel_rate;
    uint32_t last_update_ms;
    uint32_t counted_soft_start_hz;

    /* ── Step counting (ISR-written — must be volatile) ──
     *
     *   steps_remaining:
     *     > 0  : counted move in progress
     *     == 0 : move complete (set by ISR)
     *     == STEPPER_JOG_SENTINEL (-1): jog mode, ISR ignores
     *
     *   move_complete:
     *     Set to 1 by ISR when steps_remaining reaches 0.
     *     Poll with Stepper_IsDone(). Cleared by Stepper_MoveSteps().
     */
    volatile int32_t steps_remaining;
    volatile uint8_t move_complete;
    volatile uint8_t shared_move;
    volatile uint8_t pulse_armed;

    uint32_t dda_accum;
    uint32_t dda_numer;
    uint32_t dda_denom;

    /* Legacy step counter (kept for Test_StepCounter in motion_tests.c) */
    volatile uint32_t step_count;
    uint32_t          step_count_reset_val;

} Stepper_t;

/* ─── Initialization ────────────────────────────────────────────────────── */

void Stepper_Init(Stepper_t *s,
                  TIM_HandleTypeDef *htim,
                  uint32_t channel,
                  GPIO_TypeDef *dirPort, uint16_t dirPin,
                  GPIO_TypeDef *enPort,  uint16_t enPin,
                  uint8_t en_active_low,
                  uint32_t steps_per_rev);

/*
 * Stepper_RegisterTimer — register a stepper with the ISR dispatch table.
 * Call once per motor after Stepper_Init(), before osKernelStart().
 * For shared timers such as TIM3 (A+B) and TIM15 (Z+E),
 * call once per channel you use.
 */
void Stepper_RegisterTimer(Stepper_t *s,
                            TIM_HandleTypeDef *htim,
                            uint32_t channel);

/* ─── Basic control ─────────────────────────────────────────────────────── */

void Stepper_Enable(Stepper_t *s, uint8_t en);
void Stepper_SetDir(Stepper_t *s, uint8_t dir);
void Stepper_Stop(Stepper_t *s);

/* ─── JOG mode (continuous PWM — homing only) ──────────────────────────── */

void Stepper_Jog(Stepper_t *s, uint32_t steps_per_sec);

/* Stepper_SetSpeed — alias for Stepper_Jog, kept for compat */
static inline void Stepper_SetSpeed(Stepper_t *s, uint32_t steps_per_sec)
{
    Stepper_Jog(s, steps_per_sec);
}

/* ─── COUNTED MOVE mode ─────────────────────────────────────────────────── */

void    Stepper_MoveSteps(Stepper_t *s, uint32_t n_steps, uint32_t freq_hz);
void    Stepper_MoveStepsSharedTimer(Stepper_t *s1, uint32_t n1_steps,
                                     Stepper_t *s2, uint32_t n2_steps,
                                     uint32_t freq_hz);
uint8_t Stepper_IsDone(Stepper_t *s);

/* ─── Acceleration ──────────────────────────────────────────────────────── */

void Stepper_SetAcceleration(Stepper_t *s, uint32_t accel_steps_per_sec2);
void Stepper_SetTargetSpeed(Stepper_t *s, uint32_t steps_per_sec);
void Stepper_SetCountedSoftStart(Stepper_t *s, uint32_t start_steps_per_sec);
void Stepper_Task(Stepper_t *s);

/* ─── Status ────────────────────────────────────────────────────────────── */

uint8_t  Stepper_IsRunning(Stepper_t *s);
uint32_t Stepper_GetCurrentSpeed(Stepper_t *s);
void     Stepper_SetSpeedRPM(Stepper_t *s, uint32_t rpm);

/* Legacy step counter */
void     Stepper_ResetStepCount(Stepper_t *s);
uint32_t Stepper_GetStepCount(Stepper_t *s);

/* ─── ISR handlers — call from stm32g0xx_it.c ──────────────────────────── */

void Stepper_TIM1_UpdateISR(void);    /* Reserved for TIM1-based steppers */
void Stepper_TIM3_UpdateISR(void);    /* Motor A CH1 PB4 + Motor B CH2 PB5 */
void Stepper_TIM15_UpdateISR(void);   /* Motor Z CH1 */
void Stepper_TIM17_UpdateISR(void);   /* Motor E CH1 */

#endif /* STEPPER_H */
